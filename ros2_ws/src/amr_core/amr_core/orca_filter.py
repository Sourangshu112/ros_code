"""
orca_filter.py

Step 3 of the pipeline: Non-Holonomic ORCA (NH-ORCA) collision-avoidance
filter for differential-drive AMRs, running fully decentralized on each
robot's own edge hardware (e.g. a Raspberry Pi).

  Step 1 (navigator.py)     - D* Lite produces a waypoint path P
  Step 2 (local_planner.py) - Pure Pursuit turns P into v_pref = (vx, vy)
  Step 3 (this file)        - ORCA filters v_pref against neighboring
                               AMRs' broadcasted (position, velocity)
                               and returns the actual (v, omega) to
                               publish to the motor driver

Design notes for running this ON a Raspberry Pi (or similar SBC), not
just simulated on a desktop:

  * No numpy/scipy. A 2D vector here is a plain (x, y) tuple and every
    operation is a small free function. On a Pi's ARM core, importing
    numpy and marshalling 2-element arrays through it is *slower* than
    plain tuples + math.*, and it adds ~30-60ms of import time and a
    non-trivial memory footprint on a headless Pi. Everything below is
    stdlib-only (math + collections.namedtuple).
  * `ORCAFilter` and `Neighbor` use `__slots__` so per-robot state
    doesn't carry a per-instance __dict__ -- meaningful on a Pi 3/4/Zero
    where this loop is expected to run at 10-20 Hz continuously
    alongside the ROS2/Zenoh comms stack.
  * The hot path (compute_half_planes + solve_linear_program) is
    O(n) per neighbor for the common case and only falls back to the
    O(n^2) 3D LP on genuinely infeasible configurations (dense choke
    points), which is rare and short-lived.
  * No dynamic allocation inside the LP loops beyond what's needed to
    build the half-plane list once per tick; nothing here allocates
    per-frame numpy buffers or spawns threads/processes, so the whole
    filter is safe to call synchronously from a single ROS2 timer
    callback on a Pi without extra scheduling overhead.
  * If neighbor counts ever get large (dozens of AMRs seen by one
    robot's comms range), the natural next optimization is to prune
    `neighbors` to the K nearest / within a radius before calling
    `compute_half_planes` -- the math below doesn't change, only the
    input list does.

Usage from the outer control loop (ROS2 node / Zenoh subscriber / test
harness), matching local_planner.py's contract:

    planner = LocalPlanner(..., on_goal_reached=agent.advance_leg)
    orca = ORCAFilter(epsilon=0.1, radius=0.3, tau=2.0, v_max=0.5)

    # on each timer tick, at 10-20 Hz:
    vx_pref, vy_pref = planner.step()
    neighbors = [Neighbor(x, y, vx, vy, r) for ... in latest_broadcasts]
    v, omega = orca.step(planner.x, planner.y, planner.theta,
                          (vx_pref, vy_pref), neighbors)
    publish_motor_command(v, omega)
"""

import math
from collections import namedtuple

# ---------------------------------------------------------------------------
# Tunables / constants
# ---------------------------------------------------------------------------

EPSILON = 1e-5          # generic numerical tolerance (parallel lines, zero-length vectors)
DEFAULT_TIME_STEP = 0.1  # fallback horizon (s) used only when already inside the collision disc

# A half-plane / ORCA line: `point` is a point on its boundary, `direction`
# is a *unit* vector along the boundary. The outward safe-region normal is
# always direction rotated -90 deg: n = (direction.y, -direction.x), i.e.
# a velocity v is valid for this line iff dot(v - point, n) >= 0.
HalfPlane = namedtuple("HalfPlane", ["point", "direction"])


class Neighbor:
    """
    A single sensed/broadcast neighbor AMR, as received over the
    decentralized comms stack (Zenoh, DDS, etc). Plain __slots__ struct
    -- no behavior, just the (position, velocity, radius) triple ORCA
    needs.
    """
    __slots__ = ("x", "y", "vx", "vy", "radius", "responsibility")

    def __init__(self, x, y, vx, vy, radius, responsibility = 0.5):
        self.x = x
        self.y = y
        self.vx = vx
        self.vy = vy
        self.radius = radius
        self.responsibility = responsibility

# ---------------------------------------------------------------------------
# Minimal 2D vector helpers (plain tuples, no numpy)
# ---------------------------------------------------------------------------

def vec_add(a, b):
    return (a[0] + b[0], a[1] + b[1])


def vec_sub(a, b):
    return (a[0] - b[0], a[1] - b[1])


def vec_scale(a, s):
    return (a[0] * s, a[1] * s)


def vec_dot(a, b):
    return a[0] * b[0] + a[1] * b[1]


def vec_len_sq(a):
    return a[0] * a[0] + a[1] * a[1]


def vec_len(a):
    return math.hypot(a[0], a[1])


def vec_normalize(a):
    length = vec_len(a)
    if length < EPSILON:
        return (0.0, 0.0)
    return (a[0] / length, a[1] / length)


def _det(a, b):
    """2D cross-product z-component: a.x*b.y - a.y*b.x."""
    return a[0] * b[1] - a[1] * b[0]


def _normal_of(direction):
    """Outward safe-region normal for a line with this tangent direction."""
    return (direction[1], -direction[0])


# ---------------------------------------------------------------------------
# ShiftControlPoint
# ---------------------------------------------------------------------------

def shift_control_point(x, y, theta, epsilon=0.1):
    """
    Because a differential-drive AMR cannot move laterally, NH-ORCA
    tracks a virtual point `epsilon` ahead of the wheel axis instead of
    the axle center. This is what turns a nonholonomic robot into
    something the (holonomic) linear-velocity ORCA math can reason
    about safely.

    Returns p_A = (c_x, c_y).
    """
    return (x + epsilon * math.cos(theta), y + epsilon * math.sin(theta))


# ---------------------------------------------------------------------------
# ComputeHalfPlanes
# ---------------------------------------------------------------------------

def compute_half_planes(p_A, v_A, neighbors, tau, r_A, time_step=DEFAULT_TIME_STEP):
    """
    Builds one ORCA half-plane per neighbor, in the control point's
    velocity space, following the standard ORCA velocity-obstacle
    construction (van den Berg et al.):

      - Shrink the neighbor to a point and grow it by combined_radius.
      - Build the VO cone as the truncated cone (apex cut off at
        distance combined_radius/tau from the origin, along
        relative_position/tau) that rel_vel must avoid.
      - Find u, the minimum vector moving rel_vel to the cone boundary.
      - Split u 50/50 (both AMRs are expected to run this same filter
        and each take half the avoidance responsibility).

    Returns a list[HalfPlane].
    """
    lines = []

    for neighbor in neighbors:
        p_B = (neighbor.x, neighbor.y)
        v_B = (neighbor.vx, neighbor.vy)
        r_B = neighbor.radius

        rel_pos = vec_sub(p_B, p_A)
        rel_vel = vec_sub(v_A, v_B)
        dist_sq = vec_len_sq(rel_pos)
        combined_radius = r_A + r_B
        combined_radius_sq = combined_radius * combined_radius

        if dist_sq > combined_radius_sq:
            # Not currently overlapping -- normal case, cone truncated at tau.
            inv_tau = 1.0 / tau
            w = vec_sub(rel_vel, vec_scale(rel_pos, inv_tau))
            w_len_sq = vec_len_sq(w)
            dot1 = vec_dot(w, rel_pos)

            if dot1 < 0.0 and dot1 * dot1 > combined_radius_sq * w_len_sq:
                # rel_vel projects onto the truncation (cap) circle, not a leg.
                w_len = math.sqrt(w_len_sq)
                unit_w = vec_scale(w, 1.0 / w_len) if w_len > EPSILON else (0.0, 0.0)
                n = unit_w
                u = vec_scale(unit_w, combined_radius * inv_tau - w_len)
            else:
                # rel_vel projects onto one of the two side legs of the cone.
                leg = math.sqrt(max(dist_sq - combined_radius_sq, 0.0))
                if _det(rel_pos, w) > 0.0:
                    # left leg
                    leg_dir = (
                        (rel_pos[0] * leg - rel_pos[1] * combined_radius) / dist_sq,
                        (rel_pos[0] * combined_radius + rel_pos[1] * leg) / dist_sq,
                    )
                else:
                    # right leg
                    leg_dir = (
                        (rel_pos[0] * leg + rel_pos[1] * combined_radius) / dist_sq,
                        (-rel_pos[0] * combined_radius + rel_pos[1] * leg) / dist_sq,
                    )
                dot2 = vec_dot(rel_vel, leg_dir)
                u = vec_sub(vec_scale(leg_dir, dot2), rel_vel)
                n = vec_normalize(u)
        else:
            # Already inside the safety disc (should only happen transiently,
            # e.g. right after a neighbor cuts in close). Use the current
            # control tick instead of tau so avoidance is immediate rather
            # than waiting out the full horizon.
            inv_dt = 1.0 / max(time_step, EPSILON)
            w = vec_sub(rel_vel, vec_scale(rel_pos, inv_dt))
            w_len = vec_len(w)
            unit_w = vec_scale(w, 1.0 / w_len) if w_len > EPSILON else (0.0, 0.0)
            n = unit_w
            u = vec_scale(unit_w, combined_radius * inv_dt - w_len)

        point = vec_add(v_A, vec_scale(u, neighbor.responsibility))
        # direction is the tangent whose rotate(-90) recovers n, i.e. rotate(+90) of n.
        direction = (-n[1], n[0])
        lines.append(HalfPlane(point=point, direction=direction))

    return lines


# ---------------------------------------------------------------------------
# 1D line optimization (Solve1DLineOptimization)
# ---------------------------------------------------------------------------

def _line_circle_intersections(point, direction, radius):
    """
    Intersects the line {point + t*direction} with the circle of the
    given `radius` centered at the origin (the V_max speed bound).
    Returns (t_min, t_max) or None if the line misses the circle
    entirely (i.e. this half-plane can't be satisfied within V_max at all).
    """
    px, py = point
    dx, dy = direction
    a = dx * dx + dy * dy  # == 1.0 for a unit direction; kept general for safety
    b = 2.0 * (px * dx + py * dy)
    c = px * px + py * py - radius * radius
    disc = b * b - 4.0 * a * c
    if disc < 0.0:
        return None
    sq = math.sqrt(disc)
    t1 = (-b - sq) / (2.0 * a)
    t2 = (-b + sq) / (2.0 * a)
    if t1 > t2:
        t1, t2 = t2, t1
    return t1, t2


def linear_program1(lines, line_no, radius, opt_velocity, direction_opt):
    """
    Solves the 1D optimization along lines[line_no]'s boundary line,
    subject to: staying inside the V_max circle, and satisfying every
    half-plane in lines[0:line_no].

    If direction_opt is True, `opt_velocity` is treated as a direction
    to push toward the extreme feasible end of the segment (used by the
    3D LP fallback), rather than a point to project toward.

    Returns (success: bool, result: (x, y) | None).
    """
    point = lines[line_no].point
    direction = lines[line_no].direction

    bounds = _line_circle_intersections(point, direction, radius)
    if bounds is None:
        return False, None
    t_min, t_max = bounds

    for j in range(line_no):
        other = lines[j]
        denom = _det(direction, other.direction)
        numerator = _det(vec_sub(other.point, point), other.direction)

        if abs(denom) <= EPSILON:
            # Parallel boundary lines: either always satisfied, or never.
            if numerator > 0.0:
                return False, None
            continue

        t = numerator / denom
        if denom > 0.0:
            t_min = max(t_min, t)
        else:
            t_max = min(t_max, t)

        if t_min > t_max:
            return False, None

    if direction_opt:
        t = t_max if vec_dot(opt_velocity, direction) > 0.0 else t_min
    else:
        t_opt = vec_dot(vec_sub(opt_velocity, point), direction)
        if t_opt < t_min:
            t = t_min
        elif t_opt > t_max:
            t = t_max
        else:
            t = t_opt

    return True, vec_add(point, vec_scale(direction, t))


# ---------------------------------------------------------------------------
# 2D linear program (incremental, Seidel-style)
# ---------------------------------------------------------------------------

def linear_program2(lines, radius, opt_velocity, direction_opt=False):
    """
    Incrementally applies each half-plane in `lines` to the running
    optimum, starting from opt_velocity clamped to the V_max circle.
    Each time the current optimum violates a line, the problem
    collapses to the 1D optimization along that line's boundary
    (linear_program1), which already accounts for every line seen so far.

    Returns (lines_satisfied, result). `lines_satisfied` == len(lines)
    means full success; a smaller value tells the 3D LP fallback where
    the search became infeasible.
    """
    if direction_opt:
        # opt_velocity is a unit direction; push to the circle's edge along it.
        result = vec_scale(opt_velocity, radius)
    else:
        speed_sq = vec_len_sq(opt_velocity)
        if speed_sq > radius * radius:
            result = vec_scale(opt_velocity, radius / math.sqrt(speed_sq))
        else:
            result = opt_velocity

    for i, line in enumerate(lines):
        n = _normal_of(line.direction)
        if vec_dot(n, vec_sub(result, line.point)) < 0.0:
            success, new_result = linear_program1(lines, i, radius, opt_velocity, direction_opt)
            if not success:
                return i, result
            result = new_result

    return len(lines), result


# ---------------------------------------------------------------------------
# 3D linear program fallback (dense choke points / infeasible configurations)
# ---------------------------------------------------------------------------

def linear_program3(lines, begin_line, radius, result):
    """
    Called only when linear_program2 could not satisfy every half-plane
    within V_max (a genuinely infeasible configuration -- e.g. several
    AMRs converging on a corridor too narrow for reciprocal avoidance
    alone). Relaxes the problem to minimize the worst constraint
    violation instead of insisting on strict feasibility, so the robot
    prefers slowing/stopping over an unsafe velocity.
    """
    distance = 0.0

    for i in range(begin_line, len(lines)):
        line_i = lines[i]
        n_i = _normal_of(line_i.direction)
        violation = -vec_dot(n_i, vec_sub(result, line_i.point))

        if violation > distance:
            # Re-derive every earlier line as a constraint projected onto
            # line_i's boundary, then re-optimize along that 1D segment
            # (pushed toward whichever end minimizes violation of line_i).
            proj_lines = []
            for j in range(i):
                line_j = lines[j]
                denom = _det(line_i.direction, line_j.direction)

                if abs(denom) <= EPSILON:
                    if vec_dot(line_i.direction, line_j.direction) > 0.0:
                        continue  # line_j redundant on line_i's boundary
                    p = vec_scale(vec_add(line_i.point, line_j.point), 0.5)
                else:
                    t = _det(line_j.direction, vec_sub(line_i.point, line_j.point)) / denom
                    p = vec_add(line_i.point, vec_scale(line_i.direction, t))

                raw_dir = vec_sub(
                    line_j.direction,
                    vec_scale(line_i.direction, vec_dot(line_j.direction, line_i.direction)),
                )
                d = vec_normalize(raw_dir)
                proj_lines.append(HalfPlane(point=p, direction=d))

            opt_dir = (-line_i.direction[1], line_i.direction[0])
            _, new_result = linear_program2(proj_lines, radius, opt_dir, direction_opt=True)
            result = new_result
            distance = -vec_dot(n_i, vec_sub(result, line_i.point))

    return result


def solve_linear_program(v_pref, lines, v_max):
    """
    SolveLinearProgram: tries the 2D LP first; if that's infeasible at
    line index `count`, falls back to the 3D LP starting from there.
    Returns v_safe = (vx, vy).
    """
    count, v_safe = linear_program2(lines, v_max, v_pref, direction_opt=False)
    if count < len(lines):
        v_safe = linear_program3(lines, count, v_max, v_safe)
    return v_safe


# ---------------------------------------------------------------------------
# MapToDifferentialDrive
# ---------------------------------------------------------------------------

def map_to_differential_drive(v_safe, theta, epsilon, omega_max=None):
    """
    Transforms the global-frame safe velocity into the robot's local
    frame and derives the differential-drive (v, omega) command from
    the shifted control point's kinematics.
    """
    cos_t = math.cos(theta)
    sin_t = math.sin(theta)

    v_local_x = v_safe[0] * cos_t + v_safe[1] * sin_t
    v_local_y = -v_safe[0] * sin_t + v_safe[1] * cos_t

    v = v_local_x
    omega = v_local_y / epsilon

    if omega_max is not None and omega_max > 0.0:
        omega = max(-omega_max, min(omega_max, omega))

    return v, omega


# ---------------------------------------------------------------------------
# Public entry point: ORCAFilter
# ---------------------------------------------------------------------------

class ORCAFilter:
    """
    Step 3 of the pipeline. Stateless across ticks except for its own
    tuning parameters (__slots__, no per-instance __dict__), so it's
    cheap to hold one instance per robot and call `step()` from a fixed-
    rate timer callback right after local_planner.LocalPlanner.step().

    Parameters
    ----------
    epsilon : forward offset (m) of the NH-ORCA control point ahead of
        the wheel axis. Also the value MapToDifferentialDrive divides
        by, so it must stay well above 0 (0.05-0.15 m is typical).
    radius : this robot's own collision radius (m), inflated slightly
        beyond its physical footprint for safety margin.
    tau : ORCA time horizon (s) -- how far ahead a collision is
        anticipated. Larger tau reacts earlier but more conservatively.
    v_max : maximum linear speed of the control point (m/s); should be
        >= local_planner's v_max since map_to_differential_drive can
        redistribute speed between v and omega.
    time_step : the outer control loop's tick period (s); only used as
        the emergency horizon when a neighbor is already inside the
        combined safety radius.
    """

    __slots__ = ("epsilon", "radius", "tau", "v_max", "omega_max", "time_step")

    def __init__(self, epsilon=0.1, radius=0.3, tau=2.0, v_max=2.0, omega_max=1.0, time_step=0.1):
        if epsilon <= 0.0:
            raise ValueError("epsilon must be > 0 (MapToDifferentialDrive divides by it)")
        self.epsilon = epsilon
        self.radius = radius
        self.tau = tau
        self.v_max = v_max
        self.omega_max = omega_max
        self.time_step = time_step

    def step(self, x, y, theta, v_pref, neighbors, v_current=None):
        """
        Runs one full ORCA tick.

        x, y, theta   : this robot's current pose (from on_odometry).
        v_pref        : (vx, vy) from local_planner.LocalPlanner.step().
        neighbors     : iterable of Neighbor, as received over the
                        decentralized comms stack this tick.
        v_current     : optional (vx, vy) estimate of this robot's own
                        actual world-frame velocity, used to build the
                        VO cones. Defaults to v_pref (the common
                        approximation when no better estimate -- e.g.
                        the previous tick's v_safe -- is being tracked
                        by the caller).

        Returns (v, omega): the differential-drive command to publish.
        """
        p_A = shift_control_point(x, y, theta, self.epsilon)
        v_A = v_pref if v_current is None else v_current

        lines = compute_half_planes(p_A, v_A, neighbors, self.tau, self.radius, self.time_step)
        v_safe = solve_linear_program(v_pref, lines, self.v_max)

        return map_to_differential_drive(v_safe, theta, self.epsilon, self.omega_max)


# ---------------------------------------------------------------------------
# Self-contained sanity check (no ROS2 / Zenoh / Tkinter needed)
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    # Robot A wants to drive straight along +x at 0.4 m/s; robot B is
    # crossing its path. ORCA should bend v_safe (and thus omega) to
    # avoid a collision instead of driving straight through B.
    orca = ORCAFilter(epsilon=0.1, radius=0.25, tau=2.0, v_max=0.5)

    robot_x, robot_y, robot_theta = 0.0, 0.0, 0.0
    v_pref = (0.4, 0.0)

    neighbor_b = Neighbor(x=1.5, y=0.3, vx=-0.3, vy=-0.1, radius=0.25)

    v, omega = orca.step(robot_x, robot_y, robot_theta, v_pref, [neighbor_b])
    print(f"v_pref={v_pref}  ->  commanded v={v:.3f} m/s, omega={omega:.3f} rad/s")

    # No neighbors: should pass v_pref through close to unchanged
    # (modulo the epsilon control-point projection).
    v2, omega2 = orca.step(robot_x, robot_y, robot_theta, v_pref, [])
    print(f"no neighbors -> commanded v={v2:.3f} m/s, omega={omega2:.3f} rad/s")