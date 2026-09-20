"""
local_planner.py

Step 2 of the pipeline: Pure Pursuit waypoint following that outputs a
preferred velocity VECTOR instead of a differential-drive (v, omega)
command.

  Step 1 (navigator.py) - A* / D* Lite produces a waypoint path P
  Step 2 (this file)    - Pure Pursuit turns P into v_pref = (vx, vy)
  Step 3 (ORCA, external) - v_pref is fed into an ORCA collision
                            avoidance filter, which returns the actual
                            collision-free command to publish

This module expects an outer layer (a ROS2 node, a Zenoh subscriber,
or a test harness) to:

  - call on_odometry(...) whenever new pose data arrives
  - call on_path(waypoints) whenever navigator.py produces a new path
  - call step() at a fixed rate (10-20 Hz) from a timer
  - take the (vx, vy) step() returns and hand it to the ORCA filter,
    then publish ORCA's output to the actual motor/diff-drive layer
  - react to on_goal_reached firing, which is how cbba_agent.py learns
    the robot is free for the next bid
"""

import math


def normalize_angle(angle):
    """
    Wraps an angle into (-pi, pi] so heading-error comparisons always
    take the short way around.
    """
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


class LocalPlanner:
    """
    Drives a robot along a list of (x, y) waypoints using Pure Pursuit
    with a lookahead distance. Unlike a differential-drive controller,
    it does not issue a motor command directly: step() hands back a
    preferred velocity vector in the world frame, and it's Step 3's
    ORCA filter that turns this into whatever the robot actually runs.
    """

    def __init__(
        self,
        k_v=1.0,
        lookahead=0.5,
        v_max=0.5,
        d_tolerance=0.15,
        on_goal_reached=None,
    ):
        # Tuning parameters
        self.k_v = k_v
        self.lookahead = lookahead
        self.v_max = v_max
        self.d_tolerance = d_tolerance

        # Robot state, kept current by on_odometry
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Path state, set by on_path, advanced by _get_lookahead_target
        self.waypoint_array = []
        self.current_target_index = 0
        self.is_active = False

        # Fired with no arguments when the last waypoint is reached.
        # Same hook cbba_agent.py listens on as before; the vector
        # output changes nothing about this contract.
        self.on_goal_reached = on_goal_reached

    # Event handlers (called from outside, asynchronously)
    def on_odometry(self, x, y, quaternion=None, theta=None):
        """
        Updates robot state from the latest odometry reading.

        Pass `theta` directly in radians, OR `quaternion` as an
        (x, y, z, w) tuple to have yaw extracted here. If neither is
        given, only x/y are updated and theta is left as-is.
        """
        self.x = x
        self.y = y
        if quaternion is not None:
            self.theta = self._quaternion_to_yaw(quaternion)
        elif theta is not None:
            self.theta = theta

    def on_path(self, waypoints):
        """
        Called when navigator.py hands over a new path. Replaces any
        path currently in progress and restarts targeting from the
        first waypoint.
        """
        self.waypoint_array = list(waypoints)
        self.current_target_index = 0
        self.is_active = True

    # Helper math
    @staticmethod
    def _quaternion_to_yaw(quaternion):
        """
        Extracts yaw (rotation about Z) from an (x, y, z, w) quaternion.
        Only Z-axis rotation matters for a ground-based robot, so
        roll/pitch are ignored.
        """
        qx, qy, qz, qw = quaternion
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        return math.atan2(siny_cosp, cosy_cosp)

    # Pure Pursuit lookahead target selection
    def _get_lookahead_target(self):
        """
        Walks current_target_index forward while the waypoint at that
        index is already closer than the lookahead distance, so the
        robot cuts the corner toward a point further down the path
        instead of homing in on every intermediate waypoint in turn.

        Never advances past the last waypoint, since there's nothing
        beyond it to cut toward, and arrival is judged against that
        final point regardless of how close intermediate points are.
        """
        i = self.current_target_index
        P = self.waypoint_array

        while i < len(P) - 1:
            x_t, y_t = P[i]
            d = math.hypot(x_t - self.x, y_t - self.y)
            if d < self.lookahead:
                i += 1
            else:
                break

        self.current_target_index = i
        return P[i], i

    # Main control loop
    def step(self):
        """
        Runs one iteration of the control loop. Call at a fixed rate
        (10-20 Hz) from a timer. Returns v_pref = (vx, vy), a preferred
        velocity vector in the world frame, NOT a motor command. Step
        3's ORCA filter takes this as input and returns whatever should
        actually be published to the motor layer.
        """
        # 1. Failsafe -- no active path, or an empty one, means no
        # motion and no index-out-of-range risk in step 2 below.
        if not self.is_active or not self.waypoint_array:
            return 0.0, 0.0

        # 2. Acquire target via lookahead
        (x_t, y_t), i = self._get_lookahead_target()
        dx = x_t - self.x
        dy = y_t - self.y
        d = math.hypot(dx, dy)

        # 3. Goal arrival -- only the final waypoint at tolerance counts
        # as arrival. Intermediate waypoints are cut through by the
        # lookahead logic above and are never individually "reached",
        # which also means the old per-waypoint (0, 0) stutter this
        # controller used to publish on every transition is gone: the
        # vector output stays continuous until the actual goal.
        if i == len(self.waypoint_array) - 1 and d < self.d_tolerance:
            self.is_active = False
            if self.on_goal_reached is not None:
                self.on_goal_reached()
            return 0.0, 0.0

        '''# 4. Desired speed magnitude -- same proportional-with-heading-
        # error shaping as before, so a robot facing away from the
        # lookahead point slows or stops instead of driving sideways
        # into it.
        theta_desired = math.atan2(dy, dx)
        e_theta = normalize_angle(theta_desired - self.theta)
        v_mag = self.k_v * d * math.cos(e_theta)
        if v_mag < 0:
            v_mag = 0.0  # don't reverse into the target
        if v_mag > self.v_max:
            v_mag = self.v_max
        '''

        # 4. Desired speed magnitude -- proportional to distance, capped at
        # v_max. The old cos(heading error) shaping is deliberately gone:
        # with vector output it made v_pref == (0, 0) whenever the robot
        # faced >= 90 deg away from the target, so ORCA had nothing to
        # convert into a turn and the robot never rotated. Heading is now
        # handled downstream by map_to_differential_drive (omega comes
        # from the sideways component of v_pref) plus the motor clamp.
        v_mag = min(self.k_v * d, self.v_max)

        
        # 5. Resolve the scalar speed into a world-frame vector pointed
        # at the lookahead target -- this is the format ORCA expects.
        if d > 0:
            vx = v_mag * (dx / d)
            vy = v_mag * (dy / d)
        else:
            vx, vy = 0.0, 0.0

        return vx, vy