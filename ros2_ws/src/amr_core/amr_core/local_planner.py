"""
local_planner.py
Hardware-agnostic Pure Pursuit controller for waypoint following.
This module expects an outer layer (a ROS2 node, a Zenoh subscriber, or a test harness) to:

  - call on_odometry(...) whenever new pose data arrives
  - call on_path(waypoints) whenever navigator.py produces a new path
  - call step() at a fixed rate (10-20 Hz) from a timer
  - take the (v, omega) step() returns and publish it to the actual
    motor/diff-drive layer
  - react to on_goal_reached firing, which is how cbba_agent.py learns
    the robot is free for the next bid
"""

import math


def normalize_angle(angle):
    """
    Wraps an angle into (-pi, pi] so the robot always turns the short
    way, e.g. -1 degree instead of +359 degrees.
    """
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


class LocalPlanner:
    """
    Drives a differential-drive robot along a list of (x, y) waypoints
    using Pure Pursuit: rather than steering at whichever waypoint is
    next in the array, it advances the target index past any waypoint
    already inside the lookahead radius, so it's always aiming at a
    point far enough ahead to produce smooth, curve-cutting motion
    instead of a hard stop-and-turn at every intermediate node. Only
    the final waypoint uses d_tolerance for arrival; every waypoint
    before it is only ever a "carrot" the lookahead check skips past.
    """

    def __init__(self, k_v=1.0, k_omega=2.0, v_max=0.5, omega_max=1.5, d_tolerance=0.15, lookahead_distance=0.6, on_goal_reached=None,):
        # Tuning parameters
        self.k_v = k_v
        self.k_omega = k_omega
        self.v_max = v_max
        self.omega_max = omega_max
        self.d_tolerance = d_tolerance
        self.lookahead_distance = lookahead_distance

        self.prev_v = 0.0
        self.prev_omega = 0.0
        self.max_accel_v = 0.5      # Max linear acceleration (m/s^2)
        self.max_accel_omega = 1.0  # Max angular acceleration (rad/s^2)
        self.dt = 0.1

        # Robot state, kept current by on_odometry
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Path state, set by on_path, advanced by step
        self.waypoint_array = []
        self.current_target_index = 0
        self.is_active = False

        # Fired with no arguments when the last waypoint is reached.
        # This is the hook cbba_agent.py listens on to know the robot
        # is free for the next bid. Kept as a plain callback instead of
        # a direct import of cbba_agent so this module stays decoupled
        # from both the CBBA layer and any particular messaging stack.
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
        Only Z-axis rotation matters for a ground-based diff-drive robot,
        so roll/pitch are ignored.
        """
        qx, qy, qz, qw = quaternion
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        return math.atan2(siny_cosp, cosy_cosp)

    # Main control loop
    def step(self):
        """
        Runs one iteration of the control loop. Call at a fixed rate
        (10-20 Hz) from a timer. Returns (v, omega) to publish to the
        motor controller layer.
        """
        # 1. Failsafe -- no active path means no motion, no exceptions
        if not self.is_active or not self.waypoint_array:
            self.is_active = False
            return 0.0, 0.0

        # 2. Pure Pursuit lookahead -- advance the target index past
        # any waypoint already within lookahead_distance, stopping at
        # whichever one is far enough to aim at, or at the last
        # waypoint if the whole remaining tail is inside the radius.
        while self.current_target_index < len(self.waypoint_array) - 1:
            x_t, y_t = self.waypoint_array[self.current_target_index]
            d_to_current = math.hypot(x_t - self.x, y_t - self.y)
            if d_to_current < self.lookahead_distance:
                self.current_target_index += 1
            else:
                break

        x_t, y_t = self.waypoint_array[self.current_target_index]
        dx = x_t - self.x
        dy = y_t - self.y
        d = math.hypot(dx, dy)

        # 3. Goal evaluation -- only the final waypoint can end the path,
        # and only once we're within the (tight) arrival tolerance.
        is_final_waypoint = self.current_target_index == len(self.waypoint_array) - 1
        if is_final_waypoint and d < self.d_tolerance:
            self.is_active = False
            if self.on_goal_reached is not None:
                self.on_goal_reached()
            return 0.0, 0.0

        # 4. Heading error toward the current lookahead target
        theta_desired = math.atan2(dy, dx)
        e_theta = normalize_angle(theta_desired - self.theta)
        # print(f"theta_desired: {theta_desired}, e_theta: {e_theta}", flush=True)

        # 5. Proportional control
        omega = self.k_omega * e_theta
        v = self.k_v * d * math.cos(e_theta)
        if v < 0:
            v = 0.0  # spin in place rather than reverse into the target

        # 6. Kinematic clamps
        if v > self.v_max:
            v = self.v_max
        if omega > self.omega_max:
            omega = self.omega_max
        if omega < -self.omega_max:
            omega = -self.omega_max

        # Calculate maximum allowed change for this tick
        max_dv = self.max_accel_v * self.dt
        max_domega = self.max_accel_omega * self.dt

        # Clamp linear acceleration
        if v > self.prev_v + max_dv:
            v = self.prev_v + max_dv
        elif v < self.prev_v - max_dv:
            v = self.prev_v - max_dv

        # Clamp angular acceleration
        if omega > self.prev_omega + max_domega:
            omega = self.prev_omega + max_domega
        elif omega < self.prev_omega - max_domega:
            omega = self.prev_omega - max_domega

        # Store current commands for the next tick
        self.prev_v = v
        self.prev_omega = omega

        # 7. Hand the command back for the outer layer to publish
        return v, omega