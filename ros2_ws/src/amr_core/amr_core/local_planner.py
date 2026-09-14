"""
local_planner.py
Hardware-agnostic proportional (P) controller for waypoint following.
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
    Drives a differential-drive robot along a list of (x, y) waypoints,
    one at a time, using separate proportional gains for linear and
    angular velocity.
    """

    def __init__(
        self,
        k_v=1.0,
        k_omega=2.0,
        v_max=0.5,
        omega_max=1.5,
        d_tolerance=0.15,
        on_goal_reached=None,
    ):
        # Tuning parameters
        self.k_v = k_v
        self.k_omega = k_omega
        self.v_max = v_max
        self.omega_max = omega_max
        self.d_tolerance = d_tolerance

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
        if not self.is_active:
            return 0.0, 0.0

        # 2. Goal reached -- no waypoints left to target
        if self.current_target_index >= len(self.waypoint_array):
            self.is_active = False
            if self.on_goal_reached is not None:
                self.on_goal_reached()
            return 0.0, 0.0

        # 3. Fetch target, compute distance and heading error
        x_t, y_t = self.waypoint_array[self.current_target_index]
        dx = x_t - self.x
        dy = y_t - self.y
        d = math.hypot(dx, dy)
        theta_desired = math.atan2(dy, dx)
        e_theta = normalize_angle(theta_desired - self.theta)

        # 4. Waypoint reached -- advance and stop for this tick.
        # Note: this publishes (0, 0) for one control cycle at every
        # waypoint transition rather than immediately chasing the next
        # target in the same tick. Simple and matches the spec as
        # given; if that causes visible stutter at 10-20 Hz once this
        # is on real hardware, the fix is to loop back to step 3
        # instead of returning here.
        if d < self.d_tolerance:
            self.current_target_index += 1
            return 0.0, 0.0

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

        # 7. Hand the command back for the outer layer to publish
        return v, omega