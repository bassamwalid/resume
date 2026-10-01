import math
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Float64MultiArray


# ─────────────────────────────────────────────────────────────────
#  MS4 Planning Node — City Track — Team 18
#
#  This planner:
#    - publishes target speed
#    - publishes path_x
#    - publishes path_y
#    - uses exact centerline points
#    - uses more points during curves
#    - uses fewer points on straights
#    - includes a start delay so the car waits after Gazebo launch
#
#  Matches shifted City_Track.world:
#    start = (0, 0)
#    yaw   = +90 deg
#
#  Centerline unchanged:
#    left straight   x = 0.00
#    right straight  x = 4.50
#    top straight    y = 2.50
#    bottom straight y = -1.00
#
#  Publishes:
#    /team18/target_speed
#    /team18/path_x
#    /team18/path_y
# ─────────────────────────────────────────────────────────────────


SPEED_CRUISE = 0.20
SPEED_CURVE = 0.12
SPEED_STOP = 0.0

# Wait after launch before moving.
# This gives Gazebo, bridge, controllers, and RViz time to open.
START_DELAY_SEC = 20.0


class PlanningNode(Node):
    def __init__(self):
        super().__init__('planning_node_team_18')

        self.current_x = 0.0
        self.current_y = 0.0
        self.progress_index = 0

        self.start_time = self.get_clock().now()

        world_path_x = [
            0.00,

            # Start straight
            0.00,
            0.00,

            # Top-left curve shifted OUTWARD toward yellow wall
            # DO NOT CHANGE — first curve is good
            -0.06,
            -0.10,
            -0.07,
            0.01,
            0.15,
            0.33,
            0.53,
            0.75,

            # Top straight, return to center
            1.20,
            1.75,
            2.75,
            3.30,

            # Top-right curve shifted OUTWARD toward yellow wall
            # DO NOT CHANGE — second curve is good
            3.75,
            3.97,
            4.17,
            4.35,
            4.49,
            4.57,
            4.60,

            # Right straight after second curve
            # Start shifting INWARD, away from outside yellow wall
            4.44,
            4.40,
            4.40,

            # Bottom-right curve shifted INWARD
            # Opposite of outward shift
            4.42,
            4.39,
            4.32,
            4.20,
            4.05,
            3.88,
            3.70,

            # Bottom straight after third curve
            # Requested correction
            3.25,
            2.45,
            1.65,
            1.00,
            0.60,

            # Final bottom-left curve
            # Less steep turn toward 0,0
            0.50,
            0.40,
            0.30,
            0.20,
            0.11,
            0.04,

            # Finish approach to 0,0
            0.00,
            0.00
        ]

        world_path_y = [
            0.00,

            # Start straight
            0.80,
            1.30,

            # Top-left curve shifted OUTWARD toward yellow wall
            # DO NOT CHANGE — first curve is good
            1.55,
            1.75,
            1.97,
            2.17,
            2.35,
            2.49,
            2.57,
            2.60,

            # Top straight, return to center
            2.54,
            2.50,
            2.50,
            2.54,

            # Top-right curve shifted OUTWARD toward yellow wall
            # DO NOT CHANGE — second curve is good
            2.60,
            2.57,
            2.49,
            2.35,
            2.17,
            1.97,
            1.75,

            # Right straight after second curve
            # Shift inward gradually
            1.25,
            0.75,
            -0.20,

            # Bottom-right curve shifted INWARD
            # Less negative y = away from outside bottom wall
            -0.20,
            -0.38,
            -0.55,
            -0.70,
            -0.82,
            -0.90,
            -0.92,

            # Bottom straight after third curve
            # Requested values:
            # -0.70, -0.70, -0.70, -0.70, -0.70
            # changed to:
            # -0.80, -0.75, -0.75, -0.60, -0.60
            -0.80,
            -0.75,
            -0.75,
            -0.60,
            -0.60,

            # Final bottom-left curve
            # Less steep turn toward 0,0
            -0.57,
            -0.52,
            -0.45,
            -0.36,
            -0.25,
            -0.12,

            # Finish approach to 0,0
            -0.04,
            0.00
        ]

        self.path_x = []
        self.path_y = []

        for x_world, y_world in zip(world_path_x, world_path_y):
            x_odom = y_world
            y_odom = -x_world

            self.path_x.append(x_odom)
            self.path_y.append(y_odom)

        self.curve_intervals = [
            (2, 11),      # first curve
            (14, 23),     # second curve
            (24, 44),     # third curve + bottom straight + final curve
        ]

        self.speed_pub = self.create_publisher(
            Float64,
            '/team18/target_speed',
            10
        )

        self.path_x_pub = self.create_publisher(
            Float64MultiArray,
            '/team18/path_x',
            10
        )

        self.path_y_pub = self.create_publisher(
            Float64MultiArray,
            '/team18/path_y',
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/team18/filtered_odom',
            self.odom_callback,
            10
        )

        # Publish planner output at 20 Hz
        self.timer = self.create_timer(0.05, self.timer_callback)

        self.get_logger().info(
            'City planning node started.\n'
            '  Exact centerline path is used.\n'
            '  More points are used in curves.\n'
            '  Fewer points are used on straights.\n'
            f'  Number of path points = {len(self.path_x)}\n'
            f'  Start delay = {START_DELAY_SEC:.1f} sec\n'
            f'  Cruise speed = {SPEED_CRUISE:.2f} m/s\n'
            f'  Curve speed = {SPEED_CURVE:.2f} m/s'
        )

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

    def find_progress_index(self):
        """
        Find the closest path point to the car.

        Search starts slightly before the previous progress index
        so it does not jump backward too much.
        """

        start_index = max(0, self.progress_index - 2)

        nearest_index = start_index
        nearest_distance = float('inf')

        for i in range(start_index, len(self.path_x)):
            dx = self.path_x[i] - self.current_x
            dy = self.path_y[i] - self.current_y
            distance = math.sqrt(dx * dx + dy * dy)

            if distance < nearest_distance:
                nearest_distance = distance
                nearest_index = i

        self.progress_index = nearest_index
        return nearest_index

    def is_in_curve_interval(self, progress_index):
        """
        Returns True if the car is inside one of the 4 curve intervals.
        """

        for start_index, end_index in self.curve_intervals:
            if start_index <= progress_index <= end_index:
                return True

        return False

    def get_target_speed(self):
        """
        Speed logic:
          - wait at start
          - stop near the final point
          - decrease speed in curve intervals
          - increase speed on straights
        """

        elapsed_time = (
            self.get_clock().now() - self.start_time
        ).nanoseconds / 1e9

        if elapsed_time < START_DELAY_SEC:
            return SPEED_STOP

        progress_index = self.find_progress_index()

        # Stop near the final point.
        # The car may not hit exactly (0,0), so use progress and distance.
        final_dx = self.current_x - self.path_x[-1]
        final_dy = self.current_y - self.path_y[-1]
        final_distance = math.sqrt(final_dx * final_dx + final_dy * final_dy)

        if progress_index >= len(self.path_x) - 3:
            return SPEED_STOP

        if progress_index >= len(self.path_x) - 8 and final_distance < 0.22:
            return SPEED_STOP

        # Slow down only in the curve intervals.
        if self.is_in_curve_interval(progress_index):
            return SPEED_CURVE

        # Normal speed on straight sections.
        return SPEED_CRUISE

    def timer_callback(self):
        # Publish speed
        speed_msg = Float64()
        speed_msg.data = self.get_target_speed()
        self.speed_pub.publish(speed_msg)

        # Publish path x-points
        path_x_msg = Float64MultiArray()
        path_x_msg.data = self.path_x
        self.path_x_pub.publish(path_x_msg)

        # Publish path y-points
        path_y_msg = Float64MultiArray()
        path_y_msg.data = self.path_y
        self.path_y_pub.publish(path_y_msg)

        self.get_logger().info(
            f'x={self.current_x:.2f} | '
            f'y={self.current_y:.2f} | '
            f'progress={self.progress_index}/{len(self.path_x) - 1} | '
            f'target_speed={speed_msg.data:.2f}',
            throttle_duration_sec=0.5
        )


def main(args=None):
    rclpy.init(args=args)

    node = PlanningNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
