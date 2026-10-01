import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Float64MultiArray


# ─────────────────────────────────────────────────────────────────
#  MS4 Planning Node — Racing Track — Team 18
#
#  This node is intentionally simple.
#
#  It publishes:
#    1. Desired speed on /team18/target_speed
#    2. Planned path x-points on /team18/path_x
#    3. Planned path y-points on /team18/path_y
#
#  The control node uses Pure Pursuit to follow this path.
#
#  Correct Racing Track:
#    Length = 10 m
#    Width  = 0.75 m
#
#  Lane centers:
#    Lane 1 = +0.1875 m
#    Lane 2 = -0.1875 m
#
#  Obstacles:
#    Obstacle 1 at x = 4 m blocks Lane 1
#    Obstacle 2 at x = 8 m blocks Lane 2
#
#  Path logic:
#    Start near center
#    Move to Lane 1
#    Change to Lane 2 before obstacle 1
#    Change back to Lane 1 before obstacle 2
#    Stop near end
# ─────────────────────────────────────────────────────────────────


# Official lane centers
LANE_1 = 0.1875
LANE_2 = -0.1875

# Slightly safer lane targets.
# These keep the car away from the walls while still staying inside each lane.
# If your instructor requires exact lane centers, change these to LANE_1 and LANE_2.
SAFE_LANE_1 = 0.16
SAFE_LANE_2 = -0.16

# Speed values
SPEED_CRUISE = 0.4
SPEED_CHANGE = 0.25
SPEED_STOP = 0.0

# Track positions
X_FINISH = 9.40


class PlanningNode(Node):
    def __init__(self):
        super().__init__('planning_node_team_18')

        self.current_x = 0.0
        self.current_y = 0.0

        # Planned path for Racing Track.
        # The path itself contains the lane changes.
        #
        # Obstacle 1 is at x = 4.0 and blocks Lane 1.
        # Therefore, the path must already be in Lane 2 before x = 4.0.
        #
        # Obstacle 2 is at x = 8.0 and blocks Lane 2.
        # Therefore, the path must already be in Lane 1 before x = 8.0.
        self.path_x = [
            0.0,
            0.20,
            2.00,
            2.50, #decrease speed
            
            3.5, 
            5.0,  #decrease speed
            6.50, 
            
            7.75, 
            8.75, #increase speed
            10.00
        ]

        self.path_y = [
            0.000,
            SAFE_LANE_1,
            SAFE_LANE_1,
            SAFE_LANE_1,
            
            SAFE_LANE_2,
            SAFE_LANE_2,
            SAFE_LANE_2,
            
            SAFE_LANE_1,
            SAFE_LANE_1,
            SAFE_LANE_1,
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
            'Racing planning node started.\n'
            '  Publishes target speed and path points.\n'
            f'  Lane 1 official center = {LANE_1:.4f} m\n'
            f'  Lane 2 official center = {LANE_2:.4f} m\n'
            f'  Safe Lane 1 target    = {SAFE_LANE_1:.4f} m\n'
            f'  Safe Lane 2 target    = {SAFE_LANE_2:.4f} m'
        )

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

    def get_target_speed(self):
        """
        Simple speed planning using if conditions.
        Slow down in the two lane-change zones.
        Stop near the end.
        """

        x = self.current_x

        if x >= X_FINISH:
            return SPEED_STOP

        # First lane change zone: Lane 1 -> Lane 2
        if 2.5 <= x <= 3.25:
            return SPEED_CHANGE

        # Second lane change zone: Lane 2 -> Lane 1
        if 6.5 <= x <= 7.25:
            return SPEED_CHANGE

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
            f'y={self.current_y:.3f} | '
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
