import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class PlannerControllerNode(Node):

    def __init__(self):

        super().__init__(
            'Planner_Controller_Team18'
        )

        # ==================================================
        # SUBSCRIBE TO ODOMETRY
        # ==================================================
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/odom_data',
            self.odom_callback,
            10
        )

        # ==================================================
        # PUBLISH CONTROL
        # ==================================================
        self.cmd_pub = self.create_publisher(
            Float32MultiArray,
            '/cmd_vel_raw',
            10
        )

        # ==================================================
        # GOAL
        # ==================================================
        self.goal_distance = 10.0

        self.Kp_heading = 600

    # ======================================================
    # ODOM CALLBACK
    # ======================================================
    def odom_callback(self, msg):

        x = msg.data[0]
        y = msg.data[1]
        theta = msg.data[2]
        speed = msg.data[3]

        control_msg = Float32MultiArray()

        # ==================================================
        # STOP CONDITION
        # ==================================================
        if x >= self.goal_distance:

            speed_command = 0.0
            steering = 90.0

            self.get_logger().info(
                "GOAL REACHED"
            )

        else:

            # heading controller
            heading_error = 0.0 - theta

            steering = (
                90.0
                - self.Kp_heading
                * heading_error
            )

            steering = max(
                50.0,
                min(130.0, steering)
            )

            speed_command = 0.8

        control_msg.data = [
            speed_command,
            steering
        ]

        self.cmd_pub.publish(control_msg)

        self.get_logger().info(
            f"x={x:.2f} "
            f"theta={theta:.2f} "
            f"steering={steering:.2f}"
        )


def main(args=None):

    rclpy.init(args=args)

    node = PlannerControllerNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
