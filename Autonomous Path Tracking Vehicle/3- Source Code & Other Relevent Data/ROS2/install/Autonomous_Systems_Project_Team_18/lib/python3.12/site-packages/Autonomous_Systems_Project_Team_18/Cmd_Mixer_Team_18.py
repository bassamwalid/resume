import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdMixerNode(Node):
    def __init__(self):
        super().__init__('cmd_mixer_node')

        self.speed_cmd = 0.0
        self.steer_cmd = 0.0

        self.speed_sub = self.create_subscription(
            Twist, '/team18/speed_cmd', self.speed_callback, 10)
        self.steer_sub = self.create_subscription(
            Twist, '/team18/steer_cmd', self.steer_callback, 10)
        self.cmd_pub = self.create_publisher(
            Twist, '/model/vehicle_blue/cmd_vel', 10)

        self.create_timer(0.05, self.publish_cmd)
        self.get_logger().info('Command Mixer started.')

    def speed_callback(self, msg):
        self.speed_cmd = msg.linear.x

    def steer_callback(self, msg):
        self.steer_cmd = msg.angular.z

    def publish_cmd(self):
        msg = Twist()
        msg.linear.x  = float(self.speed_cmd)
        msg.angular.z = 0.8 *float(self.steer_cmd)  # pass directly, no scaling
        self.cmd_pub.publish(msg)

        print(
            f'MIXER: Speed={self.speed_cmd:.2f} | angular.z={self.steer_cmd:.3f}      ',
            end='\r'
        )


def main(args=None):
    rclpy.init(args=args)
    node = CmdMixerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
