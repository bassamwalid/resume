import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class OLRNode(Node):
    def __init__(self):
        super().__init__('olr_node')
        self.pub = self.create_publisher(Twist, '/model/vehicle_blue/cmd_vel', 10)
        self.sub = self.create_subscription(Odometry, '/model/vehicle_blue/odometry', self.odom_cb, 10)
        self.timer = self.create_timer(0.1, self.timer_cb)
        self.get_logger().info('OLR Node Started for Team 18')

    def timer_cb(self):
        msg = Twist()
        msg.linear.x = 1.5   # SPEED
        msg.angular.z = 0.4  # STEERING
        self.pub.publish(msg)

    def odom_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.get_logger().info(f'FEEDBACK: X={x:.2f}, Y={y:.2f}')

def main():
    rclpy.init()
    rclpy.spin(OLRNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
