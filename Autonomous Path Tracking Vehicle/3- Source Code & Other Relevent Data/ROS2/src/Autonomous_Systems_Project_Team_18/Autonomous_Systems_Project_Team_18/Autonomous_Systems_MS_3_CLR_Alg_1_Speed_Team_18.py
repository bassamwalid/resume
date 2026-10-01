import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

KP        =  0.8
MAX_SPEED =  5.0
MIN_SPEED = -1.0

class SpeedControllerNode(Node):
    def __init__(self):
        super().__init__('speed_controller_node')

        self.actual_speed  = 0.0
        self.desired_speed = 0.0

        self.cmd_pub    = self.create_publisher(Twist,   '/team18/speed_cmd',    10)
        self.odom_sub   = self.create_subscription(
            Odometry, '/model/vehicle_blue/odometry', self.odom_callback, 10)
        self.target_sub = self.create_subscription(
            Float64, '/team18/target_speed', self.target_cb, 10)

        self.create_timer(0.1, self.control_loop)
        self.get_logger().info('Speed controller restored - original version')

    def odom_callback(self, msg):
        self.actual_speed = msg.twist.twist.linear.x

    def target_cb(self, msg):
        self.desired_speed = msg.data

    def control_loop(self):
        error = self.desired_speed - self.actual_speed
        raw_cmd = self.desired_speed + (KP * error)
        speed_cmd = max(MIN_SPEED, min(MAX_SPEED, raw_cmd))

        msg = Twist()
        msg.linear.x = float(speed_cmd)
        self.cmd_pub.publish(msg)

        self.get_logger().info(
            f'desired={self.desired_speed:.2f} | actual={self.actual_speed:.2f} | cmd={speed_cmd:.2f}',
            throttle_duration_sec=0.5)

def main():
    rclpy.init()
    rclpy.spin(SpeedControllerNode())
    rclpy.shutdown()
