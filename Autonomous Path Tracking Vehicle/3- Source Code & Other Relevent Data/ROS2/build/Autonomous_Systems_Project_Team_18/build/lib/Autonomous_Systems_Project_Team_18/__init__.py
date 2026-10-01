import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class OLRNode(Node):
    def __init__(self):
        super().__init__('Autonomous_Systems_MS_2_OLR_Team_18')
        
        # 1. Declare Parameters (The "rosparam" part of your PDF)
        self.declare_parameter('speed', 1.0)
        self.declare_parameter('steering', 0.5)

        # 2. Publisher to the vehicle (Through the Gazebo Bridge)
        self.publisher_ = self.create_publisher(Twist, '/model/vehicle_blue/cmd_vel', 10)
        
        # 3. Subscriber to get vehicle state
        self.subscription = self.create_subscription(Odometry, '/model/vehicle_blue/odometry', self.odom_callback, 10)

        # Timer to publish commands at 10Hz
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('OLR Node for Team 18 has started.')

    def timer_callback(self):
        # Read the parameters
        target_speed = self.get_parameter('speed').get_parameter_value().double_value
        target_steer = self.get_parameter('steering').get_parameter_value().double_value

        msg = Twist()
        msg.linear.x = target_speed
        msg.angular.z = target_steer
        self.publisher_.publish(msg)

    def odom_callback(self, msg):
        # 4. Print the vehicle state to terminal (The "print to screen" part)
        pos = msg.pose.pose.position
        vel = msg.twist.twist.linear.x
        self.get_logger().info(f'Position: x={pos.x:.2f}, y={pos.y:.2f} | Velocity: {vel:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = OLRNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
