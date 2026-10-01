import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, tty, termios

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')
        self.pub = self.create_publisher(Twist, '/model/vehicle_blue/cmd_vel', 10)

def main():
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = TeleopNode()
    msg = Twist()
    print("USE W-A-S-D TO DRIVE | X TO STOP | CTRL+C TO EXIT")

    try:
        while rclpy.ok():
            tty.setraw(sys.stdin.fileno())
            key = sys.stdin.read(1)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

            if key == 'w': msg.linear.x += 0.3
            elif key == 's': msg.linear.x -= 0.3
            elif key == 'a': msg.angular.z += 0.2
            elif key == 'd': msg.angular.z -= 0.2
            elif key == 'x': msg.linear.x = 0.0; msg.angular.z = 0.0
            elif key == '\x03': break

            node.pub.publish(msg)
            print(f"TELEOP: Speed {msg.linear.x:.1f} | Steer {msg.angular.z:.1f}      ", end='\r')
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        rclpy.shutdown()

if __name__ == '__main__':
    main()
