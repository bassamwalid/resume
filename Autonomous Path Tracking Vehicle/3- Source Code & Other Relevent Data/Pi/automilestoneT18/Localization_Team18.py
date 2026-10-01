import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial
import threading
import time


class LocalizationNode(Node):

    def __init__(self):

        super().__init__('Localization_Team18')

        # ==================================================
        # SERIAL
        # ==================================================
        self.ser = serial.Serial(
            '/dev/ttyACM0',
            115200,
            timeout=1
        )

        # wait for Arduino reset
        time.sleep(2)

        self.get_logger().info(
            "Serial port opened successfully"
        )

        self.running = True

        # ==================================================
        # ODOMETRY PUBLISHER
        # [x, y, theta, speed]
        # ==================================================
        self.odom_pub = self.create_publisher(
            Float32MultiArray,
            '/odom_data',
            10
        )

        # ==================================================
        # COMMAND SUBSCRIBER
        # ==================================================
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/cmd_vel_raw',
            self.listener_callback,
            10
        )

        # ==================================================
        # SERIAL THREAD
        # ==================================================
        self.read_thread = threading.Thread(
            target=self.read_serial,
            daemon=True
        )

        self.read_thread.start()

    # ======================================================
    # RECEIVE CONTROL COMMANDS
    # ======================================================
    def listener_callback(self, msg):

        speed = msg.data[0]
        steering = msg.data[1]

        command = (
            f"S{speed:.2f},"
            f"A{steering:.2f}\n"
        )

        self.ser.write(command.encode())

        self.get_logger().info(
            f"Sent: {command.strip()}"
        )

    # ======================================================
    # READ SERIAL FROM ARDUINO
    # ======================================================
    def read_serial(self):

        while self.running:

            try:

                if self.ser.in_waiting > 0:

                    line = self.ser.readline() \
                        .decode('utf-8', errors='ignore') \
                        .strip()

                    if line:

                        print(f"Arduino: {line}")

                        self.parse_serial(line)

            except Exception as e:

                self.get_logger().error(
                    f"Error reading serial: {e}"
                )

            time.sleep(0.01)

    # ======================================================
    # PARSE SERIAL DATA
    # ======================================================
    def parse_serial(self, line):

        try:

            # Start with None instead of 0.0
            # This allows us to detect incomplete Arduino lines.
            x = None
            y = None
            theta = None
            speed = None

            parts = line.split(',')

            for part in parts:

                # ==============================
                # X POSITION
                # ==============================
                if "x:" in part:

                    x = float(
                        part.split(':')[1]
                    )

                # ==============================
                # Y POSITION
                # ==============================
                elif "y:" in part:

                    y = float(
                        part.split(':')[1]
                    )

                # ==============================
                # YAW -> THETA (radians)
                # ==============================
                elif "Yaw:" in part:

                    theta = (
                        float(part.split(':')[1])
                        * 3.14159 / 180.0
                    )

                # ==============================
                # SPEED
                # ==============================
                elif "Speed:" in part:

                    speed = float(
                        part.split(':')[1]
                    )

            # ==============================================
            # CHECK FOR INCOMPLETE SERIAL LINE
            # ==============================================
            # If any value was not received, skip this line.
            # This prevents publishing fake x=0, y=0, theta=0, speed=0.
            if x is None or y is None or theta is None or speed is None:

                self.get_logger().warn(
                    f"Skipped incomplete odom line: {line}"
                )

                return

            # ==============================================
            # PUBLISH ODOMETRY
            # ==============================================
            msg = Float32MultiArray()

            msg.data = [
                x,
                y,
                theta,
                speed
            ]

            self.odom_pub.publish(msg)

            self.get_logger().info(
                f"x={x:.2f}, "
                f"y={y:.2f}, "
                f"theta={theta:.2f}, "
                f"speed={speed:.2f}"
            )

        except Exception as e:

            self.get_logger().warn(
                f"Parse Error: {e}"
            )


def main(args=None):

    rclpy.init(args=args)

    node = LocalizationNode()

    try:

        rclpy.spin(node)

    finally:

        node.running = False

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':
    main()