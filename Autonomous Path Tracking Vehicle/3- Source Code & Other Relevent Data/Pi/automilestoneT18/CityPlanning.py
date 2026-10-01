#!/usr/bin/env python3
"""
CityPlanning.py — Team 18

This node publishes the planned city-track path and target speed.

It subscribes to:
  /odom_data
      std_msgs/Float32MultiArray
      data = [x, y, theta, speed]

It publishes:
  /team18/target_speed
      std_msgs/Float64

  /team18/path_x
      std_msgs/Float64MultiArray

  /team18/path_y
      std_msgs/Float64MultiArray

Important:
  Speed values are normalized Arduino commands.
  1.0 means full normalized speed command.
"""

import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray


# ==========================================================
# SPEED COMMANDS
# ==========================================================
# Keep speed constant. No curve slowdown.
SPEED_CRUISE = 1.0
SPEED_CURVE = 1.0
SPEED_STOP = 0.0

START_DELAY_SEC = 0.0


class CityPlanning(Node):

    def __init__(self):
        super().__init__('CityPlanning')

        # ==================================================
        # CURRENT REAL-CAR LOCALIZATION
        # ==================================================
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.current_speed = 0.0
        self.has_odom = False

        self.progress_index = 0
        self.start_time = self.get_clock().now()

        # ==================================================
        # PATH POINTS IN ODOM FRAME
        # ==================================================
        # Important:
        # These points are already in your real-car odom frame.
        # Therefore, do NOT apply:
        #   x_odom = y_world
        #   y_odom = -x_world
        #
        # Car start should be:
        #   position: x = 0, y = 0
        #   heading : theta = 0 rad
        #   direction: along +x

        self.path_x = [
            # Short start straight
            0.00,
            0.5,
            1.00,
            1.5,
            1.95,

            # First curve
            2.65,

            # Long straight 1
            2.65,
            2.65,
            2.65,
            2.65,
            2.65,

            # Second curve
            1.95,

            # Short straight 2
            1.5,
            1.00,
            0.0,

            # Third curve
            -0.88,

            # Long straight 2
            -0.80,
            -0.80,
            -0.75,
            -0.70,
            -0.70,

            # Fourth curve
            0.0,
            

        
        ]

        self.path_y = [
            # Short start straight
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,

            # First curve
            0.7,

            # Long straight 1
            1.0,
            1.5,
            2.5,
            3.2,
            4.0,

            # Second curve
            4.43,

            # Short straight 2
            4.45,
            4.45,
            4.45,

            # Third curve
            3.74,

            # Long straight 2
            3.0,
            2.5,
            1.5,
            1.0,
            0.7,

            # Fourth curve
            0.0,

            
        ]

        if len(self.path_x) != len(self.path_y):
            raise ValueError(
                f'Path length mismatch: '
                f'len(path_x)={len(self.path_x)}, '
                f'len(path_y)={len(self.path_y)}'
            )

        # ==================================================
        # CURVE INTERVALS
        # ==================================================
        # Speed is constant, so these do not reduce speed now.
        # They are kept only for logging/future tuning.
        self.curve_intervals = [
            (2, 3),      # first curve
            (6, 7),      # second curve
            (9, 10),     # third curve
            (13, 14),    # fourth curve
        ]

        # ==================================================
        # PUBLISHERS TO PURE PURSUIT
        # ==================================================
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

        # ==================================================
        # SUBSCRIBER FROM LOCALIZATION
        # ==================================================
        self.odom_sub = self.create_subscription(
            Float32MultiArray,
            '/odom_data',
            self.odom_callback,
            10
        )

        # Publish planner output at 20 Hz.
        self.timer = self.create_timer(
            0.05,
            self.timer_callback
        )

        self.get_logger().info(
            'CityPlanning node started.\n'
            '  Subscribes: /odom_data\n'
            '  Publishes : /team18/target_speed, /team18/path_x, /team18/path_y\n'
            f'  Number of path points = {len(self.path_x)}\n'
            f'  Start delay = {START_DELAY_SEC:.1f} sec\n'
            f'  Cruise command = {SPEED_CRUISE:.2f}\n'
            f'  Curve command  = {SPEED_CURVE:.2f}\n'
            '  Path frame: odom frame, no x/y transform applied\n'
            '  Start pose should be x=0, y=0, theta=0 rad, facing +x'
        )

    # ======================================================
    # RECEIVE LOCALIZATION
    # ======================================================
    def odom_callback(self, msg: Float32MultiArray):
        """
        Receive localization from Localization_Team18.py.

        Expected:
            msg.data = [x, y, theta, speed]
        """

        if len(msg.data) < 4:
            self.get_logger().warn(
                'Received /odom_data with fewer than 4 values. '
                'Expected [x, y, theta, speed].'
            )
            return

        self.current_x = float(msg.data[0])
        self.current_y = float(msg.data[1])
        self.current_theta = float(msg.data[2])
        self.current_speed = float(msg.data[3])
        self.has_odom = True

    # ======================================================
    # PROGRESS ESTIMATION
    # ======================================================
    def find_progress_index(self) -> int:
        """
        Find the closest path point to the car.

        The search starts slightly before the previous progress index
        so it avoids jumping backward too much.
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

    # ======================================================
    # CURVE CHECK
    # ======================================================
    def is_in_curve_interval(self, progress_index: int) -> bool:
        for start_index, end_index in self.curve_intervals:
            if start_index <= progress_index <= end_index:
                return True

        return False

    # ======================================================
    # SPEED PLANNING
    # ======================================================
    def get_target_speed(self) -> float:
        elapsed_time = (
            self.get_clock().now() - self.start_time
        ).nanoseconds / 1e9

        if elapsed_time < START_DELAY_SEC:
            return SPEED_STOP

        if not self.has_odom:
            return SPEED_STOP

        progress_index = self.find_progress_index()

        final_dx = self.current_x - self.path_x[-1]
        final_dy = self.current_y - self.path_y[-1]

        final_distance = math.sqrt(
            final_dx * final_dx +
            final_dy * final_dy
        )

        # Stop only when actually close to the final point.
        if progress_index >= len(self.path_x) - 2 and final_distance < 0.25:
            return SPEED_STOP

        # Keep speed constant everywhere.
        return SPEED_CRUISE

    # ======================================================
    # MAIN LOOP
    # ======================================================
    def timer_callback(self):
        speed_msg = Float64()
        speed_msg.data = float(self.get_target_speed())
        self.speed_pub.publish(speed_msg)

        path_x_msg = Float64MultiArray()
        path_x_msg.data = [float(v) for v in self.path_x]
        self.path_x_pub.publish(path_x_msg)

        path_y_msg = Float64MultiArray()
        path_y_msg.data = [float(v) for v in self.path_y]
        self.path_y_pub.publish(path_y_msg)

        self.get_logger().info(
            f'x={self.current_x:.2f} | '
            f'y={self.current_y:.2f} | '
            f'theta={self.current_theta:.2f} | '
            f'progress={self.progress_index}/{len(self.path_x) - 1} | '
            f'target_speed={speed_msg.data:.2f}',
            throttle_duration_sec=0.5
        )


def main(args=None):
    rclpy.init(args=args)

    node = CityPlanning()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()