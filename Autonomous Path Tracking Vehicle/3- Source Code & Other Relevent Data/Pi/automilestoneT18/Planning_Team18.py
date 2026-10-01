#!/usr/bin/env python3
"""
Planning_Team18.py — Team 18

This node publishes the real-life planned racing path and target speed.

It is adapted from the previous Gazebo planning_racing node, but the odometry
input was changed to match Localization_Team18.py.

It subscribes to:
  1. /odom_data              std_msgs/Float32MultiArray
                             data = [x, y, theta, speed]
                             published by Localization_Team18.py

It publishes:
  1. /team18/target_speed    std_msgs/Float64
                             used by PurePursuit_Team18.py

  2. /team18/path_x          std_msgs/Float64MultiArray
                             path x-points in metres

  3. /team18/path_y          std_msgs/Float64MultiArray
                             path y-points in metres

The Pure Pursuit node then publishes:
  /cmd_vel_raw = [speed, steering_angle]
which Localization_Team18.py sends to Arduino.
"""

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray


# ==========================================================
# TRACK CONSTANTS
# ==========================================================
# Official lane centers from the Gazebo planning file.
LANE_1 = 0.25
LANE_2 = -0.25

# Safer lane targets used by the previous planner.
# These keep the car slightly away from the walls.
SAFE_LANE_1 = 0.25
SAFE_LANE_2 = -0.25

# Speed values.
SPEED_CRUISE = 1
SPEED_CHANGE = 1
SPEED_STOP = 0.0

# Track end.
X_FINISH = 10.0


class PlanningTeam18(Node):
    def __init__(self):
        super().__init__('Planning_Team18')

        # ==================================================
        # CURRENT REAL-CAR LOCALIZATION
        # ==================================================
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.current_speed = 0.0

        # ==================================================
        # PLANNED RACING PATH
        # ==================================================
        # Obstacle 1 at x = 4.0 blocks Lane 1.
        # So the path moves to Lane 2 before x = 4.0.
        #
        # Obstacle 2 at x = 8.0 blocks Lane 2.
        # So the path moves back to Lane 1 before x = 8.0.
        self.path_x = [
            0.00,
            0.20,
            2.00,
            2.50,

            3.50,
            5.00,
            6.50,

            7.75,
            8.75,
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
            SAFE_LANE_1
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
            'Planning Team 18 node started.\n'
            '  Subscribes: /odom_data\n'
            '  Publishes : /team18/target_speed, /team18/path_x, /team18/path_y\n'
            f'  Lane 1 official center = {LANE_1:.4f} m\n'
            f'  Lane 2 official center = {LANE_2:.4f} m\n'
            f'  Safe Lane 1 target    = {SAFE_LANE_1:.4f} m\n'
            f'  Safe Lane 2 target    = {SAFE_LANE_2:.4f} m'
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
                'Received /odom_data with fewer than 4 values. Expected [x, y, theta, speed].'
            )
            return

        self.current_x = float(msg.data[0])
        self.current_y = float(msg.data[1])
        self.current_theta = float(msg.data[2])
        self.current_speed = float(msg.data[3])

    # ======================================================
    # SPEED PLANNING
    # ======================================================
    def get_target_speed(self) -> float:
        """
        Simple speed planner using the car x-position.

        Slow down in the lane-change zones.
        Stop near the end.
        """
        x = self.current_x

        if x >= X_FINISH:
            return SPEED_STOP

        # First lane change: Lane 1 -> Lane 2.
        if 2.50 <= x <= 3.25:
            return SPEED_CHANGE

        # Second lane change: Lane 2 -> Lane 1.
        if 6.50 <= x <= 7.25:
            return SPEED_CHANGE

        return SPEED_CRUISE

    # ======================================================
    # MAIN PLANNER LOOP
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
            f'y={self.current_y:.3f} | '
            f'theta={self.current_theta:.2f} | '
            f'target_speed={speed_msg.data:.2f}',
            throttle_duration_sec=0.5
        )


def main(args=None):
    rclpy.init(args=args)
    node = PlanningTeam18()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
