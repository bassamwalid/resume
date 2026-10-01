#!/usr/bin/env python3
"""
PurePursuit_Team18.py — Team 18

This node is the control node for the real car.

It does NOT use image processing.

It subscribes to:
  1. /odom_data              std_msgs/Float32MultiArray
                             data = [x, y, theta, speed]
                             published by Localization_Team18.py

  2. /team18/target_speed    std_msgs/Float64
                             published by Planning_Team18.py

  3. /team18/path_x          std_msgs/Float64MultiArray
                             planned path x-points in metres

  4. /team18/path_y          std_msgs/Float64MultiArray
                             planned path y-points in metres

It publishes:
  1. /cmd_vel_raw            std_msgs/Float32MultiArray
                             data = [speed, steering_angle]
                             subscribed by Localization_Team18.py and sent to Arduino

Pure Pursuit steering convention copied from the lane-node project:
  - 90 deg  = straight
  - >90 deg = right
  - <90 deg = left
  - steering is clipped between 45 and 135 deg

Important coordinate convention:
  This node assumes that positive lateral error means the target is to the RIGHT
  of the car, because the previous lane-node pure pursuit used:
      servo_angle = 90 + delta_deg
  and positive lateral error produced a right turn.

  If the real car turns the opposite direction, do NOT change the equations first.
  Change the ROS parameter:
      lateral_sign = -1.0
"""

import math

import numpy as np

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray


def pure_pursuit_angle_metric(forward_m: float,
                              lateral_m: float,
                              wheelbase_m: float) -> float:
    """
    Pure Pursuit steering equation copied from the previous lane_node.py logic.

    forward_m:
        Target x-coordinate in the vehicle frame.

    lateral_m:
        Target lateral coordinate in the vehicle frame.
        Positive means target is to the RIGHT of the car for this project.

    wheelbase_m:
        Distance between front and rear axle in metres.

    Returns:
        Servo steering angle in degrees.
    """
    if forward_m <= 0.01:
        return 90.0

    lookahead_dist_sq = forward_m**2 + lateral_m**2
    if lookahead_dist_sq <= 1e-6:
        return 90.0

    curvature = 2.0 * lateral_m / lookahead_dist_sq
    delta_rad = math.atan(curvature * wheelbase_m)
    delta_deg = math.degrees(delta_rad)

    servo_angle = 90.0 + delta_deg
    return float(np.clip(servo_angle, 45.0, 135.0))


class PurePursuitTeam18(Node):
    def __init__(self):
        super().__init__('PurePursuit_Team18')

        # ==================================================
        # PARAMETERS
        # ==================================================
        self.declare_parameter('wheelbase_m', 0.255)
        self.declare_parameter('lookahead_distance_m', 0.6) #0.6 for obstacle avoidance
        self.declare_parameter('smoothing_alpha', 0.90)

        # Used only if the planner has not published speed yet.
        self.declare_parameter('default_speed', 1)

        # Set to -1.0 if steering direction is reversed on the real car.
        self.declare_parameter('lateral_sign', 1.0)

        # Path interpolation spacing.
        # Smaller = smoother path following, but slightly more computation.
        self.declare_parameter('path_resolution_m', 0.02)

        # Timer period for publishing Arduino commands.
        self.declare_parameter('control_period_s', 0.05)

        # Stop the car if odometry is not received for too long.
        self.declare_parameter('odom_timeout_s', 10)

        # ==================================================
        # STATE FROM LOCALIZATION
        # ==================================================
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.current_speed = 0.0
        self.has_odom = False
        self.last_odom_time = None

        # ==================================================
        # STATE FROM PLANNING
        # ==================================================
        self.target_speed = float(self.get_parameter('default_speed').value)
        self.path_x = []
        self.path_y = []
        self.dense_path = []
        self.has_path = False

        # Steering memory for smoothing.
        self.smoothed_angle = 90.0

        # ==================================================
        # SUBSCRIBERS
        # ==================================================
        self.odom_sub = self.create_subscription(
            Float32MultiArray,
            '/odom_data',
            self.odom_callback,
            10
        )

        self.speed_sub = self.create_subscription(
            Float64,
            '/team18/target_speed',
            self.speed_callback,
            10
        )

        self.path_x_sub = self.create_subscription(
            Float64MultiArray,
            '/team18/path_x',
            self.path_x_callback,
            10
        )

        self.path_y_sub = self.create_subscription(
            Float64MultiArray,
            '/team18/path_y',
            self.path_y_callback,
            10
        )

        # ==================================================
        # PUBLISHER TO LOCALIZATION / ARDUINO
        # ==================================================
        self.cmd_pub = self.create_publisher(
            Float32MultiArray,
            '/cmd_vel_raw',
            10
        )

        control_period_s = float(self.get_parameter('control_period_s').value)
        control_period_s = max(0.01, control_period_s)

        self.timer = self.create_timer(
            control_period_s,
            self.control_callback
        )

        self.get_logger().info(
            'Pure Pursuit Team 18 node started.\n'
            '  Subscribes: /odom_data, /team18/target_speed, /team18/path_x, /team18/path_y\n'
            '  Publishes : /cmd_vel_raw\n'
            '  Command format: [speed, steering_angle]'
        )

    # ======================================================
    # CALLBACKS
    # ======================================================
    def odom_callback(self, msg: Float32MultiArray):
        """
        Receive odometry from Localization_Team18.py.

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
        self.has_odom = True
        self.last_odom_time = self.get_clock().now()

    def speed_callback(self, msg: Float64):
        self.target_speed = float(msg.data)

    def path_x_callback(self, msg: Float64MultiArray):
        self.path_x = [float(v) for v in msg.data]
        self._try_update_dense_path()

    def path_y_callback(self, msg: Float64MultiArray):
        self.path_y = [float(v) for v in msg.data]
        self._try_update_dense_path()

    # ======================================================
    # PATH PREPARATION
    # ======================================================
    def _try_update_dense_path(self):
        """
        Create a dense path from the sparse planner waypoints.

        The planner publishes only a few points. Pure pursuit works better when
        there are many intermediate points along the path.
        """
        if len(self.path_x) < 2 or len(self.path_y) < 2:
            self.has_path = False
            return

        if len(self.path_x) != len(self.path_y):
            self.has_path = False
            self.get_logger().warn(
                f'Path length mismatch: len(path_x)={len(self.path_x)}, len(path_y)={len(self.path_y)}'
            )
            return

        path_resolution_m = float(self.get_parameter('path_resolution_m').value)
        path_resolution_m = max(0.005, path_resolution_m)

        dense = []

        for i in range(len(self.path_x) - 1):
            x0 = self.path_x[i]
            y0 = self.path_y[i]
            x1 = self.path_x[i + 1]
            y1 = self.path_y[i + 1]

            dx = x1 - x0
            dy = y1 - y0
            segment_length = math.hypot(dx, dy)

            if segment_length < 1e-9:
                continue

            steps = max(2, int(math.ceil(segment_length / path_resolution_m)))

            for k in range(steps):
                t = k / float(steps)
                dense.append((
                    x0 + t * dx,
                    y0 + t * dy
                ))

        dense.append((self.path_x[-1], self.path_y[-1]))

        self.dense_path = dense
        self.has_path = len(self.dense_path) >= 2

    # ======================================================
    # PURE PURSUIT HELPERS
    # ======================================================
    def _global_to_vehicle_frame(self, target_x: float, target_y: float):
        """
        Transform a global target point into the vehicle frame.

        The normal mathematical vehicle frame gives:
            forward = cos(theta)*dx + sin(theta)*dy
            left    = -sin(theta)*dx + cos(theta)*dy

        The old lane-node steering convention uses positive lateral as RIGHT.
        Therefore:
            right = -left

        lateral_sign can flip this if your real hardware direction is reversed.
        """
        dx = target_x - self.current_x
        dy = target_y - self.current_y

        cos_t = math.cos(self.current_theta)
        sin_t = math.sin(self.current_theta)

        forward_m = cos_t * dx + sin_t * dy

        left_m = -sin_t * dx + cos_t * dy
        right_m = -left_m

        lateral_sign = float(self.get_parameter('lateral_sign').value)
        lateral_m = lateral_sign * right_m

        return forward_m, lateral_m

    def _find_lookahead_target(self):
        """
        Find the path point that should be tracked by Pure Pursuit.

        Steps:
          1. Find nearest dense path point to the car.
          2. Starting from that nearest point, move forward along the path.
          3. Select the first point that is at least lookahead_distance_m away
             and still lies in front of the car.
          4. If no such point exists, use the final path point.
        """
        if not self.has_path:
            return None

        lookahead_distance_m = float(self.get_parameter('lookahead_distance_m').value)
        lookahead_distance_m = max(0.05, lookahead_distance_m)

        car_pos = np.array([self.current_x, self.current_y], dtype=np.float32)
        path_np = np.array(self.dense_path, dtype=np.float32)

        distances = np.linalg.norm(path_np - car_pos, axis=1)
        nearest_index = int(np.argmin(distances))

        best_target = None

        for i in range(nearest_index, len(self.dense_path)):
            px, py = self.dense_path[i]
            dist = math.hypot(px - self.current_x, py - self.current_y)

            if dist < lookahead_distance_m:
                continue

            forward_m, lateral_m = self._global_to_vehicle_frame(px, py)

            if forward_m > 0.01:
                best_target = {
                    'x': px,
                    'y': py,
                    'index': i,
                    'distance': dist,
                    'forward_m': forward_m,
                    'lateral_m': lateral_m
                }
                break

        if best_target is not None:
            return best_target

        # Fallback near the end of the path.
        px, py = self.dense_path[-1]
        forward_m, lateral_m = self._global_to_vehicle_frame(px, py)

        return {
            'x': px,
            'y': py,
            'index': len(self.dense_path) - 1,
            'distance': math.hypot(px - self.current_x, py - self.current_y),
            'forward_m': forward_m,
            'lateral_m': lateral_m
        }

    def _odom_is_fresh(self) -> bool:
        if not self.has_odom or self.last_odom_time is None:
            return False

        odom_timeout_s = float(self.get_parameter('odom_timeout_s').value)
        elapsed_s = (
            self.get_clock().now() - self.last_odom_time
        ).nanoseconds * 1e-9

        return elapsed_s <= odom_timeout_s

    def _publish_command(self, speed: float, steering_angle: float):
        msg = Float32MultiArray()
        msg.data = [
            float(speed),
            float(steering_angle)
        ]
        self.cmd_pub.publish(msg)

    # ======================================================
    # MAIN CONTROL LOOP
    # ======================================================
    def control_callback(self):
        if not self._odom_is_fresh():
            self._publish_command(0.0, 90.0)
            self.get_logger().warn(
                'No fresh /odom_data. Publishing STOP.',
                throttle_duration_sec=1.0
            )
            return

        if not self.has_path:
            self._publish_command(0.0, 90.0)
            self.get_logger().warn(
                'No valid path received yet. Publishing STOP.',
                throttle_duration_sec=1.0
            )
            return

        target = self._find_lookahead_target()

        if target is None:
            self._publish_command(0.0, 90.0)
            self.get_logger().warn(
                'Could not find Pure Pursuit target. Publishing STOP.',
                throttle_duration_sec=1.0
            )
            return

        wheelbase_m = float(self.get_parameter('wheelbase_m').value)

        raw_steering_angle = pure_pursuit_angle_metric(
            target['forward_m'],
            target['lateral_m'],
            wheelbase_m
        )

        speed_command = float(self.target_speed)

        # If the planner says stop, keep the wheels centred.
        if speed_command <= 0.001:
            speed_command = 0.0
            raw_steering_angle = 90.0

        alpha = float(self.get_parameter('smoothing_alpha').value)
        alpha = float(np.clip(alpha, 0.0, 1.0))

        self.smoothed_angle = (
            alpha * raw_steering_angle
            + (1.0 - alpha) * self.smoothed_angle
        )

        self.smoothed_angle = float(np.clip(self.smoothed_angle, 45.0, 135.0))

        self._publish_command(speed_command, self.smoothed_angle)

        self.get_logger().info(
            f'x={self.current_x:.2f}, y={self.current_y:.2f}, theta={self.current_theta:.2f} | '
            f'target=({target["x"]:.2f}, {target["y"]:.2f}) | '
            f'forward={target["forward_m"]:.2f}, lateral={target["lateral_m"]:.2f} | '
            f'speed={speed_command:.2f}, steering={self.smoothed_angle:.1f}',
            throttle_duration_sec=0.5
        )

    def destroy_node(self):
        """
        Stop the car before shutting down the node.
        """
        try:
            self._publish_command(0.0, 90.0)
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitTeam18()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
