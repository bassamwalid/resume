#!/usr/bin/env python3
"""
PathPlotter_Team18.py — Team 18

Live plotting node for comparing:
  1. The input/planned path points published by CityPlanning
  2. The actual path followed by the car from localization odometry
  3. x position versus time
  4. y position versus time
  5. yaw/theta versus time
  6. speed versus time

Subscribes:
  /team18/path_x     std_msgs/Float64MultiArray
  /team18/path_y     std_msgs/Float64MultiArray
  /odom_data         std_msgs/Float32MultiArray
                     data = [x, y, theta, speed]

Displays:
  - Planned/input path points
  - Actual car trajectory
  - Current car position
  - Current heading direction
  - x vs time
  - y vs time
  - yaw/theta vs time
  - speed vs time

Notes:
  This node is for visualization/debugging only.
  It does not publish any control commands.
"""

import math
import os
from typing import List

import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float64MultiArray


class PathPlotterTeam18(Node):

    def __init__(self):
        super().__init__('PathPlotter_Team18')

        # ==================================================
        # PARAMETERS
        # ==================================================
        self.declare_parameter('update_period_s', 0.20)
        self.declare_parameter('max_actual_points', 10000)
        self.declare_parameter('min_record_distance_m', 0.005)
        self.declare_parameter('heading_arrow_length_m', 0.30)
        self.declare_parameter('axis_padding_m', 0.50)
        self.declare_parameter('save_plot_on_exit', True)
        self.declare_parameter(
            'save_path',
            os.path.expanduser('~/team18_path_plot.png')
        )

        # ==================================================
        # PATH DATA FROM PLANNER
        # ==================================================
        self.path_x: List[float] = []
        self.path_y: List[float] = []

        # ==================================================
        # ACTUAL CAR PATH DATA FROM ODOMETRY
        # ==================================================
        self.actual_x: List[float] = []
        self.actual_y: List[float] = []

        # ==================================================
        # TIME HISTORY DATA
        # ==================================================
        self.time_data: List[float] = []
        self.x_time_data: List[float] = []
        self.y_time_data: List[float] = []
        self.yaw_time_data: List[float] = []
        self.speed_time_data: List[float] = []

        self.start_time = self.get_clock().now()

        # ==================================================
        # CURRENT ODOMETRY VALUES
        # ==================================================
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.current_speed = 0.0
        self.has_odom = False

        # ==================================================
        # SUBSCRIBERS
        # ==================================================
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

        self.odom_sub = self.create_subscription(
            Float32MultiArray,
            '/odom_data',
            self.odom_callback,
            10
        )

        # ==================================================
        # MATPLOTLIB SETUP
        # ==================================================
        plt.ion()

        self.fig, self.axes = plt.subplots(3, 2, figsize=(13, 10))

        self.ax = self.axes[0, 0]
        self.ax_x_t = self.axes[0, 1]
        self.ax_y_t = self.axes[1, 0]
        self.ax_yaw_t = self.axes[1, 1]
        self.ax_v_t = self.axes[2, 0]

        # Unused subplot
        self.axes[2, 1].axis('off')

        self.fig.canvas.manager.set_window_title(
            'Team 18 Path Tracking and Odometry Graphs'
        )

        # ==================================================
        # MAIN PATH PLOT
        # ==================================================
        self.planned_line, = self.ax.plot(
            [],
            [],
            marker='o',
            linestyle='--',
            label='Input/planned path points'
        )

        self.actual_line, = self.ax.plot(
            [],
            [],
            linestyle='-',
            linewidth=2,
            label='Actual car path'
        )

        self.current_point, = self.ax.plot(
            [],
            [],
            marker='x',
            markersize=10,
            linestyle='None',
            label='Current car position'
        )

        self.heading_line, = self.ax.plot(
            [],
            [],
            linestyle='-',
            linewidth=2,
            label='Current heading'
        )

        self.ax.set_title('Path Tracking')
        self.ax.set_xlabel('x position [m]')
        self.ax.set_ylabel('y position [m]')
        self.ax.grid(True)
        self.ax.axis('equal')
        self.ax.legend(loc='best')

        # ==================================================
        # TIME GRAPHS
        # ==================================================
        self.x_time_line, = self.ax_x_t.plot(
            [],
            [],
            linewidth=2,
            label='x'
        )
        self.ax_x_t.set_title('x vs time')
        self.ax_x_t.set_xlabel('time [s]')
        self.ax_x_t.set_ylabel('x [m]')
        self.ax_x_t.grid(True)
        self.ax_x_t.legend(loc='best')

        self.y_time_line, = self.ax_y_t.plot(
            [],
            [],
            linewidth=2,
            label='y'
        )
        self.ax_y_t.set_title('y vs time')
        self.ax_y_t.set_xlabel('time [s]')
        self.ax_y_t.set_ylabel('y [m]')
        self.ax_y_t.grid(True)
        self.ax_y_t.legend(loc='best')

        self.yaw_time_line, = self.ax_yaw_t.plot(
            [],
            [],
            linewidth=2,
            label='yaw/theta'
        )
        self.ax_yaw_t.set_title('yaw/theta vs time')
        self.ax_yaw_t.set_xlabel('time [s]')
        self.ax_yaw_t.set_ylabel('yaw/theta [deg]')
        self.ax_yaw_t.grid(True)
        self.ax_yaw_t.legend(loc='best')

        self.speed_time_line, = self.ax_v_t.plot(
            [],
            [],
            linewidth=2,
            label='speed'
        )
        self.ax_v_t.set_title('speed vs time')
        self.ax_v_t.set_xlabel('time [s]')
        self.ax_v_t.set_ylabel('speed [m/s]')
        self.ax_v_t.grid(True)
        self.ax_v_t.legend(loc='best')

        self.fig.tight_layout()
        plt.show(block=False)

        # ==================================================
        # TIMER
        # ==================================================
        update_period_s = float(
            self.get_parameter('update_period_s').value
        )
        update_period_s = max(0.05, update_period_s)

        self.timer = self.create_timer(
            update_period_s,
            self.update_plot
        )

        self.get_logger().info(
            'PathPlotter_Team18 node started.\n'
            '  Subscribes: /team18/path_x, /team18/path_y, /odom_data\n'
            '  Displays : planned path, actual path, x(t), y(t), yaw(t), speed(t)\n'
            '  This node does not control the car.'
        )

    # ======================================================
    # CALLBACKS
    # ======================================================
    def path_x_callback(self, msg: Float64MultiArray):
        self.path_x = [float(v) for v in msg.data]

    def path_y_callback(self, msg: Float64MultiArray):
        self.path_y = [float(v) for v in msg.data]

    def odom_callback(self, msg: Float32MultiArray):
        """
        Expected:
            msg.data = [x, y, theta, speed]
        """

        if len(msg.data) < 4:
            self.get_logger().warn(
                'Received /odom_data with fewer than 4 values. '
                'Expected [x, y, theta, speed].'
            )
            return

        x = float(msg.data[0])
        y = float(msg.data[1])
        theta = float(msg.data[2])
        speed = float(msg.data[3])

        self.current_x = x
        self.current_y = y
        self.current_theta = theta
        self.current_speed = speed
        self.has_odom = True

        now = self.get_clock().now()
        t = (now - self.start_time).nanoseconds * 1e-9

        self.record_actual_point(x, y)
        self.record_time_point(t, x, y, theta, speed)

    # ======================================================
    # DATA RECORDING
    # ======================================================
    def record_actual_point(self, x: float, y: float):
        """
        Store the actual car position.

        A tiny distance filter is used so the path graph does not fill with
        many repeated points when the car is stopped or vibrating in place.
        """

        min_record_distance_m = float(
            self.get_parameter('min_record_distance_m').value
        )
        min_record_distance_m = max(0.0, min_record_distance_m)

        if len(self.actual_x) > 0:
            dx = x - self.actual_x[-1]
            dy = y - self.actual_y[-1]
            distance = math.hypot(dx, dy)

            if distance < min_record_distance_m:
                return

        self.actual_x.append(x)
        self.actual_y.append(y)

        max_actual_points = int(
            self.get_parameter('max_actual_points').value
        )
        max_actual_points = max(100, max_actual_points)

        if len(self.actual_x) > max_actual_points:
            extra = len(self.actual_x) - max_actual_points
            self.actual_x = self.actual_x[extra:]
            self.actual_y = self.actual_y[extra:]

    def record_time_point(
        self,
        t: float,
        x: float,
        y: float,
        theta: float,
        speed: float
    ):
        """
        Store odometry values versus time.

        theta is converted from radians to degrees for the yaw graph.
        """

        yaw_deg = math.degrees(theta)

        self.time_data.append(t)
        self.x_time_data.append(x)
        self.y_time_data.append(y)
        self.yaw_time_data.append(yaw_deg)
        self.speed_time_data.append(speed)

        max_actual_points = int(
            self.get_parameter('max_actual_points').value
        )
        max_actual_points = max(100, max_actual_points)

        if len(self.time_data) > max_actual_points:
            extra = len(self.time_data) - max_actual_points

            self.time_data = self.time_data[extra:]
            self.x_time_data = self.x_time_data[extra:]
            self.y_time_data = self.y_time_data[extra:]
            self.yaw_time_data = self.yaw_time_data[extra:]
            self.speed_time_data = self.speed_time_data[extra:]

    # ======================================================
    # PLOT UPDATE
    # ======================================================
    def update_plot(self):
        """
        Redraw all graphs.

        This is called by a ROS timer, so it updates while rclpy is spinning.
        """

        # ------------------------------
        # Planned path
        # ------------------------------
        if (
            len(self.path_x) == len(self.path_y)
            and len(self.path_x) > 0
        ):
            self.planned_line.set_data(
                self.path_x,
                self.path_y
            )

        elif len(self.path_x) != len(self.path_y):
            self.get_logger().warn(
                f'Cannot plot planned path because lengths do not match: '
                f'len(path_x)={len(self.path_x)}, '
                f'len(path_y)={len(self.path_y)}',
                throttle_duration_sec=2.0
            )

        # ------------------------------
        # Actual path
        # ------------------------------
        self.actual_line.set_data(
            self.actual_x,
            self.actual_y
        )

        # ------------------------------
        # Current position and heading
        # ------------------------------
        if self.has_odom:
            self.current_point.set_data(
                [self.current_x],
                [self.current_y]
            )

            heading_arrow_length_m = float(
                self.get_parameter('heading_arrow_length_m').value
            )
            heading_arrow_length_m = max(0.05, heading_arrow_length_m)

            hx = self.current_x + heading_arrow_length_m * math.cos(
                self.current_theta
            )
            hy = self.current_y + heading_arrow_length_m * math.sin(
                self.current_theta
            )

            self.heading_line.set_data(
                [self.current_x, hx],
                [self.current_y, hy]
            )

        # ------------------------------
        # Time plots
        # ------------------------------
        self.x_time_line.set_data(
            self.time_data,
            self.x_time_data
        )

        self.y_time_line.set_data(
            self.time_data,
            self.y_time_data
        )

        self.yaw_time_line.set_data(
            self.time_data,
            self.yaw_time_data
        )

        self.speed_time_line.set_data(
            self.time_data,
            self.speed_time_data
        )

        # ------------------------------
        # Auto-scale axes
        # ------------------------------
        self.update_axes_limits()
        self.update_time_axes_limits()

        # ------------------------------
        # Draw
        # ------------------------------
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

        plt.pause(0.001)

        self.get_logger().info(
            f'Plot updated | planned_points={len(self.path_x)} | '
            f'actual_points={len(self.actual_x)} | '
            f'time_points={len(self.time_data)}',
            throttle_duration_sec=2.0
        )

    def update_axes_limits(self):
        all_x = []
        all_y = []

        if (
            len(self.path_x) == len(self.path_y)
            and len(self.path_x) > 0
        ):
            all_x.extend(self.path_x)
            all_y.extend(self.path_y)

        if len(self.actual_x) > 0:
            all_x.extend(self.actual_x)
            all_y.extend(self.actual_y)

        if not all_x or not all_y:
            return

        padding = float(
            self.get_parameter('axis_padding_m').value
        )
        padding = max(0.05, padding)

        min_x = min(all_x) - padding
        max_x = max(all_x) + padding
        min_y = min(all_y) - padding
        max_y = max(all_y) + padding

        if abs(max_x - min_x) < 0.5:
            mid_x = 0.5 * (min_x + max_x)
            min_x = mid_x - 0.25
            max_x = mid_x + 0.25

        if abs(max_y - min_y) < 0.5:
            mid_y = 0.5 * (min_y + max_y)
            min_y = mid_y - 0.25
            max_y = mid_y + 0.25

        self.ax.set_xlim(min_x, max_x)
        self.ax.set_ylim(min_y, max_y)
        self.ax.set_aspect('equal', adjustable='box')

    def update_time_axes_limits(self):
        if len(self.time_data) == 0:
            return

        t_min = 0.0
        t_max = max(1.0, self.time_data[-1])

        time_axes = [
            self.ax_x_t,
            self.ax_y_t,
            self.ax_yaw_t,
            self.ax_v_t
        ]

        for axis in time_axes:
            axis.set_xlim(t_min, t_max)
            axis.relim()
            axis.autoscale_view(scalex=False, scaley=True)

    # ======================================================
    # SHUTDOWN
    # ======================================================
    def destroy_node(self):
        save_plot_on_exit = bool(
            self.get_parameter('save_plot_on_exit').value
        )

        if save_plot_on_exit:
            save_path = str(
                self.get_parameter('save_path').value
            )

            try:
                save_dir = os.path.dirname(save_path)
                if save_dir:
                    os.makedirs(save_dir, exist_ok=True)

                self.fig.savefig(save_path, dpi=150)
                self.get_logger().info(
                    f'Saved final plot to: {save_path}'
                )

            except Exception as e:
                self.get_logger().warn(
                    f'Could not save final plot: {e}'
                )

        try:
            plt.close(self.fig)
        except Exception:
            pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = PathPlotterTeam18()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()