#!/usr/bin/env python3
"""
MS5 Graph Recorder — Team 18
ROS 2 node. Run alongside your simulation, press Ctrl+C to save all plots.

Exact topics wired from your code:
  /model/vehicle_blue/odometry  →  nav_msgs/Odometry           (ideal Gazebo)
  /team18/noisy_odom            →  nav_msgs/Odometry           (noisy pre-KF)
  /team18/filtered_odom         →  nav_msgs/Odometry           (KF filtered)
  /team18/target_speed          →  std_msgs/Float64            (desired speed)
  /team18/speed_cmd             →  geometry_msgs/Twist         (speed controller output)
  /team18/steer_cmd             →  geometry_msgs/Twist         (pure-pursuit steer)
  /team18/path_x                →  std_msgs/Float64MultiArray  (planned x waypoints)
  /team18/path_y                →  std_msgs/Float64MultiArray  (planned y waypoints)

Saves 8 PNGs to:  ~/ms5_graphs/
Zip command:
  cd ~ && zip -r Autonomous_Systems_Project_Simulation_MS_5_Graphs_Team_18.zip ms5_graphs/
"""

import os
import math
import rclpy
from rclpy.node import Node

import numpy as np
import matplotlib
matplotlib.use('Agg')          # headless – no display needed
import matplotlib.pyplot as plt

from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Float64MultiArray
from geometry_msgs.msg import Twist

SAVE_DIR = os.path.expanduser('~/ms5_graphs')


def yaw_from_quat(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class MS5Plotter(Node):
    def __init__(self):
        super().__init__('ms5_plotter_team_18')
        self.t0 = self.get_clock().now()

        # Storage: key → ([time], [value])
        keys = [
            'ideal_x', 'ideal_y', 'ideal_theta', 'ideal_v',
            'noisy_x', 'noisy_y', 'noisy_theta', 'noisy_v',
            'filt_x',  'filt_y',  'filt_theta',  'filt_v',
            'des_speed', 'act_speed', 'steer_cmd',
        ]
        self.d = {k: ([], []) for k in keys}
        self.path_x = []
        self.path_y = []

        # ── Subscribers ───────────────────────────────────────────
        self.create_subscription(
            Odometry, '/model/vehicle_blue/odometry', self._cb_ideal, 10)
        self.create_subscription(
            Odometry, '/team18/noisy_odom', self._cb_noisy, 10)
        self.create_subscription(
            Odometry, '/team18/filtered_odom', self._cb_filt, 10)
        self.create_subscription(
            Float64, '/team18/target_speed', self._cb_des_spd, 10)
        # speed_cmd is geometry_msgs/Twist; linear.x carries the speed value
        self.create_subscription(
            Twist, '/team18/speed_cmd', self._cb_act_spd, 10)
        # steer_cmd is geometry_msgs/Twist; angular.z carries the steer value
        self.create_subscription(
            Twist, '/team18/steer_cmd', self._cb_steer, 10)
        self.create_subscription(
            Float64MultiArray, '/team18/path_x', self._cb_path_x, 10)
        self.create_subscription(
            Float64MultiArray, '/team18/path_y', self._cb_path_y, 10)

        self.get_logger().info(
            '[MS5 Plotter] Recording … press Ctrl+C to save graphs.')

    # ── helpers ───────────────────────────────────────────────────
    def _t(self):
        return (self.get_clock().now() - self.t0).nanoseconds / 1e9

    def _app(self, key, val):
        self.d[key][0].append(self._t())
        self.d[key][1].append(val)

    # ── callbacks ─────────────────────────────────────────────────
    def _cb_ideal(self, msg):
        p = msg.pose.pose.position
        self._app('ideal_x',     p.x)
        self._app('ideal_y',     p.y)
        self._app('ideal_theta', yaw_from_quat(msg.pose.pose.orientation))
        self._app('ideal_v',     msg.twist.twist.linear.x)

    def _cb_noisy(self, msg):
        p = msg.pose.pose.position
        self._app('noisy_x',     p.x)
        self._app('noisy_y',     p.y)
        self._app('noisy_theta', yaw_from_quat(msg.pose.pose.orientation))
        self._app('noisy_v',     msg.twist.twist.linear.x)

    def _cb_filt(self, msg):
        p = msg.pose.pose.position
        self._app('filt_x',     p.x)
        self._app('filt_y',     p.y)
        self._app('filt_theta', yaw_from_quat(msg.pose.pose.orientation))
        self._app('filt_v',     msg.twist.twist.linear.x)

    def _cb_des_spd(self, msg):  self._app('des_speed', msg.data)
    def _cb_act_spd(self, msg):  self._app('act_speed', msg.linear.x)
    def _cb_steer(self,   msg):  self._app('steer_cmd', msg.angular.z)
    def _cb_path_x(self,  msg):  self.path_x = list(msg.data)
    def _cb_path_y(self,  msg):  self.path_y = list(msg.data)

    # ── figure helpers ────────────────────────────────────────────
    def _savefig(self, fig, fname):
        os.makedirs(SAVE_DIR, exist_ok=True)
        out = os.path.join(SAVE_DIR, fname)
        fig.savefig(out, dpi=150, bbox_inches='tight')
        self.get_logger().info(f'  saved → {out}')
        plt.close(fig)

    def _plot3(self, title, ylabel, fname,
               t_i, y_i, t_n, y_n, t_f, y_f):
        """Ideal / Noisy / Filtered."""
        fig, ax = plt.subplots(figsize=(11, 4))
        if t_i:
            ax.plot(t_i, y_i, color='green',     lw=1.4, alpha=0.8,
                    label='Ideal (Gazebo)')
        if t_n:
            ax.plot(t_n, y_n, color='darkorange', lw=0.9, alpha=0.7,
                    label='Noisy (sensor sim)')
        if t_f:
            ax.plot(t_f, y_f, color='steelblue',  lw=2.0,
                    label='Filtered (KF)', zorder=5)
        ax.set_title(title, fontsize=13, fontweight='bold')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel(ylabel)
        ax.legend(loc='best', fontsize=9)
        ax.grid(True, alpha=0.3)
        fig.tight_layout()
        self._savefig(fig, fname)

    def _plot2(self, title, ylabel, fname,
               t1, y1, lbl1, t2, y2, lbl2, c1='steelblue', c2='tomato'):
        fig, ax = plt.subplots(figsize=(11, 4))
        if t1:
            ax.plot(t1, y1, color=c1, lw=2.0, label=lbl1)
        if t2:
            ax.plot(t2, y2, color=c2, lw=2.0, linestyle='--', label=lbl2)
        ax.set_title(title, fontsize=13, fontweight='bold')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel(ylabel)
        ax.legend(loc='best', fontsize=9)
        ax.grid(True, alpha=0.3)
        fig.tight_layout()
        self._savefig(fig, fname)

    def _plot1(self, title, ylabel, fname, t, y, color='purple'):
        fig, ax = plt.subplots(figsize=(11, 4))
        if t:
            ax.plot(t, y, color=color, lw=2.0)
        ax.set_title(title, fontsize=13, fontweight='bold')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel(ylabel)
        ax.grid(True, alpha=0.3)
        fig.tight_layout()
        self._savefig(fig, fname)

    # ── save all plots ────────────────────────────────────────────
    def save_all(self):
        self.get_logger().info('[MS5 Plotter] Saving 8 graphs …')
        d = self.d

        # 1. KF — X
        self._plot3(
            'Localization: X Position — Ideal vs Noisy vs KF Filtered',
            'x (m)', '01_kf_x.png',
            *d['ideal_x'], *d['noisy_x'], *d['filt_x'])

        # 2. KF — Y
        self._plot3(
            'Localization: Y Position — Ideal vs Noisy vs KF Filtered',
            'y (m)', '02_kf_y.png',
            *d['ideal_y'], *d['noisy_y'], *d['filt_y'])

        # 3. KF — Theta
        self._plot3(
            'Localization: Heading θ — Ideal vs Noisy vs KF Filtered',
            'θ (rad)', '03_kf_theta.png',
            *d['ideal_theta'], *d['noisy_theta'], *d['filt_theta'])

        # 4. KF — Speed
        self._plot3(
            'Localization: Speed — Ideal vs Noisy vs KF Filtered',
            'v (m/s)', '04_kf_speed.png',
            *d['ideal_v'], *d['noisy_v'], *d['filt_v'])

        # 5. Longitudinal control: desired vs actual speed
        self._plot2(
            'Longitudinal Control: Desired vs Actual Speed',
            'speed (m/s)', '05_speed_tracking.png',
            *d['des_speed'], 'Desired (/team18/target_speed)',
            *d['act_speed'], 'Actual  (/team18/speed_cmd)')

        # 6. Lateral control: planned lane y vs actual y
        fig, ax = plt.subplots(figsize=(11, 4))
        tf, yf = d['filt_y']
        if tf:
            ax.plot(tf, yf, color='steelblue', lw=2.0,
                    label='Actual y (filtered_odom)')
        # Draw planned lane targets as dashed horizontal reference lines
        if self.path_y:
            unique_y = sorted(set(round(v, 4) for v in self.path_y))
            for uy in unique_y:
                ax.axhline(uy, color='tomato', lw=1.2, linestyle='--', alpha=0.8)
            # dummy line for legend
            ax.axhline(unique_y[0], color='tomato', lw=1.2, linestyle='--',
                       alpha=0.8, label='Planned lane y (path_y waypoints)')
        ax.set_title('Lateral Control: Planned Lane vs Actual Y Position',
                     fontsize=13, fontweight='bold')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('y (m)')
        ax.legend(loc='best', fontsize=9)
        ax.grid(True, alpha=0.3)
        fig.tight_layout()
        self._savefig(fig, '06_lane_tracking.png')

        # 7. Steering command
        self._plot1(
            'Steering Command over Time',
            'steer_cmd (rad)', '07_steer_cmd.png',
            *d['steer_cmd'], color='purple')

        # 8. XY trajectory
        fig, ax = plt.subplots(figsize=(9, 9))
        if self.path_x and self.path_y:
            ax.plot(self.path_x, self.path_y, 'rs--', lw=1.5, ms=8,
                    label='Planned path (waypoints)', zorder=4)
        ti_x, xi = d['ideal_x'];  _, yi_ = d['ideal_y']
        tf_x, xf = d['filt_x'];   _, yf_ = d['filt_y']
        if ti_x:
            ax.plot(xi,  yi_,  color='green',     lw=1.2, alpha=0.7,
                    label='Ideal trajectory')
        if tf_x:
            ax.plot(xf,  yf_,  color='steelblue', lw=2.0,
                    label='Filtered trajectory')
        ax.set_title('XY Trajectory: Planned vs Actual',
                     fontsize=13, fontweight='bold')
        ax.set_xlabel('x (m)')
        ax.set_ylabel('y (m)')
        ax.set_aspect('equal', 'datalim')
        ax.legend(loc='best', fontsize=9)
        ax.grid(True, alpha=0.3)
        fig.tight_layout()
        self._savefig(fig, '08_xy_trajectory.png')

        self.get_logger().info(
            f'[MS5 Plotter] All 8 graphs saved to {SAVE_DIR}\n'
            '  Zip command:\n'
            '  cd ~ && zip -r '
            'Autonomous_Systems_Project_Simulation_MS_5_Graphs_Team_18.zip '
            'ms5_graphs/')


def main(args=None):
    rclpy.init(args=args)
    node = MS5Plotter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.save_all()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
