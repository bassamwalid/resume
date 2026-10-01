import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import numpy as np
import math

# ─────────────────────────────────────────────────────────────────
#  MS5 Localization Node — Team 18
#
#  Pipeline:
#    1. Subscribe ideal Gazebo odometry  (/model/vehicle_blue/odometry)
#    2. Add white Gaussian noise to states [x, y, θ, v]
#       using std deviations declared via rosparam
#    3. Apply Kalman Filter with a dt-based kinematic prediction model
#    4. Publish filtered odom  → /team18/filtered_odom
#       Publish noisy odom    → /team18/noisy_odom  (for graphing)
#
#  ROS Parameters (set from launch file via rosparam):
#    sigma_pos   : position noise std dev   [m]
#    sigma_yaw   : heading noise std dev    [rad]
#    sigma_speed : speed noise std dev      [m/s]
#    q_pos       : process noise for x, y
#    q_yaw       : process noise for theta
#    q_speed     : process noise for v
#
#  Kinematic state transition (linearised unicycle, dt-based):
#    x_{k+1}  = x_k  + v_k * cos(θ_k) * dt
#    y_{k+1}  = y_k  + v_k * sin(θ_k) * dt
#    θ_{k+1}  = θ_k  + ω_k * dt
#    v_{k+1}  = v_k
#
#  F is rebuilt every callback around the current estimate.
# ─────────────────────────────────────────────────────────────────


class LocalizationNode(Node):
    def __init__(self):
        super().__init__('localization_node_team_18')

        # ── Declare ROS parameters (with sensible defaults) ───────
        self.declare_parameter('sigma_pos',   0.02)   # [m]
        self.declare_parameter('sigma_yaw',   0.01)   # [rad]
        self.declare_parameter('sigma_speed', 0.05)   # [m/s]
        self.declare_parameter('q_pos',       0.001)
        self.declare_parameter('q_yaw',       0.001)
        self.declare_parameter('q_speed',     0.005)

        # ── Read parameters ───────────────────────────────────────
        sigma_pos   = self.get_parameter('sigma_pos').value
        sigma_yaw   = self.get_parameter('sigma_yaw').value
        sigma_speed = self.get_parameter('sigma_speed').value
        q_pos       = self.get_parameter('q_pos').value
        q_yaw       = self.get_parameter('q_yaw').value
        q_speed     = self.get_parameter('q_speed').value

        self.sigma_pos   = sigma_pos
        self.sigma_yaw   = sigma_yaw
        self.sigma_speed = sigma_speed

        # ── Kalman Filter matrices ────────────────────────────────
        # State vector: [x, y, θ, v]
        self.x_hat = np.zeros(4)       # state estimate
        self.P     = np.eye(4) * 0.1   # error covariance

        # Process noise Q (tuned via rosparam)
        self.Q = np.diag([q_pos, q_pos, q_yaw, q_speed])

        # Measurement noise R (derived from sensor std devs)
        self.R = np.diag([
            sigma_pos   ** 2,
            sigma_pos   ** 2,
            sigma_yaw   ** 2,
            sigma_speed ** 2,
        ])

        # Measurement matrix H (we directly observe all 4 states)
        self.H = np.eye(4)

        self.initialized = False
        self.prev_time   = None   # ROS time of last callback

        # ── Publishers & Subscribers ──────────────────────────────
        self.filtered_pub = self.create_publisher(
            Odometry, '/team18/filtered_odom', 10)

        self.raw_pub = self.create_publisher(
            Odometry, '/team18/noisy_odom', 10)

        self.odom_sub = self.create_subscription(
            Odometry, '/model/vehicle_blue/odometry', self.odom_cb, 10)

        self.get_logger().info(
            'Localization node started.\n'
            f'  sigma_pos   = {sigma_pos} m\n'
            f'  sigma_yaw   = {sigma_yaw} rad\n'
            f'  sigma_speed = {sigma_speed} m/s\n'
            f'  Q = diag([{q_pos}, {q_pos}, {q_yaw}, {q_speed}])\n'
            '  Kinematic Kalman Filter active (dt-based F matrix).\n'
            '  Filtered output -> /team18/filtered_odom\n'
            '  Noisy output    -> /team18/noisy_odom'
        )

    # ─────────────────────────────────────────────────────────────
    # Helpers
    # ─────────────────────────────────────────────────────────────

    def extract_yaw(self, q):
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny, cosy)

    def build_F(self, theta, v, dt):
        """
        Linearised kinematic F matrix (Jacobian of unicycle model).

        Model:
            x'  = x + v*cos(theta)*dt
            y'  = y + v*sin(theta)*dt
            th' = th + omega*dt         (omega handled via Q)
            v'  = v

        Jacobian F = d[x',y',th',v'] / d[x,y,th,v]:
            row x  : [1,  0,  -v*sin(th)*dt,  cos(th)*dt]
            row y  : [0,  1,   v*cos(th)*dt,  sin(th)*dt]
            row th : [0,  0,   1,              0         ]
            row v  : [0,  0,   0,              1         ]
        """
        c = math.cos(theta)
        s = math.sin(theta)
        F = np.array([
            [1.0, 0.0, -v * s * dt,  c * dt],
            [0.0, 1.0,  v * c * dt,  s * dt],
            [0.0, 0.0,  1.0,         0.0   ],
            [0.0, 0.0,  0.0,         1.0   ],
        ])
        return F

    def predict_state(self, x_hat, theta, v, omega, dt):
        """Non-linear state propagation for the mean prediction."""
        return np.array([
            x_hat[0] + v * math.cos(theta) * dt,
            x_hat[1] + v * math.sin(theta) * dt,
            x_hat[2] + omega * dt,
            v,
        ])

    # ─────────────────────────────────────────────────────────────
    # Main callback
    # ─────────────────────────────────────────────────────────────

    def odom_cb(self, msg):
        # ── 1. Extract ideal states from Gazebo ───────────────────
        x_ideal   = msg.pose.pose.position.x
        y_ideal   = msg.pose.pose.position.y
        yaw_ideal = self.extract_yaw(msg.pose.pose.orientation)
        v_ideal   = msg.twist.twist.linear.x
        omega     = msg.twist.twist.angular.z   # used in prediction step

        # ── 2. Add Gaussian noise ─────────────────────────────────
        x_noisy   = x_ideal   + np.random.normal(0.0, self.sigma_pos)
        y_noisy   = y_ideal   + np.random.normal(0.0, self.sigma_pos)
        yaw_noisy = yaw_ideal + np.random.normal(0.0, self.sigma_yaw)
        v_noisy   = v_ideal   + np.random.normal(0.0, self.sigma_speed)

        z = np.array([x_noisy, y_noisy, yaw_noisy, v_noisy])

        # Publish noisy odom for comparison/graphing
        self.publish_odom(self.raw_pub, msg, x_noisy, y_noisy, yaw_noisy, v_noisy)

        # ── 3. Compute dt ─────────────────────────────────────────
        current_time = self.get_clock().now()

        if not self.initialized:
            self.x_hat     = z.copy()
            self.prev_time = current_time
            self.initialized = True
            self.publish_odom(
                self.filtered_pub, msg,
                self.x_hat[0], self.x_hat[1],
                self.x_hat[2], self.x_hat[3]
            )
            return

        dt = (current_time - self.prev_time).nanoseconds / 1e9
        self.prev_time = current_time

        # Guard against zero or bad dt
        if dt <= 0.0 or dt > 1.0:
            dt = 0.05   # fallback: assume 20 Hz

        # ── 4. Kalman Filter ──────────────────────────────────────

        # — Prediction step —
        theta_est = self.x_hat[2]
        v_est     = self.x_hat[3]

        x_pred = self.predict_state(self.x_hat, theta_est, v_est, omega, dt)
        F      = self.build_F(theta_est, v_est, dt)
        P_pred = F @ self.P @ F.T + self.Q

        # — Update step —
        S = self.H @ P_pred @ self.H.T + self.R
        K = P_pred @ self.H.T @ np.linalg.inv(S)   # Kalman gain

        innovation    = z - self.H @ x_pred
        # Normalise angle innovation to [-pi, pi]
        innovation[2] = math.atan2(
            math.sin(innovation[2]),
            math.cos(innovation[2])
        )

        self.x_hat = x_pred + K @ innovation
        self.P     = (np.eye(4) - K @ self.H) @ P_pred

        # Keep estimated yaw in [-pi, pi]
        self.x_hat[2] = math.atan2(
            math.sin(self.x_hat[2]),
            math.cos(self.x_hat[2])
        )

        # ── 5. Publish filtered odom ──────────────────────────────
        self.publish_odom(
            self.filtered_pub, msg,
            self.x_hat[0], self.x_hat[1],
            self.x_hat[2], self.x_hat[3]
        )

        self.get_logger().info(
            f'x: ideal={x_ideal:.3f}  noisy={x_noisy:.3f}  filtered={self.x_hat[0]:.3f} | '
            f'y: ideal={y_ideal:.3f}  noisy={y_noisy:.3f}  filtered={self.x_hat[1]:.3f} | '
            f'th(deg): ideal={math.degrees(yaw_ideal):.1f}  '
            f'filtered={math.degrees(self.x_hat[2]):.1f}',
            throttle_duration_sec=0.5
        )

    # ─────────────────────────────────────────────────────────────
    # Publisher helper
    # ─────────────────────────────────────────────────────────────

    def publish_odom(self, publisher, original_msg, x, y, yaw, v):
        msg = Odometry()
        msg.header         = original_msg.header
        msg.child_frame_id = original_msg.child_frame_id

        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.position.z = 0.0

        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = float(math.sin(yaw / 2.0))
        msg.pose.pose.orientation.w = float(math.cos(yaw / 2.0))

        msg.twist.twist.linear.x  = float(v)
        msg.twist.twist.angular.z = original_msg.twist.twist.angular.z

        publisher.publish(msg)


def main():
    rclpy.init()
    rclpy.spin(LocalizationNode())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
