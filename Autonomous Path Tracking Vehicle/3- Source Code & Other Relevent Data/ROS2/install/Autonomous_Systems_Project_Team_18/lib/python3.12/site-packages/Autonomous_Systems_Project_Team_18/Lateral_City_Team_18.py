import math
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Float64MultiArray
from visualization_msgs.msg import Marker, MarkerArray


# ─────────────────────────────────────────────────────────────────
#  MS4 Stanley Control Node — City Track — Team 18
#
#  This node subscribes to:
#    /model/vehicle_blue/odometry
#    /team18/path_x
#    /team18/path_y
#
#  It publishes:
#    /team18/steer_cmd
#
#  Stanley equation:
#    delta = heading_error + atan(K * cross_track_error / front_speed)
#
#  Important:
#    - The planner already has a 20 second speed delay.
#    - This controller also keeps steer = 0 during the same 20 second delay.
#    - After the delay, Stanley starts immediately.
#    - There is NO extra straight lock using x or y.
#
#  Tuned for the provided city planner:
#    SPEED_CRUISE = 0.20 m/s
#    SPEED_CURVE  = 0.15 m/s
#    START_DELAY  = 20.0 sec
#    path uses exact centerline points
#
#  RViz fixed frame:
#    odom
# ─────────────────────────────────────────────────────────────────


WHEELBASE = 0.302284


STANLEY_GAIN = 0.65
SPEED_SOFTENING = 0.12
MAX_STEER_CMD = 0.42
STEER_FILTER_ALPHA = 0.70

# Same delay as the planner.
START_DELAY_SEC = 20.0


def clamp(value, minimum, maximum):
    return max(minimum, min(value, maximum))


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class StanleyControlNode(Node):
    def __init__(self):
        super().__init__('stanley_control_node_team_18')

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.current_speed = 0.0

        self.received_odom = False

        self.path_x = []
        self.path_y = []
        self.received_path_x = False
        self.received_path_y = False

        self.last_nearest_index = 0
        self.previous_steer_cmd = 0.0

        self.start_time = self.get_clock().now()

        # Store trajectory points for RViz.
        self.trajectory_points = []

        self.odom_sub = self.create_subscription(
            Odometry,
            '/team18/filtered_odom',
            self.odom_callback,
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

        self.steer_pub = self.create_publisher(
            Twist,
            '/team18/steer_cmd',
            10
        )

        # RViz debug publishers.
        self.path_markers_pub = self.create_publisher(
            MarkerArray,
            '/team18/path_markers',
            10
        )

        self.path_line_pub = self.create_publisher(
            Marker,
            '/team18/path_line',
            10
        )

        self.target_marker_pub = self.create_publisher(
            Marker,
            '/team18/target_marker',
            10
        )

        self.nearest_marker_pub = self.create_publisher(
            Marker,
            '/team18/nearest_marker',
            10
        )

        self.error_line_pub = self.create_publisher(
            Marker,
            '/team18/lookahead_line',
            10
        )

        self.vehicle_marker_pub = self.create_publisher(
            Marker,
            '/team18/vehicle_marker',
            10
        )

        self.trajectory_pub = self.create_publisher(
            Marker,
            '/team18/vehicle_trajectory',
            10
        )

        # 20 Hz control loop.
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info(
            'Stanley control node started.\n'
            f'  Wheelbase = {WHEELBASE:.4f} m\n'
            f'  Stanley gain = {STANLEY_GAIN:.2f}\n'
            f'  Speed softening = {SPEED_SOFTENING:.2f}\n'
            f'  Max steer command = {MAX_STEER_CMD:.2f} rad\n'
            f'  Start steering delay = {START_DELAY_SEC:.1f} sec\n'
            f'  Steering filter alpha = {STEER_FILTER_ALPHA:.2f}\n'
            '  No initial straight lock is used.\n'
            '  RViz fixed frame should be: odom'
        )

    # ─────────────────────────────────────────────────────────────
    # ROS callbacks
    # ─────────────────────────────────────────────────────────────

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation

        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)

        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

        self.current_speed = msg.twist.twist.linear.x

        self.received_odom = True

    def path_x_callback(self, msg):
        self.path_x = list(msg.data)
        self.received_path_x = True

    def path_y_callback(self, msg):
        self.path_y = list(msg.data)
        self.received_path_y = True

    # ─────────────────────────────────────────────────────────────
    # Helper functions
    # ─────────────────────────────────────────────────────────────

    def path_is_ready(self):
        if not self.received_path_x:
            return False

        if not self.received_path_y:
            return False

        if len(self.path_x) < 2:
            return False

        if len(self.path_x) != len(self.path_y):
            return False

        return True

    def get_front_axle_position(self):
        """
        Stanley uses the center of the front axle as the control point.
        """

        front_x = self.current_x + WHEELBASE * math.cos(self.current_yaw)
        front_y = self.current_y + WHEELBASE * math.sin(self.current_yaw)

        return front_x, front_y

    def find_closest_path_segment(self, front_x, front_y):
        """
        Find the closest point on a path segment to the front axle.

        The search starts slightly before the previous nearest segment.
        This prevents large jumps backward while still allowing recovery.
        """

        start_index = max(0, self.last_nearest_index - 2)

        best_segment_index = start_index
        best_closest_x = self.path_x[start_index]
        best_closest_y = self.path_y[start_index]
        best_path_yaw = 0.0
        best_cross_track_error = 0.0
        best_distance = float('inf')

        for i in range(start_index, len(self.path_x) - 1):
            x1 = self.path_x[i]
            y1 = self.path_y[i]
            x2 = self.path_x[i + 1]
            y2 = self.path_y[i + 1]

            segment_dx = x2 - x1
            segment_dy = y2 - y1
            segment_length_sq = segment_dx * segment_dx + segment_dy * segment_dy

            if segment_length_sq < 1e-9:
                continue

            # Projection of the front axle onto the segment.
            t = (
                ((front_x - x1) * segment_dx + (front_y - y1) * segment_dy)
                / segment_length_sq
            )
            t = clamp(t, 0.0, 1.0)

            closest_x = x1 + t * segment_dx
            closest_y = y1 + t * segment_dy

            error_x = front_x - closest_x
            error_y = front_y - closest_y

            distance = math.sqrt(error_x * error_x + error_y * error_y)

            if distance < best_distance:
                segment_length = math.sqrt(segment_length_sq)

                tangent_x = segment_dx / segment_length
                tangent_y = segment_dy / segment_length

                # Signed cross-track error.
                # This sign was used because it matches the Gazebo car's
                # steering convention in the previous working controller.
                cross_track_error = -(
                    tangent_x * error_y - tangent_y * error_x
                )

                best_segment_index = i
                best_closest_x = closest_x
                best_closest_y = closest_y
                best_path_yaw = math.atan2(segment_dy, segment_dx)
                best_cross_track_error = cross_track_error
                best_distance = distance

        self.last_nearest_index = best_segment_index

        return {
            'segment_index': best_segment_index,
            'closest_x': best_closest_x,
            'closest_y': best_closest_y,
            'path_yaw': best_path_yaw,
            'cross_track_error': best_cross_track_error,
            'distance': best_distance,
        }

    def smooth_steer(self, raw_steer_cmd):
        """
        First-order steering smoothing.
        """

        filtered = (
            STEER_FILTER_ALPHA * raw_steer_cmd
            + (1.0 - STEER_FILTER_ALPHA) * self.previous_steer_cmd
        )

        self.previous_steer_cmd = filtered
        return filtered

    def publish_steer(self, steer_cmd):
        msg = Twist()
        msg.angular.z = float(steer_cmd)
        self.steer_pub.publish(msg)

    def publish_zero_steer(self):
        self.previous_steer_cmd = 0.0
        self.publish_steer(0.0)

    # ─────────────────────────────────────────────────────────────
    # RViz helper functions
    # ─────────────────────────────────────────────────────────────

    def make_point(self, x, y, z):
        p = Point()
        p.x = float(x)
        p.y = float(y)
        p.z = float(z)
        return p

    def publish_path_markers(self, nearest_index):
        marker_array = MarkerArray()
        now = self.get_clock().now().to_msg()

        for i in range(len(self.path_x)):
            marker = Marker()
            marker.header.frame_id = 'odom'
            marker.header.stamp = now
            marker.ns = 'path_points'
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            marker.pose.position.x = float(self.path_x[i])
            marker.pose.position.y = float(self.path_y[i])
            marker.pose.position.z = 0.08

            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.05

            if i == nearest_index:
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
            else:
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0

            marker.color.a = 1.0
            marker_array.markers.append(marker)

        self.path_markers_pub.publish(marker_array)

    def publish_path_line(self):
        if not self.path_is_ready():
            return

        now = self.get_clock().now().to_msg()

        line = Marker()
        line.header.frame_id = 'odom'
        line.header.stamp = now
        line.ns = 'path_line'
        line.id = 1000
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD

        line.scale.x = 0.015

        line.color.r = 0.0
        line.color.g = 0.0
        line.color.b = 1.0
        line.color.a = 1.0

        for i in range(len(self.path_x)):
            line.points.append(
                self.make_point(self.path_x[i], self.path_y[i], 0.04)
            )

        self.path_line_pub.publish(line)

    def publish_closest_marker(self, closest_x, closest_y):
        now = self.get_clock().now().to_msg()

        marker = Marker()
        marker.header.frame_id = 'odom'
        marker.header.stamp = now
        marker.ns = 'stanley_closest_point'
        marker.id = 2000
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = float(closest_x)
        marker.pose.position.y = float(closest_y)
        marker.pose.position.z = 0.14

        marker.scale.x = 0.09
        marker.scale.y = 0.09
        marker.scale.z = 0.09

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        self.target_marker_pub.publish(marker)

    def publish_nearest_marker(self, closest_x, closest_y):
        now = self.get_clock().now().to_msg()

        marker = Marker()
        marker.header.frame_id = 'odom'
        marker.header.stamp = now
        marker.ns = 'nearest_path_point'
        marker.id = 3000
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        marker.pose.position.x = float(closest_x)
        marker.pose.position.y = float(closest_y)
        marker.pose.position.z = 0.12

        marker.scale.x = 0.07
        marker.scale.y = 0.07
        marker.scale.z = 0.07

        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        self.nearest_marker_pub.publish(marker)

    def publish_error_line(self, front_x, front_y, closest_x, closest_y):
        now = self.get_clock().now().to_msg()

        line = Marker()
        line.header.frame_id = 'odom'
        line.header.stamp = now
        line.ns = 'stanley_error_line'
        line.id = 4000
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD

        line.scale.x = 0.012

        line.color.r = 1.0
        line.color.g = 0.0
        line.color.b = 1.0
        line.color.a = 1.0

        line.points.append(
            self.make_point(front_x, front_y, 0.10)
        )
        line.points.append(
            self.make_point(closest_x, closest_y, 0.10)
        )

        self.error_line_pub.publish(line)

    def publish_vehicle_marker(self):
        now = self.get_clock().now().to_msg()

        vehicle = Marker()
        vehicle.header.frame_id = 'odom'
        vehicle.header.stamp = now
        vehicle.ns = 'vehicle'
        vehicle.id = 6000
        vehicle.type = Marker.ARROW
        vehicle.action = Marker.ADD

        vehicle.pose.position.x = float(self.current_x)
        vehicle.pose.position.y = float(self.current_y)
        vehicle.pose.position.z = 0.12

        vehicle.pose.orientation.z = math.sin(self.current_yaw / 2.0)
        vehicle.pose.orientation.w = math.cos(self.current_yaw / 2.0)

        vehicle.scale.x = 0.25
        vehicle.scale.y = 0.06
        vehicle.scale.z = 0.06

        vehicle.color.r = 1.0
        vehicle.color.g = 1.0
        vehicle.color.b = 0.0
        vehicle.color.a = 1.0

        self.vehicle_marker_pub.publish(vehicle)

    def publish_trajectory(self):
        now = self.get_clock().now().to_msg()

        self.trajectory_points.append(
            self.make_point(self.current_x, self.current_y, 0.03)
        )

        if len(self.trajectory_points) > 1000:
            self.trajectory_points.pop(0)

        traj = Marker()
        traj.header.frame_id = 'odom'
        traj.header.stamp = now
        traj.ns = 'vehicle_trajectory'
        traj.id = 7000
        traj.type = Marker.LINE_STRIP
        traj.action = Marker.ADD

        traj.scale.x = 0.012

        traj.color.r = 1.0
        traj.color.g = 0.5
        traj.color.b = 0.0
        traj.color.a = 1.0

        traj.points = self.trajectory_points

        self.trajectory_pub.publish(traj)

    def publish_all_debug_markers(
        self,
        front_x,
        front_y,
        closest_x,
        closest_y,
        nearest_index
    ):
        self.publish_path_markers(nearest_index)
        self.publish_path_line()
        self.publish_closest_marker(closest_x, closest_y)
        self.publish_nearest_marker(closest_x, closest_y)
        self.publish_error_line(front_x, front_y, closest_x, closest_y)
        self.publish_vehicle_marker()
        self.publish_trajectory()

    def publish_basic_debug_markers(self):
        self.publish_path_line()
        self.publish_vehicle_marker()
        self.publish_trajectory()

    # ─────────────────────────────────────────────────────────────
    # Stanley control loop
    # ─────────────────────────────────────────────────────────────

    def control_loop(self):
        if not self.received_odom:
            return

        if not self.path_is_ready():
            self.get_logger().warn(
                'Waiting for path from planning node...',
                throttle_duration_sec=1.0
            )
            return

        elapsed_time = (
            self.get_clock().now() - self.start_time
        ).nanoseconds / 1e9

        # Steering delay only.
        # No extra straight lock after this.
        if elapsed_time < START_DELAY_SEC:
            self.last_nearest_index = 0
            self.publish_zero_steer()
            self.publish_basic_debug_markers()

            self.get_logger().info(
                f'Start delay active: '
                f'{elapsed_time:.1f}/{START_DELAY_SEC:.1f} sec | '
                f'steer=0.000',
                throttle_duration_sec=0.5
            )
            return

        front_x, front_y = self.get_front_axle_position()

        closest = self.find_closest_path_segment(front_x, front_y)

        path_yaw = closest['path_yaw']
        cross_track_error = closest['cross_track_error']
        closest_x = closest['closest_x']
        closest_y = closest['closest_y']
        nearest_index = closest['segment_index']

        # Heading correction.
        heading_error = normalize_angle(path_yaw - self.current_yaw)

        # Stanley cross-track correction.
        front_speed = abs(self.current_speed) + SPEED_SOFTENING

        cross_track_correction = math.atan2(
            STANLEY_GAIN * cross_track_error,
            front_speed
        )

        # Stanley steering law.
        raw_steer_cmd = heading_error + cross_track_correction
        raw_steer_cmd = normalize_angle(raw_steer_cmd)
        raw_steer_cmd = clamp(
            raw_steer_cmd,
            -MAX_STEER_CMD,
            MAX_STEER_CMD
        )

        steer_cmd = self.smooth_steer(raw_steer_cmd)
        steer_cmd = clamp(
            steer_cmd,
            -MAX_STEER_CMD,
            MAX_STEER_CMD
        )

        self.publish_steer(steer_cmd)

        self.publish_all_debug_markers(
            front_x,
            front_y,
            closest_x,
            closest_y,
            nearest_index
        )

        self.get_logger().info(
            f'pos=({self.current_x:.2f},{self.current_y:.2f}) | '
            f'yaw={math.degrees(self.current_yaw):.1f} deg | '
            f'front=({front_x:.2f},{front_y:.2f}) | '
            f'v={self.current_speed:.2f} | '
            f'segment={nearest_index} | '
            f'path_yaw={math.degrees(path_yaw):.1f} deg | '
            f'heading_error={math.degrees(heading_error):.1f} deg | '
            f'e={cross_track_error:.3f} | '
            f'cte_corr={math.degrees(cross_track_correction):.1f} deg | '
            f'raw_steer={raw_steer_cmd:.3f} | '
            f'steer={steer_cmd:.3f}',
            throttle_duration_sec=0.5
        )


def main(args=None):
    rclpy.init(args=args)

    node = StanleyControlNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()

    try:
        rclpy.shutdown()
    except Exception:
        pass


if __name__ == '__main__':
    main()
