import math
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Float64MultiArray
from visualization_msgs.msg import Marker, MarkerArray


# ─────────────────────────────────────────────────────────────────
#  MS4 Pure Pursuit Control Node — Team 18
#
#  This node subscribes to:
#    /model/vehicle_blue/odometry
#    /team18/path_x
#    /team18/path_y
#
#  It publishes:
#    /team18/steer_cmd
#
#  The planning node provides the path.
#  This control node only follows the path using Pure Pursuit.
#
#  RViz debug topics:
#    /team18/path_markers
#    /team18/path_line
#    /team18/target_marker
#    /team18/nearest_marker
#    /team18/lookahead_line
#    /team18/lookahead_circle
#    /team18/vehicle_marker
#    /team18/vehicle_trajectory
#
#  RViz fixed frame:
#    odom
# ─────────────────────────────────────────────────────────────────


WHEELBASE = 0.302284

LOOKAHEAD_GAIN = 0.80
MIN_LOOKAHEAD = 0.35
MAX_LOOKAHEAD = 0.80

MAX_STEER_CMD = 0.45

# Wall protection for 0.75 m track.
# Track edge is at y = ±0.375.
# Do not let the car center go near this limit.
SAFE_Y_LIMIT = 0.300


def clamp(value, minimum, maximum):
    return max(minimum, min(value, maximum))


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class PurePursuitControlNode(Node):
    def __init__(self):
        super().__init__('pure_pursuit_control_node_team_18')

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

        # Store trajectory points for RViz
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

        # ─────────────────────────────────────────────────────────
        # RViz debug publishers
        # ─────────────────────────────────────────────────────────
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

        self.lookahead_line_pub = self.create_publisher(
            Marker,
            '/team18/lookahead_line',
            10
        )

        self.lookahead_circle_pub = self.create_publisher(
            Marker,
            '/team18/lookahead_circle',
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

        # 20 Hz control loop
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info(
            'Pure Pursuit control node started.\n'
            f'  Wheelbase = {WHEELBASE:.4f} m\n'
            f'  Lookahead gain = {LOOKAHEAD_GAIN:.2f}\n'
            f'  Min lookahead = {MIN_LOOKAHEAD:.2f} m\n'
            f'  Max lookahead = {MAX_LOOKAHEAD:.2f} m\n'
            f'  Max steer command = {MAX_STEER_CMD:.2f}\n'
            '  RViz debug markers enabled.\n'
            '  RViz fixed frame should be: odom'
        )

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

    def get_lookahead_distance(self):
        """
        Variable lookahead:
            Ld = k * |v|

        Clamped between MIN_LOOKAHEAD and MAX_LOOKAHEAD so that:
          - At zero/low speed the car still steers (avoids atan2(x,0))
          - At high speed the lookahead does not grow unbounded
        """
        ld = LOOKAHEAD_GAIN * abs(self.current_speed)
        ld = clamp(ld, MIN_LOOKAHEAD, MAX_LOOKAHEAD)
        return ld

    def find_nearest_index(self):
        """
        Find nearest waypoint to the car.

        To avoid jumping backward too much, search from slightly before
        the previous nearest index.
        """
        if not self.path_is_ready():
            return 0

        start_index = max(0, self.last_nearest_index - 2)

        nearest_index = start_index
        nearest_distance = float('inf')

        for i in range(start_index, len(self.path_x)):
            dx = self.path_x[i] - self.current_x
            dy = self.path_y[i] - self.current_y
            distance = math.sqrt(dx * dx + dy * dy)

            if distance < nearest_distance:
                nearest_distance = distance
                nearest_index = i

        self.last_nearest_index = nearest_index
        return nearest_index

    def find_lookahead_point(self, lookahead_distance):
        """
        Find target point ahead of the nearest point.

        This is better than simply taking the first point whose direct
        distance from the car is larger than lookahead distance.
        """

        nearest_index = self.find_nearest_index()

        accumulated_distance = 0.0

        for i in range(nearest_index, len(self.path_x) - 1):
            x1 = self.path_x[i]
            y1 = self.path_y[i]

            x2 = self.path_x[i + 1]
            y2 = self.path_y[i + 1]

            segment_dx = x2 - x1
            segment_dy = y2 - y1
            segment_length = math.sqrt(segment_dx * segment_dx + segment_dy * segment_dy)

            accumulated_distance += segment_length

            if accumulated_distance >= lookahead_distance:
                return x2, y2, i + 1

        # If the car is near the end, use final point.
        last_index = len(self.path_x) - 1
        return self.path_x[last_index], self.path_y[last_index], last_index

    # ─────────────────────────────────────────────────────────────
    # RViz helper functions
    # ─────────────────────────────────────────────────────────────

    def make_point(self, x, y, z):
        p = Point()
        p.x = float(x)
        p.y = float(y)
        p.z = float(z)
        return p

    def publish_path_markers(self, target_index):
        """
        Green spheres: all planned path points
        Red sphere: current target point
        """
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

            if i == target_index:
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
        """
        Blue line: full planned path from planner.
        """
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

    def publish_target_marker(self, target_x, target_y):
        """
        Red sphere: exact target point selected by Pure Pursuit.
        """
        now = self.get_clock().now().to_msg()

        target = Marker()
        target.header.frame_id = 'odom'
        target.header.stamp = now
        target.ns = 'pure_pursuit_target'
        target.id = 2000
        target.type = Marker.SPHERE
        target.action = Marker.ADD

        target.pose.position.x = float(target_x)
        target.pose.position.y = float(target_y)
        target.pose.position.z = 0.14

        target.scale.x = 0.09
        target.scale.y = 0.09
        target.scale.z = 0.09

        target.color.r = 1.0
        target.color.g = 0.0
        target.color.b = 0.0
        target.color.a = 1.0

        self.target_marker_pub.publish(target)

    def publish_nearest_marker(self):
        """
        Cyan cube: nearest path point currently used by the controller.
        This helps check if the controller is jumping to a wrong segment.
        """
        if not self.path_is_ready():
            return

        nearest_index = self.last_nearest_index

        if nearest_index < 0 or nearest_index >= len(self.path_x):
            return

        now = self.get_clock().now().to_msg()

        marker = Marker()
        marker.header.frame_id = 'odom'
        marker.header.stamp = now
        marker.ns = 'nearest_path_point'
        marker.id = 3000
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        marker.pose.position.x = float(self.path_x[nearest_index])
        marker.pose.position.y = float(self.path_y[nearest_index])
        marker.pose.position.z = 0.12

        marker.scale.x = 0.07
        marker.scale.y = 0.07
        marker.scale.z = 0.07

        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        self.nearest_marker_pub.publish(marker)

    def publish_lookahead_line(self, target_x, target_y):
        """
        Purple line: from vehicle position to selected target point.
        This shows exactly where Pure Pursuit is trying to steer.
        """
        now = self.get_clock().now().to_msg()

        line = Marker()
        line.header.frame_id = 'odom'
        line.header.stamp = now
        line.ns = 'lookahead_line'
        line.id = 4000
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD

        line.scale.x = 0.012

        line.color.r = 1.0
        line.color.g = 0.0
        line.color.b = 1.0
        line.color.a = 1.0

        line.points.append(
            self.make_point(self.current_x, self.current_y, 0.10)
        )
        line.points.append(
            self.make_point(target_x, target_y, 0.10)
        )

        self.lookahead_line_pub.publish(line)

    def publish_lookahead_circle(self, lookahead_distance):
        """
        Purple circle: radius = lookahead distance around vehicle.
        This helps see why a specific target point was selected.
        """
        now = self.get_clock().now().to_msg()

        circle = Marker()
        circle.header.frame_id = 'odom'
        circle.header.stamp = now
        circle.ns = 'lookahead_circle'
        circle.id = 5000
        circle.type = Marker.LINE_STRIP
        circle.action = Marker.ADD

        circle.scale.x = 0.008

        circle.color.r = 1.0
        circle.color.g = 0.0
        circle.color.b = 1.0
        circle.color.a = 0.7

        steps = 60

        for i in range(steps + 1):
            angle = 2.0 * math.pi * i / steps
            x = self.current_x + lookahead_distance * math.cos(angle)
            y = self.current_y + lookahead_distance * math.sin(angle)
            circle.points.append(
                self.make_point(x, y, 0.06)
            )

        self.lookahead_circle_pub.publish(circle)

    def publish_vehicle_marker(self):
        """
        Yellow arrow: vehicle position and heading from odometry.
        """
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
        """
        Orange line: actual vehicle trajectory.
        This lets you compare commanded path vs actual motion.
        """
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
        target_x,
        target_y,
        target_index,
        lookahead_distance
    ):
        self.publish_path_markers(target_index)
        self.publish_path_line()
        self.publish_target_marker(target_x, target_y)
        self.publish_nearest_marker()
        self.publish_lookahead_line(target_x, target_y)
        self.publish_lookahead_circle(lookahead_distance)
        self.publish_vehicle_marker()
        self.publish_trajectory()

    def control_loop(self):
        if not self.received_odom:
            return

        if not self.path_is_ready():
            self.get_logger().warn(
                'Waiting for path from planning node...',
                throttle_duration_sec=1.0
            )
            return

        lookahead_distance = self.get_lookahead_distance()

        target_x, target_y, target_index = self.find_lookahead_point(
            lookahead_distance
        )

        dx = target_x - self.current_x
        dy = target_y - self.current_y

        target_angle = math.atan2(dy, dx)

        # Alpha is the angle between vehicle heading and lookahead point.
        alpha = normalize_angle(target_angle - self.current_yaw)

        # Pure Pursuit equation.
        steer_cmd = math.atan2(
            2.0 * WHEELBASE * math.sin(alpha),
            lookahead_distance
        )

        steer_cmd = clamp(steer_cmd, -MAX_STEER_CMD, MAX_STEER_CMD)

        # Emergency wall protection.
        # Positive steer usually turns toward +y.
        # Negative steer usually turns toward -y.
        if self.current_y > SAFE_Y_LIMIT:
            steer_cmd = min(steer_cmd, -0.25)

        elif self.current_y < -SAFE_Y_LIMIT:
            steer_cmd = max(steer_cmd, 0.25)

        steer_cmd = clamp(steer_cmd, -MAX_STEER_CMD, MAX_STEER_CMD)

        msg = Twist()
        msg.angular.z = float(steer_cmd)
        self.steer_pub.publish(msg)

        # RViz debug markers only; does not affect control logic.
        self.publish_all_debug_markers(
            target_x,
            target_y,
            target_index,
            lookahead_distance
        )

        self.get_logger().info(
            f'pos=({self.current_x:.2f},{self.current_y:.3f}) | '
            f'v={self.current_speed:.2f} | '
            f'Ld={lookahead_distance:.2f} | '
            f'target#{target_index}=({target_x:.2f},{target_y:.3f}) | '
            f'alpha={math.degrees(alpha):.1f} deg | '
            f'steer={steer_cmd:.3f}',
            throttle_duration_sec=0.5
        )


def main(args=None):
    rclpy.init(args=args)

    node = PurePursuitControlNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
