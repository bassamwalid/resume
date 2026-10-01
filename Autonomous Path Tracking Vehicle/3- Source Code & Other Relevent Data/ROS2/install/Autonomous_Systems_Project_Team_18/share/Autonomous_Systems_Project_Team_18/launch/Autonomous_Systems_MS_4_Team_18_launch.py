from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
import os

PACKAGE_NAME = 'Autonomous_Systems_Project_Team_18'

WORLDS_DIR = os.path.expanduser(
    f'~/ros2_ws/src/{PACKAGE_NAME}/Worlds'
)

WORLD_FILES = {
    'empty':  os.path.join(WORLDS_DIR, 'Empty_Track.world'),
    'racing': os.path.join(WORLDS_DIR, 'Racing_Track.world'),
    'city':   os.path.join(WORLDS_DIR, 'City_Track.world'),
}

PLANNING_NODES = {
    'empty':  'Planning_Empty_node',
    'racing': 'Planning_Racing_node',
    'city':   'Planning_City_node',
}

LATERAL_NODES = {
    'empty':  'Lateral_Empty_node',
    'racing': 'Lateral_Racing_node',
    'city':   'Lateral_City_node',
}

ENV = {
    'GZ_SIM_RESOURCE_PATH': '/opt/ros/jazzy/share/gz_sim_vendor/models',
    '__NV_PRIME_RENDER_OFFLOAD': '1',
    '__GLX_VENDOR_LIBRARY_NAME': 'nvidia',
}


def launch_setup(context):
    track = LaunchConfiguration('track').perform(context)

    world_path    = WORLD_FILES.get(track,    WORLD_FILES['racing'])
    planning_exec = PLANNING_NODES.get(track, PLANNING_NODES['racing'])
    lateral_exec  = LATERAL_NODES.get(track,  LATERAL_NODES['racing'])

    if not os.path.exists(world_path):
        raise FileNotFoundError(f'World file not found: {world_path}')

    return [
        # 1. Gazebo
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', world_path],
            additional_env=ENV,
            output='screen'
        ),

        # 2. ROS-GZ Bridge
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gz_bridge',
            arguments=[
                '/model/vehicle_blue/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                '/model/vehicle_blue/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            ],
            output='screen'
        ),

        # 3. Localization Node — adds noise + Kalman Filter
        #    Subscribes: /model/vehicle_blue/odometry
        #    Publishes:  /team18/filtered_odom
        Node(
            package=PACKAGE_NAME,
            executable='Localization_node',
            name='localization_node',
            output='screen'
        ),

        # 4. Speed Controller
        #    Subscribes: /team18/filtered_odom, /team18/target_speed
        Node(
            package=PACKAGE_NAME,
            executable='Speed_Controller_node',
            name='speed_controller',
            output='screen'
        ),

        # 5. Lateral Controller (track-specific)
        #    Subscribes: /team18/filtered_odom, /team18/path_x, /team18/path_y
        Node(
            package=PACKAGE_NAME,
            executable=lateral_exec,
            name='lateral_controller',
            output='screen'
        ),

        # 6. Planning Node (track-specific)
        #    Subscribes: /team18/filtered_odom
        #    Publishes:  /team18/target_speed, /team18/path_x, /team18/path_y
        Node(
            package=PACKAGE_NAME,
            executable=planning_exec,
            name='planning_node',
            output='screen'
        ),

        # 7. Cmd Mixer
        #    Subscribes: /team18/speed_cmd, /team18/steer_cmd
        #    Publishes:  /model/vehicle_blue/cmd_vel
        Node(
            package=PACKAGE_NAME,
            executable='Cmd_Mixer_node',
            name='cmd_mixer',
            output='screen'
        ),

        # 8. rqt_graph
        ExecuteProcess(
            cmd=['rqt_graph'],
            output='screen'
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'track',
            default_value='racing',
            description='Track to load: empty | racing | city'
        ),
        OpaqueFunction(function=launch_setup)
    ])
