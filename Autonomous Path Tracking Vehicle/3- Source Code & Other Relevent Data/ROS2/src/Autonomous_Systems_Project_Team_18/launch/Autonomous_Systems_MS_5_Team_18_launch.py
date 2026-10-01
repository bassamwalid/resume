from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    ExecuteProcess,
    DeclareLaunchArgument,
    OpaqueFunction,
)
from launch.substitutions import LaunchConfiguration
import os

PACKAGE_NAME = 'Autonomous_Systems_Project_Team_18'

WORLDS_DIR = os.path.expanduser(
    f'~/ros2_ws/src/{PACKAGE_NAME}/Worlds'
)

WORLD_FILES = {
    'racing': os.path.join(WORLDS_DIR, 'Racing_Track.world'),
    'city':   os.path.join(WORLDS_DIR, 'City_Track.world'),
}

PLANNING_NODES = {
    'racing': 'Planning_Racing_node',
    'city':   'Planning_City_node',
}

LATERAL_NODES = {
    'racing': 'Lateral_Racing_node',
    'city':   'Lateral_City_node',
}

ENV = {
    'GZ_SIM_RESOURCE_PATH': '/opt/ros/jazzy/share/gz_sim_vendor/models',
    '__NV_PRIME_RENDER_OFFLOAD': '1',
    '__GLX_VENDOR_LIBRARY_NAME': 'nvidia',
}


def launch_setup(context):
    # ── Resolve track argument ────────────────────────────────────
    track = LaunchConfiguration('track').perform(context)

    world_path    = WORLD_FILES.get(track,    WORLD_FILES['racing'])
    planning_exec = PLANNING_NODES.get(track, PLANNING_NODES['racing'])
    lateral_exec  = LATERAL_NODES.get(track,  LATERAL_NODES['racing'])

    if not os.path.exists(world_path):
        raise FileNotFoundError(f'World file not found: {world_path}')

    # ── Read rosparam values ──────────────────────────────────────
    # Vehicle initial state
    init_x     = float(LaunchConfiguration('init_x').perform(context))
    init_y     = float(LaunchConfiguration('init_y').perform(context))
    init_speed = float(LaunchConfiguration('init_speed').perform(context))

    # Sensor noise standard deviations
    sigma_pos   = float(LaunchConfiguration('sigma_pos').perform(context))
    sigma_yaw   = float(LaunchConfiguration('sigma_yaw').perform(context))
    sigma_speed = float(LaunchConfiguration('sigma_speed').perform(context))

    # Kalman Filter process noise
    q_pos   = float(LaunchConfiguration('q_pos').perform(context))
    q_yaw   = float(LaunchConfiguration('q_yaw').perform(context))
    q_speed = float(LaunchConfiguration('q_speed').perform(context))

    # ── Shared params dict passed to localization node ────────────
    localization_params = [{
        'sigma_pos':   sigma_pos,
        'sigma_yaw':   sigma_yaw,
        'sigma_speed': sigma_speed,
        'q_pos':       q_pos,
        'q_yaw':       q_yaw,
        'q_speed':     q_speed,
    }]

    return [
        # 1. Gazebo Simulation
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', world_path],
            additional_env=ENV,
            output='screen'
        ),

        # 2. ROS-GZ Bridge
        #    Bridges cmd_vel and odometry between ROS 2 and Gazebo
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gz_bridge',
            arguments=[
                '/model/vehicle_blue/cmd_vel'
                '@geometry_msgs/msg/Twist@gz.msgs.Twist',
                '/model/vehicle_blue/odometry'
                '@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            ],
            output='screen'
        ),

        # 3. Localization Node (noise injection + Kalman Filter)
        #    Subscribes : /model/vehicle_blue/odometry  (ideal Gazebo states)
        #    Publishes  : /team18/filtered_odom
        #                 /team18/noisy_odom
        #    rosparam   : sigma_pos, sigma_yaw, sigma_speed, q_pos, q_yaw, q_speed
        Node(
            package=PACKAGE_NAME,
            executable='Localization_node',
            name='localization_node',
            parameters=localization_params,
            output='screen'
        ),

        # 4. Speed Controller
        #    Subscribes : /team18/filtered_odom, /team18/target_speed
        #    Publishes  : /team18/speed_cmd
        Node(
            package=PACKAGE_NAME,
            executable='Speed_Controller_node',
            name='speed_controller',
            output='screen'
        ),

        # 5. Lateral Controller (track-specific)
        #    Subscribes : /team18/filtered_odom, /team18/path_x, /team18/path_y
        #    Publishes  : /team18/steer_cmd
        Node(
            package=PACKAGE_NAME,
            executable=lateral_exec,
            name='lateral_controller',
            output='screen'
        ),

        # 6. Planning Node (track-specific)
        #    Subscribes : /team18/filtered_odom
        #    Publishes  : /team18/target_speed, /team18/path_x, /team18/path_y
        Node(
            package=PACKAGE_NAME,
            executable=planning_exec,
            name='planning_node',
            output='screen'
        ),

        # 7. Cmd Mixer
        #    Subscribes : /team18/speed_cmd, /team18/steer_cmd
        #    Publishes  : /model/vehicle_blue/cmd_vel
        Node(
            package=PACKAGE_NAME,
            executable='Cmd_Mixer_node',
            name='cmd_mixer',
            output='screen'
        ),

        # 8. rqt_graph  (visualise node & topic connections)
        ExecuteProcess(
            cmd=['rqt_graph'],
            output='screen'
        ),
    ]


def generate_launch_description():
    return LaunchDescription([

        # ── Track selection ───────────────────────────────────────
        DeclareLaunchArgument(
            'track',
            default_value='racing',
            description='Track to load: racing | city'
        ),

        # ── Vehicle initial state ─────────────────────────────────
        DeclareLaunchArgument(
            'init_x',
            default_value='0.0',
            description='Vehicle initial x position [m]'
        ),
        DeclareLaunchArgument(
            'init_y',
            default_value='0.0',
            description='Vehicle initial y position [m]'
        ),
        DeclareLaunchArgument(
            'init_speed',
            default_value='0.0',
            description='Vehicle initial speed [m/s]'
        ),

        # ── Sensor noise standard deviations ─────────────────────
        DeclareLaunchArgument(
            'sigma_pos',
            default_value='0.02',
            description='Position measurement noise std dev [m]'
        ),
        DeclareLaunchArgument(
            'sigma_yaw',
            default_value='0.01',
            description='Heading measurement noise std dev [rad]'
        ),
        DeclareLaunchArgument(
            'sigma_speed',
            default_value='0.05',
            description='Speed measurement noise std dev [m/s]'
        ),

        # ── Kalman Filter process noise ───────────────────────────
        DeclareLaunchArgument(
            'q_pos',
            default_value='0.001',
            description='Process noise for x, y states'
        ),
        DeclareLaunchArgument(
            'q_yaw',
            default_value='0.001',
            description='Process noise for heading state'
        ),
        DeclareLaunchArgument(
            'q_speed',
            default_value='0.005',
            description='Process noise for speed state'
        ),

        OpaqueFunction(function=launch_setup)
    ])
