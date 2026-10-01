from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        Node(
            package='automilestoneT18',
            executable='localization_team18',
            name='Localization_Team18',
            output='screen'
        ),

        Node(
            package='automilestoneT18',
            executable='planner_controller_team18',
            name='Planner_Controller_Team18',
            output='screen'
        ),

    ])
