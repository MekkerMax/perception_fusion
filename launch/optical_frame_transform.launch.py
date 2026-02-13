import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    optical_args = ['--x', '0', '--y', '0', '--z', '0', 
                    '--roll', '-1.5708', '--pitch', '0', '--yaw', '-1.5708']


    pkg_share = get_package_share_directory('perception_fusion')
    rviz_config_file = os.path.join(pkg_share, 'launch', 'perception.rviz')

    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='front_optical_tf',
            arguments=optical_args + ['--frame-id', 'ZED_X_FRONT', '--child-frame-id', 'ZED_X_FRONT_optical']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='right_optical_tf',
            arguments=optical_args + ['--frame-id', 'ZED_X_RIGHT', '--child-frame-id', 'ZED_X_RIGHT_optical']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='left_optical_tf',
            arguments=optical_args + ['--frame-id', 'ZED_X_LEFT', '--child-frame-id', 'ZED_X_LEFT_optical']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='rear_optical_tf',
            arguments=optical_args + ['--frame-id', 'ZED_X_BACK', '--child-frame-id', 'ZED_X_BACK_optical']
        ),

        Node(
            package='perception_fusion',
            executable='fusion_node',
            name='fusion_node',
            output='screen',

            # parameters=[{'confidence_threshold': 0.5}]
        ),

        # --- RVIZ2 ---
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen'
        )
    ])