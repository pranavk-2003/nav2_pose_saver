from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nav2_pose_saver',
            executable='pose_saver_node',
            name='pose_saver',
            output='screen',
            parameters=[
                {'save_interval_sec': 5.0},
                {'pose_file_path': '/tmp/pose_saver.yaml'}
            ]
        )
    ])
