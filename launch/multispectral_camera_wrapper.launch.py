import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument('storage_id', default_value='65537', description='Storage ID'),
        DeclareLaunchArgument('save_files', default_value='false', description='Save files'),
        DeclareLaunchArgument('capture_interval', default_value='1', description='Capture interval'),
        DeclareLaunchArgument('publish_group_images', default_value='false', description='Publish group images'),

        # Define the node
        Node(
            package='multispectral_camera_wrapper',
            executable='multispectral_camera_wrapper_node',
            name='multispectral_camera_wrapper_node',
            output='screen',
            parameters=[{
                'storage_id': launch.substitutions.LaunchConfiguration('storage_id'),
                'save_files': launch.substitutions.LaunchConfiguration('save_files'),
                'capture_interval': launch.substitutions.LaunchConfiguration('capture_interval'),
                'publish_group_images': launch.substitutions.LaunchConfiguration('publish_group_images'),
            }]
        )
    ])

