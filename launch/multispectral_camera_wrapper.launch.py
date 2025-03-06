import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            # Defautlt storage id = 65537 (internal), memory card = 131073
            DeclareLaunchArgument(
                "storage_id", default_value="65537", description="Storage ID"
            ),
            DeclareLaunchArgument(
                "save_files", default_value="false", description="Save files"
            ),
            DeclareLaunchArgument(
                "capture_interval", default_value="1", description="Capture interval"
            ),
            # Default file format = EXIF_JPEG, TIFF, UndefinedImage
            DeclareLaunchArgument(
                "file_format",
                default_value="EXIF_JPEG,TIFF,UndefinedImage",
                description="File format to publish or save",
            ),
            DeclareLaunchArgument(
                "debug", default_value="true", description="Debug mode"
            ),
            # Define the node
            Node(
                package="multispectral_camera_wrapper",
                executable="multispectral_camera_wrapper_node",
                name="multispectral_camera_wrapper_node",
                output="screen",
                parameters=[
                    {
                        "storage_id": LaunchConfiguration("storage_id"),
                        "save_files": LaunchConfiguration("save_files"),
                        "capture_interval": LaunchConfiguration("capture_interval"),
                        "file_format": LaunchConfiguration("file_format"),
                        "debug": LaunchConfiguration("debug"),
                    }
                ],
            ),
        ]
    )
