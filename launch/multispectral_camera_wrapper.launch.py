import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    return LaunchDescription(
        [
            # Defautlt storage id = 65537 (internal), memory card = 131073
            DeclareLaunchArgument(
                "storage_id", default_value="65537", description="Storage ID"
            ),
            DeclareLaunchArgument(
                "save_files", default_value="true", description="Save files"
            ),
            DeclareLaunchArgument(
                "save_path", default_value="/tmp", description="Save path for images"
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
                "mission_id", default_value="default_mission", description="Mission ID"
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
                        "save_path": LaunchConfiguration("save_path"),
                        "capture_interval": LaunchConfiguration("capture_interval"),
                        "file_format": LaunchConfiguration("file_format"),
                        "debug": LaunchConfiguration("debug"),
                        "mission_id": LaunchConfiguration("mission_id"),
                    }
                ],
            ),
        ]
    )
