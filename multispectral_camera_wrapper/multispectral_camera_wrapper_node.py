#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from multispectral_camera_wrapper.multispectral_camera_wrapper import (
    MultiSpectralCameraWrapper,
)


def main(args=None):
    rclpy.init(args=args)

    # Stick with one thread for now because the camera driver is not thread-safe
    ms_wrapper = MultiSpectralCameraWrapper()
    ms_wrapper.get_logger().info("Multispectral camera wrapper node is running...")

    try:
        rclpy.spin(ms_wrapper)
    except KeyboardInterrupt:
        ms_wrapper.get_logger().warn("Multispectral camera wrapper node interrupted.")
    finally:
        ms_wrapper.stop()
        ms_wrapper.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
