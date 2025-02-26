#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from multispectral_camera_wrapper.multispectral_camera_wrapper import MultiSpectralCameraWrapper


def main(args=None):
    rclpy.init(args=args)

    ms_wrapper = MultiSpectralCameraWrapper()
    ms_wrapper.initialize()
    
    # setup multithreaded executor
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(ms_wrapper)

    ms_wrapper.get_logger().info("Multispectral camera wrapper node is running...")

    try:
        while rclpy.ok():
           executor.spin()
    except KeyboardInterrupt:
        ms_wrapper.get_logger().warn("Multispectral camera wrapper node interrupted.")
        pass

    ms_wrapper.destroy_node()
    rclpy.shutdown()

    #try:
    #    rclpy.spin(ms_wrapper)
    #except KeyboardInterrupt:
    #    ms_wrapper.get_logger().warn("Multispectral camera wrapper node interrupted.")
    #finally:
    #    # ms_wrapper.destroy_node()
    #    rclpy.shutdown()

if __name__ == '__main__':
    main()

