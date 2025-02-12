# A ROS2 Wrapper for PTPy Library for the Parrot Multispectral Camera

## Dependencies

This ROS2 package depends on [PTPy](https://github.com/Parrot-Developers/sequoia-ptpy/tree/master). Follow the instruction and install PTPy.

* Clone and install ptpy from garrulus fork
  ```
  git clone https://github.com/garrulus-project/sequoia-ptpy.git
  cd sequoia-ptpy
  pip install . 
  ```
  > **Note:** The `sequoia-ptpy` fork supports Python 3.10  
* Connect the camera and you should be able to run some examples provided in the PTPy repo, for example:
  ```
  # from the root directory of PTPy repo
  python3 examples/list_all_properties.py
  ```

* Connect to `Sequoia_xxx` wifi (xxx should be some number) and open `http://192.168.47.1/`. From there, you can also interact with the camera e.g. see captured images

* Change the capture mode under `Home` page to `Timelapse` mode using the web interface. **This is required to enable timelapse mode**.
  
  ToDo: use PTPy to configure Timelapse mode

## Build this package

```
# ros2 workspace dir
colcon build --symlink-install
```

## Start multispectral_camera_wrapper node

Before starting this node, **make sure all images are removed from the storage** to avoid PTP communication interuption due to large number of images in the camera storage. This can be done by connecting to the Sequoia wifi, go to Gallery and `Delete all`.

```
ros2 run multispectral_camera_wrapper multispectral_camera_wrapper
```

## Send events

* Start capture
  ```
  ros2 topic pub /multispectral_camera_wrapper/event_in std_msgs/String e_start_capture
  ```
* Stop capture
  ```
  ros2 topic pub /multispectral_camera_wrapper/event_in std_msgs/String e_stop_capture
  ```
* Stop session
  ```
  ros2 topic pub /multispectral_camera_wrapper/event_in std_msgs/String e_stop_session
  ```
  Note: if you sent `e_stop_session`, you need to restart this node

## Outputs
```
# event_out output e.g., e_capture_started
/multispectral_camera_wrapper/event_out

# rgb image
/multispectral_camera_wrapper/image/rgb

# near-ir image
/multispectral_camera_wrapper/image/nir

# red-edge spectrum
/multispectral_camera_wrapper/image/reg

# red spectrum
/multispectral_camera_wrapper/image/red

# green spectrum
/multispectral_camera_wrapper/image/gre

# metadata published once the e_stop_capture event is received
/multispectral_camera_wrapper/metadata/diag
/multispectral_camera_wrapper/metadata/param

```
