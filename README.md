# multispectral_camera_wrapper

## Dependencies

This ROS package depends on [PTPy](https://github.com/Parrot-Developers/sequoia-ptpy/tree/master). Follow the instruction and install PTPy

## Build this package

```
# from this directory
catkin build --this
```

## Start multispectral_camera_wrapper node

```
rosrun multispectral_camera_wrapper multispectral_camera_wrapper
```

## Send events

* Start capture
  ```
  rostopic pub /multispectral_camera_wrapper/event_in std_msgs/String e_start_capture
  ```
* Stop capture
  ```
  rostopic pub /multispectral_camera_wrapper/event_in std_msgs/String e_stop_capture
  ```
* Stop session
  ```
  rostopic pub /multispectral_camera_wrapper/event_in std_msgs/String e_stop_session
  ```
  Note: if you sent `e_stop_session`, you need to restart this node