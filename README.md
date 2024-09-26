# A ROS Wrapper for PTPy Library for the Parrot Multispectral Camera

## Dependencies

This ROS package depends on [PTPy](https://github.com/Parrot-Developers/sequoia-ptpy/tree/master). Follow the instruction and install PTPy.

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