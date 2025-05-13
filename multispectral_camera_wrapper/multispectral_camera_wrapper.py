import os
import re
import time
import copy
from datetime import datetime

import threading
import queue

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time

import ptpy
import cv2
import numpy as np
from cv_bridge import CvBridge

from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import Image
from std_msgs.msg import String, UInt8MultiArray, Header
from uas_msgs.msg import MultispectralImage16
from pathlib import Path


class MultiSpectralCameraWrapper(Node):
    """
    Parrot Sequoia multispectral camera wrapper
    """

    def __init__(self):
        super().__init__("multispectral_camera_wrapper")

        pkg_path = get_package_share_directory("multispectral_camera_wrapper")

        # Get ROS parameters
        self.declare_parameter("storage_id", 65537)
        self.declare_parameter("object_format", 0)
        self.declare_parameter("file_format", "EXIF_JPEG,TIFF,UndefinedImage")
        self.declare_parameter("capture_interval", 1)
        self.declare_parameter("save_files", False)
        self.declare_parameter("save_path", "/tmp")
        self.declare_parameter("mission_id", "default_mission")
        self.declare_parameter("debug", False)

        self._storage_id = self.get_parameter("storage_id").value
        # this is the object format to retrieve from the camera
        self._object_format = self.get_parameter("object_format").value
        # this is the file format to publish or save
        self._file_format_to_publish = self.get_parameter("file_format").value.split(
            ","
        )
        self._capture_interval = self.get_parameter("capture_interval").value
        self._save_files = self.get_parameter("save_files").value
        self._save_path = self.get_parameter("save_path").value
        self._mission_id = str(self.get_parameter("mission_id").value)
        self._debug = self.get_parameter("debug").value

        if self._save_files:
            self.image_path = Path(self._save_path) / str(self._mission_id) / "ms_images"
            self.image_path.mkdir(parents=True, exist_ok=True)


        # Initialize the camera connection here, because if it's stopped and
        # re-initialized during runtime, it will not find the USB
        self.camera_initialized = False
        self.initialize_camera()

        # open capture flag
        self.open_capture_initiated = False

        self.bridge = CvBridge()

        # create image queue and stop event
        self.image_queue = queue.Queue()
        self.handles_queue_to_download = queue.Queue()

        # Initialize ROS communications
        self.initialize_ros_topics()

        # create threading events
        self.stop_retrieve_event = threading.Event()
        self.retrieve_image_thread = None
        self.publish_image_thread = None

        self.get_logger().info(
            "Camera session has been initiated, now send e_start_capture trigger..."
        )

    def __del__(self):
        """
        Destructor. Delete all capture and session
        """
        self.camera.terminate_open_capture(transaction_id=self.capture.TransactionID)
        self.camera.close_session()
        self.stop_retrieve_event.set()
        self.get_logger().warn("All capture and sessions are closed")

    def event_callback(self, data: String) -> None:
        event_out_data = String()
        if data.data == "e_start_capture":
            event_out_data.data = "e_capture_started"

            if not self.camera_initialized:
                self.get_logger().error(
                    "Camera is not initialized, please restart the node."
                )
                return

            self.get_logger().info("Starting open capture...")
            # Initiate open capture
            if not self.open_capture_initiated:
                self.initiate_open_capture(
                    storage_id=self._storage_id, object_format=self._object_format
                )
                self.get_logger().info("Open capture has been initiated...")

            if (
                self.retrieve_image_thread is None
                or not self.retrieve_image_thread.is_alive()
            ):
                self.stop_retrieve_event.clear()
                self.retrieve_image_thread = threading.Thread(
                    target=self.retrieve_images, daemon=False
                )
                self.retrieve_image_thread.start()
                self.get_logger().info("Retrieve image thread started...")

            if (
                self.publish_image_thread is None
                or not self.publish_image_thread.is_alive()
            ):
                self.publish_image_thread = threading.Thread(
                    target=self.publish_image, daemon=False
                )
                self.publish_image_thread.start()
                self.get_logger().info("Publish image thread started...")

        elif data.data == "e_stop_capture":
            event_out_data.data = "e_capture_stopped"

            if not self.camera_initialized:
                self.get_logger().error(
                    "Camera is not initialized, please restart the node."
                )
                return

            self.get_logger().info("Stopping open capture...")
            # disable open capture flag
            self.open_capture_initiated = False

            # stop threading
            self.stop_retrieve_event.set()
            self.stop_open_capture()

            if self._save_files:
                self._download_at_mission_ends()

        elif data.data == "e_start_camera":
            # initialize camera
            if not self.camera_initialized:
                self.initialize_camera()

            event_out_data.data = "e_session_started"

        elif data.data == "e_stop_camera":
            if not self.camera_initialized:
                self.get_logger().error(
                    "Camera is not initialized, please restart the node."
                )
                return

            # stop threading
            self.stop_retrieve_event.set()
            self.get_logger().info("Closing camera session...")
            self.close_open_session()
            self.get_logger().info("Camera session has been closed...")
            self.get_logger().warn(
                "Session has been closed, restart the node to start a new session...!"
            )

            event_out_data.data = "e_session_stopped"

        else:
            event_out_data.data = "unknown trigger"

        self._pub_event.publish(event_out_data)

    def initialize_camera(self):
        """
        Initialize the camera
        """
        self.get_logger().info("Initializing camera....")
        # catch the exception if the camera is not connected
        try:
            self.camera = ptpy.PTPy()
            # Create Open PTP session
            response = self.camera.open_session()
            self.get_logger().info(
                f"Camera initiated with session ID {response.SessionID}, transaction ID {response.TransactionID}"
            )

            storage_ids = self.camera.get_storage_ids()
            for sids in storage_ids:
                self.get_logger().info(f"Storage ID: {sids}")

            self.camera_initialized = True

        except Exception as e:
            self.get_logger().error(f"Error: {e}")
            self.get_logger().error(
                "Camera is probably not connected, "
                "please connect the camera and restart the node."
            )

    def initialize_ros_topics(self) -> None:
        """
        Initialize topics
        """
        # string event topic for event in and event out
        self._sub_event = self.create_subscription(
            String, "~/event_in", self.event_callback, 10
        )
        self._pub_event = self.create_publisher(String, "~/event_out", 10)

        # Publishers
        self._pub_nir = self.create_publisher(
            MultispectralImage16, "~/image/nir_raw", 10
        )
        self._pub_reg = self.create_publisher(
            MultispectralImage16, "~/image/reg_raw", 10
        )
        self._pub_red = self.create_publisher(
            MultispectralImage16, "~/image/red_raw", 10
        )
        self._pub_gre = self.create_publisher(
            MultispectralImage16, "~/image/gre_raw", 10
        )
        self._pub_rgb = self.create_publisher(Image, "~/image/rgb_raw", 10)
        self._pub_sequoia_diag = self.create_publisher(
            UInt8MultiArray, "~/metadata/diag", 10
        )
        self._pub_sequoia_param = self.create_publisher(
            UInt8MultiArray, "~/metadata/param", 10
        )

    def get_handles(self, storage_id=65537, object_format=0):
        """
        Get all object handles (directory for each capture)
        Defautl storage id = 65537 (internal), memory card = 131073
        Args
            storage_id: id of the storage e.g. internal or memory card
            object_format: 0 for all formats or specify the format TIFF,
                EXIF_JPEG, UndefinedImage
        """
        handles = self.camera.get_object_handles(
            storage_id=storage_id,
            object_format=object_format,
            # object_handle=0,
            all_storage_ids=False if storage_id is not None else True,
            all_formats=True if object_format == 0 else False,
            in_root=False,
        )
        return handles

    def get_directory_info(self):
        """
        Get a list directory from the camera
        """
        handles = self.get_handles()
        dir_info = {}
        current_dir_name = None
        for i, handle in enumerate(handles):
            info = self.camera.get_object_info(handle)
            if info.ObjectFormat == "Association":
                dir_info[info.Filename] = {}
                dir_info[info.Filename]["file_count"] = 0
                dir_info[info.Filename]["start_handle_idx"] = i + 1
                current_dir_name = info.Filename
            else:
                if current_dir_name:
                    dir_info[current_dir_name]["file_count"] += 1
                    dir_info[current_dir_name]["end_handle_idx"] = i
                    dir_info[current_dir_name]["parent_object"] = info.ParentObject
        return dir_info

    def initiate_open_capture(self, storage_id=65537, object_format=0):
        """
        Initiate open capture or timelapse capture
        """
        # get initial directory info before initiating open capture
        self.get_logger().info("Getting directory statistics...")
        self.dir_info_init = self.get_directory_info()
        self.get_logger().info("Directory statistic info retrieved...")

        ## Initiate timelapse capture
        self.capture = self.camera.initiate_open_capture(
            storage_id=storage_id, object_format=object_format
        )
        # Wait for capture event
        event = self.camera.event(wait=False)
        self.open_capture_initiated = True
        self.get_logger().info("Open capture has been initiated...")

        self.get_logger().info("Waiting for a new directory to be initiated...")
        self.get_new_directory_info()
        self.get_logger().info("Directory info has been initiated...")

    def stop_open_capture(self):
        # Close the open capture session
        self.camera.terminate_open_capture(transaction_id=self.capture.TransactionID)
        self.get_logger().info("Open capture has been stopped...")

    def close_open_session(self):
        # Close the PTP session
        self.camera.close_session()
        self.get_logger().info("Open camera session has been closed...")
        self.camera_initialized = False

    def delete_all_files(self):
        """
        Delete all files in the camera.
        Any session should be closed before deleting all files.
        """
        self.camera.delete_object(
            handle=0, object_format=0, delete_all=True, delete_all_images=True
        )
        self.get_logger().info("All files have been deleted...")

    def get_new_directory_info(self):
        """
        When initiate_open_capture is created, it should create a new direcotry.
        This function continuously check if a new dir has been created
        and wait until it's created
        """
        new_dir_available = False
        new_directory = None
        dir_info_new = self.get_directory_info()

        while not new_dir_available and not self.stop_retrieve_event.is_set():
            # break if stop event is detected
            if self.stop_retrieve_event.is_set():
                self.get_logger().info("Stop event detected, exiting directory check.")
                break

            ## Check if new direcory has been created
            if len(list(dir_info_new.keys())) > len(list(self.dir_info_init.keys())):
                new_dir_available = True
                new_directory = list(dir_info_new.keys())[-1]
                self.get_logger().info(
                    f"New directory has been created: {list(self.dir_info_init.keys())[-1]}"
                )

            self.get_logger().debug(f"No new dir is initiated: {new_dir_available}")
            dir_info_new = self.get_directory_info()
            time.sleep(0.1)

        self.latest_directory = new_directory

    def stop(self):
        """
        Cleanup threads on shutdown or e_stop_capture has been triggered
        """
        self.stop_retrieve_event.set()
        if self.retrieve_image_thread:
            self.retrieve_image_thread.join()
        if self.publish_image_thread:
            self.publish_image_thread.join()
        self.get_logger().info("Node shutdown complete.")

    def convert_ptp_timestamp_to_ros(self, filename, header):
        """
        Convert image time stamp from Filename to ROS timestamp
        """
        # Regex to extract date and time (matches 240930_164044)
        match = re.search(r"(\d{6}_\d{6})", filename)

        if match:
            # Extracted date-time string
            datetime_str = match.group(1)

            # Parse the date and time using the format YYMMDD_HHMMSS
            dt = datetime.strptime(datetime_str, "%y%m%d_%H%M%S")

            # Convert to ROS timestamp (seconds since epoch)
            secs = int(time.mktime(dt.timetuple()))
            nsecs = 0
            header.stamp = Time()
            header.stamp.sec = secs
            header.stamp.nanosec = nsecs

            self.get_logger().debug(
                f"Extracted ROS Time: {header.stamp.sec}.{header.stamp.nanosec} seconds since epoch"
            )
        else:
            self.get_logger().debug("No valid timestamp found in filename.")

    def retrieve_images(self):
        """
        Retrive images from the camera and stro in a queue
        """
        # get latest dir info
        dir_info_new = self.get_directory_info()

        # now start taking the image with the 2nd index of a new dir
        # +1 because the first index is the dir itself
        start_idx = dir_info_new[self.latest_directory]["start_handle_idx"] + 1

        # keep track of start_idx to be used later when retrieving
        # sequoia_diag.dat and sequoia_params.dat
        dir_start_idx = copy.deepcopy(start_idx)

        image_count = 0

        prev_handles_count = 0
        while not self.stop_retrieve_event.is_set():
            all_handles = self.get_handles()
            handles = all_handles[start_idx:]

            if len(handles) > 0:
                for i, handle in enumerate(handles):
                    info = self.camera.get_object_info(handle)
                    if info is None:
                        continue

                    if info.ObjectFormat in self._file_format_to_publish:
                        obj = self.camera.get_object(handle)

                        # determine buffer size
                        # buffer_size = len(obj.Data)
                        # if buffer_size % 1 == 0 and buffer_size % 2 != 0:
                        #     data_type = np.uint8
                        # elif buffer_size % 2 == 0:
                        #     data_type = np.uint16
                        # self.get_logger().info(f"Buffer size: {buffer_size}, Data type: {data_type}, Filename: {info.Filename}")

                        if not obj.Data:
                            self.get_logger().warn("No image data received.")
                            continue

                        if "TIF" in info.Filename:
                            data_type = np.uint16
                        # ToDo: JPG sometimes has uint16, but most of the time uint8
                        elif "JPG" in info.Filename:
                            data_type = np.uint8
                            if len(obj.Data) % 2 == 0:
                                data_type = np.uint16
                        else:
                            data_type = np.uint8

                        # convert and decode image to np array and cv
                        obj_array = np.frombuffer(obj.Data, data_type)

                        if obj_array.size == 0:
                            self.get_logger().warn(
                                "Received empty buffer from image data."
                            )
                            continue
                        self.image_queue.put((info.Filename, obj_array))
                        self.handles_queue_to_download.put(handle)


                start_idx += len(handles)
            else:
                self.get_logger().info(
                    f"No new handles to process: {str(len(all_handles))}, {str(len(handles))}, {str(start_idx)}"
                )

            # delay < 0.0 is too short and may cause the file is not yet fully written
            time.sleep(self._capture_interval)

    def _download_at_mission_ends(self):
        while not self.handles_queue_to_download.empty():
            # Get the next image from the queue
            handle = self.handles_queue_to_download.get(timeout=1)
            info = self.camera.get_object_info(handle)
            if info is None:
                continue

            if info.ObjectFormat in self._file_format_to_publish:
                obj = self.camera.get_object(handle)


                if not obj.Data:
                    self.get_logger().warn("No image data received.")
                    continue

                if "TIF" in info.Filename:
                    data_type = np.uint16
                # ToDo: JPG sometimes has uint16, but most of the time uint8
                elif "JPG" in info.Filename:
                    data_type = np.uint8
                    if len(obj.Data) % 2 == 0:
                        data_type = np.uint16
                else:
                    data_type = np.uint8

                # convert and decode image to np array and cv
                obj_array = np.frombuffer(obj.Data, data_type)

                if obj_array.size == 0:
                    self.get_logger().warn(
                        "Received empty buffer from image data."
                    )
                    continue
                file_path: Path = self.image_path / str(info.Filename)
                with open(str(file_path), mode="wb") as f:
                    f.write(obj_array)
        self.get_logger().info("Download completed...")


    def publish_image(self, delete_retrieved_file=False):
        """
        Publish images stored in image_queue to ROS topics
        """
        # get latest dir info
        dir_info_new = self.get_directory_info()

        # now start taking the image with the 2nd index of a new dir
        # +1 because the first index is the dir itself
        start_idx = dir_info_new[self.latest_directory]["start_handle_idx"] + 1

        # keep track of start_idx to be used later when retrieving
        # sequoia_diag.dat and sequoia_params.dat
        dir_start_idx = copy.deepcopy(start_idx)
        image_count = 0
        prev_handles_count = 0
        image_group = {}
        self.get_logger().info("Publishing multispectral data...")

        while not self.stop_retrieve_event.is_set():
            if not self.image_queue.empty():
                # Get the next image from the queue
                filename, obj_array = self.image_queue.get(timeout=1)
                if obj_array is None:
                    continue

                # decode image, and keep the image as is using arg IMREAD_UNCHANGED
                decoded_image = cv2.imdecode(obj_array, cv2.IMREAD_UNCHANGED)
                if decoded_image is None:
                    self.get_logger().warn(f"Failed to decode image {str(filename)}")
                    continue

                msg_header = Header()
                spectrum = None
                if "RGB" in filename:
                    img_msg = self.bridge.cv2_to_imgmsg(decoded_image, encoding="bgr8")
                    self.convert_ptp_timestamp_to_ros(filename, msg_header)
                    self._pub_rgb.publish(img_msg)
                    spectrum = "RGB"
                else:
                    # create MultispectralImage16 msg
                    img_msg = MultispectralImage16()
                    img_msg.height = decoded_image.shape[0]
                    img_msg.width = decoded_image.shape[1]
                    img_msg.channel = 1

                    # img_msg.data = decoded_image.flatten().tolist()
                    img_msg.data = obj_array.flatten().tolist()
                    self.convert_ptp_timestamp_to_ros(filename, msg_header)

                    if "NIR" in filename:
                        self._pub_nir.publish(img_msg)
                        spectrum = "NIR"
                    elif "REG" in filename:
                        self._pub_reg.publish(img_msg)
                        spectrum = "REG"
                    elif "RED" in filename:
                        self._pub_red.publish(img_msg)
                        spectrum = "RED"
                    elif "GRE" in filename:
                        self._pub_gre.publish(img_msg)
                        spectrum = "GRE"

                # save files
                # if self._save_files:
                #     self.get_logger().info(f"Saving {self.image_path}/{filename}")
                #     with open(f"{self.image_path}/{filename}", mode="wb") as f:
                #         f.write(obj_array)

            else:
                self.get_logger().debug("Queue is empty...")

            # sleep for a while
            time.sleep(0.1)

        self.get_logger().warn("Camera capture has been terminated...")
        # retrieve other files such as sequoia_diag.dat and sequoia_params.data
        self.get_logger().info(
            "Retrieving squoia_diag.dat and sequoia_params.dat files..."
        )
        all_handles = self.get_handles()
        handles = all_handles[dir_start_idx:]
        for i, handle in enumerate(handles):
            info = self.camera.get_object_info(handle)
            if info is None:
                continue

            if ".dat" in info.Filename:
                self.get_logger().info(f"Metadata: {info.Filename}")
                obj = self.camera.get_object(handle)

                metadata_msg = UInt8MultiArray()
                metadata_msg.data = list(obj.Data)

                if "diag" in info.Filename:
                    self._pub_sequoia_diag.publish(metadata_msg)
                elif "param" in info.Filename:
                    self._pub_sequoia_param.publish(metadata_msg)

                self.get_logger().debug(f"Saving image {info.Filename}")
                if self._save_files:
                    self.get_logger().info(f"Saving {self.image_path}/{info.Filename} ")
                    with open(f"{self.image_path}/{info.Filename}", mode="wb") as f:
                        f.write(obj.Data)

        self.get_logger().info("Capture has been stopped...")
        # delete all files
        # if delete_retrieved_file:
        #    self.camera.delete_object(handle=handle)
