import os
import re
import time
import copy
from datetime import datetime

import threading
import queue

import rospy
import ptpy

import cv2
import numpy as np
from cv_bridge import CvBridge

from rospkg import RosPack
from sensor_msgs.msg import Image
from std_msgs.msg import String, Byte, ByteMultiArray, UInt8MultiArray
from uas_msgs.msg import MultispectralImages


class MultiSpectralCameraWrapper(object):
    _pub_ms_data = None
    _pub_debug_img = None


    def __init__(self):
        pkg_path = RosPack().get_path("multispectral_camera_wrapper")

        self._config_file = rospy.get_param("~config_path", None)
        #if not os.path.isfile(self._config_file):
        #    rospy.logwarn("Config file is not given or does not exist")

        #self._config = file_utils.parse_yaml_config(self._config_file)

        # get ros params
        self._storage_id = rospy.get_param("~storage_id", 65537)
        self._object_format = rospy.get_param("~object_format", 0)
        self._file_format_to_save = rospy.get_param("~file_format", ["EXIF_JPEG", "TIFF", "UndefinedImage"])
        self._capture_interval = rospy.get_param("~capture_interval", 1.0)
        self._save_files = rospy.get_param("~save_files", False)
        self._data_type = rospy.get_param("~data_type", np.uint8)
        # parameter for publishing all images into a single topic
        self._publish_group_images = rospy.get_param("~_publish_group_images", False)

        if self._save_files:
            self.image_path = os.path.join(pkg_path, "images")
            os.makedirs(self.image_path, exist_ok=True)
        
        self._debug = rospy.get_param("~debug", False)
        
        # flag to continuously update object info retrived from get_object_handles
        self.update_object_info = True
        
        # if enabled, it will continuously retrieve, save or publish new images to a ros topic
        self.publish_image_flag = True
         
        # Initialize the camera connection here, because if it's stopped and
        # re-initialize during runtime, it will not find the USB
        self.camera = ptpy.PTPy()

        # Create Open PTP session
        response = self.camera.open_session()
        rospy.loginfo(f"Camera initiated with session id {response.SessionID}, transaction id {response.TransactionID}, response {response.ResponseCode}")

        storage_ids = self.camera.get_storage_ids()
        for sids in storage_ids:
            rospy.loginfo("Storage id: %s", sids)

        self.bridge = CvBridge()
   
        # start multithreading for image retrieval
        self.init_image_retrieve_threading()

        rospy.loginfo("Camera session has been initiated, now send e_start_capture trigger...")

    def __del__(self):
        """
        Destructor. Delete all capture and session
        """
        self.camera.terminate_open_capture(transaction_id=self.capture.TransactionID)
        self.camera.close_session()
        self.stop_retrieve_event.set()
        rospy.logwarn("All capture and sessions are closed")

    def event_callback(self, data: String) -> None:
        event_out_data = String()
        if data.data == "e_start_capture":
            # set publish image flag to True
            self.publish_image_flag = True
            
            ## Initiate timelapse capture
            self.initiate_open_capture(storage_id=self._storage_id, object_format=self._object_format)
            rospy.loginfo("Open capture has been initiated...")
            
            # Sent event out without waiting image retrieval to be done
            event_out_data.data = "e_capture_started"
            self._pub_event.publish(event_out_data)

            rospy.loginfo("Retrieving image...")
            #self.retrieve_image(delete_retrieved_file=False)
            #self.start_image_retrieve_threading()
            if not self.retrieve_image_thread.is_alive():
                self.retrieve_image_thread.start()

            # start publishing image
            self.publish_image()

        elif data.data == "e_stop_capture":
            # set saving image state to False to exit while-loop in image saving func
            self.publish_image_flag = False
            
            # stop threading
            self.stop_retrieve_event.set()
            
            rospy.loginfo("Stopping open capture...")
            self.stop_open_capture()
            rospy.loginfo("Open capture has been stopped...")
            
            event_out_data.data = "e_capture_stopped"
            self._pub_event.publish(event_out_data)
        
        elif data.data == "e_start_session":
            # Initialize the camera connection
            self.camera = ptpy.PTPy()
            # Create Open PTP session
            response = self.camera.open_session()
            event_out_data.data = "e_session_started"
            self._pub_event.publish(event_out_data)

        elif data.data == "e_stop_session":
            # stop threading
            self.stop_retrieve_event.set()
            rospy.loginfo("Closing camera session...")
            self.close_open_session()
            rospy.loginfo("Camera session has been closed...")
            rospy.logwarn("Session has been closed, restart the node to start a new session...!")
             
            event_out_data.data = "e_session_stopped"
            self._pub_event.publish(event_out_data)

    def init_image_retrieve_threading(self):
        """
        Start multithreading for image retrieval from the camera
        """
        # create image queue and stop event
        self.image_queue = queue.Queue()
        self.stop_retrieve_event = threading.Event()
        # Create threads
        self.retrieve_image_thread = threading.Thread(target=self.retrieve_images)

        # Start threads
        #self.retrieve_image_thread.start()

    def initialize(self) -> None:
        #start event in and out
        self._sub_event = rospy.Subscriber("~event_in", String, self.event_callback)
        self._pub_event = rospy.Publisher("~event_out", String, queue_size=1)

        # publishers
        self._pub_nir = rospy.Publisher("~image/nir", Image, queue_size=1)
        self._pub_rgb = rospy.Publisher("~image/rgb", Image, queue_size=1)
        self._pub_reg = rospy.Publisher("~image/reg", Image, queue_size=1)
        self._pub_red = rospy.Publisher("~image/red", Image, queue_size=1)
        self._pub_gre = rospy.Publisher("~image/gre", Image, queue_size=1)
        self._pub_ms_images = rospy.Publisher("~image/images", MultispectralImages, queue_size=1)
        self._pub_sequoia_diag = rospy.Publisher("~metadata/diag", UInt8MultiArray, queue_size=1)
        self._pub_sequoia_param = rospy.Publisher("~metadata/param", UInt8MultiArray, queue_size=1)

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
            all_formats=True if object_format==0 else False,
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
        for i,handle in enumerate(handles):
            info = self.camera.get_object_info(handle)
            if info.ObjectFormat == "Association":
                dir_info[info.Filename] = {}
                dir_info[info.Filename]['file_count'] = 0
                dir_info[info.Filename]['start_handle_idx'] = i+1
                current_dir_name = info.Filename
            else:
                if current_dir_name:
                    dir_info[current_dir_name]['file_count'] += 1
                    dir_info[current_dir_name]['end_handle_idx'] = i
                    dir_info[current_dir_name]['parent_object'] = info.ParentObject
        return dir_info
    
    def initiate_open_capture(self, storage_id=65537, object_format=0):
        """
        Initiate open capture or timelapse capture
        ToDo: we can filter object format to reduce computation
        on the camera device
        """
        # get initial directory info before initiating open capture
        rospy.loginfo("Getting directory statistics...")
        self.dir_info_init = self.get_directory_info()
        rospy.loginfo("Directory statistic info has been initiated...")

        ## Initiate timelapse capture
        self.capture = self.camera.initiate_open_capture(
                storage_id=storage_id,
                object_format=object_format,
        )
        # Wait for capture event
        event = self.camera.event(wait=False)
        rospy.loginfo("Open capture has been initiated...")
        
        rospy.loginfo("Waiting for a new directory to be initiated...")
        self.get_new_directory_info()
        rospy.loginfo("Directory info has been initiated...")
        
    def stop_open_capture(self):
        # Close the open capture session 
        self.camera.terminate_open_capture(transaction_id=self.capture.TransactionID)
        rospy.loginfo("Open capture has been stopped...")

    def close_open_session(self):
        # Close the PTP session
        self.camera.close_session()
        rospy.loginfo("Open camera session has been closed...")

    def delete_all_files(self):
        """
        Delete all files in the camera.
        Any session should be closed before deleting all files.
        """
        self.camera.delete_object(
            handle=0,
            object_format=0,
            delete_all=True,
            delete_all_images=True,)
        rospy.loginfo("All files have been deleted...")
        
    def get_new_directory_info(self):
        """
        When initiate open capture is created, it should create a new direcotry.
        This function continuously check if a new dir has been created
        """
        ## Check if a new dir has been created, and wait for the new dir until it's created
        new_dir_available = False
        new_directory = None
        dir_info_new = self.get_directory_info()
        while not new_dir_available:
            ## Check if new direcory has been created
            if  len(list(dir_info_new.keys())) > len(list(self.dir_info_init.keys())):
                new_dir_available = True
                rospy.loginfo("New directory has been created %s",list(self.dir_info_init.keys())[-1])
                new_directory = list(dir_info_new.keys())[-1]

            dir_info_new = self.get_directory_info()
            time.sleep(self._capture_interval)
            
        self.latest_directory = new_directory
       

    def retrieve_other_files(self):
        """
        diag.dat and params.dat are appended after a capture is stopped
        retrieve them with this funton
        """
        pass

    def convert_ptp_timestamp_to_ros(self, filename, header):
        """
        Convert image time stamp from Filename to ROS timestamp
        """
        # Regex to extract date and time (matches 240930_164044)
        match = re.search(r'(\d{6}_\d{6})', filename)

        if match:
            # Extracted date-time string
            datetime_str = match.group(1)

            # Parse the date and time using the format YYMMDD_HHMMSS
            dt = datetime.strptime(datetime_str, "%y%m%d_%H%M%S")

            # Convert to ROS timestamp (seconds since epoch)
            secs = int(time.mktime(dt.timetuple()))
            nsecs = 0  # No fractional seconds

            header.stamp.secs = secs
            header.stamp.nsecs = nsecs

            rospy.logdebug(f"Extracted ROS Time: {header.stamp.secs}.{header.stamp.nsecs} seconds since epoch")
        else:
            rospy.logdebug("No valid timestamp found in filename.")
        
    def retrieve_images(self):
        """
        Retrive new images, save or publish to a ros topic
        if publish_image_flag is enabled
        ToDo:
        add a trigger via a topic to enable and disable this
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
            # # Extract the timestamp from the image filename
            # timestamp = get_timestamp_from_filename(image_path)

            # # Group images by timestamp in a dictionary
            # image_queue.put((timestamp, image_path))
            
            all_handles = self.get_handles()
            handles = all_handles[start_idx:]

            if len(handles) > 0:
                for i,handle in enumerate(handles):
                    info = self.camera.get_object_info(handle)
                    if info is None:
                        continue

                    # save image
                    if info.ObjectFormat in self._file_format_to_save:
                        obj = self.camera.get_object(handle)

                        if not obj.Data:
                            rospy.logwarn("No image data received.")
                            continue

                        # convert and decode image to np array and cv
                        obj_array = np.frombuffer(obj.Data, self._data_type)
                        
                        if obj_array.size == 0:
                            rospy.logwarn("Received empty buffer from image data.")
                            continue

                        self.image_queue.put((info.Filename, obj_array))

                start_idx += len(handles)
            else:
                rospy.loginfo("No new handles to process: %s %s %s", str(len(all_handles)), str(len(handles)), str(start_idx))
            
            # delay < 0.0 is too short and may cause the file is not yet fully written
            time.sleep(self._capture_interval)

    def publish_image(self, delete_retrieved_file=False):
        """
        Retrive new images, save or publish to a ros topic
        if publish_image_flag is enabled
        ToDo:
        add a trigger via a topic to enable and disable this
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
        rospy.loginfo("Publishing multispectral data...")

        while not self.stop_retrieve_event.is_set():
            try:
                # Get the next image from the queue
                filename, obj_array = self.image_queue.get(timeout=1)

                img = cv2.imdecode(obj_array, cv2.IMREAD_COLOR)
                if img is None:
                    rospy.logwarn(f"Failed to decode image {str(filename)}")
                    continue

                img_msg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
                self.convert_ptp_timestamp_to_ros(filename, img_msg.header)
                
                spectrum = None

                if "RGB" in filename:
                    self._pub_rgb.publish(img_msg)
                    spectrum = "RGB"
                elif "NIR" in filename:
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

                if self._publish_group_images:
                    timestamp = img_msg.header.stam.secs
                    if timestamp not in image_groups:
                        image_groups[timestamp] = {}

                    if spectrum is not None:
                        image_groups[timestamp][spectrum] = img_msg

                    # If we have a complete set of 5 images, publish them
                    # assuming that each comes sequentially
                    # ToDo: run this in another thread???
                    if len(image_groups[timestamp]) == 5:
                        ms_msg = MultispectralImages()
                        ms_msg.rgb = image_group[timestamp]["RGB"]
                        ms_msg.nir = image_group[timestamp]["NIR"]
                        ms_msg.reg = image_group[timestamp]["REG"]
                        ms_msg.red = image_group[timestamp]["RED"]
                        ms_msg.gre = image_group[timestamp]["GRE"]
                        ms_msg.header = ms_msg.rgb.header
                        self._pub_ms_images.publish(ms_msg)

                        del image_groups[timestamp]

                # save files
                if self._save_files:
                    rospy.loginfo(f"Saving {self.image_path}/{info.Filename} ")
                    with open(f"{self.image_path}/{info.Filename}", mode='wb') as f:
                        f.write(obj_array)

            except queue.Empty:
                continue
       
        rospy.logwarn("Camera capture has been terminated...")
        # retrieve other files such as sequoia_diag.dat and sequoia_params.data
        rospy.loginfo("Retrieving squoia_diag.dat and sequoia_params.dat files...")
        all_handles = self.get_handles()
        handles = all_handles[dir_start_idx:]
        for i,handle in enumerate(handles):
            info = self.camera.get_object_info(handle)
            if info is None:
                continue

            if ".dat" in info.Filename:
                rospy.loginfo("Metadata: %s", info.Filename)
                obj = self.camera.get_object(handle)

                metadata_msg = UInt8MultiArray()
                metadata_msg.data = list(obj.Data)

                if "diag" in info.Filename:
                    self._pub_sequoia_diag.publish(metadata_msg)
                elif "param" in info.Filename:
                    self._pub_sequoia_param.publish(metadata_msg)

                # rospy.loginfo("Saving image %s", info.Filename)
                if self._save_files:
                    rospy.loginfo(f"Saving {self.image_path}/{info.Filename} ")
                    with open(f"{self.image_path}/{info.Filename}", mode='wb') as f:
                        f.write(obj.Data)
        
        rospy.loginfo("Capture has been stopped...")
        # delete all files
        #if delete_retrieved_file:
        #    self.camera.delete_object(handle=handle)
