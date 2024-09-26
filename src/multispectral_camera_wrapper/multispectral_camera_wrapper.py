import os
import time

import rospy
import ptpy

from rospkg import RosPack
from sensor_msgs.msg import Image
from std_msgs.msg import String


class MultiSpectralCameraWrapper(object):
    _pub_ms_data = None
    _pub_debug_img = None


    def __init__(self):
        self._config_file = rospy.get_param("~config_path", None)
        pkg_path = RosPack().get_path("multispectral_camera_wrapper")

        #if not os.path.isfile(self._config_file):
        #    rospy.logwarn("Config file is not given or does not exist")

        #self._config = file_utils.parse_yaml_config(self._config_file)
        self._debug = rospy.get_param("~debug", False)
        
        # flag to continuously update object info retrived from get_object_handles
        self.update_object_info = True
        
        # if enabled, it will continuously retrieve, save or publish new images to a ros topic
        self.saving_image_state = True
         
        # Initialize the camera connection here, because if it's stopped and
        # re-initialize during runtime, it will not find the USB
        self.camera = ptpy.PTPy()

        # Create Open PTP session
        response = self.camera.open_session()
        rospy.loginfo(f"Camera initiated with session id {response.SessionID}, transaction id {response.TransactionID}, response {response.ResponseCode}")

    def event_callback(self, data: String) -> None:
        event_out_data = String()
        if data.data == "e_start_capture":
            # set saving image state to True
            self.saving_image_state = True
            
            ## Initiate timelapse capture
            self.initiate_open_capture()
            rospy.loginfo("Open capture has been initiated...")
            
            # Sent event out without waiting image retrieval to be done
            event_out_data.data = "e_capture_started"
            self._pub_event.publish(event_out_data)

            rospy.loginfo("Retrieving image...")
            self.retrieve_image(delete_retrieved_file=False)

        elif data.data == "e_stop_capture":
            # set saving image state to False to exit while-loop in image saving func
            self.saving_image_state = False
            
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
            rospy.loginfo("Closing camera session...")
            self.close_open_session()
            rospy.loginfo("Camera session has been closed...")
            
            rospy.logwarn("Session has been closed, restart the node to start a new session...!")
             
            event_out_data.data = "e_session_stopped"
            self._pub_event.publish(event_out_data)
            

    def initialize(self) -> None:
        #start event in and out
        self._sub_event = rospy.Subscriber("~event_in", String, self.event_callback)
        self._pub_event = rospy.Publisher("~event_out", String, queue_size=1)

        # publishers
        self._pub_ms_data = rospy.Publisher("~ms_data", Image, queue_size=1)
        self._pub_debug_img = rospy.Publisher("~debug_image", Image, queue_size=1)

    def get_handles(self, storage_id=65537):
        """
        Get all object handles (directory for each capture)
        Defautl storage id = 65537 (internal)
        """
        handles = self.camera.get_object_handles(
            storage_id=storage_id,# memory card storage id = 131073
            object_format="TIFF",
            # object_handle=0,
            all_storage_ids=False, # disable this if storage id is specified
            all_formats=True,
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
    
    def initiate_open_capture(self):
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
                storage_id=65537,
                object_format=0,
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
            time.sleep(1.0)
            
        self.latest_directory = new_directory
        
        
    def retrieve_image(self, delete_retrieved_file=False):
        """
        Retrive new images, save or publish to a ros topic
        if saving_image_state is enabled
        ToDo:
        add a trigger via a topic to enable and disable this
        """
        # get latest dir info
        dir_info_new = self.get_directory_info()
        
        file_format_to_save = ["TIFF", "EXIF_JPEG", "UndefinedImage"]
        #file_format_to_save = ["EXIF_JPEG", "TIFF"]
        #file_format_to_save = ["TIFF"]

        # now start taking the image with the 2nd index of a new dir
        # +1 because the first index is the dir itself
        start_idx = dir_info_new[self.latest_directory]["start_handle_idx"] + 1
        
        image_count = 0

        prev_handles_count = 0

        while self.saving_image_state:
            start_time = time.time()
            all_handles = self.get_handles()
            handles = all_handles[start_idx:]

            if len(handles) > 0:
                for i,handle in enumerate(handles):
                    info = self.camera.get_object_info(handle)
                    if info is None:
                        continue

                    # save image
                    if info.ObjectFormat in file_format_to_save:
                        obj = self.camera.get_object(handle)
                        rospy.loginfo("Saving image %s", info.Filename)
                        #print(f"Saving {info.Filename}, Start ndex {start_idx}, all handles {len(all_handles)}, handles {len(handles)} ")
                        #with open(f"images/{info.Filename}", mode='wb') as f:
                        #    f.write(obj.Data)

                        image_count += 1

                    if not self.saving_image_state:
                        rospy.loginfo(f"Saving image state is disabled, waiting for the last files to be retrieved...")

                start_idx += len(handles)
            else:
                print("No new handles to process: ", len(all_handles), len(handles), start_idx)
            
            end_time = time.time()
            runtime = end_time - start_time
            print(f"Runtime: {runtime/len(handles):.2f} seconds / file")

            # delay < 0.0 is too short and may cause the file is not yet fully written
            time.sleep(1.0)

            
        # delete all files
        #if delete_retrieved_file:
        #    self.camera.delete_object(handle=handle)
