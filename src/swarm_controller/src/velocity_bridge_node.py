#!/usr/bin/env python
import rospy
import multiprocessing,multimaster_msgs_fkie

class velocityBridgeNode:
    def __init__(self):
        self.vehicle_name = rospy.get_param("~vehicle_name", "duckie1")
        self.duckie_ros_uri = rospy.get_param("~duckie_ros_uri", "http://192.168.158.187:11311")
        self.local_ros_uri = "http://localhost:11311"


        self.manager = multiprocessing.Manager()
        self.location = self.manager.Value('s', "")  # Shared string for location
        self.location_received_event = multiprocessing.Event()  # Event to signal new location
        