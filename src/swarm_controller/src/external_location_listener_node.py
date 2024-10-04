#!/usr/bin/env python
import rospy
import socket
import os
from std_msgs.msg import String

class ExternalLocationListener:
    def __init__(self):
        self.vehicle_name = rospy.get_param('vehicle_name', 'not set')

        self.duckie_ros_uri = rospy.get_param('ros_master_uri','not set')
        self.socket_num = rospy.get_param('slot_controller_server_socket_number',None)

        #self.duckie_ros_uri =  "http://192.168.158.187:11311"
        self.external_location_topic = '/{}/location'.format(self.vehicle_name)

        os.environ["ROS_MASTER_URI"] = self.duckie_ros_uri
        rospy.init_node('external_location_listener', anonymous=True)
        rospy.Subscriber(self.external_location_topic, String, self.location_callback)
        self.s = None
        self.socket_num = 5000
        
        self.initilise_local_node_connection()
    
    def initilise_local_node_connection(self):
        try:
            self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.s.connect(('localhost', self.socket_num))  # Connect to the local node once
            rospy.loginfo("Connected to the local node on port {}".format(self.socket_num))
        except socket.error as e:
            rospy.logerr("Socket connection failed: {}".format(e))
        

    def location_callback(self, msg):
        rospy.loginfo("Received external location data: {}".format(msg.data))
        self.send_to_local_node(msg.data)

    def send_to_local_node(self, location_data):
        try:
            if self.s is None:
                self.initilise_local_node_connection()
  
            self.s.send(location_data.encode('utf-8'))
        except socket.error as e:
            rospy.logerr("Socket {} connection failed: {}".format(self.socket_num,e))
            self.s.close()
            self.s = None
    

    def close_socket(self):
        if self.s:
            self.s.close()
            self.s = None
            rospy.loginfo("Closed socket connection")


    def on_shutdown(self):
        self.close_socket()



if __name__ == '__main__':
    vehicle_name = "duckie1"
    listener = ExternalLocationListener()
    rospy.spin()
