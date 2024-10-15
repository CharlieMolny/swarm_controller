#!/usr/bin/env python
import rospy
import socket
import os
from std_msgs.msg import String
import json



class ExternalLocationListener:
    def __init__(self,params):
        print("beginning initialising location node ")
        rospy.init_node('external_location_listener')

        self.node_name = rospy.get_name()

        self.vehicle_number = int(self.node_name[-1])-1
        

        self.vehicle_names = params["vehicle_names"]
        self.vehicle_name =self.vehicle_names[self.vehicle_number]

        self.socket_num = params['slot_controller_server_socket_number']

    
        self.external_location_topic = '/{}/location'.format(self.vehicle_name)
        
        rospy.Subscriber(self.external_location_topic, String, self.location_callback)
        self.s = None
        
        
        self.initilise_local_node_connection()

        
    
    def initilise_local_node_connection(self):
        try:
            self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.s.connect(('localhost', self.socket_num))  # Connect to the local node once
            rospy.loginfo("Connected to the local node on port {}".format(self.socket_num))
        except socket.error as e:
            rospy.logerr("Socket {} connection failed: {}".format(self.socket_num,e))
        

    def location_callback(self, msg):

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
    params ={
    "vehicle_names": ["duckie3", "duckie2"],
    "velocity_sender_server_sockets": [5021, 5023],
    "slot_controller_server_socket_number": 5002
            }
    
    listener = ExternalLocationListener(params)
    rospy.spin()
