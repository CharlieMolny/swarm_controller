#!/usr/bin/env python
import rospy
import socket
import os
from std_msgs.msg import String

import json 


class LocalVelocitySender:
    def __init__(self,params):
        print("velocity sender initialised ")

        rospy.init_node('external_velocity_sender')


        self.node_name = rospy.get_name()

        self.vehicle_number = int(self.node_name[-1])-1

        
        self.vehicle_names = params["vehicle_names"]
        self.vehicle_name =self.vehicle_names[self.vehicle_number]

        
        self.socket_nums = params["velocity_sender_server_sockets"]
        
        self.socket_num = self.socket_nums[self.vehicle_number]
                

        self.topic ="/{}/velocity".format(self.vehicle_name)

        self.velocity_publisher = rospy.Publisher(self.topic, String, queue_size=1)

        print("parameters set ")      

        self.should_move = False
        self.velocity = 0.0
        self.rate = rospy.Rate(10)  

        self.start_socket_server()
        print("velocity node initialised ")


    def start_socket_server(self):
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

            s.bind(('0.0.0.0', self.socket_num))  
            s.listen(1)  
            rospy.loginfo("Listening for incoming data on socket...{}".format(self.socket_num))
            
            conn, addr = s.accept() 
            rospy.loginfo("Connected by {}".format(addr))
        except socket.error as e:
            rospy.logerr("Socket {} error during setup: {}".format(self.socket_num,e))
            return

        try:
            while not rospy.is_shutdown():
                try:
                    data = conn.recv(1024) 
                    if data:
                        rospy.loginfo("{} Received velocity data from external node: {}".format(self.vehicle_name,data.decode('utf-8')))
                        self.velocity_publisher.publish(data.decode('utf-8'))
                    else:
                        break  
                except socket.error as e:
                    rospy.logerr("Socket {} error during data reception: {}".format(self.socket_num,e))
                    break
        finally:
            conn.close() 
            rospy.loginfo("Connection closed")


    def close_socket(self):
        if self.s:
            self.s.close()
            self.s = None
            rospy.loginfo("Closed socket connection")


    def on_shutdown(self):
        self.close_socket()


if __name__ == '__main__':
    try:
        params ={
    "vehicle_names": ["duckie3", "duckie2"],
    "velocity_sender_server_sockets": [5021, 5023],
    "slot_controller_server_socket_number": 5002
            }
        sender = LocalVelocitySender(params)
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node interrupted and shutting down.")
