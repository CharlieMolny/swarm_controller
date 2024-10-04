#!/usr/bin/env python
import rospy
import socket
import os
from std_msgs.msg import String

class LocalVelocitySender:
    def __init__(self):

        self.vehicle_name = rospy.get_param('vehicle_name', 'not set')
        self.duckie_ros_uri = rospy.get_param('ros_master_uri','not set')
        self.socket_num = rospy.get_param('velocity_sender_server_socket_number',None)
  
        os.environ["ROS_MASTER_URI"] = self.duckie_ros_uri
        rospy.init_node('external_velocity_sender', anonymous=True)
        self.vehicle_name = "duckie1"
        self.topic = "/{}/velocity".format(self.vehicle_name)

        self.velocity_publisher = rospy.Publisher(self.topic, String, queue_size=1)
        rospy.loginfo("Velocity sender initialized")
        self.start_socket_server()

    def start_socket_server(self):
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            socket_num = 5010
            s.bind(('0.0.0.0', socket_num))  
            s.listen(1)  
            rospy.loginfo("Listening for incoming data on socket...{}".format(socket_num))
            
            conn, addr = s.accept() 
            rospy.loginfo("Connected by {}".format(addr))
        except socket.error as e:
            rospy.logerr("Socket error during setup: {}".format(e))
            return

        try:
            while not rospy.is_shutdown():
                try:
                    data = conn.recv(1024) 
                    if data:
                        rospy.loginfo("Received velocity data from external node: {}".format(data.decode('utf-8')))
                        self.velocity_publisher.publish(data.decode('utf-8'))
                    else:
                        break  
                except socket.error as e:
                    rospy.logerr("Socket {} error during data reception: {}".format(socket_num,e))
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
        sender = LocalVelocitySender()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node interrupted and shutting down.")
