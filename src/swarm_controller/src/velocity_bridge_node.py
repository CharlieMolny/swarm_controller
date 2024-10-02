#!/usr/bin/env python
import rospy
from multiprocessing import Process, Event, Manager
import os
from std_msgs.msg import String

class velocityBridgeNode:
    def __init__(self):
        self.vehicle_name = rospy.get_param("~vehicle_name", "duckie1")
        self.duckie_ros_uri = rospy.get_param("~duckie_ros_uri", "http://192.168.158.187:11311")
        self.local_ros_uri = "http://localhost:11311"

        self.manager =Manager()
        self.velocity = self.manager.Value('s', "")  
        self.velocity_received_event = Event() 


        self.external_velocity_topic = '/{}/velocity'.format(self.vehicle_name)
     
        self.local_velocity_topic = '/vel'


        listen_process = Process(target=self.listen_to_velocity_local)
        send_process = Process(target=self.send_velocity_external)

        listen_process.start()
        send_process.start()

        listen_process.join()
        send_process.join()

    def listen_to_velocity_local(self):
               
        os.environ["ROS_MASTER_URI"] = self.local_ros_uri
        rospy.init_node('listener_node', anonymous=True)
        rospy.Subscriber(self.local_velocity_topic, String, self.velocity_callback)
        rospy.spin()  

    def send_velocity_external(self):

        os.environ["ROS_MASTER_URI"] = self.duckie_ros_uri
        rospy.init_node('sender_node', anonymous=True)

        local_location_pub = rospy.Publisher(self.external_velocity_topic, String, queue_size=1)

        
        rospy.loginfo("Starting to publish velocity location data")

        while not rospy.is_shutdown():
            if self.velocity_received_event.wait(): 
                local_location_pub.publish(self.velocity.value)  
                rospy.loginfo("Published  velocity: {}".format(self.velocity.value))
                self.velocity_received_event.clear() 
    # def send_velocity_external(self):
    #     os.environ["ROS_MASTER_URI"] = self.duckie_ros_uri
    #     rospy.init_node('sender_node', anonymous=True)

    #     # Publisher for velocity
    #     local_velocity_pub = rospy.Publisher(self.external_velocity_topic, String, queue_size=1)

    #     rospy.loginfo("sending on topic: {}".format(self.external_velocity_topic))
    #     rate = rospy.Rate(0.333)  # Frequency of publishing (1/3 Hz -> every 3 seconds)

    #     while not rospy.is_shutdown():
    #         test_velocity_data = "0.1"
    #         local_velocity_pub.publish(test_velocity_data)  # Publish test data
    #         rospy.loginfo("Published test velocity data: {}".format(test_velocity_data))
    #         rate.sleep()  # Sleep for 3 seconds


    def velocity_callback(self, msg):
        rospy.loginfo("Received velocity data: {}".format(msg.data))
        self.velocity.value = msg.data
        self.velocity_received_event.set()  


if __name__ == '__main__':
    try:
        velocityBridgeNode()
    except rospy.ROSInterruptException:
        pass


        