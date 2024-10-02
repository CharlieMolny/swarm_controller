#!/usr/bin/env python
import rospy
import os
from std_msgs.msg import String
from multiprocessing import Process, Event, Manager


class LocationBridgeNode:
    def __init__(self):
        self.vehicle_name = rospy.get_param("~vehicle_name", "duckie1")
        self.duckie_ros_uri = rospy.get_param("~duckie_ros_uri", "http://192.168.158.187:11311")
        self.local_ros_uri = "http://localhost:11311"

        # Shared data structures and synchronization event
        self.manager = Manager()
        self.location = self.manager.Value('s', "")  # Shared string for location
        self.location_received_event = Event()  # Event to signal new location

        # External and local topics
        self.external_location_topic = '/{}/location'.format(self.vehicle_name)
        self.local_location_topic = '/loc'


        # Start separate processes for listening and sending
        listen_process = Process(target=self.listen_to_external_location)
        send_process = Process(target=self.send_local_location)

        listen_process.start()
        send_process.start()

        listen_process.join()
        send_process.join()

    def listen_to_external_location(self):
        
        os.environ["ROS_MASTER_URI"] = self.duckie_ros_uri
        rospy.init_node('listener_node', anonymous=True)
        rospy.Subscriber(self.external_location_topic, String, self.location_callback)
        rospy.spin()  

    def send_local_location(self):
        # Set up the sender node in a separate process
        os.environ["ROS_MASTER_URI"] = self.local_ros_uri
        rospy.init_node('sender_node', anonymous=True)

        # Publisher for local location
        local_location_pub = rospy.Publisher(self.local_location_topic, String, queue_size=1)

        
        rospy.loginfo("Starting to publish local location data")

        while not rospy.is_shutdown():
            # Wait until new location data is received
            if self.location_received_event.wait():  # This will block until the event is set
                local_location_pub.publish(self.location.value)  # Publish the shared location
                rospy.loginfo("Published local location: {}".format(self.location.value))
                self.location_received_event.clear()  

    def location_callback(self, msg):
        rospy.loginfo("Received external location data: {}".format(msg.data))
        # Update the shared location data
        self.location.value = msg.data
        self.location_received_event.set()  # Trigger the event to notify the sending process


if __name__ == '__main__':
    try:
        LocationBridgeNode()
    except rospy.ROSInterruptException:
        pass































