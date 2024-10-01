#!/usr/bin/env python
import rospy,os
from std_msgs.msg import String

class LocalListener:
    def __init__(self):
        self.local_ros_uri = "http://localhost:11311"
        os.environ["ROS_MASTER_URI"] = self.local_ros_uri
        rospy.init_node('local_publisher', anonymous=True)

        # Subscribe to the local topic where the DuckieListenerNode publishes
        self.location_sub = rospy.Subscriber('/loc', String, self.location_callback)


        rospy.loginfo("listening to bot")

    def location_callback(self, msg):
        # Process the location data and publish it locally
        rospy.loginfo("Processing location: %s", msg.data)


if __name__ == '__main__':
    os.environ["ROS_MASTER_URI"] = "http://localhost:11311"  # Local ROS master

    try:
        publisher = LocalListener()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

