#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from slot_environment import slotEnvironment
import json

def load_config(config_path):
    with open(config_path) as f:
        data = json.loads(f.read())
    return data


class slotControllerNode:
    def __init__(self, slot_length, slot_gap, num_lanes, lane_velocities, lane_width, road_length, num_slots_per_lane, frequency):
        rospy.init_node('listener_node', anonymous=True)
        self.slot_env = slotEnvironment(slot_length, slot_gap, num_lanes, lane_velocities, lane_width, road_length, num_slots_per_lane, frequency)

        self.THRESHOLD = 0.03

        self.lane_velocities =lane_velocities

 
        self.frequency = frequency
        self.dt = 1/self.frequency
        self.rate = rospy.Rate(frequency)


        self.threshold = 0.03


        self.location_topic = '/loc'
        self.velocity_topic='/vel'
        self.location_subscriber = rospy.Subscriber(self.location_topic,String, self.vehicle_data_recieved)
        self.velocity_publisher = rospy.Publisher(self.velocity_topic, String,queue_size=1)

        

    def run(self):
        while not rospy.is_shutdown():
            self.slot_env.update_slots()
            self.rate.sleep()

    def vehicle_data_recieved(self,msg):
        velocity = self.slot_env.find_veloctity(msg)
        self.velocity_publisher.publish(str(velocity))


if __name__ == '__main__':
    road_config=load_config('/root/workspace/src/swarm_controller/src/config/road_config.json')
    node = slotControllerNode(slot_length=road_config['slot_length'], slot_gap=road_config['slot_gap'],
                            num_lanes=road_config['num_lanes'], lane_velocities=road_config['lane_velocities'],
                            lane_width=road_config['lane_width'], road_length=road_config['road_length'],
                              num_slots_per_lane=road_config['num_slots_per_lane'], frequency=1)

    node.run()
    rospy.spin()

        


