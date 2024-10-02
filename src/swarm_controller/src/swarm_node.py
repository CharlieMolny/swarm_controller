#!/usr/bin/env python

import os,rospy,time,json
from std_msgs.msg import String
from multiprocessing import Manager, Process,Lock
from slot_controller_node import slotControllerNode
import threading

DUCKIE_IP_ADDRESSES = ['http://192.168.158.187:11311' ]
FREQUENCY =1 


def load_config(config_path):
    with open(config_path) as f:
        data = json.loads(f.read())
    return data


def main():


    road_config=load_config('/root/workspace/src/swarm_controller/src/config/road_config.json')


    slot_env = slotControllerNode(slot_length=road_config['slot_length'],slot_gap=road_config['slot_gap'], num_lanes=road_config['num_lanes'],lane_velocities=road_config['lane_velocities'],
                               lane_width=road_config['lane_width'],road_length=road_config['road_length'],num_slots_per_lane=road_config['num_slots_per_lane'],frequency=FREQUENCY,duckie_ip_addrs=DUCKIE_IP_ADDRESSES )
    
    slot_env.run()



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print("ROS Interrupt Exception")
