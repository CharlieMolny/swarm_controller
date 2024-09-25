#!/usr/bin/env python

import os,rospy,time,json
from std_msgs.msg import String
from multiprocessing import Manager, Process,Lock
from communcation_node import CommunicaitonControl
from slot_controller_node import slotControllerNode
import threading

DUCKIE_IP_ADDRESSES = ['http://192.168.78.187:11311', 'http://192.168.78.205:11311']
FREQUENCY =1 



def load_config(config_path):
    with open(config_path) as f:
        data = json.loads(f.read())
    return data


def update_slots_continuously(shared_slots, slot_env):
    dt =1/FREQUENCY
    while True:
        slot_env.update_slots()
        with Lock:
            # Directly modify the shared_slots dictionary (no need to copy)
            for slot_id, slot_data in slot_env.slots.items():
                shared_slots[slot_id] = slot_data  # In-place update
        # print(shared_slots)
    
        time.sleep(dt) 

def run_duckiebot(duckie_ip, number):

    com_controller = CommunicaitonControl(duckie_ip, number,FREQUENCY)
    com_controller.run()


def main():
    manager= Manager()
    shared_slots =manager.dict()

    road_config=load_config('src/swarm_controller/src/config/road_config.json')


    slot_env = slotControllerNode(slot_length=road_config['slot_length'],slot_gap=road_config['slot_gap'], num_lanes=road_config['num_lanes'],lane_speeds=road_config['lane_speeds'],
                               lane_width=road_config['lane_width'],road_length=road_config['road_length'],num_slots_per_lane=road_config['num_slots_per_lane'],frequency=FREQUENCY )


    shared_slots = slot_env.slots
    print(shared_slots)

    # Create a thread to update slots periodically (1Hz)
    update_thread = threading.Thread(target=update_slots_continuously, args=(shared_slots, slot_env))
    update_thread.start()


    processes = []
    for i, ip in enumerate(DUCKIE_IP_ADDRESSES):
        print("Starting process for IP:", ip)
        proc = Process(target=run_duckiebot, args=(ip, i,))
        processes.append(proc)
        proc.start()

    for proc in processes:
        proc.join()

    update_thread.join()



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print("ROS Interrupt Exception")
