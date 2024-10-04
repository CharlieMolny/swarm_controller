#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from slot_environment import slotEnvironment
import json
import socket,select

def load_config(config_path):
    with open(config_path) as f:
        data = json.loads(f.read())
    return data


class slotControllerNode:
    def __init__(self, slot_length, slot_gap, num_lanes, lane_velocities, lane_width, road_length, num_slots_per_lane, frequency):
        rospy.init_node('listener_node', anonymous=True)
        self.slot_env = slotEnvironment(slot_length, slot_gap, num_lanes, lane_velocities, lane_width, road_length, num_slots_per_lane, frequency)


        self.lane_velocities =lane_velocities

 
        self.frequency = frequency
        self.dt = 1/self.frequency
        self.rate = rospy.Rate(frequency)



        self.velocity_s =None
        self.location_socket_num =rospy.get_param('slot_controller_server_socket_number',None)
        # self.location_socket_num = 5000
        self.velocity_socket_num = rospy.get_param('velocity_sender_server_socket_number',None)




    def initilise_local_node_connection(self):
        
        try:
            self.velocity_s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.velocity_s.connect(('localhost', self.velocity_socket_num))  # Connect to the local node once
            rospy.loginfo("Connected to the local node on port {}".format(self.velocity_socket_num))
        except socket.error as e:
            rospy.logerr("Socket {} connection failed: {}".format(self.velocity_socket_num,e))


            
    def run(self):
        # Create the socket
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.bind(('localhost', self.location_socket_num)) 
        s.listen(1)
        s.setblocking(0)

        inputs = [s]  

        while not rospy.is_shutdown():
   
            readable, _, _ = select.select(inputs, [], [], 0)  

            for sock in readable:
                if sock is s:
                    conn, addr = s.accept() 
                    conn.setblocking(0) 
                    inputs.append(conn) 
                else:
                    try:
                        data = sock.recv(1024) 
                        if data:
                            self.vehicle_data_recieved(data.decode('utf-8'))  
                        else:
                            inputs.remove(sock)  
                            sock.close()
                    except socket.error as e:
                        rospy.logerr("Socket error: {}".format(e))
                        inputs.remove(sock)  
                        sock.close()

            self.slot_env.update_slots()

            self.rate.sleep()
            
    def vehicle_data_recieved(self,data):
        velocity = self.slot_env.find_veloctity(data)
        rospy.loginfo("velocity value is {}".format(velocity))
        self.send_to_local_node(velocity)
    

    def send_to_local_node(self, velocity_data):
        try:
            if self.velocity_s is None:
                self.initilise_local_node_connection()
 
            self.velocity_s.send(str(velocity_data).encode('utf-8'))
        except socket.error as e:
            rospy.logerr("Socket {} connection failed: {}".format(self.velocity_socket_num,e))
            # self.velocity_s.close()
            self.velocity_s = None
    

    def close_socket(self):
        if self.velocity_s:
            self.velocity_s.close()
            self.velocity_s = None
            rospy.loginfo("Closed socket connection")



if __name__ == '__main__':
 
    node = slotControllerNode(slot_length=rospy.get_param('slot_length'), slot_gap=rospy.get_param('slot_gap'),
                            num_lanes=rospy.get_param('num_lanes'), lane_velocities=rospy.get_param('lane_velocities'),
                            lane_width=rospy.get_param('lane_width'), road_length=rospy.get_param('road_length'),
                              num_slots_per_lane=rospy.get_param('num_slots_per_lane'), frequency=rospy.get_param('frequency'))

    node.run()
    rospy.spin()

        


