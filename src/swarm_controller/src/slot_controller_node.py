#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from slot_environment import slotEnvironment
import socket,select




class slotControllerNode:
    def __init__(self, slot_length, slot_gap, num_lanes, lane_velocities, lane_width, road_length, num_slots_per_lane, frequency):
        rospy.init_node('listener_node', anonymous=True)
        self.slot_env = slotEnvironment(slot_length, slot_gap, num_lanes, lane_velocities, lane_width, road_length, num_slots_per_lane, frequency)

        self.lane_velocities =lane_velocities
        self.frequency = frequency
        self.dt = 1/self.frequency
        self.rate = rospy.Rate(frequency)


        self.vehicle_names = rospy.get_param('vehicle_names', [])

        self.location_socket_num = rospy.get_param('slot_controller_server_socket_number')

        self.socket_num_dict = {}
        self.socket_connection_dict = {}


        for i,vehicle in enumerate(self.vehicle_names):
            socket_num = rospy.get_param('velocity_sender_server_socket_number_{}'.format(i), None)
            if socket_num is not None:
                self.socket_num_dict[vehicle] = socket_num
            else:
                rospy.logwarn("No socket number found for {}".format(vehicle))
        
  
            

    def initialize_local_node_connection(self, vehicle):
        try:
            socket_num = self.socket_num_dict.get(vehicle, None)

            if socket_num is not None:
                velocity_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                velocity_socket.connect(('localhost', socket_num))
                self.socket_connection_dict[vehicle] = velocity_socket

                rospy.loginfo("Connected to the local node for {} on port {}".format(vehicle, socket_num))
            else:
                rospy.logerr("Socket number for vehicle {} not found!".format(vehicle))
        except socket.error as e:
            rospy.logerr("Socket connection for vehicle {} on port {} failed: {}".format(vehicle, socket_num, e))


    def get_vehicle_socket(self, vehicle):
        return self.socket_connection_dict.get(vehicle, None)

            
    def run(self):

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
        velocity,vehcile_name = self.slot_env.find_veloctity(data)
        rospy.loginfo("{} velocity value is {}".format(vehcile_name,velocity))
        self.send_to_local_node(velocity,vehcile_name)


    def send_to_local_node(self, velocity_data,vehicle):
        try:
            socket_connection = self.get_vehicle_socket(vehicle)
            if socket_connection is None:
                self.initialize_local_node_connection(vehicle)
                socket_connection = self.get_vehicle_socket(vehicle)

            socket_connection.send(str(velocity_data).encode('utf-8'))
            rospy.loginfo("{} velocity data sent ".format(vehicle))
        except socket.error as e:
            rospy.logerr("Socket {} connection failed: {}".format(self.velocity_socket_num,e)) 
            
    

    def close_sockets(self):
        for vehicle, velocity_socket in self.socket_connection_dict.items():
            if velocity_socket:
                try:
                    velocity_socket.close()
                    rospy.loginfo("Closed socket connection for {}".format(vehicle))
                except socket.error as e:
                    rospy.logerr("Failed to close socket for {}: {}".format(vehicle, e))
        
        self.socket_connection_dict.clear()
        rospy.loginfo("All socket connections have been closed")




if __name__ == '__main__':
 
    node = slotControllerNode(slot_length=rospy.get_param('slot_length'), slot_gap=rospy.get_param('slot_gap'),
                            num_lanes=rospy.get_param('num_lanes'), lane_velocities=rospy.get_param('lane_velocities'),
                            lane_width=rospy.get_param('lane_width'), road_length=rospy.get_param('road_length'),
                              num_slots_per_lane=rospy.get_param('num_slots_per_lane'), frequency=rospy.get_param('frequency'))

    node.run()
    rospy.spin()

        


