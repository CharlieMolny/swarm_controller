import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose2D
from threading import Thread
from multiprocessing import Manager, Process, Lock
from slot_environment import slotEnvironment, TrajectoryPlanner
import os, time
from swarm_controller.msg import velocity_msg
import re,math



class CommunicaitonControl:
    def __init__(self, number, shared_slots, lock, frequency, lane_velocity, initial_velocity=0.0):
        self.number = number
        self.vehicle_name = "duckie"+str(self.number) 
   
        self.lane_velocity = lane_velocity
        num_lanes = 2
        self.lane = (number -1) % num_lanes 

        self.shared_slots = shared_slots  
        self.lock = lock  
        self.frequency = frequency
        self.dt = 1/self.frequency
        self.velocity = initial_velocity

        self.threshold = 0.03
        self.max_acceleration = 0.01
        self.max_deceleration = -0.01

        # Set up ROS communication
        self.topic = '/{}/vel'.format(self.vehicle_name)
        self.setup_ros()

    def setup_ros(self):
       
        self.ros_uri = os.environ.get('ROS_MASTER_URI', 'Not set')
        self.velocity_pub = rospy.Publisher(self.topic, String, queue_size=10)
        
        self.location_topic = "/duckie1/location"
        rospy.loginfo("subscribing to topic is: {} and the ros URI master is {}".format(self.location_topic, os.environ.get('ROS_MASTER_URI', 'Not set')))
        self.location_sub = rospy.Subscriber(self.location_topic, String, self.location_callback)


        self.trajectory ={}
        self.sleep_rate = rospy.Rate(0.333)  
        self.sleep_rate.sleep()


    def parse_coordinates(self,input_string):

        pattern = r"x:\s*(\d+\.?\d*)\s*y:\s*(\d+\.?\d*)"
        
        # Search for the pattern in the input string
        match = re.search(pattern, input_string)
        
        if match:
            x = float(match.group(1))  # Convert the x value to a float
            y = float(match.group(2))  # Convert the y value to a float
            return {'x': x, 'y': y}
        else:
            return None

    def location_callback(self, msg):
        with self.lock:
            coorinates = self.parse_coordinates(msg.data)
            rospy.loginfo("recieved location data ({},{})".format(coorinates['x'],coorinates['y']))

            diff, nearest_slot_id = self.find_near_slot_same_lane(coorinates)
            rospy.loginfo("near slot id is: {}".format(nearest_slot_id))
            if nearest_slot_id is not None:
                slot = self.shared_slots[nearest_slot_id]
                velocity = self.compute_velocity()
                vel_msg= String(str(velocity))
                rospy.loginfo("publishing speed {}".format(velocity))
                self.velocity_pub.publish(vel_msg)  

    def find_near_slot_same_lane(self, coordinates):
        rospy.loginfo("in find near slot same lane {}".format(coordinates))
        # Logic to find the nearest slot in the same lane
        min_distance = float('inf')
        nearest_slot_id = None
        diff = None

        rospy.loginfo("begining for loop")
        for slot_id, slot in self.shared_slots.items():
            if slot['lane'] != self.lane:
                continue  

            distance = abs(slot['position']['x'] - coordinates['x'])
            if distance < min_distance and not slot['reserved']:
                min_distance = distance
                nearest_slot_id = slot_id
                diff = slot['position']['x'] - coordinates['x']


        if not self.shared_slots[nearest_slot_id]['reserved']:
            self.shared_slots[nearest_slot_id]['reserved'] = True
            rospy.loginfo("Slot {} reserved for Duckiebot {}".format(nearest_slot_id,self.number))

        
        if min_distance<self.threshold :
            self.shared_slots[nearest_slot_id]['filled'] = True

        self.find_trajectory(nearest_slot_id,diff)

        return diff, nearest_slot_id
    
    def find_trajectory(self,  slot_id, diff):
        v0 = self.lane_velocity
        if diff > 0:
            a1, a2 = self.max_acceleration, self.max_deceleration
            max_velocity, _ = self.find_max_velocity(v0, a1, a2, diff)
            self.trajectory = {
                'target_slot_id': slot_id,
                'max_velocity': max_velocity,
                'min_velocity': v0,
                'accelerate': True
            }
        else:
            a1, a2 = self.max_deceleration, self.max_acceleration
            min_velocity, _ = self.find_max_velocity(v0, a1, a2, diff)
            self.trajectory = {
                'target_slot_id': slot_id,
                'max_velocity': v0,
                'min_velocity': min_velocity,
                'accelerate': False
        }
            
        
            
    def find_max_velocity(self, v0, a1, a2, diff):
        '''------  derivation  ------
        s_slot=v0t + d
        s_vehicle= s1+s2pass
        v1^2 - 2v0v1 + v0^2 + (2a1a2d)/(a2 - a1) = 0

        a=1, b = -2v0, c = v0^2 + (2a1a2d)/(a2 - a1)

        v1 =(-b+sqrt(b^-4ac))/2a

        '''
        v1 = self.find_v_travel(v0, a1, a2, diff)

        t1 = (v1 - v0) / a1
        t2 = (v0 - v1) / a2

        return v1, t1 + t2

    def find_v_travel(self, v0, a1, a2, diff):
        if a2 == a1:
            raise ValueError("a1 and a2 must not be equal to avoid division by zero.")

        a = 1
        b = -2 * v0
        c1 = v0 ** 2
        c2 = -(2 * a1 * a2 * diff) / (a2 - a1)
        c = c1 + c2

        d = (b ** 2) - (4 * a * c)
        if d < 0:
            raise ValueError("No real solution exists for v1.")

        if diff > 0:
            v1 = (-b + math.sqrt(d)) / (2 * a)
        else:
            v1 = (-b - math.sqrt(d)) / (2 * a)

        return v1

    def compute_velocity(self):
       
        if self.trajectory['accelerate']:
            if self.velocity<self.trajectory['max_velocity']:
                self.velocity += self.max_acceleration*self.dt
            else:
                self.velocity += self.max_deceleration*self.dt
        else:
            if self.velocity>self.trajectory['min_velocity']:
                self.velocity +=self.max_deceleration*self.dt
            else:
                self.velocity +=self.max_acceleration*self.dt

        return self.velocity
    


class slotControllerNode:
    def __init__(self, slot_length, slot_gap, num_lanes, lane_velocities, lane_width, road_length, num_slots_per_lane, frequency, duckie_ip_addrs):
        self.slot_env = slotEnvironment(slot_length, slot_gap, num_lanes, lane_velocities, lane_width, road_length, num_slots_per_lane, frequency)

        self.THRESHOLD = 0.03

        self.lane_velocities =lane_velocities

        self.manager = Manager()
        self.shared_slots = self.manager.dict(self.slot_env.slots)  # Shared dict for slots
        self.lock = Lock()
        self.duckie_ip_addrs = duckie_ip_addrs

    def run(self):
        # Start thread to continuously update slots
        self.slot_update_thread = Thread(target=self.update_slots_continuously)
        self.slot_update_thread.start()

        # Create processes for each Duckiebot
        self.processes = []
        for i, ip in enumerate(self.duckie_ip_addrs, start=1):
            print("Starting process for Duckiebot at IP: {}".format(ip))
            proc = Process(target=self.run_duckiebot, args=(ip, i,))
            self.processes.append(proc)
            proc.start()

    def update_slots_continuously(self):
        while not rospy.is_shutdown():
            with self.lock:
                self.slot_env.update_slots()
                # Update shared_slots with latest slot data
                for slot_id, slot_data in self.slot_env.slots.items():
                    self.shared_slots[slot_id] = slot_data
            time.sleep(1 / self.slot_env.frequency)

    def run_duckiebot(self, duckie_ip, number):
        os.environ["ROS_MASTER_URI"] = duckie_ip
        rospy.init_node('duckiebot_{}'.format(number), anonymous=True)
        com_controller = CommunicaitonControl( number, self.shared_slots, self.lock, self.slot_env.frequency,self.lane_velocities[number-1])
        # com_controller.run()

