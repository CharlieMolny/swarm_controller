import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose2D
from threading import Thread
from multiprocessing import Manager, Process, Lock
from slot_environment import slotEnvironment, TrajectoryPlanner
import os, time
from swarm_controller.msg import velocity_msg

class CommunicaitonControl:
    def __init__(self, duckie_ip, number, shared_slots, lock, frequency, num_lanes, initial_speed=0.0):
        self.duckie_ip = duckie_ip
        self.number = number
        self.vehicle_name = "duckie"+str(self.number) 

        self.shared_slots = shared_slots  # Shared slot dict from the main process
        self.lock = lock  # Lock to prevent race conditions
        self.frequency = frequency
        self.speed = initial_speed
        self.num_lanes = num_lanes
        self.threshold = 0.03

        # Set up ROS communication
        self.topic = 'chatter'
        self.setup_ros()

    def setup_ros(self):
       
        
        self.ros_uri = os.environ.get('ROS_MASTER_URI', 'Not set')

        self.wheels_pub = rospy.Publisher(self.topic, String, queue_size=10)
        
        self.location_topic = "/duckie1/location"
        rospy.loginfo("subscribing to topic is: {} and the ros URI master is {}".format(self.location_topic, os.environ.get('ROS_MASTER_URI', 'Not set')))
        self.location_sub = rospy.Subscriber(self.location_topic, String, self.location_callback)

        self.sleep_rate = rospy.Rate(0.333)  # Sleep for 3 seconds to synchronize
        self.sleep_rate.sleep()

    # def run(self):
    #     rate = rospy.Rate(self.frequency)
    #     while not rospy.is_shutdown():
    #         # Send current speed to the Duckiebot
    #         log = "Duckiebot {} sending speed: {}".format(self.number,self.speed)
    #         rospy.loginfo(log)
    #         self.wheels_pub.publish(String(str(self.speed))) 
    #         rate.sleep()

    def location_callback(self, msg):
        rospy.loginfo("recieved location data ")
        with self.lock:
            
            velocity = None
            slot_id = None

            diff, nearest_slot_id = self.find_near_slot_same_lane(msg)
            if nearest_slot_id is not None:
                slot = self.shared_slots[nearest_slot_id]

                if not slot['reserved']:
                    slot['reserved'] = True
                    rospy.loginfo("Slot {} reserved for Duckiebot {}".format(nearest_slot_id,self.number))

                if abs(slot['x'] - msg.x) <  self.threshold:
                    slot['filled'] = True
                else:
                    slot['filled'] = False

                # Update shared_slots
                self.shared_slots[nearest_slot_id] = slot
                slot_id = nearest_slot_id

                # Compute velocity
                velocity = self.compute_velocity(msg, slot)

                # Publish velocity message (make sure velocity_msg is correctly defined)
                vel_msg = velocity_msg()
                vel_msg.velocity = velocity
                vel_msg.slot_id = slot_id
                self.wheels_pub.publish(vel_msg)  # Publish velocity message

    def find_near_slot_same_lane(self, msg):
        # Logic to find the nearest slot in the same lane
        min_distance = float('inf')
        nearest_slot_id = None
        diff = None

        with self.lock:
            for slot_id, slot in self.shared_slots.items():
                if slot['lane'] != self.number % self.num_lanes:
                    continue  # Skip slots not in the same lane

                distance = abs(slot['position']['x'] - msg.x)
                if distance < min_distance and not slot['reserved']:
                    min_distance = distance
                    nearest_slot_id = slot_id
                    diff = slot['position']['x'] - msg.x

        return diff, nearest_slot_id

    def compute_velocity(self, msg, slot):
        # Compute velocity based on slot position and bot position
        diff = slot['position']['x'] - msg.x
        # Implement your velocity calculation logic here
        # For simplicity, we'll set it proportional to the distance
        velocity = min(self.speed + diff * 0.1, self.speed)
        return velocity


class slotControllerNode:
    def __init__(self, slot_length, slot_gap, num_lanes, lane_velocities, lane_width, road_length, num_slots_per_lane, frequency, duckie_ip_addrs):
        self.slot_env = slotEnvironment(slot_length, slot_gap, num_lanes, lane_velocities, lane_width, road_length, num_slots_per_lane, frequency)

        self.max_velocity = None
        self.max_acceleration = 0.01
        self.max_deceleration = -0.01
        self.vehicle_id_to_slot_id = {}
        self.trajectories = {}
        self.THRESHOLD = 0.03

        self.trajectory_planner = TrajectoryPlanner(self.max_acceleration, self.max_deceleration, 1/frequency)

        # Create a multiprocessing manager and lock to handle shared slots
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
        com_controller = CommunicaitonControl(duckie_ip, number, self.shared_slots, self.lock, self.slot_env.frequency, len(self.slot_env.slots))
        # com_controller.run()

