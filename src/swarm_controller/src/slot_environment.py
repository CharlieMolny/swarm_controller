import math
import rospy
import re
import time

class Slot:
    def __init__(self, id, lane, location={}, reserved=False, filled=False):
        self.id = id
        self.location = location
        self.reserved = reserved
        self.filled = filled
        self.lane = lane
        self.end_of_road = False

    def to_dict(self):
        return {
            'id': self.id,
            'location': self.location,
            'reserved': self.reserved,
            'filled': self.filled,
            'lane': self.lane,
            'end_of_road': self.end_of_road
        }

    @staticmethod
    def from_dict(data):
        return Slot(
            id=data['id'],
            lane=data['lane'],
            location=data['location'],
            reserved=data['reserved'],
            filled=data['filled']
        )


class slotEnvironment:
    def __init__(self, slot_length, slot_gap, num_lanes, lane_velocities, lane_width, road_length, num_slots_per_lane, frequency):
        self.slot_length = slot_length
        self.slot_gap = slot_gap
        self.lane_width = lane_width
        self.road_length = road_length
        self.num_lanes = num_lanes
        self.lane_velocities = lane_velocities
        self.slot_count = 0
        self.slots = {}
        self.num_slots_per_lane = num_slots_per_lane
        self.frequency = frequency
        self.dt = 1/self.frequency
        self.max_acceleration = 0.005
        self.max_deceleration = -0.005
        self.threshold = 0.03
        self.trajectories = {}
        self.in_slot_counter ={}
        self.diff_tracker={}
        self.timers= {}

        self.generate_slots()

    def generate_slots(self):
        lane_spacing = self.lane_width  

        for lane in range(self.num_lanes):
            y_location = lane * lane_spacing

            total_space = self.road_length - (self.num_slots_per_lane * self.slot_length)
            if total_space < 0:
                raise ValueError("Not enough road length to fit the slots with the given slot length.")
            
            if self.num_slots_per_lane > 1:
                slot_spacing = total_space / (self.num_slots_per_lane - 1)
            else:
                slot_spacing = 0 

            for i in range(self.num_slots_per_lane):
                x_location = i * (self.slot_length + slot_spacing)
                
                new_slot = Slot(id=self.slot_count, lane=lane, location={'x': x_location, 'y': y_location})
                self.slots[self.slot_count] = new_slot.to_dict()
                self.slot_count += 1

        return self.slots

    def update_slots(self):
        dt = 1 / self.frequency
        slots_to_remove = []

        for slot_id, slot in list(self.slots.items()):
            if slot['location']['x'] < self.road_length:
                slot['location']['x'] += dt * self.lane_velocities[slot['lane']]
            else:
                slots_to_remove.append(slot_id)

        for slot_id in slots_to_remove:
            lane = self.slots[slot_id]['lane']
            y_location = self.slots[slot_id]['location']['y']

            del self.slots[slot_id]

            new_slot_id = max(self.slots.keys()) + 1 if self.slots else 0

            new_slot = {
                'id': new_slot_id,
                'location': {'x': 0, 'y': y_location},
                'lane': lane,
                'reserved': False,
                'filled': False,
                'vel': self.lane_velocities[lane]
            }

            self.slots[new_slot_id] = new_slot


    

    def find_veloctity(self,data):
 
        vehicle_name,lane,vehicle_location = self.parse_data(data)
        if vehicle_name not in self.in_slot_counter:
            self.in_slot_counter[vehicle_name] = 0
   
        
        if vehicle_name not in self.trajectories:
            print("trajectories are {}".format(self.trajectories))
            diff,nearest_slot_id=self.assign_near_slot(vehicle_location,vehicle_name,lane)
   
            

        slot_id = self.trajectories[vehicle_name]['target_slot_id']

        slot_x_location = self.slots[slot_id]['location']['x']
        diff= abs(slot_x_location-vehicle_location['x'])

        if not self.trajectories[vehicle_name]['in_slot']:
            if vehicle_name not in self.diff_tracker :
                self.diff_tracker[vehicle_name]= [diff]
            else:
                self.diff_tracker[vehicle_name].append(diff)

        rospy.loginfo("{}'s diff is {}".format(vehicle_name,diff))


        if (diff<=self.threshold) and not self.trajectories[vehicle_name]['in_slot']:
            self.trajectories[vehicle_name]['in_slot'] = True
            rospy.loginfo("{}  has caught up its slot".format(vehicle_name))
            self.timers[vehicle_name]=time.time()+1.5

        elif self.trajectories[vehicle_name]['in_slot']:
            if self.in_slot_counter[vehicle_name]%5 and diff>2*self.threshold==0:
                rospy.loginfo("{} is no longer in slot, reassigning vehicle".format(vehicle_name))
                current_velocity = self.trajectories[vehicle_name]['v_(i-1)']
                self.assign_near_slot(vehicle_location,vehicle_name,lane,current_velocity)
                self.diff_tracker.pop(vehicle_name)

            self.in_slot_counter[vehicle_name]+=1
        
        elif vehicle_name in self.diff_tracker>2:
            if len(self.diff_tracker[vehicle_name])>2:
                diff_2,diff_1 = self.diff_tracker[vehicle_name][-3],self.diff_tracker[vehicle_name][-2]
                rospy.loginfo("diff i-2: {}  diff i-1: {}  diff: {}".format(diff_2,diff_1,diff))
                if diff_2<diff_1<diff:
                    current_velocity = self.trajectories[vehicle_name]['v_(i-1)']
                    self.trajectories.pop(vehicle_name)
                    self.assign_near_slot(vehicle_location,vehicle_name,lane,current_velocity)
                    self.diff_tracker.pop(vehicle_name)
                    if vehicle_name in self.timers:
                        self.timers.pop(vehicle_name)
            
                
        # print("self timers at this point: {}".format(self.timers))
        if self.trajectories[vehicle_name]['in_slot']:
            if vehicle_name in self.timers:
                if time.time()>self.timers[vehicle_name]:
                    rospy.loginfo("time now: {} time ends: {}".format(time.time(),self.timers[vehicle_name]))
                    diff =self.move_slot(vehicle_name,vehicle_location,lane,'Forward')
                    self.timers.pop(vehicle_name)
                    if vehicle_name in self.diff_tracker:
                        self.diff_tracker.pop(vehicle_name)
            else:
                self.timers[vehicle_name] = time.time()+1.5

            
        self.trajectories[vehicle_name]['current_difference'] = diff

        return self.compute_velocity(vehicle_name),vehicle_name
        
            

    def assign_near_slot(self, location,vehicle_name,lane,current_velocity=0):
        min_distance = float('inf')
        nearest_slot_id = None
        diff = None

        for slot_id, slot in self.slots.items():
            if slot['lane'] != lane:
                continue  

            distance = abs(slot['location']['x'] - location['x'])
            if distance < min_distance and not slot['reserved']:
                min_distance = distance
                nearest_slot_id = slot_id
                diff = slot['location']['x'] - location['x']
        
       

        
        if min_distance<self.threshold :
            self.slots[nearest_slot_id]['filled'] = True

        

        self.find_trajectory(nearest_slot_id,diff,vehicle_name,lane,current_velocity)
        return diff, nearest_slot_id
    

    def is_path_clear(self, ego_location, candidate_slot_location):
        start_x = min(ego_location['x'], candidate_slot_location['x'])
        end_x = max(ego_location['x'], candidate_slot_location['x'])

        for slot in self.slots.values():
            if start_x < slot['location']['x'] < end_x and not slot['reserved'] and not slot['filled']:
                return False
        return True


    
    def find_trajectory(self,  slot_id, diff,vehicle_name,lane,current_velocity):
        final_velocity = self.lane_velocities[lane]
        print("{} difference is {}".format(vehicle_name,diff))

        if  0<abs(diff)<self.threshold:
            a1, a2 = 0,0 
            print("{} is at slot ".format(vehicle_name))
           
            self.trajectories[vehicle_name] = {
                'target_slot_id': slot_id,
                'v0': final_velocity,
                'v1': final_velocity,
                'v2':final_velocity,
                'v_(i-1)': final_velocity,
                'current_difference': diff,

                'accelerate': False,
                'in_slot': True

            }
            self.timers[vehicle_name]=time.time()+1.5
 

        elif diff > 0:
            a1, a2 = self.max_acceleration, self.max_deceleration
            max_velocity, _ = self.find_max_velocity(current_velocity,final_velocity ,a1, a2, diff)
            print("{} is catching up to slot ".format(vehicle_name))
            
            self.trajectories[vehicle_name] = {
                'target_slot_id': slot_id,
                'v0': current_velocity,
                'v1': max_velocity,
                'v2': final_velocity,
                'v_(i-1)': current_velocity,
                'current_difference': diff,
                'accelerate': True,
                'in_slot': False
            }
            print("{}  trajectory is {}".format(vehicle_name,self.trajectories[vehicle_name]))
        else:
            print("{} is slowing down to slot ".format(vehicle_name))
            a1, a2 = self.max_deceleration, self.max_acceleration
            min_velocity, _ = self.find_max_velocity(current_velocity, final_velocity,a1, a2, diff)
            self.trajectories[vehicle_name] = {
                'target_slot_id': slot_id,
                'v0': current_velocity,
                'v1': min_velocity,
                'v2': final_velocity,
                'v_(i-1)': current_velocity,
                'current_difference':diff,
                'accelerate': False,
                'in_slot': False
        }
            
            
    def find_max_velocity(self, v0,v2, a1, a2, diff):
        '''------  derivation  ------
        s_slot=v0t + d
        s_vehicle= s1+s2pass
        v1^2 - 2v0v1 + v0^2 + (2a1a2d)/(a2 - a1) = 0

        a=1, b = -2v0, c = v0^2 + (2a1a2d)/(a2 - a1)

        v1 =(-b+sqrt(b^-4ac))/2a

        '''
        v1 = self.find_v_travel(v0,v2, a1, a2, diff)

        t1 = (v1 - v0) / a1
        t2 = (v2 - v1) / a2

        print("t1: {} v1")

        s1 = (v1**2-v0**2)/(2*a1)
        s2 = (v2**2-v1**2)/(2*a2)

        LHS  = s1+ s2

        T = t1+t2
        RHS = diff + v2*T

        rospy.loginfo("left hand side is: {}, right hand side is: {}, total time is {}".format(LHS,RHS,T))

        return v1, T

    def find_v_travel(self, v0,v2, a1, a2, diff):
        if a2 == a1:
            raise ValueError("a1 and a2 must not be equal to avoid division by zero.")

        a = a2-a1
        b = -2 *(a2-a1) * v2

        c1 = -a2*(v0**2) 
        c2 =2*a2*v0*v2
        c3=-(a1*(v2**2))
        c4 = -(2 * a1 * a2 * diff)
        c = c1 + c2 + c3 + c4

        d = (b ** 2) - (4 * a * c)


        if d < 0:
            raise ValueError("No real solution exists for v1.")
        
        v_plus = (-b + math.sqrt(d)) / (2 * a)
        v_minus =  (-b - math.sqrt(d)) / (2 * a)


        v1 = v_minus



        print("a: {} b: {} c: {}  v-: {}  v+:{}".format(a,b,c,v_minus,v_plus))

        return v1

    def compute_velocity(self,vehicle_name):
        if self.trajectories[vehicle_name]['in_slot']:
            self.trajectories[vehicle_name]['v_(i-1)']=self.trajectories[vehicle_name]['v2']

        elif self.trajectories[vehicle_name]['accelerate']:
            if self.trajectories[vehicle_name]['v_(i-1)']<self.trajectories[vehicle_name]['v1']:
                self.trajectories[vehicle_name]['v_(i-1)']+= self.max_acceleration*self.dt
            else:
                self.trajectories[vehicle_name]['v_(i-1)']+= self.max_deceleration*self.dt
        else:
            if self.trajectories[vehicle_name]['v_(i-1)']>self.trajectories[vehicle_name]['v1']:
                self.trajectories[vehicle_name]['v_(i-1)'] +=self.max_deceleration*self.dt
            else:
                self.trajectories[vehicle_name]['v_(i-1)'] +=self.max_acceleration*self.dt

        return self.trajectories[vehicle_name]['v_(i-1)']
    

    def parse_data(self, input_string):
        input_string = str(input_string)

        pattern = r'vehicle_name:\s*(\w+)\s*lane:\s*(\d+)\s*x:\s*(-?\d+\.?\d*)\s*y:\s*(-?\d+\.?\d*)'

        match = re.search(pattern, input_string)
        
        if match:
            vehicle_name = str(match.group(1))  
            lane = int(match.group(2))  

            x = float(match.group(3))  
            y = float(match.group(4))  
            location ={'x':x,'y':y}
            
            return vehicle_name,lane,location
        else:
            return None
        
    def move_slot(self, vehicle_name,vehicle_location,lane,direction):
        rospy.loginfo("{} moving slot ".format(vehicle_name))
        current_velocity = self.trajectories[vehicle_name]['v_(i-1)']
        if direction =='Forward':
            slot,diff,new_slot_id=self.find_slot_in_front(vehicle_location['x'],lane)
        rospy.loginfo("new slot id is {}".format(slot['id']))
        self.find_trajectory(slot['id'],diff,vehicle_name,lane,current_velocity)
        return diff


    def find_slot_in_front(self, bot_x_location, bot_lane):
        slot_in_front = None
        new_slot_id = None
        min_distance = float('inf')



        for slot_id, slot in self.slots.items():

            if slot['lane'] == bot_lane and slot['location']['x'] > bot_x_location:
                distance = slot['location']['x'] - bot_x_location
                if distance < min_distance:
                    min_distance = distance
                    slot_in_front = slot
                    new_slot_id = slot_id
   

        return slot_in_front,min_distance,new_slot_id



