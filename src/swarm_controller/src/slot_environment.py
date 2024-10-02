import math
import rospy
import re


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
        self.max_acceleration = 0.01
        self.max_deceleration = -0.01
        self.threshold = 0.05
        self.trajectories = {}
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
        vehicle_name,lane,location = self.parse_data(data)
        if vehicle_name not in self.trajectories:
            self.assign_near_slot_same_lane(location,vehicle_name,lane)
        return self.compute_velocity(vehicle_name)
        


    def assign_near_slot_same_lane(self, location,vehcile_name,lane):
        rospy.loginfo("in find near slot same lane {}".format(location))
        min_distance = float('inf')
        nearest_slot_id = None
        diff = None

        rospy.loginfo("begining for loop")
        for slot_id, slot in self.slots.items():
            if slot['lane'] != lane:
                continue  

            distance = abs(slot['location']['x'] - location['x'])
            if distance < min_distance and not slot['reserved']:
                min_distance = distance
                nearest_slot_id = slot_id
                diff = slot['location']['x'] - location['x']

            rospy.loginfo("Slot {} reserved for Duckiebot {}".format(nearest_slot_id,vehcile_name))
        
        if min_distance<self.threshold :
            self.slots[nearest_slot_id]['filled'] = True

        self.find_trajectory(nearest_slot_id,diff,vehcile_name,lane)

        return diff, nearest_slot_id
    


    def is_path_clear(self, ego_location, candidate_slot_location):
        start_x = min(ego_location['x'], candidate_slot_location['x'])
        end_x = max(ego_location['x'], candidate_slot_location['x'])

        for slot in self.slots.values():
            if start_x < slot['location']['x'] < end_x and not slot['reserved'] and not slot['filled']:
                return False
        return True


    
    def find_trajectory(self,  slot_id, diff,vehicle_name,lane):
        final_velocity = self.lane_velocities[lane]

        if  0<abs(diff)<0.1:
            a1, a2 = 0,0
  
            
            self.trajectories[vehicle_name] = {
                'target_slot_id': slot_id,
                'max_velocity': final_velocity,
                'min_velocity': final_velocity,
                'previous_velocity':final_velocity,
                'current_difference': diff,
                'accelerate': False,
                'in_slot': True

            }
            print("trajectory is {}".format(self.trajectories[vehicle_name]))

        elif diff > 0:
            a1, a2 = self.max_acceleration, self.max_deceleration
            max_velocity, _ = self.find_max_velocity(0,final_velocity ,a1, a2, diff)
            
            self.trajectories[vehicle_name] = {
                'target_slot_id': slot_id,
                'max_velocity': max_velocity,
                'min_velocity': final_velocity,
                'previous_velocity':0,
                'current_difference': diff,
                'accelerate': True,
                'in_slot': False
            }
            print("trajectory is {}".format(self.trajectories[vehicle_name]))
        else:
            a1, a2 = self.max_deceleration, self.max_acceleration
            min_velocity, _ = self.find_max_velocity(final_velocity, 0,a1, a2, diff)
            self.trajectories[vehicle_name] = {
                'target_slot_id': slot_id,
                'max_velocity': final_velocity,
                'min_velocity': min_velocity,
                'previous_velocity':0,
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

        return v1, t1 + t2

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

        if diff > 0:
            v1 = (-b + math.sqrt(d)) / (2 * a)
        else:
            v1 = (-b - math.sqrt(d)) / (2 * a)

        return v1

    def compute_velocity(self,vehicle_name):
        if self.trajectories[vehicle_name]['in_slot']:
            self.trajectories[vehicle_name]['previous_velocity']=self.trajectories[vehicle_name]['max_velocity']
        if self.trajectories[vehicle_name]['accelerate']:
            if self.trajectories[vehicle_name]['previous_velocity']<self.trajectories[vehicle_name]['max_velocity']:
                self.trajectories[vehicle_name]['previous_velocity']+= self.max_acceleration*self.dt
            else:
                self.trajectories[vehicle_name]['previous_velocity']+= self.max_deceleration*self.dt
        else:
            if self.trajectories[vehicle_name]['previous_velocity']>self.trajectories[vehicle_name]['min_velocity']:
                self.trajectories[vehicle_name]['previous_velocity'] +=self.max_deceleration*self.dt
            else:
                self.trajectories[vehicle_name]['previous_velocity'] +=self.max_acceleration*self.dt

        return self.trajectories[vehicle_name]['previous_velocity']
    

    def parse_data(self, input_string):
        input_string = str(input_string)
        rospy.loginfo("input string is {}".format(input_string))
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

