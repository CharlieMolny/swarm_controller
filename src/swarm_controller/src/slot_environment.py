import math

class Slot:
    def __init__(self, id, lane, position={}, reserved=False, filled=False):
        self.id = id
        self.position = position
        self.reserved = reserved
        self.filled = filled
        self.lane = lane
        self.end_of_road = False

    def to_dict(self):
        return {
            'id': self.id,
            'position': self.position,
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
            position=data['position'],
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
        self.generate_slots()

    def generate_slots(self):
        lane_spacing = self.lane_width  

        for lane in range(self.num_lanes):
            y_position = lane * lane_spacing

            total_space = self.road_length - (self.num_slots_per_lane * self.slot_length)
            if total_space < 0:
                raise ValueError("Not enough road length to fit the slots with the given slot length.")
            
            if self.num_slots_per_lane > 1:
                slot_spacing = total_space / (self.num_slots_per_lane - 1)
            else:
                slot_spacing = 0 

            for i in range(self.num_slots_per_lane):
                x_position = i * (self.slot_length + slot_spacing)
                
                new_slot = Slot(id=self.slot_count, lane=lane, position={'x': x_position, 'y': y_position})
                self.slots[self.slot_count] = new_slot.to_dict()
                self.slot_count += 1

        return self.slots

    def update_slots(self):
        dt = 1 / self.frequency
        slots_to_remove = []

        for slot_id, slot in list(self.slots.items()):
            if slot['position']['x'] < self.road_length:
                slot['position']['x'] += dt * self.lane_velocities[slot['lane']]
            else:
                slots_to_remove.append(slot_id)

        for slot_id in slots_to_remove:
            lane = self.slots[slot_id]['lane']
            y_position = self.slots[slot_id]['position']['y']

            del self.slots[slot_id]

            new_slot_id = max(self.slots.keys()) + 1 if self.slots else 0

            new_slot = {
                'id': new_slot_id,
                'position': {'x': 0, 'y': y_position},
                'lane': lane,
                'reserved': False,
                'filled': False,
                'vel': self.lane_velocities[lane]
            }

            self.slots[new_slot_id] = new_slot

    def get_near_slot_same_lane(self, ego_position):
        minimum_distance = float('inf')
        near_slot = None
        diff = None

        for id, slot in self.slots.items():
            distance = math.sqrt((ego_position['x'] - slot['position']['x']) ** 2)
            if distance < minimum_distance and not slot['reserved'] and not slot['filled']:
                if self.is_path_clear(ego_position, slot['position']):
                    minimum_distance = distance
                    diff = slot['position']['x'] - ego_position['x']
                    near_slot = slot

        if near_slot is not None:
            return diff, near_slot['id']
        else:
            return None, None

    def is_path_clear(self, ego_position, candidate_slot_position):
        start_x = min(ego_position['x'], candidate_slot_position['x'])
        end_x = max(ego_position['x'], candidate_slot_position['x'])

        for slot in self.slots.values():
            if start_x < slot['position']['x'] < end_x and not slot['reserved'] and not slot['filled']:
                return False
        return True

    def find_new_slot(self, slot):
        near_index, near_distance, near_slot = self.get_near_slot_neighboring_lane(slot)
        if near_index is not None:
            self.slots[near_index]['reserved'] = True
            self.distance_count[near_index] = near_distance
        else:
            raise Exception("No available slot found")

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




class TrajectoryPlanner:
    def __init__(self, max_acceleration, max_deceleration, dt):
        self.max_acceleration = max_acceleration
        self.max_deceleration = max_deceleration
        self.dt = dt

    def compute_velocity(self, previous_vel, target_vel, accelerate):
        if accelerate:
            velocity = previous_vel + self.max_acceleration * self.dt
            return min(target_vel, velocity)
        else:
            velocity = previous_vel + self.max_deceleration * self.dt
            return max(target_vel, velocity)