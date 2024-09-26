
def find_trajectory(self, msg, slot_id, diff):
    lane = msg.lane
    v0 = self.slot_env.lane_velocities[lane]
    if diff > 0:
        a1, a2 = self.max_acceleration, self.max_deceleration
        max_velocity, _ = self.find_max_velocity(v0, a1, a2, diff)
        self.trajectories[msg.vehicle_id] = {
            'target_slot_id': slot_id,
            'max_velocity': max_velocity,
            'min_velocity': v0,
            'accelerate': True
        }
    else:
        a1, a2 = self.max_deceleration, self.max_acceleration
        min_velocity, _ = self.find_max_velocity(v0, a1, a2, diff)
        self.trajectories[msg.vehicle_id] = {
            'target_slot_id': slot_id,
            'max_velocity': min_velocity,
            'min_velocity': v0,
            'accelerate': False
        }

def correct_bot(self, msg, diff):
    self.slot_env.slots[msg.slot_id]['filled'] = False
    if msg.vehicle_id not in self.trajectories:
        self.find_trajectory(msg, msg.slot_id, diff)

def assign_bot_to_slot(self, msg):
    diff, near_slot_id = self.get_near_slot_same_lane(msg.position)
    self.slot_env.slots[near_slot_id]['reserved'] = True
    self.find_trajectory(msg, near_slot_id, diff)

def bot_approaching_slot(self, msg):
    traj = self.trajectories[msg.vehicle_id]
    previous_vel = msg.previous_vel

    velocity = self.trajectory_planner.compute_velocity(
        previous_vel, traj['max_velocity'] if traj['accelerate'] else traj['min_velocity'], traj['accelerate']
    )

    if traj['accelerate'] and previous_vel >= traj['max_velocity']:
        traj['accelerate'] = False
    elif not traj['accelerate'] and previous_vel <= traj['min_velocity']:
        traj['accelerate'] = True

    self.trajectories[msg.vehicle_id] = traj
    return velocity
    