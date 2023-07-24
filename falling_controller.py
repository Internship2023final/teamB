import math
import json
from scipy.interpolate import interp1d
from controller import Robot

def deg2rad(deg):
    return deg / 180 * math.pi
def rad2deg(rad):
    return rad / math.pi * 180

def read_json(file_path):
    with open(file_path, 'r') as file:
        data = json.load(file)
    refactored_data = {}
    for joint_name in data.keys():
        if joint_name not in ("over", "remap", "head_yaw", "head_pitch"):
            if "left" not in joint_name:
                refactored_data["left_" + joint_name] = data[joint_name]
            if "right" not in joint_name:
                refactored_data["right_" + joint_name] = data[joint_name]
        else:
            refactored_data[joint_name] = data[joint_name]
    return refactored_data
    
def interpolate(motion):
    motion_trajectory = {}
    for joint in motion.items():
        joint_name = joint[0] 
        x = []
        y = []
        points = joint[1]
        for point in points:
            x.append(point[0])
            y.append(point[1])
        if joint_name not in ("over", "remap"):
            joint_trajectory = interp1d(x, y, kind='linear', fill_value='extrapolate')
        elif joint_name == "remap":
            joint_trajectory = interp1d(x, y, kind='next', fill_value='extrapolate')
        else:
            joint_trajectory = interp1d(x, y, kind='zero', fill_value='extrapolate')
        motion_trajectory[joint_name] = joint_trajectory       
    return motion_trajectory
    
def send_commands(commands):
    for joint_name, value in commands.items():
        if joint_name not in ("over", "remap"):
            servos[joint_name].setPosition(deg2rad(value))
            
def run_standup_front():
    global standup_front_timer
    time_factor = standup_front_trajectory["remap"](standup_front_timer)
    standup_front_timer += time_factor * timestep / 1000 
    if not standup_front_trajectory["over"](standup_front_timer):
        commands = {}
        for joint_name, trajectory in standup_front_trajectory.items(): 
            target = trajectory(standup_front_timer)
            commands[joint_name] = target
    send_commands(commands)
    pass

def run_standup_back():
    global standup_back_timer
    time_factor = standup_back_trajectory["remap"](standup_back_timer)
    standup_back_timer += time_factor * timestep / 1000 
    if not standup_back_trajectory["over"](standup_back_timer):
        commands = {}
        for joint_name, trajectory in standup_back_trajectory.items(): 
            target = trajectory(standup_back_timer)
            commands[joint_name] = target
    send_commands(commands)
    pass

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

imu = robot.getDevice("imu")
imu.enable(timestep)

standup_front_motion = read_json("standup_front.json")
standup_front_timer = 0
standup_front_trajectory = interpolate(standup_front_motion)

standup_back_motion = read_json("standup_back.json")
standup_back_timer = 0
standup_back_trajectory = interpolate(standup_back_motion)

servos = {}

for joint_name in standup_front_trajectory.keys():
    if joint_name not in ("over", "remap"):
        servos[joint_name] = robot.getDevice(joint_name)

for joint_name in standup_back_trajectory.keys():
    if joint_name not in ("over", "remap"):
        servos[joint_name] = robot.getDevice(joint_name)
        
state = "ready"
standup_is_over = True
    
# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    roll, pitch, yaw = imu.getRollPitchYaw()
    #print(pitch)        #+ on back, and- on front
    if pitch < -1 and standup_is_over:
        #print("The robot is falling on the front")
        state = "front_fall"
        standup_is_over = False
        standup_back_timer = 0
    elif pitch > 1 and standup_is_over:
        #print("The robot is falling on the back")
        state = "back_fall"
        standup_is_over = False
        standup_front_timer = 0
    elif abs(roll) > 1 and standup_is_over:
        state = "side_fall"
        standup_is_over = False

    elif abs(pitch) < 0.3 and standup_is_over:
        state == "ready"
        standup_front_timer = 0
        standup_back_timer = 0
        
    print("The robot state is: ", state)
        
    if state == "front_fall":
        run_standup_front()
        if standup_front_trajectory["over"](standup_front_timer):
            standup_is_over = True
    elif state == "side_fall":
        run_standup_back()
        if standup_back_trajectory["over"](standup_back_timer):
            standup_is_over = True
    elif state == "back_fall":
        run_standup_back()
        if standup_back_trajectory["over"](standup_back_timer):
            standup_is_over = True
    else:
        pass
        
        
        