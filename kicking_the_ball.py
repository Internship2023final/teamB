"""keypoint controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
import math
import json
from scipy.interpolate import interp1d
from controller import Robot

dofs = [
    "head_pitch",
    "head_yaw",
    "left_shoulder_pitch",
    "left_shoulder_roll",
    "right_shoulder_pitch",
    "right_shoulder_roll",
    "left_elbow",
    "right_elbow",
    "left_hip_pitch",
    "left_hip_roll",
    "left_hip_yaw",
    "right_hip_pitch",
    "right_hip_roll",
    "right_hip_yaw",
    "left_knee",
    "right_knee",
    "left_ankle_pitch",
    "left_ankle_roll",
    "right_ankle_pitch",
    "right_ankle_roll"
]

def deg2rad(deg):
    return deg / 180 * math.pi
def rad2deg(rad):
    return rad / math.pi * 180


def read_json(file_path):
    with open(file_path, 'r') as file:
        data = json.load(file)
    refactored_data = {}
    for joint_name in data.keys():
        # if joint_name not in ("over", "remap", "head_yaw", "head_pitch"):
            # if "left" not in joint_name:
                # refactored_data["left_" + joint_name] = data[joint_name]
            # if "right" not in joint_name:
                # refactored_data["right_" + joint_name] = data[joint_name]
        # else:
            refactored_data[joint_name] = data[joint_name]
    print(refactored_data)

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


# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

motion = read_json("kicking_the_ball.json")

motion_timer = 0
motion_trajectory = interpolate(motion)

servos = {}

for joint_name in motion_trajectory.keys():
    if joint_name not in ("over", "remap"):
        servos[joint_name] = robot.getDevice(joint_name)
    
while robot.step(timestep) != -1:
    time_factor = motion_trajectory["remap"](motion_timer)
    motion_timer += time_factor * timestep / 1000 
    if not motion_trajectory["over"](motion_timer):
        commands = {}
        for joint_name, trajectory in motion_trajectory.items(): 
            target = trajectory(motion_timer)
            commands[joint_name] = target
    send_commands(commands)
   