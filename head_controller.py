from controller import Supervisor
from sys import stderr
import tkinter as tk
from tkinter import ttk
import cv2
import numpy as np
import copy
import math
import os
base_dir = os.getcwd()
dll_path = os.path.join(base_dir, "x64")
os.add_dll_directory(dll_path)
import starkit_ik_walk as sk

params = sk.IKWalkParameters()

params.distHipToKnee = 0.093
params.distKneeToAnkle = 0.105
params.distAnkleToGround = 0.032
params.distFeetLateral = 0.092
params.freq = 1.7
params.enabledGain = 0.0
params.supportPhaseRatio = 0.0
params.footYOffset = 0.025
params.stepGain = 0.05
params.riseGain = 0.035
params.turnGain = 0.0
params.lateralGain = 0.0
params.trunkZOffset = 0.02
params.swingGain = 0.02
params.swingRollGain = 0.0
params.swingPhase = 0.25
params.stepUpVel = 4.0
params.stepDownVel = 4.0
params.riseUpVel = 4.0
params.riseDownVel = 4.0
params.swingPause = 0.0
params.swingVel = 4.0
params.trunkXOffset = 0.02
params.trunkYOffset = 0.0
params.trunkPitch = 0.15
params.trunkRoll = 0.0
params.extraLeftX = 0.0
params.extraLeftY = 0.0
params.extraLeftZ = 0.0
params.extraRightX = 0.0
params.extraRightY = 0.0
params.extraRightZ = 0.0
params.extraLeftYaw = 0.0
params.extraLeftPitch = 0.0
params.extraLeftRoll = 0.0
params.extraRightYaw = 0.0
params.extraRightPitch = 0.0
params.extraRightRoll = 0.0

phase = 0.0

labels = {}
button_enabled = None

attribute_ranges = {
    "stepGain": (-0.1, 0.1),
    "lateralGain": (-0.06, 0.06),
    "turnGain": (-0.5, 0.5),
    "freq": (0.1, 5.0),
    "supportPhaseRatio": (0.0, 1.0),
    "footYOffset": (-0.2, 0.2),
    "riseGain": (0.0, 0.1),
    "swingGain": (0.0, 0.1),
    "swingRollGain": (-1.0, 1.0),
    "swingPhase": (0.0, 1.0),
    "swingPause": (0.0, 0.5),
    "swingVel": (0.0, 5.0),
    "trunkXOffset": (-0.2, 0.2),
    "trunkYOffset": (-0.2, 0.2),
    "trunkZOffset": (0.01, 0.2),
    "trunkPitch": (-1.0, 1.0),
    "trunkRoll": (-1.0, 1.0)
}

def update_value(param_name, value):
    global params
    global labels
    print(float(value))
    setattr(params, param_name, float(value))
    if param_name in labels.keys():
        labels[param_name].configure(text=f"{param_name}: {float(value):.3f}")

def create_window_1():
    global params
    global labels
    global button_enabled
    global attribute_ranges
    window = tk.Tk()
    window.title("Control robot")
    
    button_enabled = ttk.Button(window, text="Enable", command=lambda: toggle_enabled())
    button_enabled.pack(padx=10, pady=10)
    
    for attr_name, (min_value, max_value) in attribute_ranges.items():
        if attr_name in ("stepGain", "lateralGain", "turnGain"):
            label = ttk.Label(window, text=f"{attr_name}: {getattr(params, attr_name):.3f}")
            labels[attr_name] = label
            label.pack(pady=5)
        
            trackbar = ttk.Scale(window, from_=min_value, to=max_value, length=200, orient=tk.HORIZONTAL, command=lambda value, param_name=attr_name: update_value(param_name, value))
            trackbar.set(getattr(params, attr_name))
            trackbar.pack(pady=5)
    return window

def create_window_2():
    global params
    global labels
    global attribute_ranges
    window = tk.Tk()
    window.title("Parameter settings")
    
    attribute_names = vars(params).keys()
    
    trackbars_per_row = 4
    current_row = 0
    current_column = 0

    for attr_name, (min_value, max_value) in attribute_ranges.items():
        if attr_name not in ("stepGain", "lateralGain", "turnGain"):
            label = ttk.Label(window, text=f"{attr_name}: {getattr(params, attr_name):.3f}")
            label.grid(row=current_row, column=current_column, padx=5, pady=5)
            labels[attr_name] = label

            trackbar = ttk.Scale(window, from_=min_value, to=max_value, length=200, orient=tk.HORIZONTAL,
                                 command=lambda value, param_name=attr_name: update_value(param_name, value))
            trackbar.set(getattr(params, attr_name))
            trackbar.grid(row=current_row, column=current_column+1, padx=5, pady=5)

            current_column += 2
            if current_column >= trackbars_per_row * 2:
                current_row += 1
                current_column = 0
    return window

def toggle_enabled():
    global params
    global labels
    params.enabledGain = 1.0 if params.enabledGain == 0.0 else 0.0
    button_enabled.configure(text="Disable" if params.enabledGain == 1.0 else "Enable")

w1 = create_window_1()
w1.geometry("+1200+200")
w2 = create_window_2()
w2.geometry("+100+550")

dof_names = [
    'left_elbow',
    'right_elbow',
    'left_hip_yaw', 
    'left_hip_roll', 
    'left_hip_pitch', 
    'left_knee', 
    'left_ankle_pitch', 
    'left_ankle_roll', 
    'right_hip_yaw', 
    'right_hip_roll', 
    'right_hip_pitch', 
    'right_knee', 
    'right_ankle_pitch', 
    'right_ankle_roll']


name_obj = "red player 1"

# Create a Supervisor instance
robot = Supervisor()

ball_node = robot.getFromDef('BALL')

def nothing(val):
    pass
    
timestep = int(robot.getBasicTimeStep())
servos = {}
for name in dof_names:
    servos[name] = robot.getDevice(name)

def send_command(command: sk.IKWalkOutputs):
    for name, motor in servos.items():
        if "elbow" in name:
            motor.setPosition(-2.5)
        else:
            motor.setPosition(getattr(command, name))

camera = robot.getDevice("right_camera")
camera.enable(timestep)

head_yaw = robot.getDevice("head_yaw")
head_pitch = robot.getDevice("head_pitch")

head_yaw_sensor = robot.getDevice("head_yaw_sensor")
head_yaw_sensor.enable(timestep)
head_pitch_sensor = robot.getDevice("head_pitch_sensor")
head_pitch_sensor.enable(timestep)
robot.step(timestep)

cv2.namedWindow("frame")

# numbers
cv2.createTrackbar("lb", "frame",   14, 255, nothing)
cv2.createTrackbar("lg", "frame",  160, 255, nothing)
cv2.createTrackbar("lr", "frame",  180, 255, nothing)
cv2.createTrackbar("hb", "frame",  20, 255, nothing)
cv2.createTrackbar("hg", "frame",  210, 255, nothing)
cv2.createTrackbar("hr", "frame", 255, 255, nothing)
cv2.createTrackbar("area", "frame", 250, 1000, nothing)

img_cy = camera.getHeight()//2
img_cx = camera.getWidth()//2

k=0.001

head_yaw.setPosition(head_yaw_sensor.getValue() + 2*math.pi)

while robot.step(timestep) != -1:

    camera_data = camera.getImage()
    img = np.frombuffer(camera_data, np.uint8).reshape(camera.getHeight(), camera.getWidth(), 4)
    img = img[:, :, :3]
    
    frame_ = copy.deepcopy(img)
    w, h, _ = frame_.shape
    frame = cv2.resize(frame_, (h, w))

    lb = cv2.getTrackbarPos("lb", "frame")
    lg = cv2.getTrackbarPos("lg", "frame")
    lr = cv2.getTrackbarPos("lr", "frame")
    hb = cv2.getTrackbarPos("hb", "frame")
    hg = cv2.getTrackbarPos("hg", "frame")
    hr = cv2.getTrackbarPos("hr", "frame")
    
    frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    mask = cv2.inRange(frame_hsv, (lb, lg, lr), (hb, hg, hr))
    
    th_area = cv2.getTrackbarPos("area", "frame")
    
    connectivity = 4
    output = cv2.connectedComponentsWithStats(mask, connectivity, cv2.CV_32S)
    num_labels = output[0]
    labels = output[1]
    stats = output[2]
    
    objects = []
    
    for i in range(1, num_labels):
        t = stats[i, cv2.CC_STAT_TOP]
        l = stats[i, cv2.CC_STAT_LEFT]
        w = stats[i, cv2.CC_STAT_WIDTH]
        h = stats[i, cv2.CC_STAT_HEIGHT]
        a = stats[i, cv2.CC_STAT_AREA]
        
        if (a < th_area):
            mask[np.where(labels == i)] = 0

        else:
            objects.append(((l, t), (l + w, t + h)))
            obj_x = (objects[0][1][0] + objects[0][0][0])//2
            obj_y = (objects[0][1][1] + objects[0][0][1])//2

            yaw_diff = k*(img_cx - obj_x)
            pitch_diff = k*(img_cy - obj_y)

            head_yaw.setPosition(head_yaw_sensor.getValue() - yaw_diff)
            head_pitch.setPosition(head_pitch_sensor.getValue() - pitch_diff)
            
            w1.update_idletasks()
            w1.update()
            w2.update_idletasks()
            w2.update()
            outputs = sk.IKWalkOutputs()
            if sk.IKWalk.walk(params, timestep / 1000.0, phase, outputs):
                send_command(outputs)
                phase = outputs.phase
            else:
                print(" Inverse Kinematics error. Position not reachable.", file=stderr)
    
    mask_3ch = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    
    for (tl, br) in objects:
        cv2.rectangle(frame, tl, br, (0, 0, 255), 2)
    
    res = np.concatenate((frame, mask_3ch), axis=1)

    cv2.imshow("frame", res)
    cv2.waitKey(timestep)
cv2.destroyAllWindows()