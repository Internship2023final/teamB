from controller import Supervisor
from sys import stderr
import tkinter as tk
from tkinter import ttk
import cv2
import numpy as np
import copy
import math
import os
import json
from scipy.interpolate import interp1d
#import kicking_the_ball as kick
base_dir = os.getcwd()
dll_path = os.path.join(base_dir, "x64")
os.add_dll_directory(dll_path)
import starkit_ik_walk as sk
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
    
params = sk.IKWalkParameters()

params.distHipToKnee = 0.093
params.distKneeToAnkle = 0.105
params.distAnkleToGround = 0.032
params.distFeetLateral = 0.092
params.freq = 1.7
params.enabledGain = 0.0
params.supportPhaseRatio = 0.0
params.footYOffset = 0.025
params.stepGain = 0.0
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
params.swingVel = 4
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

def calculate_distance(p1, p2):
    return math.sqrt(sum((a - b) ** 2 for a, b in zip(p1, p2)))

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


#ball_node = robot.getFromDef('BALL')

    
timestep = int(robot.getBasicTimeStep())
imu=robot.getDevice('imu')
imu.enable(timestep)
gps=robot.getDevice('gps')
gps.enable(timestep)


servos = {}

for name in dof_names:
    servos[name] = robot.getDevice(name)



def send_command(command: sk.IKWalkOutputs):
    for name, motor in servos.items():
        if "elbow" in name:
            motor.setPosition(-2.5)
        else:
            motor.setPosition(getattr(command, name))
            
            
 
def send_commands(commands):
    for joint_name, value in commands.items():
        if joint_name not in ("over", "remap"):
            servos_two[joint_name].setPosition(deg2rad(value))




camera = robot.getDevice("right_camera")
camera.enable(timestep)



head_yaw = robot.getDevice("head_yaw")
head_pitch = robot.getDevice("head_pitch")

head_yaw_sensor = robot.getDevice("head_yaw_sensor")
head_yaw_sensor.enable(timestep)
head_pitch_sensor = robot.getDevice("head_pitch_sensor")
head_pitch_sensor.enable(timestep)




img_cy = camera.getHeight()//2
img_cx = camera.getWidth()//2

k=0.001

stop_moving=True
searching_angle=0



while robot.step(timestep) != -1:
 #       print(gps.getCoordinateSystem())
        # print(right_hip_yaw_sensor.getValue())
        #print("IMU_init" , imu.getRollPitchYaw())
        if (stop_moving):# keep turning.
            # #head_yaw.setPosition(head_yaw_sensor.getValue() + 0.3)
            outputs = sk.IKWalkOutputs()
            params.turnGain = 0.2
            #print("IMU" , imu.getRollPitchYaw())
            params.enabledGain = 1.0
            
            if sk.IKWalk.walk(params, timestep / 1000.0, phase, outputs):
                    #print("im here2")
                    #print(outputs)
                    
                    send_command(outputs)
                    phase = outputs.phase
            else:
                print(" Inverse Kinematics error. Position not reachable.", file=stderr)

                    
        camera_data = camera.getImage()
        
        img = np.frombuffer(camera_data, np.uint8).reshape(camera.getHeight(), camera.getWidth(), 4)
        img = img[:, :, :3]
        

        center_y=camera.getHeight()//2
        center_x=camera.getWidth()//2
        frame_ = copy.deepcopy(img)
        #cv2.imshow("frame_", frame_)

        w, h, _ = frame_.shape
        frame = cv2.resize(frame_, (h, w))
        #cv2.imshow("frame", frame)

        frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        #cv2.imshow("frame_hsv", frame_hsv)
        #frame_hsv=frame
        mask = cv2.inRange(frame_hsv, (110, 170, 0), (130, 255, 255))
        #cv2.imshow("mask", mask)
        # #Creating kernel
        kernel = np.ones((3, 3), np.uint8)
      
    # #Using cv2.erode() method 
        image = cv2.erode(mask, kernel) 
      
    # #Displaying the image 
        # cv2.imshow("eroded_mask", image)
        mask=image
        
        #cv2.imshow("mask", mask)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
        objects = []
        th_area=5
        for contour in contours:
            # #Filter out small contours based on the area
            area = cv2.contourArea(contour)
            if area >= th_area:
               
                x, y, w, h = cv2.boundingRect(contour)
                objects.append(((x, y), (x + w, y + h)))

        mask_3ch = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        if objects == [] : 
                  outputs = sk.IKWalkOutputs()
                  params.turnGain = 0.2
                  params.enabledGain = 1.0
                  if sk.IKWalk.walk(params, timestep / 1000.0, phase, outputs):
                    #print("im here2")
                    #print(outputs)
                    
                        send_command(outputs)
                        phase = outputs.phase
                  else:
                        print(" Inverse Kinematics error. Position not reachable.", file=stderr)
        # else:    
        for (tl, br) in objects:

            # Print the position
            #print("Ball position:", position)
            cv2.rectangle(frame, tl, br, (0, 0, 255), 2) 
            #print(tl)
            #print(br)
            
            #add centering
            before_value=head_yaw_sensor.getValue()
            obj_x=br[0]
            obj_y=br[1]
            yaw_diff = k*(img_cx - obj_x)
            pitch_diff = k*(img_cy - obj_y)
            desired_yaw=head_yaw_sensor.getValue() - yaw_diff
            head_yaw.setPosition(head_yaw_sensor.getValue() - yaw_diff)
            head_pitch.setPosition(head_pitch_sensor.getValue() - pitch_diff)
            params.turnGain = 0.1
            #params.stepGain = 0.1
            params.enabledGain = 1.0
            #phase=0
            outputs = sk.IKWalkOutputs()
            if (head_yaw_sensor.getValue()>=0.2 or  head_yaw_sensor.getValue() <=-0.2):
                    if sk.IKWalk.walk(params, timestep / 1000.0, phase, outputs):
                                                        print("rotation")
                                                        print("head_yaw",head_yaw_sensor.getValue())
            
                                                        send_command(outputs)
                                                        #once_flag=once_flag+1
                                                        #print("counter",once_flag)
                                                        phase = outputs.phase
                    else:
                                                    print(" Inverse Kinematics error. Position not reachable.", file=stderr)

            elif (head_yaw_sensor.getValue()<=0.2 and head_yaw_sensor.getValue() >=-0.2):
                    
                print("head_yaw",head_yaw_sensor.getValue())
                params.turnGain = 0.0
                params.stepGain = 0.05
                params.enabledGain = 1.0
            #phase=0
                outputs = sk.IKWalkOutputs()
                if sk.IKWalk.walk(params, timestep / 1000.0, phase, outputs):
                                                #print("walking")
                                                
                                                send_command(outputs)
                                                #once_flag=once_flag+1
                                                #print("counter",once_flag)
                                                phase = outputs.phase
                else:
                                            print(" Inverse Kinematics error. Position not reachable.", file=stderr)

            stop_moving=False
            robot_position=gps.getValues()
            ball_node = robot.getFromDef("Ball")
            ball_position = ball_node.getPosition()
            print("dist",calculate_distance(robot_position, ball_position))
            dist=calculate_distance(robot_position, ball_position)
            if dist <=0.5:
                params.turnGain = 0.0
                params.stepGain = 0.00
                params.enabledGain = 0.0
            #phase=0
                outputs = sk.IKWalkOutputs()
                if sk.IKWalk.walk(params, timestep / 1000.0, phase, outputs):
                                                print("STOP")
                                                
                                                send_command(outputs)
                                                #once_flag=once_flag+1
                                                #print("counter",once_flag)
                                                phase = outputs.phase
                else:
                                            print(" Inverse Kinematics error. Position not reachable.", file=stderr)
                # motion = read_json("kicking_the_ball.json")
                # motion_timer = 0
                # motion_trajectory = interpolate(motion)
 
                # time_factor = motion_trajectory["remap"](motion_timer)
                # motion_timer += time_factor * timestep / 1000 
                # servos_two={}
                # for joint_name in motion_trajectory.keys():
                    # if joint_name not in ("over", "remap"):
                        # servos_two[joint_name] = robot.getDevice(joint_name)
    
                # if not motion_trajectory["over"](motion_timer):
                         # commands = {}
                         # for joint_name, trajectory in motion_trajectory.items(): 
                                # target = trajectory(motion_timer)
                                # commands[joint_name] = target
                # send_commands(commands)
            #print(head_yaw_sensor.getValue() - yaw_diff)
            #print("head_yaw_sensor.getValue()",head_yaw_sensor.getValue())
            # current_yaw=head_yaw_sensor.getValue()
            #if (current_yaw>=(desired_yaw-0.1) or current_yaw<=(desired_yaw+0.1) ):
                    # while(current_yaw>=0.02 or current_yaw<=0.02 ):
                            # print(current_yaw)
                            # params.turnGain = 0.1
                            # params.enabledGain = 1.0
                            # phase=0
                            # outputs = sk.IKWalkOutputs()
                            # if sk.IKWalk.walk(params, timestep / 1000.0, phase, outputs):
                                                # print("im rotating")
                                                # #print(outputs)
                                                
                                                # send_command(outputs)
                                                # #once_flag=once_flag+1
                                                # #print("counter",once_flag)
                                                # phase = outputs.phase
                            # else:
                                            # print(" Inverse Kinematics error. Position not reachable.", file=stderr)
                            # current_yaw=head_yaw_sensor.getValue()
            # elif (head_yaw_sensor.getValue() > 0.02 or head_yaw_sensor.getValue() <-0.02):
            # params.turnGain = 0.0
            # params.stepGain = 0.05
            # params.enabledGain = 1.0
            # phase=0
            # outputs = sk.IKWalkOutputs()
            # if sk.IKWalk.walk(params, timestep / 1000.0, phase, outputs):
                                # #print("im walking")
                                # #print(outputs)
                                
                                # send_command(outputs)
                                # #once_flag=once_flag+1
                                # #print("counter",once_flag)
                                # phase = outputs.phase
            # else:
                                # print(" Inverse Kinematics error. Position not reachable.", file=stderr)
            
            # target_distance=0.4
            # robot_position=gps.getValues()
            # ball_node = robot.getFromDef("Ball")
            # ball_position = ball_node.getPosition()
            # if(calculate_distance(robot_position, ball_position) <= target_distance):
                    # params.turnGain = 0.0
                    # params.stepGain = 0.0
                    # params.enabledGain = 0.0
                    # if sk.IKWalk.walk(params, timestep / 1000.0, phase, outputs):
                                        # print("im stopping")
                                        # #print(outputs)
                                        
                                        # send_command(outputs)
                                        # #once_flag=once_flag+1
                                        # #print("counter",once_flag)
                                        # phase = outputs.phase
                    # else:
                                    # print(" Inverse Kinematics error. Position not reachable.", file=stderr)
                #params.enabledGain = 1.0
           # print(head_yaw_sensor.getValue() )
            # if (head_yaw_sensor.getValue() <0.02 and head_yaw_sensor.getValue() >-0.02):
                            # params.turnGain = 0.0
                            # params.stepGain = 0.08
                            # params.enabledGain = 1.0
            
                            # if sk.IKWalk.walk(params, timestep / 1000.0, phase, outputs):
                                                # print("im here2")
                                                # #print(outputs)
                                                
                                                # send_command(outputs)
                                                # #once_flag=once_flag+1
                                                # #print("counter",once_flag)
                                                # phase = outputs.phase
                            # else:
                                            # print(" Inverse Kinematics error. Position not reachable.", file=stderr)
            # target=0.55
            # robot_position=gps.getValues()
            # print("dist",calculate_distance(robot_position, ball_position))
            
            # if(calculate_distance(robot_position, ball_position))<=target:
                            # normalize_pose()
                            # params.turnGain = 0.0
                            # params.stepGain = 0.0
                            # params.enabledGain = 0.0
            
                            # if sk.IKWalk.walk(params, timestep / 1000.0, phase, outputs):
                                                # #print("im here2")
                                                # #print(outputs)
                                                
                                                # send_command(outputs)
                                                # #once_flag=once_flag+1
                                                # #print("counter",once_flag)
                                                # phase = outputs.phase
                            # else:
                                            # print(" Inverse Kinematics error. Position not reachable.", file=stderr)
             
        res = np.concatenate((frame, mask_3ch), axis=1)
    
        cv2.imshow("frame", res)
        cv2.waitKey(timestep)
cv2.destroyAllWindows()