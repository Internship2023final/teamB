"""my_first_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import math

#wheel radius r =0.0045
#robot radius R=0.027

R=0.5
r =0.1
# Example usage of the rotate function
# create the Robot instance.
robot = Robot()
# max_velocity=6.28
# get the time step of the current world.
timestep=int(robot.getBasicTimeStep())
gps=robot.getDevice('gps')
gps.enable(timestep)
imu=robot.getDevice('imu')
imu.enable(timestep)
timer=0

# - perform simulation steps until Webots is stopping the controller
# x_start,y_start,z_start=gps.getValues()
# print(x)
# print(y)
# print(z

while robot.step(timestep) != -1:

    #move_forward(1)
        #move_forward(50)
    #timer=timer+timestep
    x,y,z=gps.getValues()
    r,p,yaw = imu.getRollPitchYaw()

    print("GPS: ", gps.getValues())
    print("IMU" , imu.getRollPitchYaw())
    # velocity=5
    # time_1=move_forward(0.5)
    # time_2=rotate(math.pi /2)
    # #print(timer, time_1, time_2)
    # if 0<timer<=(time_1):
        # left_motor.setVelocity(velocity)
        # right_motor.setVelocity(velocity)
    # elif time_1<timer<=(time_2+time_1+100):
        # velocity=5
        # left_motor.setVelocity(-velocity)
        # right_motor.setVelocity(velocity)
    # else:
         # timer=0

    # left_motor.setPosition(float('inf'))
    #right_motor.setPosition('inf')
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    #pass

# Enter here exit cleanup code.
