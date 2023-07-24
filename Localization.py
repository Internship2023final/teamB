"""my_first_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import math

#wheel radius r =0.0045
#robot radius R=0.027
R=0.5
r =0.1

def move_forward(distance):
    # Calculate the number of steps based on the desired distance
    circumference= 2 *math.pi * r
    max_velocity=5
    time = distance / (circumference * max_velocity)
    time=time*1000
    timestep = int(robot.getBasicTimeStep())
    time= time // timestep
    return time*timestep

def rotate(angle):
    max_velocity=5
    w_1=max_velocity
    w_2=0
    time=(R*angle)/(w_1-w_2)
    time=time*1000
    timestep = int(robot.getBasicTimeStep())
    time= time//timestep
    return time*timestep




# Example usage of the rotate function
# create the Robot instance.
robot = Robot()
# max_velocity=6.28
# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())
left_motor=robot.getDevice('left wheel motor')
right_motor=robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
gps=robot.getDevice('gps')
gps.enable(timestep)
imu=robot.getDevice('imu')
imu.enable(timestep)
timer=0


# - perform simulation steps until Webots is stopping the controller
# x_start,y_start,z_start=gps.getValues()
# print(x_start)
# print(y_start)
# print(z_start)
goal_distance=2
first_time=1
while robot.step(timestep) != -1:
    if (first_time):
        x_start,y_start,z_start=gps.getValues()
        first_time=0
    #move_forward(1)
        #move_forward(50)
    #timer=timer+timestep
    x,y,z=gps.getValues()
    value_1=x-x_start

    value_2=y-y_start
    current_distance=math.sqrt((value_1**2)+math.sqrt(value_2**2))
    if current_distance <goal_distance:
        left_motor.setVelocity(5)
        right_motor.setVelocity(5)
    else:
        right_motor.setVelocity(0)
        left_motor.setVelocity(0)
    #print("GPS: ", gps.getValues())
    #print("IMU" , imu.getRollPitchYaw())
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
