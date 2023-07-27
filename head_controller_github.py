"""head_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import cv2
import numpy as np
import copy

# create the Robot instance.
robot = Robot()

def nothing(val):
    pass
    
# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

camera = robot.getDevice("right_camera")
camera.enable(timestep)

cv2.namedWindow("frame")

cv2.createTrackbar("lb", "frame",   59, 255, nothing)
cv2.createTrackbar("lg", "frame",  30, 255, nothing)
cv2.createTrackbar("lr", "frame",  49, 255, nothing)
cv2.createTrackbar("hb", "frame",  99, 255, nothing)
cv2.createTrackbar("hg", "frame",  246, 255, nothing)
cv2.createTrackbar("hr", "frame", 255, 255, nothing)
#cv2.createTrackbar("ksz", "frame", 0, 20, nothing)
cv2.createTrackbar("area", "frame", 234, 1000, nothing)

head_yaw=robot.getDevice("head_yaw")
head_pitch=robot.getDevice("head_pitch")

head_yaw_sensor=robot.getDevice("head_yaw_sensor")
head_yaw_sensor.enable(timestep)
head_pitch_sensor=robot.getDevice("head_pitch_sensor")
head_pitch_sensor.enable(timestep)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    camera_data = camera.getImage()
    img = np.frombuffer(camera_data, np.uint8).reshape(camera.getHeight(),
                                                         camera.getWidth(),
                                                        4)
    center_y=camera.getHeight()//2
    center_x=camera.getWidth()//2

    img = img[:, :, :3]
    #cv2.imshow("frame", img)
    
    frame_ = copy.deepcopy(img)
    
    w, h, _ = frame_.shape
   
    frame = cv2.resize(frame_, (h, w))

    #ksz = cv2.getTrackbarPos("ksz", "frame") + 1
    #frame = cv2.blur(frame, (ksz, ksz))
    
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
    
    mask_3ch = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    # head_yaw_pos=head_yaw_sensor
    
    for (tl, br) in objects:
        cv2.rectangle(frame, tl, br, (0, 0, 255), 2)
        
    ball_center_y=(t- (0.5*h))
    ball_center_x=(l+(0.5*w))

    k=0.001
    diff=center_x-ball_center_x
    diff2=center_y-ball_center_y
    print("diff1",diff)
    head_yaw.setPosition(head_yaw_sensor.getValue()-k*diff)
    head_pitch.setPosition((head_pitch_sensor.getValue()-(k*diff2)))
    
    print("diff2",diff)
    res = np.concatenate((frame, mask_3ch), axis=1)
    
    cv2.imshow("frame", res)
    cv2.waitKey(timestep)
cv2.destroyAllWindows()
# Enter here exit cleanup code.