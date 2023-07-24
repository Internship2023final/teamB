from controller import Robot
import cv2
import numpy as np
import copy
import math

robot = Robot()

def nothing(val):
    pass
    
timestep = int(robot.getBasicTimeStep())

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

cv2.createTrackbar("lb", "frame",   0, 255, nothing)
cv2.createTrackbar("lg", "frame",  160, 255, nothing)
cv2.createTrackbar("lr", "frame",  160, 255, nothing)
cv2.createTrackbar("hb", "frame",  40, 255, nothing)
cv2.createTrackbar("hg", "frame",  210, 255, nothing)
cv2.createTrackbar("hr", "frame", 190, 255, nothing)
cv2.createTrackbar("area", "frame", 100, 1000, nothing)

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
    
    mask_3ch = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    
    for (tl, br) in objects:
        cv2.rectangle(frame, tl, br, (0, 0, 255), 2)
    
    res = np.concatenate((frame, mask_3ch), axis=1)
    
    cv2.imshow("frame", res)
    cv2.waitKey(timestep)
cv2.destroyAllWindows()