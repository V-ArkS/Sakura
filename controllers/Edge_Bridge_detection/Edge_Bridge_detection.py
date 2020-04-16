from controller import Robot
from controller import Camera
import numpy as np
import cv2

TIME_STEP = 64
robot = Robot()
# Define motors
motors = []
motorNames = ['left motor', 'right motor']
for i in range(2):
    motors.append(robot.getMotor(motorNames[i]))
    motors[i].setPosition(float('inf'))
    motors[i].setVelocity(0.0)
    motors[i].setAcceleration(25)

camera = Camera('camera')
camera.enable(int(robot.getBasicTimeStep()))
HEIGHT = camera.getHeight()
WIDTH = camera.getWidth()
p0 = WIDTH / 2
HALF = int(WIDTH / 2)

while robot.step(TIME_STEP) != -1:
    leftSpeed = 0.0
    rightSpeed = 0.0
    # get image and process it
    cameraData = camera.getImage()
    position = p0
    position_r = 0
    position_l = 0
    frame = np.zeros((HEIGHT, WIDTH))
    for x in range(0, HEIGHT):
        for y in range(0, WIDTH):
            # red = int(camera.imageGetRed(cameraData, WIDTH, x, y))
            # blue = int(camera.imageGetBlue(cameraData, WIDTH, x, y))
            # green = int(camera.imageGetGreen(cameraData, WIDTH, x, y))
            frame[y][x] = int(camera.imageGetGray(cameraData, WIDTH, x, y))

    # edge_detect Sobel
    x = cv2.Sobel(frame, cv2.CV_16S, 1, 0)
    y = cv2.Sobel(frame, cv2.CV_16S, 0, 1)
    absX = cv2.convertScaleAbs(x)
    absY = cv2.convertScaleAbs(y)
    dst = cv2.addWeighted(absX, 0.5, absY, 0.5, 0)
    # to binary
    ret, binary = cv2.threshold(dst, 180, 255, cv2.THRESH_BINARY)
    # Smooth
    kernel1 = np.ones((20, 20), np.float) / 25
    smooth = cv2.filter2D(binary, -1, kernel1)
    # blur
    blur_im1 = cv2.boxFilter(smooth, -1, (15, 15), normalize=1)

    cv2.imwrite('smooth_blur.jpg', blur_im1)
    # Locate
    axis_l = 0
    axis_r = 0
    num_l = 0
    num_r = 0
    for axis_v in range(int(HEIGHT*0.3), int(HEIGHT*0.6)):
        for axis_h in range(WIDTH*0, HALF):
            if blur_im1[axis_v][axis_h] != 0:
                axis_l = axis_l + axis_h
                num_l = num_l + 1
            if blur_im1[axis_v][axis_h + HALF] != 0:
                axis_r = axis_r + axis_h + HALF
                num_r = num_r + 1
    if num_l:
        position_l = axis_l / num_l + 1
    if num_r:
        position_r = axis_r / num_r + 1
    position = (position_l + position_r)/2
    if abs(position-WIDTH/2) > 5:
        leftSpeed = (position/WIDTH-0.5) * 5
        rightSpeed = (0.5-position/WIDTH) * 5
    else:
        leftSpeed = 3
        rightSpeed = 3
    print(position)

    motors[0].setVelocity(leftSpeed)
    motors[1].setVelocity(rightSpeed)