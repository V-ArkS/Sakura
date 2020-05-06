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
# video = cv2.VideoWriter('edge.mp4', cv2.VideoWriter_fourcc('m', 'p', '4', 'v'), int(camera.getSamplingPeriod()), (camera.getWidth(), camera.getHeight()), 0) #创建视频流对象
print(camera.getWidth(), camera.getHeight(), camera.getSamplingPeriod())
while robot.step(TIME_STEP) != -1:
    leftSpeed = 0.0
    rightSpeed = 0.0
    # get image and process it
    image = camera.getImage()
    leftSum = 0
    rightSum = 0
    cameraData = camera.getImage()
    HEIGHT = camera.getHeight()
    WIDTH = camera.getWidth()
    position = WIDTH / 2
    frame = np.zeros((HEIGHT, WIDTH))
    for x in range(0, WIDTH):
        for y in range(0, HEIGHT):
            gray = int(camera.imageGetGray(cameraData, WIDTH, x, y))
            frame[y][x] = gray

    # edge_detect Sobel
    x = cv2.Sobel(frame, cv2.CV_16S, 1, 0)
    y = cv2.Sobel(frame, cv2.CV_16S, 0, 1)
    absX = cv2.convertScaleAbs(x)
    absY = cv2.convertScaleAbs(y)
    dst = cv2.addWeighted(absX, 0.5, absY, 0.5, 0)
    # to binary
    ret, binary = cv2.threshold(dst, 254, 255, cv2.THRESH_BINARY)
    # Smooth
    kernel1 = np.ones((20, 20), np.float) / 25
    smooth = cv2.filter2D(binary, -1, kernel1)
    # blur
    blur_im1 = cv2.boxFilter(smooth, -1, (15, 15), normalize=1)

    # cv2.imwrite('smooth_blur.jpg', blur_im1)
    # cv2.imwrite('gray.jpg', frame)

    # Locate
    axis = 0
    num = 0
    for axis_v in range(int(HEIGHT*0.7), int(HEIGHT*0.8)):
        for axis_h in range(WIDTH*0, WIDTH):
            if blur_im1[axis_v][axis_h] != 0:
                axis = axis + axis_h
                num = num + 1
    # print(num, axis)
    if num:
        position = axis / num + 1
    # print(position)
    if abs(position-WIDTH/2) > 7:
        leftSpeed = (position/WIDTH) * 3
        rightSpeed = (1-position/WIDTH) * 3
    else:
        leftSpeed = 0.5
        rightSpeed = 0.5

    motors[0].setVelocity(leftSpeed)
    motors[1].setVelocity(rightSpeed)

# video.release()