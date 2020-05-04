"""color_detect controller."""

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
print(camera.getWidth(), camera.getHeight(), camera.getSamplingPeriod())
# Set target color
colors = {'red': 0, 'yellow': 1, 'purple': 2}  # red, yellow, purple
flag = colors['purple']
while robot.step(TIME_STEP) != -1:
    leftSpeed = 2.0
    rightSpeed = 2.0
    position = p0
    position_r = WIDTH
    position_l = 0
    # get image and process it
    image = camera.getImage()
    cameraData = camera.getImage()
    frame = np.zeros((HEIGHT, WIDTH))
    for y in range(0, HEIGHT):
        for x in range(0, WIDTH):
            gray = int(camera.imageGetGray(cameraData, WIDTH, x, y))
            # color classification
            if flag == 0:
                if 80 < gray < 100:
                    frame[y][x] = 255
                else:
                    pass
            elif flag == 1:
                if 150 < gray < 160:
                    frame[y][x] = 255
                else:
                    pass
            elif flag == 2:
                if 130 < gray < 140:
                    frame[y][x] = 255
                else:
                    pass

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


    cv2.imwrite('smooth_blur.jpg', blur_im1)
    cv2.imwrite('gray.jpg', frame)
    cv2.imwrite('frame.jpg', frame)
    # video.write(blur_im1)

    # Locate
    axis = 0
    num = 0
    for axis_v in range(int(HEIGHT * 0.6), int(HEIGHT * 0.7)):
        for axis_h in range(WIDTH * 0, WIDTH):
            if blur_im1[axis_v][axis_h] != 0:
                axis = axis + axis_h
                num = num + 1
    if num:
        position = axis / num + 1
    # Steering
    if abs(position - WIDTH / 2) > 3:
        leftSpeed = (position / WIDTH-0.5) * 20
        rightSpeed = (0.5 - position / WIDTH) * 20
    else:
        leftSpeed = 2.0
        rightSpeed = 2.0

    print(position)
    print(leftSpeed)
    print(rightSpeed)

    motors[0].setVelocity(leftSpeed)
    motors[1].setVelocity(rightSpeed)

# video.release()