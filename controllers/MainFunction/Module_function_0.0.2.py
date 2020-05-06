from controller import Robot
from controller import Camera
import numpy as np
import cv2

TIME_STEP = 64
robot = Robot()

motors = []
motorNames = ['left motor', 'right motor']
for p in range(2):
    motors.append(robot.getMotor(motorNames[p]))

camera = Camera('camera')
camera.enable(int(robot.getBasicTimeStep()))

image = camera.getImage()
cameraData = camera.getImage()
HEIGHT = camera.getHeight()
WIDTH = camera.getWidth()
p0 = WIDTH / 2

leftSpeed = 0.0
rightSpeed = 0.0
leftSum = 0
rightSum = 0

HALF = int(WIDTH / 2)
frame = np.zeros((HEIGHT, WIDTH))
blur_im = np.zeros((HEIGHT, WIDTH))
position = WIDTH / 2


class Sakura:
    @staticmethod
    # in this section variables which are possibly changed will be initialized
    def sakura_initialize():
        global leftSpeed
        global rightSpeed
        global leftSum
        global rightSum
        global frame
        global blur_im
        global position

        leftSpeed = 0.0
        rightSpeed = 0.0
        leftSum = 0
        rightSum = 0
        position = WIDTH / 2
        frame = np.zeros((HEIGHT, WIDTH))
        blur_im = np.zeros((HEIGHT, WIDTH))


    @staticmethod
    def motor_initialize():
        global motors
        global motorNames
        for i in range(2):
            motors[i].setPosition(float('inf'))
            motors[i].setVelocity(0.0)
            motors[i].setAcceleration(25)

    @staticmethod
    def get_frame(type_select=0, flag=-1):
        global frame
        # get image and process it
        frame = np.zeros((HEIGHT, WIDTH))
        if type_select == 0:
            for x in range(0, WIDTH):
                for y in range(0, HEIGHT):
                    gray = int(camera.imageGetGray(cameraData, WIDTH, x, y))
                    frame[y][x] = gray
        elif type_select == 1:
            for x in range(0, WIDTH):
                for y in range(0, HEIGHT):
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
                    else:
                        print('INPUT ERROR!:get_frame:flag')
        else:
            print('INPUT ERROR!:get_frame:type_select')

    @staticmethod
    def edge_detect(threshold=254, maxVal=255, kernel_1=20, kernel_2=15):
        global blur_im
        # edge_detect Sobel
        x = cv2.Sobel(frame, cv2.CV_16S, 1, 0)
        y = cv2.Sobel(frame, cv2.CV_16S, 0, 1)
        absx = cv2.convertScaleAbs(x)
        absy = cv2.convertScaleAbs(y)
        dst = cv2.addWeighted(absx, 0.5, absy, 0.5, 0)
        # to binary
        ret, binary = cv2.threshold(dst, threshold, maxVal, cv2.THRESH_BINARY)
        # Smooth
        kernel1 = np.ones((kernel_1, kernel_1), np.float) / 25
        smooth = cv2.filter2D(binary, -1, kernel1)
        # blur
        blur_im = cv2.boxFilter(smooth, -1, (kernel_2, kernel_2), normalize=1)

    @staticmethod
    def positioning(type_select=0, height_per_1=0.7, height_per_2=0.8):
        global position
        axis = 0
        num = 0
        position = 0
        axis_l = 0
        axis_r = 0
        num_l = 0
        num_r = 0
        position_l = 0
        position_r = WIDTH - 1
        if type_select == 0:
            for axis_v in range(int(HEIGHT * height_per_1), int(HEIGHT * height_per_2)):
                for axis_h in range(WIDTH * 0, WIDTH):
                    if blur_im[axis_v][axis_h] != 0:
                        axis = axis + axis_h
                        num = num + 1
            if num:
                position = axis / num + 1

        elif type_select == 1:
            for axis_v in range(int(HEIGHT * height_per_1), int(HEIGHT * height_per_2)):
                for axis_h in range(WIDTH * 0, HALF):
                    if blur_im[axis_v][axis_h] != 0:
                        axis_l = axis_l + axis_h
                        num_l = num_l + 1
                    if blur_im[axis_v][axis_h + HALF] != 0:
                        axis_r = axis_r + axis_h + HALF
                        num_r = num_r + 1
            if num_l:
                position_l = axis_l / num_l + 1
            if num_r:
                position_r = axis_r / num_r + 1
            position = (position_l + position_r) / 2

        else:
            print('INPUT ERROR!:positioning:type_select')

    @staticmethod
    def steering(type_select=0, rectify_pixel=0, base_speed=3.0, straight_speed=2.0):
        global leftSpeed
        global rightSpeed
        global position

        if type_select == 0:
            if abs(position - WIDTH / 2) > rectify_pixel:
                leftSpeed = (position / WIDTH) * base_speed
                rightSpeed = (1 - position / WIDTH) * base_speed
            else:
                leftSpeed = straight_speed
                rightSpeed = straight_speed
        elif type_select == 1:
            if abs(position - WIDTH / 2) > rectify_pixel:
                leftSpeed = (position / WIDTH - 0.5) * base_speed
                rightSpeed = (0.5 - position / WIDTH) * base_speed
            else:
                leftSpeed = straight_speed
                rightSpeed = straight_speed
        else:
            print('INPUT ERROR!:steering:type_select')

        motors[0].setVelocity(leftSpeed)
        motors[1].setVelocity(rightSpeed)


Sakura = Sakura()
Sakura.motor_initialize()

while robot.step(TIME_STEP) != -1:
    pass
    # initialize

    # linePatrol
        # get_frame
        # edge_detect
        # positioning
        # steering