from controller import Robot
from controller import Camera
import numpy as np
import cv2

TIME_STEP = 64
robot = Robot()

motors = []
motorNames = ['left motor', 'right motor']
camera = Camera('camera')
camera.enable(int(robot.getBasicTimeStep()))
print(camera.getWidth(), camera.getHeight(), camera.getSamplingPeriod())
leftSpeed = 0.0
rightSpeed = 0.0
leftSum = 0
rightSum = 0
image = 0
cameraData = 0
HEIGHT = camera.getHeight()
WIDTH = camera.getWidth()
p0 = WIDTH / 2
pr = 319
pl = 0
HALF = int(WIDTH / 2)
frame = np.zeros((HEIGHT, WIDTH))
blur_im1 = np.zeros((HEIGHT, WIDTH))


class Sakura:
    @staticmethod
    def sakura_initiate():
        global leftSpeed
        global rightSpeed
        global leftSum
        global rightSum
        global image
        global cameraData
        global HEIGHT
        global WIDTH
        global p0
        global frame
        global pr
        global pl
        global HALF
        global blur_im1

        leftSpeed = 0.0
        rightSpeed = 0.0
        leftSum = 0
        rightSum = 0
        image = 0
        cameraData = 0
        HEIGHT = 0
        WIDTH = 0
        p0 = WIDTH / 2
        pr = 319
        pl = 0
        HALF = int(WIDTH / 2)
        frame = np.zeros((HEIGHT, WIDTH))
        blur_im1 = np.zeros((HEIGHT, WIDTH))


    @staticmethod
    def motor_initiate():
        global motors
        global motorNames
        for i in range(2):
            motors.append(robot.getMotor(motorNames[i]))
            motors[i].setPosition(float('inf'))
            motors[i].setVelocity(0.0)
            motors[i].setAcceleration(25)

    @staticmethod
    def get_frame(tasknum=1):
        global image
        global cameraData
        global HEIGHT
        global WIDTH
        global frame
        global leftSpeed
        global rightSpeed
        leftSpeed = 0.0
        rightSpeed = 0.0
        # get image and process it
        image = camera.getImage()
        cameraData = camera.getImage()
        frame = np.zeros((HEIGHT, WIDTH))
        for x in range(0, WIDTH):
            for y in range(0, HEIGHT):
                gray = int(camera.imageGetGray(cameraData, WIDTH, x, y))
                frame[y][x] = gray

    @staticmethod
    def edge_detect(threshold_1=254, threshold_2=255, ones_1=20, ones_2=20):
        global frame
        global blur_im1
        # edge_detect Sobel
        x = cv2.Sobel(frame, cv2.CV_16S, 1, 0)
        y = cv2.Sobel(frame, cv2.CV_16S, 0, 1)
        absx = cv2.convertScaleAbs(x)
        absy = cv2.convertScaleAbs(y)
        dst = cv2.addWeighted(absx, 0.5, absy, 0.5, 0)
        # to binary
        ret, binary = cv2.threshold(dst, threshold_1, threshold_2, cv2.THRESH_BINARY)
        # Smooth
        kernel1 = np.ones((ones_1, ones_2), np.float) / 25
        smooth = cv2.filter2D(binary, -1, kernel1)
        # blur
        blur_im1 = cv2.boxFilter(smooth, -1, (15, 15), normalize=1)

    @staticmethod
    def along_wline():
        global leftSpeed
        global rightSpeed
        global motors
        global HEIGHT
        global WIDTH
        global blur_im1
        axis = 0
        num = 0
        position = 0
        for axis_v in range(int(HEIGHT * 0.7), int(HEIGHT * 0.8)):
            for axis_h in range(WIDTH * 0, WIDTH):
                if blur_im1[axis_v][axis_h] != 0:
                    axis = axis + axis_h
                    num = num + 1
        # print(num, axis)
        if num:
            position = axis / num + 1
        # print(position)
        if abs(position - WIDTH / 2) > 7:
            leftSpeed = (position / WIDTH) * 3
            rightSpeed = (1 - position / WIDTH) * 3
        else:
            leftSpeed = 0.5
            rightSpeed = 0.5
        motors[0].setVelocity(leftSpeed)
        motors[1].setVelocity(rightSpeed)


Sakura = Sakura()
Sakura.motor_initiate()

while robot.step(TIME_STEP) != -1:
    Sakura.get_frame()
    Sakura.edge_detect()
    Sakura.along_wline()

