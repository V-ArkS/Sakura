from controller import Robot
from controller import Camera
from controller import Compass
import numpy as np
import cv2

global count_arch
count_arch = 0
flag = -1

class Vehicle:
    stage = 0
    MAX_SPEED = 0.3
    robot = Robot()
    motors = []
    camera = Camera('camera')
    compass = Compass('compass')
    ds = robot.getDistanceSensor('distance sensor')

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

    def __init__(self):
        #get motors, camera and initialise them.
        motorNames = ['left motor', 'right motor', 'tower rotational motor'] #, 'trigger motor']
        for i in range(3):
            self.motors.append(self.robot.getMotor(motorNames[i]))
            self.motors[i].setPosition(float('inf'))
            self.motors[i].setVelocity(0)
        #tower motor
        self.motors[2].setVelocity(1)
        self.camera.enable(int(self.robot.getBasicTimeStep()))
        self.compass.enable(int(self.robot.getBasicTimeStep()))
        self.ds.enable(int(self.robot.getBasicTimeStep()))

    def getStage(self):
        return self.stage

    def setStage(self, stage_num):
        self.stage = stage_num

    def getCompass(self):
        return np.angle(complex(self.compass.getValues()[0], self.compass.getValues()[2]))

    def getDistanceValue(self):
        return self.ds.getValue()

    def towerSeeLeft(self):
        self.motors[2].setPosition(np.pi / 2)

    @staticmethod
    def towerSeeRight():
        Vehicle.motors[2].setPosition(-np.pi / 2)

    def towerRestore(self):
        self.motors[2].setPosition(0)

    def releaseFood(self):
        self.motors[3].setPosition(0.1)
        self.motors[3].setVelocity(0.6)

#Img Processing related
    @staticmethod
    def get_frame():
        HEIGHT = Vehicle.HEIGHT
        WIDTH = Vehicle.WIDTH
        camera = Vehicle.camera
        cameraData = camera.getImage()

        # get image and process it
        frame = np.zeros((HEIGHT, WIDTH))
        for x in range(0, Vehicle.WIDTH):
            for y in range(0, HEIGHT):
                gray = int(camera.imageGetGray(cameraData, WIDTH, x, y))
                frame[y][x] = gray

        return frame

    @staticmethod
    def edge_detect(frame, threshold=254, maxVal=255, kernel_1=20, kernel_2=15):
        # threshold
        # Edge_LineTracking_T1:254; Edge_Bridge_detection_T3:180; Edge_Arch_detection_T4:100;Edge_Color_detect_T5:254
        # maxVal
        # Edge_LineTracking_T1:255; Edge_Bridge_detection_T3:255; Edge_Arch_detection_T4:255;Edge_Color_detect_T5:255
        # kernel_1
        # Edge_LineTracking_T1:20; Edge_Bridge_detection_T3:20; Edge_Arch_detection_T4:20;Edge_Color_detect_T5:20
        # kernel_2
        # Edge_LineTracking_T1:15; Edge_Bridge_detection_T3:15; Edge_Arch_detection_T4:15;Edge_Color_detect_T5:15
        # edge_detect Sobel
        x = cv2.Sobel(frame, cv2.CV_16S, 1, 0)
        y = cv2.Sobel(frame, cv2.CV_16S, 0, 1)
        absx = cv2.convertScaleAbs(x)
        absy = cv2.convertScaleAbs(y)
        dst = cv2.addWeighted(absx, 0.5, absy, 0.5, 0)
        # cv2.imwrite("dst.jpg", dst)
        # to binary
        ret, binary = cv2.threshold(dst, threshold, maxVal, cv2.THRESH_BINARY)
        # cv2.imwrite("binary.jpg", binary)
        # Smooth
        kernel1 = np.ones((kernel_1, kernel_1), np.float) / 25
        smooth = cv2.filter2D(binary, -1, kernel1)
        # blur
        blur_im = cv2.boxFilter(smooth, -1, (kernel_2, kernel_2), normalize=1)
        # cv2.imwrite("blur.jpg", blur_im)
        return blur_im

    @staticmethod
    def positioning(blur_im, type_select=0, height_per_1=0.7, height_per_2=0.8):
        # height_per_1
        # Edge_LineTracking_T1:0.7; Edge_Bridge_detection_T3:0.3; Edge_Arch_detection_T4:0.3;Edge_Color_detect_T5:0.6
        # height_per_2
        # Edge_LineTracking_T1:0.8; Edge_Bridge_detection_T3:0.6; Edge_Arch_detection_T4:0.8;Edge_Color_detect_T5:0.7
        HEIGHT = Vehicle.HEIGHT
        WIDTH = Vehicle.WIDTH
        HALF = Vehicle.HALF
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
        return position

    @staticmethod
    def steering(position, type_select=0, rectify_pixel=0, base_speed=3.0, straight_speed=2.0):
        # rectify_pixel
        # Edge_LineTracking_T1:7; Edge_Bridge_detection_T3:5; Edge_Arch_detection_T4:5;Edge_Color_detect_T5:3
        # base_speed
        # Edge_LineTracking_T1:3; Edge_Bridge_detection_T3:5; Edge_Arch_detection_T4:5;Edge_Color_detect_T5:20
        # straight_speed
        # Edge_LineTracking_T1:2; Edge_Bridge_detection_T3:3; Edge_Arch_detection_T4:2;Edge_Color_detect_T5:2
        WIDTH = Vehicle.WIDTH
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
        return leftSpeed, rightSpeed

#Speed setting functions
    def setSpeed(self, left, right):
        #set speed for four tracks
        self.motors[0].setVelocity(left)
        self.motors[1].setVelocity(right)

    def turnRound(self, diff):
        #set speed for turning left or right
        if abs(diff) < 0.001:
            self.setSpeed(0, 0)
            return False
        else:
            self.setSpeed(-0.3 * diff, 0.3 * diff)
            return True

    @staticmethod
    def linePatrol():
        # Task 1
        # get_frame
        frame = Vehicle.get_frame()
        # edge_detect
        blur_im = Vehicle.edge_detect(frame, 100, 255, 20, 15)
        # positioning
        position = Vehicle.positioning(blur_im, 0, 0.7, 0.8)
        # steering
        leftSpeed, rightSpeed = Vehicle.steering(position, 0, 4, 0.3, 2)

        return leftSpeed, rightSpeed

    @staticmethod
    def box_found():
        # Task 2
        HALF = Vehicle.HALF
        frame = Vehicle.get_frame()
        x = cv2.Sobel(frame, cv2.CV_16S, 1, 0)
        y = cv2.Sobel(frame, cv2.CV_16S, 0, 1)
        absx = cv2.convertScaleAbs(x)
        absy = cv2.convertScaleAbs(y)
        dst = cv2.addWeighted(absx, 0.5, absy, 0.5, 0)
        # cv2.imwrite("dst.jpg", dst)
        # to binary
        ret, filtered = cv2.threshold(dst, 50, 255, cv2.THRESH_TOZERO)
        # cv2.imwrite("binary.jpg", filtered)
        position_i = Vehicle.positioning(filtered, 0, 0.32, 0.35)
        if abs(position_i - HALF) < 10:
            position = Vehicle.positioning(filtered, 0, 0.25, 0.3)
            if abs(position - HALF) < 5:
                return True
            else:
                return False

    @staticmethod
    def bridge_found():
        # Task 3
        HALF = Vehicle.HALF
        WIDTH = Vehicle.WIDTH
        HEIGHT = Vehicle.HEIGHT
        count = 0
        num1 = 0
        num = 0
        axis = WIDTH
        position_i = 0
        frame = Vehicle.get_frame()
        x = cv2.Sobel(frame, cv2.CV_16S, 1, 0)
        y = cv2.Sobel(frame, cv2.CV_16S, 0, 1)
        absx = cv2.convertScaleAbs(x)
        absy = cv2.convertScaleAbs(y)
        dst = cv2.addWeighted(absx, 0.5, absy, 0.5, 0)
        # cv2.imwrite("dst.jpg", dst)
        # to binary
        ret, filtered = cv2.threshold(dst, 50, 255, cv2.THRESH_TOZERO)
        # cv2.imwrite("filtered.jpg", filtered)

        for axis_v in range(int(HEIGHT * 0.64), int(HEIGHT * 0.69)):
            for axis_h in range(HALF, WIDTH):
                if filtered[axis_v][axis_h] != 0:
                    axis = axis + axis_h
                    num1 = num1 + 1
        if num1:
            position_i = axis / num1 + 1
        if abs(position_i - 1.5 * HALF) < 10:
            # print('pi = ', position_i)
            for axis_v in range(int(HEIGHT * 0.60), int(HEIGHT * 0.67)):
                for axis_h in range(WIDTH * 0, int(WIDTH * 1)):
                    if filtered[axis_v][axis_h] != 0:
                        axis_i = axis_h
                        axis = min(axis_i, axis)
            # print('axis = ', axis)
            if abs(axis - 0.5 * WIDTH) < 5:
                return True
            else:
                return False

    def arch_found(self, a, b):
        # Task 4
        global count_arch
        HEIGHT = Vehicle.HEIGHT
        WIDTH = Vehicle.WIDTH
        camera = Vehicle.camera
        cameraData = camera.getImage()
        frame = np.zeros((HEIGHT, WIDTH))
        i = 0

        position = 0
        for x in range(0, WIDTH):
            for y in range(0, HEIGHT):
                gray = int(camera.imageGetGray(cameraData, WIDTH, x, y))
                red = int(camera.imageGetRed(cameraData, WIDTH, x, y))
                if 110 < gray < 130:
                    frame[y][x] = 255
                    i = i+1
        else:
            pass
        #print(i)
        # cv2.imwrite('frame.jpg', frame)
        #blur_im = self.edge_detect(frame, 254, 255)
        #cv2.imwrite('blur.jpg', blur_im)
        if i > 50:
            position = self.positioning(frame, 0, a, b)
            print(i, position)
            if count_arch == 0:
                if int(position) < WIDTH / 2:
                    count_arch = count_arch + 1
                    for i in range(2):
                        self.motors[i].setVelocity(0)
                    return 1
            else:
                if int(position) == WIDTH / 2:
                    for i in range(2):
                        self.motors[i].setVelocity(0)
                    return 1
                else:
                    return 0

        else:
            return 0

    @staticmethod
    def colourPatrol():
        # Task 5
        HEIGHT = Vehicle.HEIGHT
        WIDTH = Vehicle.WIDTH
        camera = Vehicle.camera
        cameraData = camera.getImage()
        frame = np.zeros((HEIGHT, WIDTH))
        color_kernel = np.zeros((3, 3))
        compass = Vehicle.getCompass(Vehicle)
        position = HEIGHT / 2
        leftSpeed = 0
        rightSpeed = 0
        global flag
        if (flag == -1) and (3.12 < abs(compass) < 3.15):
            for x in range(0, 3):
                for y in range(0, 3):
                    gray = int(camera.imageGetGray(cameraData, WIDTH, x + int(0.5 * WIDTH), y + int(0.80 * HEIGHT)))
                    color_kernel[y][x] = gray

            gray_average = np.mean(color_kernel)
            # color classification
            if 100 < gray_average < 130:
                flag = 0  # red
            elif 175 < gray_average < 200:
                flag = 1  # yellow
            elif 140 < gray_average < 170:
                flag = 2  # purple
            else:
                pass
            # print('color flag=', flag)
            leftSpeed = 2.0
            rightSpeed = 2.0

        elif flag == 0 or flag == 1 or flag == 2:
            for x in range(0, 3):
                for y in range(0, 3):
                    gray = int(camera.imageGetGray(cameraData, WIDTH, x + int(0.5 * WIDTH), y + int(0.95 * HEIGHT)))
                    color_kernel[y][x] = gray
            gray_average = np.mean(color_kernel)
            if 90 < gray_average < 100:
                flag = 3  # finished

            for y in range(0, HEIGHT):
                for x in range(0, WIDTH):
                    gray = int(camera.imageGetGray(cameraData, WIDTH, x, y))
                    # color classification
                    if flag == 0:
                        if 100 < gray < 130:
                            frame[y][x] = 255
                        else:
                            pass
                    elif flag == 1:
                        if 175 < gray < 200:
                            frame[y][x] = 255
                        else:
                            pass
                    elif flag == 2:
                        if 140 < gray < 170:
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

            # cv2.imwrite('smooth_blur.jpg', blur_im1)
            # cv2.imwrite('gray.jpg', frame)
            # cv2.imwrite('frame.jpg', frame)
            # video.write(blur_im1)

            # Locate
            axis = 0
            num = 0
            for axis_v in range(int(HEIGHT * 0.8), int(HEIGHT * 0.9)):
                for axis_h in range(WIDTH * 0, WIDTH):
                    if blur_im1[axis_v][axis_h] != 0:
                        axis = axis + axis_h
                        num = num + 1
            if num:
                position = axis / num + 1
            # Steering
            if 5 <= abs(position - WIDTH / 2):
                leftSpeed = (position / WIDTH) * 1.5
                rightSpeed = (1 - position / WIDTH) * 1.5
            elif 2.5 < abs(position - WIDTH / 2) < 5:
                leftSpeed = (position / WIDTH) * 0.4
                rightSpeed = (1 - position / WIDTH) * 0.4

            else:
                leftSpeed = 2.0
                rightSpeed = 2.0

        else:
            pass
        return leftSpeed, rightSpeed

if __name__ == "__main__":
    TIME_STEP = 64
    vehicle = Vehicle()
    vehicle.towerRestore()
    target = -np.pi / 2
    turnFlag = False
    released = False
    vehicle.setStage(1)
    vehicle.motors[0].setAcceleration(10)
    vehicle.motors[1].setAcceleration(10)
    arch_found = 0
    box_found = 0
    bridge_found = 0
    count = 0
    while vehicle.robot.step(TIME_STEP) != -1:
        # pass
        # Switch stage for different task, if a task is completed, increase the stage by
        # using vehicle.setstage()

        # Task 2
        # if not (vehicle.turnRound(target - vehicle.getCompass())):
        #     if not released:
        #         vehicle.releaseFood()
        #         released = True
        # Task 1 调试样码
        '''leftSpeed, rightSpeed = Vehicle.linePatrol()
        vehicle.setSpeed(leftSpeed, rightSpeed)'''

        # Task 2 box_found 调试样码
        # 状态一：停在中点
        '''vehicle.towerSeeRight()
        if vehicle.box_found() and box_found == 0:
            box_found = 1
        if box_found == 1:
            vehicle.setSpeed(-0.35, -0.35)
            if vehicle.box_found():
                vehicle.setSpeed(0, 0)
                if -10e-12 < vehicle.getCompass() < -9e-12:
                    box_found = 2
        # 状态二：转向
        elif box_found == 2:
            vehicle.setSpeed(0.5, -0.5)
            vehicle.towerRestore()
            if -1.4 < vehicle.getCompass() < -1.1:
                box_found = 3
        # 状态三：停止转向且恢复摄像头
        elif box_found == 3:
            vehicle.setSpeed(0, 0)
            vehicle.towerRestore()
            if -1.52 < vehicle.getCompass() < -1.5:
                box_found = 4
        # 状态四：前进🐛！！！
        elif box_found == 4:
            vehicle.towerRestore()
            vehicle.setSpeed(0.5, 0.5)
        else:
            Vehicle.setSpeed(vehicle, 2.0, 2.0)

        print(vehicle.getCompass())'''

        # Task 3 bridge_found 调试样码
        '''vehicle.towerSeeRight()
        if vehicle.bridge_found():
            bridge_found = 1
        if bridge_found == 1:
            vehicle.setSpeed(-0.3, -0.3)
            if vehicle.bridge_found():
                vehicle.setSpeed(0, 0)
        else:
            vehicle.setSpeed(2.0, 2.0)'''

        # Task 4 bridge_found 调试样码
        '''if vehicle.arch_found(0.1, 0.2):
            arch_found = 1
        if arch_found == 1:
            vehicle.setSpeed(-0.3, -0.3)
            if vehicle.arch_found(0.1, 0.2):
                vehicle.setSpeed(0, 0)
        else:
            vehicle.setSpeed(2.0, 2.0)'''


        # Task 5 调试样码
        vehicle.colourPatrol()
        if flag == -1:
            leftSpeed, rightSpeed = vehicle.linePatrol()
            vehicle.setSpeed(leftSpeed, rightSpeed)
        # stage 5
        elif flag == 0 or flag == 1 or flag == 2:
            leftSpeed, rightSpeed = vehicle.colourPatrol()
            vehicle.setSpeed(leftSpeed, rightSpeed)
        elif flag == 3:
            if count < 10:
                vehicle.setSpeed(0.5, 0.5)
                count = count + 1
            else:
                vehicle.setSpeed(0, 0)
            print("Finished!")
