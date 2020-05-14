from controller import Robot
from controller import Camera
from controller import Compass
import numpy as np
import cv2

global count_arch
count_arch = 0

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
    def get_frame(type_select=0, flag=-1):
        HEIGHT = Vehicle.HEIGHT
        WIDTH = Vehicle.WIDTH
        camera = Vehicle.camera
        cameraData = camera.getImage()

        # get image and process it
        frame = np.zeros((HEIGHT, WIDTH))
        if type_select == 0:
            for x in range(0, Vehicle.WIDTH):
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
                    elif flag == 3:
                        if 110 < gray < 130:
                            frame[y][x] = gray
                        else:
                            pass
                    else:
                        print('INPUT ERROR!:get_frame:flag')
        else:
            print('INPUT ERROR!:get_frame:type_select')
        cv2.imwrite('frame.jpg',frame)
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
        cv2.imwrite("dst.jpg", dst)
        # to binary
        ret, binary = cv2.threshold(dst, threshold, maxVal, cv2.THRESH_BINARY)
        cv2.imwrite("binary.jpg", binary)
        # Smooth
        kernel1 = np.ones((kernel_1, kernel_1), np.float) / 25
        smooth = cv2.filter2D(binary, -1, kernel1)
        # blur
        blur_im = cv2.boxFilter(smooth, -1, (kernel_2, kernel_2), normalize=1)
        cv2.imwrite("blur.jpg", blur_im)
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
    def linePatrol(self):
        #get camera image, process it and set speed based on line patrolling algorithm
        #return False if there is no line
        pass
    def boxPatrol(self):
        #get camera image and find orange box, then adjust the speed to go to the box
        #return False if there is no box
        pass
    def bridgePatrol(self):
        #get camera image and find bridge, then adjust the speed to go to the bridge
        #return False if there is no bridge
        pass
    def archPatrol(self):
        #get camera image and find arch, then adjust the speed to go to the arch
        #return False if there is no arch
        pass
    def colourPatrol(self):
        #for task 5
        pass

    @staticmethod
    def linePatrol(type_select_frame=0, color_flag=-1, threshold=254, maxVal=255, kernel_1=20, kernel_2=15,
                   type_select_position=0, height_per_1=0.7, height_per_2=0.8, rectify_pixel=4, base_speed=2,
                   straight_speed=2):
        # get_frame
        frame = Vehicle.get_frame(type_select_frame, color_flag)
        # edge_detect
        blur_im = Vehicle.edge_detect(frame, threshold, maxVal, kernel_1, kernel_2)
        # positioning
        position = Vehicle.positioning(blur_im, type_select_position, height_per_1, height_per_2)
        # steering
        leftSpeed, rightSpeed = Vehicle.steering(position, type_select_position, rectify_pixel, base_speed, straight_speed)

        return leftSpeed, rightSpeed

    @staticmethod
    def box_found():
        HALF = Vehicle.HALF
        frame = Vehicle.get_frame(0, -1)
        x = cv2.Sobel(frame, cv2.CV_16S, 1, 0)
        y = cv2.Sobel(frame, cv2.CV_16S, 0, 1)
        absx = cv2.convertScaleAbs(x)
        absy = cv2.convertScaleAbs(y)
        dst = cv2.addWeighted(absx, 0.5, absy, 0.5, 0)
        cv2.imwrite("dst.jpg", dst)
        # to binary
        ret, filtered = cv2.threshold(dst, 50, 255, cv2.THRESH_TOZERO)
        cv2.imwrite("binary.jpg", filtered)
        position_i = Vehicle.positioning(filtered, 0, 0.32, 0.35)
        if abs(position_i - HALF) < 10:
            position = Vehicle.positioning(filtered, 0, 0.25, 0.3)
            if abs(position - HALF) < 10: # this is a predict letting it stop exactly at the box center
                return True
            else:
                pass

    @staticmethod
    def bridge_found():
        HALF = Vehicle.HALF
        WIDTH = Vehicle.WIDTH
        HEIGHT = Vehicle.HEIGHT
        count = 0
        num1 = 0
        num = 0
        axis = 0
        position_i = 0
        frame = Vehicle.get_frame(0, -1)
        x = cv2.Sobel(frame, cv2.CV_16S, 1, 0)
        y = cv2.Sobel(frame, cv2.CV_16S, 0, 1)
        absx = cv2.convertScaleAbs(x)
        absy = cv2.convertScaleAbs(y)
        dst = cv2.addWeighted(absx, 0.5, absy, 0.5, 0)
        # cv2.imwrite("dst.jpg", dst)
        # to binary
        ret, filtered = cv2.threshold(dst, 50, 255, cv2.THRESH_TOZERO)
        # cv2.imwrite("filtered.jpg", filtered)
        ret, binary = cv2.threshold(dst, 10, 255, cv2.THRESH_BINARY)
        # cv2.imwrite("binary.jpg", binary)

        for axis_v in range(int(HEIGHT * 0.32), int(HEIGHT * 0.35)):
            for axis_h in range(HALF, WIDTH):
                if filtered[axis_v][axis_h] != 0:
                    axis = axis + axis_h
                    num1 = num1 + 1
        if num1:
            position_i = axis / num1 + 1

        for axis_v in range(int(HEIGHT * 0.3), int(HEIGHT * 0.35)):
            for axis_h in range(WIDTH * 0, int(WIDTH * 0.5)):
                if binary[axis_v][axis_h] == 0:
                    num = num + 1
                count = count + 1
        if num:
            per = num / count
        '''print(per)
        print(num)
        print(count)
        print(position_i)'''
        if abs(position_i - 1.5 * HALF) < 10:
            if per > 0.99:
                return True
            else:
                pass

    '''@staticmethod
    def arch_found():
        HALF = Vehicle.HALF
        global count_arch
        frame = Vehicle.get_frame(0, -1)
        x = cv2.Sobel(frame, cv2.CV_16S, 1, 0)
        y = cv2.Sobel(frame, cv2.CV_16S, 0, 1)
        absx = cv2.convertScaleAbs(x)
        absy = cv2.convertScaleAbs(y)
        dst = cv2.addWeighted(absx, 0.5, absy, 0.5, 0)
        cv2.imwrite("dst.jpg", dst)
        # to binary
        ret, binary = cv2.threshold(dst, 80, 255, cv2.THRESH_BINARY)
        cv2.imwrite("binary.jpg", binary)
        # filtered = Vehicle.edge_detect(frame, 50, 255, 20, 15)
        # position_i = Vehicle.positioning(filtered, 0, 0.32, 0.35)
        # if abs(position_i - HALF) < 10:
        position = Vehicle.positioning(binary, 0, 0.1, 0.125)
        print(position)
        if abs(position - HALF) < 5:
            count_arch = count_arch + 1
            print("count = ", count_arch)
            # if count_arch == 2:
            return 0
        else:
            pass
        '''


if __name__ == "__main__":
    TIME_STEP = 64
    vehicle = Vehicle()
    vehicle.towerRestore()
    target = -np.pi / 2
    turnFlag = False
    released = False
    vehicle.setStage(1)
    vehicle.motors[0].setAcceleration(4)
    vehicle.motors[1].setAcceleration(4)
    box_found = 0

    while vehicle.robot.step(TIME_STEP) != -1:
        # pass
        # Switch stage for different task, if a task is completed, increase the stage by
        # using vehicle.setstage()

        # Task 2
        # if not (vehicle.turnRound(target - vehicle.getCompass())):
        #     if not released:
        #         vehicle.releaseFood()
        #         released = True
        Vehicle.towerSeeRight()
        if Vehicle.bridge_found():
            box_found = 1
        if box_found == 1:
            Vehicle.setSpeed(vehicle, -0.3, -0.3)
            if Vehicle.bridge_found():
                vehicle.setSpeed(0, 0)
        else:
            Vehicle.setSpeed(vehicle, 2.0, 2.0)

        '''leftSpeed, rightSpeed = Vehicle.linePatrol()
        vehicle.setSpeed(leftSpeed, rightSpeed)'''
        # Task 4
        # if (vehicle.getDistanceValue() < 900):
        #     turnFlag = True
        # if (turnFlag):
        #     if not vehicle.turnRound(target - vehicle.getCompass()):
        #         turnFlag = False
        #         vehicle.setSpeed(0.1, 0.1, 0.1, 0.1)