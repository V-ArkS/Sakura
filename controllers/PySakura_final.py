from controller import Robot
from controller import Camera
from controller import Compass
from controller import PositionSensor
import numpy as np
import time
import cv2

global count_arch
count_arch = 0
flag = -1

class Vehicle:
    stage = 0
    MAX_SPEED = 2
    robot = Robot()
    motors = []
    armMotors = []
    armSensors = []
    armPs = []
    camera = Camera('camera')
    compass = Compass('compass')
    ds = robot.getDistanceSensor('distance sensor')
    ds2 = robot.getDistanceSensor('distance sensor2')

    HEIGHT = camera.getHeight()
    WIDTH = camera.getWidth()
    foodStage = 0
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
            if i <= 1:
                self.motors[i].setVelocity(0)
            else:
                self.motors[i].setVelocity(0.8)
                self.motors[i].setPosition(0)
            
        armMotorNames = ["arm_motor_1","arm_motor_2","arm_motor_3","arm_motor_4","arm_motor_5","arm_motor_6","arm_motor_7","arm_motor_8"]
        for i in range(len(armMotorNames)):
            self.armMotors.append(self.robot.getMotor(armMotorNames[i]))
            self.armMotors[i].setPosition(0)
            self.armMotors[i].setVelocity(0.3)
            
        armSensorNames = ["arm_position_sensor_1", "arm_position_sensor_2","arm_position_sensor_3","arm_position_sensor_4","arm_position_sensor_5","arm_position_sensor_6","arm_position_sensor_7","arm_position_sensor_8"]
        for i in range(len(armSensorNames)):
            self.armSensors.append(self.robot.getPositionSensor(armSensorNames[i]))
            self.armSensors[i].enable(int(self.robot.getBasicTimeStep()))
            
        self.camera.enable(int(self.robot.getBasicTimeStep()))
        self.compass.enable(int(self.robot.getBasicTimeStep()))
        self.ds.enable(int(self.robot.getBasicTimeStep()))
        self.ds2.enable(int(self.robot.getBasicTimeStep()))

    def getStage(self):
        return self.stage

    def setStage(self, stage_num):
        self.stage = stage_num

    def getCompass(self):
        return np.angle(complex(self.compass.getValues()[0], self.compass.getValues()[2]))

    def getDistanceValue(self,input_ds = 1):
        if input_ds == 1:
            return self.ds.getValue()
        if input_ds == 2:
            return self.ds2.getValue()
    
    def towerSeeLeft(self):
        self.motors[2].setPosition(np.pi / 2)

    @staticmethod
    def towerSeeRight():
        Vehicle.motors[2].setPosition(-np.pi / 2)

    def towerRestore(self):
        self.motors[2].setPosition(0)

    def pickFood(self):
        if self.foodStage == 0:
            self.armMotors[0].setPosition(0.039)#upper-arm extend
            self.armMotors[1].setPosition(0.93)#upper-arm rotate
            if (self.armSensors[1].getValue()>0.92):
                self.armMotors[2].setPosition(0.038)#mid-arm extend
                #print(sensor3.getValue())
                if (self.armSensors[2].getValue()>0.0379):
                    self.armMotors[4].setPosition(0.005)#grab the box
                    self.armMotors[5].setPosition(0.005)
                    if((self.armSensors[4].getValue()>=0.00269) or (self.armSensors[5].getValue()>=0.0031)):
                        self.foodStage = 1
        elif self.foodStage == 1:
            self.armMotors[2].setPosition(0)#raise the box
            self.armMotors[2].setVelocity(.1)
            self.armMotors[1].setPosition(0)
            self.armMotors[6].setPosition(0.9)
            if(self.armSensors[1].getValue()<0.002):
                self.armMotors[0].setPosition(0)
                self.armMotors[0].setVelocity(0.01)#grabbing task finished
                if(self.armSensors[0].getValue()<0.003):
                    return True
        return False
    def releaseFood(self):
        if(self.armSensors[6].getValue()>0.899):
            self.armMotors[3].setPosition(0.06)
            self.armMotors[7].setPosition(0.06)
            if((self.armSensors[3].getValue()>0.059) and (self.armSensors[7].getValue()>0.059)):
                self.armMotors[4].setPosition(0)
                self.armMotors[5].setPosition(0)
                if((self.armSensors[4].getValue()<=0.00269) or (self.armSensors[5].getValue()>=0.0031)):
                    self.armMotors[3].setPosition(0)
                    self.armMotors[7].setPosition(0)
                    self.armMotors[6].setPosition(0)
                    return True

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
        #cv2.imwrite("dst.jpg", dst)
        # to binary
        ret, binary = cv2.threshold(dst, threshold, maxVal, cv2.THRESH_BINARY)
        #cv2.imwrite("binary.jpg", binary)
        # Smooth
        kernel1 = np.ones((kernel_1, kernel_1), np.float) / 25
        smooth = cv2.filter2D(binary, -1, kernel1)
        # blur
        blur_im = cv2.boxFilter(smooth, -1, (kernel_2, kernel_2), normalize=1)
        #cv2.imwrite("blur.jpg", blur_im)
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
            self.setSpeed(-0.3*diff, 0.3*diff)
            return True

    def linePatrol(self):
        #get camera image, process it and set speed based on line patrolling algorithm
        #return False if there is no line
        image = self.camera.getImage()
        leftSum = 0
        rightSum = 0
        leftSpeed = 0
        rightSpeed = 0
        cameraData = self.camera.getImage()
        HEIGHT = self.camera.getHeight()
        WIDTH = self.camera.getWidth()
        frame = np.zeros((HEIGHT, WIDTH))
        for x in range(0, HEIGHT):
            for y in range(0, WIDTH):
                frame[y][x] = int(self.camera.imageGetGray(cameraData, WIDTH, x, y))
        absX = cv2.convertScaleAbs(cv2.Sobel(frame, cv2.CV_16S, 1, 0))
        absY = cv2.convertScaleAbs(cv2.Sobel(frame, cv2.CV_16S, 0, 1))
        # to binary
        ret, binary = cv2.threshold(cv2.addWeighted(absX, 0.5, absY, 0.5, 0), 254, 255, cv2.THRESH_BINARY)
        binaryImg = cv2.boxFilter(cv2.filter2D(binary, -1, np.ones((20, 20), np.float) / 25), -1, (15, 15), normalize=1)

        positionSum = 0
        positionCount = 0
        for i in range(int(HEIGHT*0.75), HEIGHT):
            for j in range(WIDTH*0, WIDTH):
                if binaryImg[i][j] != 0:
                    positionSum += j
                    positionCount += 1
        farpositionSum = 0
        farpositionCount = 0
        for i in range(int(HEIGHT*0.32), int(HEIGHT*0.44)):
            for j in range(WIDTH*0, WIDTH):
                if binaryImg[i][j] != 0:
                    farpositionSum += j
                    farpositionCount += 1
        if farpositionCount == 0:
            farcenter = 0
        else:
            farcenter = farpositionSum / farpositionCount
        if abs(farcenter - WIDTH/2) < 0.1:
            self.motors[0].setAcceleration(0.3)
            self.motors[1].setAcceleration(0.3)
            #("straight")
        else:
            self.motors[0].setAcceleration(20)
            self.motors[1].setAcceleration(20)
            #print("not straight")
        if positionCount or farpositionCount:
            if positionCount == 0:
                center = 80
            else:
                center = positionSum / positionCount
            diff = 4 * (0.5 - center/WIDTH)
            if diff > 0.01:
                leftSpeed = (1 - 0.6 * diff) * self.MAX_SPEED
                rightSpeed = (1 - 0.3 * diff) * self.MAX_SPEED
            elif diff < -0.01:
                leftSpeed = (1 + 0.3 * diff) * self.MAX_SPEED
                rightSpeed = (1 + 0.6 * diff) * self.MAX_SPEED
            else:
                leftSpeed = self.MAX_SPEED
                rightSpeed = self.MAX_SPEED
          
            self.setSpeed(leftSpeed, rightSpeed)
            return True
        else:
            return False

    @staticmethod
    def linePatrol2():
        # Task 1
        # get_frame
        frame = Vehicle.get_frame()
        # edge_detect
        blur_im = Vehicle.edge_detect(frame, 100, 255, 20, 15)
        # positioning
        position = Vehicle.positioning(blur_im, 0, 0.7, 0.8)
        # steering
        leftSpeed, rightSpeed = Vehicle.steering(position, 0, 4, 0.8, 0.8)

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

 

        for axis_v in range(int(HEIGHT * 0.8), int(HEIGHT * 0.87)):
            for axis_h in range(HALF, WIDTH):
                if filtered[axis_v][axis_h] != 0:
                    axis = axis + axis_h
                    num1 = num1 + 1
        if num1:
            position_i = axis / num1 + 1
        if abs(position_i - 1.5 * HALF) < 10:
            # print('pi = ', position_i)
            for axis_v in range(int(HEIGHT * 0.82), int(HEIGHT * 0.85)):
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
            #print(i, position)
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
            
    def count(self):
        HEIGHT = Vehicle.HEIGHT
        WIDTH = Vehicle.WIDTH
        camera = Vehicle.camera
        cameraData = camera.getImage()
        i = 0
        for x in range(0, WIDTH):
            for y in range(0, HEIGHT):
                gray = int(camera.imageGetGray(cameraData, WIDTH, x, y))
                if 110 < gray < 130:
                    i = i+1
        else:
           pass
        #print(i)
        return i
        
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
                leftSpeed = (position / WIDTH) * 0.9
                rightSpeed = (1 - position / WIDTH) * 0.9
            # if lines are losted in the view of camera, try adjust parameters here
            else:
                leftSpeed = 1
                rightSpeed = 1
        else:
            pass
        return leftSpeed, rightSpeed

if __name__ == "__main__":
    TIME_STEP = 64
    ts = 1/TIME_STEP
    vehicle = Vehicle()
    vehicle.towerRestore()
    target = -np.pi / 2
    turnFlag = False
    released = False
    findbridge = 0
    findcolor = False
    rotating1 = 0
    rotating2 = 0
    rotating3 = 0
    stop = 0
    vehicle.motors[0].setAcceleration(10)
    vehicle.motors[1].setAcceleration(10)
    vehicle.setStage(1)
    vehicle.time_tmp = time.time()
    while vehicle.robot.step(TIME_STEP) != -1:
        stage = vehicle.getStage()
        if stage == 1:
            if vehicle.pickFood():
#there is tiny possibility that during line-patrolling, the fish food will drop.
#I've adjust the position to fix it, if you still can find this situation, please inform in group.
                if not vehicle.linePatrol():
                    vehicle.setStage(2)
        elif stage == 2:
            vehicle.setSpeed (0.8, 0.8)
            if not released:
                vehicle.towerSeeRight()
                if vehicle.box_found():
                    released = True
            else:
                target = -np.pi / 2
                if not (vehicle.turnRound(target - vehicle.getCompass())):
                    vehicle.towerRestore()
                    if vehicle.getDistanceValue(1) > 950:
#there is tiny possibility that the distance sensor can not find orange box
#because specification of PC is different, the stop position of vehicle can have slight difference.
#if it happens, try to adjust the position of "distance sensor" (the first one)set in children of robot to fix it.
                        vehicle.setSpeed(0.1, 0.1)
                    else:
                        vehicle.setSpeed(0, 0)
                        if vehicle.releaseFood():
                            vehicle.towerSeeRight()
                            vehicle.setStage(3)
        elif stage == 3:
            #print(rotating1)
            if rotating1 == 0:
                target = 0
                if not (vehicle.turnRound(target - vehicle.getCompass())):
                    if findbridge == 0:
                        vehicle.setSpeed(1, 1)
                        vehicle.towerSeeRight()
                        if vehicle.bridge_found():
                            findbridge = 1
                    if findbridge == 1:
                        vehicle.setSpeed(0, 0)
                        rotating1 = 1    
            elif rotating1 == 1:
                target = np.pi / 2
                if not (vehicle.turnRound(target - vehicle.getCompass())):
                    vehicle.towerRestore()
                    rotating1 = 2
            elif rotating1 == 2:
                if vehicle.getDistanceValue(2) > 990:  
                    vehicle.setSpeed(0.8, 0.8)
                    #if your vehicle can not pass the bridge, try to change the speed here
                else:
                    vehicle.setSpeed(0, 0) 
                    rotating1 = 3
            elif rotating1 == 3:
                  target = 0
                  if not (vehicle.turnRound(target - vehicle.getCompass())):
                       vehicle.setSpeed(0.5, 0.5)
                       rotating1 = 4
            else:
                  vehicle.setStage(4)
        elif stage == 4:
            if rotating2 == 0:
                vehicle.towerSeeLeft()
                if vehicle.arch_found(0.1, 0.2):
                    vehicle.towerRestore()
                    rotating2 = 1
            elif rotating2 == 1:
                  target = np.pi / 2
                  if not (vehicle.turnRound(target - vehicle.getCompass())):
                      vehicle.setSpeed(0.5, 0.5)
                      rotating2 = 2
            elif rotating2 == 2:
                  if rotating3 == 0:
                      vehicle.setSpeed(0.5, 0.5)
                      i = vehicle.count()
                      if i < 10:
                          rotating3 = 1
                  elif rotating3 == 1:
                      vehicle.setSpeed(0.5, 0.5)
                      i = vehicle.count()
                      if i > 14:
                          vehicle.setSpeed(0, 0)
                          target = np.pi / 2
                          if not (vehicle.turnRound(target - vehicle.getCompass())):
                              vehicle.setStage(5)
        elif stage == 5:
            # if lines are losted in the view of camera, try adjust parameters in colourpatrol()
            vehicle.colourPatrol()
            #print(flag)
            if flag == -1:
                leftSpeed, rightSpeed = vehicle.linePatrol2()
                #print(leftSpeed, rightSpeed)
                vehicle.setSpeed(leftSpeed, rightSpeed)
            elif flag == 0 or flag == 1 or flag == 2:
                leftSpeed, rightSpeed = vehicle.colourPatrol()
                vehicle.setSpeed(leftSpeed, rightSpeed)
            elif flag == 3:
                stop = stop+1
                if stop>=10:
                    vehicle.setSpeed(0, 0)
                    print("Finished!")