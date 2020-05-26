from controller import Robot
from controller import Camera
from controller import Compass
from controller import PositionSensor
import numpy as np
import cv2, time

class Vehicle:
    stage = 0
    MAX_SPEED = 4
    robot = Robot()
    timeReg = 0
    boolReg = False
    directionReg = 0
    colourFlag = 0
    foodStage = 0
    motors = []
    armMotors = []
    armSensors = []
    armPs = []
    camera = Camera('camera')
    compass = Compass('compass')
    ds1 = robot.getDistanceSensor('distance sensor 1')
    ds2 = robot.getDistanceSensor('distance sensor 2')

    def __init__(self):
        #get motors, camera and initialise them.
        motorNames = ['left motor', 'right motor', 'tower rotational motor']
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
            self.armMotors[i].setVelocity(0.8)
        
        armSensorNames = ["arm_position_sensor_1", "arm_position_sensor_2","arm_position_sensor_3","arm_position_sensor_4","arm_position_sensor_5","arm_position_sensor_6","arm_position_sensor_7","arm_position_sensor_8"]
        for i in range(len(armSensorNames)):
            self.armSensors.append(self.robot.getPositionSensor(armSensorNames[i]))
            self.armSensors[i].enable(int(self.robot.getBasicTimeStep()))
        # sensor1=self.robot.getPositionSensor("arm_position_sensor_1")
        # sensor1.enable(64)
        self.camera.enable(int(self.robot.getBasicTimeStep()))
        self.compass.enable(int(self.robot.getBasicTimeStep()))
        self.ds1.enable(int(self.robot.getBasicTimeStep()))
        self.ds2.enable(int(self.robot.getBasicTimeStep()))
        
    def getStage(self):
        return self.stage
    def setStage(self, stage_num):
        self.stage = stage_num
    def getBool(self):
        return self.boolReg
    def setBool(self, boolean):
        self.boolReg = boolean
        return boolean
    def getCompass(self):
        return np.angle(complex(self.compass.getValues()[0], self.compass.getValues()[2]))
    def getDistanceValue(self, sensor):
        if sensor == 1:
            return self.ds1.getValue()
        else:
            return self.ds2.getValue()
    def towerSeeLeft(self):
        self.motors[2].setPosition(np.pi / 2)
    def towerSeeRight(self):
        self.motors[2].setPosition(-np.pi / 2)
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
            self.armMotors[2].setVelocity(.3)
            self.armMotors[1].setPosition(0)
            self.armMotors[6].setPosition(0.9)
            if(self.armSensors[1].getValue()<0.002):
                self.armMotors[0].setPosition(0)
                self.armMotors[0].setVelocity(0.1)#grabbing task finished
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
            
#Speed setting functions
    def setSpeed(self, left, right):
        #print("%f  %f" % (left, right))
        self.motors[0].setVelocity(left)
        self.motors[1].setVelocity(right)
    def turnRound(self, diff, threshold):
        #set speed for turning left or right
        if abs(diff) < threshold:
            #self.setSpeed(self.MAX_SPEED, self.MAX_SPEED)
            return False
        else:
            self.setSpeed(-0.3 * diff, 0.3 * diff)
            return True
    def steering(self, diff, maxDiff, mul, gain, gain2 = 0, slow = 0):
        if slow:
            self.motors[0].setAcceleration(15)
            self.motors[1].setAcceleration(15)
            if diff > 0:
                self.setSpeed(((1 - gain * (diff / maxDiff)) * mul * self.MAX_SPEED), ((1 - gain2 * (diff / maxDiff)) * mul * self.MAX_SPEED))
            else:
                self.setSpeed(((1 + gain2 * (diff / maxDiff)) * mul * self.MAX_SPEED), ((1 + gain * (diff / maxDiff)) * mul * self.MAX_SPEED))
        else:
            self.motors[0].setAcceleration(20)
            self.motors[1].setAcceleration(20)
        self.setSpeed(((1 - gain * (diff / maxDiff)) * mul * self.MAX_SPEED), ((1 + gain * (diff / maxDiff)) * mul * self.MAX_SPEED))
       
    def linePatrol(self):
        #get camera image, process it and set speed based on line patrolling algorithm
        #return False if there is no line
        leftSum = 0
        rightSum = 0
        leftSpeed = 0
        rightSpeed = 0
        HEIGHT = self.camera.getHeight()
        WIDTH = self.camera.getWidth()
        cameraData = self.camera.getImage()
        frame = np.zeros((HEIGHT, WIDTH))
        for x in range(int(0.6 * HEIGHT), HEIGHT):
            for y in range(0, WIDTH):
                frame[x][y] = int(self.camera.imageGetGray(cameraData, WIDTH, y, x))
        leftGray = int(np.average(frame[int(0.7 * HEIGHT):HEIGHT, 0:int(0.5 * WIDTH)]))
        rightGray = int(np.average(frame[int(0.7 * HEIGHT):HEIGHT, int(0.5 * WIDTH):WIDTH]))
        if leftGray + rightGray < 151:
            return False
        #print(leftGray + rightGray)
        diff = leftGray-rightGray
        #print(diff)
        if abs(diff) > 85:
            self.steering(diff, 120, 0.9, 0.2)
        else:
            self.steering(diff, 120, 0.9, 0.2 * abs(diff) / 100)
        return True
     
    def boxFound(self):
        #get camera image and find orange box, then adjust the speed to go to the box
        #return False if there is no box
        cameraData = self.camera.getImage()
        HEIGHT = self.camera.getHeight()
        WIDTH = self.camera.getWidth()
        redSum = 0
        count = 0
        for w in range(int(0.42625 * WIDTH), int(0.52 * WIDTH)):
            for h in range(int(0.25625 * HEIGHT), int(0.3375 * HEIGHT)):
                redSum += self.camera.imageGetRed(cameraData, WIDTH, w, h)
                count += 1
        if redSum / count >= 120:
            return True
        return False
        
    def bridgeFound(self):
        leftSum = 0
        midSum = 0
        rightSum = 0
        HEIGHT = self.camera.getHeight()
        WIDTH = self.camera.getWidth()
        cameraData = self.camera.getImage()
        frame = np.zeros((HEIGHT, WIDTH))
        for x in range(int(HEIGHT * 0.3), int(HEIGHT * 0.9)):
            for y in range(int(WIDTH * 0.35), int(WIDTH * 0.65)):
                frame[x][y] = int(self.camera.imageGetGray(cameraData, WIDTH, y, x))
        absX = cv2.convertScaleAbs(cv2.Sobel(frame, cv2.CV_16S, 1, 0))
        absY = cv2.convertScaleAbs(cv2.Sobel(frame, cv2.CV_16S, 0, 1))
        dst = cv2.addWeighted(absX, 0.5, absY, 0.5, 0)  
        _, filtered = cv2.threshold(dst, 50, 255, cv2.THRESH_TOZERO)
        left = np.average(filtered[int(HEIGHT * 0.35):int(HEIGHT * 0.85), int(WIDTH * 0.41):int(WIDTH * 0.47)])
        mid = np.average(filtered[int(HEIGHT * 0.35):int(HEIGHT * 0.85), int(WIDTH * 0.47):int(WIDTH * 0.54)])
        right = np.average(filtered[int(HEIGHT * 0.35):int(HEIGHT * 0.85), int(WIDTH * 0.54):int(WIDTH * 0.59)])
        #print("%d  %d  %d" % (left, mid, right))
        if mid >= 10 and left <= 2 and right <= 2:
            return True
        return False
    
    def archFound(self):
        HEIGHT = self.camera.getHeight()
        WIDTH = self.camera.getWidth()
        cameraData = self.camera.getImage()
        frame = np.zeros((HEIGHT, WIDTH))
        for x in range(0, int(0.25 * HEIGHT)):
            for y in range(int(0.25 * WIDTH), int(0.75 * WIDTH)):
                frame[x][y] = int(self.camera.imageGetBlue(cameraData, WIDTH, y, x))
        # absX = cv2.convertScaleAbs(cv2.Sobel(frame, cv2.CV_16S, 1, 0))
        # absY = cv2.convertScaleAbs(cv2.Sobel(frame, cv2.CV_16S, 0, 1))
        # dst = cv2.addWeighted(absX, 0.5, absY, 0.5, 0)  
        # _, filtered = cv2.threshold(dst, 50, 255, cv2.THRESH_TOZERO)
        left = np.average(frame[:int(0.2 * HEIGHT), int(0.28125 * WIDTH):int(0.4 * WIDTH)])
        right = np.average(frame[:int(0.2 * HEIGHT), int(0.6 * WIDTH):int(0.71875 * WIDTH)])
        # cv2.imwrite("bin1.jpg", frame[:int(0.2 * HEIGHT), int(0.28125 * WIDTH):int(0.4 * WIDTH)])
        # cv2.imwrite("bin2.jpg", frame[:int(0.2 * HEIGHT), int(0.6 * WIDTH):int(0.71875 * WIDTH)])
        #print("%d  %d" % (left, right))
        if left > 80 and right > 80:
            return True
        return False

    def colourPatrol(self):
        HEIGHT = self.camera.getHeight()
        WIDTH = self.camera.getWidth()
        cameraData = self.camera.getImage()
        frame = np.zeros((HEIGHT, WIDTH))
        frame_gray = np.zeros((HEIGHT, WIDTH))
        frame_red = np.zeros((HEIGHT, WIDTH))
        for x in range(int(0.8 * HEIGHT), HEIGHT):
            for y in range(0, WIDTH):
                frameData = int(self.camera.imageGetGray(cameraData, WIDTH, y, x))
                if self.colourFlag == 1:
                    if frameData in range(180, 186):
                        frame[x][y] = 255
                    else:
                        frame[x][y] = 0
                elif self.colourFlag == 2:
                    if frameData in range(109, 115):
                        frame[x][y] = 255
                    else:
                        frame[x][y] = 0
                elif self.colourFlag == 3:
                    if frameData in range(160, 166):
                        frame[x][y] = 255
                    else:
                        frame[x][y] = 0
                else:
                    frame[x][y] = frameData
                if x in range(int(0.95 * HEIGHT), HEIGHT):
                    frame_red[x][y] = int(self.camera.imageGetRed(cameraData, WIDTH, y, x))
                    frame_gray[x][y] = int(self.camera.imageGetGray(cameraData, WIDTH, y, x))

        leftGray = int(np.average(frame[int(0.8 * HEIGHT):HEIGHT, 0:int(0.5 * WIDTH)]))
        rightGray = int(np.average(frame[int(0.8 * HEIGHT):HEIGHT, int(0.5 * WIDTH):WIDTH]))
        diff = leftGray - rightGray
        red = int(np.average(frame_red[int(0.95 * HEIGHT):HEIGHT, :]))
        #Finished?
        if int(np.average(frame_gray[int(0.95*HEIGHT):HEIGHT, int(0.2 * WIDTH):int(0.8 * WIDTH)])) in range(93, 95) and red > 190:
            # return False
            return True
        #determine whether the robot reaches the colour box
        if not self.colourFlag:
            if leftGray == rightGray and leftGray in range(182, 184) and red == 252:   #yellow
                self.colourFlag = 1
            elif leftGray == rightGray and leftGray in range(111, 113) and red == 252:   #red
                self.colourFlag = 2
            elif leftGray == rightGray and leftGray in range(162, 164) and red == 197:   #violet
                self.colourFlag = 3
            else:
                if abs(diff) <= 60:
                    if self.directionReg == 0:
                        self.steering(diff, 120, 0.65, 0.2 * abs(diff) / 80)
                    elif self.directionReg == 1:
                        if leftGray < 90 and rightGray < 90 and abs(diff) < 10:
                            diff = 120
                            self.steering(diff, 120, 0.6, 0.5)
                        elif leftGray >= 110 and rightGray >= 110:
                            self.directionReg = 0
                            self.steering(diff, 120, 0.65, 0.2 * abs(diff) / 80)
                        else:
                            self.steering(diff, 120, 0.65, 0.2 * abs(diff) / 80)

                    elif self.directionReg == 2:
                        if leftGray < 90 and rightGray < 90 and abs(diff) < 10:
                            diff = -120
                            self.steering(diff, 120, 0.6, 0.5)
                        elif leftGray >= 110 and rightGray >= 110:
                            self.directionReg = 0
                            self.steering(diff, 120, 0.65, 0.2 * abs(diff) / 80)
                        else:
                            self.steering(diff, 120, 0.65, 0.2 * abs(diff) / 80)
                else:
                    if diff > 0:
                        self.directionReg = 1
                    else:
                        self.directionReg = 2
                    self.steering(diff, 120, 0.6, 0.2)
        else:
            # print(diff)
            # print("Reg: " + str(self.directionReg))
            # print("left: %d, right: %d" % (leftGray, rightGray))
            mul = 0.65
            if abs(diff) <= 80:
                if self.directionReg == 0:
                    self.steering(diff, 120, mul, 0.2 * abs(diff) / 80)
                elif self.directionReg == 1:
                    if leftGray < 10 and rightGray < 10 and abs(diff) < 10:
                        diff = 120
                        self.steering(diff, 120, mul, 0.5)
                    elif leftGray >= 60 and rightGray >= 60:
                        self.directionReg = 0
                        self.steering(diff, 120, mul, 0.2 * abs(diff) / 120)
                    else:
                        self.steering(diff, 120, mul, 0.2 * abs(diff) / 120)

                elif self.directionReg == 2:
                    if leftGray < 10 and rightGray < 10 and abs(diff) < 10:
                        diff = -120
                        self.steering(diff, 120, mul, 0.5)
                    elif leftGray >= 60 and rightGray >= 60:
                        self.directionReg = 0
                        self.steering(diff, 120, mul, 0.2 * abs(diff) / 120)
                    else:
                        self.steering(diff, 120, mul, 0.2 * abs(diff) / 120)
            else:
                if diff > 0:
                    self.directionReg = 1
                else:
                    self.directionReg = 2
                self.steering(diff, 160, mul, 0.2)
        return False

if __name__ == "__main__":
    TIME_STEP = 32
    vehicle = Vehicle()
    vehicle.towerRestore()
    target = -np.pi / 2

    t1Start = False
    t2Turned = False
    t2BoxFound1 = False
    t2BoxFound2 = False
    t2Released = False
    t3Turned1 = False
    t3Turned2 = False
    t3BridgeFound1 = False
    t3BridgeFound2 = False
    t4Turned1 = False
    t4Turned2 = False
    t4ArchFound1 = False
    t4ArchFound2 = False
    t6Finished = False
    vehicle.setStage(0)
    while vehicle.robot.step(TIME_STEP) != -1:
        stage = vehicle.getStage()
        if stage == 0:
            if vehicle.pickFood():
                if not t1Start:
                    t1Start = True
                    vehicle.timeReg = time.time()
                if vehicle.linePatrol():
                    if time.time() - vehicle.timeReg > 4:
                        vehicle.setStage(1)
                    else:
                        vehicle.motors[0].setAcceleration(1)
                        vehicle.motors[1].setAcceleration(1)

        elif stage == 1:
            if not vehicle.linePatrol():
                vehicle.setStage(2)
        elif stage == 2:
            vehicle.motors[0].setAcceleration(3)
            vehicle.motors[1].setAcceleration(3)
            if not t2BoxFound1:
                vehicle.towerSeeRight()
                if vehicle.setBool(vehicle.boxFound()):
                    t2BoxFound1 = True
                else:
                    vehicle.setSpeed(vehicle.MAX_SPEED * 0.8, vehicle.MAX_SPEED * 0.8)
            else:
                if not t2BoxFound2:
                    if vehicle.getBool() == False and vehicle.boxFound() == True:
                        t2BoxFound2 = True
                        vehicle.timeReg = time.time()
                    else:
                        vehicle.setSpeed(-0.1 * vehicle.MAX_SPEED, -0.1 * vehicle.MAX_SPEED)
                        vehicle.setBool(vehicle.boxFound())
                else:
                    if time.time() - vehicle.timeReg < 0.5:
                        vehicle.setSpeed(0, 0)
                    else:
                        target = -np.pi / 2
                        if not t2Turned:
                            if not vehicle.turnRound(target - vehicle.getCompass(), 0.0001):
                                t2Turned = True
                        else:
                            vehicle.towerRestore()
                            if vehicle.getDistanceValue(1) > 600 and t2Released == False:
                                vehicle.setSpeed(0.06 * vehicle.MAX_SPEED, 0.06 * vehicle.MAX_SPEED)
                            else:
                                if not t2Released:
                                    if vehicle.releaseFood():
                                        t2Released = True
                                    else:
                                        vehicle.setSpeed(0, 0)  
                                else:
                                    if vehicle.getDistanceValue(1) > 900:
                                        vehicle.setStage(3)
                                        vehicle.setSpeed(0, 0) 
                                    else:
                                        vehicle.setSpeed(-0.06 * vehicle.MAX_SPEED, -0.06 * vehicle.MAX_SPEED)                 
        elif stage == 3:       
            if not t3BridgeFound1:
                target = 0
                if not t3Turned1:
                    if not vehicle.turnRound(target - vehicle.getCompass(), 0.0001):
                        t3Turned1 = True
                else:
                    vehicle.setSpeed(vehicle.MAX_SPEED * 0.6, vehicle.MAX_SPEED * 0.6)
                    vehicle.towerSeeRight()
                    if vehicle.setBool(vehicle.bridgeFound()):
                        t3BridgeFound1 = True
            else:
                if not t3BridgeFound2:
                    if vehicle.getBool() == False and vehicle.bridgeFound():
                        t3BridgeFound2 = True
                        vehicle.timeReg = time.time()
                    else:
                        vehicle.setSpeed(-0.06 * vehicle.MAX_SPEED, -0.06 * vehicle.MAX_SPEED)
                        vehicle.setBool(vehicle.bridgeFound())
                else:
                    currentTime = time.time()
                    if currentTime - vehicle.timeReg < 0.5:
                        vehicle.setSpeed(0, 0)
                    else:
                        target = np.pi / 2
                        if not t3Turned2:
                            if not (vehicle.turnRound(target - vehicle.getCompass(), 0.0001)):
                                t3Turned2 = True
                        else:
                            vehicle.towerRestore()
                            vehicle.setSpeed(0.1 * vehicle.MAX_SPEED, 0.1 * vehicle.MAX_SPEED)
                            if vehicle.getDistanceValue(2) < 900:
                                vehicle.setSpeed(0, 0)
                                vehicle.setStage(4)
        elif stage == 4:
            if not t4ArchFound1:
                target = 0
                if not t4Turned1:
                    if not vehicle.turnRound(target - vehicle.getCompass(), 0.0001):
                        t4Turned1 = True
                else:
                    vehicle.setSpeed(0.6 * vehicle.MAX_SPEED, 0.6 * vehicle.MAX_SPEED)
                    vehicle.towerSeeLeft()
                    if vehicle.setBool(vehicle.archFound()):
                        t4ArchFound1 = True
            else:  
                if not t4ArchFound2:
                    if vehicle.getBool() == False and vehicle.archFound():
                        t4ArchFound2 = True
                        vehicle.timeReg = time.time()
                    else:
                        vehicle.setSpeed(-0.06 * vehicle.MAX_SPEED, -0.06 * vehicle.MAX_SPEED)
                        vehicle.setBool(vehicle.archFound())
                else:
                    currentTime = time.time()
                    if currentTime - vehicle.timeReg < 0.5:
                        vehicle.setSpeed(0, 0)
                    else:
                        target = np.pi / 2
                        if not t4Turned2:
                            if not (vehicle.turnRound(target - vehicle.getCompass(), 0.0001)):
                                t4Turned2 = True
                        else:
                            vehicle.towerRestore()
                            vehicle.setStage(5)
        elif stage == 5:
            if vehicle.colourPatrol():
                vehicle.setStage(6)
        elif stage == 6:
            if not t6Finished:
                t6Finished = True
                vehicle.timeReg = time.time()
            else:
                if time.time() - vehicle.timeReg > 0.5:
                    vehicle.setSpeed(0,0)
