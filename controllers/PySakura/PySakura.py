from controller import Robot
from controller import Camera
from controller import Compass
from controller import PositionSensor
import numpy as np
import cv2, time

class Vehicle:
    stage = 0
    MAX_SPEED = 2
    time_tmp = 0
    robot = Robot()
    motors = []
    armMotors = []
    armPs = []
    camera = Camera('camera')
    compass = Compass('compass')
    ds = robot.getDistanceSensor('distance sensor')

    #tmp
    picked = False

    def __init__(self):
        #get motors, camera and initialise them.
        motorNames = ['left motor', 'right motor', 'tower rotational motor']
        for i in range(3):
            self.motors.append(self.robot.getMotor(motorNames[i]))
            self.motors[i].setPosition(float('inf'))
            if i <= 1:
                self.motors[i].setVelocity(0)
            else:
                self.motors[i].setVelocity(0.2)
                self.motors[i].setPosition(0)
        
        #tower motor
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
    def towerSeeRight(self):
        self.motors[2].setPosition(-np.pi / 2)
    def towerRestore(self):
        self.motors[2].setPosition(0)
    def releaseFood(self):
        pass
            
        
        
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
            print("straight")
        else:
            self.motors[0].setAcceleration(20)
            self.motors[1].setAcceleration(20)
            print("not straight")
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

    def boxPatrol(self):
        #get camera image and find orange box, then adjust the speed to go to the box
        #return False if there is no box
        self.motors[0].setAcceleration(1)
        self.motors[1].setAcceleration(1)
        image = self.camera.getImage()
        cameraData = self.camera.getImage()
        HEIGHT = self.camera.getHeight()
        WIDTH = self.camera.getWidth()
        redSum = 0
        self.setSpeed(1, 1)
        for w in range(int(0.1 * WIDTH), int(0.3 * WIDTH)):
            for h in range(int(0.24 * HEIGHT), int(0.33 * HEIGHT)):
                redSum += self.camera.imageGetRed(cameraData, WIDTH, w, h)
        print(redSum)
        if redSum >= 65000:
            self.setSpeed(0, 0)
            return True
        return False
        
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
    
if __name__ == "__main__":
    TIME_STEP = 64
    vehicle = Vehicle()
    vehicle.towerRestore()
    target = -np.pi / 2
    turnFlag = False
    released = False
    vehicle.setStage(1)
    vehicle.time_tmp = time.time()
    while vehicle.robot.step(TIME_STEP) != -1:
        stage = vehicle.getStage()
        if stage == 1:
            if not vehicle.linePatrol():
                pass
        elif stage == 2:
            if not released:
                vehicle.towerSeeRight()
                if vehicle.boxPatrol():
                    released = True
            else:
                target = -np.pi / 2
                if not (vehicle.turnRound(target - vehicle.getCompass())):
                    vehicle.towerRestore()
                    if vehicle.getDistanceValue() > 900:
                        vehicle.setSpeed(0.1, 0.1)
                    else:
                        vehicle.setSpeed(0, 0)
                        vehicle.releaseFood()
                        vehicle.setStage(3)
        elif stage == 3:
            target = 0
            if not (vehicle.turnRound(target - vehicle.getCompass())):
                vehicle.setSpeed(0.1, 0.1)
        # Switch stage for different task, if a task is completed, increase the stage by
        # using vehicle.setstage()

        # Task 2
        # if not (vehicle.turnRound(target - vehicle.getCompass())):
        #     if not released:
        #         vehicle.releaseFood()
        #         released = True


        # Task 4
        # if (vehicle.getDistanceValue() < 900):
        #     turnFlag = True
        # if (turnFlag):
        #     if not vehicle.turnRound(target - vehicle.getCompass()):
        #         turnFlag = False
        #         vehicle.setSpeed(0.1, 0.1, 0.1, 0.1)
