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
        #release food
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
    
if __name__ == "__main__":
    TIME_STEP = 64
    vehicle = Vehicle()
    vehicle.towerRestore()
    while vehicle.robot.step(TIME_STEP) != -1:
        target = np.pi / 2
        if not (vehicle.turnRound(target - vehicle.getCompass())):
            vehicle.setSpeed(0.1, 0.1)
        