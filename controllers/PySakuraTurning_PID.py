from controller import Robot
from controller import Camera
from controller import Compass
from controller import PositionSensor
import numpy as np
#import cv2, time

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
        global error_integral
        global previous_diff#set variable as global will accelerate the speed
        error_integral += diff * ts;
        error_derivative = (previous_diff - diff) / ts;
        Vc = 0.5* diff  + 0.00 * error_derivative + 0.05* error_integral ;
        #set as 0.35/0.001/0.02 for mass=40kg
        #set as 0.5/0/0.05 respectively for mass=400kg
        if Vc > 1:
            Vc = 1
        if abs(diff) < 0.001:
            self.setSpeed(0, 0)
            return False
        else:
            self.setSpeed(-Vc, Vc)
            previous_diff = diff
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
    ts = 1/TIME_STEP
    error_integral = 0
    error_derivative = 0
    previous_diff = 0
    vehicle = Vehicle()
    vehicle.towerRestore()
    while vehicle.robot.step(TIME_STEP) != -1:
        target = np.pi / 2
        if not (vehicle.turnRound(target - vehicle.getCompass())):
            vehicle.setSpeed(0.1, 0.1)
        