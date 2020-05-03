class Vehicle:
    stage = 0
    MAX_SPEED = 2
    speed = 0
    motors = []
    camera = Camera('camera')
    #compass?
    def __init__(self):
        #get motors, camera and initialise them.
        pass
    def getStage(self):
        return stage
    def setStage(self, stage_num):
        self.stage = stage_num
    def getSpeed(self):
        return self.speed
    def towerTurnLeft(self):
        #turn the tower to the left for 90 degrees
        pass
    def towerTurnRight(self):
        #turn the tower to the right for 90 degrees
        pass

#Speed setting functions
    def setSpeed(self, speed1, speed2, speed3, speed4):
        #set speed for four tracks
        pass
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
        #return False if there is no beidge
        pass
    def archPatrol(self):
        #get camera image and find arch, then adjust the speed to go to the arch
        #return False if there is no arch
        pass
    def colourPatrol(self):
        #for task 5
        pass
    def turnLeft(self):
        #set speed for turning left (90 degrees)
        #compass?
        pass
    def turnRight(self):
        #set speed for turning right (90 degrees)
        pass
