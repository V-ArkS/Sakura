"""PyControllerTest1 controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import numpy as np
from controller import PositionSensor

# create the Robot instance.
robot = Robot()
k=0
motors=[]
motorNames=["motor1","motor2","motor3","motor4","motor5","motor6"]
sensor1=robot.getPositionSensor("sensor1")
sensor1.enable(64)
sensor2=robot.getPositionSensor("sensor2")
sensor2.enable(64)
sensor3=robot.getPositionSensor("sensor3")
sensor3.enable(64)
sensor4=robot.getPositionSensor("sensor4")
sensor4.enable(64)
for i in range(6):
    motors.append(robot.getMotor(motorNames[i]))
    motors[i].setPosition(0)
    motors[i].setVelocity(1)

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getMotor('motorname')
#  ds = robot.getDistanceSensor('dsname')
#  ds.enable(timestep)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    if(k==0):
        motors[0].setPosition(0.04)#upper-arm extend
        motors[1].setPosition(0.93)#upper-arm rotate
        if (sensor1.getValue()>0.92):
            motors[2].setPosition(0.038)#mid-arm extend
            motors[3].setPosition(0.06)#forearm extend
            if (sensor2.getValue()>0.059):
                motors[4].setPosition(0.005)#grab the box
                motors[5].setPosition(0.005)
                if((sensor3.getValue()>=0.004998) or (sensor4.getValue()>=0.004998)):
                    motors[2].setPosition(0)#raise the box
                    k=1
                #Add the code controlling the rover get close to the edge of pond
                #motors[4].setPosition(0)#release the box
                #motors[5].setPosition(0)
    if(k==1):
        motors[3].setPosition(0)
        motors[1].setPosition(0)
        if(sensor1.getValue()<0.002):
            motors[0].setPosition(0)
    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
   

# Enter here exit cleanup code.
