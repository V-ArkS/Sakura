# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import numpy as np
from controller import PositionSensor

# create the Robot instance.
robot = Robot()
k=0
motors=[]
motorNames=["arm_motor_1","arm_motor_2","arm_motor_3","arm_motor_4","arm_motor_5","arm_motor_6","arm_motor_7","arm_motor_8"]
sensor1=robot.getPositionSensor("arm_position_sensor_1")
sensor1.enable(64)
sensor2=robot.getPositionSensor("arm_position_sensor_2")
sensor2.enable(64)
sensor3=robot.getPositionSensor("arm_position_sensor_3")
sensor3.enable(64)
sensor4=robot.getPositionSensor("arm_position_sensor_4")
sensor4.enable(64)
sensor5=robot.getPositionSensor("arm_position_sensor_5")
sensor5.enable(64)
sensor6=robot.getPositionSensor("arm_position_sensor_6")
sensor6.enable(64)
sensor7=robot.getPositionSensor("arm_position_sensor_7")
sensor7.enable(64)
sensor8=robot.getPositionSensor("arm_position_sensor_8")
sensor8.enable(64)
for i in range(8):
    motors.append(robot.getMotor(motorNames[i]))
    motors[i].setPosition(0)
    motors[i].setVelocity(.5)

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
        motors[0].setPosition(0.039)#upper-arm extend
        motors[1].setPosition(0.93)#upper-arm rotate
        if (sensor2.getValue()>0.92):
            motors[2].setPosition(0.038)#mid-arm extend
            #print(sensor3.getValue())
            if (sensor3.getValue()>0.0379):
                motors[4].setPosition(0.005)#grab the box
                motors[5].setPosition(0.005)
                if((sensor5.getValue()>=0.00269) or (sensor6.getValue()>=0.0031)):
                    k=1
    if(k==1):
        motors[2].setPosition(0)#raise the box
        motors[2].setVelocity(.1)
        motors[1].setPosition(0)
        motors[6].setPosition(0.9)
        if(sensor2.getValue()<0.002):
            motors[0].setPosition(0)
            motors[0].setVelocity(0.01)#grabbing task finished
            if(sensor1.getValue()<0.003):
                motors[6].setPosition(0)
                if(sensor7.getValue()<0.002):
                    print("Grabbing Finished!")
                    k=2
    if(k==2):
        motors[6].setPosition(0.9)
        if(sensor7.getValue()>0.899):
            motors[3].setPosition(0.06)
            motors[7].setPosition(0.06)
            if((sensor4.getValue()>0.059) and (sensor8.getValue()>0.059)):
                motors[4].setPosition(0)
                motors[5].setPosition(0)
                if((sensor5.getValue()<=0.00269) or (sensor6.getValue()>=0.0031)):
                    k=3
    if(k==3):
        motors[3].setPosition(0)
        motors[7].setPosition(0)
        motors[6].setPosition(0)
    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
# Enter here exit cleanup code.