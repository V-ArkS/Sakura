from controller import Robot 
import random
robot = Robot()
motor = robot.getMotor('motor1')
TIME_STEP = 64 
number = random.randint(1,3)
while robot.step(TIME_STEP) != -1:
    motor.setVelocity(0.5)
    if number == 1:
        motor.setPosition(-1.4)
    elif number == 2:
        motor.setPosition(-0.7)
    elif number == 3:
        motor.setPosition(0) 