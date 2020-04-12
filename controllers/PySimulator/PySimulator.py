from controller import Robot
from controller import Keyboard

TIME_STEP = 1
robot = Robot()
motors = []
motorNames = ['left motor', 'right motor']
for i in range(2):
    motors.append(robot.getMotor(motorNames[i]))
    motors[i].setPosition(float('inf'))
    motors[i].setVelocity(0.0)
    keyboard = robot.getKeyboard()
    keyboard.enable(TIME_STEP)
    leftSpeed = 0.0
    rightSpeed = 0.0
    SPEED = 0.0
while robot.step(TIME_STEP) != -1:
    key1=keyboard.getKey()
    key2=keyboard.getKey()
    #Forward
    if (key1==87 or key2 == 87):
        if SPEED < 5:
            SPEED += 0.02
            leftSpeed = SPEED
            rightSpeed = SPEED
        if (key1 == 65 or key2 == 65):
            leftSpeed = SPEED * 0.9
            rightSpeed = SPEED
        elif (key1 == 68 or key2 == 68):
            leftSpeed = SPEED
            rightSpeed = SPEED * 0.9      
    #Break
    elif (key1 == 83 or key2 == 83):
        if SPEED > 0:
            SPEED -= 0.05
            leftSpeed = SPEED
            rightSpeed = SPEED
        else:
            SPEED = 0
    elif (key1 == 65 or key2 == 65):
        if SPEED == 0:
            rightSpeed = 0.7
            leftSpeed = -0.7
        else:
            leftSpeed = SPEED * 0.85
            rightSpeed = SPEED
    elif (key1 == 68 or key2 == 68):
        if SPEED == 0:
            rightSpeed = -0.7
            leftSpeed = 0.7
        else:
            leftSpeed = SPEED
            rightSpeed = SPEED * 0.85
    else:
        leftSpeed = SPEED;
        rightSpeed = SPEED;
    # if (key==65):
        # leftSpeed *= 0.8
    # if (key==68):
        # rihgtSpeed *= 0.8
    motors[0].setVelocity(leftSpeed)
    motors[1].setVelocity(rightSpeed)