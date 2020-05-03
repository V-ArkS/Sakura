from controller import Robot
from controller import Keyboard
from controller import Camera

TIME_STEP = 1
robot = Robot()
camera = Camera("camera")
camera.enable(int(robot.getBasicTimeStep()))
motors = []
motorNames = ['left motor', 'right motor', 'tower rotational motor']
for i in range(3):
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
    #tower camera rotation
    if (key1 == 81 or key2 == 81):
        motors[2].setVelocity(0.8)
    elif (key1 == 69 or key2 == 69):
        motors[2].setVelocity(-0.8)
    else:
        motors[2].setVelocity(0)
    #Forward
    if (key1==87 or key2 == 87):
        if SPEED < 5:
            SPEED += 0.01
            leftSpeed = SPEED
            rightSpeed = SPEED
        if (key1 == 65 or key2 == 65):
            leftSpeed = SPEED * 0.4
            rightSpeed = SPEED * 0.8
        elif (key1 == 68 or key2 == 68):
            leftSpeed = SPEED * 0.8
            rightSpeed = SPEED * 0.4  
    #Break
    elif (key1 == 83 or key2 == 83):
        if SPEED > 0:
            SPEED -= 0.02
            leftSpeed = SPEED
            rightSpeed = SPEED
        else:
            SPEED = 0
    elif (key1 == 65 or key2 == 65):
        if SPEED == 0:
            rightSpeed = 0.4
            leftSpeed = -0.4
        else:
            leftSpeed = SPEED * 0.4
            rightSpeed = SPEED * 0.8
    elif (key1 == 68 or key2 == 68):
        if SPEED == 0:
            rightSpeed = -0.4
            leftSpeed = 0.4
        else:
            leftSpeed = SPEED * 0.8
            rightSpeed = SPEED * 0.4
    else:
        leftSpeed = SPEED
        rightSpeed = SPEED
    
    motors[0].setVelocity(leftSpeed)
    motors[1].setVelocity(rightSpeed)