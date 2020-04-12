from controller import Robot
from controller import Camera
TIME_STEP = 64
robot = Robot()
#Define motors
motors = []
motorNames = ['left motor', 'right motor']
for i in range(2):
    motors.append(robot.getMotor(motorNames[i]))
    motors[i].setPosition(float('inf'))
    motors[i].setVelocity(0.0)
    motors[i].setAcceleration(25)

camera = Camera('camera')
camera.enable(int(robot.getBasicTimeStep()))
SPEED = 2
while robot.step(TIME_STEP) != -1:
    leftSpeed = 0.0
    rightSpeed = 0.0
    #get image and process it
    image = camera.getImage()
    leftSum = 0
    rightSum = 0
    cameraData = camera.getImage()

    for x in range(0, camera.getWidth()):
        for y in range(int(camera.getHeight() * 0.9), camera.getHeight()):
            gray = Camera.imageGetGray(cameraData, camera.getWidth(), x, y)
            if x < camera.getWidth() / 2:
                leftSum += gray
            else:
                rightSum += gray
                
        if leftSum > rightSum + 1000:
            leftSpeed = SPEED * (1 - 0.8 * (leftSum - rightSum) / 460000)
            rightSpeed = SPEED * (1 - 0.6 * (leftSum - rightSum) / 460000)
        elif rightSum > leftSum + 1000:
            leftSpeed = SPEED * (1 - 0.6 * (rightSum - leftSum) / 460000)
            rightSpeed = SPEED * (1 - 0.8 * (rightSum - leftSum) / 460000)
        else:
            leftSpeed = SPEED
            rightSpeed = SPEED
    
    motors[0].setVelocity(leftSpeed);
    motors[1].setVelocity(rightSpeed);