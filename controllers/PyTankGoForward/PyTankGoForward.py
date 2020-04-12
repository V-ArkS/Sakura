from controller import Robot
TIME_STEP = 64
robot = Robot()
motors = []
motorNames = ['left motor', 'right motor']
for i in range(2):
    motors.append(robot.getMotor(motorNames[i]))
    motors[i].setPosition(float('inf'))
    motors[i].setVelocity(0.0)
 
while robot.step(TIME_STEP) != -1:
    leftSpeed = 1.0
    rightSpeed = 1.0
    motors[0].setVelocity(leftSpeed)
    motors[1].setVelocity(rightSpeed)