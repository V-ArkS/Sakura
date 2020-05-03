from controller import Robot
robot = Robot()
motor = robot.getMotor('motor1')
TIME_STEP = 64
while robot.step(TIME_STEP) != -1:
    motor.setVelocity(0.5)
    motor.setPosition(-1.4)
