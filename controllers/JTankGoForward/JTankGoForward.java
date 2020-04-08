import com.cyberbotics.webots.controller.Robot;
import com.cyberbotics.webots.controller.Motor;

public class JTankGoForward {
  public static void main(String[] args) {

    int TIME_STEP = 64;
    Robot robot = new Robot();
    Motor[] wheels = new Motor[2];
    String[] wheelsNames = {"left motor", "right motor"};
    for (int i = 0; i < 2; i++) {
      wheels[i] = robot.getMotor(wheelsNames[i]);
      wheels[i].setPosition(Double.POSITIVE_INFINITY);
      wheels[i].setVelocity(0.3);
    }
    while (robot.step(TIME_STEP) != -1) {
    
    };
  }
}