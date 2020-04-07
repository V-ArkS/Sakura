// File:          TankGoForward.java
// Date:
// Description:
// Author:
// Modifications:

// You may need to add other webots classes such as
//  import com.cyberbotics.webots.controller.DistanceSensor;
//  import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.Robot;
import com.cyberbotics.webots.controller.Motor;
// Here is the main class of your controller.
// This class defines how to initialize and how to run your controller.
public class TankGoForward {
  public static void main(String[] args) {

    int TIME_STEP = 64;
    Robot robot = new Robot();
    Motor[] wheels = new Motor[2];
    String[] wheelsNames = {"left motor", "right motor"};
    for (int i = 0; i < 2; i++) {
      wheels[i] = robot.getMotor(wheelsNames[i]);
      wheels[i].setPosition(Double.POSITIVE_INFINITY);
      wheels[i].setVelocity(0.1);
    }
    while (robot.step(TIME_STEP) != -1) {
    
    };
  }
}
