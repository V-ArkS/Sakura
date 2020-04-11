import com.cyberbotics.webots.controller.Robot;
import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.Camera;

public class JTankPatrol {
 public static void main(String[] args) {

   int TIME_STEP = 64;
   Robot robot = new Robot();
   Motor[] wheels = new Motor[2];
   String[] wheelsNames = {"left motor", "right motor"};
   //define two track motors
   for (int i = 0; i < 2; i++) {
     wheels[i] = robot.getMotor(wheelsNames[i]);
     wheels[i].setPosition(Double.POSITIVE_INFINITY);
     wheels[i].setVelocity(0.2);
   }
  
   int timeStep = (int) robot.getBasicTimeStep();
   //define the camera and enable it
   Camera camera = new Camera("camera");
   camera.enable(timeStep);
   //set the default speed
   double SPEED = 0.5;
   double leftSpeed = 0;
   double rightSpeed = 0;
   while (robot.step(TIME_STEP) != -1) {
     int[] image = camera.getImage();
     int left = 0;
     int right = 0;
     //sum gray values for both parts
     for (int i=368640; i < image.length; i++) {
       int pixel = image[i];
       int gray = Camera.pixelGetGray(pixel);
       if (i % 640 < 320)
         left += gray;
       else
         right += gray;
     }
     //Compare the summation, the threshold here is 280000
     if (left > right + 300000) {
       //System.out.println("Left");
       //The speed when turning is smaller for safety
       rightSpeed = SPEED * 0.7;
       leftSpeed = 0;
     }
     else if (right > left + 300000) {
       //System.out.println("right");
       rightSpeed = 0;
       leftSpeed = SPEED * 0.7;
     }
     else {
       //System.out.println("Forward");
       rightSpeed = SPEED;
        leftSpeed = SPEED;
      }
      //set velocity every loop
     wheels[0].setVelocity(leftSpeed);
     wheels[1].setVelocity(rightSpeed);
   };
 }
}
