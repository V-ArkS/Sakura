#include <webots/robot.h>
#include <webots/motor.h>

#define TIME_STEP 64


int main(int argc, char **argv) {
  wb_robot_init();
  WbDeviceTag wheels[2];
  char wheels_names[2][16] = {"left motor", "right motor"};
  for (int i = 0; i < 2; i++) {
    wheels[i] = wb_robot_get_device(wheels_names[i]);
    wb_motor_set_position(wheels[i], INFINITY);
  }
  wb_motor_set_velocity(wheels[0], 0.3);
  wb_motor_set_velocity(wheels[1], 0.3);
  while (wb_robot_step(TIME_STEP) != -1) {
    
    
  };

  wb_robot_cleanup();

  return 0;
}
