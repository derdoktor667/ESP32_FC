#include "motor_control.h"

// External motor objects
DShotRMT motor1(ESC_PIN_1, DSHOT300, false);
DShotRMT motor2(ESC_PIN_2, DSHOT300, false);
DShotRMT motor3(ESC_PIN_3, DSHOT300, false);
DShotRMT motor4(ESC_PIN_4, DSHOT300, false);

void setupMotors()
{
  motor1.begin();
  motor2.begin();
  motor3.begin();
  motor4.begin();
  Serial.println("DShotRMT initialized successfully!");

  // Send zero throttle to all motors to arm them (DShot requires this)
  motor1.sendThrottle(0);
  motor2.sendThrottle(0);
  motor3.sendThrottle(0);
  motor4.sendThrottle(0);
}

void sendMotorCommands(int throttle, float pid_output_roll, float pid_output_pitch, float pid_output_yaw, bool armed)
{
  // Only send throttle commands if armed
  if (armed)
  {
    // Motor Mixing (adjust these values based on your quadcopter's configuration)
    // Assuming X-quad configuration
    int motor_fr = throttle - pid_output_roll + pid_output_pitch + pid_output_yaw; // Front Right
    int motor_fl = throttle + pid_output_roll + pid_output_pitch - pid_output_yaw; // Front Left
    int motor_rr = throttle - pid_output_roll - pid_output_pitch - pid_output_yaw; // Rear Right
    int motor_rl = throttle + pid_output_roll - pid_output_pitch + pid_output_yaw; // Rear Left

    // Constrain motor values to DShot range
    motor_fr = constrain(motor_fr, DSHOT_MIN_THROTTLE, DSHOT_MAX_THROTTLE);
    motor_fl = constrain(motor_fl, DSHOT_MIN_THROTTLE, DSHOT_MAX_THROTTLE);
    motor_rr = constrain(motor_rr, DSHOT_MIN_THROTTLE, DSHOT_MAX_THROTTLE);
    motor_rl = constrain(motor_rl, DSHOT_MIN_THROTTLE, DSHOT_MAX_THROTTLE);

    motor1.sendThrottle(motor_fr);
    motor2.sendThrottle(motor_fl);
    motor3.sendThrottle(motor_rr);
    motor4.sendThrottle(motor_rl);
  }
  else
  {
    // Ensure motors are off when disarmed
    motor1.sendThrottle(0);
    motor2.sendThrottle(0);
    motor3.sendThrottle(0);
    motor4.sendThrottle(0);
  }
}
