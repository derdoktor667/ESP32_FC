#include "motor_control.h"
#include "flight_controller.h"
#include <DShotRMT.h>

// External motor objects
void setupMotors()
{
  motorFrontRight.begin();
  motorFrontLeft.begin();
  motorRearRight.begin();
  motorRearLeft.begin();
  Serial.println("DShotRMT initialized successfully!");

  // Send zero throttle to all motors to arm them (DShot requires this)
  motorFrontRight.sendThrottle(0);
  motorFrontLeft.sendThrottle(0);
  motorRearRight.sendThrottle(0);
  motorRearLeft.sendThrottle(0);
}

void sendMotorCommands(int throttle, float pid_output_roll, float pid_output_pitch, float pid_output_yaw, bool armed)
{
  // Only send throttle commands if armed
  if (armed)
  {
    // Motor Mixing (adjust these values based on your quadcopter's configuration)
    // Assuming X-quad configuration
    int throttleFrontRight = throttle - pid_output_roll + pid_output_pitch + pid_output_yaw; // Front Right
    int throttleFrontLeft = throttle + pid_output_roll + pid_output_pitch - pid_output_yaw;  // Front Left
    int throttleRearRight = throttle - pid_output_roll - pid_output_pitch - pid_output_yaw;  // Rear Right
    int throttleRearLeft = throttle + pid_output_roll - pid_output_pitch + pid_output_yaw;   // Rear Left

    // Constrain motor values to DShot range
    throttleFrontRight = constrain(throttleFrontRight, settings.dshotThrottle.min, settings.dshotThrottle.max);
    throttleFrontLeft = constrain(throttleFrontLeft, settings.dshotThrottle.min, settings.dshotThrottle.max);
    throttleRearRight = constrain(throttleRearRight, settings.dshotThrottle.min, settings.dshotThrottle.max);
    throttleRearLeft = constrain(throttleRearLeft, settings.dshotThrottle.min, settings.dshotThrottle.max);

    motorFrontRight.sendThrottle(throttleFrontRight);
    motorFrontLeft.sendThrottle(throttleFrontLeft);
    motorRearRight.sendThrottle(throttleRearRight);
    motorRearLeft.sendThrottle(throttleRearLeft);
  }
  else
  {
    // Ensure motors are off when disarmed
    motorFrontRight.sendThrottle(0);
    motorFrontLeft.sendThrottle(0);
    motorRearRight.sendThrottle(0);
    motorRearLeft.sendThrottle(0);
  }
}
