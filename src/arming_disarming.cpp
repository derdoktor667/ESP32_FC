#include "arming_disarming.h"

// External IBUS object
extern FlyskyIBUS ibusReceiver;
// External motor objects
extern DShotRMT motorFrontRight, motorFrontLeft, motorRearRight, motorRearLeft;

// Safety states
bool armed = false;
bool failsafeActive = false;
int arming_channel_value = 0;

void handleSafetySwitches()
{
  int failsafe_channel_value = ibusReceiver.getChannel(IBUS_CHANNEL_FAILSAFE);

  // Failsafe has the highest priority
  if (failsafe_channel_value > settings.receiver.failsafeThreshold)
  {
    if (!failsafeActive)
    {
      failsafeActive = true;
      armed = false; // Force disarm
      Serial.println("FAILSAFE ACTIVATED - MOTORS STOPPED");
      // Immediately command motors to stop
      motorFrontRight.sendThrottle(0);
      motorFrontLeft.sendThrottle(0);
      motorRearRight.sendThrottle(0);
      motorRearLeft.sendThrottle(0);
    }
    return; // Skip arming logic if failsafe is active
  }
  else
  {
    if (failsafeActive)
    {
      failsafeActive = false;
      Serial.println("Failsafe deactivated");
    }
  }

  // Arming logic is only processed if failsafe is not active
  arming_channel_value = ibusReceiver.getChannel(IBUS_CHANNEL_ARMING);

  if (arming_channel_value > settings.receiver.armingThreshold && !armed)
  {
    armed = true;
    Serial.println("ARMED");
  }
  else if (arming_channel_value < settings.receiver.armingThreshold && armed)
  {
    armed = false;
    Serial.println("DISARMED");
    // Set all motors to zero throttle when disarmed
    motorFrontRight.sendThrottle(0);
    motorFrontLeft.sendThrottle(0);
    motorRearRight.sendThrottle(0);
    motorRearLeft.sendThrottle(0);
  }
}
