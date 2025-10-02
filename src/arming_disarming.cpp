#include "arming_disarming.h"

// External IBUS object
extern FlyskyIBUS ibus;
// External motor objects
extern DShotRMT motor1, motor2, motor3, motor4;

// Arming status
bool armed = false;
int arming_channel_value = 0;

void handleArming()
{
  arming_channel_value = ibus.getChannel(IBUS_CHANNEL_ARMING);

  if (arming_channel_value > IBUS_ARMING_THRESHOLD && !armed)
  {
    armed = true;
    Serial.println("ARMED");
  }
  else if (arming_channel_value < IBUS_ARMING_THRESHOLD && armed)
  {
    armed = false;
    Serial.println("DISARMED");
    // Set all motors to zero throttle when disarmed
    motor1.sendThrottle(0);
    motor2.sendThrottle(0);
    motor3.sendThrottle(0);
    motor4.sendThrottle(0);
  }
}
