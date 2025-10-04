#include "serial_logger.h"
#include "flight_controller.h"
#include "flight_modes.h"

// External attitude variables
extern float roll, pitch, yaw;
// External target setpoints
extern float target_roll, target_pitch, target_yaw;
// External arming status
extern bool armed;
// External failsafe status
extern bool failsafeActive;
// External arming channel value
extern int arming_channel_value;
// External current flight mode
extern FlightMode current_flight_mode;

unsigned long last_print_time = 0;

void printFlightStatus()
{
  Serial.print("AccX: ");
  Serial.print(imuSensor.readings.accelerometer.x);
  Serial.print("\tAccY: ");
  Serial.print(imuSensor.readings.accelerometer.y);
  Serial.print("\tAccZ: ");
  Serial.print(imuSensor.readings.accelerometer.z);
  Serial.print("\tGyroX: ");
  Serial.print(imuSensor.readings.gyroscope.x);
  Serial.print("\tGyroY: ");
  Serial.print(imuSensor.readings.gyroscope.y);
  Serial.print("\tGyroZ: ");
  Serial.print(imuSensor.readings.gyroscope.z);

  Serial.print("\tRoll: ");
  Serial.print(roll);
  Serial.print("\tPitch: ");
  Serial.print(pitch);
  Serial.print("\tYaw: ");
  Serial.print(yaw);

  static int ibus_throttle = 0;
  ibus_throttle = ibusReceiver.getChannel(IBUS_CHANNEL_THROTTLE);
  Serial.print("\tIBUS Channel 0: ");
  Serial.print(ibus_throttle);

  Serial.print("\tTarget Roll: ");
  Serial.print(target_roll);
  Serial.print("\tTarget Pitch: ");
  Serial.print(target_pitch);
  Serial.print("\tTarget Yaw: ");
  Serial.print(target_yaw);
  Serial.print("\tArmed: ");
  Serial.print(armed ? "YES" : "NO");
  Serial.print("\tFailsafe: ");
  Serial.print(failsafeActive ? "ACTIVE" : "inactive");
  Serial.print("\tArming Channel: ");
  Serial.print(arming_channel_value);
  Serial.print("\tFlight Mode: ");
  switch (current_flight_mode)
  {
  case ACRO_MODE:
    Serial.print("ACRO");
    break;
  case ANGLE_MODE:
    Serial.print("ANGLE");
    break;
  default:
    Serial.print("UNKNOWN");
    break;
  }
  Serial.println();
}
