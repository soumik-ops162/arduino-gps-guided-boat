#include "defines.h"

void manual_control() {
  int throttleOffset = rcThrottleUs - RC_MID;
  int steeringOffset = rcSteeringUs - RC_MID;

  throttleOffset = constrain(throttleOffset, -250, 250);
  steeringOffset = constrain(steeringOffset, -MANUAL_MIX_LIMIT, MANUAL_MIX_LIMIT);

  int leftUs = ESC_STOP_US + throttleOffset + steeringOffset;
  int rightUs = ESC_STOP_US + throttleOffset - steeringOffset;

  leftUs = constrain(leftUs, ESC_MIN_US, ESC_MAX_US);
  rightUs = constrain(rightUs, ESC_MIN_US, ESC_MAX_US);

  differential_thrust_write(leftUs, rightUs);
}

void autopilot_control() {
  if (!gps_fix_valid || current_wp >= total_waypoints) {
    stop_motors();
    return;
  }

  double headingForControl = currentHeadingDeg;
  if (groundSpeedMps > 1.2 && gps.course.isValid()) {
    headingForControl = currentCourseGPS;
  }

  float headingError = (float)wrap_180(wpBearing - headingForControl);
  float correction = PID_heading(headingError);

  int leftUs = AP_BASE_THRUST_US - (int)correction;
  int rightUs = AP_BASE_THRUST_US + (int)correction;

  leftUs = constrain(leftUs, AP_MIN_THRUST_US, AP_MAX_THRUST_US);
  rightUs = constrain(rightUs, AP_MIN_THRUST_US, AP_MAX_THRUST_US);

  differential_thrust_write(leftUs, rightUs);
}

void differential_thrust_write(int leftUs, int rightUs) {
  escLeft.writeMicroseconds(leftUs);
  escRight.writeMicroseconds(rightUs);
}

void finish_mission() {
  stop_motors();
  while (true) {
    digitalWrite(LED_STATUS, HIGH);
    delay(150);
    digitalWrite(LED_STATUS, LOW);
    delay(150);
  }
}
