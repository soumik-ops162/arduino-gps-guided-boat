#include "defines.h"

void print_header() {
  Serial.println(F("mode\twp\tlat\tlon\thead\twpBear\twpDist\tspd\tp\ti\td\trcMode"));
}

void send_to_ground() {
  Serial.print(autopilotMode ? F("AUTO") : F("MAN"));
  Serial.print('\t');
  Serial.print(current_wp);
  Serial.print('\t');
  Serial.print(currentLat, 6);
  Serial.print('\t');
  Serial.print(currentLon, 6);
  Serial.print('\t');
  Serial.print(currentHeadingDeg, 1);
  Serial.print('\t');
  Serial.print(wpBearing, 1);
  Serial.print('\t');
  Serial.print(wpDistance, 1);
  Serial.print('\t');
  Serial.print(groundSpeedMps, 2);
  Serial.print('\t');
  Serial.print(pid_p, 2);
  Serial.print('\t');
  Serial.print(pid_i, 2);
  Serial.print('\t');
  Serial.print(pid_d, 2);
  Serial.print('\t');
  Serial.println(rcModeUs);
}
