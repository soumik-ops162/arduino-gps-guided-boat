#include "defines.h"
#include "waypoints.h"

void read_gps() {
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }

  if (gps.location.isUpdated()) {
    currentLat = gps.location.lat();
    currentLon = gps.location.lng();
    new_gps_data = true;
  }
  if (gps.speed.isUpdated()) {
    groundSpeedMps = gps.speed.mps();
  }
  if (gps.course.isUpdated()) {
    currentCourseGPS = gps.course.deg();
  }

  gps_fix_valid = gps.location.isValid() && gps.location.age() < 3000;
  digitalWrite(LED_FIX, gps_fix_valid ? HIGH : LOW);
}

void read_hmc5883l() {
  int16_t mx = 0, my = 0, mz = 0;
  hmc5883l_read_raw(&mx, &my, &mz);

  float x = (float)(mx - magOffsetX);
  float y = (float)(my - magOffsetY);

  float heading = atan2(y, x) * 180.0 / PI;

  const float magneticDeclinationDeg = 0.0;
  heading += magneticDeclinationDeg;
  currentHeadingDeg = wrap_360(heading);
}

void read_rc_inputs() {
  rcThrottleUs = pulseIn(RC_THROTTLE_PIN, HIGH, 25000);
  rcSteeringUs = pulseIn(RC_STEERING_PIN, HIGH, 25000);
  rcModeUs = pulseIn(RC_MODE_PIN, HIGH, 25000);

  if (rcThrottleUs < 900 || rcThrottleUs > 2100) rcThrottleUs = RC_MID;
  if (rcSteeringUs < 900 || rcSteeringUs > 2100) rcSteeringUs = RC_MID;
  if (rcModeUs < 900 || rcModeUs > 2100) rcModeUs = RC_MIN;
}

void update_mode() {
  autopilotMode = (rcModeUs > MODE_AUTOPILOT_THRESHOLD);
  digitalWrite(LED_STATUS, autopilotMode ? HIGH : LOW);
}

void navigation_update() {
  if (!gps_fix_valid || current_wp >= total_waypoints) {
    return;
  }

  wpBearing = get_bearing_deg(currentLat, currentLon, wps[current_wp].lat, wps[current_wp].lon);
  wpDistance = get_distance_m(currentLat, currentLon, wps[current_wp].lat, wps[current_wp].lon);

  if (wpDistance <= WP_RADIUS_METERS) {
    current_wp++;
    reset_PID();

    if (current_wp >= total_waypoints) {
      mission_finished = true;
      stop_motors();
    }
  }
}

double get_distance_m(double lat1, double lon1, double lat2, double lon2) {
  const double R = 6371000.0;
  double phi1 = radians(lat1);
  double phi2 = radians(lat2);
  double dphi = radians(lat2 - lat1);
  double dlambda = radians(lon2 - lon1);

  double a = sin(dphi / 2.0) * sin(dphi / 2.0) +
             cos(phi1) * cos(phi2) *
             sin(dlambda / 2.0) * sin(dlambda / 2.0);
  double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
  return R * c;
}

double get_bearing_deg(double lat1, double lon1, double lat2, double lon2) {
  double phi1 = radians(lat1);
  double phi2 = radians(lat2);
  double dlambda = radians(lon2 - lon1);

  double y = sin(dlambda) * cos(phi2);
  double x = cos(phi1) * sin(phi2) - sin(phi1) * cos(phi2) * cos(dlambda);

  return wrap_360(degrees(atan2(y, x)));
}

double wrap_360(double angle) {
  while (angle < 0.0) angle += 360.0;
  while (angle >= 360.0) angle -= 360.0;
  return angle;
}

double wrap_180(double angle) {
  while (angle > 180.0) angle -= 360.0;
  while (angle < -180.0) angle += 360.0;
  return angle;
}
