#include "defines.h"
#include "waypoints.h"

SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN);
TinyGPSPlus gps;
Servo escLeft;
Servo escRight;

volatile bool gps_fix_valid = false;
volatile bool new_gps_data = false;

double currentLat = 0.0;
double currentLon = 0.0;
double currentCourseGPS = 0.0;
double groundSpeedMps = 0.0;
double currentHeadingDeg = 0.0;
double wpDistance = 0.0;
double wpBearing = 0.0;
uint8_t current_wp = 0;
uint8_t total_waypoints = 0;
bool mission_finished = false;

bool autopilotMode = false;
int rcThrottleUs = RC_MID;
int rcSteeringUs = RC_MID;
int rcModeUs = RC_MIN;

float pid_p = 0.0f;
float pid_i = 0.0f;
float pid_d = 0.0f;
float heading_integrator = 0.0f;
float heading_prev_error = 0.0f;

int16_t magOffsetX = 0;
int16_t magOffsetY = 0;

unsigned long lastPidMs = 0;
unsigned long lastDebugMs = 0;

void setup() {
  init_boat();
  total_waypoints = sizeof(wps) / sizeof(wps[0]);
  print_header();
  esc_arm();
  init_startup_parameters();
}

void loop() {
  read_gps();
  read_hmc5883l();
  read_rc_inputs();
  update_mode();

  if (mission_finished) {
    finish_mission();
    return;
  }

  if (autopilotMode) {
    if (millis() - lastPidMs >= PID_INTERVAL_MS) {
      lastPidMs = millis();
      navigation_update();
      autopilot_control();
    }
  } else {
    manual_control();
    reset_PID();
  }

  if (millis() - lastDebugMs >= DEBUG_INTERVAL_MS) {
    lastDebugMs = millis();
    send_to_ground();
  }
}
