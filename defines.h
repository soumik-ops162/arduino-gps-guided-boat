#ifndef DEFINES_H
#define DEFINES_H

#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include <SoftwareSerial.h>
#include <TinyGPSPlus.h>

#define GPS_RX_PIN 4
#define GPS_TX_PIN 3
#define ESC_LEFT_PIN 9
#define ESC_RIGHT_PIN 10
#define RC_THROTTLE_PIN 7
#define RC_STEERING_PIN 8
#define RC_MODE_PIN 6

#define LED_FIX 12
#define LED_STATUS 13

#define HMC5883L_ADDR 0x1E

#define RC_MIN 1000
#define RC_MID 1500
#define RC_MAX 2000
#define MODE_AUTOPILOT_THRESHOLD 1500

#define ESC_MIN_US 1000
#define ESC_STOP_US 1500
#define ESC_MAX_US 2000

#define MANUAL_MIX_LIMIT 300

#define AP_BASE_THRUST_US 1620
#define AP_MIN_THRUST_US 1500
#define AP_MAX_THRUST_US 1750

#define WP_RADIUS_METERS 5.0

#define KP_HEADING 3.0f
#define KI_HEADING 0.02f
#define KD_HEADING 1.5f
#define INTEGRATOR_LIMIT 200.0f
#define HEADING_OUTPUT_LIMIT 220.0f

#define DEBUG_INTERVAL_MS 500UL
#define PID_INTERVAL_MS 100UL

typedef struct {
  double lat;
  double lon;
} Waypoint;

extern SoftwareSerial gpsSerial;
extern TinyGPSPlus gps;
extern Servo escLeft;
extern Servo escRight;

extern volatile bool gps_fix_valid;
extern volatile bool new_gps_data;
extern double currentLat;
extern double currentLon;
extern double currentCourseGPS;
extern double groundSpeedMps;
extern double currentHeadingDeg;
extern double wpDistance;
extern double wpBearing;
extern uint8_t current_wp;
extern uint8_t total_waypoints;
extern bool mission_finished;

extern bool autopilotMode;
extern int rcThrottleUs;
extern int rcSteeringUs;
extern int rcModeUs;

extern float pid_p;
extern float pid_i;
extern float pid_d;
extern float heading_integrator;
extern float heading_prev_error;

extern int16_t magOffsetX;
extern int16_t magOffsetY;

void init_boat();
void init_startup_parameters();
void read_gps();
void read_hmc5883l();
void read_rc_inputs();
void update_mode();
void navigation_update();
void autopilot_control();
void manual_control();
void differential_thrust_write(int leftUs, int rightUs);
void esc_arm();
void stop_motors();
void finish_mission();
double get_distance_m(double lat1, double lon1, double lat2, double lon2);
double get_bearing_deg(double lat1, double lon1, double lat2, double lon2);
double wrap_360(double angle);
double wrap_180(double angle);
float PID_heading(float errorDeg);
void reset_PID();
void send_to_ground();
void print_header();
void hmc5883l_init();
void hmc5883l_read_raw(int16_t *mx, int16_t *my, int16_t *mz);

#endif
