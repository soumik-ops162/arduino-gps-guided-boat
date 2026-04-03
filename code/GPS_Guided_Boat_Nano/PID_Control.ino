#include "defines.h"

float PID_heading(float errorDeg) {
  heading_integrator += errorDeg;
  heading_integrator = constrain(heading_integrator, -INTEGRATOR_LIMIT, INTEGRATOR_LIMIT);

  float derivative = errorDeg - heading_prev_error;

  pid_p = KP_HEADING * errorDeg;
  pid_i = KI_HEADING * heading_integrator;
  pid_d = KD_HEADING * derivative;

  float output = pid_p + pid_i + pid_d;
  output = constrain(output, -HEADING_OUTPUT_LIMIT, HEADING_OUTPUT_LIMIT);

  heading_prev_error = errorDeg;
  return output;
}

void reset_PID() {
  heading_integrator = 0.0f;
  heading_prev_error = 0.0f;
  pid_p = 0.0f;
  pid_i = 0.0f;
  pid_d = 0.0f;
}
