#include "defines.h"

void init_boat() {
  pinMode(LED_FIX, OUTPUT);
  pinMode(LED_STATUS, OUTPUT);
  pinMode(RC_THROTTLE_PIN, INPUT);
  pinMode(RC_STEERING_PIN, INPUT);
  pinMode(RC_MODE_PIN, INPUT);

  Serial.begin(115200);
  gpsSerial.begin(9600);
  Wire.begin();
  hmc5883l_init();

  escLeft.attach(ESC_LEFT_PIN, ESC_MIN_US, ESC_MAX_US);
  escRight.attach(ESC_RIGHT_PIN, ESC_MIN_US, ESC_MAX_US);

  stop_motors();
  delay(1000);
}

void init_startup_parameters() {
  Serial.println(F("Waiting for GPS fix..."));
  unsigned long start = millis();

  while (millis() - start < 120000UL) {
    read_gps();
    read_hmc5883l();
    if (gps_fix_valid) {
      Serial.println(F("GPS fix acquired."));
      digitalWrite(LED_FIX, HIGH);
      return;
    }
  }

  Serial.println(F("GPS fix timeout. Bench mode continue."));
}

void esc_arm() {
  escLeft.writeMicroseconds(ESC_STOP_US);
  escRight.writeMicroseconds(ESC_STOP_US);
  delay(3000);
}

void stop_motors() {
  escLeft.writeMicroseconds(ESC_STOP_US);
  escRight.writeMicroseconds(ESC_STOP_US);
}

void hmc5883l_init() {
  Wire.beginTransmission(HMC5883L_ADDR);
  Wire.write(0x00);
  Wire.write(0x70);
  Wire.endTransmission();

  Wire.beginTransmission(HMC5883L_ADDR);
  Wire.write(0x01);
  Wire.write(0x20);
  Wire.endTransmission();

  Wire.beginTransmission(HMC5883L_ADDR);
  Wire.write(0x02);
  Wire.write(0x00);
  Wire.endTransmission();
}

void hmc5883l_read_raw(int16_t *mx, int16_t *my, int16_t *mz) {
  Wire.beginTransmission(HMC5883L_ADDR);
  Wire.write(0x03);
  Wire.endTransmission();
  Wire.requestFrom(HMC5883L_ADDR, 6);

  if (Wire.available() == 6) {
    int16_t x = (int16_t)(Wire.read() << 8 | Wire.read());
    int16_t z = (int16_t)(Wire.read() << 8 | Wire.read());
    int16_t y = (int16_t)(Wire.read() << 8 | Wire.read());
    *mx = x;
    *my = y;
    *mz = z;
  }
}
