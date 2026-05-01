// ============================================================
// Accelerometer.cpp
// ============================================================
#include "Accelerometer.h"
#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#define MPU_ADDR            0x68
#define REG_WHO_AM_I        0x75
#define REG_PWR_MGMT_1      0x6B
#define REG_PWR_MGMT_2      0x6C
#define REG_ACCEL_CONFIG    0x1C
#define REG_ACCEL_XOUT_H    0x3B
#define REG_INT_ENABLE      0x38
#define REG_INT_PIN_CFG     0x37
#define REG_MOT_THR         0x1F
#define REG_MOT_DUR         0x20
#define REG_MOT_DETECT_CTRL 0x69
#define REG_INT_STATUS      0x3A


#define IMPACT_THRESHOLD_G  2.5f
#define TAP_CEILING_G       5.5f
#define SPIKE_DELTA_G       0.18f
#define IMPACT_DEBOUNCE_MS  130

static float prevMagnitudeG = 0.0f;
static float lastMagnitudeG = 0.0f;


static void i2c_write(uint8_t reg, uint8_t data) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}

static bool i2c_read(uint8_t reg, uint8_t *buffer, uint8_t len) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;

  uint8_t count = Wire.requestFrom((uint8_t)MPU_ADDR, len);
  if (count != len) return false;

  for (uint8_t i = 0; i < len; i++) {
    buffer[i] = Wire.read();
  }
  return true;
}


bool accel_init(void) {
  uint8_t who = 0;
  if (!i2c_read(REG_WHO_AM_I, &who, 1)) return false;
  if (who != 0x68) return false;


  i2c_write(REG_PWR_MGMT_1, 0x00);
  delay(100);

  
  i2c_write(REG_ACCEL_CONFIG, 0x10);
  delay(10);

  
  i2c_write(REG_INT_PIN_CFG, 0x00);

  
  uint8_t dummy;
  i2c_read(REG_INT_STATUS, &dummy, 1);

  return true;
}


void accel_read_raw(AccelData_t *data) {
  uint8_t buf[6];
  if (!i2c_read(REG_ACCEL_XOUT_H, buf, 6)) {
    data->x = 0; data->y = 0; data->z = 0;
    return;
  }
  data->x = (int16_t)((buf[0] << 8) | buf[1]);
  data->y = (int16_t)((buf[2] << 8) | buf[3]);
  data->z = (int16_t)((buf[4] << 8) | buf[5]);
}


float accel_get_magnitude_g(void) {
  AccelData_t raw;
  accel_read_raw(&raw);

  const float sensitivity = 4096.0f;  
  float x = raw.x / sensitivity;
  float y = raw.y / sensitivity;
  float z = raw.z / sensitivity;

  prevMagnitudeG = lastMagnitudeG;
  lastMagnitudeG = sqrtf(x*x + y*y + z*z);
  return lastMagnitudeG;
}


bool accel_check_impact(void) {
  static uint32_t lastImpactTime = 0;
  uint32_t now = millis();

  float magnitude = lastMagnitudeG;
  float delta     = magnitude - prevMagnitudeG;

  if (magnitude > IMPACT_THRESHOLD_G &&
      magnitude < TAP_CEILING_G      &&
      delta     > SPIKE_DELTA_G) {
    if (now - lastImpactTime > IMPACT_DEBOUNCE_MS) {
      lastImpactTime = now;
      return true;
    }
  }
  return false;
}


void accel_enable_motion_wake(void) {
  uint8_t dummy;
  i2c_read(REG_INT_STATUS, &dummy, 1);
  delay(10);

  i2c_write(REG_MOT_THR, 3);
  i2c_write(REG_MOT_DUR, 1);
  i2c_write(REG_MOT_DETECT_CTRL, 0x15);
  i2c_write(REG_INT_ENABLE, 0x40);
  i2c_write(REG_INT_PIN_CFG, 0x00);

  i2c_write(REG_PWR_MGMT_1, 0x20);
  i2c_write(REG_PWR_MGMT_2, 0x07);

  delay(150);
}


void accel_disable_motion_wake(void) {
  i2c_write(REG_PWR_MGMT_1, 0x00);
  delay(100);

  i2c_write(REG_PWR_MGMT_2, 0x00);
  i2c_write(REG_INT_ENABLE, 0x00);

  i2c_write(REG_ACCEL_CONFIG, 0x10);
  delay(10);

  uint8_t dummy;
  i2c_read(REG_INT_STATUS, &dummy, 1);
}
