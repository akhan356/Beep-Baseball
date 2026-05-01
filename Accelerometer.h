// ============================================================
// Accelerometer.h
// ============================================================
#ifndef ACCELEROMETER_H
#define ACCELEROMETER_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
  int16_t x;
  int16_t y;
  int16_t z;
} AccelData_t;

bool  accel_init(void);
void  accel_read_raw(AccelData_t *data);
bool  accel_check_impact(void);
float accel_get_magnitude_g(void);
void  accel_enable_motion_wake(void);
void  accel_disable_motion_wake(void);

#endif
