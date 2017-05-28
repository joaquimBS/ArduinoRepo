#ifndef ACCEL_GYRO_H
#define ACCEL_GYRO_H

// #include <..\project_cfg.h>

typedef struct ypr_degrees {
    float yaw;
    float pitch;
    float roll;
};

ypr_degrees ypr_deg;

/* Function prototypes */
void accel_calibration();
void accel_init();
void accel_tick();
void meansensors();
void calibration();
/***********************/

#endif
