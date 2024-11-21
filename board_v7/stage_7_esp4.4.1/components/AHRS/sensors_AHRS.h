/*
 * File: sensors_AHRS.h
 *
 * MATLAB Coder version            : 5.1
 * C/C++ source code generated on  : 31-Mar-2022 07:55:09
 */

#pragma once

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>
#ifdef __cplusplus

extern "C" {

#endif

  /* Function Declarations */
  extern void sensors_AHRS(float imu_time, float imu_acc_x_G, float imu_acc_y_G,
    float imu_acc_z_G, float imu_gyro_x_dps, float imu_gyro_y_dps, float
    imu_gyro_z_dps, float imu_rate, float mag_time, float mag_x, float mag_y,
    float mag_z, float mag_map_dec, float mag_input_valid, float
    static_time_for_sleep, float *quat_l2b1, float *quat_l2b2, float *quat_l2b3,
    float *quat_l2b4, float *Euler_l2b_psi, float *Euler_l2b_theta, float
    *Euler_l2b_phi, float *ahrs_counter, float *valid, float *imu_static, float *
    activity, float *activity_class_acc_rss_var);
  extern void sensors_AHRS_initialize(void);
  extern void sensors_AHRS_terminate(void);

#ifdef __cplusplus

}
#endif

/*
 * File trailer for sensors_AHRS.h
 *
 * [EOF]
 */
