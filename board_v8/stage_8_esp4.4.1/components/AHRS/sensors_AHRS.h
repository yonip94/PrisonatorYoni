/*
 * File: sensors_AHRS.h
 *
 * MATLAB Coder version            : 5.5
 * C/C++ source code generated on  : 27-Mar-2023 15:12:25
 */

/* ver 3.0 : 26/03/2023 changes:
% 1) add acc bias, sf compensation
% 2) add gyro drift compensation
% 3) valid output is initialized at 0, and turns to 1 only after correction
% is epsilon
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
extern void sensors_AHRS(
    float imu_time, float imu_acc_x_G, float imu_acc_y_G, float imu_acc_z_G,
    float imu_gyro_x_dps, float imu_gyro_y_dps, float imu_gyro_z_dps,
    float imu_rate, float mag_time, float mag_x, float mag_y, float mag_z,
    float mag_map_dec, float mag_input_valid, float static_time_for_sleep,
    float acc_bias_x, float acc_bias_y, float acc_bias_z, float acc_sf_x,
    float acc_sf_y, float acc_sf_z, float gyro_drift_x, float gyro_drift_y,
    float gyro_drift_z, float *quat_l2b1, float *quat_l2b2, float *quat_l2b3,
    float *quat_l2b4, float *Euler_l2b_psi, float *Euler_l2b_theta,
    float *Euler_l2b_phi, float *ahrs_counter, float *ahrs_output_valid,
    float *imu_static, float *activity, float *activity_class_acc_rss_var);

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
