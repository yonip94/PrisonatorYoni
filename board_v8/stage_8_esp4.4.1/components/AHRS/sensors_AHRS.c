/*
 * File: sensors_AHRS.c
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

/* Include Files */
#include "sensors_AHRS.h"
#include "rt_nonfinite.h"
#include "sensors_AHRS_emxutil.h"
#include "sensors_AHRS_types.h"
#include "rt_defines.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <string.h>

/* Variable Definitions */
static float ahrs_Quat_l_2_b[4];

static float ahrs_gyro_bias[3];

static float ahrs_global_counter;

static float ahrs_leveling_complete;

static float imu_data_buffer[700];

static boolean_T imu_data_buffer_not_empty;

static float imu_data_counter;

static float last_activity_time;

static boolean_T isInitialized_sensors_AHRS = false;

/* Function Declarations */
static void binary_expand_op(emxArray_boolean_T *in1,
                             const emxArray_real32_T *in2, float in3, int in4);

static void rssq(const float x[300], float y[100]);

static float rt_atan2f_snf(float u0, float u1);

static float rt_powf_snf(float u0, float u1);

static void sensor_activity_classifier_init(void);

static void sensors_AHRS_init(void);

/* Function Definitions */
/*
 * Arguments    : emxArray_boolean_T *in1
 *                const emxArray_real32_T *in2
 *                float in3
 *                int in4
 * Return Type  : void
 */
static void binary_expand_op(emxArray_boolean_T *in1,
                             const emxArray_real32_T *in2, float in3, int in4)
{
  const float *in2_data;
  int i;
  int loop_ub;
  int stride_0_0;
  boolean_T *in1_data;
  in2_data = in2->data;
  i = in1->size[0] * in1->size[1];
  if (in4 == 1) {
    in1->size[1] = in2->size[1];
  } else {
    in1->size[1] = in4;
  }
  in1->size[0] = 1;
  emxEnsureCapacity_boolean_T(in1, i);
  in1_data = in1->data;
  stride_0_0 = (in2->size[1] != 1);
  if (in4 == 1) {
    loop_ub = in2->size[1];
  } else {
    loop_ub = in4;
  }
  for (i = 0; i < loop_ub; i++) {
    in1_data[i] =
        ((in2_data[i * stride_0_0] == 0.0F) && (in3 + 3.14159274F > 0.0F));
  }
}

/*
 * Arguments    : const float x[300]
 *                float y[100]
 * Return Type  : void
 */
static void rssq(const float x[300], float y[100])
{
  int k;
  for (k = 0; k < 100; k++) {
    float absxk;
    float scale;
    float t;
    float yv;
    scale = 1.29246971E-26F;
    absxk = fabsf(x[3 * k]);
    if (absxk > 1.29246971E-26F) {
      yv = 1.0F;
      scale = absxk;
    } else {
      t = absxk / 1.29246971E-26F;
      yv = t * t;
    }
    absxk = fabsf(x[3 * k + 1]);
    if (absxk > scale) {
      t = scale / absxk;
      yv = yv * t * t + 1.0F;
      scale = absxk;
    } else {
      t = absxk / scale;
      yv += t * t;
    }
    absxk = fabsf(x[3 * k + 2]);
    if (absxk > scale) {
      t = scale / absxk;
      yv = yv * t * t + 1.0F;
      scale = absxk;
    } else {
      t = absxk / scale;
      yv += t * t;
    }
    y[k] = scale * sqrtf(yv);
  }
}

/*
 * Arguments    : float u0
 *                float u1
 * Return Type  : float
 */
static float rt_atan2f_snf(float u0, float u1)
{
  float y;
  if (rtIsNaNF(u0) || rtIsNaNF(u1)) {
    y = rtNaNF;
  } else if (rtIsInfF(u0) && rtIsInfF(u1)) {
    int i;
    int i1;
    if (u0 > 0.0F) {
      i = 1;
    } else {
      i = -1;
    }
    if (u1 > 0.0F) {
      i1 = 1;
    } else {
      i1 = -1;
    }
    y = atan2f((float)i, (float)i1);
  } else if (u1 == 0.0F) {
    if (u0 > 0.0F) {
      y = RT_PIF / 2.0F;
    } else if (u0 < 0.0F) {
      y = -(RT_PIF / 2.0F);
    } else {
      y = 0.0F;
    }
  } else {
    y = atan2f(u0, u1);
  }
  return y;
}

/*
 * Arguments    : float u0
 *                float u1
 * Return Type  : float
 */
static float rt_powf_snf(float u0, float u1)
{
  float y;
  if (rtIsNaNF(u0) || rtIsNaNF(u1)) {
    y = rtNaNF;
  } else {
    float f;
    float f1;
    f = fabsf(u0);
    f1 = fabsf(u1);
    if (rtIsInfF(u1)) {
      if (f == 1.0F) {
        y = 1.0F;
      } else if (f > 1.0F) {
        if (u1 > 0.0F) {
          y = rtInfF;
        } else {
          y = 0.0F;
        }
      } else if (u1 > 0.0F) {
        y = 0.0F;
      } else {
        y = rtInfF;
      }
    } else if (f1 == 0.0F) {
      y = 1.0F;
    } else if (f1 == 1.0F) {
      if (u1 > 0.0F) {
        y = u0;
      } else {
        y = 1.0F / u0;
      }
    } else if (u1 == 2.0F) {
      y = u0 * u0;
    } else if ((u1 == 0.5F) && (u0 >= 0.0F)) {
      y = sqrtf(u0);
    } else if ((u0 < 0.0F) && (u1 > floorf(u1))) {
      y = rtNaNF;
    } else {
      y = powf(u0, u1);
    }
  }
  return y;
}

/*
 * function [activity,activity_class_acc_rss_var] =
 * sensor_activity_classifier(imu_time,imu_acc_x_G,imu_acc_y_G,imu_acc_z_G,...
 *     imu_gyro_x_dps,imu_gyro_y_dps,imu_gyro_z_dps, static_time_for_sleep)
 *
 * SENSOR_ACTIVITY_CLASSIFIER Summary of this function goes here
 *    Detailed explanation goes here
 *
 * Arguments    : void
 * Return Type  : void
 */
static void sensor_activity_classifier_init(void)
{
  imu_data_counter = 0.0F;
  memset(&imu_data_buffer[0], 0, 700U * sizeof(float));
  last_activity_time = 0.0F;
}

/*
 * function [quat_l2b1,quat_l2b2,quat_l2b3,quat_l2b4, ...
 *     Euler_l2b_psi , Euler_l2b_theta , Euler_l2b_phi, ahrs_counter,
 * ahrs_output_valid, imu_static,activity,activity_class_acc_rss_var] = ...
 *     sensors_AHRS(imu_time,imu_acc_x_G,imu_acc_y_G,imu_acc_z_G,...
 *     imu_gyro_x_dps,imu_gyro_y_dps,imu_gyro_z_dps,imu_rate,...
 *     mag_time,mag_x,mag_y,mag_z,mag_map_dec,mag_input_valid,static_time_for_sleep,...
 *     acc_bias_x,acc_bias_y,acc_bias_z,acc_sf_x,acc_sf_y,acc_sf_z, ...
 *     gyro_drift_x,gyro_drift_y,gyro_drift_z)
 *
 * sensors_AHRS gets accelerometer XYZ, gyro XYZ, imu rate
 *  and propagates AHRS quaternion & euler angles
 *  output: quat_l2b1,quat_l2b2,quat_l2b3,quat_l2b4 : quaternion 4th real,
 *  local to body
 *                Euler_l2b_psi , Euler_l2b_theta , Euler_l2b_phi : euler
 *                angles 321 , local to body
 *                valid : validity of AHRS solution
 *
 * Arguments    : void
 * Return Type  : void
 */
static void sensors_AHRS_init(void)
{
  /* 'sensors_AHRS:39' ahrs_Quat_l_2_b = [0 0 0 1]; */
  ahrs_Quat_l_2_b[0] = 0.0F;
  ahrs_Quat_l_2_b[1] = 0.0F;
  ahrs_Quat_l_2_b[2] = 0.0F;
  ahrs_Quat_l_2_b[3] = 1.0F;
  /* 'sensors_AHRS:40' ahrs_gyro_bias = [0 0 0]; */
  ahrs_gyro_bias[0] = 0.0F;
  ahrs_gyro_bias[1] = 0.0F;
  ahrs_gyro_bias[2] = 0.0F;
  /* 'sensors_AHRS:41' ahrs_leveling_complete = 0; */
  ahrs_leveling_complete = 0.0F;
  /* 'sensors_AHRS:42' ahrs_global_counter = 1; */
  ahrs_global_counter = 1.0F;
}

/*
 * function [quat_l2b1,quat_l2b2,quat_l2b3,quat_l2b4, ...
 *     Euler_l2b_psi , Euler_l2b_theta , Euler_l2b_phi, ahrs_counter,
 * ahrs_output_valid, imu_static,activity,activity_class_acc_rss_var] = ...
 *     sensors_AHRS(imu_time,imu_acc_x_G,imu_acc_y_G,imu_acc_z_G,...
 *     imu_gyro_x_dps,imu_gyro_y_dps,imu_gyro_z_dps,imu_rate,...
 *     mag_time,mag_x,mag_y,mag_z,mag_map_dec,mag_input_valid,static_time_for_sleep,...
 *     acc_bias_x,acc_bias_y,acc_bias_z,acc_sf_x,acc_sf_y,acc_sf_z, ...
 *     gyro_drift_x,gyro_drift_y,gyro_drift_z)
 *
 * sensors_AHRS gets accelerometer XYZ, gyro XYZ, imu rate
 *  and propagates AHRS quaternion & euler angles
 *  output: quat_l2b1,quat_l2b2,quat_l2b3,quat_l2b4 : quaternion 4th real,
 *  local to body
 *                Euler_l2b_psi , Euler_l2b_theta , Euler_l2b_phi : euler
 *                angles 321 , local to body
 *                valid : validity of AHRS solution
 *
 * Arguments    : float imu_time
 *                float imu_acc_x_G
 *                float imu_acc_y_G
 *                float imu_acc_z_G
 *                float imu_gyro_x_dps
 *                float imu_gyro_y_dps
 *                float imu_gyro_z_dps
 *                float imu_rate
 *                float mag_time
 *                float mag_x
 *                float mag_y
 *                float mag_z
 *                float mag_map_dec
 *                float mag_input_valid
 *                float static_time_for_sleep
 *                float acc_bias_x
 *                float acc_bias_y
 *                float acc_bias_z
 *                float acc_sf_x
 *                float acc_sf_y
 *                float acc_sf_z
 *                float gyro_drift_x
 *                float gyro_drift_y
 *                float gyro_drift_z
 *                float *quat_l2b1
 *                float *quat_l2b2
 *                float *quat_l2b3
 *                float *quat_l2b4
 *                float *Euler_l2b_psi
 *                float *Euler_l2b_theta
 *                float *Euler_l2b_phi
 *                float *ahrs_counter
 *                float *ahrs_output_valid
 *                float *imu_static
 *                float *activity
 *                float *activity_class_acc_rss_var
 * Return Type  : void
 */
void sensors_AHRS(
    float imu_time, float imu_acc_x_G, float imu_acc_y_G, float imu_acc_z_G,
    float imu_gyro_x_dps, float imu_gyro_y_dps, float imu_gyro_z_dps,
    float imu_rate, float mag_time, float mag_x, float mag_y, float mag_z,
    float mag_map_dec, float mag_input_valid, float static_time_for_sleep,
    float acc_bias_x, float acc_bias_y, float acc_bias_z, float acc_sf_x,
    float acc_sf_y, float acc_sf_z, float gyro_drift_x, float gyro_drift_y,
    float gyro_drift_z, float *quat_l2b1, float *quat_l2b2, float *quat_l2b3,
    float *quat_l2b4, float *Euler_l2b_psi, float *Euler_l2b_theta,
    float *Euler_l2b_phi, float *ahrs_counter, float *ahrs_output_valid,
    float *imu_static, float *activity, float *activity_class_acc_rss_var)
{
  static float static_start_time;
  emxArray_boolean_T *r;
  emxArray_real32_T *fcnOutput;
  float a_tmp;
  float absxk_tmp;
  float accel_norm_sq;
  float b_absxk_tmp;
  float correction_idx_0;
  float correction_idx_1;
  float correction_idx_2;
  float dt;
  float gyro_idx_0;
  float gyro_idx_1;
  float gyro_idx_2;
  float q;
  float scale;
  float t;
  float xbar;
  float *fcnOutput_data;
  int b_i;
  int i;
  int k;
  int trueCount;
  boolean_T *r1;
  (void)acc_bias_x;
  (void)acc_bias_y;
  (void)acc_bias_z;
  (void)acc_sf_x;
  (void)acc_sf_y;
  (void)acc_sf_z;
  (void)gyro_drift_x;
  (void)gyro_drift_y;
  (void)gyro_drift_z;
  if (!isInitialized_sensors_AHRS) {
    sensors_AHRS_initialize();
  }
  /*  ver 3.0 : 26/03/2023 changes: */
  /*  1) add acc bias, sf compensation */
  /*  2) add gyro drift compensation */
  /*  3) valid output is initialized at 0, and turns to 1 only after correction
   */
  /*  is epsilon */
  /*  ver 2.1 : 17/07/2022 changes */
  /*  1) if mag not valid : init_az = 0; */
  /*  ver 2.0 : 27/06/2022 changes */
  /*  1) add initialize theta phi & az by leveling + mag */
  /*  init persistent vars */
  /* 'sensors_AHRS:34' if (isempty(ahrs_global_counter)) */
  /*  ahrs_output_valid is false, untill levelling is done good */
  /* 'sensors_AHRS:47' ahrs_output_valid = 0; */
  *ahrs_output_valid = 0.0F;
  /* 'sensors_AHRS:49' [activity,activity_class_acc_rss_var] =
   * sensor_activity_classifier(imu_time,imu_acc_x_G,imu_acc_y_G,imu_acc_z_G,...
   */
  /* 'sensors_AHRS:50'
   * imu_gyro_x_dps,imu_gyro_y_dps,imu_gyro_z_dps,static_time_for_sleep); */
  /* SENSOR_ACTIVITY_CLASSIFIER Summary of this function goes here */
  /*    Detailed explanation goes here */
  /* 'sensor_activity_classifier:6' SAVED_DATA_SIZE = 100; */
  /*  data of 1 seconds at 100 hz */
  /* 'sensor_activity_classifier:7' MINIMUM_DELTA_TIME_FOR_IMU_SAVE = 0.099; */
  /* 'sensor_activity_classifier:8' MINIMUM_DELTA_TIME_FOR_ACTIVITY_CLASS = 1.0;
   */
  /* 'sensor_activity_classifier:9' ACC_VAR_THRESH = 0.01; */
  /*  initialize: */
  /* 'sensor_activity_classifier:13' if (isempty(imu_data_buffer)) */
  if (!imu_data_buffer_not_empty) {
    /* 'sensor_activity_classifier:14' imu_data_counter = 0; */
    /* 'sensor_activity_classifier:15' imu_data_buffer =
     * zeros(SAVED_DATA_SIZE,7); */
    imu_data_buffer_not_empty = true;
    /* 'sensor_activity_classifier:16' last_activity_time = 0; */
    /* 'sensor_activity_classifier:17' static_start_time = imu_time; */
    static_start_time = imu_time;
  }
  /*  initialize output vars: */
  /* 'sensor_activity_classifier:21' activity = 0; */
  *activity = 0.0F;
  /*  unknown activity */
  /* 'sensor_activity_classifier:22' activity_class_acc_rss_var = 0; */
  *activity_class_acc_rss_var = 0.0F;
  /*  check if we should go to sleep (this could be overriden later by
   * classifier) */
  /* 'sensor_activity_classifier:27' prev_time = imu_data_buffer(1,1); */
  /* 'sensor_activity_classifier:28' if (imu_time - prev_time >
   * MINIMUM_DELTA_TIME_FOR_IMU_SAVE) */
  if (imu_time - imu_data_buffer[0] > 0.099F) {
    boolean_T b;
    boolean_T b1;
    /* 'sensor_activity_classifier:29' imu_data_buffer(1,:) =
     * [imu_time,imu_acc_x_G,imu_acc_y_G,imu_acc_z_G,... */
    /* 'sensor_activity_classifier:30'
     * imu_gyro_x_dps,imu_gyro_y_dps,imu_gyro_z_dps]; */
    imu_data_buffer[0] = imu_time;
    imu_data_buffer[1] = imu_acc_x_G;
    imu_data_buffer[2] = imu_acc_y_G;
    imu_data_buffer[3] = imu_acc_z_G;
    imu_data_buffer[4] = imu_gyro_x_dps;
    imu_data_buffer[5] = imu_gyro_y_dps;
    imu_data_buffer[6] = imu_gyro_z_dps;
    /* 'sensor_activity_classifier:31' imu_data_buffer =
     * circshift(imu_data_buffer,1); */
    b = true;
    i = 0;
    b1 = true;
    trueCount = 0;
    for (b_i = 0; b_i < 7; b_i++) {
      int i1;
      int i2;
      int pageroot;
      boolean_T b2;
      boolean_T b3;
      pageroot = b_i * 100;
      if (b1) {
        b1 = false;
        trueCount = pageroot % 100 * 7 + pageroot / 100;
      } else {
        trueCount += 700;
        trueCount %= 699;
        if (trueCount == 0) {
          trueCount = 699;
        }
      }
      if (b) {
        b = false;
        i = (pageroot + 99) % 100 * 7 + (pageroot + 99) / 100;
      } else {
        i += 700;
        i %= 699;
        if (i == 0) {
          i = 699;
        }
      }
      xbar = imu_data_buffer[i];
      b2 = true;
      i1 = 0;
      b3 = true;
      i2 = 0;
      for (k = 98; k >= 0; k--) {
        if (b3) {
          b3 = false;
          i2 = pageroot + k;
          i2 = i2 % 100 * 7 + i2 / 100;
        } else {
          i2 += -7;
          if (i2 < 0) {
            i2 += 699;
          }
        }
        if (b2) {
          b2 = false;
          i1 = (pageroot + k) + 1;
          i1 = i1 % 100 * 7 + i1 / 100;
        } else {
          i1 += -7;
          if (i1 < 0) {
            i1 += 699;
          }
        }
        imu_data_buffer[i1] = imu_data_buffer[i2];
      }
      imu_data_buffer[trueCount] = xbar;
    }
    /* 'sensor_activity_classifier:32' imu_data_counter = imu_data_counter + 1;
     */
    imu_data_counter++;
  }
  /*  else - quit function */
  /*  determine if we should activate classifier */
  /* 'sensor_activity_classifier:38' if (imu_data_counter > SAVED_DATA_SIZE ) */
  if (imu_data_counter > 100.0F) {
    /* 'sensor_activity_classifier:39' if (imu_time - last_activity_time >
     * MINIMUM_DELTA_TIME_FOR_ACTIVITY_CLASS) */
    if (imu_time - last_activity_time > 1.0F) {
      float fv[300];
      float acc_rss[100];
      float acc_rss_var;
      /* 'sensor_activity_classifier:40' acc_rss =
       * rssq(imu_data_buffer(:,2:4),2); */
      for (i = 0; i < 100; i++) {
        fv[3 * i] = imu_data_buffer[7 * i + 1];
        fv[3 * i + 1] = imu_data_buffer[7 * i + 2];
        fv[3 * i + 2] = imu_data_buffer[7 * i + 3];
      }
      rssq(fv, acc_rss);
      /* 'sensor_activity_classifier:41' acc_rss_var = var(acc_rss); */
      xbar = acc_rss[0];
      for (k = 0; k < 99; k++) {
        xbar += acc_rss[k + 1];
      }
      xbar /= 100.0F;
      q = 0.0F;
      for (k = 0; k < 100; k++) {
        t = acc_rss[k] - xbar;
        q += t * t;
      }
      acc_rss_var = q / 99.0F;
      /* 'sensor_activity_classifier:42' if (acc_rss_var > ACC_VAR_THRESH) */
      if (acc_rss_var > 0.01F) {
        /* 'sensor_activity_classifier:43' activity = 1; */
        *activity = 1.0F;
        /*  dynamic */
        /* 'sensor_activity_classifier:44' static_start_time = imu_time; */
        static_start_time = imu_time;
      } else {
        /* 'sensor_activity_classifier:45' else */
        /* 'sensor_activity_classifier:46' activity = 2; */
        *activity = 2.0F;
        /*  static!             */
      }
      /* 'sensor_activity_classifier:48' last_activity_time = imu_time; */
      last_activity_time = imu_time;
      /* 'sensor_activity_classifier:49' activity_class_acc_rss_var =
       * acc_rss_var; */
      *activity_class_acc_rss_var = acc_rss_var;
    }
    /* 'sensor_activity_classifier:51' if (imu_time - static_start_time >
     * static_time_for_sleep) */
    if (imu_time - static_start_time > static_time_for_sleep) {
      /* 'sensor_activity_classifier:52' activity = 4; */
      *activity = 4.0F;
      /*  go to sleep */
    }
  }
  /*  init outputs */
  /* 'sensors_AHRS:53' valid = 0; */
  /* 'sensors_AHRS:54' quat_l2b1 = 0; */
  *quat_l2b1 = 0.0F;
  /* 'sensors_AHRS:54' quat_l2b2 = 0; */
  *quat_l2b2 = 0.0F;
  /* 'sensors_AHRS:54' quat_l2b3 = 0; */
  *quat_l2b3 = 0.0F;
  /* 'sensors_AHRS:54' quat_l2b4 = 1; */
  *quat_l2b4 = 1.0F;
  /* 'sensors_AHRS:55' Euler_l2b_psi = 0; */
  *Euler_l2b_psi = 0.0F;
  /* 'sensors_AHRS:55' Euler_l2b_theta = 0; */
  *Euler_l2b_theta = 0.0F;
  /* 'sensors_AHRS:55' Euler_l2b_phi = 0; */
  *Euler_l2b_phi = 0.0F;
  /* 'sensors_AHRS:56' ahrs_counter = 0; */
  *ahrs_counter = 0.0F;
  /* 'sensors_AHRS:57' imu_static = 0; */
  *imu_static = 0.0F;
  /*  set constant dt */
  /* 'sensors_AHRS:59' dt = 1/imu_rate; */
  dt = 1.0F / imu_rate;
  /*  tuning */
  /* 'sensors_AHRS:62' w_accel = 0.3; */
  /*  tune */
  /* 'sensors_AHRS:63' w_gyro_bias = 0.01; */
  /*  tune */
  /* 'sensors_AHRS:64' w_mag = 0.5; */
  /*  tune */
  /* 'sensors_AHRS:65' SPIN_RATE_THRESH = 1; */
  /* 0.14; % tune */
  /*  constants */
  /* 'sensors_AHRS:68' fifty_dps = 0.873; */
  /* 'sensors_AHRS:69' gamma_e = 9.7803; */
  /* 'sensors_AHRS:70' DEG2RAD = pi/180; */
  /* 'sensors_AHRS:72' gyro = [imu_gyro_x_dps,imu_gyro_y_dps,imu_gyro_z_dps]; */
  /* 'sensors_AHRS:73' gyro = gyro*DEG2RAD; */
  gyro_idx_0 = imu_gyro_x_dps * 0.0174532924F;
  gyro_idx_1 = imu_gyro_y_dps * 0.0174532924F;
  gyro_idx_2 = imu_gyro_z_dps * 0.0174532924F;
  /* 'sensors_AHRS:74' accel_G = [imu_acc_x_G,imu_acc_y_G,imu_acc_z_G]; */
  /* 'sensors_AHRS:75' correction = [ 0 0 0]; */
  /* 'sensors_AHRS:76' accel_norm_sq = norm(accel_G)^2; */
  scale = 1.29246971E-26F;
  correction_idx_0 = 0.0F;
  xbar = fabsf(imu_acc_x_G);
  if (xbar > 1.29246971E-26F) {
    a_tmp = 1.0F;
    scale = xbar;
  } else {
    t = xbar / 1.29246971E-26F;
    a_tmp = t * t;
  }
  correction_idx_1 = 0.0F;
  absxk_tmp = fabsf(imu_acc_y_G);
  if (absxk_tmp > scale) {
    t = scale / absxk_tmp;
    a_tmp = a_tmp * t * t + 1.0F;
    scale = absxk_tmp;
  } else {
    t = absxk_tmp / scale;
    a_tmp += t * t;
  }
  correction_idx_2 = 0.0F;
  b_absxk_tmp = fabsf(imu_acc_z_G);
  if (b_absxk_tmp > scale) {
    t = scale / b_absxk_tmp;
    a_tmp = a_tmp * t * t + 1.0F;
    scale = b_absxk_tmp;
  } else {
    t = b_absxk_tmp / scale;
    a_tmp += t * t;
  }
  a_tmp = scale * sqrtf(a_tmp);
  accel_norm_sq = a_tmp * a_tmp;
  /* 'sensors_AHRS:77' upper_accel_limit =  1.1; */
  /* 'sensors_AHRS:78' lower_accel_limit =  0.9; */
  /* 'sensors_AHRS:80' if( accel_norm_sq < 0.001) */
  emxInit_boolean_T(&r);
  emxInit_real32_T(&fcnOutput);
  if (!(accel_norm_sq < 0.001F)) {
    float spinRate;
    /* 'sensors_AHRS:85' spinRate = norm(gyro); */
    scale = 1.29246971E-26F;
    xbar = fabsf(gyro_idx_0);
    if (xbar > 1.29246971E-26F) {
      spinRate = 1.0F;
      scale = xbar;
    } else {
      t = xbar / 1.29246971E-26F;
      spinRate = t * t;
    }
    xbar = fabsf(gyro_idx_1);
    if (xbar > scale) {
      t = scale / xbar;
      spinRate = spinRate * t * t + 1.0F;
      scale = xbar;
    } else {
      t = xbar / scale;
      spinRate += t * t;
    }
    xbar = fabsf(gyro_idx_2);
    if (xbar > scale) {
      t = scale / xbar;
      spinRate = spinRate * t * t + 1.0F;
      scale = xbar;
    } else {
      t = xbar / scale;
      spinRate += t * t;
    }
    spinRate = scale * sqrtf(spinRate);
    /*  check if imu is static enough  */
    /*  this will be a condition to make first leveling and make ahrs output */
    /*  valid */
    /*  after ahrs output valid, this will indicate if to apply accelerometer
     * correction and */
    /*  gyro bias correction */
    /* 'sensors_AHRS:93' if ( (accel_norm_sq > lower_accel_limit *
     * lower_accel_limit & ... */
    /* 'sensors_AHRS:94'         accel_norm_sq < upper_accel_limit *
     * upper_accel_limit) & ... */
    /* 'sensors_AHRS:95'         spinRate < SPIN_RATE_THRESH) */
    if ((accel_norm_sq > 0.809999943F) && (accel_norm_sq < 1.21F) &&
        (spinRate < 1.0F)) {
      /* 'sensors_AHRS:97' imu_static = 1; */
      *imu_static = 1.0F;
    }
    /*  if leveling is not complete and imu_static is false - return no valid */
    /* 'sensors_AHRS:101' if (imu_static == 0 & ahrs_leveling_complete == 0) */
    if ((!(*imu_static == 0.0F)) || (!(ahrs_leveling_complete == 0.0F))) {
      float attitude_ptf_idx_2;
      float k_idx_0;
      float k_idx_1;
      float k_idx_2;
      float mag_data_idx_0;
      float mag_data_idx_1;
      float mag_data_idx_2;
      float q_tmp_idx_1;
      float q_tmp_idx_2;
      float q_tmp_idx_3;
      int local_mag_valid;
      /* 'sensors_AHRS:105' if (imu_static == 1 & ahrs_leveling_complete == 0)
       */
      if ((*imu_static == 1.0F) && (ahrs_leveling_complete == 0.0F)) {
        float init_az;
        float init_phi;
        float init_theta;
        float sin_ea_idx_0;
        float sin_ea_idx_1;
        float sin_ea_idx_2;
        /* 'sensors_AHRS:107' init_theta = atan(imu_acc_x_G/norm([imu_acc_y_G,
         * imu_acc_z_G])); */
        scale = 1.29246971E-26F;
        if (absxk_tmp > 1.29246971E-26F) {
          q = 1.0F;
          scale = absxk_tmp;
        } else {
          t = absxk_tmp / 1.29246971E-26F;
          q = t * t;
        }
        if (b_absxk_tmp > scale) {
          t = scale / b_absxk_tmp;
          q = q * t * t + 1.0F;
          scale = b_absxk_tmp;
        } else {
          t = b_absxk_tmp / scale;
          q += t * t;
        }
        init_theta = atanf(imu_acc_x_G / (scale * sqrtf(q)));
        /* 'sensors_AHRS:108' init_phi = atan2(-imu_acc_y_G, -imu_acc_z_G); */
        init_phi = rt_atan2f_snf(-imu_acc_y_G, -imu_acc_z_G);
        /* 'sensors_AHRS:109' if (mag_input_valid == 1) */
        if (mag_input_valid == 1.0F) {
          float D_tmp[9];
          float mag_NE[3];
          /* 'sensors_AHRS:110' init_az =
           * MNF_calc_psi_from_levelled_mag([mag_x,mag_y,mag_z],[0,init_theta,init_phi],mag_map_dec);
           */
          /* CALC_PSI_FROM_LEVELLED_MAG Summary of this function goes here */
          /*    Detailed explanation goes here */
          /* 'MNF_calc_psi_from_levelled_mag:5' DEG2RAD = pi/180; */
          /*  Leshem_magnetic_declination_deg = 4; */
          /*  if (nargin == 2) */
          /*      mag_declination_rad = Leshem_magnetic_declination_deg*DEG2RAD;
           */
          /*  % else if (nargin == 3) */
          /*  end         */
          /*  rotate raw magnetometer data to NE     */
          /* 'MNF_calc_psi_from_levelled_mag:14' dcm_l2m =
           * MNF_Euler2DCM(0,attitude_ptf(2),attitude_ptf(3)); */
          /* function D = euler2dcm(psi, theta, phi) */
          /* 'MNF_Euler2DCM:4' if nargin == 3 */
          /* 'MNF_Euler2DCM:5' psi = varargin{1}; */
          /* 'MNF_Euler2DCM:6' theta = varargin{2}; */
          /* 'MNF_Euler2DCM:7' phi = varargin{3}; */
          /* 'MNF_Euler2DCM:11' if nargin == 1 */
          /* 'MNF_Euler2DCM:19' D = [cos(psi)*cos(theta) sin(psi)*cos(theta)
           * -sin(theta) */
          /* 'MNF_Euler2DCM:20' cos(psi)*sin(theta)*sin(phi)-sin(psi)*cos(phi)
           * sin(psi)*sin(theta)*sin(phi)+cos(psi)*cos(phi) sin(phi)*cos(theta)
           */
          /* 'MNF_Euler2DCM:21' cos(psi)*sin(theta)*cos(phi)+sin(psi)*sin(phi)
           * sin(psi)*sin(theta)*cos(phi)-cos(psi)*sin(phi)
           * cos(theta)*cos(phi)]; */
          xbar = cosf(init_theta);
          q = sinf(init_theta);
          scale = sinf(init_phi);
          absxk_tmp = cosf(init_phi);
          b_absxk_tmp = 0.0F * q;
          /* 'MNF_calc_psi_from_levelled_mag:15' dcm_m2l = dcm_l2m'; */
          /* 'MNF_calc_psi_from_levelled_mag:17' if (size(mag_xyz,1) == 3) */
          /* 'MNF_calc_psi_from_levelled_mag:19' else */
          /* 'MNF_calc_psi_from_levelled_mag:20' mag_NE = dcm_m2l*mag_xyz'; */
          D_tmp[0] = xbar;
          D_tmp[3] = 0.0F * xbar;
          D_tmp[6] = -q;
          D_tmp[1] = q * scale - 0.0F * absxk_tmp;
          D_tmp[4] = b_absxk_tmp * scale + absxk_tmp;
          D_tmp[7] = scale * xbar;
          D_tmp[2] = q * absxk_tmp + 0.0F * scale;
          D_tmp[5] = b_absxk_tmp * absxk_tmp - scale;
          D_tmp[8] = xbar * absxk_tmp;
          for (i = 0; i < 3; i++) {
            mag_NE[i] = (mag_x * D_tmp[3 * i] + mag_y * D_tmp[3 * i + 1]) +
                        mag_z * D_tmp[3 * i + 2];
          }
          /* 'MNF_calc_psi_from_levelled_mag:22' psi = ...     */
          /* 'MNF_calc_psi_from_levelled_mag:23' (-atan2(mag_NE(2),mag_NE(1))
           * + (mag_declination_rad)); */
          init_az = -rt_atan2f_snf(mag_NE[1], mag_NE[0]) + mag_map_dec;
        } else {
          /* 'sensors_AHRS:111' else */
          /* 'sensors_AHRS:112' init_az = 0; */
          init_az = 0.0F;
        }
        /* 'sensors_AHRS:115' ahrs_Quat_l_2_b =
         * MNF_Euler2Quat([init_az,init_theta,init_phi])'; */
        /*  Q_L2B - real element is in fourth row */
        /*  euler_angles=[psi;theta;phi] */
        /*  if abs(euler_angles(2,:))>=(pi/2) */
        /*      error('Pitch must be in (-pi/2,pi/2)'); */
        /*  end */
        /* 'MNF_Euler2Quat:11' euler_angles_FTP = flip(euler_angles); */
        /* 'MNF_Euler2Quat:12' sin_ea=sin(euler_angles_FTP/2); */
        /* 'MNF_Euler2Quat:13' cos_ea=cos(euler_angles_FTP/2); */
        attitude_ptf_idx_2 = init_phi / 2.0F;
        sin_ea_idx_0 = sinf(attitude_ptf_idx_2);
        attitude_ptf_idx_2 = cosf(attitude_ptf_idx_2);
        t = attitude_ptf_idx_2;
        attitude_ptf_idx_2 = init_theta / 2.0F;
        sin_ea_idx_1 = sinf(attitude_ptf_idx_2);
        attitude_ptf_idx_2 = cosf(attitude_ptf_idx_2);
        scale = attitude_ptf_idx_2;
        attitude_ptf_idx_2 = init_az / 2.0F;
        sin_ea_idx_2 = sinf(attitude_ptf_idx_2);
        attitude_ptf_idx_2 = cosf(attitude_ptf_idx_2);
        /*  Q_L2B=[sin_ea(1,:).*cos_ea(2,:).*cos_ea(3,:)-cos_ea(1,:).*sin_ea(2,:).*sin_ea(3,:);
         */
        /*      cos_ea(1,:).*sin_ea(2,:).*cos_ea(3,:)+sin_ea(1,:).*cos_ea(2,:).*sin_ea(3,:);
         */
        /*      -sin_ea(1,:).*sin_ea(2,:).*cos_ea(3,:)+cos_ea(1,:).*cos_ea(2,:).*sin_ea(3,:);
         */
        /*      cos_ea(1,:).*cos_ea(2,:).*cos_ea(3,:)+sin_ea(1,:).*sin_ea(2,:).*sin_ea(3,:)];
         */
        /* 'MNF_Euler2Quat:20'
         * Q_L2B=[sin_ea(1).*cos_ea(2).*cos_ea(3)-cos_ea(1).*sin_ea(2).*sin_ea(3);
         */
        /* 'MNF_Euler2Quat:21'
         * cos_ea(1).*sin_ea(2).*cos_ea(3)+sin_ea(1).*cos_ea(2).*sin_ea(3); */
        /* 'MNF_Euler2Quat:22'
         * -sin_ea(1).*sin_ea(2).*cos_ea(3)+cos_ea(1).*cos_ea(2).*sin_ea(3); */
        /* 'MNF_Euler2Quat:23'
         * cos_ea(1).*cos_ea(2).*cos_ea(3)+sin_ea(1).*sin_ea(2).*sin_ea(3)]; */
        xbar = t * sin_ea_idx_1;
        q = sin_ea_idx_0 * scale;
        scale *= t;
        t = q * attitude_ptf_idx_2 - xbar * sin_ea_idx_2;
        absxk_tmp = xbar * attitude_ptf_idx_2 + q * sin_ea_idx_2;
        b_absxk_tmp = -sin_ea_idx_0 * sin_ea_idx_1 * attitude_ptf_idx_2 +
                      scale * sin_ea_idx_2;
        xbar = scale * attitude_ptf_idx_2 +
               sin_ea_idx_0 * sin_ea_idx_1 * sin_ea_idx_2;
        /* 'MNF_Euler2Quat:25' Q_L2B=Quat_normalize_4st(Q_L2B); */
        /* 'Quat_normalize_4st:4' Q_norm=sqrt(sum(Q.^2)); */
        q = sqrtf(((rt_powf_snf(t, 2.0F) + rt_powf_snf(absxk_tmp, 2.0F)) +
                   rt_powf_snf(b_absxk_tmp, 2.0F)) +
                  rt_powf_snf(xbar, 2.0F));
        /* 'Quat_normalize_4st:5' Q_norm=repmat(Q_norm,4,1); */
        /* 'Quat_normalize_4st:6' normalized_Q=Q./Q_norm; */
        t /= q;
        absxk_tmp /= q;
        b_absxk_tmp /= q;
        xbar /= q;
        /* 'Quat_normalize_4st:8' if normalized_Q(4)<0 */
        if (xbar < 0.0F) {
          /* 'Quat_normalize_4st:9' normalized_Q=-normalized_Q; */
          t = -t;
          absxk_tmp = -absxk_tmp;
          b_absxk_tmp = -b_absxk_tmp;
          xbar = -xbar;
        }
        ahrs_Quat_l_2_b[0] = t;
        ahrs_Quat_l_2_b[1] = absxk_tmp;
        ahrs_Quat_l_2_b[2] = b_absxk_tmp;
        ahrs_Quat_l_2_b[3] = xbar;
        /* 'sensors_AHRS:116' ahrs_leveling_complete = 1; */
        ahrs_leveling_complete = 1.0F;
      }
      /* 'sensors_AHRS:119' q = ahrs_Quat_l_2_b(:); */
      /* 'sensors_AHRS:120' mag_data = [0 0 0]; */
      mag_data_idx_0 = 0.0F;
      mag_data_idx_1 = 0.0F;
      mag_data_idx_2 = 0.0F;
      /* 'sensors_AHRS:121' local_mag_valid = 0; */
      local_mag_valid = 0;
      /* 'sensors_AHRS:122' if (mag_input_valid == 1) */
      if ((mag_input_valid == 1.0F) && (fabsf(imu_time - mag_time) < 0.05F)) {
        /*          magn_time =
         * nav_dat.savedSensorBuffer.mag_data_buffer(1).magn_time; */
        /* 'sensors_AHRS:125' if(abs(imu_time - mag_time) < 0.05) */
        /* 'sensors_AHRS:126' local_mag_valid = 1; */
        local_mag_valid = 1;
        /* 'sensors_AHRS:127' mag_data = [mag_x,mag_y,mag_z]; */
        mag_data_idx_0 = mag_x;
        mag_data_idx_1 = mag_y;
        mag_data_idx_2 = mag_z;
      }
      /* 'sensors_AHRS:131' if (local_mag_valid == 1) */
      if (local_mag_valid == 1) {
        float mag_err;
        float q_tmp_idx_0;
        boolean_T b_q;
        /*     		// Magnetometer correction */
        /* 		// Project mag field vector to global frame and extract
         * XY component */
        /*      mag_earth = q.conjugate(mag); */
        /* 'sensors_AHRS:137' v = [mag_data(1), mag_data(2), mag_data(3), 0]; */
        /* 'sensors_AHRS:138' q_tmp = MNF_Quat_mul(q,v); */
        /* c     R and S are two quaternions (the free elment is the forth one)
         */
        /* c     T=R*S is computed. */
        /*  */
        /* 	T = R x S  */
        /* 	function [T] = MULQUAT(R,S) */
        /*  */
        /*   R - might be matrix, nx4, */
        /*   S -  ditto. */
        /*   T -  ditto. */
        /*  */
        /* 'MNF_Quat_mul:15' R = R(:); */
        /* 'MNF_Quat_mul:16' S = S(:); */
        /* 'MNF_Quat_mul:17' [m,n] = size(R) ; */
        /* 'MNF_Quat_mul:19' T = zeros(m,n) ; */
        /* 'MNF_Quat_mul:21' if (m+n)==5 */
        /*  then vectors : */
        /* 'MNF_Quat_mul:22' T(1)=R(4)*S(1)+R(1)*S(4)+R(2)*S(3)-R(3)*S(2); */
        q_tmp_idx_0 =
            ((mag_data_idx_0 * ahrs_Quat_l_2_b[3] + ahrs_Quat_l_2_b[0] * 0.0F) +
             ahrs_Quat_l_2_b[1] * mag_data_idx_2) -
            mag_data_idx_1 * ahrs_Quat_l_2_b[2];
        /* 'MNF_Quat_mul:23' T(2)=R(4)*S(2)+R(2)*S(4)+R(3)*S(1)-R(1)*S(3); */
        q_tmp_idx_1 =
            ((mag_data_idx_1 * ahrs_Quat_l_2_b[3] + ahrs_Quat_l_2_b[1] * 0.0F) +
             mag_data_idx_0 * ahrs_Quat_l_2_b[2]) -
            ahrs_Quat_l_2_b[0] * mag_data_idx_2;
        /* 'MNF_Quat_mul:24' T(3)=R(4)*S(3)+R(3)*S(4)+R(1)*S(2)-R(2)*S(1); */
        q_tmp_idx_2 =
            ((mag_data_idx_2 * ahrs_Quat_l_2_b[3] + ahrs_Quat_l_2_b[2] * 0.0F) +
             ahrs_Quat_l_2_b[0] * mag_data_idx_1) -
            mag_data_idx_0 * ahrs_Quat_l_2_b[1];
        /* 'MNF_Quat_mul:25' T(4)=R(4)*S(4)-R(1)*S(1)-R(2)*S(2)-R(3)*S(3); */
        q_tmp_idx_3 =
            ((ahrs_Quat_l_2_b[3] * 0.0F - ahrs_Quat_l_2_b[0] * mag_data_idx_0) -
             ahrs_Quat_l_2_b[1] * mag_data_idx_1) -
            ahrs_Quat_l_2_b[2] * mag_data_idx_2;
        /* 'sensors_AHRS:139' q_inv = Quat_conjugate_4st(q); */
        /* 'Quat_conjugate_4st:4' Q_A2B=Q_A2B(:); */
        /* 'Quat_conjugate_4st:5' Q_B2A=[-Q_A2B(1:3);Q_A2B(4)]; */
        /* 'sensors_AHRS:140' q_res = MNF_Quat_mul(q_tmp,q_inv); */
        /* c     R and S are two quaternions (the free elment is the forth one)
         */
        /* c     T=R*S is computed. */
        /*  */
        /* 	T = R x S  */
        /* 	function [T] = MULQUAT(R,S) */
        /*  */
        /*   R - might be matrix, nx4, */
        /*   S -  ditto. */
        /*   T -  ditto. */
        /*  */
        /* 'MNF_Quat_mul:15' R = R(:); */
        /* 'MNF_Quat_mul:16' S = S(:); */
        /* 'MNF_Quat_mul:17' [m,n] = size(R) ; */
        /* 'MNF_Quat_mul:19' T = zeros(m,n) ; */
        /* 'MNF_Quat_mul:21' if (m+n)==5 */
        /*  then vectors : */
        /* 'MNF_Quat_mul:22' T(1)=R(4)*S(1)+R(1)*S(4)+R(2)*S(3)-R(3)*S(2); */
        /* 'MNF_Quat_mul:23' T(2)=R(4)*S(2)+R(2)*S(4)+R(3)*S(1)-R(1)*S(3); */
        /* 'MNF_Quat_mul:24' T(3)=R(4)*S(3)+R(3)*S(4)+R(1)*S(2)-R(2)*S(1); */
        /* 'MNF_Quat_mul:25' T(4)=R(4)*S(4)-R(1)*S(1)-R(2)*S(2)-R(3)*S(3); */
        /* 'sensors_AHRS:141' mag_earth = [q_res(1) q_res(2) q_res(3)]; */
        /*              float mag_err = wrap_pi(atan2f(mag_earth(1),
         * mag_earth(0)) - _mag_decl); */
        /* 'sensors_AHRS:144' mag_err = MNF_wrap_to_pi(atan2(mag_earth(2),
         * mag_earth(1)) - mag_map_dec); */
        mag_err = rt_atan2f_snf(((-ahrs_Quat_l_2_b[1] * q_tmp_idx_3 +
                                  q_tmp_idx_1 * ahrs_Quat_l_2_b[3]) +
                                 -ahrs_Quat_l_2_b[0] * q_tmp_idx_2) -
                                    q_tmp_idx_0 * -ahrs_Quat_l_2_b[2],
                                ((-ahrs_Quat_l_2_b[0] * q_tmp_idx_3 +
                                  q_tmp_idx_0 * ahrs_Quat_l_2_b[3]) +
                                 q_tmp_idx_1 * -ahrs_Quat_l_2_b[2]) -
                                    -ahrs_Quat_l_2_b[1] * q_tmp_idx_2) -
                  mag_map_dec;
        /* wrapToPi Wrap angle in radians to [-pi pi] */
        /* 'MNF_wrap_to_pi:5' q = (value < -pi) | (pi < value); */
        b_q = ((mag_err < -3.14159274F) || (mag_err > 3.14159274F));
        /* 'MNF_wrap_to_pi:6' value(q) = MNF_wrap_to_2_pi(value(q) + pi) - pi;
         */
        trueCount = 0;
        if (b_q) {
          trueCount = 1;
        }
        /* wrapTo2Pi Wrap angle in radians to [0 2*pi] */
        /*  */
        /* 'MNF_wrap_to_2_pi:5' positiveInput = (value > 0); */
        /* 'MNF_wrap_to_2_pi:6' value = mod(value, 2*pi); */
        i = fcnOutput->size[0] * fcnOutput->size[1];
        fcnOutput->size[1] = trueCount;
        fcnOutput->size[0] = 1;
        emxEnsureCapacity_real32_T(fcnOutput, i);
        fcnOutput_data = fcnOutput->data;
        if (trueCount - 1 >= 0) {
          if (rtIsNaNF(mag_err + 3.14159274F) ||
              rtIsInfF(mag_err + 3.14159274F)) {
            xbar = rtNaNF;
          } else if (mag_err + 3.14159274F == 0.0F) {
            xbar = 0.0F;
          } else {
            boolean_T rEQ0;
            xbar = fmodf(mag_err + 3.14159274F, 6.28318548F);
            rEQ0 = (xbar == 0.0F);
            if (!rEQ0) {
              q = fabsf((mag_err + 3.14159274F) / 6.28318548F);
              rEQ0 = !(fabsf(q - floorf(q + 0.5F)) > 1.1920929E-7F * q);
            }
            if (rEQ0) {
              xbar = 0.0F;
            } else if (mag_err + 3.14159274F < 0.0F) {
              xbar += 6.28318548F;
            }
          }
          fcnOutput_data[0] = xbar;
        }
        /* 'MNF_wrap_to_2_pi:7' value((value == 0) & positiveInput) = 2*pi; */
        if (fcnOutput->size[1] == trueCount) {
          i = r->size[0] * r->size[1];
          r->size[1] = fcnOutput->size[1];
          r->size[0] = 1;
          emxEnsureCapacity_boolean_T(r, i);
          r1 = r->data;
          trueCount = fcnOutput->size[1];
          for (i = 0; i < trueCount; i++) {
            r1[0] =
                ((fcnOutput_data[0] == 0.0F) && (mag_err + 3.14159274F > 0.0F));
          }
        } else {
          binary_expand_op(r, fcnOutput, mag_err, trueCount);
          r1 = r->data;
        }
        if ((r->size[1] - 1 >= 0) && r1[0]) {
          fcnOutput_data[0] = 6.28318548F;
        }
        i = fcnOutput->size[0] * fcnOutput->size[1];
        fcnOutput->size[0] = 1;
        emxEnsureCapacity_real32_T(fcnOutput, i);
        fcnOutput_data = fcnOutput->data;
        trueCount = fcnOutput->size[1] - 1;
        for (i = 0; i <= trueCount; i++) {
          fcnOutput_data[i] -= 3.14159274F;
        }
        absxk_tmp = mag_err;
        if (b_q) {
          absxk_tmp = fcnOutput_data[0];
        }
        /* 'sensors_AHRS:146' gainMult = 1; */
        b_absxk_tmp = 1.0F;
        /* 'sensors_AHRS:148' mag_smal_gain = 0; */
        /* 'sensors_AHRS:149' if (spinRate > fifty_dps) */
        if (spinRate > 0.873F) {
          /* 'sensors_AHRS:150' gainMult = min(spinRate / fifty_dps, 10.0); */
          b_absxk_tmp = fminf(spinRate / 0.873F, 10.0F);
          /* 'sensors_AHRS:151' mag_smal_gain = 1; */
        }
        /*  // Project magnetometer correction to body frame */
        /*  corr += _q.conjugate_inversed(Vector3f(0.0f, 0.0f, -mag_err)) *
         * _w_mag * gainMult; */
        /* 'sensors_AHRS:157' v = [0, 0, -mag_err]; */
        /* 'sensors_AHRS:158' q_v = [v, 0]; */
        /* 'sensors_AHRS:159' q_tmp = MNF_Quat_mul(Quat_conjugate_4st(q),q_v);
         */
        /* c     R and S are two quaternions (the free elment is the forth one)
         */
        /* c     T=R*S is computed. */
        /*  */
        /* 	T = R x S  */
        /* 	function [T] = MULQUAT(R,S) */
        /*  */
        /*   R - might be matrix, nx4, */
        /*   S -  ditto. */
        /*   T -  ditto. */
        /*  */
        /* 'MNF_Quat_mul:15' R = R(:); */
        /* 'MNF_Quat_mul:16' S = S(:); */
        /* 'MNF_Quat_mul:17' [m,n] = size(R) ; */
        /* 'MNF_Quat_mul:19' T = zeros(m,n) ; */
        /* 'MNF_Quat_mul:21' if (m+n)==5 */
        /*  then vectors : */
        /* 'MNF_Quat_mul:22' T(1)=R(4)*S(1)+R(1)*S(4)+R(2)*S(3)-R(3)*S(2); */
        xbar = -ahrs_Quat_l_2_b[2] * 0.0F;
        q = -ahrs_Quat_l_2_b[0] * 0.0F;
        q_tmp_idx_0 = ((ahrs_Quat_l_2_b[3] * 0.0F + q) +
                       -ahrs_Quat_l_2_b[1] * -absxk_tmp) -
                      xbar;
        /* 'MNF_Quat_mul:23' T(2)=R(4)*S(2)+R(2)*S(4)+R(3)*S(1)-R(1)*S(3); */
        scale = -ahrs_Quat_l_2_b[1] * 0.0F;
        q_tmp_idx_1 = ((ahrs_Quat_l_2_b[3] * 0.0F + scale) + xbar) -
                      -ahrs_Quat_l_2_b[0] * -absxk_tmp;
        /* 'MNF_Quat_mul:24' T(3)=R(4)*S(3)+R(3)*S(4)+R(1)*S(2)-R(2)*S(1); */
        q_tmp_idx_2 = ((-absxk_tmp * ahrs_Quat_l_2_b[3] + xbar) + q) - scale;
        /* 'MNF_Quat_mul:25' T(4)=R(4)*S(4)-R(1)*S(1)-R(2)*S(2)-R(3)*S(3); */
        q_tmp_idx_3 = ((ahrs_Quat_l_2_b[3] * 0.0F - q) - scale) -
                      -ahrs_Quat_l_2_b[2] * -absxk_tmp;
        /* 'sensors_AHRS:160' q_res = MNF_Quat_mul(q_tmp,q); */
        /* c     R and S are two quaternions (the free elment is the forth one)
         */
        /* c     T=R*S is computed. */
        /*  */
        /* 	T = R x S  */
        /* 	function [T] = MULQUAT(R,S) */
        /*  */
        /*   R - might be matrix, nx4, */
        /*   S -  ditto. */
        /*   T -  ditto. */
        /*  */
        /* 'MNF_Quat_mul:15' R = R(:); */
        /* 'MNF_Quat_mul:16' S = S(:); */
        /* 'MNF_Quat_mul:17' [m,n] = size(R) ; */
        /* 'MNF_Quat_mul:19' T = zeros(m,n) ; */
        /* 'MNF_Quat_mul:21' if (m+n)==5 */
        /*  then vectors : */
        /* 'MNF_Quat_mul:22' T(1)=R(4)*S(1)+R(1)*S(4)+R(2)*S(3)-R(3)*S(2); */
        /* 'MNF_Quat_mul:23' T(2)=R(4)*S(2)+R(2)*S(4)+R(3)*S(1)-R(1)*S(3); */
        /* 'MNF_Quat_mul:24' T(3)=R(4)*S(3)+R(3)*S(4)+R(1)*S(2)-R(2)*S(1); */
        /* 'MNF_Quat_mul:25' T(4)=R(4)*S(4)-R(1)*S(1)-R(2)*S(2)-R(3)*S(3); */
        /* 'sensors_AHRS:161' q_res_vec = [q_res(1) q_res(2) q_res(3)]; */
        /* 'sensors_AHRS:162' correction = correction + q_res_vec * w_mag
         * *gainMult; */
        correction_idx_0 = (((ahrs_Quat_l_2_b[0] * q_tmp_idx_3 +
                              q_tmp_idx_0 * ahrs_Quat_l_2_b[3]) +
                             q_tmp_idx_1 * ahrs_Quat_l_2_b[2]) -
                            ahrs_Quat_l_2_b[1] * q_tmp_idx_2) *
                           0.5F * b_absxk_tmp;
        correction_idx_1 = (((ahrs_Quat_l_2_b[1] * q_tmp_idx_3 +
                              q_tmp_idx_1 * ahrs_Quat_l_2_b[3]) +
                             ahrs_Quat_l_2_b[0] * q_tmp_idx_2) -
                            q_tmp_idx_0 * ahrs_Quat_l_2_b[2]) *
                           0.5F * b_absxk_tmp;
        correction_idx_2 = (((ahrs_Quat_l_2_b[2] * q_tmp_idx_3 +
                              q_tmp_idx_2 * ahrs_Quat_l_2_b[3]) +
                             q_tmp_idx_0 * ahrs_Quat_l_2_b[1]) -
                            ahrs_Quat_l_2_b[0] * q_tmp_idx_1) *
                           0.5F * b_absxk_tmp;
      }
      /*  Accelerometer correction */
      /* 	// Project 'k' unit vector of earth frame to body frame */
      /* 	// Vector3f k = q.conjugate_inversed(Vector3f(0.0f,
       * 0.0f, 1.0f)); */
      /* 	// Optimized version with dropped zeros */
      /* 'sensors_AHRS:169' q_first_real = [q(4), q(1), q(2), q(3)]; */
      /* 'sensors_AHRS:170' k = [	2.0 * (q_first_real(2) *
       * q_first_real(4) - q_first_real(1) * q_first_real(3)), ... */
      /* 'sensors_AHRS:171'     2.0 * (q_first_real(3) * q_first_real(4) +
       * q_first_real(1) * q_first_real(2)),... */
      /* 'sensors_AHRS:172'     (q_first_real(1) * q_first_real(1) -
       * q_first_real(2) * q_first_real(2) - q_first_real(3) * q_first_real(3) +
       * q_first_real(4) * q_first_real(4))	]; */
      k_idx_0 = 2.0F * (ahrs_Quat_l_2_b[0] * ahrs_Quat_l_2_b[2] -
                        ahrs_Quat_l_2_b[1] * ahrs_Quat_l_2_b[3]);
      k_idx_1 = 2.0F * (ahrs_Quat_l_2_b[1] * ahrs_Quat_l_2_b[2] +
                        ahrs_Quat_l_2_b[0] * ahrs_Quat_l_2_b[3]);
      k_idx_2 = ((ahrs_Quat_l_2_b[3] * ahrs_Quat_l_2_b[3] -
                  ahrs_Quat_l_2_b[0] * ahrs_Quat_l_2_b[0]) -
                 ahrs_Quat_l_2_b[1] * ahrs_Quat_l_2_b[1]) +
                ahrs_Quat_l_2_b[2] * ahrs_Quat_l_2_b[2];
      /*  check if imu is static enough to apply accelerometer correction and */
      /*  gyro bias correction */
      /* 'sensors_AHRS:176' if (imu_static == 1) */
      if (*imu_static == 1.0F) {
        /*  corr += (k % (_accel - _pos_acc).normalized()) * _w_accel; */
        /* 'sensors_AHRS:178' correction = correction + cross(k,((accel_G
         * )/norm(accel_G )))*w_accel; */
        t = imu_acc_x_G / a_tmp;
        scale = imu_acc_y_G / a_tmp;
        attitude_ptf_idx_2 = imu_acc_z_G / a_tmp;
        q = (t * k_idx_2 - k_idx_0 * attitude_ptf_idx_2) * 0.3F;
        xbar = (k_idx_0 * scale - t * k_idx_1) * 0.3F;
        t = correction_idx_0 +
            (k_idx_1 * attitude_ptf_idx_2 - scale * k_idx_2) * 0.3F;
        scale = correction_idx_1 + q;
        attitude_ptf_idx_2 = correction_idx_2 + xbar;
        /* 'sensors_AHRS:179' ahrs_gyro_bias = ahrs_gyro_bias +  correction *
         * (w_gyro_bias * dt); */
        b_absxk_tmp = 0.01F * dt;
        correction_idx_0 = t;
        ahrs_gyro_bias[0] += t * b_absxk_tmp;
        correction_idx_1 = scale;
        ahrs_gyro_bias[1] += scale * b_absxk_tmp;
        correction_idx_2 = attitude_ptf_idx_2;
        ahrs_gyro_bias[2] += attitude_ptf_idx_2 * b_absxk_tmp;
      }
      /*  Gyro bias estimation */
      /*  	if (spinRate < 0.175f) { */
      /*      if (norm(gyro) < 0.175) */
      /*  		gyro_bias = gyro_bias +  correction * (w_gyro_bias *
       * dt); */
      /*      end */
      /*  		for (int i = 0; i < 3; i++) { */
      /*  			_gyro_bias(i) = math::constrain(_gyro_bias(i),
       * -_bias_max, _bias_max); */
      /*  		} */
      /*  	} */
      /* 'sensors_AHRS:194' rates = gyro + ahrs_gyro_bias; */
      /*       rates = [0 0 0 ]; %% dbg */
      /*  	// Feed forward gyro */
      /* 'sensors_AHRS:198' correction = correction + rates; */
      correction_idx_0 += gyro_idx_0 + ahrs_gyro_bias[0];
      correction_idx_1 += gyro_idx_1 + ahrs_gyro_bias[1];
      correction_idx_2 += gyro_idx_2 + ahrs_gyro_bias[2];
      /*      correction = rates; */
      /*  	// Apply correction to state */
      /* 'sensors_AHRS:202' v = [ correction(1), correction(2), correction(3) ,
       * 0]; */
      /*  quat 4th real! */
      /* _q += _q.derivative1(corr) * dt; */
      /* 'sensors_AHRS:205' q_tmp = MNF_Quat_mul(q,v); */
      /* c     R and S are two quaternions (the free elment is the forth one) */
      /* c     T=R*S is computed. */
      /*  */
      /* 	T = R x S  */
      /* 	function [T] = MULQUAT(R,S) */
      /*  */
      /*   R - might be matrix, nx4, */
      /*   S -  ditto. */
      /*   T -  ditto. */
      /*  */
      /* 'MNF_Quat_mul:15' R = R(:); */
      /* 'MNF_Quat_mul:16' S = S(:); */
      /* 'MNF_Quat_mul:17' [m,n] = size(R) ; */
      /* 'MNF_Quat_mul:19' T = zeros(m,n) ; */
      /* 'MNF_Quat_mul:21' if (m+n)==5 */
      /*  then vectors : */
      /* 'MNF_Quat_mul:22' T(1)=R(4)*S(1)+R(1)*S(4)+R(2)*S(3)-R(3)*S(2); */
      /* 'MNF_Quat_mul:23' T(2)=R(4)*S(2)+R(2)*S(4)+R(3)*S(1)-R(1)*S(3); */
      q_tmp_idx_1 =
          ((correction_idx_1 * ahrs_Quat_l_2_b[3] + ahrs_Quat_l_2_b[1] * 0.0F) +
           correction_idx_0 * ahrs_Quat_l_2_b[2]) -
          ahrs_Quat_l_2_b[0] * correction_idx_2;
      /* 'MNF_Quat_mul:24' T(3)=R(4)*S(3)+R(3)*S(4)+R(1)*S(2)-R(2)*S(1); */
      q_tmp_idx_2 =
          ((correction_idx_2 * ahrs_Quat_l_2_b[3] + ahrs_Quat_l_2_b[2] * 0.0F) +
           ahrs_Quat_l_2_b[0] * correction_idx_1) -
          correction_idx_0 * ahrs_Quat_l_2_b[1];
      /* 'MNF_Quat_mul:25' T(4)=R(4)*S(4)-R(1)*S(1)-R(2)*S(2)-R(3)*S(3); */
      q_tmp_idx_3 =
          ((ahrs_Quat_l_2_b[3] * 0.0F - ahrs_Quat_l_2_b[0] * correction_idx_0) -
           ahrs_Quat_l_2_b[1] * correction_idx_1) -
          ahrs_Quat_l_2_b[2] * correction_idx_2;
      /* 'sensors_AHRS:206' q_tmp =  q_tmp * 0.5 * dt; */
      /* 'sensors_AHRS:207' q = q + q_tmp; */
      /*  Normalize quaternion */
      /* 'sensors_AHRS:209' q = Quat_normalize_4st(q); */
      /* 'Quat_normalize_4st:4' Q_norm=sqrt(sum(Q.^2)); */
      ahrs_Quat_l_2_b[0] += (((correction_idx_0 * ahrs_Quat_l_2_b[3] +
                               ahrs_Quat_l_2_b[0] * 0.0F) +
                              ahrs_Quat_l_2_b[1] * correction_idx_2) -
                             correction_idx_1 * ahrs_Quat_l_2_b[2]) *
                            0.5F * dt;
      ahrs_Quat_l_2_b[1] += q_tmp_idx_1 * 0.5F * dt;
      ahrs_Quat_l_2_b[2] += q_tmp_idx_2 * 0.5F * dt;
      ahrs_Quat_l_2_b[3] += q_tmp_idx_3 * 0.5F * dt;
      q = sqrtf(((rt_powf_snf(ahrs_Quat_l_2_b[0], 2.0F) +
                  rt_powf_snf(ahrs_Quat_l_2_b[1], 2.0F)) +
                 rt_powf_snf(ahrs_Quat_l_2_b[2], 2.0F)) +
                rt_powf_snf(ahrs_Quat_l_2_b[3], 2.0F));
      /* 'Quat_normalize_4st:5' Q_norm=repmat(Q_norm,4,1); */
      /* 'Quat_normalize_4st:6' normalized_Q=Q./Q_norm; */
      ahrs_Quat_l_2_b[0] /= q;
      ahrs_Quat_l_2_b[1] /= q;
      ahrs_Quat_l_2_b[2] /= q;
      ahrs_Quat_l_2_b[3] /= q;
      /* 'Quat_normalize_4st:8' if normalized_Q(4)<0 */
      if (ahrs_Quat_l_2_b[3] < 0.0F) {
        /* 'Quat_normalize_4st:9' normalized_Q=-normalized_Q; */
        ahrs_Quat_l_2_b[0] = -ahrs_Quat_l_2_b[0];
        ahrs_Quat_l_2_b[1] = -ahrs_Quat_l_2_b[1];
        ahrs_Quat_l_2_b[2] = -ahrs_Quat_l_2_b[2];
        ahrs_Quat_l_2_b[3] = -ahrs_Quat_l_2_b[3];
      }
      /*  set ahrs output */
      /* 'sensors_AHRS:211' ahrs_Quat_l_2_b = q'; */
      /* 'sensors_AHRS:212' quat_l2b1 = q(1); */
      *quat_l2b1 = ahrs_Quat_l_2_b[0];
      /* 'sensors_AHRS:213' quat_l2b2 = q(2); */
      *quat_l2b2 = ahrs_Quat_l_2_b[1];
      /* 'sensors_AHRS:214' quat_l2b3 = q(3); */
      *quat_l2b3 = ahrs_Quat_l_2_b[2];
      /* 'sensors_AHRS:215' quat_l2b4 = q(4); */
      *quat_l2b4 = ahrs_Quat_l_2_b[3];
      /* 'sensors_AHRS:216' [euler_l2b] = MNF_Quat2Euler(q); */
      /*  Q_L2B - real element is 4th  */
      /*  euler_angles=[psi;theta;phi] */
      /* 'MNF_Quat2Euler:8' qR=Q_L2B(4,:); */
      /* 'MNF_Quat2Euler:9' qx=Q_L2B(1,:); */
      /* 'MNF_Quat2Euler:10' qy=Q_L2B(2,:); */
      /* 'MNF_Quat2Euler:11' qz=Q_L2B(3,:); */
      /* 'MNF_Quat2Euler:13' phi=atan2(2*(qR.*qx+qy.*qz),1-2*(qx.^2+qy.^2)); */
      /* 'MNF_Quat2Euler:14' theta=asin(2*(qR.*qy-qz.*qx)); */
      /* 'MNF_Quat2Euler:15' psi=atan2(2*(qR.*qz+qx.*qy),1-2*(qy.^2+qz.^2)); */
      /* 'MNF_Quat2Euler:17' euler_angles=[psi;theta;phi]; */
      /* 'sensors_AHRS:217' Euler_l2b_psi  = euler_l2b(1); */
      xbar = ahrs_Quat_l_2_b[1] * ahrs_Quat_l_2_b[1];
      *Euler_l2b_psi = rt_atan2f_snf(
          2.0F * (ahrs_Quat_l_2_b[2] * ahrs_Quat_l_2_b[3] +
                  ahrs_Quat_l_2_b[0] * ahrs_Quat_l_2_b[1]),
          1.0F - 2.0F * (xbar + ahrs_Quat_l_2_b[2] * ahrs_Quat_l_2_b[2]));
      /* 'sensors_AHRS:218' Euler_l2b_theta = euler_l2b(2); */
      *Euler_l2b_theta =
          asinf(2.0F * (ahrs_Quat_l_2_b[1] * ahrs_Quat_l_2_b[3] -
                        ahrs_Quat_l_2_b[0] * ahrs_Quat_l_2_b[2]));
      /* 'sensors_AHRS:219' Euler_l2b_phi = euler_l2b(3); */
      *Euler_l2b_phi = rt_atan2f_snf(
          2.0F * (ahrs_Quat_l_2_b[0] * ahrs_Quat_l_2_b[3] +
                  ahrs_Quat_l_2_b[1] * ahrs_Quat_l_2_b[2]),
          1.0F - 2.0F * (ahrs_Quat_l_2_b[0] * ahrs_Quat_l_2_b[0] + xbar));
      /*  raise ahrs counter */
      /* 'sensors_AHRS:221' ahrs_global_counter = ahrs_global_counter + 1; */
      ahrs_global_counter++;
      /* 'sensors_AHRS:222' ahrs_output_valid = 1; */
      *ahrs_output_valid = 1.0F;
      /* 'sensors_AHRS:223' ahrs_counter = ahrs_global_counter; */
      *ahrs_counter = ahrs_global_counter;
    } else {
      /* 'sensors_AHRS:102' ahrs_output_valid = 0; */
    }
  }
  emxFree_real32_T(&fcnOutput);
  emxFree_boolean_T(&r);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void sensors_AHRS_initialize(void)
{
  imu_data_buffer_not_empty = false;
  sensors_AHRS_init();
  sensor_activity_classifier_init();
  isInitialized_sensors_AHRS = true;
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void sensors_AHRS_terminate(void)
{
  isInitialized_sensors_AHRS = false;
}

/*
 * File trailer for sensors_AHRS.c
 *
 * [EOF]
 */
