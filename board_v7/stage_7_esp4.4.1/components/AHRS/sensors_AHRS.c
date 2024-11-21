/*
 * File: sensors_AHRS.c
 *
 * MATLAB Coder version            : 5.1
 * C/C++ source code generated on  : 31-Mar-2022 07:55:09
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
static float static_counter;
static float imu_data_buffer[700];
static boolean_T imu_data_buffer_not_empty;
static float imu_data_counter;
static float last_activity_time;
static float static_start_time;
static boolean_T isInitialized_sensors_AHRS = false;

/* Function Declarations */
static float rt_atan2f_snf(float u0, float u1);
static float rt_powf_snf(float u0, float u1);
static void sensor_activity_classifier_init(void);
static void sensors_AHRS_init(void);

/* Function Definitions */
/*
 * Arguments    : float u0
 *                float u1
 * Return Type  : float
 */
static float rt_atan2f_snf(float u0, float u1)
{
  float y;
  int b_u0;
  int b_u1;
  if (rtIsNaNF(u0) || rtIsNaNF(u1)) {
    y = rtNaNF;
  } else if (rtIsInfF(u0) && rtIsInfF(u1)) {
    if (u0 > 0.0F) {
      b_u0 = 1;
    } else {
      b_u0 = -1;
    }

    if (u1 > 0.0F) {
      b_u1 = 1;
    } else {
      b_u1 = -1;
    }

    y = atan2f((float)b_u0, (float)b_u1);
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
  float f;
  float f1;
  float y;
  if (rtIsNaNF(u0) || rtIsNaNF(u1)) {
    y = rtNaNF;
  } else {
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
 * function [activity,activity_class_acc_rss_var] = sensor_activity_classifier(imu_time,imu_acc_x_G,imu_acc_y_G,imu_acc_z_G,...
 *     imu_gyro_x_dps,imu_gyro_y_dps,imu_gyro_z_dps, static_time_for_sleep)
 * SENSOR_ACTIVITY_CLASSIFIER Summary of this function goes here
 *    Detailed explanation goes here
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
 *     Euler_l2b_psi , Euler_l2b_theta , Euler_l2b_phi, ahrs_counter, valid, imu_static,activity,activity_class_acc_rss_var] = ...
 *     sensors_AHRS(imu_time,imu_acc_x_G,imu_acc_y_G,imu_acc_z_G,...
 *     imu_gyro_x_dps,imu_gyro_y_dps,imu_gyro_z_dps,imu_rate,...
 *     mag_time,mag_x,mag_y,mag_z,mag_map_dec,mag_input_valid,static_time_for_sleep)
 * sensors_AHRS gets accelerometer XYZ, gyro XYZ, imu rate
 *  and propagates AHRS quaternion & euler angles
 *  output: quat_l2b1,quat_l2b2,quat_l2b3,quat_l2b4 : quaternion 4th real,
 *  local to body
 *                Euler_l2b_psi , Euler_l2b_theta , Euler_l2b_phi : euler
 *                angles 321 , local to body
 *                valid : validity of AHRS solution
 * Arguments    : void
 * Return Type  : void
 */
static void sensors_AHRS_init(void)
{
  /* 'sensors_AHRS:18' ahrs_Quat_l_2_b = [0 0 0 1]; */
  ahrs_Quat_l_2_b[0] = 0.0F;
  ahrs_Quat_l_2_b[1] = 0.0F;
  ahrs_Quat_l_2_b[2] = 0.0F;
  ahrs_Quat_l_2_b[3] = 1.0F;

  /* 'sensors_AHRS:19' ahrs_gyro_bias = [0 0 0]; */
  ahrs_gyro_bias[0] = 0.0F;
  ahrs_gyro_bias[1] = 0.0F;
  ahrs_gyro_bias[2] = 0.0F;

  /* 'sensors_AHRS:20' static_counter = 1; */
  static_counter = 1.0F;
}

/*
 * function [quat_l2b1,quat_l2b2,quat_l2b3,quat_l2b4, ...
 *     Euler_l2b_psi , Euler_l2b_theta , Euler_l2b_phi, ahrs_counter, valid, imu_static,activity,activity_class_acc_rss_var] = ...
 *     sensors_AHRS(imu_time,imu_acc_x_G,imu_acc_y_G,imu_acc_z_G,...
 *     imu_gyro_x_dps,imu_gyro_y_dps,imu_gyro_z_dps,imu_rate,...
 *     mag_time,mag_x,mag_y,mag_z,mag_map_dec,mag_input_valid,static_time_for_sleep)
 * sensors_AHRS gets accelerometer XYZ, gyro XYZ, imu rate
 *  and propagates AHRS quaternion & euler angles
 *  output: quat_l2b1,quat_l2b2,quat_l2b3,quat_l2b4 : quaternion 4th real,
 *  local to body
 *                Euler_l2b_psi , Euler_l2b_theta , Euler_l2b_phi : euler
 *                angles 321 , local to body
 *                valid : validity of AHRS solution
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
 *                float *quat_l2b1
 *                float *quat_l2b2
 *                float *quat_l2b3
 *                float *quat_l2b4
 *                float *Euler_l2b_psi
 *                float *Euler_l2b_theta
 *                float *Euler_l2b_phi
 *                float *ahrs_counter
 *                float *valid
 *                float *imu_static
 *                float *activity
 *                float *activity_class_acc_rss_var
 * Return Type  : void
 */
void sensors_AHRS(float imu_time, float imu_acc_x_G, float imu_acc_y_G, float
                  imu_acc_z_G, float imu_gyro_x_dps, float imu_gyro_y_dps, float
                  imu_gyro_z_dps, float imu_rate, float mag_time, float mag_x,
                  float mag_y, float mag_z, float mag_map_dec, float
                  mag_input_valid, float static_time_for_sleep, float *quat_l2b1,
                  float *quat_l2b2, float *quat_l2b3, float *quat_l2b4, float
                  *Euler_l2b_psi, float *Euler_l2b_theta, float *Euler_l2b_phi,
                  float *ahrs_counter, float *valid, float *imu_static, float
                  *activity, float *activity_class_acc_rss_var)
{
  emxArray_boolean_T *positiveInput;
  emxArray_boolean_T *r;
  emxArray_real32_T *c;
  emxArray_real32_T *x;
  float y[100];
  float a;
  float absxk;
  float acc_rss_var;
  float accel_norm_sq;
  float b_b;
  float c_idx_1;
  float correction_idx_0;
  float correction_idx_1;
  float correction_idx_2;
  float dt;
  float gyro_idx_0;
  float gyro_idx_1;
  float gyro_idx_2;
  float k_idx_0;
  float k_idx_1;
  float k_idx_2;
  float mag_data_idx_0;
  float mag_data_idx_1;
  float mag_data_idx_2;
  float mag_err;
  float q_idx_0;
  float q_idx_1;
  float q_idx_2;
  float q_idx_3;
  float q_tmp_idx_0;
  float q_tmp_idx_1;
  float q_tmp_idx_1_tmp;
  float q_tmp_idx_2;
  float q_tmp_idx_3;
  float spinRate;
  float t;
  float xbar;
  int N;
  int b_i;
  int i;
  int i1;
  int i2;
  int i3;
  int k;
  int local_mag_valid;
  int pageroot;
  unsigned int unnamed_idx_1;
  boolean_T b;
  boolean_T b1;
  boolean_T b2;
  boolean_T b3;
  boolean_T b4;
  boolean_T b5;
  boolean_T rEQ0;
  if (!isInitialized_sensors_AHRS) {
    sensors_AHRS_initialize();
  }

  /*  init persistent vars */
  /* 'sensors_AHRS:17' if (isempty(static_counter)) */
  /* 'sensors_AHRS:25' [activity,activity_class_acc_rss_var] = sensor_activity_classifier(imu_time,imu_acc_x_G,imu_acc_y_G,imu_acc_z_G,... */
  /* 'sensors_AHRS:26'     imu_gyro_x_dps,imu_gyro_y_dps,imu_gyro_z_dps,static_time_for_sleep); */
  /* SENSOR_ACTIVITY_CLASSIFIER Summary of this function goes here */
  /*    Detailed explanation goes here */
  /* 'sensor_activity_classifier:6' SAVED_DATA_SIZE = 100; */
  /*  data of 1 seconds at 100 hz */
  /* 'sensor_activity_classifier:7' MINIMUM_DELTA_TIME_FOR_IMU_SAVE = 0.099; */
  /* 'sensor_activity_classifier:8' MINIMUM_DELTA_TIME_FOR_ACTIVITY_CLASS = 1.0; */
  /* 'sensor_activity_classifier:9' ACC_VAR_THRESH = 0.01; */
  /*  initialize: */
  /* 'sensor_activity_classifier:13' if (isempty(imu_data_buffer)) */
  if (!imu_data_buffer_not_empty) {
    /* 'sensor_activity_classifier:14' imu_data_counter = 0; */
    /* 'sensor_activity_classifier:15' imu_data_buffer = zeros(SAVED_DATA_SIZE,7); */
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

  /*  check if we should go to sleep (this could be overriden later by classifier) */
  /* 'sensor_activity_classifier:27' prev_time = imu_data_buffer(1,1); */
  /* 'sensor_activity_classifier:28' if (imu_time - prev_time > MINIMUM_DELTA_TIME_FOR_IMU_SAVE) */
  if (imu_time - imu_data_buffer[0] > 0.099F) {
    /* 'sensor_activity_classifier:29' imu_data_buffer(1,:) = [imu_time,imu_acc_x_G,imu_acc_y_G,imu_acc_z_G,... */
    /* 'sensor_activity_classifier:30'         imu_gyro_x_dps,imu_gyro_y_dps,imu_gyro_z_dps]; */
    imu_data_buffer[0] = imu_time;
    imu_data_buffer[1] = imu_acc_x_G;
    imu_data_buffer[2] = imu_acc_y_G;
    imu_data_buffer[3] = imu_acc_z_G;
    imu_data_buffer[4] = imu_gyro_x_dps;
    imu_data_buffer[5] = imu_gyro_y_dps;
    imu_data_buffer[6] = imu_gyro_z_dps;

    /* 'sensor_activity_classifier:31' imu_data_buffer = circshift(imu_data_buffer,1); */
    b = true;
    i = 0;
    b1 = true;
    N = 0;
    for (b_i = 0; b_i < 7; b_i++) {
      pageroot = b_i * 100;
      if (b1) {
        b1 = false;
        N = pageroot % 100 * 7 + pageroot / 100;
      } else {
        N += 700;
        N %= 699;
        if (N == 0) {
          N = 699;
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
        i3 = pageroot + k;
        if (b3) {
          b3 = false;
          i2 = i3 % 100 * 7 + i3 / 100;
        } else {
          i2 += -7;
          if (i2 < 0) {
            i2 += 699;
          }
        }

        if (b2) {
          b2 = false;
          i1 = (i3 + 1) % 100 * 7 + (i3 + 1) / 100;
        } else {
          i1 += -7;
          if (i1 < 0) {
            i1 += 699;
          }
        }

        imu_data_buffer[i1] = imu_data_buffer[i2];
      }

      imu_data_buffer[N] = xbar;
    }

    /* 'sensor_activity_classifier:32' imu_data_counter = imu_data_counter + 1; */
    imu_data_counter++;
  }

  /*  else - quit function */
  /*  determine if we should activate classifier */
  /* 'sensor_activity_classifier:38' if (imu_data_counter > SAVED_DATA_SIZE ) */
  if (imu_data_counter > 100.0F) {
    /* 'sensor_activity_classifier:39' if (imu_time - last_activity_time > MINIMUM_DELTA_TIME_FOR_ACTIVITY_CLASS) */
    if (imu_time - last_activity_time > 1.0F) {
      /* 'sensor_activity_classifier:40' acc_rss = rssq(imu_data_buffer(:,2:4),2); */
      /* RSSQ   Root sum squared value. */
      /*    For vectors, RSSQ(X) is the root sum squared value in X. For matrices, */
      /*    RSSQ(X) is a row vector containing the RSSQ value from each column. For */
      /*    N-D arrays, RSSQ(X) operates along the first non-singleton dimension. */
      /*  */
      /*    Y = RSSQ(X,DIM) operates along the dimension DIM. */
      /*  */
      /*    When X is complex, the RSSQ is computed using the magnitude */
      /*    RSSQ(ABS(X)).  */
      /*  */
      /*    % Example 1: rssq of a vector */
      /*    x = 1./(1:1000); */
      /*    y = rssq(x)  */
      /*  */
      /*    % Example 2: rssq of the columns of a matrix */
      /*    x = [4 5 7; 3 12 24]; */
      /*    y = rssq(x, 1) */
      /*  */
      /*    % Example 3: rssq of the rows of a matrix */
      /*    x = [4 5 7; 3 12 24]; */
      /*    y = rssq(x, 2) */
      /*  */
      /*    See also MIN, MAX, MEDIAN, MEAN, STD, RMS. */
      /*    Copyright 2011 The MathWorks, Inc. */
      /* 'rssq:28' if nargin==1 */
      /* 'rssq:30' else */
      /* 'rssq:31' y = sqrt(sum(x .* conj(x), dim)); */
      for (k = 0; k < 100; k++) {
        q_tmp_idx_1_tmp = imu_data_buffer[7 * k + 1];
        xbar = q_tmp_idx_1_tmp * q_tmp_idx_1_tmp;
        q_tmp_idx_1_tmp = imu_data_buffer[7 * k + 2];
        absxk = q_tmp_idx_1_tmp * q_tmp_idx_1_tmp;
        q_tmp_idx_1_tmp = imu_data_buffer[7 * k + 3];
        y[k] = xbar;
        y[k] += absxk;
        y[k] += q_tmp_idx_1_tmp * q_tmp_idx_1_tmp;
      }

      for (k = 0; k < 100; k++) {
        y[k] = sqrtf(y[k]);
      }

      /* 'sensor_activity_classifier:41' acc_rss_var = var(acc_rss); */
      xbar = y[0];
      for (k = 0; k < 99; k++) {
        xbar += y[k + 1];
      }

      xbar /= 100.0F;
      absxk = 0.0F;
      for (k = 0; k < 100; k++) {
        t = y[k] - xbar;
        absxk += t * t;
      }

      acc_rss_var = absxk / 99.0F;

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

      /* 'sensor_activity_classifier:49' activity_class_acc_rss_var = acc_rss_var; */
      *activity_class_acc_rss_var = acc_rss_var;
    }

    /* 'sensor_activity_classifier:51' if (imu_time - static_start_time > static_time_for_sleep) */
    if (imu_time - static_start_time > static_time_for_sleep) {
      /* 'sensor_activity_classifier:52' activity = 4; */
      *activity = 4.0F;

      /*  go to sleep */
    }
  }

  /*  init outputs */
  /* 'sensors_AHRS:29' valid = 0; */
  *valid = 0.0F;

  /* 'sensors_AHRS:30' quat_l2b1 = 0; */
  *quat_l2b1 = 0.0F;

  /* 'sensors_AHRS:30' quat_l2b2 = 0; */
  *quat_l2b2 = 0.0F;

  /* 'sensors_AHRS:30' quat_l2b3 = 0; */
  *quat_l2b3 = 0.0F;

  /* 'sensors_AHRS:30' quat_l2b4 = 1; */
  *quat_l2b4 = 1.0F;

  /* 'sensors_AHRS:31' Euler_l2b_psi = 0; */
  *Euler_l2b_psi = 0.0F;

  /* 'sensors_AHRS:31' Euler_l2b_theta = 0; */
  *Euler_l2b_theta = 0.0F;

  /* 'sensors_AHRS:31' Euler_l2b_phi = 0; */
  *Euler_l2b_phi = 0.0F;

  /* 'sensors_AHRS:32' ahrs_counter = 0; */
  *ahrs_counter = 0.0F;

  /* 'sensors_AHRS:33' imu_static = 0; */
  *imu_static = 0.0F;

  /*  set constant dt */
  /* 'sensors_AHRS:35' dt = 1/imu_rate; */
  dt = 1.0F / imu_rate;

  /*  tuning */
  /* 'sensors_AHRS:38' w_accel = 0.3; */
  /*  tune */
  /* 'sensors_AHRS:39' w_gyro_bias = 0.01; */
  /*  tune */
  /* 'sensors_AHRS:40' w_mag = 0.5; */
  /*  tune */
  /* 'sensors_AHRS:41' SPIN_RATE_THRESH = 1; */
  /* 0.14; % tune */
  /*  constants */
  /* 'sensors_AHRS:44' fifty_dps = 0.873; */
  /* 'sensors_AHRS:45' gamma_e = 9.7803; */
  /* 'sensors_AHRS:46' DEG2RAD = pi/180; */
  /* 'sensors_AHRS:48' gyro = [imu_gyro_x_dps,imu_gyro_y_dps,imu_gyro_z_dps]; */
  /* 'sensors_AHRS:49' gyro = gyro*DEG2RAD; */
  gyro_idx_0 = imu_gyro_x_dps * 0.0174532924F;
  gyro_idx_1 = imu_gyro_y_dps * 0.0174532924F;
  gyro_idx_2 = imu_gyro_z_dps * 0.0174532924F;

  /* 'sensors_AHRS:50' accel_G = [imu_acc_x_G,imu_acc_y_G,imu_acc_z_G]; */
  /* 'sensors_AHRS:51' correction = [ 0 0 0]; */
  /* 'sensors_AHRS:52' if( norm(accel_G) < 0.001) */
  xbar = 1.29246971E-26F;
  correction_idx_0 = 0.0F;
  absxk = fabsf(imu_acc_x_G);
  if (absxk > 1.29246971E-26F) {
    a = 1.0F;
    xbar = absxk;
  } else {
    t = absxk / 1.29246971E-26F;
    a = t * t;
  }

  correction_idx_1 = 0.0F;
  absxk = fabsf(imu_acc_y_G);
  if (absxk > xbar) {
    t = xbar / absxk;
    a = a * t * t + 1.0F;
    xbar = absxk;
  } else {
    t = absxk / xbar;
    a += t * t;
  }

  correction_idx_2 = 0.0F;
  absxk = fabsf(imu_acc_z_G);
  if (absxk > xbar) {
    t = xbar / absxk;
    a = a * t * t + 1.0F;
    xbar = absxk;
  } else {
    t = absxk / xbar;
    a += t * t;
  }

  a = xbar * sqrtf(a);
  if (!(a < 0.001F)) {
    /* 'sensors_AHRS:56' q = ahrs_Quat_l_2_b(:); */
    /* 'sensors_AHRS:57' spinRate = norm(gyro); */
    xbar = 1.29246971E-26F;
    absxk = fabsf(gyro_idx_0);
    if (absxk > 1.29246971E-26F) {
      spinRate = 1.0F;
      xbar = absxk;
    } else {
      t = absxk / 1.29246971E-26F;
      spinRate = t * t;
    }

    absxk = fabsf(gyro_idx_1);
    if (absxk > xbar) {
      t = xbar / absxk;
      spinRate = spinRate * t * t + 1.0F;
      xbar = absxk;
    } else {
      t = absxk / xbar;
      spinRate += t * t;
    }

    absxk = fabsf(gyro_idx_2);
    if (absxk > xbar) {
      t = xbar / absxk;
      spinRate = spinRate * t * t + 1.0F;
      xbar = absxk;
    } else {
      t = absxk / xbar;
      spinRate += t * t;
    }

    spinRate = xbar * sqrtf(spinRate);

    /*  find magn times in buffer that fits imu time */
    /*      imu_time = imu_time; */
    /*      mag_valid = 0; */
    /* 'sensors_AHRS:61' mag_data = [0 0 0]; */
    mag_data_idx_0 = 0.0F;
    mag_data_idx_1 = 0.0F;
    mag_data_idx_2 = 0.0F;

    /* 'sensors_AHRS:62' local_mag_valid = 0; */
    local_mag_valid = 0;

    /* 'sensors_AHRS:63' if (mag_input_valid == 1) */
    if ((mag_input_valid == 1.0F) && (fabsf(imu_time - mag_time) < 0.05F)) {
      /*          magn_time = nav_dat.savedSensorBuffer.mag_data_buffer(1).magn_time; */
      /* 'sensors_AHRS:66' if(abs(imu_time - mag_time) < 0.05) */
      /* 'sensors_AHRS:67' local_mag_valid = 1; */
      local_mag_valid = 1;

      /* 'sensors_AHRS:68' mag_data = [mag_x,mag_y,mag_z]; */
      mag_data_idx_0 = mag_x;
      mag_data_idx_1 = mag_y;
      mag_data_idx_2 = mag_z;
    }

    /* 'sensors_AHRS:72' if (local_mag_valid == 1) */
    if (local_mag_valid == 1) {
      /*     		// Magnetometer correction */
      /* 		// Project mag field vector to global frame and extract XY component */
      /*      mag_earth = q.conjugate(mag); */
      /* 'sensors_AHRS:77' v = [mag_data(1), mag_data(2), mag_data(3), 0]; */
      /* 'sensors_AHRS:78' q_tmp = MNF_Quat_mul(q,v); */
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
      q_tmp_idx_0 = ((ahrs_Quat_l_2_b[3] * mag_data_idx_0 + ahrs_Quat_l_2_b[0] *
                      0.0F) + ahrs_Quat_l_2_b[1] * mag_data_idx_2) -
        ahrs_Quat_l_2_b[2] * mag_data_idx_1;

      /* 'MNF_Quat_mul:23' T(2)=R(4)*S(2)+R(2)*S(4)+R(3)*S(1)-R(1)*S(3); */
      q_tmp_idx_1 = ((ahrs_Quat_l_2_b[3] * mag_data_idx_1 + ahrs_Quat_l_2_b[1] *
                      0.0F) + ahrs_Quat_l_2_b[2] * mag_data_idx_0) -
        ahrs_Quat_l_2_b[0] * mag_data_idx_2;

      /* 'MNF_Quat_mul:24' T(3)=R(4)*S(3)+R(3)*S(4)+R(1)*S(2)-R(2)*S(1); */
      q_tmp_idx_2 = ((ahrs_Quat_l_2_b[3] * mag_data_idx_2 + ahrs_Quat_l_2_b[2] *
                      0.0F) + ahrs_Quat_l_2_b[0] * mag_data_idx_1) -
        ahrs_Quat_l_2_b[1] * mag_data_idx_0;

      /* 'MNF_Quat_mul:25' T(4)=R(4)*S(4)-R(1)*S(1)-R(2)*S(2)-R(3)*S(3); */
      q_tmp_idx_3 = ((ahrs_Quat_l_2_b[3] * 0.0F - ahrs_Quat_l_2_b[0] *
                      mag_data_idx_0) - ahrs_Quat_l_2_b[1] * mag_data_idx_1) -
        ahrs_Quat_l_2_b[2] * mag_data_idx_2;

      /* 'sensors_AHRS:79' q_inv = Quat_conjugate_4st(q); */
      /* 'Quat_conjugate_4st:4' Q_A2B=Q_A2B(:); */
      /* 'Quat_conjugate_4st:5' Q_B2A=[-Q_A2B(1:3);Q_A2B(4)]; */
      /* 'sensors_AHRS:80' q_res = MNF_Quat_mul(q_tmp,q_inv); */
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
      /* 'MNF_Quat_mul:24' T(3)=R(4)*S(3)+R(3)*S(4)+R(1)*S(2)-R(2)*S(1); */
      /* 'MNF_Quat_mul:25' T(4)=R(4)*S(4)-R(1)*S(1)-R(2)*S(2)-R(3)*S(3); */
      /* 'sensors_AHRS:81' mag_earth = [q_res(1) q_res(2) q_res(3)]; */
      /*              float mag_err = wrap_pi(atan2f(mag_earth(1), mag_earth(0)) - _mag_decl); */
      /* 'sensors_AHRS:84' mag_err = MNF_wrap_to_pi(atan2(mag_earth(2), mag_earth(1)) - mag_map_dec); */
      mag_err = rt_atan2f_snf(((q_tmp_idx_3 * -ahrs_Quat_l_2_b[1] + q_tmp_idx_1 *
        ahrs_Quat_l_2_b[3]) + q_tmp_idx_2 * -ahrs_Quat_l_2_b[0]) - q_tmp_idx_0 *
        -ahrs_Quat_l_2_b[2], ((q_tmp_idx_3 * -ahrs_Quat_l_2_b[0] + q_tmp_idx_0 *
        ahrs_Quat_l_2_b[3]) + q_tmp_idx_1 * -ahrs_Quat_l_2_b[2]) - q_tmp_idx_2 *
        -ahrs_Quat_l_2_b[1]) - mag_map_dec;

      /* wrapToPi Wrap angle in radians to [-pi pi] */
      /* 'MNF_wrap_to_pi:5' q = (value < -pi) | (pi < value); */
      b4 = (mag_err < -3.14159274F);
      b5 = (3.14159274F < mag_err);

      /* 'MNF_wrap_to_pi:6' value(q) = MNF_wrap_to_2_pi(value(q) + pi) - pi; */
      N = 0;
      if (b4 || b5) {
        N = 1;
      }

      emxInit_real32_T(&c, 2);
      i = c->size[0] * c->size[1];
      c->size[1] = N;
      c->size[0] = 1;
      emxEnsureCapacity_real32_T(c, i);
      for (i = 0; i < N; i++) {
        c->data[0] = mag_err + 3.14159274F;
      }

      emxInit_boolean_T(&positiveInput, 2);

      /* wrapTo2Pi Wrap angle in radians to [0 2*pi] */
      /*  */
      /* 'MNF_wrap_to_2_pi:5' positiveInput = (value > 0); */
      i = positiveInput->size[0] * positiveInput->size[1];
      positiveInput->size[1] = c->size[1];
      positiveInput->size[0] = 1;
      emxEnsureCapacity_boolean_T(positiveInput, i);
      N = c->size[1] * c->size[0];
      for (i = 0; i < N; i++) {
        positiveInput->data[i] = (c->data[i] > 0.0F);
      }

      emxInit_real32_T(&x, 2);

      /* 'MNF_wrap_to_2_pi:6' value = mod(value, 2*pi); */
      i = x->size[0] * x->size[1];
      x->size[1] = c->size[1];
      x->size[0] = 1;
      emxEnsureCapacity_real32_T(x, i);
      N = c->size[1] * c->size[0];
      for (i = 0; i < N; i++) {
        x->data[i] = c->data[i];
      }

      unnamed_idx_1 = (unsigned int)c->size[1];
      i = c->size[0] * c->size[1];
      c->size[1] = (int)unnamed_idx_1;
      c->size[0] = 1;
      emxEnsureCapacity_real32_T(c, i);
      N = (int)unnamed_idx_1;
      for (k = 0; k < N; k++) {
        xbar = x->data[k];
        if (rtIsNaNF(xbar) || rtIsInfF(xbar)) {
          absxk = rtNaNF;
        } else if (xbar == 0.0F) {
          absxk = 0.0F;
        } else {
          absxk = fmodf(xbar, 6.28318548F);
          rEQ0 = (absxk == 0.0F);
          if (!rEQ0) {
            t = fabsf(xbar / 6.28318548F);
            rEQ0 = !(fabsf(t - floorf(t + 0.5F)) > 1.1920929E-7F * t);
          }

          if (rEQ0) {
            absxk = 0.0F;
          } else {
            if (xbar < 0.0F) {
              absxk += 6.28318548F;
            }
          }
        }

        c->data[k] = absxk;
      }

      emxFree_real32_T(&x);
      emxInit_boolean_T(&r, 2);

      /* 'MNF_wrap_to_2_pi:7' value((value == 0) & positiveInput) = 2*pi; */
      i = r->size[0] * r->size[1];
      r->size[1] = c->size[1];
      r->size[0] = 1;
      emxEnsureCapacity_boolean_T(r, i);
      N = c->size[1] * c->size[0];
      for (i = 0; i < N; i++) {
        r->data[i] = (c->data[i] == 0.0F);
      }

      N = r->size[1];
      for (b_i = 0; b_i < N; b_i++) {
        if (r->data[b_i] && positiveInput->data[b_i]) {
          c->data[b_i] = 6.28318548F;
        }
      }

      emxFree_boolean_T(&r);
      emxFree_boolean_T(&positiveInput);
      i = c->size[1] * c->size[0];
      N = c->size[0] * c->size[1];
      c->size[0] = 1;
      emxEnsureCapacity_real32_T(c, N);
      N = i - 1;
      for (i = 0; i <= N; i++) {
        c->data[i] -= 3.14159274F;
      }

      absxk = mag_err;
      if (b4 || b5) {
        absxk = c->data[0];
      }

      emxFree_real32_T(&c);

      /* 'sensors_AHRS:86' gainMult = 1; */
      b_b = 1.0F;

      /* 'sensors_AHRS:88' mag_smal_gain = 0; */
      /* 'sensors_AHRS:89' if (spinRate > fifty_dps) */
      if (spinRate > 0.873F) {
        /* 'sensors_AHRS:90' gainMult = min(spinRate / fifty_dps, 10.0); */
        b_b = fminf(spinRate / 0.873F, 10.0F);

        /* 'sensors_AHRS:91' mag_smal_gain = 1; */
      }

      /*  // Project magnetometer correction to body frame */
      /*  corr += _q.conjugate_inversed(Vector3f(0.0f, 0.0f, -mag_err)) * _w_mag * gainMult; */
      /* 'sensors_AHRS:97' v = [0, 0, -mag_err]; */
      /* 'sensors_AHRS:98' q_v = [v, 0]; */
      /* 'sensors_AHRS:99' q_tmp = MNF_Quat_mul(Quat_conjugate_4st(q),q_v); */
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
      xbar = -ahrs_Quat_l_2_b[2] * 0.0F;
      t = -ahrs_Quat_l_2_b[0] * 0.0F;
      q_tmp_idx_0 = ((ahrs_Quat_l_2_b[3] * 0.0F + t) + -ahrs_Quat_l_2_b[1] *
                     -absxk) - xbar;

      /* 'MNF_Quat_mul:23' T(2)=R(4)*S(2)+R(2)*S(4)+R(3)*S(1)-R(1)*S(3); */
      q_tmp_idx_1_tmp = -ahrs_Quat_l_2_b[1] * 0.0F;
      q_tmp_idx_1 = ((ahrs_Quat_l_2_b[3] * 0.0F + q_tmp_idx_1_tmp) + xbar) -
        -ahrs_Quat_l_2_b[0] * -absxk;

      /* 'MNF_Quat_mul:24' T(3)=R(4)*S(3)+R(3)*S(4)+R(1)*S(2)-R(2)*S(1); */
      q_tmp_idx_2 = ((ahrs_Quat_l_2_b[3] * -absxk + xbar) + t) - q_tmp_idx_1_tmp;

      /* 'MNF_Quat_mul:25' T(4)=R(4)*S(4)-R(1)*S(1)-R(2)*S(2)-R(3)*S(3); */
      q_tmp_idx_3 = ((ahrs_Quat_l_2_b[3] * 0.0F - t) - q_tmp_idx_1_tmp) -
        -ahrs_Quat_l_2_b[2] * -absxk;

      /* 'sensors_AHRS:100' q_res = MNF_Quat_mul(q_tmp,q); */
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
      /* 'MNF_Quat_mul:24' T(3)=R(4)*S(3)+R(3)*S(4)+R(1)*S(2)-R(2)*S(1); */
      /* 'MNF_Quat_mul:25' T(4)=R(4)*S(4)-R(1)*S(1)-R(2)*S(2)-R(3)*S(3); */
      /* 'sensors_AHRS:101' q_res_vec = [q_res(1) q_res(2) q_res(3)]; */
      /* 'sensors_AHRS:102' correction = correction + q_res_vec * w_mag *gainMult; */
      correction_idx_0 = (((q_tmp_idx_3 * ahrs_Quat_l_2_b[0] + q_tmp_idx_0 *
                            ahrs_Quat_l_2_b[3]) + q_tmp_idx_1 * ahrs_Quat_l_2_b
                           [2]) - q_tmp_idx_2 * ahrs_Quat_l_2_b[1]) * 0.5F * b_b;
      correction_idx_1 = (((q_tmp_idx_3 * ahrs_Quat_l_2_b[1] + q_tmp_idx_1 *
                            ahrs_Quat_l_2_b[3]) + q_tmp_idx_2 * ahrs_Quat_l_2_b
                           [0]) - q_tmp_idx_0 * ahrs_Quat_l_2_b[2]) * 0.5F * b_b;
      correction_idx_2 = (((q_tmp_idx_3 * ahrs_Quat_l_2_b[2] + q_tmp_idx_2 *
                            ahrs_Quat_l_2_b[3]) + q_tmp_idx_0 * ahrs_Quat_l_2_b
                           [1]) - q_tmp_idx_1 * ahrs_Quat_l_2_b[0]) * 0.5F * b_b;
    }

    /*  Accelerometer correction */
    /* 	// Project 'k' unit vector of earth frame to body frame */
    /* 	// Vector3f k = q.conjugate_inversed(Vector3f(0.0f, 0.0f, 1.0f)); */
    /* 	// Optimized version with dropped zeros */
    /* 'sensors_AHRS:109' q_first_real = [q(4), q(1), q(2), q(3)]; */
    /* 'sensors_AHRS:110' k = [	2.0 * (q_first_real(2) * q_first_real(4) - q_first_real(1) * q_first_real(3)), ... */
    /* 'sensors_AHRS:111'     2.0 * (q_first_real(3) * q_first_real(4) + q_first_real(1) * q_first_real(2)),... */
    /* 'sensors_AHRS:112'     (q_first_real(1) * q_first_real(1) - q_first_real(2) * q_first_real(2) - q_first_real(3) * q_first_real(3) + q_first_real(4) * q_first_real(4))	]; */
    k_idx_0 = 2.0F * (ahrs_Quat_l_2_b[0] * ahrs_Quat_l_2_b[2] - ahrs_Quat_l_2_b
                      [3] * ahrs_Quat_l_2_b[1]);
    k_idx_1 = 2.0F * (ahrs_Quat_l_2_b[1] * ahrs_Quat_l_2_b[2] + ahrs_Quat_l_2_b
                      [3] * ahrs_Quat_l_2_b[0]);
    k_idx_2 = ((ahrs_Quat_l_2_b[3] * ahrs_Quat_l_2_b[3] - ahrs_Quat_l_2_b[0] *
                ahrs_Quat_l_2_b[0]) - ahrs_Quat_l_2_b[1] * ahrs_Quat_l_2_b[1]) +
      ahrs_Quat_l_2_b[2] * ahrs_Quat_l_2_b[2];

    /* 'sensors_AHRS:114' accel_norm_sq = norm(accel_G)^2; */
    accel_norm_sq = a * a;

    /* 'sensors_AHRS:115' upper_accel_limit =  1.1; */
    /* 'sensors_AHRS:116' lower_accel_limit =  0.9; */
    /*  check if imu is static enough to apply accelerometer correction and */
    /*  gyro bias correction */
    /* 'sensors_AHRS:121' if ( (accel_norm_sq > lower_accel_limit * lower_accel_limit & ... */
    /* 'sensors_AHRS:122'         accel_norm_sq < upper_accel_limit * upper_accel_limit)) */
    if ((accel_norm_sq > 0.809999943F) && (accel_norm_sq < 1.21F) && (spinRate <
         1.0F)) {
      /* 'sensors_AHRS:123' if (spinRate < SPIN_RATE_THRESH) */
      /* 'sensors_AHRS:125' imu_static = 1; */
      *imu_static = 1.0F;

      /*  corr += (k % (_accel - _pos_acc).normalized()) * _w_accel; */
      /* 'sensors_AHRS:127' correction = correction + cross(k,((accel_G )/norm(accel_G )))*w_accel; */
      q_tmp_idx_1_tmp = imu_acc_x_G / a;
      c_idx_1 = imu_acc_y_G / a;
      t = imu_acc_z_G / a;
      absxk = (k_idx_2 * q_tmp_idx_1_tmp - k_idx_0 * t) * 0.3F;
      xbar = (k_idx_0 * c_idx_1 - k_idx_1 * q_tmp_idx_1_tmp) * 0.3F;
      q_tmp_idx_1_tmp = correction_idx_0 + (k_idx_1 * t - k_idx_2 * c_idx_1) *
        0.3F;
      c_idx_1 = correction_idx_1 + absxk;
      t = correction_idx_2 + xbar;

      /* 'sensors_AHRS:130' ahrs_gyro_bias = ahrs_gyro_bias +  correction * (w_gyro_bias * dt); */
      b_b = 0.01F * dt;
      correction_idx_0 = q_tmp_idx_1_tmp;
      ahrs_gyro_bias[0] += q_tmp_idx_1_tmp * b_b;
      correction_idx_1 = c_idx_1;
      ahrs_gyro_bias[1] += c_idx_1 * b_b;
      correction_idx_2 = t;
      ahrs_gyro_bias[2] += t * b_b;
    }

    /*  Gyro bias estimation */
    /*  	if (spinRate < 0.175f) { */
    /*      if (norm(gyro) < 0.175) */
    /*  		gyro_bias = gyro_bias +  correction * (w_gyro_bias * dt); */
    /*      end */
    /*  		for (int i = 0; i < 3; i++) { */
    /*  			_gyro_bias(i) = math::constrain(_gyro_bias(i), -_bias_max, _bias_max); */
    /*  		} */
    /*  	} */
    /* 'sensors_AHRS:145' rates = gyro + ahrs_gyro_bias; */
    /*       rates = [0 0 0 ]; %% dbg */
    /*  	// Feed forward gyro */
    /* 'sensors_AHRS:149' correction = correction + rates; */
    correction_idx_0 += gyro_idx_0 + ahrs_gyro_bias[0];
    correction_idx_1 += gyro_idx_1 + ahrs_gyro_bias[1];
    correction_idx_2 += gyro_idx_2 + ahrs_gyro_bias[2];

    /*      correction = rates; */
    /*  	// Apply correction to state */
    /* 'sensors_AHRS:153' v = [ correction(1), correction(2), correction(3) , 0]; */
    /*  quat 4th real! */
    /* _q += _q.derivative1(corr) * dt; */
    /* 'sensors_AHRS:156' q_tmp = MNF_Quat_mul(q,v); */
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
    /* 'MNF_Quat_mul:24' T(3)=R(4)*S(3)+R(3)*S(4)+R(1)*S(2)-R(2)*S(1); */
    /* 'MNF_Quat_mul:25' T(4)=R(4)*S(4)-R(1)*S(1)-R(2)*S(2)-R(3)*S(3); */
    /* 'sensors_AHRS:157' q_tmp =  q_tmp * 0.5 * dt; */
    /* 'sensors_AHRS:158' q = q + q_tmp; */
    /*  Normalize quaternion */
    /* 'sensors_AHRS:160' q = Quat_normalize_4st(q); */
    /* 'Quat_normalize_4st:4' Q_norm=sqrt(sum(Q.^2)); */
    q_tmp_idx_1_tmp = ahrs_Quat_l_2_b[0] + (((ahrs_Quat_l_2_b[3] *
      correction_idx_0 + ahrs_Quat_l_2_b[0] * 0.0F) + ahrs_Quat_l_2_b[1] *
      correction_idx_2) - ahrs_Quat_l_2_b[2] * correction_idx_1) * 0.5F * dt;
    q_idx_0 = q_tmp_idx_1_tmp;
    xbar = rt_powf_snf(q_tmp_idx_1_tmp, 2.0F);
    q_tmp_idx_1_tmp = ahrs_Quat_l_2_b[1] + (((ahrs_Quat_l_2_b[3] *
      correction_idx_1 + ahrs_Quat_l_2_b[1] * 0.0F) + ahrs_Quat_l_2_b[2] *
      correction_idx_0) - ahrs_Quat_l_2_b[0] * correction_idx_2) * 0.5F * dt;
    q_idx_1 = q_tmp_idx_1_tmp;
    absxk = rt_powf_snf(q_tmp_idx_1_tmp, 2.0F);
    q_tmp_idx_1_tmp = ahrs_Quat_l_2_b[2] + (((ahrs_Quat_l_2_b[3] *
      correction_idx_2 + ahrs_Quat_l_2_b[2] * 0.0F) + ahrs_Quat_l_2_b[0] *
      correction_idx_1) - ahrs_Quat_l_2_b[1] * correction_idx_0) * 0.5F * dt;
    q_idx_2 = q_tmp_idx_1_tmp;
    t = rt_powf_snf(q_tmp_idx_1_tmp, 2.0F);
    q_tmp_idx_1_tmp = ahrs_Quat_l_2_b[3] + (((ahrs_Quat_l_2_b[3] * 0.0F -
      ahrs_Quat_l_2_b[0] * correction_idx_0) - ahrs_Quat_l_2_b[1] *
      correction_idx_1) - ahrs_Quat_l_2_b[2] * correction_idx_2) * 0.5F * dt;
    a = sqrtf(((xbar + absxk) + t) + rt_powf_snf(q_tmp_idx_1_tmp, 2.0F));

    /* 'Quat_normalize_4st:5' Q_norm=repmat(Q_norm,4,1); */
    /* 'Quat_normalize_4st:6' normalized_Q=Q./Q_norm; */
    q_idx_0 /= a;
    q_idx_1 /= a;
    q_idx_2 /= a;
    q_idx_3 = q_tmp_idx_1_tmp / a;

    /* 'Quat_normalize_4st:8' if normalized_Q(4)<0 */
    if (q_idx_3 < 0.0F) {
      /* 'Quat_normalize_4st:9' normalized_Q=-normalized_Q; */
      q_idx_0 = -q_idx_0;
      q_idx_1 = -q_idx_1;
      q_idx_2 = -q_idx_2;
      q_idx_3 = -q_idx_3;
    }

    /*  set ahrs output */
    /* 'sensors_AHRS:162' ahrs_Quat_l_2_b = q'; */
    ahrs_Quat_l_2_b[0] = q_idx_0;
    ahrs_Quat_l_2_b[1] = q_idx_1;
    ahrs_Quat_l_2_b[2] = q_idx_2;
    ahrs_Quat_l_2_b[3] = q_idx_3;

    /* 'sensors_AHRS:163' quat_l2b1 = q(1); */
    *quat_l2b1 = q_idx_0;

    /* 'sensors_AHRS:164' quat_l2b2 = q(2); */
    *quat_l2b2 = q_idx_1;

    /* 'sensors_AHRS:165' quat_l2b3 = q(3); */
    *quat_l2b3 = q_idx_2;

    /* 'sensors_AHRS:166' quat_l2b4 = q(4); */
    *quat_l2b4 = q_idx_3;

    /* 'sensors_AHRS:167' [euler_l2b] = MNF_Quat2Euler(q); */
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
    /* 'sensors_AHRS:168' Euler_l2b_psi  = euler_l2b(1); */
    xbar = q_idx_1 * q_idx_1;
    *Euler_l2b_psi = rt_atan2f_snf(2.0F * (q_idx_3 * q_idx_2 + q_idx_0 * q_idx_1),
      1.0F - 2.0F * (xbar + q_idx_2 * q_idx_2));

    /* 'sensors_AHRS:169' Euler_l2b_theta = euler_l2b(2); */
    *Euler_l2b_theta = asinf(2.0F * (q_idx_3 * q_idx_1 - q_idx_2 * q_idx_0));

    /* 'sensors_AHRS:170' Euler_l2b_phi = euler_l2b(3); */
    *Euler_l2b_phi = rt_atan2f_snf(2.0F * (q_idx_3 * q_idx_0 + q_idx_1 * q_idx_2),
      1.0F - 2.0F * (q_idx_0 * q_idx_0 + xbar));

    /*  raise ahrs counter */
    /* 'sensors_AHRS:172' static_counter = static_counter + 1; */
    static_counter++;

    /* 'sensors_AHRS:173' valid = 1; */
    *valid = 1.0F;

    /* 'sensors_AHRS:174' ahrs_counter = static_counter; */
    *ahrs_counter = static_counter;
  }
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
  /* (no terminate code required) */
  isInitialized_sensors_AHRS = false;
}

/*
 * File trailer for sensors_AHRS.c
 *
 * [EOF]
 */
