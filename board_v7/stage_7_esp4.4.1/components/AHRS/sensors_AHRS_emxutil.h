/*
 * File: sensors_AHRS_emxutil.h
 *
 * MATLAB Coder version            : 5.1
 * C/C++ source code generated on  : 31-Mar-2022 07:55:09
 */

#pragma once

/* Include Files */
#include "rtwtypes.h"
#include "sensors_AHRS_types.h"
#include <stddef.h>
#include <stdlib.h>
#ifdef __cplusplus

extern "C" {

#endif

  /* Function Declarations */
  extern void emxEnsureCapacity_boolean_T(emxArray_boolean_T *emxArray, int
    oldNumel);
  extern void emxEnsureCapacity_real32_T(emxArray_real32_T *emxArray, int
    oldNumel);
  extern void emxFree_boolean_T(emxArray_boolean_T **pEmxArray);
  extern void emxFree_real32_T(emxArray_real32_T **pEmxArray);
  extern void emxInit_boolean_T(emxArray_boolean_T **pEmxArray, int
    numDimensions);
  extern void emxInit_real32_T(emxArray_real32_T **pEmxArray, int numDimensions);

#ifdef __cplusplus

}
#endif

/*
 * File trailer for sensors_AHRS_emxutil.h
 *
 * [EOF]
 */
