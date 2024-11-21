/*
 * File: rt_nonfinite.h
 *
 * MATLAB Coder version            : 5.5
 * C/C++ source code generated on  : 27-Mar-2023 15:12:25
 */

#pragma once

/* Include Files */
#include "rtwtypes.h"

#ifdef __cplusplus
extern "C" {
#endif

extern real_T rtInf;
extern real_T rtMinusInf;
extern real_T rtNaN;
extern real32_T rtInfF;
extern real32_T rtMinusInfF;
extern real32_T rtNaNF;

extern boolean_T rtIsInf(real_T value);
extern boolean_T rtIsInfF(real32_T value);
extern boolean_T rtIsNaN(real_T value);
extern boolean_T rtIsNaNF(real32_T value);

#ifdef __cplusplus
}
#endif

/*
 * File trailer for rt_nonfinite.h
 *
 * [EOF]
 */
