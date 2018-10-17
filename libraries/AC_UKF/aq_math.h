#ifndef _aq_math_h
#define _aq_math_h

#include "arm_math.h"

#ifdef	__cplusplus
extern "C"
{
#endif

#ifndef __sqrtf
#define __sqrtf sqrtf
#endif

extern void matrixInit(arm_matrix_instance_f32 *m, int rows, int cols);
extern void matrixFree(arm_matrix_instance_f32 *m);
extern void matrixDump(char *name, arm_matrix_instance_f32 *m);
extern int qrDecompositionT_f32(arm_matrix_instance_f32 *A, arm_matrix_instance_f32 *Q, arm_matrix_instance_f32 *R);
extern void matrixDiv_f32(arm_matrix_instance_f32 *X, arm_matrix_instance_f32 *A, arm_matrix_instance_f32 *B, arm_matrix_instance_f32 *Q, arm_matrix_instance_f32 *R, arm_matrix_instance_f32 *AQ);

extern void vectorNormalize(float32_t *v, int n);
extern int cholF(float32_t *U);

extern void *aqDataCalloc(uint16_t count, uint16_t size);
void svd(float32_t *A, float32_t *S2, int n);


#ifdef  __cplusplus
}
#endif  /* end of __cplusplus */

#endif
