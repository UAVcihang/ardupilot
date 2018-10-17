#ifndef _ARM_MATH_H
#define _ARM_MATH_H

#include <stdint.h>
#include <math.h>
  /**
   * @brief Instance structure for the floating-point matrix structure.
   */
#ifdef	__cplusplus
extern "C"
{
#endif

  typedef float float32_t;
   
  typedef struct
  {
    uint16_t numRows;     /**< number of rows of the matrix.     */
    uint16_t numCols;     /**< number of columns of the matrix.  */
    float32_t *pData;     /**< points to the data of the matrix. */
  } arm_matrix_instance_f32;
  
  
  typedef enum
  {
    ARM_MATH_SUCCESS = 0,                /**< No error */
    ARM_MATH_ARGUMENT_ERROR = -1,        /**< One or more arguments are incorrect */
    ARM_MATH_LENGTH_ERROR = -2,          /**< Length of data buffer is incorrect */
    ARM_MATH_SIZE_MISMATCH = -3,         /**< Size of matrices is not compatible with the operation. */
    ARM_MATH_NANINF = -4,                /**< Not-a-number (NaN) or infinity is generated */
    ARM_MATH_SINGULAR = -5,              /**< Generated by matrix inversion if the input matrix is singular and cannot be inverted. */
    ARM_MATH_TEST_FAILURE = -6           /**< Test Failed  */
  } arm_status;

  	void arm_std_f32(float32_t * pSrc, uint32_t blockSize, float32_t * pResult);
	void arm_mat_init_f32(arm_matrix_instance_f32 * S, uint16_t nRows, uint16_t nColumns,float32_t * pData);
	void arm_fill_f32(float32_t value, float32_t * pDst, uint32_t blockSize);
	arm_status arm_mat_mult_f32(const arm_matrix_instance_f32 * pSrcA, const arm_matrix_instance_f32 * pSrcB, arm_matrix_instance_f32 * pDst);
	arm_status arm_sqrt_f32(float32_t in, float32_t * pOut);
	arm_status arm_mat_trans_f32(const arm_matrix_instance_f32 * pSrc, arm_matrix_instance_f32 * pDst);
#ifdef  __cplusplus
}
#endif  /* end of __cplusplus */

  #endif
