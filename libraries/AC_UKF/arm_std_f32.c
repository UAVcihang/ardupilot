#include "arm_math.h"
/**
 * @brief Standard deviation of the elements of a floating-point vector.
 * @param[in]       *pSrc points to the input vector
 * @param[in]       blockSize length of the input vector
 * @param[out]      *pResult standard deviation value returned here
 * @return none.
 *
 */


void arm_std_f32(
  float32_t * pSrc,
  uint32_t blockSize,
  float32_t * pResult)
{
  float32_t sum = 0.0f;                          /* Temporary result storage */
  float32_t sumOfSquares = 0.0f;                 /* Sum of squares */
  float32_t in;                                  /* input value */
  uint32_t blkCnt;                               /* loop counter */

#ifndef ARM_MATH_CM0_FAMILY

  /* Run the below code for Cortex-M4 and Cortex-M3 */

  float32_t meanOfSquares, mean, squareOfMean;

	if(blockSize == 1)
	{
		*pResult = 0;
		return;
	}

  /*loop Unrolling */
  blkCnt = blockSize >> 2u;

  /* First part of the processing with loop unrolling.  Compute 4 outputs at a time.
   ** a second loop below computes the remaining 1 to 3 samples. */
  while(blkCnt > 0u)
  {
    /* C = (A[0] * A[0] + A[1] * A[1] + ... + A[blockSize-1] * A[blockSize-1])  */
    /* Compute Sum of squares of the input samples
     * and then store the result in a temporary variable, sum. */
    in = *pSrc++;
    sum += in;
    sumOfSquares += in * in;
    in = *pSrc++;
    sum += in;
    sumOfSquares += in * in;
    in = *pSrc++;
    sum += in;
    sumOfSquares += in * in;
    in = *pSrc++;
    sum += in;
    sumOfSquares += in * in;

    /* Decrement the loop counter */
    blkCnt--;
  }

  /* If the blockSize is not a multiple of 4, compute any remaining output samples here.
   ** No loop unrolling is used. */
  blkCnt = blockSize % 0x4u;

  while(blkCnt > 0u)
  {
    /* C = (A[0] * A[0] + A[1] * A[1] + ... + A[blockSize-1] * A[blockSize-1]) */
    /* Compute Sum of squares of the input samples
     * and then store the result in a temporary variable, sum. */
    in = *pSrc++;
    sum += in;
    sumOfSquares += in * in;

    /* Decrement the loop counter */
    blkCnt--;
  }

  /* Compute Mean of squares of the input samples
   * and then store the result in a temporary variable, meanOfSquares. */
  meanOfSquares = sumOfSquares / ((float32_t) blockSize - 1.0f);

  /* Compute mean of all input values */
  mean = sum / (float32_t) blockSize;

  /* Compute square of mean */
  squareOfMean = (mean * mean) * (((float32_t) blockSize) /
                                  ((float32_t) blockSize - 1.0f));

  /* Compute standard deviation and then store the result to the destination */
  arm_sqrt_f32((meanOfSquares - squareOfMean), pResult);

#else

  /* Run the below code for Cortex-M0 */

  float32_t squareOfSum;                         /* Square of Sum */
  float32_t var;                                 /* Temporary varaince storage */

	if(blockSize == 1)
	{
		*pResult = 0;
		return;
	}

  /* Loop over blockSize number of values */
  blkCnt = blockSize;

  while(blkCnt > 0u)
  {
    /* C = (A[0] * A[0] + A[1] * A[1] + ... + A[blockSize-1] * A[blockSize-1]) */
    /* Compute Sum of squares of the input samples
     * and then store the result in a temporary variable, sumOfSquares. */
    in = *pSrc++;
    sumOfSquares += in * in;

    /* C = (A[0] + A[1] + ... + A[blockSize-1]) */
    /* Compute Sum of the input samples
     * and then store the result in a temporary variable, sum. */
    sum += in;

    /* Decrement the loop counter */
    blkCnt--;
  }

  /* Compute the square of sum */
  squareOfSum = ((sum * sum) / (float32_t) blockSize);

  /* Compute the variance */
  var = ((sumOfSquares - squareOfSum) / (float32_t) (blockSize - 1.0f));

  /* Compute standard deviation and then store the result to the destination */
  arm_sqrt_f32(var, pResult);

#endif /* #ifndef ARM_MATH_CM0_FAMILY */

}
