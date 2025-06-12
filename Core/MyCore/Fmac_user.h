/******************************************************************************
*** @File           : Fmac_user.h
*** @Description    : Fmac配置及其电流环
*** @Attention      : None
*** @Author         : TJL
*** @Date           : 2025/5/5
*** @版权归属:
*                        ___          ___          ___
*             ___       /\__\        /\  \        /\  \
*            /\  \     /::|  |      /::\  \      /::\  \
*            \:\  \   /:|:|  |     /:/\:\  \    /:/\:\  \
*            /::\__\ /:/|:|__|__  /:/  \:\  \  /::\~\:\  \
*         __/:/\/__//:/ |::::\__\/:/__/ \:\__\/:/\:\ \:\__\
*        /\/:/  /   \/__/~~/:/  /\:\  \  \/__/\/__\:\/:/  /
*        \::/__/          /:/  /  \:\  \           \::/  /
*         \:\__\         /:/  /    \:\  \          /:/  /
*          \/__/        /:/  /      \:\__\        /:/  /
*                       \/__/        \/__/        \/__/
******************************************************************************/
#ifndef _Fmac_user_H
#define _Fmac_user_H

#include "fmac.h"

#define CALLBACK_NOT_CALLED     0
#define CALLBACK_CALLED         1

/* Size of the data arrays */
#define ARRAY_SIZE              1

#define  fmac_A3 0
#define  fmac_B3 0

/////////////////////////////////////////////////

#define fmac_B0 (0x52FA)
#define fmac_B1	(0x392)
#define fmac_B2	(0xB099)
#define fmac_A1	(0x6394)
#define fmac_A2	(0x1C6C)
#define post_shift (+0)

/////////////////////////////////////////////////



/* Filter parameter P: number of feed-forward taps or coefficients in the range [2:64] */
#define COEFF_VECTOR_B_SIZE     4

/* Filter parameter Q: number of feedback coefficients in the range [1:COEFF_VECTOR_B_SIZE-1] */
#define COEFF_VECTOR_A_SIZE     3

/* Filter parameter R: gain in the range [0:7] */
#define GAIN                    post_shift



/* Throughput parameter: extra space in the input buffer (minimum: 0) */
#define MEMORY_PARAMETER_D1     1

/* Throughput parameter: extra space in the output buffer (minimum: 1) */
#define MEMORY_PARAMETER_D2     1

/* Throughput parameter: watermark threshold for the input buffer */
#define INPUT_THRESHOLD         FMAC_THRESHOLD_1

/* Throughput parameter: watermark threshold for the output buffer (inferior or equal to MEMORY_PARAMETER_D1) */
#define OUTPUT_THRESHOLD        FMAC_THRESHOLD_1



/* FMAC internal memory configuration: base address of the coefficient buffer */
#define COEFFICIENT_BUFFER_BASE 0

/* FMAC internal memory configuration: size of the two coefficient buffers */
#define COEFFICIENT_BUFFER_SIZE COEFF_VECTOR_B_SIZE + COEFF_VECTOR_A_SIZE

/* FMAC internal memory configuration: base address of the input buffer */
#define INPUT_BUFFER_BASE       COEFFICIENT_BUFFER_SIZE

/* FMAC internal memory configuration: size of the input buffer */
#define INPUT_BUFFER_SIZE       COEFF_VECTOR_B_SIZE + MEMORY_PARAMETER_D1

/* FMAC internal memory configuration: base address of the input buffer */
#define OUTPUT_BUFFER_BASE      COEFFICIENT_BUFFER_SIZE + INPUT_BUFFER_SIZE

/* FMAC internal memory configuration: size of the input buffer */
#define OUTPUT_BUFFER_SIZE      COEFF_VECTOR_A_SIZE + MEMORY_PARAMETER_D2

void famc_test(void);

void Fmac_Init_user(void);

void Fmac_write_data(float fdata);

void Fmac_perload_reload(int16_t Data);

volatile  float Fmac_Read_data(void);

#endif


