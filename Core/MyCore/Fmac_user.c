/******************************************************************************
*** @File           : Fmac_user.c
*** @Description    : Fmac配置及其电流环
*** @Attention      : 移植自ST官方例程
*** @Author         : TJL
*** @Date           : 2025/5/5
******************************************************************************/
#include "stm32g4xx_hal.h"
#include "Fmac_user.h"
#include "control_buck_boost.h"
#include "math.h"
FMAC_FilterConfigTypeDef sFmacConfig;

/* Array of filter coefficients A (feedback coefficients) in Q1.15 format */
static int16_t aFilterCoeffA[COEFF_VECTOR_A_SIZE] =
	{
		fmac_A1, fmac_A2, fmac_A3
};

/* Array of filter coefficients B (feed-forward taps) in Q1.15 format */
static int16_t aFilterCoeffB[COEFF_VECTOR_B_SIZE] =
	{
		fmac_B0, fmac_B1, fmac_B2, fmac_B3
};
/* Array of input values in Q1.15 format */
static int16_t aInputValues[ARRAY_SIZE] =
	{
		0};

/* Array of output data to preload in Q1.15 format */
static int16_t aOutputDataToPreload[COEFF_VECTOR_A_SIZE] =
	{
		0x0000, 0x0000, 0x0000};

/* Array of calculated filtered data in Q1.15 format */
static int16_t aCalculatedFilteredData[ARRAY_SIZE];

/* Expected number of calculated samples */
uint16_t ExpectedCalculatedOutputSize = MEMORY_PARAMETER_D2;

/* Status of the FMAC callbacks */
__IO uint32_t HalfOutputDataReadyCallback = CALLBACK_NOT_CALLED;
__IO uint32_t OutputDataReadyCallback = CALLBACK_NOT_CALLED;
__IO uint32_t ErrorCount = 0;

/* Array of reference filtered data for IIR "7 feed-forward taps, 6 feedback coefficients, gain = 1" in Q1.15 format */
static const int16_t aRefFilteredData[ARRAY_SIZE] =
	{
		0};


void Fmac_Init_user(void)
{

	/*## Configure the FMAC peripheral ###########################################*/
	sFmacConfig.InputBaseAddress = INPUT_BUFFER_BASE;
	sFmacConfig.InputBufferSize = INPUT_BUFFER_SIZE;
	sFmacConfig.InputThreshold = INPUT_THRESHOLD;
	sFmacConfig.CoeffBaseAddress = COEFFICIENT_BUFFER_BASE;
	sFmacConfig.CoeffBufferSize = COEFFICIENT_BUFFER_SIZE;
	sFmacConfig.OutputBaseAddress = OUTPUT_BUFFER_BASE;
	sFmacConfig.OutputBufferSize = OUTPUT_BUFFER_SIZE;
	sFmacConfig.OutputThreshold = OUTPUT_THRESHOLD;
	sFmacConfig.pCoeffA = aFilterCoeffA;
	sFmacConfig.CoeffASize = COEFF_VECTOR_A_SIZE;
	sFmacConfig.pCoeffB = aFilterCoeffB;
	sFmacConfig.CoeffBSize = COEFF_VECTOR_B_SIZE;
	sFmacConfig.Filter = FMAC_FUNC_IIR_DIRECT_FORM_1;
	sFmacConfig.OutputAccess = FMAC_BUFFER_ACCESS_DMA;
	sFmacConfig.P = COEFF_VECTOR_B_SIZE;
	sFmacConfig.Q = COEFF_VECTOR_A_SIZE;
	sFmacConfig.R = GAIN;
	sFmacConfig.CoeffBaseAddress = 0;
	sFmacConfig.InputAccess = FMAC_BUFFER_ACCESS_NONE;
	sFmacConfig.Clip = FMAC_CLIP_ENABLED;

	/* R parameter contains the post-shift value */
	if (HAL_FMAC_FilterConfig(&hfmac, &sFmacConfig) != HAL_OK)
	{
		/* Configuration Error */
		Error_Handler();
	}
	/*## Preload the input and output buffers ####################################*/
	//   if (HAL_FMAC_FilterPreload(&hfmac, aInputValues,INPUT_BUFFER_SIZE,aOutputDataToPreload, COEFF_VECTOR_A_SIZE) != HAL_OK)
	if (HAL_FMAC_FilterPreload(&hfmac, NULL, INPUT_BUFFER_SIZE, aOutputDataToPreload, COEFF_VECTOR_A_SIZE) != HAL_OK)
	{
		/* Configuration Error */
		Error_Handler();
	}

	/*## Start calculation of IIR filter in polling/DMA mode #####################*/
	if (HAL_FMAC_FilterStart(&hfmac, aCalculatedFilteredData, &ExpectedCalculatedOutputSize) != HAL_OK)
	{
		/* Processing Error */
		Error_Handler();
	}
}

void Fmac_write_data(float fdata)
{
	int16_t CORDIC15;
	CORDIC15 = (int)((fdata) * 0x8000);
	hfmac.Instance->WDATA = CORDIC15;
}

void Fmac_perload_reload(int16_t Data)
{
	for (uint32_t i = 0; i < COEFF_VECTOR_A_SIZE; i++)
	{
		aOutputDataToPreload[i] = Data;
	}
	HAL_FMAC_FilterPreload(&hfmac, NULL, INPUT_BUFFER_SIZE, aOutputDataToPreload, COEFF_VECTOR_A_SIZE);
}

volatile float Fmac_Read_data(void)
{
	float temp;
	if (aCalculatedFilteredData[0] & 0x8000) // 处理低16位
	{										 /*为负数*/
		temp = ((float)(aCalculatedFilteredData[0] & 0x7FFF) - 0x8000) / 0x8000;
	}
	else
	{ /*为正数*/
		temp = (float)(aCalculatedFilteredData[0] & 0xFFFF) / 0x8000;
	}
	return (temp + 1) * 13.5f;
}
/**
 * @brief FMAC half output data ready callback
 * @par hfmac: FMAC HAL handle
 * @retval None
 */
void HAL_FMAC_HalfOutputDataReadyCallback(FMAC_HandleTypeDef *hfmac)
{
	HalfOutputDataReadyCallback = CALLBACK_CALLED;
}

/**
 * @brief FMAC output data ready callback
 * @par hfmac: FMAC HAL handle
 * @retval None
 */
extern uint32_t cnt_on_I;
void HAL_FMAC_OutputDataReadyCallback(FMAC_HandleTypeDef *hfmac)
{

	OutputDataReadyCallback = CALLBACK_CALLED;
	//***I_Loop_Fmac */
	cnt_on_I++;
	float temp = Fmac_Read_data(); // 0-0.125
	Cap_Data.Ratio = (temp) / Sample_Data.sample_Pin24V_V;
	Duty_calcuate_V2_0(Cap_Data.Ratio);
	//**************** */
}

/**
 * @brief FMAC error callback
 * @par hfmac: FMAC HAL handle
 * @retval None
 */
void HAL_FMAC_ErrorCallback(FMAC_HandleTypeDef *hfmac)
{
	ErrorCount++;
}
