/* Includes ------------------------------------------------------------------*/
#include "datalog_application.h"
#include "main.h"
#include "cmsis_os.h"
#include "usbd_cdc_interface.h"
#include "string.h"
#include "STWIN.h"


#if SENSIML_RECOGNITION
#include "kb.h"
#include "sml_recognition_run.h"
#endif


#include "stm32l4xx_hal.h"
#include "STWIN_audio.h"
#include "STWIN_bus.h"
#include "audio.h"

#if SENSIML_RECOGNITION
extern osMessageQId recogDataQueue_id;
extern osPoolId recogSensorPool_id;
#else
extern osMessageQId sensorDataQueue_id;
extern osPoolId sensorPool_id;
#endif
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/** @defgroup AUDIO_APPLICATION_Exported_Variables
 * @{
 */
BSP_AUDIO_Init_t MicParams;


uint16_t PCM_Buffer[DEFAULT_AUDIO_IN_BUFFER_SIZE/2];


/**
 * @}
 */

/** @defgroup AUDIO_APPLICATION_Private_Variables
 * @{
 */
/* Private variables ---------------------------------------------------------*/
/**
 * @}
 */

/** @defgroup AUDIO_APPLICATION_Exported_Function
 * @{
 */

/**
 * @brief  Half Transfer user callback, called by BSP functions.
 * @param  None
 * @retval None
 */
void BSP_AUDIO_IN_HalfTransfer_CallBack(uint32_t Instance) {
	UNUSED(Instance);
	AudioProcess();
	BSP_LED_Off(LED1);
}

/**
 * @brief  Transfer Complete user callback, called by BSP functions.
 * @param  None
 * @retval None
 */
void BSP_AUDIO_IN_TransferComplete_CallBack(uint32_t Instance) {
	UNUSED(Instance);
	AudioProcess();
	BSP_LED_On(LED1);
}

/**
 * @brief  User function that is called when 1 ms of PDM data is available.
 * 		  In this application only PDM to PCM conversion and USB streaming
 *                  is performed.
 * 		  User can add his own code here to perform some DSP or audio analysis.
 * @param  none
 * @retval None
 */


void AudioProcess(void) {
	T_AudioData *pAudioData;
	osMessageQId q_id;
	osPoolId pool_id;
#if SENSIML_RECOGNITION
	q_id = recogDataQueue_id;
	pool_id = recogSensorPool_id;

#else
	q_id = sensorDataQueue_id;
	pool_id = sensorPool_id;
#endif
	pAudioData = (T_AudioData*) osPoolCAlloc(pool_id);
	if (pAudioData != NULL) {
		memcpy((uint8_t *)pAudioData->audio, (uint8_t *) PCM_Buffer, DEFAULT_AUDIO_IN_BUFFER_SIZE);
		osMessagePut(q_id, (uint32_t) pAudioData, 0);
	}
}

/**
 * @brief  User function that is called when 1 ms of PDM data is available.
 * 		  In this application only PDM to PCM conversion and USB streaming
 *                  is performed.
 * 		  User can add his own code here to perform some DSP or audio analysis.
 * @param  none
 * @retval None
 */
void Audio_Init_Acquisition_Peripherals(uint32_t AudioFreq, uint32_t Device,
		uint32_t ChnlNbr) {
	MicParams.BitsPerSample = 16;
	MicParams.ChannelsNbr = ChnlNbr;
	MicParams.Device = Device;
	MicParams.SampleRate = AudioFreq;
	MicParams.Volume = AUDIO_VOLUME_INPUT;

	if (BSP_AUDIO_IN_Init(BSP_AUDIO_IN_INSTANCE, &MicParams) != BSP_ERROR_NONE) {
		Error_Handler();
	}
}

/**
 * @brief  User function that is called when 1 ms of PDM data is available.
 * 		  In this application only PDM to PCM conversion and USB streaming
 *                  is performed.
 * 		  User can add his own code here to perform some DSP or audio analysis.
 * @param  none
 * @retval None
 */
void Audio_Start_Acquisition(void) {
	if (BSP_AUDIO_IN_Record(BSP_AUDIO_IN_INSTANCE, (uint8_t*) PCM_Buffer,
	DEFAULT_AUDIO_IN_BUFFER_SIZE) != BSP_ERROR_NONE) {
		Error_Handler();
	}
}

void Audio_Stop_Acquisition(void) {
	if (BSP_AUDIO_IN_Stop(BSP_AUDIO_IN_INSTANCE) != BSP_ERROR_NONE) {
		Error_Handler();
	}
}

void Error_Handler(void) {
	while (1)
		;
}
