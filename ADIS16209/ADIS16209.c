/*
 *
 */

#include "ADIS16209_2.h"

MW_ADIS_RESULT regReadChannels(MW_ADIS_HandleTypeDef *hADIS, uint32_t channels)
{
	uint8_t byteCount = 0;
//channels are ored together
	if (channels & INCLINE_X) {
		aTxBuffer[byteCount++] = 0x4;
		aTxBuffer[byteCount++] = 0x4;
	}
	if (channels & INCLINE_Y) {
		aTxBuffer[byteCount++] = 0x4;
		aTxBuffer[byteCount++] = 0x4;		
	}
	if (channels & ACCEL_X) {
		aTxBuffer[byteCount++] = 0x4;
		aTxBuffer[byteCount++] = 0x4;		
	}
	if (channels & ACCEL_Y) {
		aTxBuffer[byteCount++] = 0x4;
		aTxBuffer[byteCount++] = 0x4;		
	}

	//need to sum the counting
	uint16_t bytesToSend = byteCount >> 1;  //divide by two

	//needs a timer to sned the data so don't do to fast.
	/* While the SPI in TransmitReceive process, user can transmit data through
	   "aTxBuffer" buffer & receive data through "aRxBuffer" */
	if (HAL_SPI_TransmitReceive_DMA(&SpiHandle, (uint8_t*)aTxBuffer, (uint8_t *)aRxBuffer, bytesToSend) != HAL_OK)
	{
		/* Transfer error in transmission process */
		Error_Handler();
	}
}

  /*need a function to be called from the SPI resposne in main*/
  void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
  {

  }


// Reads register (two bytes) Returns signed 16 bit data, into handle
  ADIS_RESULT regReadPoll(MW_ADIS_HandleTypeDef *hADIS, uint8_t regAddr, int16_t *result)
  {
	//number of bytes?	  
		  uint16_t NumberBytes = 2;
		  uint32_t Timeout = 1000;
		  ADIS_RESULT res = ADIS_OK;

	  switch (HAL_SPI_TransmitReceive(hADIS->SpiHandle, (uint8_t*)hADIS->aTxBuffer, (uint8_t *)hADIS->aRxBuffer, NumberBytes, Timeout))
	  {
	  case HAL_OK:
		  /* Communication is completed ___________________________________________ */		  
		  //result be n th aRxBuffer?		  
		  BSP_LED_On(LED2);
		  res = ADIS_OK;
		  break;
	  case HAL_TIMEOUT:
		  /* An Timeout Occur ______________________________________________________ */
		  res = ADIS_ERROR;		  
		  break;
	  case HAL_ERROR:		  
		  /* An Error Occur ______________________________________________________ */
		  res = ADIS_ERROR;		  
		  //hADIS->errorCallback;
		  //Error_Handler();
		  break;
	  default:
		  res = ADIS_ERROR;
		  break;
	  }
	  return res;
  }
