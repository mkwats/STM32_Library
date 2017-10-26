////////need a microsecond delay, have to use ms, until use timer.


#ifndef ADIS16209_H_
#define ADIS16209_H_

#include <stdio.h>
#include <stdint.h>
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_spi.h"
#include "stm32l4xx_hal_tim.h"

// User Register Memory Map from Table 6
#define ENDURANCE 0x00  //Flash memory write count
#define SUPPLY_OUT 0x02 //Power supply measurement output
#define XACCL_OUT 0x04  //X-axis accelerometer output
#define YACCL_OUT 0x06  //Y-axis accelerometer output
#define AUX_ADC 0x08    //Auxialiary ADC output
#define TEMP_OUT 0x0A   //Temperature output
#define XINCL_OUT 0x0C  //X-axis inclinometer ouput, horizontal
#define YINCL_OUT 0x0E  //Y-axis inclinometer ouput, horizontal
#define ROT_OUT 0x10    //Vertical orientation output
#define XACCL_NULL 0x12 //X-axis accelerometer offset correction factor
#define YACCL_NULL 0x14 //Y-axis accelerometer offset correction factor
#define XINCL_NULL 0x16 //X-axis inclinometer offset correction factor
#define YINCL_NULL 0x18 //Y-axis inclinometer offset correction factor
#define ROT_NULL 0x1A   //Vertical orientation offset correction factor
#define ALM_MAG1 0x20   //Alarm 1 amplitude threshold
#define ALM_MAG2 0x22   //Alarm 2 amplitude threshold
#define ALM_SMPL1 0x24  //Alarm 1 sample size/time
#define ALM_SMPL2 0x26  //Alarm 2 sample size/time
#define ALM_CTRL 0x28   //Alarm control
#define AUX_DAC 0x30    //Auxilary DAC data input
#define GPIO_CTRL 0x32  //GPIO control
#define MSC_CTRL 0x34   //MISC control
#define SMPL_PRD 0x36   //Sample clock/Decimation filter control
#define AVG_CNT 0x38    //Average count control (filter setting)
#define SLP_CNT 0x3A    //Sleep mode control
#define STATUS 0x3C     //System status (error flags)
#define COMMAND 0x3E    //System global commands
#define PROD_ID 0x4A   //Product identifier (16,209 = 0x3F51)

/* Macros */
#define __SET_PIN(__PIN_TO_SET__)		HAL_GPIO_WritePin(__PIN_TO_SET__.port, __PIN_TO_SET__.pin, GPIO_PIN_SET);
#define __RESET_PIN(__PIN_TO_SET__)		HAL_GPIO_WritePin(__PIN_TO_SET__.port, __PIN_TO_SET__.pin, GPIO_PIN_RESET);

//need to check these --- maybe use arduino for quick output
//floats are 4 bytes
#define __MW_ADIS_SUPPLY_SCALED(__SENSOR_DATA__) (uint16_t)(__SENSOR_DATA__ & 0x3FFF * 0.000305176)
#define __MW_ADIS_TEMP_SCALED(__SENSOR_DATA__) (uint16_t)((__SENSOR_DATA__ & 0x0FFF) - 1278) * -0.47) + 25))
#define __MW_ADIS_ACCEL_SCALED(__SENSOR_DATA__) (float)((__SENSOR_DATA__ & 2000) || ((__SENSOR_DATA__ & 1FFF)*0.24414))
#define __MW_ADIS_INCLINE_SCALED(__SENSOR_DATA__) (float)((__SENSOR_DATA__ & 2000) || ((__SENSOR_DATA__ & 1FFF)>>2)) //divide by 4


/* channels to sample */
#define ADIS_INCLINE_X		1
#define ADIS_INCLINE_Y		2
#define ADIS_ACCEL_X		4
#define ADIS_ACCEL_Y		8
#define ADIS_TEMP			16
#define	ADIS_SUPPLY			32


typedef enum {
	ADIS_OK,		/*!< Everything went OK */
	ADIS_ERROR	/*!< An error occured */
} MW_ADIS_RESULT;

typedef struct {
	GPIO_TypeDef* 			port;
	uint16_t 				pin;
} MW_ADIS_PINPORT;

typedef struct {	
	SPI_HandleTypeDef		SpiHandle;						/*Handle to the SPI*/
	TIM_HandleTypeDef		TimHandle;						/*Handle to the timer*/
	uint8_t*				aTxBuffer;						/*Tx Buffer*/
	uint8_t*				aRxBuffer;						/*Rx buffer*/
	uint8_t*				TxToSend;						/*bytes left to send*/
	uint8_t*				aTxByteToSend[2];				/*byte to send on next OC*/
	uint16_t*				data;						/*Data received*/
	uint16_t				TxTimeout;						/*TxTimeout*/
	uint16_t				RxTimeout;						/*RxTimeout*/
	MW_ADIS_PINPORT			CS;								/*chip select pin*/
	MW_ADIS_PINPORT			DR;								/*data pin*/
	MW_ADIS_PINPORT			RST;							/*RESET PIN, needs port too*/	
	uint8_t 				state;							/** errors here*/	
	uint16_t				channels;						/** OR the channels together */
} MW_ADIS_HandleTypeDef;

/*Setup the SPI to suit*/
MW_ADIS_RESULT initSPI(MW_ADIS_HandleTypeDef *hADIS)
{
	hADIS->SpiHandle.Instance = SPI2;
	hADIS->SpiHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
	hADIS->SpiHandle.Init.Direction = SPI_DIRECTION_2LINES;
	hADIS->SpiHandle.Init.CLKPhase = SPI_PHASE_1EDGE;
	hADIS->SpiHandle.Init.CLKPolarity = SPI_POLARITY_LOW;
	hADIS->SpiHandle.Init.DataSize = SPI_DATASIZE_16BIT;
	hADIS->SpiHandle.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hADIS->SpiHandle.Init.TIMode = SPI_TIMODE_DISABLE;
	hADIS->SpiHandle.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hADIS->SpiHandle.Init.CRCPolynomial = 7;
	hADIS->SpiHandle.Init.CRCLength = SPI_CRC_LENGTH_8BIT;
	hADIS->SpiHandle.Init.NSS = SPI_NSS_SOFT;
	hADIS->SpiHandle.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;

	if (HAL_SPI_Init(hADIS->SpiHandle) != HAL_OK)
	{
	_Error_Handler(__FILE__, __LINE__);
	}

	return ADIS_OK;
}



// Reads register (two bytes) Returns signed 16 bit data, into handle
// regAddr - address of register to be read
// return - (int) signed 16 bit 2's complement number
MW_ADIS_RESULT regReadPoll(MW_ADIS_HandleTypeDef *hADIS, uint16_t regAddr, int16_t *result)
{
	MW_ADIS_RESULT res = ADIS_OK;
	uint16_t NumberBytes = 2;
	HAL_StatusTypeDef HAL_Result = HAL_OK;

	//need to spit the regAddr in upper and lower.. 
	//does it need to be a pointer 		

	hADIS->aTxBuffer[0] = (uint8_t*)(0x0);
	hADIS->aTxBuffer[1] = (uint8_t*)(0x0);

	//Send request for data from address
	__RESET_PIN(hADIS->CS)
	HAL_Result = HAL_SPI_TransmitReceive(hADIS->SpiHandle, (uint8_t*)hADIS->aTxBuffer, (uint8_t *)hADIS->aRxBuffer, NumberBytes, hADIS->TxTimeout);
	ADIS_Handle_HAL_Response(HAL_Result);
	__SET_PIN(hADIS->CS)		// Set CS high to disable device
	HAL_Delay(1);				// Delay to not violate read rate (40us)


	// Read data from requested register
	__RESET_PIN(hADIS->CS)		// Set CS low to enable device		
	HAL_Result = HAL_SPI_TransmitReceive(hADIS->SpiHandle, (uint8_t*)hADIS->aTxBuffer, (uint8_t *)hADIS->aRxBuffer, NumberBytes, hADIS->TxTimeout);
	ADIS_Handle_HAL_Response(HAL_Result);
	result = hADIS->aRxBuffer[0] << 8 || hADIS->aRxBuffer[1];
	//uint8_t _msbData = SPI.transfer(0x00); // Send (0x00) and place upper byte into variable
	//uint8_t _lsbData = SPI.transfer(0x00); // Send (0x00) and place lower byte into variable
	__SET_PIN(hADIS->CS)
	HAL_Delay(1);				// Delay to not violate read rate (40us)

	hADIS->data = result;
								// Shift MSB data left by 8 bits, mask LSB data with 0xFF, and OR both bits
	//result = (_msbData << 8) | (_lsbData & 0xFF); // Concatenate upper and lower bytes
	return res;
}


// Write register (two bytes). Returns 1 when complete.
MW_ADIS_RESULT regWritePoll(MW_ADIS_HandleTypeDef *hADIS, uint8_t regAddr, int16_t regData)
{
	MW_ADIS_RESULT res = ADIS_OK;
	uint8_t NumberBytes = 4;
	HAL_StatusTypeDef HAL_Result = HAL_OK;

	// Write register address and data
	uint16_t addr = (((regAddr & 0x7F) | 0x80) << 8); // Toggle sign bit, and check that the address is 8 bits
	uint16_t lowWord = (addr | (regData & 0xFF)); // OR Register address (A) with data(D) (AADD)
	uint16_t highWord = ((addr | 0x100) | ((regData >> 8) & 0xFF)); // OR Register address with data and increment address

																	// Split words into chars			
	hADIS->aTxBuffer[0] = (uint8_t*)(highWord >> 8);
	hADIS->aTxBuffer[1] = (uint8_t*)(highWord & 0xFF);
	hADIS->aTxBuffer[2] = (uint8_t*)(lowWord >> 8);
	hADIS->aTxBuffer[3] = (uint8_t*)(lowWord & 0xFF);
	//uint8_t highBytehighWord = (highWord >> 8);
	//uint8_t lowBytehighWord = (highWord & 0xFF);
	//uint8_t highBytelowWord = (lowWord >> 8);
	//uint8_t lowBytelowWord = (lowWord & 0xFF);

	// Write highWord to SPI bus
	__RESET_PIN(hADIS->CS)		// Set CS low to enable device		

	HAL_Result = HAL_SPI_TransmitReceive(hADIS->SpiHandle, (uint8_t*)hADIS->aTxBuffer, (uint8_t *)hADIS->aRxBuffer, NumberBytes, hADIS->TxTimeout);
	ADIS_Handle_HAL_Response(HAL_Result);
	//SPI.transfer(highBytehighWord); // Write high byte from high word to SPI bus
	//SPI.transfer(lowBytehighWord); // Write low byte from high word to SPI bus

	__SET_PIN(hADIS->CS)

	HAL_Delay(1);
	//delayMicroseconds(40); // Delay to not violate read rate (40us)

	// Write lowWord to SPI bus
	__RESET_PIN(hADIS->CS)		// Set CS low to enable device		
								//digitalWrite(_CS, LOW); // Set CS low to enable device

	HAL_Result = HAL_SPI_TransmitReceive(hADIS->SpiHandle, (uint8_t*)hADIS->aTxBuffer, (uint8_t *)hADIS->aRxBuffer, NumberBytes, hADIS->TxTimeout);
	ADIS_Handle_HAL_Response(HAL_Result);
	//SPI.transfer(highBytelowWord); // Write high byte from low word to SPI bus
	//SPI.transfer(lowBytelowWord); // Write low byte from low word to SPI bus

	__SET_PIN(hADIS->CS)
	//digitalWrite(_CS, HIGH); // Set CS high to disable device

	return res;
}


//needs to be called from the TIMER OC
MW_ADIS_RESULT ADIS_SendStuff(*hADIS) 
{		
#if defined (SEND_SUFF_IT)
	(uint8_t*)hADIS->aTxByteToSend[2] = aTxBuffer[TxByteCount]
			
	if (HAL_SPI_TransmitReceive_DMA(&SpiHandle, (uint8_t*)aTxBuffer, (uint8_t *)aRxBuffer, bytesToSend) != HAL_OK)
	{
		/* Transfer error in transmission process */
		Error_Handler();
	}
		
	hADIS->TxByteCount == Max ? 0 : += 1;//increment the counter;
#endif 
	return ADIS_OK;
}


// Performs hardware reset. Delay in miliseconds.
MW_ADIS_RESULT resetADIS(MW_ADIS_HandleTypeDef *hADIS, uint8_t ms)
{
	__RESET_PIN(hADIS->RST)
	HAL_Delay(100);
	__SET_PIN(hADIS->RST)
	HAL_Delay(ms);
	return ADIS_OK;
};
	


//put error handle into one function
MW_ADIS_RESULT ADIS_Handle_HAL_Response(HAL_StatusTypeDef HAL_Result)
{
	MW_ADIS_RESULT res = ADIS_OK;
	switch (HAL_Result) {
	case HAL_OK:
		/* Communication is completed ___________________________________________ */
		//result be n th aRxBuffer?
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



//Start DMA  
MW_ADIS_RESULT regReadChannels(MW_ADIS_HandleTypeDef *hADIS, uint32_t channels) 
{
#if defined(REGULAR_READ)
//need to send recieve one at a tiem using teh hDIS->TimHandle
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

	  //start timer and do a send every trigger
	  HAL_TIM_Base_Start_IT(&htim5);

	  //stop when all done
	  //HAL_TIM_Base_Stop(&htim5);


											//needs a timer to sned the data so don't do to fast.
											/* While the SPI in TransmitReceive process, user can transmit data through
											"aTxBuffer" buffer & receive data through "aRxBuffer" */
	if (HAL_SPI_TransmitReceive_DMA(&SpiHandle, (uint8_t*)aTxBuffer, (uint8_t *)aRxBuffer, bytesToSend) != HAL_OK)
	{
		/* Transfer error in transmission process */
		Error_Handler();
	}
#endif
	return ADIS_OK;
}
/*Setup the GPIOs to suit*/
MW_ADIS_RESULT initGPIO(MW_ADIS_HandleTypeDef *hADIS)
{
#if defined(INIT_GPIO)
	/*##-1- Enable peripherals and GPIO Clocks #################################*/
	/* TIM1 Peripheral clock enable */
	TIMx_CLK_ENABLE();

	/* Enable GPIO Channels Clock */
	TIMx_CHANNEL_GPIO_PORT;

	/*##-2- Configure I/Os #####################################################*/
	/* Configure PA.08 (TIM1_Channel1), PA.09 (TIM1_Channel2), PA.10 (TIM1_Channel3),
	PA.11 (TIM1_Channel4) in output, pull-up, alternate function mode
	*/
	/* Common configuration for all channels */
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
#endif
	return ADIS_OK;
}

/*Setup timer to send the data*/
MW_ADIS_RESULT initTimer(MW_ADIS_HandleTypeDef *hADIS)
{
#if defined(INIT_TIM)
	/* Timer Output Compare Configuration Structure declaration */
	TIM_OC_InitTypeDef sConfig;

	/* Compute the prescaler value to have TIMx counter clock equal to 10 kHz *////need this every 50uS
	uint32_t uwPrescalerValue = ((SystemCoreClock) / 10000) - 1;

	/*##-1- Configure the TIM peripheral #######################################*/
	hADIS->TimHandle.Instance = TIM1;
	hADIS->TimHandle.Init.Period = 65535;
	hADIS->TimHandle.Init.Prescaler = uwPrescalerValue;
	hADIS->TimHandle.Init.ClockDivision = 0;
	hADIS->TimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;

	if (HAL_TIM_OC_Init(hADIS->TimHandle) != HAL_OK)
	{
		Error_Handler();
	}

	/*##-2- Configure the Output Compare channels #########################################*/
	sConfig.OCMode = TIM_OCMODE_ACTIVE;
	sConfig.OCPolarity = TIM_OCPOLARITY_HIGH;

	/* Set the pulse (delay1)  value for channel 1 */
	sConfig.Pulse = PULSE1_VALUE;
	if (HAL_TIM_OC_ConfigChannel(hADIS->TimHandle, &sConfig, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
#endif
	return ADIS_OK;
}

#if defined(HIDE_THIS_STUFF)
	// Scale accelerometer data. Returns scaled data as float.
	float accelScale(int16_t sensorData);

	// Scale incline data. Returns scaled data as float.
	float inclineScale(int16_t sensorData);

	// Scale temperature data. Returns scaled data as float.
	float tempScale(int16_t sensorData);

	// Scale VDD supply. Returns scaled data as float.
	float supplyScale(int16_t sensorData);

	/////////////////////////////////////////////////////////////////////////////////////////
	// Converts accelerometer data output from the regRead() function and returns
	// acceleration in mg's
	/////////////////////////////////////////////////////////////////////////////////////////
	// sensorData - data output from regRead()
	// return - (float) signed/scaled accelerometer in mg's
	/////////////////////////////////////////////////////////////////////////////////////////
	float ADIS16209::accelScale(int16_t sensorData)
	{
		int signedData = 0;
		sensorData = sensorData & 0x3FFF; // Discard upper two bits
		int isNeg = sensorData & 0x2000;
		if (isNeg == 0x2000) // If the number is negative, scale and sign the output
			signedData = sensorData - 0x3FFF;
		else
			signedData = sensorData;
		float finalData = signedData * 0.24414; // Multiply by accel sensitivity (244.14 uG/LSB)
		return finalData;
	}

	/////////////////////////////////////////////////////////////////////////////////////////////
	// Converts incline angle data output from the regRead() function and returns incline angle
	// in degrees
	/////////////////////////////////////////////////////////////////////////////////////////////
	// sensorData - data output from regRead()
	// return - (float) signed/scaled incline or rotation in degrees
	/////////////////////////////////////////////////////////////////////////////////////////
	float ADIS16209::inclineScale(int16_t sensorData)
	{
		int signedData = 0;
		sensorData = sensorData & 0x3FFF; // Discard upper two bits
		int isNeg = sensorData & 0x2000;
		if (isNeg == 0x2000) // If the number is negative, scale and sign the output
			signedData = sensorData - 0x3FFF;
		else
			signedData = sensorData;
		float finalData = signedData * 0.025; // Multiply by (0.025 degrees/LSB)
		return finalData;
	}

	/////////////////////////////////////////////////////////////////////////////////////////////
	// Converts temperature data output from the regRead() function and returns temperature 
	// in degrees Celcius
	/////////////////////////////////////////////////////////////////////////////////////////////
	// sensorData - data output from regRead()
	// return - (float) signed/scaled temperature in degrees Celcius
	/////////////////////////////////////////////////////////////////////////////////////////
	float ADIS16209::tempScale(int16_t sensorData)
	{
		sensorData = sensorData & 0x0FFF; // Discard upper two bits
		float finalData = (((sensorData - 1278) * -0.47) + 25); // Multiply by temperature scale. 25C = 0x04FE
		return finalData;
	}

	/////////////////////////////////////////////////////////////////////////////////////////////
	// Converts voltage supply data output from the regRead() function and returns voltage 
	// in volts.
	/////////////////////////////////////////////////////////////////////////////////////////////
	// sensorData - data output from regRead()
	// return - (float) signed/scaled voltage in Volts
	/////////////////////////////////////////////////////////////////////////////////////////
	float ADIS16209::supplyScale(int16_t sensorData)
	{
		sensorData = sensorData & 0x3FFF; // Discard upper two bits
		float finalData = sensorData * 0.000305176; // Multiply by 0.000305176 Volts/LSB)
		return finalData;
	}
#endif

#endif /* HD44780_H_ */
