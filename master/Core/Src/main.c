/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "fatfs_sd.h"
#include "stdio.h"
#include "sys/time.h"
#include "string.h"
#include "stdint.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MASTER_BOARD
#define I2C_ADDRESS        2
#define I2C_ADDRESS2       4
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c1_tx;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
/* TEST Buffer used for transmission */
uint8_t aTxBuffer[] = " ****I2C_TwoBoards communication based on DMA****  ****I2C_TwoBoards communication based on DMA****  ****I2C_TwoBoards communication based on DMA****";

/* TEST Buffer used for reception */
uint8_t aRxBuffer[RXBUFFERSIZE];


FATFS fs;		//	file system
FIL fil;		// file
FILE *fp;
FRESULT fresult;	//	store result

char buffer[70000];
char destbuffer[70000];
UINT br, bw;	//	file read/write count
int hist[256] = {0};

/*	capacity related variables	*/
FATFS *pfs;
DWORD fre_clust;
uint32_t total, free_space;

//	Timer tracker
uint32_t IC_Value1 = 0;
uint32_t IC_Value2 = 0;
uint32_t IC_Value3 = 0;
uint32_t IC_Value4 = 0;

uint32_t IC_Value5 = 0;
uint32_t IC_Value6 = 0;
uint32_t IC_Value7 = 0;
uint32_t IC_Value8 = 0;

uint32_t singleNode1 = 0;
uint32_t singleNode2 = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
static uint16_t Buffercmp(uint8_t *pBuffer1, uint8_t *pBuffer2, uint16_t BufferLength);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//	THIS WRITE FUNCTION IS REQUIRED TO USE printf DEBUGGING TOOL USING SWV DATA CONSOLE
int _write(int file, char *ptr, int len)
{
  for(int i=0 ; i<len ; i++)
    ITM_SendChar((*ptr++));
  return len;
}


//	Find Size of Non-null Data in Buffer
//		- maybe replaceable with strlen function?
int bufsize(char *buf){
	int i=0;
	while(*buf++ != '\0'){
		i++;
	}
	return i;
}

//	Clear Buffer
void bufclear(char * buffer, int size) {
	for (int i = 0; i < size; i++) {
		buffer[i] = '\0';
	}
}

//	Unpack PACKBITS Algorithm to Decode Raw .TIFF File and Store into destcount
void *unpackbits(char *testbuf, char *testdestbuf, int destcount) {

  	int runlength;

  	while (destcount > 0) {
  		runlength = *testbuf++;
  		if (runlength == -128)
  			continue;
  		else if (runlength >= 128)
  		{
  			runlength = 256 - runlength + 1;
  			if (runlength > destcount)
  				return NULL;
  			memset(testdestbuf, *testbuf, runlength);
  			testbuf += 1;
  			testdestbuf += runlength;
  		}
  		else
  		{
  			++runlength;
  			if (runlength > destcount)
  				return NULL;
  			memcpy(testdestbuf, testbuf, runlength);
  			testbuf += runlength;
  			testdestbuf += runlength;
  		}
  		destcount -= runlength;
  	}

}

//	Equalization Algorithm
void histogramEqualization(char *inputbuffer, int bufferSize){


	int i;

	for(i = 0; i < bufferSize; i++){
		hist[(int)*inputbuffer++]++;
	}

	int newGrayHist[256] = {0};
	float idealPixelCount[256];
	for(i = 1; i < 256; i++){ //cumulative probability pixel counts

		newGrayHist[i] = newGrayHist[i-1] + hist[i];
	}

	for(i = 0; i < 256; i++){ //
		idealPixelCount[i] = (float)(newGrayHist[i]*255)/(float)(bufferSize-1);
	}

	for(i = 0; i < bufferSize; i++){
		buffer[i] = (int)idealPixelCount[destbuffer[i]];
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_SPI1_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */

  HAL_Delay(1000);	//	delay for SD card to process mounting

  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_4);	// Start input capture in interrupt mode


  //	Mount SD Card
  fresult = f_mount(&fs, "/", 1);
  if (fresult != FR_OK) //printf ("ERROR!!! in mounting SD CARD...\n\n");
	  printf("Error in mounting SD CARD...\n\n");
  else printf("SD CARD mounted successfully...\n\n");

  /*********************	Card Capacity Details	***********************/
  f_getfree("", &fre_clust, &pfs);

  total = (uint32_t)((pfs->n_fatent - 2) * pfs->csize * 0.5);
  sprintf(buffer, "SD CARD Total Size: \t%lu\n", total);
  printf(buffer);
  bufclear(buffer, sizeof(buffer));
  free_space = (uint32_t)(fre_clust * pfs->csize * 0.5);
  sprintf(buffer, "SD CARD Free Space: \t%lu\n", free_space);
  printf(buffer);
  bufclear(buffer, sizeof(buffer));

  bufclear(destbuffer, sizeof(destbuffer));

// @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ READ POUT.TIF START

  	//	TIMER START - Start Measuring
  	__HAL_TIM_SetCounter(&htim2, 0);
    singleNode1 = __HAL_TIM_GET_COUNTER(&htim2);


  fresult = f_open(&fil, "pout.tif", FA_READ);
  f_read (&fil, buffer, f_size(&fil), &br);
  f_gets(buffer, f_size(&fil), &fil);

	printf("File1.txt is opened and it contains the data as shown below\n");
	printf(buffer);
	printf("\n\n");

	/* Close file */
	f_close(&fil);

// @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@


	  // FIND OUT HOW TO GET PIXEL COUNT OR NUMBER OF PIXELS
	  int imgSize = 69841;

	  unpackbits(buffer + 8, destbuffer, imgSize);
	  bufclear(buffer, sizeof(buffer));
	  histogramEqualization(destbuffer, imgSize);

	  //	TIMER END - Measure time it takes to read from SD Card, and perform equalization on full image
	  singleNode2 = __HAL_TIM_GET_COUNTER(&htim2);





//	  // EXPORT EQUALIZED DATA TO A .TXT FILE SO IT CAN BE READ IN MATLAAB TO SHOW PROCESSED(EQUALIZED) IMAGE	- START

//	  fresult = f_open(&fil, "pout.txt", FA_WRITE | FA_OPEN_APPEND | FA_CREATE_ALWAYS);
//
//	  char s3[961];
//	  char s4[10];
//	  int byteSizeWrote = 0;
//
//
//	  int i = 0;
//	  int lineInc = 1;
//	  int newLineCount = 291;
//	  int totalPixelSize = 69840;
//	  int newLineFlag = 0;
//
//
//
//	  for(i = 0; i < totalPixelSize; i++){
//		  if((int)buffer[i] > 99){
//			  byteSizeWrote = byteSizeWrote+4;
//		  } else if ((int)buffer[i] > 9){
//			  byteSizeWrote = byteSizeWrote+3;
//		  } else {
//			  byteSizeWrote = byteSizeWrote+2;
//		  }
//
//		  if(i == 0 || byteSizeWrote == 0){ // if start
//			  sprintf(s3, "%d", (int)buffer[i]);
//			  sprintf(s4, "%s", ",");
//			  strcat(s3,s4);
//			  newLineFlag = 1;
//
//		  }else if(i == totalPixelSize){ //last line
//			  sprintf(s4, "%d", (int)buffer[i]);
//			  strcat(s3, s4);
//		  } else if(i == ((totalPixelSize/newLineCount)*lineInc)-1){ // if last line
//			  lineInc++;
//			  sprintf(s4, "%d", (int)buffer[i]);
//			  strcat(s3, s4);
//			  sprintf(s4, "%c", 13);
//			  strcat(s3, s4);
//			  sprintf(s4, "%c", 10);
//			  strcat(s3, s4);
//			  byteSizeWrote = byteSizeWrote + 1;
//			  fresult = f_write(&fil, (char*)s3, byteSizeWrote, &bw);
//			  if(fresult == FR_OK) {
//			  }
//
//			  byteSizeWrote = 0;
//			  bufclear(s3, sizeof(s3));
//			  newLineFlag = 0;
//
//		  } else{
//			  sprintf(s4, "%d", (int)buffer[i]);
//			  strcat(s3, s4);
//			  sprintf(s4, "%s", ",");
//			  strcat(s3, s4);
//		  }
//
//	  }
//	  // EXPORT EQUALIZED DATA TO A .TXT FILE SO IT CAN BE READ IN MATLAAB TO SHOW PROCESSED(EQUALIZED) IMAGE	- END





	/* Close file */
	f_close(&fil);
	//We're done, so de-mount the drive
	f_mount(NULL, "", 0);


	  // @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ EQUALIZATION DONE

#ifdef MASTER_BOARD

	//	WAIT UNTIL USER BUTTON (blue button) PRESS - start transmission to both nodes
  while (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) != 1)
  {
  }

  while (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) != 0)
  {
  }
  // @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ to p1
  //	TIMER START - TRANSMISSION
  __HAL_TIM_SetCounter(&htim2, 0);
    IC_Value1 = __HAL_TIM_GET_COUNTER(&htim2);

//    HAL_I2C_StateTypeDef temp = HAL_I2C_IsDeviceReady(&hi2c1, I2C_ADDRESS, 2, 10);

  while(HAL_I2C_Master_Transmit_DMA(&hi2c1, (uint16_t)I2C_ADDRESS, (uint8_t*)aTxBuffer, TXBUFFERSIZE)!= HAL_OK)
  {
    if (HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_AF)
    {
      Error_Handler();
    }
  }

  while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)
  {
  }
  //	TIMER END - TRANSMISSION
  IC_Value2 = __HAL_TIM_GET_COUNTER(&htim2);


  HAL_Delay(1000);

  // @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ to p2
  //	TIMER START - TRANSMISSION
  __HAL_TIM_SetCounter(&htim2, 0);
    IC_Value3 = __HAL_TIM_GET_COUNTER(&htim2);

//    HAL_I2C_StateTypeDef temp = HAL_I2C_IsDeviceReady(&hi2c1, I2C_ADDRESS, 2, 10);

  while(HAL_I2C_Master_Transmit_DMA(&hi2c1, (uint16_t)I2C_ADDRESS2, (uint8_t*)aTxBuffer, TXBUFFERSIZE)!= HAL_OK)
  {
    if (HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_AF)
    {
      Error_Handler();
    }
  }

  while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)
  {
  }
  //	TIMER END - TRANSMISSION
  IC_Value4 = __HAL_TIM_GET_COUNTER(&htim2);





	//	WAIT UNTIL USER BUTTON (blue button) PRESS - start receiving from both nodes
  while (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) != 1)
  {
  }
  while (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) != 0)
  {
  }
  // @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ from p1
  //	TIMER START - RECEPTION
  __HAL_TIM_SetCounter(&htim2, 0);
  IC_Value5 = __HAL_TIM_GET_COUNTER(&htim2);

  while(HAL_I2C_Master_Receive_DMA(&hi2c1, (uint16_t)I2C_ADDRESS, (uint8_t *)aRxBuffer, RXBUFFERSIZE) != HAL_OK)
  {
    if (HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_AF)
    {
      Error_Handler();
    }
  }

  while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)
  {
  }

  //	TIMER END - RECEPTION
  IC_Value6 = __HAL_TIM_GET_COUNTER(&htim2);

  HAL_Delay(1000);

  // @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ from p2
  //	TIMER START - RECEPTION
  __HAL_TIM_SetCounter(&htim2, 0);
  IC_Value7 = __HAL_TIM_GET_COUNTER(&htim2);

  while(HAL_I2C_Master_Receive_DMA(&hi2c1, (uint16_t)I2C_ADDRESS2, (uint8_t *)aRxBuffer, RXBUFFERSIZE) != HAL_OK)
  {
    if (HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_AF)
    {
      Error_Handler();
    }
  }

  while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)
  {
  }

  //	TIMER END - RECEPTION
  IC_Value8 = __HAL_TIM_GET_COUNTER(&htim2);

  /*##-6- Compare the sent and received buffers ##############################*/
  if(Buffercmp((uint8_t*)aTxBuffer,(uint8_t*)aRxBuffer,RXBUFFERSIZE))
  {
    /* Processing Error */
    Error_Handler();
  }

#else	// Slave node start

  /* The board receives the message and sends it back */

  /*##-2- Put I2C peripheral in reception process ############################*/
  if(HAL_I2C_Slave_Receive_DMA(&hi2c1, (uint8_t *)aRxBuffer, RXBUFFERSIZE) != HAL_OK)
  {
    /* Transfer error in reception process */
    Error_Handler();
  }

  /*##-3- Wait for the end of the transfer ###################################*/
  /*  Before starting a new communication transfer, you need to check the current
      state of the peripheral; if it�s busy you need to wait for the end of current
      transfer before starting a new one.
      For simplicity reasons, this example is just waiting till the end of the
      transfer, but application may perform other tasks while transfer operation
      is ongoing. */
  while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)
  {
  }

  /*##-4- Start the transmission process #####################################*/
  /* While the I2C in reception process, user can transmit data through
     "aTxBuffer" buffer */
  if(HAL_I2C_Slave_Transmit_DMA(&hi2c1, (uint8_t*)aTxBuffer, TXBUFFERSIZE)!= HAL_OK)
  {
    /* Transfer error in transmission process */
    Error_Handler();
  }

#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);	// TURN LED ON
//	  HAL_Delay(1000);							// Wait for 1 sec
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;	// standard speed mode 100kb/s
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;	// don't need slave add in master node
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;	// don't need slave add in master node
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_MDC_Pin RMII_RXD0_Pin RMII_RXD1_Pin */
  GPIO_InitStruct.Pin = RMII_MDC_Pin|RMII_RXD0_Pin|RMII_RXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_REF_CLK_Pin RMII_MDIO_Pin */
  GPIO_InitStruct.Pin = RMII_REF_CLK_Pin|RMII_MDIO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : RMII_TXD1_Pin */
  GPIO_InitStruct.Pin = RMII_TXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(RMII_TXD1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : STLK_RX_Pin STLK_TX_Pin */
  GPIO_InitStruct.Pin = STLK_RX_Pin|STLK_TX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_SOF_Pin USB_ID_Pin USB_DM_Pin USB_DP_Pin */
  GPIO_InitStruct.Pin = USB_SOF_Pin|USB_ID_Pin|USB_DM_Pin|USB_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_VBUS_Pin */
  GPIO_InitStruct.Pin = USB_VBUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_VBUS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_CS_Pin */
  GPIO_InitStruct.Pin = SD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SD_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_TX_EN_Pin RMII_TXD0_Pin */
  GPIO_InitStruct.Pin = RMII_TX_EN_Pin|RMII_TXD0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/**
  * @brief  Tx Transfer completed callback.
  * @param  I2cHandle: I2C handle
  * @note   This example shows a simple way to report end of DMA Tx transfer, and
  *         you can add your own implementation.
  * @retval None
  */
#ifdef MASTER_BOARD
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *I2cHandle)
{
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);	//	Toggle GREEN LED: Transfer in reception process is correct
}
#else
void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *I2cHandle)
{
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);	//	Toggle GREEN LED: Transfer in reception process is correct
}
#endif /* MASTER_BOARD */

/**
  * @brief  Rx Transfer completed callback.
  * @param  I2cHandle: I2C handle
  * @note   This example shows a simple way to report end of DMA Rx transfer, and
  *         you can add your own implementation.
  * @retval None
  */
#ifdef MASTER_BOARD
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *I2cHandle)
{
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);	//	Toggle BLUE LED: Transfer in reception process is correct
}
#else
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *I2cHandle)
{
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);	//	Toggle BLUE LED: Transfer in reception process is correct
}
#endif /* MASTER_BOARD */

/**
  * @brief  I2C error callbacks.
  * @param  I2cHandle: I2C handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
 void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *I2cHandle)
{
	 HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);	//	Turn RED LED on: Error in I2C handler
}

/**
  * @brief  Compares two buffers.
  * @param  pBuffer1, pBuffer2: buffers to be compared.
  * @param  BufferLength: buffer's length
  * @retval 0  : pBuffer1 identical to pBuffer2
  *         >0 : pBuffer1 differs from pBuffer2
  */
static uint16_t Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength)
{
  while (BufferLength--)
  {
    if ((*pBuffer1) != *pBuffer2)
    {
      return BufferLength;
    }
    pBuffer1++;
    pBuffer2++;
  }

  return 0;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);	//	Turn RED LED on: Transfer error in reception/transmission process
  while (1)
  {
  }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
