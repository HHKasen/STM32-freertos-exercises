/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "stdarg.h"
#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */



/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// HAL_GPIO_WritePin	(	GPIO_TypeDef * 	GPIOx,
//uint16_t 	GPIO_Pin,
//GPIO_PinState 	PinState
//)

#define SD_CS_PIN  GPIO_PIN_1
#define SD_CS_BANK GPIOB
#define SD_CS_LOW()   HAL_GPIO_WritePin	(SD_CS_BANK,SD_CS_PIN,GPIO_PIN_RESET)
#define SD_CS_HIGH()  HAL_GPIO_WritePin	(SD_CS_BANK,SD_CS_PIN,GPIO_PIN_SET)
#define N_CS 8
//CMD DEFS


typedef enum {
	CMD0 = 0, //GO_IDLE_STATE
	CMD1 = 1,//SEND_OP_CMD
	ACMD41 = 41, //APP_SEND_OP_CODE - SDC only - app cmd
	CMD8 = 8, //SEND_IF_COND
	CMD9 = 9, //SEND_CSD
	CMD10= 10, //SEND_CID
	CMD12 = 12, //STOP_TRANSMISSION
	CMD16 = 16, //SET_BLOCKLEN
	CMD17 = 17, //READ_SINGLE_BLOCK
	CMD18 = 18, //READ_MULTIPLE_BLOCKS
	CMD23 = 23, // SET_BLOCK_COUNT if appcmd first, becomes SET_WE_BLOCK_ERASE_CNT
	ACMD23 = 23|0x40, //SET_WE_BLOCK_ERASE_CNT - really 23, names like this to prevent conflict
	CMD24 = 24, //WRITE_BLOCK
	CMD25 = 25, //WRITE_MULTIPLE_BLOCK
	CMD55 = 55, //APP_CMD must precede any app cmd
	CMD58 = 58 //READ_OCR
}SD_cmd_t;





/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */



uint8_t CRCTable[256];
void GenerateCRCTable();
uint8_t CRCAdd(uint8_t CRC_val , uint8_t message_byte);
uint8_t getCRC(uint8_t message[], int length);
uint8_t fixedPointLog2(uint8_t val);


//Really should be error type
//Should also take read buffer? write buffer?
uint8_t send_SD_cmd(SD_cmd_t cmd, uint32_t cmd_arg){

	//should assert that resp!=NULL
	/*if( (cmd==ACMD41)||(cmd==ACMD23)){
		uint8_t acmd_resp = send_SD_cmd(CMD55,0);
		if(acmd_resp == 0xFF){
			return 0xFF;
		}
	}*/


	uint8_t tx_high = 0xFF;

	uint8_t MSG[35] = {'\0'};
	uint8_t spi_rx = 0xFF;
	uint8_t rec_res = 0;
	printf("cmd:%u\n",cmd);
    HAL_SPI_Transmit(&hspi2, &tx_high , 1, 50);
    HAL_SPI_Transmit(&hspi2, &tx_high , 1, 50);


	HAL_StatusTypeDef status;
	uint8_t spi_tx_bf[6] = {0};

	spi_tx_bf[0] = (0x40)| cmd;

	spi_tx_bf[4] =  cmd_arg & 0x000000FF;
	spi_tx_bf[3] = (cmd_arg & 0x0000FF00) >> 8;
	spi_tx_bf[2] = (cmd_arg & 0x00FF0000) >> 16;
	spi_tx_bf[1] = (cmd_arg & 0xFF000000) >> 24;

	spi_tx_bf[5] = (getCRC(spi_tx_bf,5)<<1)  + 1;

	printf(MSG, "cmd:%u %u %u %u %u %u\n",spi_tx_bf[0],spi_tx_bf[1],spi_tx_bf[2],spi_tx_bf[3],spi_tx_bf[4],spi_tx_bf[5]);

    HAL_SPI_Transmit(&hspi2, spi_tx_bf , 6, 50);

	int count = 0;


	while( count<N_CS && !rec_res  ){
		HAL_SPI_TransmitReceive(&hspi2, &tx_high, &spi_rx , 1, 50);
		if( (spi_rx&0x80) == 0){
			rec_res = 1;
		}
		printf("resp:%u\n",spi_rx);

		count++;
	}
	//will this work?


	if(cmd!=CMD58 && cmd!=CMD8 && spi_rx!=0xFF){
		uint8_t spi_rx_clear = 0x0;

		for(int ii=0; ii<8; ii++){
			HAL_SPI_TransmitReceive(&hspi2, &tx_high, &spi_rx_clear , 1, 50);
			printf("clr return:%u\n",spi_rx_clear);
			if(spi_rx_clear==0xFF){
				break;
			}
		}
	}

	return spi_rx;
}

void get_OCR(uint8_t* ocr){
	    uint8_t tx_high = 0xFF;
		uint8_t rx_buff[4] = {0};
		HAL_SPI_TransmitReceive(&hspi2, &tx_high, ocr, 1, 50);
		HAL_SPI_TransmitReceive(&hspi2, &tx_high, ocr+1, 1, 50);
		HAL_SPI_TransmitReceive(&hspi2, &tx_high, ocr+2, 1, 50);
		HAL_SPI_TransmitReceive(&hspi2, &tx_high, ocr+3, 1, 50);


		//sprintf(MSG, "trail:%u %u %u %u\n",ocr[0],ocr[1],ocr[2],ocr[3]);
		//HAL_UART_Transmit(&huart2, MSG, sizeof(MSG), 100);
		printf("trail:%u %u %u %u\n",ocr[0],ocr[1],ocr[2],ocr[3]);
		uint8_t spi_rx_clear = 0x0;
		while(spi_rx_clear!=0xFF){
			HAL_SPI_TransmitReceive(&hspi2, &tx_high, &spi_rx_clear , 1, 50);
		}
}


	//spare cpi cycles
	//HAL_SPI_Transmit(&hspi2, &tx_high , 1, 0);

	//SD_CS_HIGH();
	//HAL_SPI_Transmit(&hspi2, &tx_high , 1, 0);



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
  MX_USART2_UART_Init();
  MX_SPI2_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */

  GenerateCRCTable();

  printf("\n-------------\n-------------\n  Starting...\n-------------\n\n-------------\n\n\n");

  uint8_t spi_tx = 0xFF;
  uint8_t spi_rx = 0x00;


    //init seq
	for(int ii = 0; ii<2; ii++){
		HAL_SPI_Transmit(&hspi2, &spi_tx , 1, 0);
	}
	SD_CS_HIGH();

	for(int ii = 0; ii<2; ii++){
		HAL_SPI_Transmit(&hspi2, &spi_tx , 1, 0);
	}
	HAL_Delay(50); //delay at least 1 ms

	  //only need 72, do a bunch more
	for(int ii = 0; ii<10; ii++){
		HAL_SPI_Transmit(&hspi2, &spi_tx , 1, 0);
	}

	uint8_t R1_resp = 0;
	uint8_t ocr[4] = {0};

	for(int ii = 0; ii<2; ii++){
		HAL_SPI_Transmit(&hspi2, &spi_tx , 1, 0);
	}
	SD_CS_LOW();
	for(int ii = 0; ii<2; ii++){
		HAL_SPI_Transmit(&hspi2, &spi_tx , 1, 0);
	}

	/*
	R1_resp = send_SD_cmd(CMD58,0x00000000);
    get_OCR(ocr);
	R1_resp = send_SD_cmd(CMD58,0x00000000);
    get_OCR(ocr);
	R1_resp = send_SD_cmd(CMD58,0x00000000);

	get_OCR(ocr);
    printf("OCR status:\n");
    printf("\tCCS(0=bytes,1=blocks):%u\n", (ocr[3]>>6)&0x01 );
    if(ocr[2]==0x00){
    	printf("Error: No voltage range specified\n");
    }
    else{
    	uint8_t min_voltage = fixedPointLog2(ocr[2] & (~ocr[2] + 1));
    	uint8_t max_voltage = fixedPointLog2(ocr[2]);
    	printf("\tmin,max:%u %u\n",min_voltage,max_voltage);

    }
	*/


	R1_resp = send_SD_cmd(CMD0,0);

	uint8_t SD_valid = 0;

	if(R1_resp == 0x01){

    	for(int ii = 0; ii<2; ii++){
    		HAL_SPI_Transmit(&hspi2, &spi_tx , 1, 0);
    	}
    	SD_CS_HIGH();
    	for(int ii = 0; ii<2; ii++){
    		HAL_SPI_Transmit(&hspi2, &spi_tx , 1, 0);
    	}
    	for(int ii = 0; ii<2; ii++){
    		HAL_SPI_Transmit(&hspi2, &spi_tx , 1, 0);
    	}
    	SD_CS_LOW();
    	for(int ii = 0; ii<2; ii++){
    		HAL_SPI_Transmit(&hspi2, &spi_tx , 1, 0);
    	}
		R1_resp = send_SD_cmd(CMD8,0x01AA);
	    get_OCR(ocr);


	    if( (R1_resp!=0x01)){ //if error or no response, SD1 or MMC
	    	printf("SD 1.0 or MMC\n");

	    }
	    else if( (ocr[2] == 0x01)&&(ocr[3] == 0xAA)  ){ //SD v2
	    	printf("SD 2.0+\n");

	    	for(int ii = 0; ii<2; ii++){
	    		HAL_SPI_Transmit(&hspi2, &spi_tx , 1, 0);
	    	}
	    	SD_CS_HIGH();
	    	for(int ii = 0; ii<2; ii++){
	    		HAL_SPI_Transmit(&hspi2, &spi_tx , 1, 0);
	    	}
	    	for(int ii = 0; ii<2; ii++){
	    		HAL_SPI_Transmit(&hspi2, &spi_tx , 1, 0);
	    	}
	    	SD_CS_LOW();
	    	for(int ii = 0; ii<2; ii++){
	    		HAL_SPI_Transmit(&hspi2, &spi_tx , 1, 0);
	    	}


	    	R1_resp = send_SD_cmd(CMD58,0x00000000);
	    	get_OCR(ocr);
	    	printf("Voltage supported:%u\n",ocr[1]&0x38==0x38);


	    	for(int ii = 0; ii<2; ii++){
	    		HAL_SPI_Transmit(&hspi2, &spi_tx , 1, 0);
	    	}
	    	SD_CS_HIGH();
	    	for(int ii = 0; ii<2; ii++){
	    		HAL_SPI_Transmit(&hspi2, &spi_tx , 1, 0);
	    	}
	    	for(int ii = 0; ii<2; ii++){
	    		HAL_SPI_Transmit(&hspi2, &spi_tx , 1, 0);
	    	}
	    	SD_CS_LOW();
	    	for(int ii = 0; ii<2; ii++){
	    		HAL_SPI_Transmit(&hspi2, &spi_tx , 1, 0);
	    	}




	    	for(int ii=0; ii<2000; ii++){ //replace this with a proper timed loop

		    	for(int ii = 0; ii<2; ii++){
		    		HAL_SPI_Transmit(&hspi2, &spi_tx , 1, 0);
		    	}
		    	SD_CS_HIGH();
		    	for(int ii = 0; ii<2; ii++){
		    		HAL_SPI_Transmit(&hspi2, &spi_tx , 1, 0);
		    	}
		    	for(int ii = 0; ii<2; ii++){
		    		HAL_SPI_Transmit(&hspi2, &spi_tx , 1, 0);
		    	}
		    	SD_CS_LOW();
		    	for(int ii = 0; ii<2; ii++){
		    		HAL_SPI_Transmit(&hspi2, &spi_tx , 1, 0);
		    	}


//	    		R1_resp = send_SD_cmd(ACMD41,0x40000000);
	    		R1_resp = send_SD_cmd(CMD55,0x40FF8000);

		    	for(int ii = 0; ii<2; ii++){
		    		HAL_SPI_Transmit(&hspi2, &spi_tx , 1, 0);
		    	}
		    	SD_CS_HIGH();
		    	for(int ii = 0; ii<2; ii++){
		    		HAL_SPI_Transmit(&hspi2, &spi_tx , 1, 0);
		    	}
		    	for(int ii = 0; ii<2; ii++){
		    		HAL_SPI_Transmit(&hspi2, &spi_tx , 1, 0);
		    	}
		    	SD_CS_LOW();
		    	for(int ii = 0; ii<2; ii++){
		    		HAL_SPI_Transmit(&hspi2, &spi_tx , 1, 0);
		    	}


//	    		R1_resp = send_SD_cmd(ACMD41,0x40000000);
	    		R1_resp = send_SD_cmd(ACMD41,0x40FF8000);


	    		printf("returned\n");
	    		if(R1_resp==0x00){
	    			break;

	    		}
	    	}


	    	for(int ii = 0; ii<2; ii++){
	    		HAL_SPI_Transmit(&hspi2, &spi_tx , 1, 0);
	    	}
	    	SD_CS_HIGH();
	    	for(int ii = 0; ii<2; ii++){
	    		HAL_SPI_Transmit(&hspi2, &spi_tx , 1, 0);
	    	}
	    	for(int ii = 0; ii<2; ii++){
	    		HAL_SPI_Transmit(&hspi2, &spi_tx , 1, 0);
	    	}
	    	SD_CS_LOW();
	    	for(int ii = 0; ii<2; ii++){
	    		HAL_SPI_Transmit(&hspi2, &spi_tx , 1, 0);
	    	}


	    	R1_resp = send_SD_cmd(CMD58,0x00000000);
	    	get_OCR(ocr);

    	    printf("OCR status:\n");
    	    printf("\tCCS(0=bytes,1=blocks):%u\n", (ocr[3]>>6)&0x01 );
    	    if(ocr[2]==0x00){
    	    	printf("Error: No voltage range specified\n");
    	    }

	    	R1_resp = send_SD_cmd(CMD58,0x00000000);
	    	get_OCR(ocr);

    	    printf("OCR status:\n");
    	    printf("\tCCS(0=bytes,1=blocks):%u\n", (ocr[3]>>6)&0x01 );
    	    if(ocr[2]==0x00){
    	    	printf("Error: No voltage range specified\n");
    	    }

    	    else{
    	    	uint8_t min_voltage = fixedPointLog2(ocr[2] & (~ocr[2] + 1));
    	    	uint8_t max_voltage = fixedPointLog2(ocr[2]);
    	    	printf("\tmin,max:%u %u\n",min_voltage,max_voltage);

    	    }
	    }
	    else{
			printf("error, CMD8 response:%u\n",R1_resp);

	    }
	    //else,  simply error
	}
	else{
		printf("error, CMD0 response:%u\n",R1_resp);
	}

	R1_resp = send_SD_cmd(CMD8,0x01AA);
    get_OCR(ocr);


	for(int ii = 0; ii<2; ii++){
		HAL_SPI_Transmit(&hspi2, &spi_tx , 1, 0);
	}
	SD_CS_HIGH();
	for(int ii = 0; ii<2; ii++){
		HAL_SPI_Transmit(&hspi2, &spi_tx , 1, 0);
	}
//	sprintf(MSG, "resp:%u\n",cmd_resp[0]);
//	HAL_UART_Transmit(&huart2, MSG, sizeof(MSG), 100);



  //send_SD_cmd(SEND_IF_COND,0x1AA);

  //
  //
  //


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  uint8_t parity = 0;
  spi_tx = 0xFF;
  while (1)
  {
//	  HAL_SPI_TransmitReceive(&hspi2, &spi_tx , &spi_rx, 1, 0);
//	  sprintf(MSG, "Received:%u\n",spi_rx);
//	  HAL_UART_Transmit(&huart2, MSG, sizeof(MSG), 100);


	  //HAL_GPIO_TogglePin(GPIOB, SD_CS);

	  HAL_Delay(5000);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_CS_Pin */
  GPIO_InitStruct.Pin = SD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SD_CS_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART1 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
}

void GenerateCRCTable()
{
  int i, j;
  uint8_t CRCPoly = 0x89;  // the value of our CRC-7 polynomial

  // generate a table value for all 256 possible byte values
  for (i = 0; i < 256; ++i) {
    CRCTable[i] = (i & 0x80) ? i ^ CRCPoly : i;
    for (j = 1; j < 8; ++j) {
        CRCTable[i] <<= 1;
        if (CRCTable[i] & 0x80){
            CRCTable[i] ^= CRCPoly;
        }
    }
  }
}

/*
// adds a message byte to the current CRC-7 to get a the new CRC-7
uint8_t CRCAdd(uint8_t CRC, uint8_t message_byte)
{
	return 0;
 //   return CRCTable[(CRC << 1) ^ message_byte];
}*/
uint8_t CRCAdd(uint8_t CRC_val , uint8_t message_byte){
	 return CRCTable[(CRC_val << 1) ^ message_byte];
}


// returns the CRC-7 for a message of "length" bytes
uint8_t getCRC(uint8_t message[], int length)
{

  int i;
  uint8_t CRC_val = 0;

  for (i = 0; i < length; ++i){
	  CRC_val = CRCAdd(CRC_val, message[i]);
  }

  return CRC_val;

}

uint8_t fixedPointLog2(uint8_t val){
	uint8_t r = 0;
	while (val >>= 1) // unroll for more speed...
	{
	  r++;
	}
	return r;
}



/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
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

