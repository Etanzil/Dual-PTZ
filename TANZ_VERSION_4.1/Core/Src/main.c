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

//flags
#define NO      		0
#define YES					1

//Commands
#define UP					10
#define DOWN    		11
#define RIGHT   		12
#define LEFT    		13
#define UP_RIGHT  	14
#define UP_LEFT   	15
#define DOWN_RIGHT  16
#define DOWN_LEFT 	17

#define ZOOM_IN  		18
#define ZOOM_OUT  	19
#define STOP_MOTORS 20
#define DOWM_L_SW   21
#define UP_L_SW     22
#define GO_TO     	23
#define SCAN				24
#define SET_LEFT		25
#define SET_RIGHT		26
#define TOUR				27
#define PAN				  28

#define EEPROM_BASE_ADDR	0x08080000		// EEPROM Address
#define EEPROM_BYTE_SIZE	0x0FFF		

//#define   PAGE                  0
//#define   SECTOR                1

#define   _EE_SIZE              512				// EEPROM size
#define   _EE_ADDR_INUSE        0x08080000
#define   _EE_FLASH_BANK        FLASH_BANK_1
#define   _EE_PAGE_OR_SECTOR    PAGE

#define   _EE_USE_RAM_BYTE                          (512)					// size of EEPROM of STM32l011k4
#if (_EE_USE_RAM_BYTE > 0)
uint8_t ee_ram[_EE_USE_RAM_BYTE];
#endif


#define DATA_E2_ADDR 0x08080000				//initial address of EEPROM start
uint32_t addr = 0;				// virtual address

#define EN_INT      	__enable_irq();		// System open global interrupt
#define DIS_INT     	__disable_irq();	// System off global interrupt

uint8_t rx_data;
uint8_t rx_buffer[7];
uint8_t rx_index = 0;
uint8_t info = 0;

uint8_t header = 0;
uint8_t compId1 = 0;
uint8_t compId2 = 0;
uint8_t compId3 = 0;
uint8_t compId4 = 0;
uint8_t compId5 = 0;
uint8_t compId6 = 0;

int16_t counter_limit[4];	// 	limit array for scan feature
int16_t counter_main[32][2];	// preset array for cam1 
int16_t counter_main_1[32][2];	// preset array for cam2 
uint8_t cam_sel = 0x00;	// detection for cam if 0x01 -> cam1 && 0x02 -> cam2

int16_t xmove = 0; 
int16_t ymove = 0; 


UART_HandleTypeDef hlpuart1;
UART_HandleTypeDef huart2;

TIM_HandleTypeDef htim2;

void SEND_TO_NEXTION(uint8_t string[]);
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
/*
	For EEPROM writing
		the sequence must be followed
			-> unlocking_eeprom();
			-> update_preset
			-> locking_eeprom();
*/

void unlocking_eeprom();	// unlocking the EEPROM
void locking_eeprom();	// locking the eeprom
void unlock_nvm_prog_mem();	
void unlocking_byte_area();
void erase_to_data_eeprom();	
void write_eeprom(int16_t data, uint32_t virtual_address);	
int16_t read_eeprom(int virtual_address);	// to read the byte from eeprom
void get_presets(void);	// it will update the presets arrays for both cams
void get_limits(void);	// it will update the limit array for both cams
void update_preset(int16_t x, int16_t y, int virtual_address, uint8_t sel);	// every preset has two values x and y, these are the parameters and virtual address is for preset number and sel is for cam selection either cam1 or cam2
void update_limit(int16_t x, uint8_t sel, char limit); //it will save limits of scan in EEPROM after preset memory
void reset_presets(void);	// to delete all the presets from eeprom
void setSpeed(int whatSpeed);	// to set the speed of stepper motor
void stepMotorNew(int thisStep);//edited by Engr Aftab
void Ramp(int rspeed, int xdir, int ydir);
void steps(int num_steps); // the motor will move num_steps  at a time 
void stepsNew(int num_steps); //edited by Engr Aftab
void stepsNew_d(int num_steps_p,int num_steps_t);	//Function for diagonal movement (Edited by Engr Tanzeel)
void PERFORM_TASK(void);	// to perform the motor movement
void detectCommand(void);	// to detect which command is received for motor movement
void selfTest(void);	// it will move the cam to its home position AND total tilt steps are calculated in the self-test 
void call_operation(int sel, uint8_t preset);	// it will drive the cam to specific preset ,as every cam has different presets so sel is for cam selection and preset is for preset number
void Scan(uint8_t c_sel);			//it will move the cam whithin set limits when Scan is started
void Pan(void);			//it will keep rotating the cam 360* horizontally
int16_t Abs(int16_t val);	// absolute function -> if val = -1 then it will return +1

int direction = 0;			  // Direction of rotation
unsigned long step_delay; // delay between steps, in ms, based on speed
int number_of_steps = 2800;		// total number of steps this motor can take 
int step_number = 0;		  // which step the motor is on
int step_number_d = 0;		  // which step the pan motor is on in diagonal movement
int t_limit_count =0;	//edited by Engr Tanzeel
unsigned long last_step_time = 0; // time stamp in us of when the last step was taken

int steps_to_move;
int stepss = 100; //Edited by Engr Aftab
int16_t x_total =2000; //for xorogn for 360 rotation


int16_t xorgn = 0;	// number of steps moved in horizontal axis wrt origin // XORGN VALUE IS +VE WHEN CAM IS AT Right SIDE.
int16_t yorgn = 0;	// number of steps moved in vertical axis wrt origin //YORGN VALUE IS +VE WHEN CAM IS AT DOWN SIDE.
uint8_t patrolling_flag = NO;	// if patrolling is enabled(1) or not (0)
uint8_t tour_flag = NO;
uint8_t rec_cmd = STOP_MOTORS;	// command detection

int main(void)
{

	HAL_GPIO_WritePin(enab_tilt_GPIO_Port,enab_tilt_Pin, 1); 
	HAL_GPIO_WritePin(enab_pan_GPIO_Port,enab_pan_Pin,1);
  HAL_Init();


  SystemClock_Config();


  MX_GPIO_Init();
  MX_LPUART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();

  for (int r = 0; r < 32; r++)	//clearing all preset arrays
  {
    counter_main[r][0] = 0;     // [x, y, locate]
    counter_main[r][1] = 0;
		counter_main_1[r][0] = 0;     // [x, y, locate]
    counter_main_1[r][1] = 0;
  }
	for (int t = 0; t < 4; t++)	//clearing limit array
  {
    counter_limit[t] = 0;     // [x locate]
  }
	
	
  get_presets();	// adding presets to presets arrays from eeprom
	get_limits();		// adding limits to limit arrays from eeprom
	//reset_presets();	// clearing whole eeprom ... uncomment it for clearing then it must be commented for next uploading
// as writing in eeprom is limited upto 100000 if the cycle reached 100000 then eeprom will be crashed
	HAL_UART_Receive_IT(&hlpuart1, &rx_data, 1);
  HAL_UART_Receive_IT(&huart2, &rx_data, 1);
	selfTest();

  while (1)
  {
		if (t_limit_count == 5) selfTest();	//will perform self test if motor stalling ever happens
		else PERFORM_TASK();		// will perform task according to the last detected command
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_LPUART1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 9600;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

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
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 32;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, in1_zoom_Pin|in2_zoom_Pin|en_zoom_Pin|in1_Pin|dir_tilt_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, dir_pan_Pin|debug_Pin|enab_tilt_Pin|enab_pan_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : in1_zoom_Pin in2_zoom_Pin en_zoom_Pin in1_Pin dir_tilt_Pin*/
  GPIO_InitStruct.Pin = in1_zoom_Pin|in2_zoom_Pin|en_zoom_Pin|in1_Pin|dir_tilt_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : dir_pan_Pin debug_Pin enab_tilt_Pin enab_pan_Pin */
  GPIO_InitStruct.Pin = dir_pan_Pin|debug_Pin|enab_tilt_Pin|enab_pan_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : t1_sw_Pin */
  GPIO_InitStruct.Pin = t1_sw_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(t1_sw_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : t2_sw_Pin p_sw_Pin */
  GPIO_InitStruct.Pin = t2_sw_Pin|p_sw_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}


/* USER CODE BEGIN 4 */
void SEND_TO_NEXTION(uint8_t string[])
{
	/*
		the function is used for debugging purposes
	*/
	uint8_t end=0xFF;
	while(*string)
	HAL_UART_Transmit(&huart2,string++, 1, 200);
	HAL_UART_Transmit(&huart2,&end, 1, 200);
	HAL_UART_Transmit(&huart2,&end, 1, 200);
	HAL_UART_Transmit(&huart2,&end, 1, 200);
}
void detectCommand(void)	{
	/*
		the function is used to detect the command received in HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) function
		a command is of 7 bytes
		header is 0xFF
	*/
	HAL_GPIO_WritePin(debug_GPIO_Port, debug_Pin, 1);	

		if((compId1 == 0x01 || compId1 == 0x02) && (compId3 == 0x08) )	// if compId3 == 0x08 it is in upward direction
		{
			patrolling_flag = NO;
			tour_flag = NO;
			rec_cmd = UP;        //UP
			//setSpeed(compId5 + 0x14);	// compId5 is the speed of tilt motor
			HAL_GPIO_WritePin(debug_GPIO_Port, debug_Pin, 0);
		}
		
		else if((compId1 == 0x01 || compId1 == 0x02) && compId3 == 0x02 ) // if compId3 == 0x02 it is in right direction
		{
			patrolling_flag = NO;
			tour_flag = NO;
			rec_cmd = RIGHT;				//RIGHT				//Edited by Egnr Tanzeel
			HAL_GPIO_WritePin(debug_GPIO_Port, debug_Pin, 0);
		}
		
		else if((compId1 == 0x01 || compId1 == 0x02) && (compId3 == 0x10) ) // if compId3 == 0x10 it is in downward direction
		{
			patrolling_flag = NO;
			tour_flag = NO;
			rec_cmd = DOWN;		//DOWN
			HAL_GPIO_WritePin(debug_GPIO_Port, debug_Pin, 0);
		}
		
		else if((compId1 == 0x01 || compId1 == 0x02) && compId3 == 0x04) // if compId3 == 0x04 it is in left direction
		{
			patrolling_flag = NO;
			rec_cmd = LEFT;				//LEFT			//Edited by Egnr Tanzeel
			HAL_GPIO_WritePin(debug_GPIO_Port, debug_Pin, 0);
		}
			else if((compId1 == 0x01 || compId1 == 0x02) && compId3 == 0x0A )	// if compId3 == 0x0A it is in UpRight direction
		{
			patrolling_flag = NO;
			tour_flag = NO;
			rec_cmd = UP_RIGHT;
			HAL_GPIO_WritePin(debug_GPIO_Port, debug_Pin, 0);
		}
			else if((compId1 == 0x01 || compId1 == 0x02) && compId3 == 0x0C )	// if compId3 == 0x0C it is in UpLeft direction
		{
			patrolling_flag = NO;
			tour_flag = NO;
			rec_cmd = UP_LEFT;
			HAL_GPIO_WritePin(debug_GPIO_Port, debug_Pin, 0);
		}
			else if((compId1 == 0x01 || compId1 == 0x02) && compId3 == 0x12 )	// if compId3 == 0x12 it is in DownRight direction
		{
			patrolling_flag = NO;
			tour_flag = NO;
			rec_cmd = DOWN_RIGHT;
			HAL_GPIO_WritePin(debug_GPIO_Port, debug_Pin, 0);
		}
			else if((compId1 == 0x01 || compId1 == 0x02) && compId3 == 0x14 )	// if compId3 == 0x14 it is in LeftDown direction
		{
			patrolling_flag = NO;
			tour_flag = NO;
			rec_cmd = DOWN_LEFT;
			HAL_GPIO_WritePin(debug_GPIO_Port, debug_Pin, 0);
		}
//		else if((compId1 == 0x01 || compId1 == 0x02) && compId3 == 0x0A)
//		{
//			patrolling_flag = 0;
//			locate = 0;
//			setSpeed(40);
//		}
		    //check for focus+ command is FF 01 01 00 00 00 02
    else if ((compId1 == 0x01) && (compId2 == 0x01)) // for thermal cam1 
    {
			rec_cmd = ZOOM_IN; 
			HAL_GPIO_WritePin(debug_GPIO_Port, debug_Pin, 0);
		}

    //check for focus- command is FF 01 00 80 00 00 81
    else if ((compId1 == 0x01)  && (compId3 == 0x80)) // focus out of thermal
    {
				
			rec_cmd = ZOOM_OUT;
			HAL_GPIO_WritePin(debug_GPIO_Port, debug_Pin, 0);
    }
		else if((compId1 == 0x01 || compId1 == 0x02) && (compId2 == 0x00 ||compId2 == 0x20) && (compId3 == 0x00 ||compId3 == 0x29) && compId4 == 0x00 && compId5 == 0x00 && (compId6 == 0x01 || compId1 == 0x02 || compId6 == 0x4A)) // stop command
		{
			patrolling_flag = NO;
			tour_flag = NO;
			rec_cmd = STOP_MOTORS;
			HAL_GPIO_WritePin(debug_GPIO_Port, debug_Pin, 0);
		}
		
		else if((compId1 == 0x01 || compId1 == 0x02) && compId3 == 0x03){			// for adding preset 
			patrolling_flag = 0;
			if(compId5>0 && compId5 <= 32){	// compId5 must be in limit 0~32 ---- compId5 is our preset number
				if (cam_sel == 1){	//for cam1
					counter_main[compId5-1][0] = xorgn;				//xorgn is the current position in X. -1 is being done because GUI will send 1-32 and we'll convert it in 0-31
					counter_main[compId5-1][1] = yorgn;
				}
				else if (cam_sel == 2){	//forcam2
					counter_main_1[compId5-1][0] = xorgn;
					counter_main_1[compId5-1][1] = yorgn;
				}
				update_preset(xorgn, yorgn, compId5-1, cam_sel);	// updating the presets in eeprom
				HAL_GPIO_WritePin(debug_GPIO_Port, debug_Pin, 0);
			}
		}
		
		else if((compId1 == 0x01|| compId1 == 0x02) && compId3 == 0x05){	// deleting the presets
			patrolling_flag = 0;
			if(compId5>0 && compId5<=32){	// compId5 must be in limit 0~32	---- compId5 is our preset number
				if (cam_sel == 1){	//for cam1
					counter_main[compId5-1][0] = 0;
					counter_main[compId5-1][1] = 0;
				}
				else if (cam_sel == 2){	//forcam2
					counter_main_1[compId5-1][0] = 0;
					counter_main_1[compId5-1][1] = 0;
				}
				update_preset(0,0,compId5-1,cam_sel);
				HAL_GPIO_WritePin(debug_GPIO_Port, debug_Pin, 0);
			}
		}
		
		else if ((compId1 == 0x01 || compId1 == 0x02) && compId3 == 0x07){	// calling the specific preset (Goto)
			if(tour_flag == YES)
			{
			rec_cmd = GO_TO;
			patrolling_flag = NO;
			HAL_GPIO_WritePin(debug_GPIO_Port, debug_Pin, 0);
			}
		}
					
		else if ((compId1 == 0x01 || compId1 == 0x02) && (compId3 == 0x1B || compId3 == 0x0D)){					//start Scan
			rec_cmd = SCAN;		//Edited by Engr Tanzeel
			HAL_GPIO_WritePin(debug_GPIO_Port, debug_Pin, 0);
		}
		else if( (compId1 == 0x01|| compId1 == 0x02) && (compId3 == 0x1D)){				// stop the Scan
			patrolling_flag = NO;
			rec_cmd = STOP_MOTORS;
			HAL_GPIO_WritePin(debug_GPIO_Port, debug_Pin, 0);
		}
		
		else if( (compId1 == 0x01|| compId1 == 0x02) && (compId3 == 0x09 || compId3 == 0x0B || compId3 == 0x11 || compId3 == 0x13)) // set limits in scan		
			{				
			patrolling_flag = NO;
			if (compId5 == 0x01 && compId3 == 0x09){	//for cam1 left limit
				counter_limit[0] = xorgn;
				update_limit(xorgn, 1, 'l');
				}
			else if (compId5 == 0x00 && compId3 == 0x11){	//for cam2 left limit
				counter_limit[2] = xorgn;
				update_limit(xorgn, 2, 'l');
				}
			else if (compId5 == 0x01 && compId3 == 0x0B){	//for cam1 right limit
				counter_limit[1] = xorgn;
				update_limit(xorgn, 1, 'r');
				}
			else if (compId5 == 0x00 && compId3 == 0x13){	//for cam2 right limt
				counter_limit[3] = xorgn;
				update_limit(xorgn, 2, 'r');
				}
			HAL_GPIO_WritePin(debug_GPIO_Port, debug_Pin, 0);
			}
			else if ((compId1 == 0x01 || compId1 == 0x02) && (compId2 == 0x90 || compId6 == 0x4B)){					//start Pan
			rec_cmd = PAN;		//Edited by Engr Tanzeel
			HAL_GPIO_WritePin(debug_GPIO_Port, debug_Pin, 0);
			}
}
int16_t Abs(int16_t val){
	if (val > 0)
		return val;
	else
		return (val * (-1));
}

void clearBuffer(void)
{
	for(int j = 0; j < 7; j++)
		rx_buffer[j] = 0x00;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{		
	{

	if(huart->Instance==LPUART1)
		{
		
		//Day
		HAL_UART_Receive_IT(&hlpuart1,&rx_data,1);		// data receive on LPUART 
		if(rx_data==0xFF) info=1;	// if we get 0xFF then we will receive next 6 bytes, FF is header
		if(info==1)
			{rx_buffer[rx_index]=rx_data;	//received data in array 
			rx_index++;}	

		if(rx_index>=7)	// if 7 bytes received
			{
			header = rx_buffer[0];
			compId1 = rx_buffer[1];
			compId2 = rx_buffer[2];
			compId3 = rx_buffer[3];
			compId4 = rx_buffer[4];
			compId5 = rx_buffer[5];
			compId6 = rx_buffer[6];
			rx_index=0;
			info=0;
			if(header==0xFF)
				{
					cam_sel = compId1;		// compId1 is address line which is camera selection also
					detectCommand();		// function to detect command and check if cmd is true or not
				}
			}
		//if(rx_index>=7) 
		if(info==0)
			rx_index=0;

		}
		else if(huart->Instance==USART2)
		{
		
		//NIGHT
		HAL_UART_Receive_IT(&huart2,&rx_data,1); // data receive on USART2 
		if(rx_data==0xFF) info=1; // if we get 0xFF then we will receive next 6 bytes
		if(info==1)
			{rx_buffer[rx_index]=rx_data; //received data in array 
			rx_index++;}

		if(rx_index>=7) // if 7 bytes received
			{
			header = rx_buffer[0];
			compId1 = rx_buffer[1];
			compId2 = rx_buffer[2];
			compId3 = rx_buffer[3];
			compId4 = rx_buffer[4];
			compId5 = rx_buffer[5];
			compId6 = rx_buffer[6];
			rx_index=0;
			info=0;
			if(header==0xFF)//&&(compId3!=0X00))
				{
					cam_sel = compId1; // compId1 is address line which is camera selection also
					detectCommand();	// function to detect command and check if cmd is true or not
				}
			
			}
		//if(rx_index>=7) 
		if(info==0)
			rx_index=0;

		}
	}
}
void setSpeed(int whatSpeed){					//SETS THE SPEED IN REVS PER MINUTE
	step_delay = 60L * 1000L * 1000L/ number_of_steps / whatSpeed; // equation for speed calculation... do not change
}

void stepMotorNew(int thisStep){					//stepper motor library. do not edit this function //Edited by Engr Aftab
    switch (thisStep) {
      case 0:  // 1010
				HAL_GPIO_WritePin(in1_GPIO_Port, in1_Pin, 1);				
      break;
      case 1:  // 0110
				HAL_GPIO_WritePin(in1_GPIO_Port, in1_Pin, 0);					//ENABLE CLK PIN OF TB6065
      break; 
    }
  }
void Ramp(int rspeed, int xdir, int ydir){					//Ramp function for goto feature, ramp the speed depending upon the distance to be covered //Edited by Engr Tanzeel
int w=0,e=0;
			for(int q=1; w<rspeed; q++)
			{
				w=q*30;										
				setSpeed(w);						//speed will be incremented by 30 to avoid motor stalling.
				stepsNew_d(xdir,ydir); 	// move 100 steps
				e=q;
			}
//////increment or decrement xorgn&yorgn depending upon  xdir&ydir, while xmove&ymove will always be decremented.////
			if (xmove > 0)
				{
				xorgn = xorgn + (xdir/100)*e;			
				if (xorgn == x_total || xorgn == -x_total)
				xorgn = 0;
				else if (xorgn < -x_total)
				xorgn=xorgn+x_total;
				else if (xorgn > x_total)
				xorgn=xorgn - x_total;
				xmove = xmove - Abs((xdir/100)*e);
				}
			if (ymove > 0)
				{
				yorgn = yorgn + (ydir/100)*e;
				ymove = ymove - Abs((ydir/100)*e);
				}
  }
void stepsNew(int num_steps){			//stepper motor library by Engr Hassan. do not edit this function //Edited by Engr Aftab
	/*
		for this functiom
		one tick for timer will be of 1 microsec in cubeMX
	*/
	// Variables
	int flag = 0;
	long now = 0;
	int steps_left;
	last_step_time = 0;
	// getting direction
	if (num_steps > 0){
		direction = 1;						//RIGHT or DOWN MOVEMENT
	steps_left = num_steps;}
	else if (num_steps < 0){
		direction = 0;						//LEFT or UP MOVEMENT
	steps_left = -num_steps;}
	
	if (flag == 0){
		HAL_TIM_Base_Start(&htim2);
		htim2.Instance->CNT = 0;						// CURRENT TIMER2 VALUE //Edited by Engr TANZIL
		flag = 1;
	}
	
	while (steps_left > 0)
		{	
		if (flag == 0){
			HAL_TIM_Base_Start(&htim2);
			htim2.Instance->CNT = 0;
			flag = 1;
		}
		now = htim2.Instance->CNT;								// CURRENT TIMER2 VALUE //Edited by Engr TANZIL
		if (now - last_step_time >= step_delay)		//CURRENT TIME - LAST STEP TIME >= TIME B/W STEPS BASED ON SPEED //// move only if the appropriate delay has passed:
			{
			HAL_TIM_Base_Stop(&htim2);
			flag = 0;
			
			if (direction == 1)										//IF DIRECTION OF ROTATION IS +VE(clk wise), Right or DOWN MOVEMENT
				{
				HAL_GPIO_WritePin(dir_pan_GPIO_Port, dir_pan_Pin, 1); //Edited by Engr Tanzeel
				HAL_GPIO_WritePin(dir_tilt_GPIO_Port, dir_tilt_Pin, 1); //Edited by Engr Tanzeel
				step_number++;																//INCRECEMENT IN STEP, ON WHICH THE MOTOR IS AT
				if (step_number == number_of_steps)
					step_number = 0;
				}
			else {					//IF DIRECTION OF ROTATION IS -VE(anti-clk wise),left from cam POV or UP MOVEMENT
				HAL_GPIO_WritePin(dir_pan_GPIO_Port, dir_pan_Pin, 0); //Edited by Engr Tanzeel
				HAL_GPIO_WritePin(dir_tilt_GPIO_Port, dir_tilt_Pin, 0); //Edited by Engr Tanzeel
				if (step_number == 0)														
					step_number = number_of_steps;
				step_number--;
			}
			steps_left--;
			stepMotorNew(step_number%2); //Edited by Engr Aftab
		}
		// comment below if not working properly
		// from here
		if (htim2.Instance->CNT >= 65500){
			HAL_TIM_Base_Stop(&htim2);
			flag = 0;
		}
	}
}

void stepsNew_d(int num_steps_p,int num_steps_t){			//stepper motor library for diagonal movement by Engr Tanzeel.
	/*
		for this functiom
		one tick for timer will be of 1 microsec in cubeMX
	*/
	// Variables
	int flag = 0;
	long now = 0;
	int steps_left_p = 0;
	int steps_left_t = 0;
	last_step_time = 0;
	// getting direction
	if ((num_steps_p >= 0) && (num_steps_t <= 0))
		{
		direction = 1;						//UpRight MOVEMENT
		steps_left_p = num_steps_p;
		steps_left_t = -num_steps_t;
		}
	else if ((num_steps_p > 0) && (num_steps_t > 0))
		{
		direction = 2;						//DownRight MOVEMENT
		steps_left_p = num_steps_p;
		steps_left_t = num_steps_t;
		}
	else if ((num_steps_p < 0) && (num_steps_t < 0))
		{
		direction = 3;						//UpLeft MOVEMENT
		steps_left_p = -num_steps_p;
		steps_left_t = -num_steps_t;
		}
	else if ((num_steps_p < 0) && (num_steps_t > 0))
		{
		direction = 4;						//DownLeft MOVEMENT
		steps_left_p = -num_steps_p;
		steps_left_t = num_steps_t;
		}		
	
	if (flag == 0){
		HAL_TIM_Base_Start(&htim2);
		htim2.Instance->CNT = 0;						// CURRENT TIMER2 VALUE 
		flag = 1;
	}
	
	while (steps_left_p > 0 && steps_left_t > 0)
		{
			//Prevent head from body encounter incase of malfunctioning
			if((HAL_GPIO_ReadPin(t1_sw_GPIO_Port,t1_sw_Pin) == 1 && yorgn<-430) || (HAL_GPIO_ReadPin(t2_sw_GPIO_Port,t2_sw_Pin) == 1 && yorgn>420))
			{
				t_limit_count = 5;	//edited by Engr Tanzeel
				break;
			}
		if (flag == 0){
			HAL_TIM_Base_Start(&htim2);
			htim2.Instance->CNT = 0;
			flag = 1;
		}
		now = htim2.Instance->CNT;								// CURRENT TIMER2 VALUE 
		if (now - last_step_time >= step_delay)		//CURRENT TIME - LAST STEP TIME >= TIME B/W STEPS BASED ON SPEED //// move only if the appropriate delay has passed:
			{
			HAL_TIM_Base_Stop(&htim2);
			flag = 0;
		
			if (direction == 1)		
				{
				HAL_GPIO_WritePin(dir_pan_GPIO_Port, dir_pan_Pin, 1); 
				HAL_GPIO_WritePin(dir_tilt_GPIO_Port, dir_tilt_Pin, 0); 
				step_number_d++;																//INCRECEMENT IN STEP, ON WHICH THE PAN MOTOR IS AT
				if (step_number_d == number_of_steps)
					step_number_d = 0;
				}
			else if(direction == 2)
				{
				HAL_GPIO_WritePin(dir_pan_GPIO_Port, dir_pan_Pin, 1); 
				HAL_GPIO_WritePin(dir_tilt_GPIO_Port, dir_tilt_Pin, 1);					
				step_number_d++;
				if (step_number_d == number_of_steps)
					step_number_d = 0;
				}
			else if(direction == 3)
				{
				HAL_GPIO_WritePin(dir_pan_GPIO_Port, dir_pan_Pin, 0); 
				HAL_GPIO_WritePin(dir_tilt_GPIO_Port, dir_tilt_Pin, 0);						
				if (step_number_d == 0)
					step_number_d = number_of_steps;
				step_number_d--;
				}
			else if(direction == 4)
				{
				HAL_GPIO_WritePin(dir_pan_GPIO_Port, dir_pan_Pin, 0); 
				HAL_GPIO_WritePin(dir_tilt_GPIO_Port, dir_tilt_Pin, 1);
				if (step_number_d == 0)
					step_number_d = number_of_steps;
				step_number_d--;
		
				}
			steps_left_p--;
			steps_left_t--;	
			stepMotorNew(step_number_d%2); 
		}
		// comment below if not working properly
		// from here
		if (htim2.Instance->CNT >= 65500){
			HAL_TIM_Base_Stop(&htim2);
			flag = 0;
		}
	}
}

void PERFORM_TASK(){
	if(rec_cmd == GO_TO)
	{
			rec_cmd = NO;
		patrolling_flag = YES;
			call_operation(cam_sel, compId5-1);	// compId5 is the preset number
	}
	else if(rec_cmd == SCAN)
	{
			patrolling_flag = 2;
			cam_sel = compId5;
			HAL_Delay(100);
			Scan(cam_sel); 	
	}
	else if(rec_cmd == PAN)
	{
		Pan();
	}
 else if (rec_cmd == UP_RIGHT)		//EDITED BY ENGR TANZEEL
		{	
		patrolling_flag = NO;
		if(HAL_GPIO_ReadPin(t1_sw_GPIO_Port,t1_sw_Pin) == 0)		//up sw     will move upwards till hit upper sw
			{
		HAL_GPIO_WritePin(enab_tilt_GPIO_Port, enab_tilt_Pin, 0); 
		HAL_GPIO_WritePin(enab_pan_GPIO_Port, enab_pan_Pin, 0);
	
		setSpeed(30);
		stepsNew_d(100,-100); 
		
		setSpeed(60);		
    stepsNew_d(100,-100); 
		
		setSpeed(90);	
    stepsNew_d(100,-100);		
		
		xorgn = xorgn + 3;
		yorgn = yorgn - 3;
		if (xorgn == x_total) 
		xorgn = 0;
		else if (xorgn > x_total)
		xorgn=xorgn-x_total;
		setSpeed(compId4 + 0x14 + 70);
		while ( rec_cmd == UP_RIGHT || compId3 == 0x07 || compId4 > 0x00) //compId3 == 0x07 will keep the camera in user control when tour is left on
				{
					tour_flag = NO;
		stepsNew_d(stepss,-stepss);
		xorgn++;
		yorgn--;
			if ((xorgn >= x_total) || (xorgn <= -x_total))
			xorgn = 0;
			if(HAL_GPIO_ReadPin(t1_sw_GPIO_Port,t1_sw_Pin) == 1)
				break;
				}
			}
			else 
			{
		HAL_GPIO_WritePin(enab_tilt_GPIO_Port, enab_tilt_Pin, 1); 
		HAL_GPIO_WritePin(enab_pan_GPIO_Port, enab_pan_Pin, 0);	
			goto Right;
			}
		}
		
		else if (rec_cmd == DOWN_LEFT)		//EDITED BY ENGR TANZEEL
		{
		patrolling_flag = NO;
		if(HAL_GPIO_ReadPin(t2_sw_GPIO_Port,t2_sw_Pin) == 0)		//lower limit sw     will move downward till hit lower sw
		{
		HAL_GPIO_WritePin(enab_tilt_GPIO_Port, enab_tilt_Pin, 0); 
		HAL_GPIO_WritePin(enab_pan_GPIO_Port, enab_pan_Pin, 0);
				
		setSpeed(30);
		stepsNew_d(-100,100); 
		
		setSpeed(60);		
    stepsNew_d(-100,100); 
		
		setSpeed(90);	
    stepsNew_d(-100,100);		
		
		xorgn = xorgn - 3;
		yorgn = yorgn + 3;
		if (xorgn == -x_total) 
		xorgn = 0;
		else if (xorgn < -x_total)
		xorgn=xorgn+x_total;
		
		setSpeed(compId4 + 0x14 + 70);
		while ( rec_cmd == DOWN_LEFT || compId3 == 0x07 || compId4 > 0x00)
			{
				tour_flag = NO;
		stepsNew_d(-stepss,stepss);
		xorgn--;
		yorgn++;
			if ((xorgn >= x_total) || (xorgn <= -x_total))
			xorgn = 0;
			if(HAL_GPIO_ReadPin(t2_sw_GPIO_Port,t2_sw_Pin) == 1)
			break;
			}
		}
			else 
		{
		HAL_GPIO_WritePin(enab_tilt_GPIO_Port, enab_tilt_Pin, 1); 
		HAL_GPIO_WritePin(enab_pan_GPIO_Port, enab_pan_Pin, 0);	
		goto Left;
		}
	}
		else if (rec_cmd == DOWN_RIGHT)		//EDITED BY ENGR TANZEEL
		{
		patrolling_flag = NO;
		if(HAL_GPIO_ReadPin(t2_sw_GPIO_Port,t2_sw_Pin) == 0)		//lower limit sw     will move downward till hit lower sw
			{
		HAL_GPIO_WritePin(enab_tilt_GPIO_Port, enab_tilt_Pin, 0); 
		HAL_GPIO_WritePin(enab_pan_GPIO_Port, enab_pan_Pin, 0);
				
		setSpeed(30);
		stepsNew_d(100,100); 
		
		setSpeed(60);		
    stepsNew_d(100,100); 
		
		setSpeed(90);	
    stepsNew_d(100,100);		
		
		xorgn = xorgn + 3;
		yorgn = yorgn + 3;
		if (xorgn == x_total) 
		xorgn = 0;
		else if (xorgn > x_total)
		xorgn=xorgn-x_total;
		
		setSpeed(compId4 + 0x14 + 70);
		while ( rec_cmd == DOWN_RIGHT || compId3 == 0x07 || compId4 > 0x00)
				{
					tour_flag = NO;
		stepsNew_d(stepss,stepss);
		xorgn++;
		yorgn++;
			if ((xorgn >= x_total) || (xorgn <= -x_total))
			xorgn = 0;
			if(HAL_GPIO_ReadPin(t2_sw_GPIO_Port,t2_sw_Pin) == 1)
			break;
			}
		}
			else 
		{
		HAL_GPIO_WritePin(enab_tilt_GPIO_Port, enab_tilt_Pin, 1); 
		HAL_GPIO_WritePin(enab_pan_GPIO_Port, enab_pan_Pin, 0);
		goto Right;
		}
	}
		else if (rec_cmd == UP_LEFT)		//EDITED BY ENGR TANZEEL
		{	
		patrolling_flag = NO;
		if(HAL_GPIO_ReadPin(t1_sw_GPIO_Port,t1_sw_Pin) == 0)		//up sw     will move upwards till hit upper sw
			{
		HAL_GPIO_WritePin(enab_tilt_GPIO_Port, enab_tilt_Pin, 0); 
		HAL_GPIO_WritePin(enab_pan_GPIO_Port, enab_pan_Pin, 0);
				
		setSpeed(30);
		stepsNew_d(-100,-100); 
		
		setSpeed(60);		
    stepsNew_d(-100,-100); 
		
		setSpeed(90);	
    stepsNew_d(-100,-100);		
		
		xorgn = xorgn - 3;
		yorgn = yorgn - 3;
		if (xorgn == -x_total) 
		xorgn = 0;
		else if (xorgn < -x_total)
		xorgn=xorgn + x_total;
		
		setSpeed(compId4 + 0x14 + 70);
		while ( rec_cmd == UP_LEFT || compId3 == 0x07 || compId4 > 0x00)
				{
					tour_flag = NO;
		stepsNew_d(-stepss,-stepss);
		xorgn--;
		yorgn--;
			if ((xorgn >= x_total) || (xorgn <= -x_total))
			xorgn = 0;
			if(HAL_GPIO_ReadPin(t1_sw_GPIO_Port,t1_sw_Pin) == 1)
				break;
				}
			}
			else 
		{
		HAL_GPIO_WritePin(enab_tilt_GPIO_Port, enab_tilt_Pin, 1); 
		HAL_GPIO_WritePin(enab_pan_GPIO_Port, enab_pan_Pin, 0);	
		goto Left;
		}
	}		
		else if (rec_cmd == UP){	
		if(HAL_GPIO_ReadPin(t1_sw_GPIO_Port,t1_sw_Pin) == 0)
		{
		patrolling_flag = NO;
		HAL_GPIO_WritePin(enab_pan_GPIO_Port, enab_pan_Pin, 1); //Edited by Engr Aftab 
		HAL_GPIO_WritePin(enab_tilt_GPIO_Port, enab_tilt_Pin, 0); //Edited by Engr Aftab
	//	HAL_UART_Transmit(&hlpuart1, (uint8_t*)"UP: ", 4, 10);
		
		
		setSpeed(30);
		stepsNew(-100); //Edited by Engr Aftab
		
		setSpeed(60);		//up sw     will move upwards till hit upper sw
    stepsNew(-100);
		
		setSpeed(90);		//up sw     will move upwards till hit upper sw
    stepsNew(-100);
		
		yorgn = yorgn - 3;
		
		setSpeed(compId5 + 0x14 + 70);	
		
		while ( rec_cmd == UP || compId3 == 0x07 || compId4 > 0x00){
			tour_flag = NO;
		stepsNew(-stepss); //Edited by Engr Aftab
		yorgn--;
		if(HAL_GPIO_ReadPin(t1_sw_GPIO_Port,t1_sw_Pin)==1)
			break;
		}
	}
	}
	else if (rec_cmd == DOWN){
		if(HAL_GPIO_ReadPin(t2_sw_GPIO_Port,t2_sw_Pin) == 0)
		{	
		patrolling_flag = NO;
		HAL_GPIO_WritePin(enab_pan_GPIO_Port, enab_pan_Pin, 1); //Edited by Engr Aftab
		HAL_GPIO_WritePin(enab_tilt_GPIO_Port, enab_tilt_Pin, 0); //Edited by Engr Aftab
		
		setSpeed(30);
		stepsNew(100); //Edited by Engr Aftab
		
		setSpeed(60);		//up sw     will move upwards till hit upper sw
    stepsNew(100);
		
		setSpeed(90);		//up sw     will move upwards till hit upper sw
    stepsNew(100);
		
		yorgn = yorgn + 3;
		
		setSpeed(compId5 + 0x14 + 70);
		while ( rec_cmd == DOWN || compId3 == 0x07 || compId4 > 0x00){
			tour_flag = NO;
		stepsNew(stepss); //Edited by Engr Aftab
		yorgn++;
		if(HAL_GPIO_ReadPin(t2_sw_GPIO_Port,t2_sw_Pin) == 1)
		{
			break;
		}
		}
	}
	}
	else if (rec_cmd == RIGHT){			//EDITED BY ENGR TANZEEL
		patrolling_flag = NO;
		HAL_GPIO_WritePin(enab_tilt_GPIO_Port, enab_tilt_Pin, 1); //Edited by Engr Aftab
		HAL_GPIO_WritePin(enab_pan_GPIO_Port, enab_pan_Pin, 0);  //Edited by Engr Aftab
	//	HAL_UART_Transmit(&hlpuart1, (uint8_t*)"RIGHT: ", 7, 10);
		
		setSpeed(30);
		stepsNew(100); 
		
		setSpeed(60);		
    stepsNew(100);
		
		setSpeed(90);	
    stepsNew(100);
		
		xorgn = xorgn + 3;
		if (xorgn == x_total) //Edited by Engr Tanzeel
		xorgn = 0;
		else if (xorgn > x_total)
		xorgn=xorgn-x_total;		
		setSpeed(compId4 + 0x14 + 70);
		Right:									//Right label for goto statement
		while (compId3 == 0x02 || rec_cmd == UP_RIGHT ||  rec_cmd == DOWN_RIGHT || compId3 == 0x07 || compId4 > 0x00 )
			{
				tour_flag = NO;
		stepsNew(stepss); //Edited by Engr Aftab			\\stepss=100
		xorgn++;			//FOR 100 stepss IN stepsNew, CAM WILL MOVE xorgn+1.
			if ((xorgn >= x_total) || (xorgn <= -x_total))	//Edited by Engr Tanzeel 
			xorgn = 0;
		}						
	}
	else if(rec_cmd == LEFT)		
		{						
		patrolling_flag = NO;
		HAL_GPIO_WritePin(enab_tilt_GPIO_Port, enab_tilt_Pin, 1); //Edited by Engr Aftab
		HAL_GPIO_WritePin(enab_pan_GPIO_Port, enab_pan_Pin, 0);  //Edited by Engr Aftab
		
		setSpeed(30);
		stepsNew(-100); 
		
		setSpeed(60);		
    stepsNew(-100);
		
		setSpeed(90);		
    stepsNew(-100);
		
		xorgn = xorgn - 3;
		if (xorgn == -x_total) //Edited by Engr Tanzeel
		xorgn = 0;
		else if (xorgn < -x_total)
		xorgn=xorgn+x_total;		
		setSpeed(compId4 + 0x14 + 70);
		Left:
		while ( rec_cmd == LEFT|| rec_cmd == UP_LEFT ||  rec_cmd == DOWN_LEFT || compId3 == 0x07 || compId4 > 0x00)
			{
				tour_flag = NO;
		stepsNew(-stepss); //Edited by Engr Aftab
		xorgn--;
		if ((xorgn >= x_total) || (xorgn <= -x_total))	//Edited by Engr Tanzeel
			xorgn = 0;
		}
	}
	else if (rec_cmd == ZOOM_IN){
		patrolling_flag = NO;
		tour_flag = NO;
		if (cam_sel == 1){
		HAL_GPIO_WritePin(en_zoom_GPIO_Port, en_zoom_Pin, 1);					//ENABLE of L293D
		HAL_GPIO_WritePin(in2_zoom_GPIO_Port, in2_zoom_Pin, 1);
		HAL_GPIO_WritePin(in1_zoom_GPIO_Port, in1_zoom_Pin, 0);}
	}
	else if (rec_cmd == ZOOM_OUT){
		patrolling_flag = NO;
		tour_flag = NO;
		if(cam_sel == 1){
		HAL_GPIO_WritePin(en_zoom_GPIO_Port, en_zoom_Pin, 1);
		HAL_GPIO_WritePin(in2_zoom_GPIO_Port, in2_zoom_Pin, 0);
		HAL_GPIO_WritePin(in1_zoom_GPIO_Port, in1_zoom_Pin, 1);}
	}
	else if (rec_cmd == STOP_MOTORS){
		patrolling_flag = NO;
		tour_flag = NO;
		HAL_GPIO_WritePin(enab_tilt_GPIO_Port, enab_tilt_Pin, 1); //Edited by Engr Aftab
		HAL_GPIO_WritePin(enab_pan_GPIO_Port, enab_pan_Pin, 1); //Edited by Engr Aftab
		HAL_GPIO_WritePin(en_zoom_GPIO_Port, en_zoom_Pin, 0);
		HAL_GPIO_WritePin(in2_zoom_GPIO_Port, in2_zoom_Pin, 0);
		HAL_GPIO_WritePin(in1_zoom_GPIO_Port, in1_zoom_Pin, 0);
		rec_cmd = NO;
		HAL_GPIO_WritePin(in1_GPIO_Port,in1_Pin,0);
		HAL_GPIO_WritePin(dir_pan_GPIO_Port,dir_pan_Pin,0);
		HAL_GPIO_WritePin(dir_tilt_GPIO_Port,dir_pan_Pin,0);	
	}
	//rec_cmd = NO;
	tour_flag = YES;
}
void selfTest(void) {
	HAL_GPIO_WritePin(enab_tilt_GPIO_Port,enab_tilt_Pin, 1); //Edited by Engr Aftab
	HAL_GPIO_WritePin(enab_pan_GPIO_Port,enab_pan_Pin,0);  //Edited by Engr Aftab
	
	if(HAL_GPIO_ReadPin(p_sw_GPIO_Port,p_sw_Pin)==1)		// camera is already on home position in x axis
		{
		setSpeed(30);
		stepsNew(12000);	// moving it away from home //Edited by Engr Aftab
		xorgn = xorgn + 120;		// 12000/100=120
		}

 if(HAL_GPIO_ReadPin(p_sw_GPIO_Port,p_sw_Pin)==0){ // checking if pan switch is detected or not
		
			/* 
			gaining momentum as we slowly ramp up the speed of the motor as nema motors can't be operated on high speed from the start
		  Edited by Engr Shahmeer
			*/
			 
		setSpeed(30);
		stepsNew(100);  
		
		setSpeed(60);		
    stepsNew(100);
		
		setSpeed(90);		
    stepsNew(100);
	
		setSpeed(120);		
    stepsNew(100);
		
		setSpeed(150);		
    stepsNew(100);
		
		xorgn = xorgn + 5; // (100*5=500)/100=5
		
		setSpeed(180);
		while(HAL_GPIO_ReadPin(p_sw_GPIO_Port,p_sw_Pin)==0){
			stepsNew(stepss);		//stepss=100.
			xorgn++;				//FOR EVERY 100 STEPS IT WILL INCRECEMENT XORGN BY 1.
		}	
	}				//will stop once hit pan limit switch
	
//	double xp = xorgn;
//	sprintf(curr_msg,"t0.txt=\"X: %0.1f\"", xp);
//	SEND_TO_NEXTION(curr_msg);
	
	HAL_GPIO_WritePin(enab_tilt_GPIO_Port,enab_tilt_Pin, 1); //Edited by Engr Aftab
	HAL_GPIO_WritePin(enab_pan_GPIO_Port,enab_pan_Pin,1);  //Edited by Engr Aftab
	
	HAL_Delay(700);
	
	HAL_GPIO_WritePin(enab_tilt_GPIO_Port,enab_tilt_Pin,0); //Edited by Engr Aftab
	HAL_GPIO_WritePin(enab_pan_GPIO_Port,enab_pan_Pin,1);  //Edited by Engr Aftab
	
	if(HAL_GPIO_ReadPin(t1_sw_GPIO_Port,t1_sw_Pin) == 0){ // checking if upper tilt switch is detected or not / Edidted by Engr Shahmeer
		
			/* 
			gaining momentum as we slowly ramp up the speed of the motor as nema motors can't be operated on high speed from the start
		  Edited by Engr Shahmeer
			*/
		
		setSpeed(30);
		stepsNew(-100);  
		
		setSpeed(60);		
    stepsNew(-100);
		
		setSpeed(90);		
    stepsNew(-100);
	
		setSpeed(120);		
    stepsNew(-100);
		
		setSpeed(150);		
    stepsNew(-100);
		
		yorgn = yorgn - 5;
		
		setSpeed(180);
	
  while (HAL_GPIO_ReadPin(t1_sw_GPIO_Port,t1_sw_Pin) == 0) {  	//will move upwards till hit upper sw (yorgn=-429)
    stepsNew(-stepss); //Edited by Engr Aftab
		yorgn--;
  }
	}
  HAL_GPIO_WritePin(enab_tilt_GPIO_Port,enab_tilt_Pin, 1); //Edited by Engr Aftab
  HAL_Delay(700);
  int step_count = 0;
	
	HAL_GPIO_WritePin(enab_tilt_GPIO_Port,enab_tilt_Pin, 0); //Edited by Engr Aftab
	
	if(HAL_GPIO_ReadPin(t2_sw_GPIO_Port,t2_sw_Pin) == 0){ // checking if lower tilt switch is detected or not
		
			/* 
			gaining momentum as we slowly ramp up the speed of the motor as nema motors can't be operated on high speed from the start
		  Edited by Engr Shahmeer
			*/
			
		setSpeed(30);
		stepsNew(100);  
		
		setSpeed(60);		
    stepsNew(100);
		
		setSpeed(90);		
    stepsNew(100);
	
		setSpeed(120);		
    stepsNew(100);
		
		setSpeed(150);		
    stepsNew(100);
		
		yorgn=yorgn+5;
		
		setSpeed(180);
		
  while (HAL_GPIO_ReadPin(t2_sw_GPIO_Port,t2_sw_Pin) == 0) {       //will move downwards till hit lower sw yorgn=417)
    stepsNew(stepss); 		//Edited by Engr Aftab
		yorgn++;
    step_count++;					//counting steps from upper sw to lower switch
  }
	}
  HAL_GPIO_WritePin(enab_tilt_GPIO_Port,enab_tilt_Pin, 1); //Edited by Engr Aftab
  HAL_Delay(700);
  step_count = step_count / 2;					//for mid point btween 2 limit swtches
  HAL_GPIO_WritePin(enab_tilt_GPIO_Port,enab_tilt_Pin, 0); //Edited by Engr Aftab
	
			/* 
			gaining momentum as we slowly ramp up the speed of the motor as nema motors can't be operated on high speed from the start
		  Edited by Engr Shahmeer
			*/
	
		setSpeed(30);
		stepsNew(-100); 
		
		setSpeed(40);		
    stepsNew(-1000);
		
		setSpeed(60);		
    stepsNew(-1000);
		
		setSpeed(80);		
    stepsNew(-1000);
		
		setSpeed(100);		
    stepsNew(-1000);
		
		setSpeed(120);		
    stepsNew(-1000);
		
		setSpeed(140);		
    stepsNew(-1000);
		
		setSpeed(160);		
    stepsNew(-1000);
		
		
		step_count = step_count - 71;
		
		setSpeed(180);
		
  while (step_count > 0) {				//bringing to midpoint btween two tilt limit switches
    stepsNew(-stepss); //Edited by Engr Aftab
    step_count--;
  }
	
//  myStepper.step(-200);
//  HAL_GPIO_WritePin(enab_tilt_GPIO_Port,enab_tilt_Pin, 1); //Edited by Engr Aftab
	
  xorgn = 0;
  yorgn = 0;				//setting origin for first time only
  HAL_Delay(1000);
	HAL_GPIO_WritePin(enab_tilt_GPIO_Port, enab_tilt_Pin, 1);
	HAL_GPIO_WritePin(enab_pan_GPIO_Port, enab_pan_Pin, 1);
	t_limit_count = 0;
}

void unlocking_eeprom(){				//library. do not edit
	while ((FLASH->SR & FLASH_SR_BSY) != 0) /* (1) */
		{
			/* For robust implementation, add here time-out management */
		}
	if ((FLASH->PECR & FLASH_PECR_PELOCK) != 0) /* (2) */
		{
			FLASH->PEKEYR = FLASH_PEKEY1; /* (3) */
			FLASH->PEKEYR = FLASH_PEKEY2;
		}
}

void locking_eeprom(){			//library. do not edit
	while ((FLASH->SR & FLASH_SR_BSY) != 0) /* (1) */
		{
			/* For robust implementation, add here time-out management */
		}
	FLASH->PECR |= FLASH_PECR_PELOCK; /* (2) */
}

void unlock_nvm_prog_mem(){			//library. do not edit
	while ((FLASH->SR & FLASH_SR_BSY) != 0)
		{
			/* For robust implementation, add here time-out management */
		}
	if ((FLASH->PECR & FLASH_PECR_PELOCK) == 0) /* (2) */
		{
			if ((FLASH->PECR & FLASH_PECR_PRGLOCK) != 0) /* (3) */
				{
					FLASH->PRGKEYR = FLASH_PRGKEY1; /* (4) */
					FLASH->PRGKEYR = FLASH_PRGKEY2;
				}
		}
}

void unlocking_byte_area(){			//library. do not edit
	/* (1) Wait till no operation is on going */
	/* (2) Check that the PELOCK is unlocked */
	/* (3) Check if the OPTLOCK is unlocked */
	/* (4) Perform unlock sequence */
	while ((FLASH->SR & FLASH_SR_BSY) != 0) /* (1) */
		{
			/* For robust implementation, add here time-out management */
		}
	if ((FLASH->PECR & FLASH_PECR_PELOCK) == 0) /* (2) */
		{
			if ((FLASH->PECR & FLASH_PECR_OPTLOCK) != 0) /* (2) */
			{
				FLASH->OPTKEYR = FLASH_OPTKEY1; /* (3) */
				FLASH->OPTKEYR = FLASH_OPTKEY2;
			}
		}
}

void erase_to_data_eeprom(){			//library. do not edit
	FLASH->PECR |= FLASH_PECR_ERASE | FLASH_PECR_DATA; /* (1) */
	*(__IO uint32_t *)addr = (uint32_t)0; /* (2) */
	__WFI(); /* (3) */
	FLASH->PECR &= ~(FLASH_PECR_ERASE | FLASH_PECR_DATA);
}


void write_eeprom(int16_t data, uint32_t	virtual_address){			//library. do not edit
	virtual_address = virtual_address*2;
 *(int16_t *)(_EE_ADDR_INUSE + virtual_address) = data;
}

int16_t read_eeprom(int virtual_address){			//library. do not edit
	virtual_address = virtual_address*2;
	return *(int16_t*)(_EE_ADDR_INUSE + virtual_address);
}
void get_presets(void){
/*
	total presets of each cam is 32
	1 preset is taking 4 bytes 
		2 bytes for x and 2 bytes for y axis... 16 bit
	As the total size of EEPROM is 512 bytes
		0~63 x presets of cam 1 
		64~127 y presets of cam 1
		128~191 x presets of cam 2
		192~255 y presets of cam 2
	
	for first preset the x will be at 0 and 1 byte of cam 1
	and y will be at 64 and 65 byte	
*/
	uint8_t LSB= 0x00,MSB = 0x00;		
	int t_addr=0;		// temporary address
	for(int i = 0; i < 32; i++){					//because total presets from camera can be 32, comprising of 64 bytes of each axis
		t_addr = i*2;	// as 1 preset is of two byte on each axis so the index will be multipled with 2... so one preset is taking 16 bit in X AND 16 bit in Y.
		//for x-axis of cam1	0~63
		MSB = *(uint8_t*)(DATA_E2_ADDR + t_addr);	// DATA_E2_ADDR is the initial address of EEPROM, t_addr is virtual address it saves MSB... *(uint8_t*)  using pointer, getting 8 bit value from location and putting it in uint8 MSB
		LSB = *(uint8_t*)(DATA_E2_ADDR + (t_addr+1));	// it will be used to save (t_addr+1 for LSB) LSB from EEPROM
		counter_main[i][0] = MSB;															//loading the MSB of x coording from eeprom
		counter_main[i][0] = (counter_main[i][0]<<8);	// left shifting to make it MSB
		counter_main[i][0] |= LSB;// ADDING LSB to array
		//////////////////////////////////////////////////
		LSB = 0x00;
		MSB = 0x00;
		t_addr = (i*2) + 64;			//for y-axis of cam1	64-127
		MSB = *(uint8_t*)(DATA_E2_ADDR + t_addr);
		LSB = *(uint8_t*)(DATA_E2_ADDR + (t_addr+1));
		counter_main[i][1] = MSB;
		counter_main[i][1] = (counter_main[i][1]<<8);
		counter_main[i][1] |= LSB;
		/////////////////////////////////////////////////
		/////////////////////////////////////////////////
		LSB = 0x00;
		MSB = 0x00;
		t_addr = (i*2) + 128;	// cam2 x-preset addresses starts 128-191
		MSB = *(uint8_t*)(DATA_E2_ADDR + t_addr);
		LSB = *(uint8_t*)(DATA_E2_ADDR + (t_addr+1));
		counter_main_1[i][0] = MSB;															//loading the counter_main array with x coord values from eeprom
		counter_main_1[i][0] = (counter_main_1[i][0]<<8);
		counter_main_1[i][0] |= LSB;
		/////////////////////////////////////////////////
		LSB = 0x00;
		MSB = 0x00;
		t_addr = (i*2) + 192;// cam2 y-preset addresses starts 192-255
		MSB = *(uint8_t*)(DATA_E2_ADDR + t_addr);
		LSB = *(uint8_t*)(DATA_E2_ADDR + (t_addr+1));
		counter_main_1[i][1] = MSB;
		counter_main_1[i][1] = (counter_main_1[i][1]<<8);
		counter_main_1[i][1] |= LSB;		
	}
}

void get_limits(void){
/*
	total scan limits are 4, for each cam is 2, left and right
	1 limit is taking 2 bytes (16 bit of x corrdinate)
	As the total size of EEPROM is 512 bytes
		At address 256 is the (x) left limit for cam 1 
		At 258 is the right limit for cam 1
		At 260 is the left limit for cam 2
		At 262 is the right limit for cam 2
*/
	uint8_t LSB= 0x00,MSB = 0x00;		
	int t_addr=0;		// temporary address
	for(int i = 0; i < 4; i++){
		t_addr = (i*2) + 256;	// as 1 limit is of two byte so the index will be multipled with 2... so one limit is taking 16 bit of memory
		MSB = *(uint8_t*)(DATA_E2_ADDR + t_addr);	// DATA_E2_ADDR is the initial address of EEPROM, t_addr is virtual address it saves MSB... *(uint8_t*)  using pointer, getting 8 bit value from location and putting it in uint8 MSB
		LSB = *(uint8_t*)(DATA_E2_ADDR + (t_addr+1));	// it will be used to save (t_addr+1 for LSB) LSB from EEPROM
		counter_limit[i] = MSB;															//loading the MSB of x coording from eeprom
		counter_limit[i] = (counter_limit[i]<<8);	// left shifting to make it MSB
		counter_limit[i] |= LSB;// ADDING LSB to array	
	}
}

void call_operation(int sel, uint8_t preset){		// it will drive the cam to specific preset ,as every cam has different presets so sel is for cam selection and preset is for preset number
	//tour_flag = NO;
	int16_t x_Axis=0, y_Axis=0, xcurr = 0;
	if (sel == 1)
	{
		x_Axis = counter_main[preset][0];				//counter_main array will be loaded from get_presets, ONLY the FIRST TIME. and will be updated thru update_presets function
		y_Axis = counter_main[preset][1];
	}
	else
	{
		x_Axis = counter_main_1[preset][0];
		y_Axis = counter_main_1[preset][1];
	}
	xmove=0, ymove = 0;
	xmove = Abs(xorgn - x_Axis);				//manhattan distance formula
	ymove = Abs(yorgn - y_Axis);
	int Dir = 0,Psw_count=0;
	if (xmove >= (x_total/2)){	//for shortest ditance to travel
		Dir = 1;
		if (x_Axis > xorgn) xmove = Abs(x_total + xorgn - x_Axis); 
		else if (x_Axis < xorgn) xmove = Abs(x_total - xorgn + x_Axis);
		if( Abs(x_total+xorgn)<x_Axis || Abs(x_total+x_Axis)<xorgn ) Dir=0;	
		if (xmove > (x_total/2))
			{
				xmove = Abs(xmove - x_total);
				Dir = 1;
			}
	}
	xcurr=xorgn;	//after 2000 xorgn will be reseted so taking xcurr as reference
	
////*****If x/ymove is less than ramp movement,****////
///******moving Cam Non-diagonally (x and y dir seprated) for	accurate position****////
		if(xmove<=12 || ymove<=12)
			{
			if(xmove<=12)
			{
				HAL_GPIO_WritePin(enab_pan_GPIO_Port, enab_pan_Pin, 0);
				HAL_GPIO_WritePin(enab_tilt_GPIO_Port, enab_tilt_Pin, 1);
				setSpeed(40);
			while( xmove > 0) {
			if (xcurr > x_Axis) {
				stepsNew(-stepss);
				xorgn--;
				if ((xorgn >= x_total) || (xorgn <= -x_total))
				xorgn = 0;
				}
		else if (xcurr < x_Axis){
				stepsNew(stepss); 
				xorgn++;
				if ((xorgn >= x_total) || (xorgn <= -x_total))
				xorgn = 0;
				}
					xmove--;
				}
			}
			if(ymove<=12)
			{
				HAL_GPIO_WritePin(enab_tilt_GPIO_Port, enab_tilt_Pin, 0);
				HAL_GPIO_WritePin(enab_pan_GPIO_Port, enab_pan_Pin, 1);
				setSpeed(40);
		while (ymove > 0)
					{
		if (yorgn > y_Axis) {
			stepsNew(-stepss); 
			yorgn--;
		}
		else if (yorgn < y_Axis) {
			stepsNew(stepss); 
			yorgn++;
		}			
		ymove--;
				}
			}
		}

	if(xmove > 0 || ymove > 0)
	{
		if (patrolling_flag == NO)
			return;
		if(xmove > 0 && ymove > 0)
		{
		HAL_GPIO_WritePin(enab_tilt_GPIO_Port, enab_tilt_Pin, 0);
		HAL_GPIO_WritePin(enab_pan_GPIO_Port, enab_pan_Pin, 0);
		}
		if(xmove == 0)
		{
		HAL_GPIO_WritePin(enab_pan_GPIO_Port, enab_pan_Pin, 1);
		HAL_GPIO_WritePin(enab_tilt_GPIO_Port, enab_tilt_Pin, 0);
		}
		if(ymove == 0)
		{
		HAL_GPIO_WritePin(enab_tilt_GPIO_Port, enab_tilt_Pin, 1);
		HAL_GPIO_WritePin(enab_pan_GPIO_Port, enab_pan_Pin, 0);
		}
		
		if ((xcurr >= x_Axis) && (yorgn >= y_Axis))
			{
			if (Dir == 0){
				
				/* 
				gaining momentum as we slowly ramp up the speed of the motor as nema motors can't be operated on high speed from the start
				*/
//					if (xmove>=1000 || ymove>=1000)
//					Ramp(330, -100, -100);
//					else if (xmove>=900 || ymove>=900)
//					Ramp(300, -100, -100);					
//					else if (xmove>=800 || ymove>=800)
//					Ramp(270, -100, -100);
					if (xmove>=700 || ymove>=700)
					Ramp(240, -100, -100);
					else if (xmove>=600 || ymove>=600)
					Ramp(210, -100, -100);
					else if (xmove>=500 || ymove>=500)
					Ramp(180, -100, -100);
					else if (xmove>=400 || ymove>=400)
					Ramp(150, -100, -100);
					else if (xmove>=300 || ymove>=300)
					Ramp(120, -100, -100);
					else
					Ramp(90, -100, -100);				
			}
			
			else
				{
				
				/* 
				Speed parameter is calculated manually so that cam will reach farest(move=1000) within approx 10 sec. 
				*/
				
//					if (xmove>=1000 || ymove>=1000)
//					Ramp(330, 100, -100);
//					else if (xmove>=900 || ymove>=900)
//					Ramp(300, 100, -100);					
//					else if (xmove>=800 || ymove>=800)
//					Ramp(270, 100, -100);
					 if (xmove>=700 || ymove>=700)
					Ramp(240, 100, -100); // it will start motor with 30 and with increment of 30 after each 100 steps, motor will reach the speed of 240
					else if (xmove>=600 || ymove>=600)
					Ramp(210, 100, -100);
					else if (xmove>=500 || ymove>=500)
					Ramp(180, 100, -100);
					else if (xmove>=400 || ymove>=400)
					Ramp(150, 100, -100);
					else if (xmove>=300 || ymove>=300)
					Ramp(120, 100, -100);
					else
					Ramp(90, 100, -100);		
												
			}
	}
		else if((xcurr <= x_Axis) && (yorgn >= y_Axis))
			{
			if (Dir == 0){
				
				/* 
				gaining momentum as we slowly ramp up the speed of the motor as nema motors can't be operated on high speed from the start
				*/
				
//					if (xmove>=1000 || ymove>=1000)
//					Ramp(330, 100, -100);
//					else if (xmove>=900 || ymove>=900)
//					Ramp(300, 100, -100);					
//					else if (xmove>=800 || ymove>=800)
//					Ramp(270, 100, -100);
					 if (xmove>=700 || ymove>=700)
					Ramp(240, 100, -100);
					else if (xmove>=600 || ymove>=600)
					Ramp(210, 100, -100);
					else if (xmove>=500 || ymove>=500)
					Ramp(180, 100, -100);
					else if (xmove>=400 || ymove>=400)
					Ramp(150, 100, -100);
					else if (xmove>=300 || ymove>=300)
					Ramp(120, 100, -100);
					else
					Ramp(90, 100, -100);				
				
		}
			else{
				
				/* 
				gaining momentum as we slowly ramp up the speed of the motor as nema motors can't be operated on high speed from the start
				*/
				
//					if (xmove>=1000 || ymove>=1000)
//					Ramp(330, -100, -100);
//					else if (xmove>=900 || ymove>=900)
//					Ramp(300, -100, -100);					
//					 else if (xmove>=800 || ymove>=800)
//					Ramp(270, -100, -100);
					 if (xmove>=700 || ymove>=700)
					Ramp(240, -100, -100);
					else if (xmove>=600 || ymove>=600)
					Ramp(210, -100, -100);
					else if (xmove>=500 || ymove>=500)
					Ramp(180, -100, -100);
					else if (xmove>=400 || ymove>=400)
					Ramp(150, -100, -100);
					else if (xmove>=300 || ymove>=300)
					Ramp(120, -100, -100);
					else
					Ramp(90, -100, -100);			
		}
		
	}
			else if ((xcurr >= x_Axis) && (yorgn <= y_Axis))
			{
			if (Dir == 0){
				
				/* 
				gaining momentum as we slowly ramp up the speed of the motor as nema motors can't be operated on high speed from the start
				*/
				
//					if (xmove>=1000 || ymove>=1000)
//					Ramp(330, -100, 100);
//					else if (xmove>=900 || ymove>=900)
//					Ramp(300, -100, 100);					
//					else if (xmove>=800 || ymove>=800)
//					Ramp(270, -100, 100);
					 if (xmove>=700 || ymove>=700)
					Ramp(240, -100, 100);
					else if (xmove>=600 || ymove>=600)
					Ramp(210, -100, 100);
					else if (xmove>=500 || ymove>=500)
					Ramp(180, -100, 100);
					else if (xmove>=400 || ymove>=400)
					Ramp(150, -100, 100);
					else if (xmove>=300 || ymove>=300)
					Ramp(120, -100, 100);
					else
					Ramp(90, -100, 100);			
			}
			
			else
				{
				 
				/* 
				gaining momentum as we slowly ramp up the speed of the motor as nema motors can't be operated on high speed from the start
				*/
				
//					if (xmove>=1000 || ymove>=1000)
//					Ramp(330, 100, 100);
//					else if (xmove>=900 || ymove>=900)
//					Ramp(300, 100, 100);					
//					else if (xmove>=800 || ymove>=800)
//					Ramp(270, 100, 100);
					 if (xmove>=700 || ymove>=700)
					Ramp(240, 100, 100);
					else if (xmove>=600 || ymove>=600)
					Ramp(210, 100, 100);
					else if (xmove>=500 || ymove>=500)
					Ramp(180, 100, 100);
					else if (xmove>=400 || ymove>=400)
					Ramp(150, 100, 100);
					else if (xmove>=300 || ymove>=300)
					Ramp(120, 100, 100);
					else
					Ramp(90, 100, 100);		
						
			}
	}
		else if((xcurr <= x_Axis) && (yorgn <= y_Axis))
			{
			if (Dir == 0){
				
				/* 
				gaining momentum as we slowly ramp up the speed of the motor as nema motors can't be operated on high speed from the start
				*/
				
//					if (xmove>=1000 || ymove>=1000)
//					Ramp(330, 100, 100);
//					else if (xmove>=900 || ymove>=900)
//					Ramp(300, 100, 100);					
//					else if (xmove>=800 || ymove>=800)
//					Ramp(270, 100, 100);
					 if (xmove>=700 || ymove>=700)
					Ramp(240, 100, 100);
					else if (xmove>=600 || ymove>=600)
					Ramp(210, 100, 100);
					else if (xmove>=500 || ymove>=500)
					Ramp(180, 100, 100);
					else if (xmove>=400 || ymove>=400)
					Ramp(150, 100, 100);
					else if (xmove>=300 || ymove>=300)
					Ramp(120, 100, 100);
					else
					Ramp(90, 100, 100);						
				
		}
			else{
				
				/* 
				gaining momentum as we slowly ramp up the speed of the motor as nema motors can't be operated on high speed from the start
				*/
				
//					if (xmove>=1000 || ymove>=1000)
//					Ramp(330, -100, 100);
//					else if (xmove>=900 || ymove>=900)
//					Ramp(300, -100, 100);					
//					else if (xmove>=800 || ymove>=800)
//					Ramp(270, -100, 100);
					 if (xmove>=700 || ymove>=700)
					Ramp(240, -100, 100);
					else if (xmove>=600 || ymove>=600)
					Ramp(210, -100, 100);
					else if (xmove>=500 || ymove>=500)
					Ramp(180, -100, 100);
					else if (xmove>=400 || ymove>=400)
					Ramp(150, -100, 100);
					else if (xmove>=300 || ymove>=300)
					Ramp(120, -100, 100);
					else
					Ramp(90, -100, 100);	
	
		}
	}
			while( xmove > 0 || ymove > 0) { 
			if (patrolling_flag == NO)
			break;
			/*
			Keep moving cam if xmove and ymove are left, disbale motor driver when corresponding move become 0.
			*/
				if (xmove <= 0)
				HAL_GPIO_WritePin(enab_pan_GPIO_Port, enab_pan_Pin, 1);
				if (ymove <= 0)
				HAL_GPIO_WritePin(enab_tilt_GPIO_Port, enab_tilt_Pin, 1);
		if ((xcurr >= x_Axis) && (yorgn >= y_Axis)) {		
			if (Dir == 0){
				stepsNew_d(-stepss,-stepss);		//left up
				if (xmove > 0)
				{
					xorgn--;
					if(HAL_GPIO_ReadPin(p_sw_GPIO_Port,p_sw_Pin)==1 ||Psw_count==1)	//feedback from pan limit switch from left side
					{
						//xorgn=90;
						Psw_count=1;
						if(HAL_GPIO_ReadPin(p_sw_GPIO_Port,p_sw_Pin)== 0 && Psw_count==1)
						{
							xorgn=-10;
							Psw_count=0;
						}
						
					}
					if ((xorgn >= x_total) || (xorgn <= -x_total))
							xorgn = 0;
				}
				if (ymove > 0)
				{
					yorgn--;
				}
			}
			else
			{
				stepsNew_d(stepss,-stepss);	
				if (xmove > 0)
				{
					xorgn++;
					if(HAL_GPIO_ReadPin(p_sw_GPIO_Port,p_sw_Pin)==0 || Psw_count==1)	//feedback from pan limit switch from right side
					{
						Psw_count=1;
						if(HAL_GPIO_ReadPin(p_sw_GPIO_Port,p_sw_Pin)== 1 && Psw_count==1)
						{
							xorgn=-2;
							Psw_count=0;
						}
					}
					if ((xorgn >= x_total) || (xorgn <= -x_total))
							xorgn = 0;
				}
				if (ymove > 0)
				{
					yorgn--;
				}
			}
		}
		else if ((xcurr <= x_Axis) && (yorgn >= y_Axis)) {
			if (Dir == 0){
				stepsNew_d(stepss,-stepss);		//right_up
				if (xmove > 0)
				{
					xorgn++;
					if(HAL_GPIO_ReadPin(p_sw_GPIO_Port,p_sw_Pin)==0 || Psw_count==1)
					{
						Psw_count=1;
						if(HAL_GPIO_ReadPin(p_sw_GPIO_Port,p_sw_Pin)== 1 && Psw_count==1)
						{
							xorgn=-2;
							Psw_count=0;
						}
					}
					if ((xorgn >= x_total) || (xorgn <= -x_total))
							xorgn = 0;
				}
				if (ymove > 0)
				{
					yorgn--;
				}
			}
			else{			
				stepsNew_d(-stepss,-stepss);
				if (xmove > 0)
				{
					xorgn--;
					if(HAL_GPIO_ReadPin(p_sw_GPIO_Port,p_sw_Pin)==1 ||Psw_count==1)
					{
						//xorgn=90;
						Psw_count=1;
						if(HAL_GPIO_ReadPin(p_sw_GPIO_Port,p_sw_Pin)== 0 && Psw_count==1)
						{
							xorgn=-10;
							Psw_count=0;
						}
					}
					if ((xorgn >= x_total) || (xorgn <= -x_total))
							xorgn = 0;
				}
				if (ymove > 0)
				{
					yorgn--;
				}
			}
		}
			else if ((xcurr >= x_Axis) && (yorgn <= y_Axis)) {			
			if (Dir == 0){
				stepsNew_d(-stepss,stepss);	//left_down
				if (xmove > 0)
				{
					xorgn--;
					if(HAL_GPIO_ReadPin(p_sw_GPIO_Port,p_sw_Pin)==1 ||Psw_count==1)
					{
						//xorgn=90;
						Psw_count=1;
						if(HAL_GPIO_ReadPin(p_sw_GPIO_Port,p_sw_Pin)== 0 && Psw_count==1)
						{
							xorgn=-10;
							Psw_count=0;
						}
					}
					if ((xorgn >= x_total) || (xorgn <= -x_total))
							xorgn = 0;
				}
					if (ymove > 0)
				{
					yorgn++;
				}
			}
			else
			{
				stepsNew_d(stepss,stepss);
				if (xmove > 0)
				{
					xorgn++;
					if(HAL_GPIO_ReadPin(p_sw_GPIO_Port,p_sw_Pin)==0 || Psw_count==1)
					{
						Psw_count=1;
						if(HAL_GPIO_ReadPin(p_sw_GPIO_Port,p_sw_Pin)== 1 && Psw_count==1)
						{
							xorgn=-2;
							Psw_count=0;
						}
					}
					if ((xorgn >= x_total) || (xorgn <= -x_total))
							xorgn = 0;
				}
					if (ymove > 0)
				{
					yorgn++;
				}
			}
		}
		else if ((xcurr <= x_Axis) && (yorgn <= y_Axis)) {
			if (Dir == 0){
				stepsNew_d(stepss,stepss); 	//right_down
				if (xmove > 0)
				{
					xorgn++;
					if(HAL_GPIO_ReadPin(p_sw_GPIO_Port,p_sw_Pin)==0 || Psw_count==1)
					{
						Psw_count=1;
						if(HAL_GPIO_ReadPin(p_sw_GPIO_Port,p_sw_Pin)== 1 && Psw_count==1)
						{
							xorgn=-2;
							Psw_count=0;
						}
					}
					if ((xorgn >= x_total) || (xorgn <= -x_total))
							xorgn = 0;
				}
				 if (ymove > 0)
				{
					yorgn++;
				}
			}
			else{			
				stepsNew_d(-stepss,stepss);
				if (xmove > 0)
				{
					xorgn--;
					if(HAL_GPIO_ReadPin(p_sw_GPIO_Port,p_sw_Pin)==1 ||Psw_count==1)
					{
						//xorgn=90;
						Psw_count=1;
						if(HAL_GPIO_ReadPin(p_sw_GPIO_Port,p_sw_Pin)== 0 && Psw_count==1)
						{
							xorgn=-10;
							Psw_count=0;
						}
					}
					if ((xorgn >= x_total) || (xorgn <= -x_total))
							xorgn = 0;
				}
				 if (ymove > 0)
				{
					yorgn++;
				}			
			}
		}
		if (xmove > 0)
				{
					xmove--;
				}
	  if (ymove > 0)
				{
					ymove--;
				}		
		}
		HAL_GPIO_WritePin(enab_tilt_GPIO_Port, enab_tilt_Pin, 1);
		HAL_GPIO_WritePin(enab_pan_GPIO_Port, enab_pan_Pin, 1);
	}	
}

void Scan(uint8_t c_sel)
{ 
	int16_t l_limit=0, r_limit=0, xcurr=0;
	if (c_sel == 1)		//compId5=1 for day cam
	{
		l_limit = counter_limit[0];				//counter_limit array will be loaded from get_limits, ONLY the FIRST TIME. and will be updated thru update_limits function
		r_limit = counter_limit[1];
	}
	else						//compId5=0 for day cam
	{
		l_limit = counter_limit[2];
		r_limit = counter_limit[3];
	}
	
	int lmove=0, rmove=0;
	lmove = Abs(xorgn - l_limit);				//manhattan distance formula for left limit
	rmove = Abs(xorgn - r_limit);				//manhattan distance formula for right limit
	xcurr=xorgn;
	
			if(lmove<=3)
			{
				HAL_GPIO_WritePin(enab_pan_GPIO_Port, enab_pan_Pin, 0);
				HAL_GPIO_WritePin(enab_tilt_GPIO_Port, enab_tilt_Pin, 1);
				setSpeed(40);
			while( lmove > 0) {
			if (xcurr > l_limit) {		
				stepsNew(-stepss);
				xorgn--;
				if ((xorgn >= x_total) || (xorgn <= -x_total))
				xorgn = 0;
				}
			else if (xcurr < l_limit){
				stepsNew(stepss); 
				xorgn++;
				if ((xorgn >= x_total) || (xorgn <= -x_total))
				xorgn = 0;
				}
					lmove--;
				}
			}
			HAL_Delay(2000);
			if(rmove<=3)
			{
				HAL_GPIO_WritePin(enab_pan_GPIO_Port, enab_pan_Pin, 0);
				HAL_GPIO_WritePin(enab_tilt_GPIO_Port, enab_tilt_Pin, 1);
				setSpeed(40);
			while( rmove > 0) {
			if (xcurr > r_limit) {		
				stepsNew(-stepss);
				xorgn--;
				if ((xorgn >= x_total) || (xorgn <= -x_total))
				xorgn = 0;
				}
			else if (xcurr < r_limit){
				stepsNew(stepss); 
				xorgn++;
				if ((xorgn >= x_total) || (xorgn <= -x_total))
				xorgn = 0;
				}
					rmove--;
				}
			}
			
		if ((lmove > 0) && (patrolling_flag == 2)){
		HAL_GPIO_WritePin(enab_tilt_GPIO_Port, enab_tilt_Pin, 1); 
		HAL_GPIO_WritePin(enab_pan_GPIO_Port, enab_pan_Pin, 0);
		
		if (xcurr > l_limit){
				
				setSpeed(30);
				stepsNew(-100); 
				
				setSpeed(60);		
				stepsNew(-100);
				
				setSpeed(90);		
				stepsNew(-100);
					
				xorgn = xorgn - 3;
				if (xorgn == -x_total) //Edited by Engr Tanzeel
				xorgn = 0;
				else if (xorgn < -x_total)
				xorgn=xorgn+x_total;
				
				lmove = lmove - 3;
				//setSpeed(compId5 + 0x14 + 50);
				}
		else if (xcurr < l_limit){
				
				setSpeed(30);
				stepsNew(100); 
				
				setSpeed(60);		
				stepsNew(100);
				
				setSpeed(90);		
				stepsNew(100);
						
				xorgn = xorgn + 3;
				if (xorgn == x_total) //Edited by Engr Tanzeel
				xorgn = 0;
				else if (xorgn > x_total)
				xorgn=xorgn-x_total;
				
				lmove = lmove - 3;
				//setSpeed(compId5 + 0x14 + 50);
				}
		
		while( lmove > 0) {
			if (patrolling_flag == 0)
			break;
		if (xcurr > l_limit) {	
				stepsNew(-stepss);
				xorgn--;
				if ((xorgn >= x_total) || (xorgn <= -x_total))
				xorgn = 0;
			}
		else if (xcurr < l_limit) {
				stepsNew(stepss);
				xorgn++;
				if ((xorgn >= x_total) || (xorgn <= -x_total))
				xorgn = 0;
			}
		lmove--;
		}
	HAL_Delay(2000);		
	}
	rmove = Abs(xorgn - r_limit);				//manhattan distance formula for right limit
	xcurr=xorgn;
	if ((rmove > 0) && (patrolling_flag == 2)){
		HAL_GPIO_WritePin(enab_tilt_GPIO_Port, enab_tilt_Pin, 1); 
		HAL_GPIO_WritePin(enab_pan_GPIO_Port, enab_pan_Pin, 0);
		
		if (xcurr > r_limit){
				
				setSpeed(30);
				stepsNew(-100); 
				
				setSpeed(60);		
				stepsNew(-100);
				
				setSpeed(90);		
				stepsNew(-100);
					
				xorgn = xorgn - 3;
				if (xorgn == -x_total) //Edited by Engr Tanzeel
				xorgn = 0;
				else if (xorgn < -x_total)
				xorgn=xorgn+x_total;
				
				rmove = rmove - 3;
				//setSpeed(compId5 + 0x14 + 50);
				}
		else if (xcurr < r_limit){
				
				setSpeed(30);
				stepsNew(100); 
				
				setSpeed(60);		
				stepsNew(100);
				
				setSpeed(90);		
				stepsNew(100);
						
				xorgn = xorgn + 3;
				if (xorgn == x_total) //Edited by Engr Tanzeel
				xorgn = 0;
				else if (xorgn > x_total)
				xorgn=xorgn-x_total;
				
				rmove = rmove - 3;
				//setSpeed(compId5 + 0x14 + 50);
				}
		
		while( rmove > 0) {
			if (patrolling_flag == 0)
			break;
		if (xcurr > r_limit) {	
				stepsNew(-stepss);
				xorgn--;
				if ((xorgn >= x_total) || (xorgn <= -x_total))
				xorgn = 0;
			}
		else if (xcurr < r_limit) {
				stepsNew(stepss);
				xorgn++;
				if ((xorgn >= x_total) || (xorgn <= -x_total))
				xorgn = 0;
			}
		rmove--;
		}
	HAL_Delay(2000);		
	}
		HAL_GPIO_WritePin(enab_tilt_GPIO_Port, enab_tilt_Pin, 1); 
		HAL_GPIO_WritePin(enab_pan_GPIO_Port, enab_pan_Pin, 1);
}

void Pan(void)
{
		HAL_GPIO_WritePin(enab_tilt_GPIO_Port, enab_tilt_Pin, 1);
		HAL_GPIO_WritePin(enab_pan_GPIO_Port, enab_pan_Pin, 0);  

		setSpeed(30);
		stepsNew(100); 
		
		setSpeed(60);		
    stepsNew(100);
		
		setSpeed(90);	
    stepsNew(100);
		
		xorgn = xorgn + 3;
		if (xorgn == x_total) //Edited by Engr Tanzeel
		xorgn = 0;
		else if (xorgn > x_total)
		xorgn=xorgn-x_total;		

		while (1)
			{
			if (rec_cmd == STOP_MOTORS)
			break;		
			stepsNew(stepss); 
			xorgn++;			
			if ((xorgn >= x_total) || (xorgn <= -x_total))	//Edited by Engr Tanzeel 
			xorgn = 0;
			}	
}

//will be called when SAVE preset or DELETE preset is pressed in GUI, rcvd via USART
void update_preset(int16_t x, int16_t y, int virtual_address_1, uint8_t sel){				//xorgn, yorgn, preset # (0-31), cam_sel(0 or 1)... xorgn yorgn existing position values
	unlocking_eeprom();			//to start writing to EEPROM
	if (sel == 1){
		int virtual_address = virtual_address_1 * 2;			//as one coordinate takes 16 bits... for xorgn of cam1
		int16_t xindex = x;					//will store complete xorgn, 16 bit
		uint8_t DATA_BYTE_L,DATA_BYTE_M;
		DATA_BYTE_L = xindex;		//LSB transferred
		*(uint8_t *)(DATA_E2_ADDR+virtual_address+1) = DATA_BYTE_L;			//LSB transferred into EEPROM address+1
		xindex = (xindex >> 8);			//right shifting, so LSB out MSB in right most bits
		DATA_BYTE_M = xindex;
		*(uint8_t *)(DATA_E2_ADDR+virtual_address) = DATA_BYTE_M;				//MSB transferred into EEPROM address
		//////////////////////////////////////////////////
		virtual_address = (virtual_address_1*2) + 64;			//as one coordinate takes 16 bits... for yorgn of cam1 thats why adding 64
		uint8_t y_DATA_BYTE_L,y_DATA_BYTE_M;
		int16_t yindex = y;
		y_DATA_BYTE_L = yindex;
		*(uint8_t *)(DATA_E2_ADDR+virtual_address+1) = y_DATA_BYTE_L;
		yindex = (yindex >> 8);
		y_DATA_BYTE_M = yindex;
		*(uint8_t *)(DATA_E2_ADDR+virtual_address) = y_DATA_BYTE_M;	
	}
	else if (sel == 2){
		int virtual_address = (virtual_address_1 * 2) + 128;		//as one coordinate takes 16 bits... for xorgn of cam2 thats why adding 128
		int16_t xindex = x;
		uint8_t DATA_BYTE_L,DATA_BYTE_M;
		DATA_BYTE_L = xindex;
		*(uint8_t *)(DATA_E2_ADDR+virtual_address+1) = DATA_BYTE_L;
		xindex = (xindex >> 8);
		DATA_BYTE_M = xindex;
		*(uint8_t *)(DATA_E2_ADDR+virtual_address) = DATA_BYTE_M;
		//////////////////////////////////////////////////
		virtual_address = (virtual_address_1*2) + 192;		//as one coordinate takes 16 bits... for yorgn of cam2 thats why adding 192
		uint8_t y_DATA_BYTE_L,y_DATA_BYTE_M;
		int16_t yindex = y;
		y_DATA_BYTE_L = yindex;
		*(uint8_t *)(DATA_E2_ADDR+virtual_address+1) = y_DATA_BYTE_L;
		yindex = (yindex >> 8);
		y_DATA_BYTE_M = yindex;
		*(uint8_t *)(DATA_E2_ADDR+virtual_address) = y_DATA_BYTE_M;	
	}
	locking_eeprom();
}

void update_limit(int16_t x, uint8_t sel, char limit){				//xorgn, cam_sel(0 or 1)... xorgn is the existing position values
	unlocking_eeprom();			//to start writing to EEPROM
	if (limit == 'l')
	{
	if (sel == 1){		//for left limit value of cam1
		int virtual_address = 256;			//as one coordinate takes 16 bits... for xorgn of cam1
		int16_t xindex = x;					//will store complete xorgn, 16 bit
		uint8_t DATA_BYTE_L,DATA_BYTE_M;
		DATA_BYTE_L = xindex;		//LSB transferred
		*(uint8_t *)(DATA_E2_ADDR+virtual_address+1) = DATA_BYTE_L;			//LSB transferred into EEPROM address+1
		xindex = (xindex >> 8);			//right shifting, so LSB out MSB in right most bits
		DATA_BYTE_M = xindex;
		*(uint8_t *)(DATA_E2_ADDR+virtual_address) = DATA_BYTE_M;				//MSB transferred into EEPROM address
	}
	else if (sel == 2){		//for left limit value of cam2
		int virtual_address = 260;	
		int16_t xindex = x;
		uint8_t DATA_BYTE_L,DATA_BYTE_M;
		DATA_BYTE_L = xindex;
		*(uint8_t *)(DATA_E2_ADDR+virtual_address+1) = DATA_BYTE_L;
		xindex = (xindex >> 8);
		DATA_BYTE_M = xindex;
		*(uint8_t *)(DATA_E2_ADDR+virtual_address) = DATA_BYTE_M;	
	}
	}
	////////////////////////////////////////////////////////////////
	else if(limit == 'r')
	{
	if (sel == 1){		//for right limit value of cam1
		int virtual_address = 258;			
		int16_t xindex = x;					//will store complete xorgn, 16 bit
		uint8_t DATA_BYTE_L,DATA_BYTE_M;
		DATA_BYTE_L = xindex;		//LSB transferred
		*(uint8_t *)(DATA_E2_ADDR+virtual_address+1) = DATA_BYTE_L;			//LSB transferred into EEPROM address+1
		xindex = (xindex >> 8);			//right shifting, so LSB out MSB in right most bits
		DATA_BYTE_M = xindex;
		*(uint8_t *)(DATA_E2_ADDR+virtual_address) = DATA_BYTE_M;				//MSB transferred into EEPROM address
	}
	else if (sel == 2){		//for right limit value of cam2
		int virtual_address = 262;	
		int16_t xindex = x;
		uint8_t DATA_BYTE_L,DATA_BYTE_M;
		DATA_BYTE_L = xindex;
		*(uint8_t *)(DATA_E2_ADDR+virtual_address+1) = DATA_BYTE_L;
		xindex = (xindex >> 8);
		DATA_BYTE_M = xindex;
		*(uint8_t *)(DATA_E2_ADDR+virtual_address) = DATA_BYTE_M;	
		}
	}
	locking_eeprom();
}

void reset_presets(void){					//for developer ONLY, if EEPROM is to be cleared. it is not called anywhere in code or in GUI. call it in main, before while (1) IF REQUIRED. it must not be called in normal working!!!
	unlocking_eeprom();
	for(int k = 0; k < 512; k++){
		*(uint8_t *)(DATA_E2_ADDR+k) = 0;	
	}
	locking_eeprom();
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
