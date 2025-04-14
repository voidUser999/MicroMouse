/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "main.h"
#include "app_bluenrg_2.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include<stdio.h>
#include<string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
uint32_t raw;
char msg[20];
char msg2[15];
const float threshold = 2.3;
const float warn = 2.5;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
void ihm12a1_stspin240_init(void);
void enable_dualbridge(void);
void motordata(uint8_t *data);
void battery_indicator(void);
//void battery_led_blink(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t counter_1 = 0; //encoder 1 pulse counter
int16_t count_1  = 0;
uint32_t counter_2 = 0; // encoder 2 pulse counter
int16_t  count_2  = 0;


int st_line1 = 0, st_line2 = 0, cw90 = 1;
uint32_t previousMillis = 0;
uint32_t currentMillis = 0;
uint32_t counterOutside = 0;
uint32_t counterInside = 0;
volatile int state =0;
volatile int count=0;
int flag = 0;
uint32_t buffer;
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	raw=buffer;
//	adc_val=HAL_ADC_GetValue(&hadc1);
}

//void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
//{
//  /* Prevent unused argument(s) compilation warning */
//	UNUSED(GPIO_Pin);
//if(GPIO_Pin == GPIO_PIN_1)
//{
//	if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_1) != 0x00u)
//	   {
//	     __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_1); // Clears The Interrupt Flag
//	     HAL_GPIO_EXTI_Rising_Callback(GPIO_PIN_1);   // Calls The ISR Handler CallBack Function
//	   }
//	if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3)==GPIO_PIN_SET)
//	{
//	count++;
//	}
//	else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3)==GPIO_PIN_RESET)
//	{
//		count--;
//		}
//
//	printf("count = %d\r\n", count);
//}
//}
//void EXTI1_IRQHandler(void) {
//   /* Make sure that interrupt flag is set */
//   if (EXTI_GetITStatus(EXTI_LINE_1) != RESET)
//   {
//      /* Do your stuff when EXTI0 is changed */
//	   if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3)==GPIO_PIN_SET)
//	   	{
//	   		flag = 1;
//	   	count++;
//	   	}
//	   	else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3)==GPIO_PIN_RESET)
//	   	{
//	   		flag = 2;
//	   		count--;
//	   		}
//	   printf("count = %d\r\n", count);
//      /* Clear interrupt flag */
//      EXTI_ClearITPendingBit(EXTI_LINE_1);
//   }
//   }
   /* Handle IRQ10-15 interrupt */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */

#define  ROWS 10
#define COLS 10
/* defining the cells of each point
 */
typedef struct {
	uint8_t visited;
	uint8_t walls[4];
}MazeCell;
#define MAZE_ROWS 5
#define MAZE_COLS 5
typedef enum {
    NORTH = 0,
    EAST  = 1,
    SOUTH = 2,
    WEST  = 3
} Direction;
//relative direction to absolute direction
uint8_t leftOf(uint8_t dir)  { return (dir + 3) % 4; }
uint8_t rightOf(uint8_t dir) { return (dir + 1) % 4; }
uint8_t behind(uint8_t dir)  { return (dir + 2) % 4; }

int x =0;//this is only for the simulation purposes
  int y =0;//this is only for the simulation purposes
  uint8_t dir = SOUTH;
  MazeCell maze[MAZE_ROWS][MAZE_COLS] = {
      // Row 0
      {
          {0, {1, 0, 1, 1}}, {0, {1, 1, 0, 0}}, {0, {1, 0, 1, 1}}, {0, {1, 1, 1, 0}}, {0, {1, 1, 0, 1}}
      },
      // Row 1
      {
          {0, {1, 1, 0, 1}}, {0, {0, 1, 1, 1}}, {0, {1, 0, 0, 1}}, {0, {1, 1, 0, 0}}, {0, {0, 1, 1, 1}}
      },
      // Row 2
      {
          {0, {0, 0, 1, 1}}, {0, {1, 1, 0, 0}}, {0, {0, 1, 1, 1}}, {0, {0, 0, 1, 1}}, {0, {1, 1, 0, 0}}
      },
      // Row 3
      {
          {0, {1, 1, 1, 1}}, {0, {0, 0, 0, 1}}, {0, {1, 1, 0, 0}}, {0, {1, 0, 1, 1}}, {0, {0, 1, 0, 0}}
      },
      // Row 4
      {
          {0, {1, 1, 1, 1}}, {0, {0, 1, 1, 1}}, {0, {0, 0, 1, 1}}, {0, {1, 1, 1, 0}}, {0, {0, 1, 1, 1}}
      }
  };


int dx[4] = {0, 1, 0, -1};  //movement in x direction
int dy[4] = {-1, 0, 1, 0};  //movement in y direction

MazeCell discoveredMaze[MAZE_ROWS][MAZE_COLS];
uint8_t isValid(int x, int y) {
    return x >= 0 && x < MAZE_COLS && y >= 0 && y < MAZE_ROWS;
}
void storeWallData(int x, int y, uint8_t dir, uint8_t frontWall, uint8_t leftWall, uint8_t rightWall){
	MazeCell* cell = &discoveredMaze[y][x];
	cell->visited = 1;

	//finding absolute positions
	uint8_t absFront = dir;
	uint8_t absLeft = leftOf(dir);
	uint8_t absRight = rightOf(dir);

	cell->walls[absFront] = frontWall;
	cell->walls[absLeft]  = leftWall;
	cell->walls[absRight] = rightWall;

	// Update neighbor cells to be symmetric (opposite side)
	    if (frontWall == 1 && isValid(x + dx[absFront], y + dy[absFront])) {
	        discoveredMaze[y + dy[absFront]][x + dx[absFront]].walls[behind(absFront)] = 1;
	    }
	    if (leftWall == 1 && isValid(x + dx[absLeft], y + dy[absLeft])) {
	        discoveredMaze[y + dy[absLeft]][x + dx[absLeft]].walls[behind(absLeft)] = 1;
	    }
	    if (rightWall == 1 && isValid(x + dx[absRight], y + dy[absRight])) {
	        discoveredMaze[y + dy[absRight]][x + dx[absRight]].walls[behind(absRight)] = 1;
	    }

}
void getSimulatedSensorData(int x, int y, uint8_t dir) {
    if (!isValid(x, y)) return;

    // Get reference to simulated cell
    MazeCell *simCell = &maze[y][x];

    // Convert relative directions to absolute
    uint8_t absFront = dir;
    uint8_t absLeft  = leftOf(dir);
    uint8_t absRight = rightOf(dir);

    // Get wall data from simulated maze
    uint8_t frontWall = simCell->walls[absFront];
    uint8_t leftWall  = simCell->walls[absLeft];
    uint8_t rightWall = simCell->walls[absRight];

    // Store the retrieved wall data into discovered maze
    storeWallData(x, y, dir, frontWall, leftWall, rightWall);
}




typedef struct {
    int x, y;
} Point;

typedef struct {
    Point point;
    Direction fromDir;
} QueueNode;

Direction bfs_find_next_move(int startX, int startY) {
    uint8_t visited[MAZE_ROWS][MAZE_COLS] = {0};
    QueueNode queue[MAZE_ROWS * MAZE_COLS];
    int front = 0, rear = 0;

    Point parent[MAZE_ROWS][MAZE_COLS];
    for (int y = 0; y < MAZE_ROWS; y++)
        for (int x = 0; x < MAZE_COLS; x++)
            parent[y][x] = (Point){-1, -1};

    queue[rear++] = (QueueNode){.point = {startX, startY}};
    visited[startY][startX] = 1;

    Point goal = {-1, -1};

    // Direction priority: South, East, North, West (Priority order)
    int bfsPriority[4] = {SOUTH, EAST, NORTH, WEST};

    while (front < rear) {
        QueueNode node = queue[front++];
        int cx = node.point.x;
        int cy = node.point.y;

        // If the goal is reached (last cell in maze)
        if (cx == MAZE_COLS - 1 && cy == MAZE_ROWS - 1) {
            goal = (Point){cx, cy};
            break;
        }

        // Check directions based on BFS priority order: South -> East -> North -> West
        for (int i = 0; i < 4; i++) {
            int d = bfsPriority[i];
            int nx = cx + dx[d];
            int ny = cy + dy[d];

            // Only move if the cell is valid and not visited, and there is no wall in the direction
            if (isValid(nx, ny) && !visited[ny][nx] &&
                discoveredMaze[cy][cx].walls[d] == 0) {
                visited[ny][nx] = 1;
                parent[ny][nx] = (Point){cx, cy};
                queue[rear++] = (QueueNode){.point = {nx, ny}};
            }
        }
    }

    // Trace back from goal to start to get the first move
    if (goal.x == -1) return SOUTH; // No path found, default to SOUTH

    int tx = goal.x;
    int ty = goal.y;

    while (!(parent[ty][tx].x == startX && parent[ty][tx].y == startY)) {
        Point p = parent[ty][tx];
        tx = p.x;
        ty = p.y;
    }

    int dxMove = tx - startX;
    int dyMove = ty - startY;

    // Find the direction to move based on dxMove and dyMove
    for (int d = 0; d < 4; d++) {
        if (dx[d] == dxMove && dy[d] == dyMove) {
            return (Direction)d;
        }
    }

    return SOUTH; // Default direction if no valid movement found
}
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
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_BlueNRG_2_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  ihm12a1_stspin240_init(); // initializes XNucleo-IHM12A1 board
  enable_dualbridge();
//  MX_BlueNRG_2_Init();
  HAL_ADC_Start_DMA(&hadc1,&buffer,1);

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (x != MAZE_COLS - 1 || y != MAZE_ROWS - 1) {
        getSimulatedSensorData(x, y, dir); // updates discovered maze

        Direction nextDir = bfs_find_next_move(x, y);

        uint8_t moveCmd = 'S';
        if (nextDir == dir) {
            moveCmd = 'F';
            motordata(&moveCmd);
        } else if (nextDir == leftOf(dir)) {
            moveCmd = 'L';
            motordata(&moveCmd);
            HAL_Delay(430);
            moveCmd = 'F';
            motordata(&moveCmd);
        } else if (nextDir == rightOf(dir)) {
        	moveCmd = 'R';
        	motordata(&moveCmd);
        	HAL_Delay(430);
        	moveCmd = 'F';
        	motordata(&moveCmd);
        } else if (nextDir == behind(dir)) {
        	moveCmd = 'L';
        	motordata(&moveCmd);
            HAL_Delay(430);
            motordata(&moveCmd);  // another 'L'
            moveCmd = 'S';
            motordata(&moveCmd);
            HAL_Delay(430);
            moveCmd = 'L';
            motordata(&moveCmd);
            HAL_Delay(430);
            moveCmd = 'F';
            motordata(&moveCmd);
            HAL_Delay(430);  // or handle as two turns
            moveCmd = 'S';
            motordata(&moveCmd);
        }

        // Move

        HAL_Delay(430);
        moveCmd = 'S';
        motordata(&moveCmd);


        // Update position **based on nextDir**
        x = x + dx[nextDir];
        y = y + dy[nextDir];
        dir = nextDir;
        HAL_Delay(1000);

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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* EXTI0_1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);
  /* EXTI2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);
  /* EXTI4_15_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 15;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 99;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED4_Pin|LED3_Pin|BLNRG_RST_Pin|RST_Pin
                          |LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SPI_CS_Pin|PHB_Pin|REF_Pin|EN_IHM_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PHA_GPIO_Port, PHA_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED4_Pin LED3_Pin BLNRG_RST_Pin RST_Pin
                           LED2_Pin */
  GPIO_InitStruct.Pin = LED4_Pin|LED3_Pin|BLNRG_RST_Pin|RST_Pin
                          |LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : INT_ENCODER1_Pin INT_ENCODER2_Pin */
  GPIO_InitStruct.Pin = INT_ENCODER1_Pin|INT_ENCODER2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI_BNRG_EXTI_Pin */
  GPIO_InitStruct.Pin = SPI_BNRG_EXTI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SPI_BNRG_EXTI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI_CS_Pin PHB_Pin REF_Pin EN_IHM_Pin */
  GPIO_InitStruct.Pin = SPI_CS_Pin|PHB_Pin|REF_Pin|EN_IHM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PHA_Pin */
  GPIO_InitStruct.Pin = PHA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PHA_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED5_Pin */
  GPIO_InitStruct.Pin = LED5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED5_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ENCODER1_DATA_Pin ENCODER2_DATA_Pin */
  GPIO_InitStruct.Pin = ENCODER1_DATA_Pin|ENCODER2_DATA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	counterOutside++;
	  currentMillis = HAL_GetTick();
	  if (GPIO_Pin == GPIO_PIN_13 && (currentMillis - previousMillis > 20))
	  {
	    counterInside++;
	   state=!state;
	    previousMillis = currentMillis;
	  }
}
void enable_dualbridge(void)
{
  TIM3->CCR1 = 0; //pwma
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); //timer 15 channel 1
  TIM3->CCR2 = 0; //pwmb
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);//timer 15 channel 2
}
void ihm12a1_stspin240_init(void)
{
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET); //sys_rst high
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);   //dir2 high
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);  //dir1 high
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);   //ref high = 100% Duty cycle
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
//  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
//  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
}
void motordata(uint8_t *data)
{
	 switch(data[0])
	 {
	            /*F: to move forward*/
	 case 'F':
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);//DIR_1
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);//DIR_2
		TIM3->CCR2 = 100;
		TIM3->CCR1 = 100;
		HAL_Delay(25);
		TIM3->CCR2 = 90;
		TIM3->CCR1 = 90;
		HAL_Delay(15);
		TIM3->CCR2 = 100;
		TIM3->CCR1 = 100;
		break;
	/*L: to turn left*/
	case 'L':
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);//DIR_2
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);//DIR_1
		TIM3->CCR2 = 100;
		TIM3->CCR1 = 100;
		break;
	/*R: to turn right*/
	case 'R':
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);//DIR_1
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);//DIR_2
		TIM3->CCR2 = 100;
		TIM3->CCR1 = 100;
		break;
	/*B: to move backward*/
	case 'B':
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);//DIR_2
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);//DIR_1
		TIM3->CCR2 = 100;
		TIM3->CCR1 = 100;
		HAL_Delay(25);
		TIM3->CCR2 = 90;
		TIM3->CCR1 = 90;
		HAL_Delay(15);
		TIM3->CCR2 = 100;
		TIM3->CCR1 = 100;
		break;
	/*Q: to move anti-clockwise until another button pressed*/
	case 'Q':
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);//DIR_1
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);//DIR_2
		TIM3->CCR2 = 60;
		TIM3->CCR1 = 25;
		break;
	/*C: to move clockwise until another button pressed*/
	case 'C':
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);//DIR_1
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);//DIR_2
		TIM3->CCR2 = 60;
		TIM3->CCR1 = 25;
		break;
	case 'S':
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);//DIR_1
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);//DIR_2
		TIM3->CCR2 = 0;
		TIM3->CCR1 = 0;
		break;
	default:
		TIM3->CCR2 = 0;
		TIM3->CCR1 = 0;
		break;
	}
}

//void battery_indicator(void){
////	 HAL_ADC_Start(&hadc1);
////		  HAL_ADC_PollForConversion(&hadc1, 300);
////		  raw=HAL_ADC_GetValue(&hadc1);
//	 float vin=raw*(3.3/4096);
//		  sprintf(msg2,"vol=%.2f\r\n",vin);
//		  HAL_UART_Transmit(&huart2,(uint8_t*)msg2, strlen(msg2),300);
////		  HAL_Delay(2000);
//		  if(vin<threshold){
//			  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET );
//			  HAL_Delay(1000);
//			  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET );
//			  HAL_Delay(500);
//		  }
//		  if(vin>threshold && vin<warn){
//			  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET );
//		  	  }
//
//}
//void battery_led_blink(void){
//	float vin=raw*(3.3/4096);
//	 if(vin<threshold){
//				  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET );
//	//			  HAL_Delay(100);
//	//			  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET );
//	//			 		  HAL_Delay(100);
//			  }
//			  if(vin>threshold && vin<warn){
//			  		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET );
//			  		  HAL_Delay(2000);
//			  		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET );
//			  		  		  HAL_Delay(2000);
//			  	  }
//}

//PUTCHAR_PROTOTYPE
//{
//  /* Place your implementation of fputc here */
//  /* e.g. write a character to the USART1 and Loop until the end of transmission */
//  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
//
//  return ch;
//}

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
