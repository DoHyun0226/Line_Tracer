/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/*
 * [프로그램 개요] 버튼(PC13)을 누르면 스텝모터를 STEPS_PER_PRESS 칸 돌린다.
 *
 * 하드웨어
 *  - 모터: 28BYJ-48 / 드라이버: ULN2003
 *  - IN1=PA8, IN2=PA9, IN3=PA10, IN4=PB5 (Output Push Pull)
 *  - 버튼: PC13 (EXTI13, Rising edge 인터럽트)
 *
 * 동작 순서
 *  1) 버튼을 누르면 Rising edge가 감지되어 인터럽트 발생
 *  2) EXTI15_10_IRQHandler -> HAL_GPIO_EXTI_IRQHandler -> HAL_GPIO_EXTI_Callback 순서로 호출
 *  3) 콜백은 채터링을 걸러 낸 뒤 step_request만 1로 만들고 바로 끝남
 *  4) main의 while문이 step_request를 보고 모터를 돌림
 *
 * 모터를 콜백에서 돌리지 않는 이유
 *  - 인터럽트 안에서는 오래 머물면 안 된다.
 *  - HAL_Delay()는 SysTick 인터럽트로 시간을 재는데, 이 EXTI는 우선순위가 0이고
 *    SysTick은 15라서 콜백 안에서는 시간이 흐르지 않아 멈출 수 있다.
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* 버튼 한 번에 돌릴 칸 수.
 * 1칸은 약 0.18도라서 1이면 축이 도는 것이 거의 보이지 않는다.
 * 눈으로 확인하려면 256(약 45도)으로 바꾼다. (약 2048칸 = 1바퀴) */
#define STEPS_PER_PRESS 2048

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* 코일 구동 패턴 (2상 여자 풀스텝). bit0~3 = IN1~IN4, 1이면 해당 코일 ON.
 * 항상 코일 2개를 켜고, 표를 순서대로 지나가면 모터가 한 방향으로 돈다.
 *   0x3 = 0011 : IN1, IN2
 *   0x6 = 0110 : IN2, IN3
 *   0xC = 1100 : IN3, IN4
 *   0x9 = 1001 : IN4, IN1 */
static const uint8_t step_table[4] = {0x3, 0x6, 0xC, 0x9};

/* step_table에서 현재 출력 중인 위치(0~3). 다음 칸은 여기서 이어서 진행한다. */
static uint8_t step_index = 0;

/* 모터를 돌려 달라는 요청 표시. 콜백이 1로 만들고 main이 확인 후 0으로 지운다.
 * 인터럽트와 main이 함께 쓰는 변수라서, 컴파일러가 읽기를 생략하지 않도록 volatile을 붙인다. */
volatile uint8_t step_request = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void Motor_Write(uint8_t pattern);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (step_request)
    {
        step_request = 0;   // 요청을 지워서 버튼 한 번에 한 번만 실행

        for (int i = 0; i < STEPS_PER_PRESS; i++)
        {
            step_index = (step_index + 1) & 0x3;   // 0->1->2->3->0 순환 (% 4와 같음)
            Motor_Write(step_table[step_index]);   // 다음 패턴을 출력해서 한 칸 이동
            HAL_Delay(10);                          // 모터가 따라올 시간. 너무 짧으면 돌지 못하고 떤다
        }
    }
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV6;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}


/* USER CODE BEGIN 4 */

/* 패턴의 각 비트를 IN1~IN4 핀에 출력한다.
 * pattern: bit0~3 = IN1~IN4. 예) 0x3이면 IN1, IN2 High / IN3, IN4 Low. 0이면 코일 전부 OFF */
static void Motor_Write(uint8_t pattern)
{
    HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, (pattern & 0x1) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, (pattern & 0x2) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, (pattern & 0x4) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, (pattern & 0x8) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* EXTI 인터럽트마다 HAL이 호출하는 함수.
 * HAL에 있는 같은 이름의 __weak 빈 함수를 이 함수가 대신한다. 이름이 틀리면 호출되지 않는다.
 * GPIO_Pin: 인터럽트가 발생한 핀 번호 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    /* 마지막으로 입력을 인정한 시각(ms). static이라 함수가 끝나도 값이 유지된다. */
    static uint32_t last_tick = 0;

    /* 채터링 방지: 버튼 접점이 떨리면 한 번 눌러도 인터럽트가 여러 번 들어오므로,
     * 직전 입력 후 50 ms가 지나지 않은 입력은 무시한다. */
    if (GPIO_Pin == GPIO_PIN_13 && HAL_GetTick() - last_tick > 50)
    {
        last_tick = HAL_GetTick();
        step_request = 1;   // 실제 모터 구동은 main의 while문이 한다
    }
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
#ifdef USE_FULL_ASSERT
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
