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
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*
 * TIM1 카운터가 1초에 몇 번 세는지(틱 주파수).
 * 타이머 클록 170MHz ÷ (PSC 169 + 1) = 1MHz → 1틱 = 1µs
 * PSC를 바꾸면 이 값도 같이 바꿔야 한다.
 */
#define STEP_TICK_HZ  1000000UL     // PSC 169 → 1MHz

/*
 * [목적] STEP 펄스 주파수(= 모터 속도)를 바꾼다. 1Hz = 1초에 1스텝.
 *
 * [작동 방식]
 *  - TIM1 CH1은 PWM1 모드: CNT < CCR 동안 High, 나머지는 Low.
 *  - PWM 한 주기 = (ARR + 1)틱 → 주파수 = 1MHz / (ARR + 1)
 *    예) 600Hz → ARR = 1,000,000 / 600 - 1 = 1665
 *  - CCR = (ARR + 1) / 2 → 듀티 50%. 한 주기마다 STEP 상승 에지가
 *    1번 생기고, DRV8825는 상승 에지마다 1스텝 움직인다.
 *  - hz = 0 이면 CCR = 0 → 출력이 항상 Low → 펄스가 없어서 정지.
 *    (타이머는 계속 돌고, 출력만 Low로 고정)
 *  - ARR preload Enable이라 새 ARR/CCR은 "현재 주기가 끝날 때"
 *    적용된다. 그래서 주기 중간에 바꿔도 펄스가 깨지지 않는다.
 *    (preload가 없으면 CNT가 새 ARR보다 이미 큰 경우 65535까지
 *     세고 넘어가서 약 65ms 동안 펄스가 끊길 수 있다)
 *
 * [범위] TIM1은 16비트라 ARR ≤ 65535 → 최소 약 16Hz.
 *        16Hz 미만은 16Hz로 올려서 처리한다.
 */
void step_set_freq(uint32_t hz)
{
  if (hz == 0) {                    // 정지: 듀티 0% → STEP Low 유지
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    return;
  }
  if (hz < 16) hz = 16;             // ARR 16비트 한계
  uint32_t arr = STEP_TICK_HZ / hz - 1;
  __HAL_TIM_SET_AUTORELOAD(&htim1, arr);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (arr + 1) / 2);
}

/*
 * [목적] 모터 회전 방향을 정한다.
 *
 * [작동 방식]
 *  - PB5(→ DRV8825 DIR)를 forward = 1이면 High, 0이면 Low로 출력.
 *  - DRV8825는 STEP 상승 에지 순간의 DIR 값을 보고 방향을 정한다.
 *  - 실제로 어느 쪽이 "정방향"인지는 모터 코일 배선에 따라 다르다.
 *    (A1/A2 한 쌍을 서로 바꾸면 방향이 반대가 된다)
 *
 * [주의] 돌고 있는 중에 방향을 바꾸면 회전자 관성 때문에 탈조한다.
 *        반드시 감속 → 정지 → 방향 변경 → 다시 가속 순서로 쓴다.
 */
void step_set_dir(int forward)
{
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, forward ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/*
 * [목적] 속도를 from Hz → to Hz로 ms 밀리초 동안 서서히 바꾼다 (가속/감속).
 *        스텝모터는 정지 상태에서 갑자기 빠른 속도를 주면 회전자가
 *        따라가지 못해 탈조하므로, 천천히 올리고 내려야 한다.
 *
 * [작동 방식]
 *  - 1ms마다 직선 보간으로 주파수를 계산해서 step_set_freq()로 적용.
 *    freq = from + (to - from) × t / ms   (t = 0 ~ ms)
 *    예) step_ramp(100, 600, 500): 1ms마다 1Hz씩 올라 0.5초 뒤 600Hz
 *  - 감속(to < from)일 때 (to - from)이 음수가 되도록 int32_t로 계산.
 *  - HAL_Delay(1)로 기다리는 "블로킹" 함수: 끝날 때까지(약 ms + 1 ms)
 *    다른 일을 하지 못한다. 테스트용으로만 쓰고, 라인트레이서에서는
 *    타이머 인터럽트 등 논블로킹 방식으로 바꿔야 한다.
 *
 * [주의] ms = 0이면 0으로 나누게 되므로 ms는 1 이상으로 넣는다.
 */
void step_ramp(uint32_t from, uint32_t to, uint32_t ms)   // 블로킹 가감속
{
  for (uint32_t t = 0; t <= ms; t++) {
    step_set_freq(from + (int32_t)(to - from) * (int32_t)t / (int32_t)ms);
    HAL_Delay(1);
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
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  step_set_freq(0);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  int dir = 1;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    step_set_dir(dir);
    HAL_Delay(5);                    // 방향은 멈춘 상태에서 바꾸기
    step_ramp(100, 600, 500);        // 0.5초 동안 100 → 600Hz 가속
    HAL_Delay(2000);                 // 600Hz 유지
    step_ramp(600, 100, 500);        // 감속
    step_set_freq(0);                // 정지
    HAL_Delay(1000);
    dir = !dir;
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
}

/* USER CODE BEGIN 4 */

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
