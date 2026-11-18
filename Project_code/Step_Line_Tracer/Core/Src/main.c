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
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stepper.h"
#include "tcrt5000.h"
#include "pid.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* ===== 스텝모터 테스트 (DRV8825 + KH42JM2-901) =====
 * 1 = while(1)에서 라인트레이서 대신 아래 모터 테스트를 실행.
 * 전진 가속 -> 유지 -> 감속 -> 쉼 -> 후진도 똑같이, 무한 반복. */
#define MOTOR_TEST          1
#define TEST_HZ             (1.0f * STEPPER_STEPS_PER_REV)   // 초당 1회전
#define TEST_ACCEL_HZ       (10.0f * STEPPER_MICROSTEP)      // 10 ms마다 더하는 Hz (1/8 모드: 80 Hz = 8000 Hz/s = 초당 5바퀴씩 가속)
#define TEST_HOLD_MS        2000                      // 최고 속도 유지 시간
#define TEST_PAUSE_MS       1000                      // 방향 바꾸기 전 쉬는 시간

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
static void Motor_Test_Run(float target_hz);

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


  /* =====   PID   ===== */
  /* PID 계수, 출력 제한, 내부 상태(gain_p/i/d, err_prev)는 pid.h / pid.c에 있음 */
  float err_curr; 						// 현재 오차 (라인 위치, -7 ~ +7, 0 = 가운데) -> PID 입력
  uint32_t dt; 							// 주기 간격 (ms)
  uint32_t last_tick;					// 지난 주기의 HAL_GetTick() 값 (HAL_GetTick()과 같은 uint32_t)
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
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  tcrt5000_init();
  Stepper_Init();                                                  // TIM1, TIM3 PWM 시작 (CCR=0이라 아직 펄스 없음)
#if MOTOR_TEST
  Stepper_Enable(1);                                               // EN LOW -> 코일에 전류 (테스트 모드에서만 바로 켬)
#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
#if MOTOR_TEST
    Motor_Test_Run(+TEST_HZ);      // 전진
    HAL_Delay(TEST_PAUSE_MS);
    Motor_Test_Run(-TEST_HZ);      // 후진
    HAL_Delay(TEST_PAUSE_MS);
#endif
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
/**
  * @brief  두 모터를 0 -> target_hz로 가속, 유지, 다시 0으로 감속.
  *         스텝모터는 너무 갑자기 출발하거나 멈추면 스텝을 놓치거나(탈조)
  *         웅웅거리기만 하므로, 10 ms마다 조금씩 속도를 바꾼다.
  * @param  target_hz  최고 속도 (+ 전진 / - 후진)
  */
static void Motor_Test_Run(float target_hz)
{
  float step = (target_hz > 0.0f) ? TEST_ACCEL_HZ : -TEST_ACCEL_HZ;   // 한 번에 바꿀 양 (방향 포함)
  float hz = 0.0f;

  while (hz != target_hz)                         // 가속 (넘으면 아래에서 target_hz로 딱 맞추므로 != 비교가 끝남)
  {
    hz += step;
    if ((step > 0.0f && hz > target_hz) || (step < 0.0f && hz < target_hz)) hz = target_hz;   // 목표 넘지 않게
    Stepper_SetSpeed(STEPPER_LEFT,  hz);
    Stepper_SetSpeed(STEPPER_RIGHT, hz);
    HAL_Delay(10);
  }

  HAL_Delay(TEST_HOLD_MS);                        // 일정 속도 유지

  while (hz != 0.0f)                              // 감속
  {
    hz -= step;
    if ((step > 0.0f && hz < 0.0f) || (step < 0.0f && hz > 0.0f)) hz = 0.0f;   // 0 넘지 않게
    Stepper_SetSpeed(STEPPER_LEFT,  hz);
    Stepper_SetSpeed(STEPPER_RIGHT, hz);
    HAL_Delay(10);
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
