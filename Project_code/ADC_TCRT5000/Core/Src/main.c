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
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>	// snprintf 선언 (HAL이 아니라 C 표준 라이브러리)
					// 구현은 툴체인의 newlib-nano(libc_nano.a)에 있음
					// nano 버전은 기본 설정에서 %f(실수) 출력이 꺼져 있음 → 정수만 사용
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
 * [회로] TCRT5000 x2 (센서 하나당 아래 회로 하나씩)
 *
 *   IR LED (Top view 오른쪽, A/C)
 *     5V(USB) -> 200Ω -> A(Anode) ... C(Cathode) -> GND
 *     I = (5 - VF) / R = (5 - 1.25) / 200 ≈ 19mA   (최대 60mA)
 *     - VF 1.25V는 "60mA일 때 LED 전압"이라는 측정 조건.
 *       LED는 전류가 바뀌어도 전압이 거의 일정(약 1.1~1.25V) → 전류는 저항이 정함
 *     - 3.3V 핀을 쓰면 100Ω (약 21mA)
 *
 *   포토트랜지스터 (Top view 왼쪽, C/E)
 *     3.3V -> 10kΩ -> C(Collector) -> PA0(센서1) / PA1(센서2)
 *                     E(Emitter)   -> GND
 *     - 풀업은 반드시 3.3V (ADC 입력이 3.3V를 넘으면 안 됨)
 *     - Vout = 3.3V - Ic x R  : R이 클수록 전류 변화에 민감 (10kΩ이면 1uA당 10mV)
 *       너무 크면 금방 0V에 붙어(포화) 흰/검 구분이 안 됨
 *     - 흰색: 반사 많음 -> Ic 큼 -> Vout 낮음 -> ADC 값 작음
 *       검은색: 반사 적음 -> Ic 작음 -> Vout 높음 -> ADC 값 큼
 *     - VCEO 70V / VECO 5V : C,E를 거꾸로 꽂아도 3.3V라 안 망가지지만 동작 안 함
 *
 *   GND로 가는 핀: LED Cathode, 포토TR Emitter (둘 다 같은 GND에 묶어도 됨)
 *   주의: 그림의 "C"가 두 개 → 왼쪽 위 C = Collector, 오른쪽 아래 C = Cathode
 *
 *   NUCLEO 5V 핀(USB 전원): 보드 전체 300mA 이하 권장(UM2505 p.17), LED 2개 ≈ 38mA
 *
 * [CubeMX 설정]
 *   ADC1 IN1(PA0), IN2(PA1) : Single-ended
 *     - Single-ended: INP 핀만 사용, INN은 칩 안에서 VREF-(=VSSA=GND)에 연결됨
 *       → (입력 핀 - GND) 측정, 0~3.3V -> 0~4095 (12bit, 1칸 ≈ 0.8mV)
 *     - Differential(차동): (INP - INN) 측정, 두 핀 모두 0~3.3V 안이어야 함
 *   Clock Prescaler: Synchronous /4 → 170MHz / 4 = 42.5MHz (ADC 최대 약 60MHz)
 *   Scan + Number Of Conversion 2 : Rank1 = Ch1, Rank2 = Ch2
 *   Continuous Conversion Mode : Enabled  → 한 바퀴(IN1->IN2) 끝나면 자동 반복
 *     (Single conversion mode = 시퀀스를 한 바퀴만 돌고 멈춤. Single-ended와 다른 설정)
 *   DMA Continuous Requests : Enabled     → DMA 요청을 멈추지 않고 계속 보냄
 *   Sampling Time 640.5 Cycles
 *     - 10kΩ처럼 출력 임피던스가 큰 센서는 샘플링이 길수록 정확
 *     - 변환 1개 ≈ (640.5 + 12.5) / 42.5MHz ≈ 15us
 *       → DMA 인터럽트(절반/전체 완료) 횟수가 줄어 CPU 부담 감소
 *   Oversampling : Disable (값이 들쭉날쭉하면 Ratio 16, Shift 4 = 16번 평균)
 *
 *   DMA: ADC1 → DMA1 Channel1, Peripheral To Memory, Circular
 *     - Data Width: Peripheral/Memory 둘 다 Half Word(16bit) → 배열도 uint16_t
 *     - Increment: Peripheral X (항상 ADC_DR 하나에서 읽음), Memory O (buf[0] -> buf[1])
 *     - Normal  : 정해진 개수(2개)를 옮기면 멈춤
 *       Circular: 다 옮기면 buf[0]부터 다시 → Continuous ADC와 짝
 *     - DMA는 자체 주기가 없음. ADC 변환 1개가 끝날 때마다 요청 → 1개 옮김
 *       (주기를 정하려면 ADC를 타이머 트리거로 시작)
 *
 * [동작]
 *   ADC는 ADC 하나당 데이터 레지스터(DR)가 하나뿐 (Regular 그룹)
 *     IN1 변환 -> DR -> DMA가 buf[0]으로
 *     IN2 변환 -> DR(덮어씀) -> DMA가 buf[1]로
 *     IN1 ... 반복 (CPU 개입 없음)
 *   DR을 제때 안 꺼내면 다음 값이 덮어씀 = Overrun → 그래서 DMA 사용
 *   (Injected 그룹은 JDR1~4가 따로 있음. 급한 측정용 끼어들기, 이번엔 사용 X)
 */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	// ADC 결과 저장 배열: buf[0] = 센서1(PA0), buf[1] = 센서2(PA1)
	// - uint16_t: 12bit 값(0~4095) + DMA Data Width가 Half Word
	//   (uint32_t로 잡으면 한 칸에 두 센서 값이 같이 들어가 버림)
	// - volatile: CPU 모르게 DMA가 값을 바꿈. 안 붙이면 컴파일러가
	//   "안 바뀌네" 하고 옛날 값을 재사용할 수 있음
	// - DMA가 쓰는 중에 읽어도 값 하나가 깨지진 않음(16bit를 한 번에 씀).
	//   다만 buf[0]과 buf[1]이 몇 us 차이 나는 다른 바퀴 값일 수 있음 → 센서에선 무시 가능
	volatile uint16_t buf[2];

	// UART로 보낼 문자열 공간 (반드시 배열! char *만 선언하면 쓸 공간이 없음)
	// - 가장 긴 경우 "4095 4095\r\n" = 4 + 1 + 4 + 2 = 11글자 + '\0' 1칸 = 12칸
	// - 참고: char a[] = "Hi";  → RAM에 복사, 수정 가능
	//         char *p = "Hi";   → Flash의 리터럴을 가리킴, 수정 불가(const char * 권장)
	char uart_buf[12];

	// 실제로 보낼 글자 수 (snprintf 반환값)
	// - sizeof(uart_buf) = 배열 크기(항상 12)  ← "그릇 크기"
	// - snprintf 반환값  = 이번에 쓴 글자 수('\0' 제외, 값마다 다름) ← "담긴 양"
	//   예) "5 7\r\n" → 5.  sizeof로 보내면 '\0'과 쓰레기 값까지 전송됨
	uint16_t len_buf;
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
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  // ADC 자체 오차 보정. 변환 시작 전에 한 번 (Single-ended로 쓰니까 ADC_SINGLE_ENDED)
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);

  // DMA 연결 + 변환 시작까지 한 번에 (HAL_ADC_Start를 따로 부르면 이미 동작 중이라 BUSY)
  // - 1번 인자: 핸들의 "주소" (&hadc1)
  // - 2번 인자: 함수 원형이 uint32_t * 라서 캐스팅. 실제 저장 크기는 DMA 설정(Half Word)을 따름
  // - 3번 인자: 바이트 수가 아니라 "옮기는 횟수" = 채널 수 2
  // - 이 함수가 DMA 절반/전체 완료 인터럽트도 켬
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)buf, 2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  // 숫자(2진수 값) -> 10진수 문자열로 변환해서 uart_buf에 저장
	  // - %d: 2진수 값을 10진수 ASCII 문자로 바꿔 줌 (4095 -> '4''0''9''5')
	  //   → 값에 따라 글자 수가 달라짐 (5는 1글자, 4095는 4글자) → 전송 길이가 매번 다름
	  //   (uint16_t는 int로 자동 변환돼서 %d도 OK. 부호 없는 값이라 의미상은 %u)
	  // - 2번 인자 sizeof(uart_buf): 최대 크기. 넘치면 잘라서 써 줌(뒤의 \r\n이 잘릴 수 있음)
	  // - \r (CR, 0x0D): 커서를 줄 맨 앞으로 / \n (LF, 0x0A): 한 줄 아래로
	  //   둘 다 보내야 Tera Term에서 다음 줄 맨 앞부터 찍힘
	  // - 끝의 '\0'(NULL 문자, ASCII 0)은 snprintf가 자동으로 붙임. 숫자 '0'(0x30)과 다름
	  len_buf = snprintf(uart_buf, sizeof(uart_buf), "%d %d\r\n", buf[0], buf[1]);

	  // '\0' 앞까지(len_buf 글자)만 전송. 9600bps에서 1글자 ≈ 1ms → 최대 약 11ms
	  HAL_UART_Transmit(&huart1, (uint8_t *)uart_buf, len_buf, 100);

	  // 단위 ms → 0.1초. 실제 한 바퀴 ≈ 전송 시간 + 100ms
	  // (정확한 주기가 필요하면 타이머 인터럽트나 HAL_GetTick() 비교)
	  HAL_Delay(100);
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
