/**
  ******************************************************************************
  * @file    stepper.c
  * @brief   DRV8825 + KH42JM2-901 스텝모터 2개 구동 (STEP = 타이머 PWM)
  ******************************************************************************
  * 핀과 타이머 초기화는 CubeMX가 한다 (MX_GPIO_Init, MX_TIM1_Init, MX_TIM3_Init).
  * 이 파일은 PWM 시작과 속도(Hz) -> ARR 변환만 담당한다.
  ******************************************************************************
  */
#include "stepper.h"
#include "tim.h"

/* 모터 하나에 필요한 정보 묶음 */
typedef struct
{
  TIM_HandleTypeDef *htim;        /* STEP 펄스를 만드는 타이머 */
  uint32_t           channel;     /* 그 타이머의 PWM 채널 */
  GPIO_TypeDef      *dir_port;    /* DIR 핀 포트 */
  uint16_t           dir_pin;     /* DIR 핀 번호 */
  uint8_t            dir_invert;  /* 1 = 방향 반대로 (마주 보고 달린 모터용) */
  float              hz;          /* 현재 속도 (부호 있음, 0 = 정지) */
} Stepper;

/* 두 모터는 서로 마주 보고 달려 있어서, 로봇이 앞으로 가려면 한쪽은
 * 시계 방향, 다른 쪽은 반시계 방향으로 돌아야 한다. dir_invert로
 * 오른쪽을 뒤집어서 "+Hz = 전진"이 양쪽 다 맞게 한다.
 * 로봇이 뒤로 가거나 제자리에서 돌면 이 0/1을 바꾼다. */
static Stepper motor[STEPPER_NUM] =
{
  [STEPPER_LEFT]  = { &htim1, TIM_CHANNEL_1, LM_Dir_GPIO_Port, LM_Dir_Pin, 0, 0.0f },
  [STEPPER_RIGHT] = { &htim3, TIM_CHANNEL_2, RM_Dir_GPIO_Port, RM_Dir_Pin, 1, 0.0f },
};

/**
  * @brief  DIR 변경 후 STEP 펄스 전까지 잠깐 대기
  * @note   DRV8825는 STEP 상승 엣지 전에 DIR이 650 ns 이상 안정되어 있어야 한다.
  *         170 MHz에서 400번 루프 = 2 us 남짓 (여유 있게).
  */
static void dir_setup_delay(void)
{
  for (volatile uint32_t i = 0; i < 400; i++) { __NOP(); }
}

/**
  * @brief  TIM1, TIM3 PWM 출력 시작
  * @note   CubeMX에서 Pulse(CCR) = 0으로 했으므로 아직 펄스는 안 나간다.
  *         EN도 HIGH(꺼짐)로 초기화되어 있으므로 Stepper_Enable(1) 전까지 모터는 풀려 있다.
  */
void Stepper_Init(void)
{
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);   /* TIM1은 advanced timer: MOE 비트도 여기서 켜짐 */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
}

/**
  * @brief  두 드라이버의 출력 켜기/끄기 (EN 핀)
  * @param  on  1 = EN LOW -> 코일에 전류, 회전 가능 (정지 중에도 홀딩 토크)
  *             0 = EN HIGH -> 코일 전류 차단, 바퀴 풀림
  */
void Stepper_Enable(uint8_t on)
{
  HAL_GPIO_WritePin(Motor_en_GPIO_Port, Motor_en_Pin, on ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

/**
  * @brief  모터 하나의 STEP 주파수 설정
  * @param  id  어느 모터인지 (STEPPER_LEFT / STEPPER_RIGHT)
  * @param  hz  부호 있는 초당 스텝 수. + = 전진, - = 후진.
  *             |hz| < STEPPER_MIN_HZ 이면 정지 (EN이 LOW면 홀딩 토크는 유지).
  */
void Stepper_SetSpeed(Stepper_Id id, float hz)
{
  Stepper *m = &motor[id];
  float f = (hz < 0.0f) ? -hz : hz;   /* 크기만 */

  /* 범위 제한 */
  if (f < STEPPER_MIN_HZ) { hz = 0.0f; f = 0.0f; }
  if (f > STEPPER_MAX_HZ) { f = STEPPER_MAX_HZ; hz = (hz < 0.0f) ? -f : f; }

  int8_t old_sign = (m->hz > 0.0f) - (m->hz < 0.0f);   /* -1, 0, +1 */
  int8_t new_sign = (hz > 0.0f) - (hz < 0.0f);

  /* 정지: CCR = 0 -> 다음 주기부터 출력이 계속 LOW (0으로 나누기도 피함) */
  if (f == 0.0f)
  {
    __HAL_TIM_SET_COMPARE(m->htim, m->channel, 0);
    m->hz = 0.0f;
    return;
  }

  /* Hz -> ARR 변환은 여기서만. float -> 정수는 이 한 줄뿐.
   * +0.5f = 반올림 (그냥 자르면 항상 내림이라 실제 주파수가 조금씩 높아짐).
   * f가 16~20000 사이라 결과는 62499~49: 16bit ARR 안에 들어간다. */
  uint32_t arr = (uint32_t)(STEPPER_TICK_HZ / f + 0.5f) - 1;

  if (old_sign != new_sign)
  {
    /* 정지에서 출발 또는 방향 반전:
     * 펄스 멈춤 -> DIR 변경 -> 대기 -> 새 주기로 바로 재시작 */
    __HAL_TIM_SET_COMPARE(m->htim, m->channel, 0);
    HAL_TIM_GenerateEvent(m->htim, TIM_EVENTSOURCE_UPDATE);   /* CCR=0 즉시 적용 -> STEP LOW */

    GPIO_PinState dir = (new_sign > 0) ? GPIO_PIN_RESET : GPIO_PIN_SET;
    if (m->dir_invert) dir = (dir == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET;
    HAL_GPIO_WritePin(m->dir_port, m->dir_pin, dir);
    dir_setup_delay();

    __HAL_TIM_SET_AUTORELOAD(m->htim, arr);
    __HAL_TIM_SET_COMPARE(m->htim, m->channel, (arr + 1) / 2);   /* duty 50% */
    HAL_TIM_GenerateEvent(m->htim, TIM_EVENTSOURCE_UPDATE);      /* CNT=0부터 새 주기 시작 */
  }
  else
  {
    /* 같은 방향: 주기만 바꾼다. preload가 켜져 있어서 새 값은
     * 지금 주기가 끝날 때 적용된다 (펄스가 깨지지 않음). */
    __HAL_TIM_SET_AUTORELOAD(m->htim, arr);
    __HAL_TIM_SET_COMPARE(m->htim, m->channel, (arr + 1) / 2);
  }

  m->hz = hz;
}

/**
  * @brief  현재 설정된 속도 읽기
  * @param  id  어느 모터인지 (STEPPER_LEFT / STEPPER_RIGHT)
  * @retval 현재 속도 (Hz, 부호 있음, 0 = 정지)
  */
float Stepper_GetSpeed(Stepper_Id id)
{
  return motor[id].hz;
}






