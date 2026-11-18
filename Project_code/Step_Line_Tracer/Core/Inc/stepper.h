/**
  ******************************************************************************
  * @file    stepper.h
  * @brief   DRV8825 + KH42JM2-901 스텝모터 2개 구동 (STEP = 타이머 PWM)
  ******************************************************************************
  *
  * 동작 원리
  *   DRV8825는 STEP 핀의 상승 엣지마다 모터를 한 (마이크로)스텝 돌린다.
  *   그래서 "속도" = STEP 주파수(Hz). 타이머 PWM(duty 50%)이 이 펄스를
  *   하드웨어로 만들고, CPU는 ARR만 바꿔서 속도를 바꾼다.
  *
  *   타이머 클럭 170 MHz / (PSC+1 = 170) = 1 MHz tick (1 us)
  *   STEP 주파수 = 1 MHz / (ARR + 1)      ->  ARR = 1,000,000 / Hz - 1
  *   TIM1, TIM3는 16bit라서 최저 주파수는 1 MHz / 65536 = 약 15.3 Hz.
  *
  * 핀 (CubeMX에서 설정, User Label 이름)
  *
  *   신호      핀     Arduino   설정
  *   -------   ----   -------   -----------------------------
  *   L_STEP    PA8    D7        TIM1_CH1, PWM
  *   L_DIR     PB10   D6        GPIO 출력  (LM_Dir)
  *   R_STEP    PC7    D9        TIM3_CH2, PWM
  *   R_DIR     PB4    D5        GPIO 출력  (RM_Dir)
  *   EN        PB5    D4        GPIO 출력  (Motor_en), LOW = 동작
  *
  *   DRV8825 나머지 핀
  *     RST, SLP : 둘 다 3.3V에 연결. RST가 떠 있으면 안 돈다.
  *     M0~M2    : 마이크로스텝 설정. 점퍼로 고정 (STEPPER_MICROSTEP과 맞출 것).
  *     GND      : Nucleo와 GND 공통 연결 필수.
  ******************************************************************************
  */
#ifndef __STEPPER_H
#define __STEPPER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* DRV8825 M0~M2 점퍼 설정 (1, 2, 4, 8, 16, 32).
 * 점퍼를 바꾸면 이 값도 같이 바꾼다.
 * 1/8 모드: M0 = HIGH, M1 = HIGH, M2 = LOW (두 드라이버 모두) */
#define STEPPER_MICROSTEP       8
#define STEPPER_STEPS_PER_REV   (200 * STEPPER_MICROSTEP)   /* 1.8도 모터 */

/* Hz 값은 전부 float로 다룬다. 정수(ARR)로 바꾸는 건 Stepper_SetSpeed() 안에서만. */
#define STEPPER_TICK_HZ         1000000.0f  /* 프리스케일러 거친 타이머 tick */
#define STEPPER_MIN_HZ          16.0f       /* 이보다 느리면 정지 (16bit ARR 한계) */
#define STEPPER_MAX_HZ          20000.0f    /* 안전 상한 */

/* 가속 제한(Stepper_RampTo)과 PID 튜닝값은 pid.h / pid.c에 있다. */

typedef enum
{
  STEPPER_LEFT = 0,
  STEPPER_RIGHT,
  STEPPER_NUM
} Stepper_Id;

void  Stepper_Init(void);
void  Stepper_Enable(uint8_t on);
void  Stepper_SetSpeed(Stepper_Id id, float hz);
float Stepper_GetSpeed(Stepper_Id id);

#ifdef __cplusplus
}
#endif

#endif /* __STEPPER_H */
