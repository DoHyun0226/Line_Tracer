/**
  ******************************************************************************
  * @file    pid.h
  * @brief   라인 추종 제어: PID / 가속 튜닝 기본값, 공개 변수, 함수 선언
  ******************************************************************************
  * PID_MOTOR_* : 전원 켤 때의 시작값. 금요일 튜닝 결과를 여기에 적는다.
  * k_p, k_acc 등 : 실제로 쓰는 값 (volatile 전역). 실행 중에 바꿀 수 있다.
  ******************************************************************************
  */
#ifndef _PID_H
#define _PID_H

#include "stepper.h"   /* Stepper_Id, STEPPER_MICROSTEP, Stepper_SetSpeed/GetSpeed */
#include "tcrt5000.h"

/* ===== 가속 튜닝값 =====
 * 매 제어 주기마다: step = k_acc * (목표 Hz - 현재 Hz), 그 다음 ±accel_max_hz로 자름.
 * 아래 _DEFAULT는 전원 켤 때의 시작값. 실행 중에는 변수(k_acc, accel_max_hz)를 바꾼다. */
#define PID_MOTOR_K_ACC_DEFAULT   1.0f                        /* 비례 계수 (0 < k ≤ 1). 1이면 MAX로만 제한 */
#define PID_MOTOR_ACCEL_MAX_HZ    (25.0f * STEPPER_MICROSTEP) /* 호출 1회당 최대 Hz 변화 (1/8 모드: 200 Hz) */

/* ===== PID 튜닝값 =====
 * 출력 = 좌우 속도 차이 (Hz). 아래 값은 임시 시작값이며 금요일 주행에서 튜닝한다.
 * 예: Kp 100이면 error 7(가장 바깥 센서)일 때 700 Hz 차이 (1/8 모드에서 약 0.44 회전/초). */
#define PID_MOTOR_KP_DEFAULT      100.0f
#define PID_MOTOR_KI_DEFAULT      0.0f      /* 처음엔 P만, 안정되면 D, I는 마지막 */
#define PID_MOTOR_KD_DEFAULT      0.0f

/* 정의는 pid.c에 있음. volatile: 나중에 UART 인터럽트에서 바꿔도 안전하게 */
extern volatile float k_acc;
extern volatile float accel_max_hz;
extern volatile float k_p, k_i, k_d;
extern volatile float base_hz;          /* 직진 기본 속도 (Hz) */

void Stepper_RampTo(Stepper_Id id, float target_hz);
void pid_task(uint32_t dt_ms);          /* 제어 주기마다 한 번: PID 계산 -> 좌우 목표 Hz -> RampTo */

#endif /* _PID_H */
