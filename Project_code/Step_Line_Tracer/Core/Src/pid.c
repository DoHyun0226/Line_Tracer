/**
  ******************************************************************************
  * @file    pid.c
  * @brief   라인 추종 제어: PID 계수, 가속 제한 (모터 목표 속도 계산)
  *          Created on: 2026. 11. 19.  Author: yoodo
  ******************************************************************************
  * 흐름: 센서(tcrt5000) -> 라인 위치 -> PID -> 좌우 목표 Hz -> Stepper_RampTo -> Stepper_SetSpeed
  * 튜닝 기본값은 pid.h의 PID_MOTOR_* define.
  ******************************************************************************
  */


#include "pid.h"

#define PID_GAIN_MAX 1000.0f   /* gain 최대값 (Hz, 좌우 속도 차이). ± 대칭으로 자름 */
#define PID_MOTOR_BASE_HZ_DEFAULT   2000.0f   /* 직진 기본 속도 (Hz). 1/8 모드, 바퀴 65mm 기준 약 0.25 m/s */

volatile float base_hz = PID_MOTOR_BASE_HZ_DEFAULT;   /* 실제 값 (튜닝 중 변경) */

/* 가속 튜닝값 (pid.h에서 extern으로 공개 -> main.c 등에서 블루투스로 변경 가능)
 * static을 붙이면 다른 파일에서 못 보므로 붙이지 않는다. */
volatile float k_acc        = PID_MOTOR_K_ACC_DEFAULT;
volatile float accel_max_hz = PID_MOTOR_ACCEL_MAX_HZ;

/* PID 계수 (같은 방식으로 extern 공개) */
volatile float k_p = PID_MOTOR_KP_DEFAULT;
volatile float k_i = PID_MOTOR_KI_DEFAULT;
volatile float k_d = PID_MOTOR_KD_DEFAULT;

/* PID 내부 상태 (이 파일 안에서만 사용 -> static)
 * double 말고 float 사용:
 * STM32G4 (Cortex-M4F)의 FPU는 단정밀도(float)만 지원한다.
 * double 연산은 하드웨어 FPU가 아니라 소프트웨어로 처리되어 훨씬 느리다.
 * 상수도 float로 쓴다 (예: 1.0f). */
static float gain_p   = 0.0f;   /* P항 결과, 매 주기 새로 계산 (디버깅 관찰용) */
static float gain_i   = 0.0f;   /* I항 결과 = k_i * err_int, 매 주기 새로 계산 (디버깅 관찰용) */
static float gain_d   = 0.0f;   /* D항 결과, 매 주기 새로 계산 (디버깅 관찰용) */
static float err_prev = 0.0f;   /* 이전 오차 (지난 주기 값, D항 계산용, 재출발 시 0으로 리셋) */
static float err_int  = 0.0f;   /* 오차 누적 Σ(err x dt), mm·s (꼭 필요한 상태, 재출발 시 0으로 리셋) */
static float gain	  = 0.0f;   /* PID 최종 출력 = P + I + D, ±PID_GAIN_MAX로 자름 (Hz, 좌우 속도 차이) */

/**
  * @brief  PID 계산: 라인 위치를 읽어 gain(좌우 속도 차이)을 갱신
  * @param  dt_ms  지난 호출과의 시간 간격 (ms). 제어 주기 고정이면 CONTROL_PERIOD_MS
  * @note   제어 주기마다 한 번만 부를 것 (두 번 부르면 err_int가 두 번 쌓이고 D항이 0이 됨).
  *         오차 = 0 - 라인 위치 (목표는 가운데 0). 라인이 오른쪽(+)이면 오차는 -.
  */
void update_gain(uint32_t dt_ms)
{
	float err_curr;
	float dt;

	if(dt_ms == 0)
	{
		return;   /* 0으로 나누기 방지: 이번 주기는 건너뛰고 이전 gain 유지 */
	}
	else
	{
		err_curr = - get_current_pos();
		dt = dt_ms/1000.0f;


		gain_p = k_p * err_curr;
		gain_d = k_d * (err_curr - err_prev) / dt;
		err_prev = err_curr;
		err_int = err_int + err_curr * dt;
		gain_i = k_i * err_int;

		gain = gain_p + gain_d + gain_i;

		if (gain > PID_GAIN_MAX)
		{
			gain = PID_GAIN_MAX;
		}
		else if (gain < -PID_GAIN_MAX)
		{
			gain = -PID_GAIN_MAX;
		}
	}
}


void pid_task(uint32_t dt_ms)
{
	update_gain(dt_ms);
	Stepper_RampTo(STEPPER_RIGHT, base_hz + gain);
	Stepper_RampTo(STEPPER_LEFT, base_hz - gain);
}

/**
  * @brief  목표 속도를 향해 한 걸음 가속/감속 (탈조 방지용 가속 제한)
  *         매 제어 주기마다 (PID 계산 직후) 같은 간격으로 호출한다.
  *
  *         변화량 = k_acc * (목표 Hz - 현재 Hz)       : 멀수록 크게, 가까울수록 작게 (비례)
  *         변화량을 ±accel_max_hz로 자름               : 한 번에 너무 크게 바뀌면 탈조
  *         새 속도 = 현재 + 변화량 -> Stepper_SetSpeed()로 바로 적용
  *
  *         예 (k_acc 1.0, accel_max_hz 200):
  *           현재 2000, 목표 3000 -> 2200  (최대 가속)
  *           현재 2000, 목표 1000 -> 1800  (최대 감속)
  *           현재 2000, 목표 2100 -> 2100  (한 번에 도착)
  *
  * @param  id         어느 모터인지 (STEPPER_LEFT / STEPPER_RIGHT)
  * @param  target_hz  목표 속도 (부호 있음, + 전진 / - 후진)
  * @note   현재 속도는 Stepper_GetSpeed 함수를 통해 current_hz에 저장한다. SetSpeed가 |Hz| < 16을 0으로
  *         저장하므로, k_acc * 목표 Hz가 16보다 작으면 정지 상태에서 출발하지 못한다.
  *         (기본값 k_acc 1.0, 추천 범위 0.5~1에서는 문제없음)
  */
void Stepper_RampTo(Stepper_Id id, float target_hz)
{
	float diff_hz;      /* 목표 - 현재 */
	float gain_acc;     /* 이번 주기 변화량 (비례) */
	float new_acc;      /* 이번 주기에 설정할 새 속도 (Hz) */
	float current_hz;   /* 현재 속도 (Hz) */

	current_hz = Stepper_GetSpeed(id);
	diff_hz = target_hz - current_hz;
	gain_acc = k_acc * diff_hz;

	/* 변화량이 ±accel_max_hz를 넘으면 그 한도만큼만 바꿈 (위/아래 방향 구분) */
	if (gain_acc > accel_max_hz || gain_acc < - accel_max_hz)
	{
		new_acc = gain_acc > 0 ? current_hz + accel_max_hz : current_hz -accel_max_hz;
	}
	else
	{
		new_acc = current_hz + gain_acc;   /* 한도 안: 비례 변화량 그대로 */
	}

	Stepper_SetSpeed(id, new_acc);   /* Hz -> ARR, DIR 처리는 stepper가 함 */
}
