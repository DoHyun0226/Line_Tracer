/**
  ******************************************************************************
  * @file    tcrt5000.h
  * @brief   TCRT5000 반사형 적외선 센서 10개 읽기 (ADC1, ADC2 + DMA)
  *          Created on: 2026. 11. 19.  Author: yoodo
  ******************************************************************************
  *
  * 동작 원리
  *   ADC1, ADC2가 각각 5채널을 연속 변환(Continuous)하고, DMA가 Circular로
  *   버퍼를 계속 덮어쓴다. CPU는 아무것도 안 해도 버퍼에 항상 최신 값이 있다.
  *
  *   ADC 12bit -> 값 0 ~ 4095 (전압 = 값 x 3.3V / 4095)
  *   흰색 ≈ 200, 검정 ≈ 2000 (11/10 실험값, 트랙에서 다시 잴 것)
  *   검정일수록 값이 크다 (반사가 적어 포토트랜지스터 전류가 작음 -> 컬렉터 전압 높음)
  *
  *   한 바퀴(5채널) 시간: ADC 클럭 42.5 MHz, 샘플링 640.5 + 변환 12.5 사이클
  *   -> 채널당 약 15.4 us, 5채널 약 77 us
  *
  * CubeMX 설정 (이 파일의 define과 맞아야 함)
  *   ADC1, ADC2 : 12bit, Continuous ON, DMA Continuous Requests ON, Rank 5개
  *   DMA        : Circular, Half Word (uint16_t 버퍼와 크기 일치)
  ******************************************************************************
  */


/* 센서 배치 (위에서 본 모습). 아래 그림의 b1 = buf_tcrt_1, b2 = buf_tcrt_2 (tcrt5000.c)
 *
 */
//                                 ^ FRONT ^
//
// <------------ ADC1 ------------->   |   <------------ ADC2 ------------->
//                                     |
// +-----+-----+-----+-----+-----+     |     +-----+-----+-----+-----+-----+
// | S1  | S2  | S3  | S4  | S5  |     |     | S6  | S7  | S8  | S9  | S10 |
// +-----+-----+-----+-----+-----+     |     +-----+-----+-----+-----+-----+
//                                     |
//  PB14  PB12  PB1   PB0   PA0        |      PC4   PC5   PB2   PA4   PA6     <- 핀
//  IN5   IN11  IN12  IN15  IN1        |      IN5   IN11  IN12  IN17  IN3     <- 채널
//  R1    R2    R3    R4    R5         |      R1    R2    R3    R4    R5      <- Rank
//  b1[0] b1[1] b1[2] b1[3] b1[4]      |      b2[0] b2[1] b2[2] b2[3] b2[4]   <- DMA 버퍼
//  -57.5 -45.0 -32.5 -20.0 -7.5       |      +7.5  +20.0 +32.5 +45.0 +57.5   <- 위치 (mm)
//                                     |
// Rank 1부터 왼쪽 -> 오른쪽 >>>     CENTER     Rank 1부터 왼쪽 -> 오른쪽 >>>
//                                (error = 0)
//
// 두 ADC 모두 Rank 1이 왼쪽 (역순 없음): S1~S5 = b1[0]~b1[4], S6~S10 = b2[0]~b2[4]
// 센서 간격 12.5 mm, 가운데(S5~S6)만 15 mm


#ifndef INC_TCRT5000_H_
#define INC_TCRT5000_H_

#include "main.h"
#include "adc.h"


/* 센서 개수. CubeMX의 Number Of Conversion과 같아야 한다.
 * 다르면 DMA 버퍼 칸이 밀려서 값이 엉뚱한 칸에 들어가므로, tcrt5000_init()에서 검사한다. */
#define TCRT5000_NUM_ADC1 5     /* ADC1에 연결된 센서 수 (S1~S5) */
#define TCRT5000_NUM_ADC2 5     /* ADC2에 연결된 센서 수 (S6~S10) */

/* 센서 번호 (왼쪽 -> 오른쪽). tcrt5000.c의 tcrt[] 표에서 칸 번호로 쓴다.
 * 앞에 TCRT_를 붙인 이유: S1 같은 짧은 이름은 다른 헤더의 이름과 겹칠 수 있다. */
typedef enum {
	TCRT_S1 = 0,
	TCRT_S2,
	TCRT_S3,
	TCRT_S4,
	TCRT_S5,
	TCRT_S6,
	TCRT_S7,
	TCRT_S8,
	TCRT_S9,
	TCRT_S10,
	SENSOR_NUM      /* 센서 개수 (= 10) */
} TCRT_ID;



void     tcrt5000_init(void);          /* ADC 보정 + DMA 시작. MX_ADC1/2_Init() 다음에 한 번 호출 */
uint16_t get_tcrt_raw(TCRT_ID id);     /* ADC 원래 값 (0 ~ 4095), 흰/검 보정값 잴 때 사용 */
void            update_tcrt_raw_array(void);   /* 정규화 세팅할 때만 호출 */
const uint16_t *get_tcrt_raw_array(void);      /* 주소만 돌려줌 */
float    get_tcrt_norm(TCRT_ID id);    /* 정규화 값 (0 = 흰색 ~ 1000 = 검정) */
float    get_tcrt_pos(TCRT_ID id);     /* 센서 위치 (mm, 가운데 = 0, + = 오른쪽) */
void            update_tcrt_norm_array(void);  /* 정규화 값 10개 갱신. 정지 상태 확인용 (주행 중엔 get_current_pos가 함), 인터럽트에서 부르지 말 것 */
const float    *get_tcrt_norm_array(void);     /* 정규화 값 배열 주소만 돌려줌, 읽기 전용 */
float    get_current_pos(void);        /* 라인 위치 (mm). 제어 주기마다 한 번 호출 -> PID 오차 */


#endif /* INC_TCRT5000_H_ */
