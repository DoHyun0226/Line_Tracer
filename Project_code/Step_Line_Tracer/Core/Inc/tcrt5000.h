/**
  ******************************************************************************
  * @file    tcrt5000.h
  * @brief   TCRT5000 반사형 적외선 센서 8개 읽기 (ADC1, ADC2 + DMA)
  *          Created on: 2026. 11. 19.  Author: yoodo
  ******************************************************************************
  *
  * 동작 원리
  *   ADC1, ADC2가 각각 4채널을 연속 변환(Continuous)하고, DMA가 Circular로
  *   버퍼를 계속 덮어쓴다. CPU는 아무것도 안 해도 버퍼에 항상 최신 값이 있다.
  *
  *   ADC 12bit -> 값 0 ~ 4095 (전압 = 값 x 3.3V / 4095)
  *   흰색 ≈ 200, 검정 ≈ 2000 (11/10 실험값, 트랙에서 다시 잴 것)
  *   검정일수록 값이 크다 (반사가 적어 포토트랜지스터 전류가 작음 -> 컬렉터 전압 높음)
  *
  *   한 바퀴(4채널) 시간: ADC 클럭 42.5 MHz, 샘플링 640.5 + 변환 12.5 사이클
  *   -> 채널당 약 15.4 us, 4채널 약 61.5 us
  *
  * CubeMX 설정 (이 파일의 define과 맞아야 함)
  *   ADC1, ADC2 : 12bit, Continuous ON, DMA Continuous Requests ON, Rank 4개
  *   DMA        : Circular, Half Word (uint16_t 버퍼와 크기 일치)
  ******************************************************************************
  */


/* 센서 배치 (위에서 본 모습). 아래 그림의 b1 = buf_tcrt_1, b2 = buf_tcrt_2 (tcrt5000.c)
 *
 */
//                         ^ FRONT ^
//
// <-------- ADC1 -------->    |    <-------- ADC2 -------->
//                             |
// +-----+-----+-----+-----+   |   +-----+-----+-----+-----+
// | S1  | S2  | S3  | S4  |   |   | S5  | S6  | S7  | S8  |
// +-----+-----+-----+-----+   |   +-----+-----+-----+-----+
//                             |
//   PB14  PB12  PB1   PB0     |     PA4   PB2   PC5   PC4    <- 핀
//   IN5   IN11  IN12  IN15    |     IN17  IN12  IN11  IN5    <- 채널
//   R1    R2    R3    R4      |     R4    R3    R2    R1     <- Rank
//  b1[0] b1[1] b1[2] b1[3]    |    b2[3] b2[2] b2[1] b2[0]   <- DMA 버퍼
//   -7    -5    -3    -1      |     +1    +3    +5    +7     <- 가중치
//                             |
// <<< Rank 1부터            CENTER            Rank 1부터 >>>
//                          (error = 0)
//
// ADC2 버퍼는 순서가 거꾸로: S5 = b2[3], S8 = b2[0]


#ifndef INC_TCRT5000_H_
#define INC_TCRT5000_H_

#include "main.h"
#include "adc.h"


/* 센서 개수. CubeMX의 Number Of Conversion과 같아야 한다.
 * 다르면 DMA 버퍼 칸이 밀려서 값이 엉뚱한 칸에 들어가므로, tcrt5000_init()에서 검사한다. */
#define TCRT5000_NUM_ADC1 4     /* ADC1에 연결된 센서 수 (S1~S4) */
#define TCRT5000_NUM_ADC2 4     /* ADC2에 연결된 센서 수 (S5~S8) */

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
	SENSOR_NUM      /* 센서 개수 (= 8) */
} TCRT_ID;



void     tcrt5000_init(void);          /* ADC 보정 + DMA 시작. MX_ADC1/2_Init() 다음에 한 번 호출 */
uint16_t get_tcrt_raw(TCRT_ID id);     /* ADC 원래 값 (0 ~ 4095), 흰/검 보정값 잴 때 사용 */
float    get_tcrt_norm(TCRT_ID id);    /* 정규화 값 (0 = 흰색 ~ 1000 = 검정) */
float    get_tcrt_pos(TCRT_ID id);     /* 센서 위치 (mm, 가운데 = 0, + = 오른쪽) */
void     get_tcrt_norm_array(void);    /* 정규화 값 8개를 내부 배열에 갱신 (get_current_pos가 부름) */
float    get_current_pos(void);        /* 라인 위치 (mm). 제어 주기마다 한 번 호출 -> PID 오차 */


#endif /* INC_TCRT5000_H_ */
