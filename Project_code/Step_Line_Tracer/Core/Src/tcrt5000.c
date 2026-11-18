/**
  ******************************************************************************
  * @file    tcrt5000.c
  * @brief   TCRT5000 센서 8개 읽기 (ADC1, ADC2 + DMA)
  *          Created on: 2026. 11. 19.  Author: yoodo
  ******************************************************************************
  * ADC와 DMA 초기화는 CubeMX가 한다 (MX_DMA_Init, MX_ADC1_Init, MX_ADC2_Init).
  * 이 파일은 ADC 보정, DMA 시작, 버퍼 관리만 담당한다.
  * 센서 배치와 버퍼 순서는 tcrt5000.h 그림 참고.
  ******************************************************************************
  */


#include "tcrt5000.h"

/* 정규화용 센서별 흰색(MIN) / 검정(MAX) ADC 값 (0 ~ 4095)
 * 지금은 전부 임시값. 트랙 위에서 raw 값을 찍어 보고 센서마다 바꾼다.
 * (11/10 실험: 흰 종이 ≈ 200, 두꺼운 검정 ≈ 2000)
 *
 * 사용: tcrt_min_max[id][TCRT_MIN], tcrt_min_max[id][TCRT_MAX]
 *       첫 번째 [ ] = 센서 번호 (TCRT_ID: TCRT_S1 = 0 ... TCRT_S8 = 7)
 *       두 번째 [ ] = 0: 흰색(MIN), 1: 검정(MAX) */
enum { TCRT_MIN = 0, TCRT_MAX = 1 };

static const uint16_t tcrt_min_max[SENSOR_NUM][2] =
{
  /*           MIN   MAX  */
  [TCRT_S1] = { 100, 2000 },
  [TCRT_S2] = { 100, 2000 },
  [TCRT_S3] = { 100, 2000 },
  [TCRT_S4] = { 100, 2000 },
  [TCRT_S5] = { 100, 2000 },
  [TCRT_S6] = { 100, 2000 },
  [TCRT_S7] = { 100, 2000 },
  [TCRT_S8] = { 100, 2000 },
};

/* 라인 판정 기준: 가장 검은 센서의 정규화 값이 이보다 작으면 "라인 없음" (0 ~ 1000, 튜닝값) */
#define TCRT_LINE_TH  300.0f


/* =====   TCRT5000 (ADC + DMA)   ===== */
/* DMA가 백그라운드에서 이 버퍼를 계속 덮어쓰므로 volatile:
* 컴파일러가 값을 캐시하지 말고 매번 메모리에서 다시 읽게 한다.
* 인덱스 순서는 위 센서 그림 참고 (b1[k]와 b2[k]는 중심 기준 좌우 대칭 쌍). */
static volatile uint16_t buf_tcrt_1[TCRT5000_NUM_ADC1] = {0,}; // ADC1 값 (S1~S4), DMA가 갱신
static volatile uint16_t buf_tcrt_2[TCRT5000_NUM_ADC2] = {0,}; // ADC2 값 (S8~S5, 역순), DMA가 갱신

static float tcrt_norm_array[SENSOR_NUM] = {0,};

/* 센서 하나에 필요한 정보 묶음 */
typedef struct
{
  volatile uint16_t *raw;    /* 이 센서 값이 들어오는 DMA 버퍼 칸의 주소 */
  float              pos;    /* 센서 위치 (mm, 가운데 = 0, + = 오른쪽) */
} Tcrt;

/* 센서 표. TCRT_ID로 칸을 고른다 (예: tcrt[TCRT_S5].pos).
 * ADC2 버퍼는 역순이라 S5 = buf_tcrt_2[3], S8 = buf_tcrt_2[0]: 이 역순 처리는 이 표 한 곳에서만 한다.
 * 값 읽기: *tcrt[TCRT_S5].raw  (raw가 버퍼 칸의 주소라서 * 로 꺼냄) */
static const Tcrt tcrt[SENSOR_NUM] =
{
  [TCRT_S1] = { &buf_tcrt_1[0], -55.0f },
  [TCRT_S2] = { &buf_tcrt_1[1], -39.5f },
  [TCRT_S3] = { &buf_tcrt_1[2], -24.0f },
  [TCRT_S4] = { &buf_tcrt_1[3],  -8.5f },
  [TCRT_S5] = { &buf_tcrt_2[3],   8.5f },
  [TCRT_S6] = { &buf_tcrt_2[2],  24.0f },
  [TCRT_S7] = { &buf_tcrt_2[1],  39.5f },
  [TCRT_S8] = { &buf_tcrt_2[0],  55.0f },
};


/**
  * @brief  ADC 보정 후 DMA로 연속 변환 시작
  *         1. CubeMX 채널 수와 define이 같은지 검사
  *         2. ADC 자체 보정 (ADC가 꺼져 있을 때만 가능하므로 시작 전에)
  *         3. DMA 시작 -> 이후 buf_tcrt_1, buf_tcrt_2가 계속 갱신됨
  *         4. 필요 없는 DMA 인터럽트 끄기
  * @note   MX_DMA_Init(), MX_ADC1_Init(), MX_ADC2_Init() 다음에 한 번만 호출한다.
  *         실패하면 Error_Handler()에서 멈춘다 (아직 모터가 꺼져 있을 때라 안전).
  */
void tcrt5000_init(void)
{
	/* 1. 채널 수 검사: CubeMX에서 센서를 바꾸고 define을 안 고치면 여기서 멈춤 */
	if (hadc1.Init.NbrOfConversion != TCRT5000_NUM_ADC1 ||
	    hadc2.Init.NbrOfConversion != TCRT5000_NUM_ADC2)
	{
	  Error_Handler();
	}


	/* 2. ADC 자체 보정 (오프셋 오차 보정). 성공하면 HAL_OK (= 0) */
	if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK)
	{
	  Error_Handler();
	}
	if (HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED) != HAL_OK)
	{
	  Error_Handler();
	}



	/* 3. DMA 시작. (uint32_t *)는 HAL 함수 인자 형식일 뿐, 실제 전송은 Half Word(16bit)씩 */
	if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)buf_tcrt_1, TCRT5000_NUM_ADC1) != HAL_OK)
	{
	  Error_Handler();
	}
	if (HAL_ADC_Start_DMA(&hadc2, (uint32_t *)buf_tcrt_2, TCRT5000_NUM_ADC2) != HAL_OK)
	{
	  Error_Handler();
	}


	/* 4. DMA 절반/전체 완료 인터럽트 끄기: 버퍼는 직접 읽으므로 필요 없고, 켜 두면 초당 수만 번 인터럽트가 걸린다.
	 * 꺼도 DMA는 계속 버퍼를 채운다. Start_DMA가 인터럽트를 켜므로 반드시 그 뒤에 호출.
	 * hadc1.DMA_Handle = &hdma_adc1 (CubeMX가 HAL_ADC_MspInit에서 연결). 이미 포인터라 & 없음. */
	__HAL_DMA_DISABLE_IT(hadc1.DMA_Handle, DMA_IT_HT | DMA_IT_TC);
	__HAL_DMA_DISABLE_IT(hadc2.DMA_Handle, DMA_IT_HT | DMA_IT_TC);
}

/**
  * @brief  센서 위치 읽기
  * @param  id  센서 번호 (TCRT_S1 ~ TCRT_S8, 왼쪽 -> 오른쪽)
  * @retval 센서 위치 (mm, 가운데 = 0, - = 왼쪽, + = 오른쪽)
  */
float get_tcrt_pos(TCRT_ID id)
{
	return tcrt[id].pos;          // get_tcrt_pos(TCRT_S5) → 8.5
}

/**
  * @brief  센서의 ADC 원래 값 읽기 (DMA 버퍼의 최신 값)
  * @param  id  센서 번호 (TCRT_S1 ~ TCRT_S8)
  * @retval ADC 값 (12bit, 0 ~ 4095). 흰색일수록 작고 검정일수록 큼
  * @note   ADC2 버퍼의 역순은 tcrt[] 표에서 처리되므로 id만 넣으면 된다.
  *         흰/검 보정값(tcrt_min_max)을 잴 때 이 값을 블루투스로 찍어 본다.
  */
uint16_t get_tcrt_raw(TCRT_ID id)
{
	return *(tcrt[id].raw);   /* raw는 버퍼 칸의 주소 -> * 로 값을 꺼냄 */
}

/**
  * @brief  센서 값 정규화 (센서마다 다른 감도를 같은 기준으로 맞춤)
  *
  *                   raw - min
  *         norm = ─────────────── x 1000      min = 흰색 값, max = 검정 값 (tcrt_min_max 표)
  *                   max - min
  *
  *         예 (min 100, max 2000): raw 100 -> 0,  raw 1050 -> 500,  raw 2000 -> 1000
  * @param  id  센서 번호 (TCRT_S1 ~ TCRT_S8)
  * @retval 0 (흰색) ~ 1000 (검정). 범위를 넘으면 0 또는 1000으로 자름
  * @note   표 값이 잘못되면 (max <= min) 0을 돌려준다 (그 센서는 흰색 취급).
  *         Error_Handler()를 쓰지 않는 이유: 주행 중에 불리는 함수라 멈추면 모터가 폭주한다.
  *         TODO: tcrt5000_init()에서 표를 한 번 검사하거나 LED로 표시
  */
float  get_tcrt_norm(TCRT_ID id)
{
	uint16_t raw = get_tcrt_raw(id);
	float norm;

	/* 표 값 오류 (max <= min): 0으로 나누기 방지. 정상이면 절대 안 들어옴 */
	if (tcrt_min_max[id][TCRT_MAX] <= tcrt_min_max[id][TCRT_MIN])
	{
		return 0.0f;
	}

	/* (float)raw, 1000.0f: 계산 전체를 float로 (정수 나눗셈으로 소수점이 버려지지 않게) */
	norm = ((float)raw - tcrt_min_max[id][TCRT_MIN])*1000.0f/(tcrt_min_max[id][TCRT_MAX] - tcrt_min_max[id][TCRT_MIN]);

	/* 0 ~ 1000으로 자르기: 흰색보다 밝으면 음수, 공중에 들면 1000 넘게 나옴 */
	if (norm > 1000.0f)
	{
		return 1000.0f;
	}
	else if (norm < 0.0f)
	{
		return 0.0f;
	}
	else
	{
	return norm;
	}
}


void get_tcrt_norm_array(void)
{
	int i = 0;
	while (i < SENSOR_NUM)
	{
		tcrt_norm_array[i] = get_tcrt_norm(i);
		i++;
	}
}


/**
  * @brief  정규화 값이 가장 큰 (가장 검은) 센서 번호 찾기
  * @note   tcrt_norm_array를 읽으므로 get_tcrt_norm_array()를 먼저 불러야 한다.
  *         값이 같으면 가운데에 더 가까운 센서가 우선 (|pos|가 작은 쪽).
  *         예: S2와 S4가 둘 다 1000이면 -> S4 (|-8.5| < |-39.5|)
  *         가운데에서 거리가 같은 S4(-8.5)와 S5(+8.5)가 같으면 먼저 본 S4.
  * @retval 센서 번호 (TCRT_S1 ~ TCRT_S8)
  */
static TCRT_ID get_max_norm_index(void)
{
	int   max_i = 0;                      /* 지금까지 가장 큰 값의 센서 번호 */
	float max_v = tcrt_norm_array[0];     /* 지금까지 가장 큰 값 */

	for (int i = 1; i < SENSOR_NUM; i++)
	{
		float v = tcrt_norm_array[i];

		if (v > max_v)
		{
			/* 더 큰 값: 무조건 바꿈 */
			max_v = v;
			max_i = i;
		}
		else if (v == max_v)
		{
			/* 같은 값: 가운데에서의 거리(|pos|)가 더 작으면 바꿈.
			 * 정규화 값은 0이나 1000으로 잘리는 경우가 많아서 == 비교가 실제로 자주 맞는다. */
			float d_new = (tcrt[i].pos     < 0.0f) ? -tcrt[i].pos     : tcrt[i].pos;
			float d_old = (tcrt[max_i].pos < 0.0f) ? -tcrt[max_i].pos : tcrt[max_i].pos;

			if (d_new < d_old)
			{
				max_i = i;
			}
		}
	}

	return (TCRT_ID)max_i;
}


/**
  * @brief  현재 라인 위치 계산 (가장 검은 센서와 양옆, 3개로 가중평균)
  *
  *                Σ (norm[i] x pos[i])
  *         pos = ──────────────────────     i = max_i-1, max_i, max_i+1
  *                    Σ norm[i]
  *
  *         예: S4 = 800, S5 = 200 -> (800 x -8.5 + 200 x 8.5) / 1000 = -5.1 mm
  * @retval 라인 위치 (mm, 가운데 = 0, - = 왼쪽, + = 오른쪽) -> PID의 오차로 사용
  * @note   센서 값 갱신(get_tcrt_norm_array)도 여기서 한다. 제어 주기마다 한 번만 부를 것.
  *         라인을 놓치면 (가장 검은 값 < TCRT_LINE_TH) 마지막으로 본 쪽 끝 위치(±55 mm)를 돌려준다.
  *         -> PID가 그쪽으로 최대한 꺾어서 라인을 다시 찾게 된다.
  */
float get_current_pos(void)
{
	static float last_pos = 0.0f;   /* 마지막으로 라인을 본 위치 (함수가 끝나도 유지) */

	get_tcrt_norm_array();          /* 1. 센서 8개 정규화 값 갱신 */

	TCRT_ID max_i = get_max_norm_index();   /* 2. 가장 검은 센서 */

	/* 3. 라인 놓침: 가장 검은 센서도 흰색 수준이면 계산하지 않고 마지막 방향 끝으로 */
	if (tcrt_norm_array[max_i] < TCRT_LINE_TH)
	{
		return (last_pos < 0.0f) ? tcrt[TCRT_S1].pos : tcrt[TCRT_S8].pos;
	}

	/* 4. 양옆 범위 (양 끝 센서면 한쪽만) */
	int lo = (max_i > 0)              ? max_i - 1 : 0;
	int hi = (max_i < SENSOR_NUM - 1) ? max_i + 1 : SENSOR_NUM - 1;

	/* 5. 가중평균. 합은 최소 TCRT_LINE_TH 이상이라 0으로 나눌 일 없음 */
	float sum_v  = 0.0f;
	float sum_vp = 0.0f;
	for (int i = lo; i <= hi; i++)
	{
		sum_v  += tcrt_norm_array[i];
		sum_vp += tcrt_norm_array[i] * tcrt[i].pos;
	}

	last_pos = sum_vp / sum_v;
	return last_pos;
}

