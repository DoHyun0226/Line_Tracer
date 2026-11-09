# ADC_TCRT5000

TCRT5000 센서 2개 값을 **ADC1 + DMA**로 계속 읽고, 0.1초마다 USART1로 보내는 테스트 프로젝트이다.
USART1은 블루투스 모듈 BT04-A에 연결되어 있어서, PC에서는 블루투스 COM 포트(Tera Term, 9600bps)로 값을 본다.
직접 작성한 코드는 모두 `Core/Src/main.c`에 있고, 설정은 `ADC_TCRT5000.ioc`로 만들었다.

회로도, 측정값, 실험 사진은 [TCRT5000_Test](../../실험_사진/TCRT5000_Test)에 정리되어 있다.

## 하드웨어

- 보드: NUCLEO-G474RE (SYSCLK 170MHz, HSE 24MHz → PLL)
- 센서: TCRT5000 2개 (IR LED 5V + 200Ω, 포토트랜지스터 풀업 3.3V + 10kΩ)
- 블루투스: BT04-A (ZS-040 기판, VCC 5V)

| 신호 | 핀 | 설정 |
|---|---|---|
| 센서 1 | PA0 (A0) | ADC1_IN1, Single-ended |
| 센서 2 | PA1 (A1) | ADC1_IN2, Single-ended |
| UART TX | PC4 | USART1_TX, 9600bps 8N1 |
| UART RX | PC5 | USART1_RX |

## ADC / DMA 설정

| 항목 | 값 | 이유 |
|---|---|---|
| Clock Prescaler | Synchronous /4 | 170MHz / 4 = 42.5MHz |
| Resolution | 12bit | 0~4095 |
| Number Of Conversion | 2 | Rank1 = IN1, Rank2 = IN2 |
| Continuous Conversion | Enabled | 한 바퀴 끝나면 자동 반복 |
| DMA Continuous Requests | Enabled | DMA 요청을 계속 보냄 |
| Sampling Time | 640.5 cycles | 10kΩ 풀업처럼 출력 임피던스가 큰 센서용. DMA 인터럽트 횟수도 줄어듦 |
| DMA | DMA1 Channel1, Circular, Half Word | `buf[0]`, `buf[1]`을 계속 덮어씀 |

## 동작 흐름

```
초기화
 ├─ HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED)
 └─ HAL_ADC_Start_DMA(&hadc1, (uint32_t *)buf, 2)   ← 변환 시작까지 같이 함

ADC1 (CPU 개입 없음)
 └─ IN1 → DR → DMA → buf[0]
    IN2 → DR → DMA → buf[1]
    IN1 → ...  (Continuous + Circular로 계속 반복)

main의 while(1)
 ├─ snprintf(uart_buf, sizeof(uart_buf), "%d %d\r\n", buf[0], buf[1])
 ├─ HAL_UART_Transmit(&huart1, uart_buf, snprintf 반환값, 100)
 └─ HAL_Delay(100)
```

출력 예시 (센서 1, 센서 2):

```
212 3905
215 3898
```

`main.c` 주석에 회로, CubeMX 설정, `volatile`, `sizeof`와 `snprintf` 반환값의 차이 등 실습하면서 헷갈렸던 내용을 정리해 두었다.

## 빌드 방법

1. STM32CubeIDE에서 `File > Import > Existing Projects into Workspace`로 이 폴더를 불러온다.
2. Build 후 NUCLEO-G474RE에 Run으로 올린다.
3. PC에서 블루투스 COM 포트를 Tera Term으로 연다 (9600bps).
