# Step_Line_Tracer

라인트레이서 본 프로젝트이다. TCRT5000 8개로 라인 위치를 구하고, PID로 좌우 스텝모터 속도를 정한다.
**코드는 빌드까지 확인했고, 하드웨어 주행은 아직 해 보지 않았다** (2026-11-19 기준).

배선 전체 그림: [Step_Line_Tracer_배선도.svg](../../자료/Step_Line_Tracer_배선도.svg)

## 전체 흐름

```
TCRT5000 8개 ── ADC1/ADC2 + DMA (CPU 없이 계속 갱신)
   │
tcrt5000.c   get_current_pos()      정규화(0~1000) → 가장 검은 센서와 양옆 3개로 가중평균 → 라인 위치(mm)
   │
pid.c        pid_task(dt_ms)        오차 = 0 - 위치 → P + I + D → gain(좌우 속도 차이, Hz)
   │                                LEFT = base_hz - gain, RIGHT = base_hz + gain
   │
pid.c        Stepper_RampTo()       목표 Hz를 향해 주기당 최대 accel_max_hz씩 (탈조 방지)
   │
stepper.c    Stepper_SetSpeed()     Hz → ARR, DIR 처리 → TIM1 / TIM3 PWM → DRV8825 STEP
```

## 파일

| 파일 | 내용 |
|---|---|
| [stepper.c/h](Core/Src/stepper.c) | 모터 2개 구동. Hz(float)를 받아 ARR, CCR, DIR 설정. 하드웨어만 담당 |
| [pid.c/h](Core/Src/pid.c) | PID 계산(`update_gain`), 주기 함수(`pid_task`), 가속 제한(`Stepper_RampTo`), 튜닝 변수 |
| [tcrt5000.c/h](Core/Src/tcrt5000.c) | ADC 보정 + DMA 시작, 센서 표(위치 mm, 버퍼 칸), 흰/검 보정 표, 정규화, 라인 위치 |
| [main.c](Core/Src/main.c) | 초기화, `MOTOR_TEST` (모터만 앞뒤로 돌리는 시험 모드) |

## 핀

| 신호 | 핀 | 설정 |
|---|---|---|
| 왼쪽 STEP | PA8 (D7) | TIM1_CH1, PWM |
| 왼쪽 DIR | PB10 (D6) | GPIO (`LM_Dir`) |
| 오른쪽 STEP | PC7 (D9) | TIM3_CH2, PWM |
| 오른쪽 DIR | PB4 (D5) | GPIO (`RM_Dir`) |
| EN (두 드라이버 공통) | PB5 (D4) | GPIO (`Motor_en`), LOW = 동작, 부팅 시 HIGH |
| 센서 S1~S4 | PB14, PB12, PB1, PB0 | ADC1 IN5, IN11, IN12, IN15 |
| 센서 S5~S8 | PA4, PB2, PC5, PC4 | ADC2 IN17, IN12, IN11, IN5 (버퍼 역순) |
| 블루투스 | PA9 TX, PA10 RX | USART1, 115200 bps |
| 버튼 | PC13 | EXTI13 Rising |
| LED | PA5 | GPIO (`State_LED`) |

- 타이머: PSC 169 → 1 MHz tick, ARR/CCR preload ON
- DRV8825: 1/8 마이크로스텝 (M0, M1 → 3.3V, M2 → GND), 1600 스텝/회전

## 튜닝값 (금요일에 조정)

| 값 | 위치 | 지금 | 비고 |
|---|---|---|---|
| `k_p`, `k_i`, `k_d` | pid.h `PID_MOTOR_K*_DEFAULT` | 100 / 0 / 0 | 센서 위치를 mm로 바꿔서 Kp는 약 12.7로 낮춰야 함 |
| `base_hz` | pid.c `PID_MOTOR_BASE_HZ_DEFAULT` | 2000 Hz | 바퀴 65mm 기준 약 0.25 m/s |
| `PID_GAIN_MAX` | pid.c | 1000 Hz | 좌우 속도 차이 상한 |
| `k_acc`, `accel_max_hz` | pid.h | 1.0 / 200 Hz | 10 ms 제어 주기 기준 |
| `tcrt_min_max` | tcrt5000.c | 전부 100 / 2000 | 트랙에서 센서별 흰/검 raw 값을 재서 채움 |
| `TCRT_LINE_TH` | tcrt5000.c | 300 | 라인 판정 기준 (정규화 값) |

## 남은 일

- [ ] main 루프: 10 ms마다 `pid_task(10)`, `MOTOR_TEST` 0으로
- [ ] 버튼으로 IDLE ⇄ RUN 전환 (EN 켜고 끄기, PID 상태 리셋)
- [ ] 라인을 놓치면 200 ms 동안 마지막 방향으로 꺾다가 정지 (pid로 옮길 예정)
- [ ] 교차로 처리 (검정 센서가 많으면 8개 전부로 평균), 정지선 감지
- [ ] 하드웨어 시험: 모터 방향, 센서 raw 값, P 제어만으로 저속 주행
