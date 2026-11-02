# Line Tracer

NUCLEO-G474RE 보드와 스텝모터로 라인트레이서를 만드는 프로젝트이다.
지금은 부품을 하나씩 시험하는 단계이며, 첫 실험으로 28BYJ-48 스텝모터를 구동해 보았다.

- 보드: NUCLEO-G474RE (STM32G474RE, SYSCLK 170MHz)
- 개발 환경: STM32CubeIDE (HAL)
- 구동 모터: KH42JM2-901 2개
- 스테퍼 드라이버: DRV8825
- 라인 센서: TCRT5000 적외선 광센서 어레이

## 폴더 구성

| 폴더 | 내용 |
|---|---|
| [Project_code](Project_code) | STM32CubeIDE 프로젝트 |
| [실험_사진](실험_사진) | 실험 사진 축소본 (원본 사진과 동영상 `*.mp4`는 용량 때문에 저장소에 올리지 않음) |
| [자료](자료) | 데이터시트, 구매 체크리스트, 참고 라이브러리, 학습용 시뮬레이션 HTML |

## 프로젝트 목록

### step_motor

버튼(PC13)을 누르면 28BYJ-48 스텝모터를 `STEPS_PER_PRESS`(현재 2048칸, 약 1바퀴)만큼 돌린다.
세부 내용은 [step_motor README](Project_code/step_motor/README.md), 실험 기록은 [28_BYJ_48_Test README](실험_사진/28_BYJ_48_Test/README.md)를 참고.

```
버튼 입력 (PC13, Rising edge)
 └─ EXTI15_10_IRQHandler → HAL_GPIO_EXTI_IRQHandler → HAL_GPIO_EXTI_Callback
     └─ 50ms 채터링 필터를 통과하면 step_request = 1

main의 while(1)
 └─ step_request가 1이면 0으로 지우고
     └─ step_table을 한 칸씩 출력 (HAL_Delay(10) 간격, 2048회)
```

모터 구동은 2상 여자 풀스텝 방식이다. 코일 패턴 `0x3 → 0x6 → 0xC → 0x9`를 순서대로 출력한다.
콜백에서 직접 돌리지 않는 이유는 [main.c](Project_code/step_motor/Core/Src/main.c) 상단 주석에 정리되어 있다.

| 신호 | 핀 | 설정 |
|---|---|---|
| IN1 | PA8 | Output Push Pull |
| IN2 | PA9 | Output Push Pull |
| IN3 | PA10 | Output Push Pull |
| IN4 | PB5 | Output Push Pull |
| 버튼 | PC13 | EXTI13 Rising edge |

- 모터 드라이버: ULN2003
- 실험 영상: [YouTube](https://www.youtube.com/shorts/aOwYWziigPU)

## 빌드 방법

1. STM32CubeIDE에서 `File > Import > Existing Projects into Workspace`로 `Project_code/step_motor`를 불러온다.
2. Build 후 NUCLEO-G474RE에 Debug 또는 Run으로 올린다.

`Debug/` 폴더, `.mxproject` 등 IDE가 다시 만드는 파일은 저장소에 포함하지 않는다.

## 진행 상황

- [x] 28BYJ-48 + ULN2003 구동 코드 작성 및 실험 (2026-11-02)
- [ ] DRV8825로 KH42JM2-901 구동
- [ ] TCRT5000 센서 어레이 구성과 다채널 ADC 읽기
- [ ] 전원부(3S LiPo, 5V 강압) 구성
- [ ] 차체 제작
- [ ] 라인 추종 제어 구현

## 참고 자료

전체 목록은 [자료 README](자료/README.md)를 참고.

- [KH42J.pdf](자료/KH42J.pdf): KH42 스텝모터 데이터시트
- [drv8825.pdf](자료/drv8825.pdf): DRV8825 스테퍼 드라이버 데이터시트
- [tcrt5000.pdf](자료/tcrt5000.pdf): TCRT5000 광센서 데이터시트
- [라인트레이서 추가 구매 체크리스트](<자료/라인트레이서 추가 구매 체크리스트.pdf>): 필요한 부품 목록
- [Arduino_Stepper](자료/Arduino_Stepper): Arduino Stepper 라이브러리 (구동 방식 참고용)
