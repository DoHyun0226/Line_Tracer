# step_motor

버튼을 누르면 **28BYJ-48 스텝모터**를 약 1바퀴 돌리는 테스트 프로젝트이다.
직접 작성한 코드는 모두 `Core/Src/main.c`에 있고, 핀 설정은 `step_motor.ioc`로 만들었다.

실험 사진과 영상은 [28_BYJ_48_Test](../../실험_사진/28_BYJ_48_Test)에 정리되어 있다.

## 하드웨어

- 보드: NUCLEO-G474RE
- 모터: 28BYJ-48
- 모터 드라이버: ULN2003

| 신호 | 핀 | 설정 |
|---|---|---|
| IN1 | PA8 | Output Push Pull, Medium speed |
| IN2 | PA9 | Output Push Pull, Medium speed |
| IN3 | PA10 | Output Push Pull, Medium speed |
| IN4 | PB5 | Output Push Pull, Medium speed |
| 버튼 | PC13 | EXTI13 Rising edge, 내부 풀업/풀다운 없음 |

## 코드 구성

| 이름 | 역할 |
|---|---|
| `step_table[4]` | 코일 구동 패턴 `0x3, 0x6, 0xC, 0x9`. bit0~3이 IN1~IN4에 대응 |
| `step_index` | `step_table`에서 현재 출력 중인 위치 |
| `step_request` | 버튼 콜백이 1로 만들고 `main`이 지우는 요청 표시 (`volatile`) |
| `Motor_Write()` | 패턴의 각 비트를 IN1~IN4 핀에 출력 |
| `HAL_GPIO_EXTI_Callback()` | 채터링을 걸러 낸 뒤 `step_request`만 1로 만듦 |

버튼 인터럽트 안에서는 모터를 돌리지 않는다. `HAL_Delay()`가 SysTick 인터럽트로 시간을 재는데, EXTI 우선순위(0)가 SysTick(15)보다 높아서 콜백 안에서는 시간이 흐르지 않기 때문이다.

## 설정값

| 항목 | 현재 값 | 위치 |
|---|---|---|
| 버튼 한 번에 도는 칸 수 | 2048 (약 1바퀴) | `STEPS_PER_PRESS` |
| 칸 사이 대기 시간 | `HAL_Delay(10)` | `main()`의 `while(1)` |
| 채터링 무시 시간 | 50ms | `HAL_GPIO_EXTI_Callback()` |

`HAL_Delay()`는 최소 대기 시간을 보장하려고 1틱을 더 기다리므로 한 칸에 약 11ms가 걸린다.
계산상 1바퀴에는 `2048 × 11ms ≈ 22.5초`가 걸린다.

## 개선할 점 / 주의사항

- **코일이 계속 켜져 있음**: 회전이 끝나도 마지막 패턴이 그대로 출력된다. 모터를 세워 둘 때 열이 나지 않게 하려면 회전 후 `Motor_Write(0)`으로 코일을 끈다.
- **회전 중 버튼 입력**: 도는 동안 버튼을 누르면 `step_request`가 다시 1이 되어, 회전이 끝난 뒤 한 번 더 돈다.
- **회전 중 다른 일을 못 함**: `HAL_Delay()`로 기다리는 동안 메인 루프가 멈춘다. 라인트레이서에서는 센서를 읽으면서 모터를 돌려야 하므로 타이머로 스텝을 내보내는 방식이 필요하다.
- **CubeMX 코드 재생성**: `USER CODE BEGIN`과 `USER CODE END` 사이의 코드만 유지된다.
