# vtol_control_nav

PX4 Offboard 제어 기반 VTOL 웨이포인트 미션 비행 패키지.
이륙 → 고정익 전환 → 웨이포인트 순항 → 멀티콥터 전환 → (정밀 착륙) → 착륙까지의 전 과정을 상태 머신으로 관리합니다.

---

## 목차

1. [패키지 구조](#1-패키지-구조)
2. [아키텍처 개요](#2-아키텍처-개요)
3. [상태 머신](#3-상태-머신)
4. [ROS2 인터페이스](#4-ros2-인터페이스)
5. [파라미터 전체 목록](#5-파라미터-전체-목록)
6. [미션 플래너 (MissionPlanner)](#6-미션-플래너-missionplanner)
7. [궤적 플래너 (Trajectory Planners)](#7-궤적-플래너-trajectory-planners)
8. [확장 포인트](#8-확장-포인트)
9. [장애 대응 (Fault Handling)](#9-장애-대응-fault-handling)
10. [착륙 확인 로직](#10-착륙-확인-로직)
11. [빌드 및 설치](#11-빌드-및-설치)
12. [테스트](#12-테스트)
13. [파라미터 튜닝 가이드](#13-파라미터-튜닝-가이드)

---

## 1. 패키지 구조

```
vtol_control_nav/
├── CMakeLists.txt
├── package.xml
├── README.md
└── src/
    ├── waypoint_nav_node.py          # 메인 노드 (상태 머신, 미션 제어)
    ├── trajectory_planners/          # 궤적 플래너 (플러그인 구조)
    │   ├── __init__.py
    │   ├── base.py                   # 추상 인터페이스
    │   ├── factory.py                # 플래너 생성 팩토리
    │   ├── simple.py                 # PointJump / Linear / Smoothstep
    │   ├── mpc_planner.py            # 배치 MPC 플래너 (이중 적분기 모델)
    │   └── mpc_stub.py               # 하위 호환 재익스포트
    ├── precision_landing/            # [확장] 정밀 착륙 컨트롤러 인터페이스
    │   ├── __init__.py
    │   └── base.py                   # BasePrecisionLandingController
    └── mission_executor/             # [확장] 웨이포인트 임무 실행기 인터페이스
        ├── __init__.py
        └── base.py                   # BaseMissionExecutor
```

---

## 2. 아키텍처 개요

```
┌─────────────────────────────────────────────────────────┐
│                  WaypointNavNode (10 Hz)                 │
│                                                          │
│  ┌─────────────┐   ┌──────────────┐   ┌──────────────┐  │
│  │MissionPlanner│  │TrajectoryPlanner│ │ State Machine│  │
│  │  GPS → NED  │  │  next_setpoint │ │  10 States   │  │
│  │  파싱·검증   │  │  (pluggable)  │ │  handler map │  │
│  └─────────────┘  └──────────────┘  └──────────────┘  │
│         ↑                 ↑                  ↑           │
│   vtol.waypoints   vtol.trajectory     VehicleStatus     │
│   (ROS params)     .type / .mpc        VehicleLocalPos   │
└─────────────────────────────────────────────────────────┘
         │ publisher                       │ subscriber
   ┌─────▼──────────────┐         ┌────────▼────────────┐
   │ OffboardControlMode │         │   VehicleStatus      │
   │ TrajectorySetpoint  │         │ VehicleLocalPosition │
   │ VehicleCommand      │         │  VehicleLandDetected │
   └────────────────────┘         └─────────────────────┘
                    PX4 uXRCE-DDS (/{drone_id}/fmu/...)
```

**핵심 설계 원칙:**
- **상태 머신 + 핸들러 맵**: 각 상태의 로직이 독립 메서드로 분리되어 추가·수정이 쉬움
- **플래너 플러그인**: `BaseTrajectoryPlanner`를 상속하면 모드 전환 없이 새 플래너 삽입 가능
- **외부 의존성 없음**: numpy·scipy 없이 순수 Python으로 동작 (ROS2 환경 제약 대응)

---

## 3. 상태 머신

### 정상 비행 흐름

```
  ┌────────┐  10사이클   ┌─────────┐  armed   ┌──────────┐  고도 도달
  │  IDLE  │ ──────────▶│ ARMING  │ ────────▶│ TAKEOFF  │ ──────────▶
  └────────┘            └─────────┘          └──────────┘
                                                               ↓ FW전환 명령
  ┌──────────────┐  ctrl 설정   ┌──────────────────┐  MC모드/타임아웃
  │PRECISION_LAND│ ◀─────────── │ TRANSITION_TO_MC │ ◀──────────────── ...
  │  [확장 상태] │  없으면 skip  └──────────────────┘
  └──────┬───────┘ (→ LAND)              ↑
         │ 완료                  마지막 WP 도달
         ▼                               │
       ┌──────┐              ┌───────────────────┐  ┌──────────────────┐
       │ LAND │◀─────────────│     NAVIGATE      │◀─│ TRANSITION_TO_FW │
       └──┬───┘              └───────────┬───────┘  └──────────────────┘
          │ 즉시                  WP 도달 │ (executor 설정 시)
          ▼                              ▼
  ┌──────────────────┐        ┌────────────────┐
  │ LANDING_CONFIRM  │        │  MISSION_EXEC  │ 완료 후 → 다음 WP
  └────────┬─────────┘        │  [확장 상태]   │
           │ 착륙 확인 후      └────────────────┘
           ▼
        ┌──────┐
        │ DONE │
        └──────┘
```

> **[확장 상태]** 로 표시된 `PRECISION_LAND`, `MISSION_EXEC` 는 현재 pass-through 상태입니다.
> 컨트롤러/실행기 미할당 시 즉시 다음 상태로 전이되며, 기존 동작에 영향이 없습니다.

### 장애 발생 시

```
  [임의 상태] ──(GPS 오류 / 통신 끊김 / 알 수 없는 상태)──▶
  ┌───────────────┐         ┌──────────────────┐         ┌──────┐
  │ FAILSAFE_LAND │ ───────▶│ LANDING_CONFIRM  │ ───────▶│ DONE │
  └───────────────┘         └──────────────────┘         └──────┘
```

### 상태별 동작 요약

| 상태 | 진입 조건 | 동작 | 전이 조건 |
|------|-----------|------|-----------|
| `IDLE` | 시작 | offboard 모드 준비, takeoff_z 셋포인트 송출 | 10 사이클 경과 |
| `ARMING` | IDLE 10사이클 | arm + offboard 명령 전송 | `arming_state == ARMED` |
| `TAKEOFF` | 무장 완료 | takeoff_z 셋포인트 유지 | 이륙 고도 0.5 m 이내 도달 |
| `TRANSITION_TO_FW` | 이륙 고도 도달 | VTOL→FW 명령, 첫 WP 홀드 | FW 모드 확인 또는 8 s 타임아웃 |
| `NAVIGATE` | FW 전환 완료 | 궤적 플래너 경유 WP 추종 | WP 도달(→MISSION_EXEC 또는 다음WP) 또는 타임아웃 스킵 |
| `MISSION_EXEC` | WP 정상 도달 | WP 위치 홀드 + 임무 실행기 tick | `is_complete()` 또는 실행기 없음 |
| `TRANSITION_TO_MC` | 모든 WP 완료 | VTOL→MC 명령, 마지막 WP 홀드 | MC 모드 확인 또는 8 s 타임아웃 |
| `PRECISION_LAND` | MC 전환 완료 | 컨트롤러 tick, 정밀 위치 유지 | `is_complete()` 또는 컨트롤러 없음 |
| `LAND` | PRECISION_LAND 완료 또는 skip | NAV_LAND 명령 전송 | 즉시 LANDING_CONFIRM으로 |
| `LANDING_CONFIRM` | LAND / FAILSAFE_LAND | 착륙 여부 폴링 | 착륙 확인 1 s 유지 |
| `FAILSAFE_LAND` | 임의 상태(예외) | NAV_LAND 명령 전송(1회) | 즉시 LANDING_CONFIRM으로 |
| `DONE` | 착륙 확인 완료 | 아무것도 하지 않음 | — |

> **FAILSAFE_LAND 트리거 예외**: `DONE`, `LAND`, `LANDING_CONFIRM`, `FAILSAFE_LAND` 상태에서는 failsafe가 재진입하지 않습니다.

---

## 4. ROS2 인터페이스

모든 토픽은 `/{drone_id}/fmu/...` 네임스페이스를 사용합니다 (`drone_id` 기본값: `drone1`).

### 퍼블리셔

| 토픽 | 메시지 타입 | 설명 |
|------|------------|------|
| `/{id}/fmu/in/offboard_control_mode` | `px4_msgs/OffboardControlMode` | 위치 제어 모드 활성화 (10 Hz) |
| `/{id}/fmu/in/trajectory_setpoint` | `px4_msgs/TrajectorySetpoint` | 위치 셋포인트 (NED, m) |
| `/{id}/fmu/in/vehicle_command` | `px4_msgs/VehicleCommand` | ARM / 모드전환 / VTOL전환 / 착륙 명령 |

### 서브스크라이버

| 토픽 | 메시지 타입 | 설명 |
|------|------------|------|
| `/{id}/fmu/out/vehicle_status` | `px4_msgs/VehicleStatus` | 무장 상태, 기체 타입(FW/MC) |
| `/{id}/fmu/out/vehicle_local_position` | `px4_msgs/VehicleLocalPosition` | 현재 위치·속도 (NED, m / m/s) |
| `/{id}/fmu/out/vehicle_land_detected` | `px4_msgs/VehicleLandDetected` | 착륙 감지 플래그 |

### QoS 프로파일 (PX4 uXRCE-DDS 필수)

```python
QoSProfile(
    reliability = BEST_EFFORT,
    durability  = TRANSIENT_LOCAL,
    history     = KEEP_LAST,
    depth       = 1,
)
```

---

## 5. 파라미터 전체 목록

`global_params.yaml` 또는 launch 파일에서 설정합니다. 모든 키는 `vtol.` 네임스페이스를 사용합니다.

### 기본 비행

| 파라미터 | 타입 | 기본값 | 설명 |
|---------|------|--------|------|
| `vtol.drone_id` | string | `"drone1"` | PX4 토픽 네임스페이스 prefix |
| `vtol.takeoff_altitude` | float | `5.0` | 이륙 목표 고도 (m, 지상 기준 양수) |
| `vtol.cruise_altitude` | float | `30.0` | 순항 고도 (m, GPS WP의 alt 값 대체에 사용) |

### 웨이포인트 미션

| 파라미터 | 타입 | 기본값 | 설명 |
|---------|------|--------|------|
| `vtol.waypoint_frame` | string | `"gps"` | `gps` (lat/lon/alt) 또는 `local_ned` (N/E/D m) |
| `vtol.waypoints` | list | `[]` | WP 목록. GPS: `[lat, lon, alt_m]`, NED: `[north_m, east_m, down_m]` |
| `vtol.dynamic_waypoint_update` | bool | `true` | 런타임 파라미터 변경 시 미션 자동 재로드 |
| `vtol.dynamic_reload_period_sec` | float | `1.0` | 재로드 확인 주기 (s) |

### 판정 임계값

| 파라미터 | 타입 | 기본값 | 설명 |
|---------|------|--------|------|
| `vtol.waypoint_reached_threshold` | float | `2.0` | WP 도달 판정 반경 (m) |
| `vtol.transition_timeout_sec` | float | `8.0` | FW↔MC 전환 확인 타임아웃 (s) |
| `vtol.waypoint_unreachable_timeout_sec` | float | `45.0` | WP 도달 불가 판단 후 스킵 시간 (s) |
| `vtol.comm_loss_timeout_sec` | float | `2.0` | 토픽 미수신 시 failsafe 진입 시간 (s) |
| `vtol.gps_error_failsafe` | bool | `true` | GPS WP 오류 시 failsafe 진입 여부 |

### 궤적 플래너

| 파라미터 | 타입 | 기본값 | 설명 |
|---------|------|--------|------|
| `vtol.trajectory.type` | string | `""` | 플래너 선택 (우선). 빈 문자열이면 아래 키 사용 |
| `vtol.trajectory_mode` | string | `"smoothstep"` | 하위 호환 플래너 키 (`point_jump` / `linear` / `smoothstep` / `mpc`) |
| `vtol.trajectory_max_step_m` | float | `2.0` | 한 제어 주기 최대 이동 거리 (m) |

### MPC 파라미터 (`vtol.trajectory.type = "mpc"` 시 활성)

| 파라미터 | 타입 | 기본값 | 설명 |
|---------|------|--------|------|
| `vtol.mpc.horizon_steps` | int | `15` | 예측 수평 N (스텝 수) |
| `vtol.mpc.dt` | float | `0.1` | 제어 주기 (s). 노드 주기와 일치시킬 것 |
| `vtol.mpc.w_pos` | float | `1.0` | 위치 오차 가중치 (클수록 빠른 수렴) |
| `vtol.mpc.w_vel` | float | `0.2` | 속도 감쇠 가중치 (클수록 오버슈트 억제) |
| `vtol.mpc.w_u` | float | `0.05` | 가속도 입력 가중치 (클수록 부드러운 기동, 반드시 > 0) |

### 착륙 확인

| 파라미터 | 타입 | 기본값 | 설명 |
|---------|------|--------|------|
| `vtol.landing_confirm_alt_threshold` | float | `0.4` | 저고도 판정 기준 (m, kinematic fallback 전용) |
| `vtol.landing_confirm_speed_threshold` | float | `0.5` | 저속 판정 기준 (m/s, kinematic fallback 전용) |
| `vtol.landing_confirm_hold_sec` | float | `1.0` | 착륙 확인 유지 시간 (s) |
| `vtol.landing_confirm_use_kinematic_fallback` | bool | `false` | 저고도+저속 기반 착륙 판정 활성화 |

---

## 6. 미션 플래너 (MissionPlanner)

`MissionPlanner` 클래스가 파라미터 기반 웨이포인트를 내부 NED 미션 리스트로 변환합니다.

### 처리 흐름

```
vtol.waypoints (raw list)
        │
        ▼
  _parse_waypoints()
  · 각 항목이 길이 3 tuple/list인지 확인
  · float 변환 실패 항목 무시 (silent filter)
        │
        ▼
  waypoint_frame == 'gps' ?
  ┌──────┴──────┐
  Yes           No (local_ned)
  │             │
  ▼             ▼
_gps_waypoints_to_local_ned()   [(x, y, -abs(z)), ...]
· lat [-90,90] / lon [-180,180]  바로 사용
  범위 검사 → 유효하지 않으면 has_gps_error=True
· 첫 번째 유효 WP를 원점(0,0)으로 설정
· WGS84 지구 반경 6,378,137 m 사용
  north = Δlat_rad × R
  east  = Δlon_rad × R × cos(lat0)
  down  = -abs(alt)
        │
        ▼
  mission: list[(north, east, down)]
  has_gps_error: bool
```

### 동적 미션 리로드

`vtol.dynamic_waypoint_update = true` 설정 시 `dynamic_reload_period_sec` 주기마다 파라미터 변경을 감지합니다.

- 서명(frame + raw WP 문자열) 비교로 불필요한 재로드 방지
- 비행 중 미션이 변경되면 `_wp_idx`를 0으로 리셋하고 궤적 플래너를 초기화
- GPS 오류가 있는 경우 경고 로그 출력 후 유효한 WP만으로 미션 구성

---

## 7. 궤적 플래너 (Trajectory Planners)

`vtol.trajectory.type` 파라미터로 선택합니다.
모든 플래너는 `BaseTrajectoryPlanner`를 상속하며 `next_setpoint(current, target) → setpoint` 인터페이스를 구현합니다.

### 플래너 비교

| 모드 | 클래스 | 특징 | 권장 용도 |
|------|--------|------|-----------|
| `point_jump` | `PointJumpPlanner` | 즉시 목표로 점프 | 디버깅·시뮬 단순 테스트 |
| `linear` | `LinearPlanner` | 매 스텝 `max_step_m` 직선 이동 | 안전 기준선, 빠른 검증 |
| `smoothstep` | `SmoothstepPlanner` | Hermite 보간 S-커브 + step 제한 | **기본 추천** (부드러운 순항) |
| `mpc` | `MPCPlanner` | 배치 MPC, 이중 적분기 모델 | 속도/오버슈트 최적화 필요 시 |

### 상세: SmoothstepPlanner

세그먼트 시작점과 목표 사이에서 Hermite 보간을 수행합니다.

```
진행도 t = 1 - (현재→목표 거리) / (세그먼트 총 거리)
S-커브  s = t² (3 - 2t)
보간점  = start + (target - start) × s
출력   = move_toward(current, 보간점, max_step_m)
```

웨이포인트 변경 시 `reset_segment()`가 호출되어 새 세그먼트 시작점이 설정됩니다.

### 상세: MPCPlanner

외부 라이브러리 없이 순수 Python으로 구현된 유한 수평 LQR/MPC입니다.

**시스템 모델 (각 축 독립):**

```
s_{k+1} = A·s_k + B·u_k

A = [[1, dt ],    B = [[dt²/2],    s_k = [위치 오차]
     [0, 1  ]]         [dt   ]]          [속도     ]

u_k = 가속도 (최적화 변수)
```

**비용함수:**

```
J = Σ_{k=1}^{N} (w_pos·e_k² + w_vel·v_k²)  +  Σ_{k=0}^{N-1} w_u·u_k²
```

**사전 계산 (초기화 1회):**

```
배치 전개:   X = Phi·s₀ + Gamma·U
최적 제어:   U* = -(GᵀQ̄Gamma + R̄)⁻¹ GᵀQ̄Phi · s₀  =  -F·s₀
게인 행렬:   F (N×2) → 초기화 시 계산 후 캐싱
```

**매 스텝 연산 (O(1)):**

```
u₀ = -(F[0][0]·e₀ + F[0][1]·v₀)          # 첫 번째 최적 가속도
p_next = p + v·dt + ½·u₀·dt²              # 다음 위치 예측
출력 = move_toward(current, p_next, max_step_m)  # 상한 클램핑
```

속도 `v`는 연속된 호출 사이의 위치 차분으로 추정합니다.
`reset_segment()` 호출 시 속도 추정값이 초기화됩니다.

**파라미터 영향:**

```
w_pos ↑  →  빠른 수렴 (오버슈트 가능)
w_vel ↑  →  부드러운 감속·오버슈트 억제
w_u   ↑  →  완만한 기동 (수렴 속도 감소)
N     ↑  →  더 긴 예측 → 더 부드럽지만 초기화 비용 증가
```

> **주의**: `w_u`는 반드시 0보다 커야 합니다 (게인 행렬 정칙성 보장).
> 파라미터 변경 시 `update_config()`가 자동으로 F 행렬을 재계산합니다.

### 커스텀 플래너 추가 방법

```python
# 1. BaseTrajectoryPlanner 상속
from trajectory_planners.base import BaseTrajectoryPlanner

class MyPlanner(BaseTrajectoryPlanner):
    def next_setpoint(self, current, target):
        # ... 구현 ...
        return setpoint

# 2. factory.py에 등록
if t == 'my_planner':
    return MyPlanner(...)

# 3. 파라미터로 선택
# vtol.trajectory.type: "my_planner"
```

---

## 8. 확장 포인트

`waypoint_nav_node.py` 는 두 가지 선택적 확장 슬롯을 제공합니다.
두 슬롯 모두 기본값이 `None` 이며, 할당하지 않으면 기존 동작과 완전히 동일합니다.

---

### 8-1. 정밀 착륙 (`PRECISION_LAND`)

```
TRANSITION_TO_MC 완료
    → _precision_landing_ctrl is not None  →  PRECISION_LAND
    → _precision_landing_ctrl is None      →  LAND (기존 동작 유지)
```

**인터페이스 파일**: [src/precision_landing/base.py](src/precision_landing/base.py)

```python
class BasePrecisionLandingController(ABC):
    def reset(self) -> None: ...         # PRECISION_LAND 진입 시 1회
    def tick(self, local_pos) -> tuple | None: ...  # 매 루프, 위치 셋포인트 반환
    def is_complete(self) -> bool: ...   # True → LAND 로 전이
```

**구현 및 주입 예시 (ArUco 마커 기반):**

```python
from precision_landing import BasePrecisionLandingController

class ArucoPrecisionLander(BasePrecisionLandingController):
    def __init__(self, node, marker_id: int):
        self._node = node
        self._marker_id = marker_id
        self._done = False

    def reset(self):
        self._done = False

    def tick(self, local_pos):
        # 마커 감지 → 수평 오프셋 계산 → 보정 셋포인트 반환
        offset = self._detect_marker_offset()
        if offset is None:
            return None   # 마커 미감지: 현재 위치 홀드
        x = local_pos.x + offset[0]
        y = local_pos.y + offset[1]
        z = local_pos.z + 0.05       # 서서히 하강
        if abs(local_pos.z) < 1.5 and abs(offset[0]) < 0.3 and abs(offset[1]) < 0.3:
            self._done = True
        return (x, y, z)

    def is_complete(self):
        return self._done

# 노드에 주입
node = WaypointNavNode()
node._precision_landing_ctrl = ArucoPrecisionLander(node, marker_id=0)
```

**관련 파라미터** (`global_params.yaml`에 이미 정의됨):
- `vtol.aruco_marker_id` — 착륙 지점 마커 ID

---

### 8-2. 웨이포인트 임무 실행 (`MISSION_EXEC`)

```
NAVIGATE (WP 정상 도달)
    → _mission_executor is not None  →  MISSION_EXEC  →  다음 WP
    → _mission_executor is None      →  다음 WP (기존 동작 유지)
```

> **주의**: WP 도달 타임아웃으로 스킵된 경우에는 `MISSION_EXEC` 에 진입하지 않습니다.
> 정상 도달한 WP에서만 임무가 실행됩니다.

**인터페이스 파일**: [src/mission_executor/base.py](src/mission_executor/base.py)

```python
class BaseMissionExecutor(ABC):
    def reset(self) -> None: ...                   # WP 도달 시 1회
    def tick(self, wp_idx, waypoint, local_pos) -> None: ...  # 매 루프
    def is_complete(self) -> bool: ...             # True → 다음 WP 로 전이
```

**구현 및 주입 예시 (그리퍼 투하):**

```python
from mission_executor import BaseMissionExecutor

class GripperDropExecutor(BaseMissionExecutor):
    def __init__(self, gripper_pub, hold_sec: float = 2.0, ctrl_dt: float = 0.1):
        self._pub = gripper_pub
        self._required_cycles = int(hold_sec / ctrl_dt)
        self._cnt = 0

    def reset(self):
        self._cnt = 0
        self._pub.publish(GripperCommand(open=True))   # 투하

    def tick(self, wp_idx, waypoint, local_pos):
        self._cnt += 1

    def is_complete(self):
        return self._cnt >= self._required_cycles

# 노드에 주입
node = WaypointNavNode()
node._mission_executor = GripperDropExecutor(gripper_pub, hold_sec=2.0)
```

**관련 파라미터** (`global_params.yaml`에 이미 정의됨):
- `vtol.gripper_open_angle` / `vtol.gripper_close_angle`

---

### 8-3. 확장 슬롯 위치 (코드 참조)

| 슬롯 | 파일:라인 | 설명 |
|------|-----------|------|
| `_precision_landing_ctrl` | [waypoint_nav_node.py](src/waypoint_nav_node.py) | `__init__` 에서 선언, `_handle_transition_to_mc` 에서 분기 |
| `_mission_executor` | [waypoint_nav_node.py](src/waypoint_nav_node.py) | `__init__` 에서 선언, `_handle_navigate` 에서 분기 |
| `_handle_precision_land()` | [waypoint_nav_node.py](src/waypoint_nav_node.py) | pass-through 구현 참고 |
| `_handle_mission_exec()` | [waypoint_nav_node.py](src/waypoint_nav_node.py) | pass-through 구현 참고 |

---

## 9. 장애 대응 (Fault Handling)

매 제어 루프에서 `_check_faults_and_enter_failsafe()`가 아래 조건을 순서대로 검사합니다.
조건이 충족되면 즉시 `FAILSAFE_LAND`로 전환하고 해당 루프를 종료합니다.

| 우선순위 | 트리거 조건 | 파라미터 | 비고 |
|---------|------------|---------|------|
| 1 | GPS WP 오류 검출 (`has_gps_error=True`) | `vtol.gps_error_failsafe` | `IDLE` 상태에서는 미적용 |
| 2 | `vehicle_status` 토픽 수신 중단 | `vtol.comm_loss_timeout_sec` | 최초 수신 후부터 카운팅 |
| 3 | `vehicle_local_position` 토픽 수신 중단 | `vtol.comm_loss_timeout_sec` | 최초 수신 후부터 카운팅 |

**WP 도달 불가 타임아웃** (`_handle_navigate` 내부):
- 현재 WP에서 `waypoint_unreachable_timeout_sec` 이상 진행 없으면 다음 WP로 스킵
- 마지막 WP였다면 `TRANSITION_TO_MC` → 착륙 절차로 전환

**failsafe 진입 면제 상태**: `DONE`, `LAND`, `LANDING_CONFIRM`, `FAILSAFE_LAND`

---

## 10. 착륙 확인 로직

`_is_landed()` 메서드는 아래 우선순위로 착륙 여부를 판정합니다.

```
1순위: VehicleLandDetected.landed == True          (PX4 1차 신호)
    ↓ (1순위 해당 없을 때)
2순위: VehicleStatus.arming_state == STANDBY       (디스암 감지)
    ↓ (2순위 해당 없을 때)
3순위: (옵션) 저고도 + 저속 kinematic fallback
       |z| ≤ landing_confirm_alt_threshold (0.4 m)
       ||v|| ≤ landing_confirm_speed_threshold (0.5 m/s)
       → vtol.landing_confirm_use_kinematic_fallback = true 시에만 활성
```

> **kinematic fallback이 기본 비활성인 이유**: 저고도 호버링(촬영, 마커 탐색 등) 중 오착륙 판정을 방지하기 위해서입니다.

착륙 확인 후 `landing_confirm_hold_sec`(기본 1 s) 동안 착륙 상태가 유지되어야 `DONE`으로 전이합니다.
유지 중 착륙 취소(예: 바운싱)가 감지되면 카운터가 초기화됩니다.

---

## 11. 빌드 및 설치

### 의존성

```xml
<!-- package.xml -->
<depend>rclpy</depend>
<depend>px4_msgs</depend>
<depend>std_msgs</depend>
<depend>geometry_msgs</depend>
<depend>nav_msgs</depend>
```

### 빌드

```bash
cd ~/skyedge-vtol-ws
colcon build --packages-select vtol_control_nav
source install/setup.bash
```

### 노드 실행 (단독)

```bash
ros2 run vtol_control_nav waypoint_nav_node \
  --ros-args \
  --params-file src/vtol_bringup/config/global_params.yaml
```

### 설치 경로

`CMakeLists.txt`에서 다음 두 항목이 `lib/vtol_control_nav/`에 설치됩니다.

```cmake
install(PROGRAMS src/waypoint_nav_node.py ...)
install(DIRECTORY src/trajectory_planners/ ...)
```

---

## 12. 테스트

### 테스트 실행

```bash
# 단위 + 계약 테스트 (ROS2 불필요)
python3 -m unittest discover -s test/unit    -v
python3 -m unittest discover -s test/contract -v

# 통합 테스트 (시뮬레이터 실행 필요)
SIM_RUNNING=1 python3 -m unittest discover -s test/integration -v
```

### 테스트 구조

| 파일 | 분류 | 테스트 수 | 설명 |
|------|------|-----------|------|
| `test/unit/test_waypoint_nav.py` | 단위 | 33 | 노드 초기화, 상태 머신, WP 파싱, 플래너, 착륙 판정 |
| `test/contract/test_spec_contract.py` | 계약 | 7 | 스펙 문서·설정·소스 코드 일치성 검증 |
| `test/integration/test_tc003_takeoff.py` | 통합 | 4 | 시뮬 이륙 시나리오 (TC-003) |
| `test/integration/test_tc007_full_mission.py` | 통합 | 3 | 시뮬 전체 미션 시나리오 (TC-007) |

### 주요 단위 테스트 항목

- `TestStateMachine`: IDLE→ARMING→TAKEOFF→FW→NAVIGATE→MC→LAND→DONE 전 전이
- `TestStateMachine.test_comm_loss_enters_failsafe`: 통신 끊김 → FAILSAFE_LAND
- `TestStateMachine.test_waypoint_unreachable_skips_waypoint`: 타임아웃 WP 스킵
- `TestTrajectoryPlanner.test_mpc_moves_toward_target_bounded`: MPC 출력 상한 준수
- `TestLandingCondition.test_not_landed_on_low_hover_by_default`: kinematic fallback 기본 비활성

---

## 13. 파라미터 튜닝 가이드

### 비행 성능 튜닝

**빠른 WP 추종이 필요할 때:**
```yaml
vtol.waypoint_reached_threshold: 3.0    # 도달 반경을 늘려 조기 전진
vtol.trajectory_max_step_m: 3.0         # 스텝 상한 확대
vtol.trajectory.type: "linear"          # 최대 속도로 직선 이동
```

**부드러운 궤적이 필요할 때 (기본):**
```yaml
vtol.trajectory.type: "smoothstep"
vtol.trajectory_max_step_m: 2.0
```

**속도 오버슈트 억제가 필요할 때:**
```yaml
vtol.trajectory.type: "mpc"
vtol.mpc.w_pos: 1.0      # 수렴 속도
vtol.mpc.w_vel: 0.5      # 높을수록 감속 부드러움 (기본 0.2보다 높게)
vtol.mpc.w_u: 0.1        # 높을수록 가속도 완만 (기본 0.05보다 높게)
```

### 장애·안전 튜닝

**통신 환경이 불안정할 때:**
```yaml
vtol.comm_loss_timeout_sec: 5.0         # failsafe 진입 시간을 늘림
```

**GPS가 불안정한 환경에서 테스트할 때:**
```yaml
vtol.gps_error_failsafe: false          # GPS 오류에도 계속 비행 (주의)
vtol.waypoint_frame: "local_ned"        # NED 프레임 사용으로 GPS 의존 제거
```

**WP 도달 실패를 빠르게 스킵하고 싶을 때:**
```yaml
vtol.waypoint_unreachable_timeout_sec: 15.0   # 기본 45 s → 15 s
```

### MPC 초기 튜닝 절차 (Gazebo 환경)

1. `smoothstep`으로 기본 미션 동작 확인
2. `trajectory.type: "mpc"`, 기본값으로 전환 후 비행 로그 수집
3. 오버슈트 발생 시 → `w_vel` 0.1씩 증가
4. 수렴이 너무 느릴 때 → `w_pos` 0.5씩 증가, `w_u` 0.01씩 감소
5. 기동이 급격할 때 → `w_u` 0.05씩 증가
6. 튜닝된 값을 `global_params.yaml`에 반영
