# TDD 개발 가이드

> 대상 독자: **코드 작성이 처음인 팀원** — Python 기초 수준이면 충분합니다.

## 아주 먼저 알아둘 점

이 프로젝트는 초보자가 겁먹지 않도록 기본 브랜치를 항상 가능한 한 초록 상태로 유지하는 것을 목표로 합니다.

즉:

- 지금 당장 구현하지 않은 기능 테스트는 기본적으로 `skip` 처리되어 있습니다.
- 담당자가 구현을 시작하면 해당 테스트의 `skip` 을 제거하고, 구현과 테스트를 같이 맞춰갑니다.
- 그래서 "기본 상태가 실패 20개 이상" 같은 흐름을 권장하지 않습니다.

처음이라면 아래 순서로 시작하세요.

1. 전체 테스트 한 번 실행
2. 본인 담당 테스트 파일 열기
3. 파일 상단의 `skip` 이유 읽기
4. 구현 시작 시 `skip` 제거
5. 한 테스트씩 PASS 만들기

---

## 목차

1. [TDD가 뭔가요?](#1-tdd가-뭔가요)
2. [테스트가 실제 내 코드를 어떻게 가져오나요?](#2-테스트가-실제-내-코드를-어떻게-가져오나요)
3. [ROS2 없이 어떻게 테스트가 실행되나요? — mock_ros2](#3-ros2-없이-어떻게-테스트가-실행되나요--mock_ros2)
4. [MockNode가 내 코드를 어떻게 감시하나요?](#4-mocknode가-내-코드를-어떻게-감시하나요)
5. [FAIL 메시지 읽는 법](#5-fail-메시지-읽는-법)
6. [노드 구현 단계별 가이드](#6-노드-구현-단계별-가이드)
7. [완성 예시 — WaypointNavNode](#7-완성-예시--waypointnavnode)
8. [내 파일 테스트 실행 방법](#8-내-파일-테스트-실행-방법)
9. [자주 하는 실수](#9-자주-하는-실수)

---

## 1. TDD가 뭔가요?

**TDD (Test-Driven Development)** = 코드 짜기 전에 테스트를 먼저 만들어 두는 방식

```
일반 개발:  코드 짜기 → 직접 실행해서 확인 → 버그 발견 → 코드 수정
TDD:        테스트 먼저 (FAIL) → 코드 짜기 → 테스트 PASS → 완료
```

이 프로젝트에서 TDD를 쓰는 이유:
- 드론 코드는 직접 실행하려면 시뮬레이터, PX4, ROS2가 전부 켜져 있어야 함
- 테스트는 **노트북에서도, ROS2 없이도** 1초 만에 확인 가능
- 내 코드가 인터페이스 규칙(토픽 이름, 파라미터 이름)을 지키는지 자동 검증

---

## 2. 테스트가 실제 내 코드를 어떻게 가져오나요?

테스트 파일이 `sys.path`를 조작해서 실제 `src/` 폴더를 Python 경로에 추가한 뒤 직접 import합니다.

예를 들어 `test/unit/test_vision_aruco.py` 상단을 보면:

```python
# 1단계: mock_ros2를 경로에 추가하고 ROS2 mock 주입
sys.path.insert(0, str(Path(__file__).parents[1]))   # test/ 폴더 추가
from mock_ros2 import install
install()                                             # rclpy 등을 가짜로 교체

# 2단계: 실제 소스 파일 경로를 추가
sys.path.insert(0, str(Path(__file__).parents[2] / 'src' / 'vtol_vision_aruco' / 'src'))
                   #  ↑ vtol_ws/                         ↑ 실제 노드 파일이 있는 폴더

# 3단계: 실제 파일에서 클래스를 import
from aruco_detect_node import ArucoDetectNode        # ← 내가 작성하는 파일!
```

경로를 그림으로 표현하면:

```
vtol_ws/
  test/
    unit/
      test_vision_aruco.py   ← 테스트 파일 (여기서 parents[2] = vtol_ws/)
  src/
    vtol_vision_aruco/
      src/
        aruco_detect_node.py ← 내가 구현해야 하는 파일
```

**결론**: 테스트 파일은 바꾸지 않아도 됩니다. 내가 할 일은 `src/.../내파일.py` 안에 코드를 채우는 것뿐입니다.

---

## 3. ROS2 없이 어떻게 테스트가 실행되나요? — mock_ros2

ROS2 없이 테스트할 수 있는 이유는 `test/mock_ros2.py` 덕분입니다.

### mock_ros2가 하는 일

```
실제 환경:
  내 노드 파일 ──import──▶ rclpy (ROS2 설치 필요)
                    └──▶ px4_msgs (ROS2 빌드 필요)

테스트 환경:
  install() 실행
       ↓
  sys.modules['rclpy']      = 가짜 객체 ← 설치 불필요
  sys.modules['px4_msgs']   = 가짜 객체
  sys.modules['rclpy.node'] = { Node: MockNode } ← 핵심!

  내 노드 파일 ──import──▶ rclpy (가짜) → 에러 없이 통과
                    └──▶ Node → 실제로는 MockNode 클래스
```

내 코드가 `from rclpy.node import Node` 하고 `class MyNode(Node)` 라고 써도,
테스트 중에는 실제로 `MockNode`를 상속받게 됩니다.

---

## 4. MockNode가 내 코드를 어떻게 감시하나요?

`MockNode`는 `create_publisher`, `create_subscription`, `declare_parameter`를 호출할 때마다
내부 딕셔너리에 **무엇을 호출했는지 기록**합니다.

```python
class MockNode:
    def __init__(self, name):
        self._publishers    = {}   # { 토픽이름: Mock객체 }
        self._subscriptions = {}   # { 토픽이름: (콜백함수, Mock객체) }
        self._parameters    = {}   # { 파라미터이름: 기본값 }
```

내 노드가 `__init__`에서 이렇게 쓰면:

```python
# 내 노드 코드 (aruco_detect_node.py)
self.create_publisher(PoseStamped, '/vtol/aruco/pose', 10)
```

MockNode 내부에서는 이렇게 저장됩니다:

```python
self._publishers['/vtol/aruco/pose'] = MagicMock()  # 가짜 publisher 저장
```

테스트는 이 딕셔너리를 조회해서 확인합니다:

```python
# 테스트 코드 (test_vision_aruco.py)
self.assertIn('/vtol/aruco/pose', self.node._publishers)
#              ↑ 이 토픽 이름이 딕셔너리에 있으면 PASS, 없으면 FAIL
```

### 전체 흐름 정리

```
테스트 시작
    ↓
node = ArucoDetectNode()   ← 내 __init__ 실행
    ↓
__init__ 안에서:
  create_subscription(..., '/drone1/camera/image_raw', callback)
    → MockNode._subscriptions['/drone1/camera/image_raw'] = (callback, Mock)
  create_publisher(..., '/vtol/aruco/pose', ...)
    → MockNode._publishers['/vtol/aruco/pose'] = Mock
  declare_parameter('vtol.aruco_marker_id', 0)
    → MockNode._parameters['vtol.aruco_marker_id'] = 0
    ↓
테스트가 딕셔너리 확인
  assertIn('/drone1/camera/image_raw', node._subscriptions)  → PASS
  assertIn('/vtol/aruco/pose',         node._publishers)     → PASS
  assertIn('vtol.aruco_marker_id',     node._parameters)     → PASS
```

---

## 5. FAIL 메시지 읽는 법

```bash
python -m unittest test.unit.test_vision_aruco -v
```

실행 결과 예시:

```
FAIL: test_subscribes_to_camera_image (test_vision_aruco.TestArucoInit)
----------------------------------------------------------------------
AssertionError: '/drone1/camera/image_raw' not found in {} : 카메라 토픽 구독 없음

                                              ^^
                                 현재 _subscriptions 딕셔너리 (비어 있음)
```

해석:
- `not found in {}` → `_subscriptions`이 비어 있음
- 즉, `__init__`에서 `create_subscription('/drone1/camera/image_raw', ...)` 을 **한 번도 안 불렀다**는 뜻

수정 방법: `__init__` 안에 구독 코드를 추가합니다 → 아래 [6장](#6-노드-구현-단계별-가이드) 참고.

---

## 6. 노드 구현 단계별 가이드

각 담당 노드를 어떻게 채워야 하는지 ArucoDetectNode를 예로 설명합니다.
다른 노드도 같은 패턴으로 작성하면 됩니다.

### 시작 전 확인

```bash
# 기본 상태 확인
python -m unittest discover -s test -v
# → PASS 와 skip 이 보여야 정상
#
# 구현을 시작할 때는:
# 1) 본인 담당 테스트 파일의 skip 제거
# 2) 그 다음 개별 테스트 실행
python -m unittest test.unit.test_vision_aruco -v
```

---

### Step 1. import 추가

파일: `src/vtol_vision_aruco/src/aruco_detect_node.py`

기존 코드:
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String     # ← 안 쓰는 import, 지워도 됨
```

수정 후:
```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
```

> **왜?** 구독/퍼블리시할 메시지 타입을 import해야 합니다.

---

### Step 2. `__init__` 안에 파라미터 선언 추가

```python
def __init__(self):
    super().__init__('aruco_detect_node')

    # 파라미터 선언 (테스트: test_declares_marker_id_parameter)
    self.declare_parameter('vtol.aruco_marker_id', 0)
```

> **왜?** `declare_parameter`를 호출해야 `_parameters` 딕셔너리에 저장됩니다.

테스트 실행:
```bash
python -m unittest test.unit.test_vision_aruco.TestArucoInit.test_declares_marker_id_parameter -v
# → PASS
```

---

### Step 3. 구독(Subscription) 추가

```python
def __init__(self):
    super().__init__('aruco_detect_node')
    self.declare_parameter('vtol.aruco_marker_id', 0)

    # 카메라 구독 (테스트: test_subscribes_to_camera_image, test_image_callback_exists)
    self.create_subscription(
        Image,
        '/drone1/camera/image_raw',   # ← 토픽 이름이 테스트와 정확히 같아야 함
        self._cb_image,               # ← 콜백 함수 (아래서 정의)
        10,
    )
```

> **왜?** `create_subscription`을 호출해야 `_subscriptions['/drone1/camera/image_raw']`에 저장됩니다.

---

### Step 4. 퍼블리셔(Publisher) 추가

```python
def __init__(self):
    ...
    # pose 퍼블리셔 (테스트: test_publishes_aruco_pose)
    self._pub_pose = self.create_publisher(
        PoseStamped,
        '/vtol/aruco/pose',           # ← 토픽 이름이 테스트와 정확히 같아야 함
        10,
    )
```

> **왜?** `create_publisher`를 호출해야 `_publishers['/vtol/aruco/pose']`에 저장됩니다.

---

### Step 5. 콜백 함수 작성

```python
def _cb_image(self, msg: Image) -> None:
    """카메라 이미지를 받아 ArUco 마커를 검출하고 pose를 퍼블리시"""
    # 실제 구현에서는 cv2.aruco로 마커 검출
    # 지금은 테스트 통과를 위해 항상 퍼블리시하는 stub
    pose_msg = PoseStamped()
    self._pub_pose.publish(pose_msg)
    # (테스트: test_pose_published_when_marker_detected)
```

---

### Step 6. 테스트 전체 실행

```bash
python -m unittest test.unit.test_vision_aruco -v
```

예상 결과:
```
test_declares_marker_id_parameter ... ok
test_image_callback_exists        ... ok
test_pose_published_when_marker_detected ... ok
test_publishes_aruco_pose         ... ok
test_subscribes_to_camera_image   ... ok

Ran 5 tests in 0.003s

OK
```

---

### 완성된 최소 구현 예시 (ArucoDetectNode)

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped


class ArucoDetectNode(Node):
    def __init__(self):
        super().__init__('aruco_detect_node')

        # 파라미터
        self.declare_parameter('vtol.aruco_marker_id', 0)

        # 구독
        self.create_subscription(Image, '/drone1/camera/image_raw', self._cb_image, 10)

        # 퍼블리셔
        self._pub_pose = self.create_publisher(PoseStamped, '/vtol/aruco/pose', 10)

    def _cb_image(self, msg: Image) -> None:
        # TODO: cv2.aruco로 실제 마커 검출 구현
        pose_msg = PoseStamped()
        self._pub_pose.publish(pose_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

이 최소 구현으로 **5개 테스트 전부 PASS**됩니다.
실제 ArUco 검출 로직은 `_cb_image` 안에 나중에 채우면 됩니다.

---

## 7. 완성 예시 — WaypointNavNode

이미 모든 테스트가 통과된 `WaypointNavNode`를 참고하세요.

파일: `src/vtol_control_nav/src/waypoint_nav_node.py`

핵심 패턴만 추려서 설명합니다:

### 파라미터 선언

```python
self.declare_parameter('vtol.drone_id', 'drone1')
self.declare_parameter('vtol.takeoff_altitude', 5.0)
self.declare_parameter('vtol.cruise_altitude', 30.0)

# 선언한 파라미터 값 읽기
drone_id = self.get_parameter('vtol.drone_id').value
```

### Publisher 등록 패턴

```python
self._pub_offboard = self.create_publisher(
    OffboardControlMode,           # 메시지 타입
    f'/{drone_id}/fmu/in/offboard_control_mode',  # 토픽 이름
    _PX4_QOS,                      # QoS 설정
)
```

> `_PX4_QOS`는 PX4 통신에 필요한 QoS입니다. 위 파일 상단에 정의되어 있습니다.
> 다른 노드에서는 그냥 숫자 `10` 을 써도 됩니다.

### Subscription 등록 패턴

```python
self.create_subscription(
    VehicleStatus,                         # 메시지 타입
    f'/{drone_id}/fmu/out/vehicle_status', # 토픽 이름
    self._cb_status,                       # 콜백 함수 이름
    _PX4_QOS,
)
```

### 타이머 등록 패턴

```python
self._timer = self.create_timer(0.1, self._control_loop)  # 0.1초 = 10Hz
```

> `create_timer`를 호출하면 `MockNode._timer_callbacks`에 콜백이 저장됩니다.
> `TelemetryNode`의 `test_timer_registered_for_periodic_send` 테스트가 이것을 확인합니다.

---

## 8. 내 파일 테스트 실행 방법

```bash
# 프로젝트 루트(vtol_ws/)에서 실행해야 합니다

# 내 파일 하나만 테스트
python -m unittest test.unit.test_vision_aruco -v

# 특정 테스트 하나만 실행
python -m unittest test.unit.test_vision_aruco.TestArucoInit.test_subscribes_to_camera_image -v

# 단위 테스트 전체 실행
python -m unittest discover -s test/unit -v

# contract + unit 전체 (통합 테스트 제외)
python -m unittest discover -s test -v
```

담당자별 테스트 파일:

| 담당 | 테스트 파일 | 구현 파일 |
|------|-------------|-----------|
| 비전 A | `test/unit/test_vision_yolo.py` | `src/vtol_vision_yolo/src/yolo_detect_node.py` |
| 비전 B | `test/unit/test_vision_aruco.py` | `src/vtol_vision_aruco/src/aruco_detect_node.py` |
| 제어 B | `test/unit/test_control_task.py` | `src/vtol_control_task/src/precision_task_node.py` |
| HW A | `test/unit/test_hw_gripper.py` | `src/vtol_hw_gripper/src/arduino_cmd_node.py` |
| 통신 B | `test/unit/test_comm_lte.py` | `src/vtol_comm_lte/src/telemetry_node.py` |

---

## 9. 자주 하는 실수

### 실수 1. 토픽 이름 오타

```python
# 잘못됨 ❌
self.create_subscription(Image, '/drone1/camera/imageraw', self._cb, 10)
#                                                  ↑ 언더스코어 누락

# 올바름 ✅
self.create_subscription(Image, '/drone1/camera/image_raw', self._cb, 10)
```

테스트 파일에 있는 토픽 이름을 **복사해서 붙여넣기** 하세요. 대소문자, 슬래시, 언더스코어가 한 글자라도 다르면 FAIL입니다.

---

### 실수 2. 파라미터 이름 오타

```python
# 잘못됨 ❌
self.declare_parameter('aruco_marker_id', 0)

# 올바름 ✅
self.declare_parameter('vtol.aruco_marker_id', 0)
#                       ↑ 반드시 'vtol.' 접두사 포함
```

---

### 실수 3. mock 주입 전에 import

```python
# 잘못됨 ❌ — rclpy를 먼저 import하면 실제 rclpy를 찾으려고 해서 에러
from aruco_detect_node import ArucoDetectNode
from mock_ros2 import install
install()

# 올바름 ✅ — install() 먼저, 그 다음 노드 import
from mock_ros2 import install
install()
from aruco_detect_node import ArucoDetectNode
```

> 이 순서는 테스트 파일에 이미 올바르게 작성되어 있습니다. 테스트 파일은 건드리지 마세요.

---

### 실수 4. `create_timer` 빠뜨리기 (comm_lte)

```python
# TelemetryNode: 주기 전송 타이머가 없으면 FAIL
# 테스트: test_timer_registered_for_periodic_send

# 잘못됨 ❌
def __init__(self):
    super().__init__('telemetry_node')
    # 타이머 없음

# 올바름 ✅
def __init__(self):
    super().__init__('telemetry_node')
    self.create_timer(1.0, self._send_telemetry)  # 1초마다 전송
```

---

### 실수 5. publish를 `__init__`에서 하지 않음 (콜백 미등록)

```python
# 잘못됨 ❌ — 콜백을 람다나 다른 방식으로 등록하면 테스트가 콜백을 못 찾음
self.create_subscription(Image, '/drone1/camera/image_raw',
                         lambda msg: self._pub_pose.publish(PoseStamped()), 10)

# 올바름 ✅ — 메서드로 분리
self.create_subscription(Image, '/drone1/camera/image_raw', self._cb_image, 10)

def _cb_image(self, msg):
    self._pub_pose.publish(PoseStamped())
```

> 람다도 사실 callable이라 `test_image_callback_exists`는 통과하지만,
> 가독성과 유지보수를 위해 메서드로 분리하는 것을 권장합니다.

---

## 관련 문서

| 문서 | 내용 |
|------|------|
| [test_spec.md](test_spec.md) | 각 노드 테스트 항목 상세 명세 |
| [architecture.md](architecture.md) | 전체 토픽 목록 및 노드 관계도 |
| [test/README.md](../test/README.md) | 테스트 실행 명령어 빠른 참조 |
