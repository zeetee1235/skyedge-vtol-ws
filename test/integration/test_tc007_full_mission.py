"""
integration › TC-007 간단 통합 미션 테스트

실행 조건: docker compose --profile sim up 이후
  SIM_RUNNING=1 python -m unittest test/integration/test_tc007_full_mission.py

스킵 조건: SIM_RUNNING 환경변수 없을 때 (기본)
"""
import os
import subprocess
import unittest


SIM_RUNNING = os.getenv('SIM_RUNNING', '0') == '1'


@unittest.skipUnless(SIM_RUNNING, 'SIM_RUNNING=1 필요 (docker compose --profile sim up)')
class TestTC007FullMission(unittest.TestCase):
    """TC-007: 이륙 → 웨이포인트 → 착륙 전체 흐름 검증"""

    def _topic_hz(self, topic: str, duration: int = 3) -> float:
        """topic의 publish 주파수를 측정해 반환 (Hz)"""
        result = subprocess.run(
            ['ros2', 'topic', 'hz', topic, '--window', '10'],
            capture_output=True, text=True, timeout=duration + 5,
        )
        for line in result.stdout.splitlines():
            if 'average rate' in line:
                try:
                    return float(line.split(':')[1].strip().split()[0])
                except (IndexError, ValueError):
                    pass
        return 0.0

    def test_offboard_setpoint_publishing(self):
        """waypoint_nav_node 가 setpoint 를 주기적으로 퍼블리시하는지 확인 (≥ 5 Hz)"""
        hz = self._topic_hz('/drone1/fmu/in/trajectory_setpoint', duration=4)
        self.assertGreaterEqual(hz, 5.0,
                                f"trajectory_setpoint 주파수 낮음: {hz:.1f} Hz (기대 ≥ 5 Hz)")

    def test_offboard_control_mode_publishing(self):
        """offboard_control_mode 주기 퍼블리시 확인 (≥ 5 Hz)"""
        hz = self._topic_hz('/drone1/fmu/in/offboard_control_mode', duration=4)
        self.assertGreaterEqual(hz, 5.0,
                                f"offboard_control_mode 주파수 낮음: {hz:.1f} Hz")

    def test_all_required_nodes_alive(self):
        result = subprocess.run(
            ['ros2', 'node', 'list'],
            capture_output=True, text=True, timeout=10,
        )
        nodes = result.stdout
        for expected in ['waypoint_nav', 'precision_task', 'yolo_detect', 'aruco_detect']:
            self.assertIn(expected, nodes, f"{expected} 노드가 실행되지 않음")
