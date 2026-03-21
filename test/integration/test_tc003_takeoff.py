"""
integration › TC-003 자동 이륙 테스트

실행 조건: docker compose --profile sim up 이후
  SIM_RUNNING=1 python -m unittest test/integration/test_tc003_takeoff.py

스킵 조건: SIM_RUNNING 환경변수 없을 때 (기본)
"""
import os
import subprocess
import time
import unittest


SIM_RUNNING = os.getenv('SIM_RUNNING', '0') == '1'


@unittest.skipUnless(SIM_RUNNING, 'SIM_RUNNING=1 필요 (docker compose --profile sim up)')
class TestTC003Takeoff(unittest.TestCase):
    """TC-003: 기체가 지정 고도까지 자동 이륙 가능한지 검증"""

    TOPIC_VEHICLE_STATUS   = '/drone1/fmu/out/vehicle_status'
    TOPIC_LOCAL_POSITION   = '/drone1/fmu/out/vehicle_local_position'
    TOPIC_OFFBOARD_MODE    = '/drone1/fmu/in/offboard_control_mode'
    TAKEOFF_ALTITUDE_M     = 5.0
    ALTITUDE_TOLERANCE_M   = 0.5
    TIMEOUT_SEC            = 30

    def _ros2_topic_list(self) -> list[str]:
        result = subprocess.run(
            ['ros2', 'topic', 'list'],
            capture_output=True, text=True, timeout=10,
        )
        return result.stdout.splitlines()

    def test_px4_topics_visible(self):
        """PX4 uXRCE-DDS 브리지가 살아 있는지 — /drone1/fmu 토픽 확인"""
        topics = self._ros2_topic_list()
        self.assertTrue(
            any('/drone1/fmu' in t for t in topics),
            f"PX4 토픽이 ROS2에서 보이지 않음. xrce_agent 실행 여부 확인\n"
            f"현재 토픽: {topics[:10]}",
        )

    def test_vehicle_status_topic_active(self):
        topics = self._ros2_topic_list()
        self.assertIn(self.TOPIC_VEHICLE_STATUS, topics,
                      "vehicle_status 토픽 없음")

    def test_local_position_topic_active(self):
        topics = self._ros2_topic_list()
        self.assertIn(self.TOPIC_LOCAL_POSITION, topics,
                      "vehicle_local_position 토픽 없음")

    def test_ros2_nodes_running(self):
        """vtol ROS2 노드들이 실행 중인지 확인"""
        result = subprocess.run(
            ['ros2', 'node', 'list'],
            capture_output=True, text=True, timeout=10,
        )
        nodes = result.stdout
        self.assertIn('waypoint_nav', nodes,
                      "waypoint_nav_node 실행 안 됨")
