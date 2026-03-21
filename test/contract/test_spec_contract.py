"""
계약 테스트 — 명세 문서·코드 구조 검증

이 파일의 테스트는 항상 PASS 상태를 유지해야 합니다.
새 파일을 추가하거나 구조를 바꿀 때 이 파일도 함께 업데이트하세요.
"""
import re
import unittest
from pathlib import Path


# 프로젝트 루트 (test/contract/ 에서 2단계 상위)
ROOT = Path(__file__).resolve().parents[2]

# 검증 대상 파일 경로
TEST_SPEC    = ROOT / "docs" / "test_spec.md"
SIM_LAUNCH   = ROOT / "src" / "vtol_bringup" / "launch" / "sim_launch.py"
REAL_LAUNCH  = ROOT / "src" / "vtol_bringup" / "launch" / "real_launch.py"
GLOBAL_PARAMS = ROOT / "src" / "vtol_bringup" / "config" / "global_params.yaml"
WAYPOINT_NAV = ROOT / "src" / "vtol_control_nav" / "src" / "waypoint_nav_node.py"


class SpecContractTests(unittest.TestCase):
    """명세 문서·런치 파일·핵심 노드 구조가 계약을 만족하는지 검사"""

    # ── 명세 문서 ──────────────────────────────────────────────────

    def test_spec_contains_core_test_cases(self):
        """docs/test_spec.md 에 TC-001 ~ TC-007 이 모두 존재해야 한다"""
        spec = TEST_SPEC.read_text(encoding="utf-8")

        for case_id in ["TC-001", "TC-002", "TC-003",
                         "TC-004", "TC-005", "TC-006", "TC-007"]:
            self.assertIn(case_id, spec)

    # ── 파라미터 파일 ──────────────────────────────────────────────

    def test_global_params_define_simulation_baseline(self):
        """global_params.yaml 에 시뮬 기본값 키와 waypoint 2개 이상이 있어야 한다"""
        params = GLOBAL_PARAMS.read_text(encoding="utf-8")

        # 필수 키 존재 확인
        for key in ["use_sim: true", "takeoff_altitude:",
                     "cruise_altitude:", "max_velocity:", "waypoints:"]:
            self.assertIn(key, params)

        # waypoint 항목 2개 이상 ( - [x, y, z] 형식 )
        waypoint_matches = re.findall(r"^\s*-\s*\[[^\]]+\]$", params, flags=re.MULTILINE)
        self.assertGreaterEqual(len(waypoint_matches), 2)

    # ── 시뮬 런치 파일 ─────────────────────────────────────────────

    def test_sim_launch_contains_required_simulation_nodes(self):
        """sim_launch.py 에 시뮬용 노드 5개와 TimerAction(3.0 s) 이 있어야 한다"""
        launch_text = SIM_LAUNCH.read_text(encoding="utf-8")

        # 시뮬에서 기동해야 하는 패키지 목록
        for pkg in ["vtol_control_nav", "vtol_control_task",
                     "vtol_vision_yolo", "vtol_vision_aruco", "vtol_comm_lte"]:
            self.assertIn(f"package='{pkg}'", launch_text)

        # DDS 에이전트 준비 대기용 지연 액션
        self.assertIn("TimerAction", launch_text)
        self.assertIn("period=3.0", launch_text)

        # 그리퍼는 실기체 전용 — 시뮬에서는 기동 금지
        self.assertNotIn("package='vtol_hw_gripper'", launch_text)

    # ── 실기체 런치 파일 ───────────────────────────────────────────

    def test_real_launch_contains_runtime_nodes_including_gripper(self):
        """real_launch.py 에 그리퍼를 포함한 전체 노드와 use_sim=False 가 있어야 한다"""
        launch_text = REAL_LAUNCH.read_text(encoding="utf-8")

        # 실기체에서 기동해야 하는 패키지 목록 (그리퍼 포함)
        for pkg in ["vtol_control_nav", "vtol_control_task",
                     "vtol_vision_yolo", "vtol_vision_aruco",
                     "vtol_hw_gripper", "vtol_comm_lte"]:
            self.assertIn(f"package='{pkg}'", launch_text)

        # 실기체 모드 플래그
        self.assertIn("{'vtol.use_sim': False}", launch_text)

    # ── waypoint 내비게이션 노드 ───────────────────────────────────

    def test_waypoint_nav_has_takeoff_navigate_and_land_flow(self):
        """waypoint_nav_node.py 에 전체 상태 머신과 핵심 명령 메서드가 있어야 한다"""
        nav_code = WAYPOINT_NAV.read_text(encoding="utf-8")

        # 상태 머신 상태 6개
        for state in ["_IDLE", "_ARMING", "_TAKEOFF", "_NAVIGATE", "_LAND", "_DONE"]:
            self.assertIn(state, nav_code)

        # 핵심 명령 메서드 호출
        self.assertIn("self._cmd_arm()", nav_code)
        self.assertIn("self._cmd_set_offboard_mode()", nav_code)
        self.assertIn("self._cmd_land()", nav_code)
        self.assertIn("self._send_setpoint(*wp)", nav_code)

    def test_waypoint_nav_defines_multiple_waypoints(self):
        """waypoint_nav_node.py 에 (x, y, z) 튜플이 2개 이상 정의되어 있어야 한다"""
        nav_code = WAYPOINT_NAV.read_text(encoding="utf-8")

        # self._waypoints: list[tuple[float, float, float]] = [...] 블록 추출
        waypoint_block_match = re.search(
            r"self\._waypoints: list\[tuple\[float, float, float\]\] = \[(.*?)\]",
            nav_code,
            flags=re.DOTALL,
        )
        self.assertIsNotNone(waypoint_block_match, "waypoints 리스트 정의를 찾을 수 없음")

        waypoint_block = waypoint_block_match.group(1)
        tuple_count = len(re.findall(r"\([^()]+\)", waypoint_block))
        self.assertGreaterEqual(tuple_count, 2, "waypoint 가 2개 미만임")


if __name__ == "__main__":
    unittest.main()
