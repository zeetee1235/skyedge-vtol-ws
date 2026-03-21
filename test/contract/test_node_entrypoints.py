import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]

NODE_FILES = {
    "vtol_vision_yolo": ROOT / "src" / "vtol_vision_yolo" / "src" / "yolo_detect_node.py",
    "vtol_vision_aruco": ROOT / "src" / "vtol_vision_aruco" / "src" / "aruco_detect_node.py",
    "vtol_control_nav": ROOT / "src" / "vtol_control_nav" / "src" / "waypoint_nav_node.py",
    "vtol_control_task": ROOT / "src" / "vtol_control_task" / "src" / "precision_task_node.py",
    "vtol_hw_gripper": ROOT / "src" / "vtol_hw_gripper" / "src" / "arduino_cmd_node.py",
    "vtol_comm_lte": ROOT / "src" / "vtol_comm_lte" / "src" / "telemetry_node.py",
}


class NodeEntrypointTests(unittest.TestCase):
    def test_all_runtime_nodes_have_main_entrypoint(self):
        for package_name, node_path in NODE_FILES.items():
            with self.subTest(package=package_name):
                code = node_path.read_text(encoding="utf-8")
                self.assertIn("def main(args=None):", code)
                self.assertIn("rclpy.init(args=args)", code)
                self.assertIn("rclpy.spin(node)", code)
                self.assertIn("node.destroy_node()", code)
                self.assertIn("rclpy.shutdown()", code)
                self.assertIn("if __name__ == '__main__':", code)

    def test_node_classes_match_expected_runtime_names(self):
        expected_classes = {
            "vtol_vision_yolo": "class YoloDetectNode(Node):",
            "vtol_vision_aruco": "class ArucoDetectNode(Node):",
            "vtol_control_nav": "class WaypointNavNode(Node):",
            "vtol_control_task": "class PrecisionTaskNode(Node):",
            "vtol_hw_gripper": "class ArduinoCmdNode(Node):",
            "vtol_comm_lte": "class TelemetryNode(Node):",
        }

        for package_name, class_decl in expected_classes.items():
            with self.subTest(package=package_name):
                code = NODE_FILES[package_name].read_text(encoding="utf-8")
                self.assertIn(class_decl, code)


if __name__ == "__main__":
    unittest.main()
