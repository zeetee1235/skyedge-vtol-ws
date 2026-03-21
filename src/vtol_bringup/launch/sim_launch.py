"""
시뮬레이션 환경 통합 launch 파일
실행: ros2 launch vtol_bringup sim_launch.py

전제조건:
  - docker compose --profile sim up (px4_sitl + xrce_agent 실행 중)
  - 또는 네이티브:
      MicroXRCEAgent udp4 -p 8888 &
      make px4_sitl_default gazebo-classic_standard_vtol

총괄만 수정 가능
"""
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('vtol_bringup'),
        'config', 'global_params.yaml'
    )

    # xrce_agent가 PX4 토픽을 브리지할 시간을 확보 (3초 대기)
    ros_nodes = [
        # 제어 A: 웨이포인트 네비게이션 (PX4 offboard 제어)
        Node(
            package='vtol_control_nav',
            executable='waypoint_nav_node',
            name='waypoint_nav',
            parameters=[config],
            output='screen',
        ),
        # 제어 B: 정밀 착륙
        Node(
            package='vtol_control_task',
            executable='precision_task_node',
            name='precision_task',
            parameters=[config],
            output='screen',
        ),
        # 비전 A: YOLO 탐지 (/drone1/camera/image_raw 구독)
        Node(
            package='vtol_vision_yolo',
            executable='yolo_detect_node',
            name='yolo_detect',
            parameters=[config],
            output='screen',
        ),
        # 비전 B: ArUco 마커
        Node(
            package='vtol_vision_aruco',
            executable='aruco_detect_node',
            name='aruco_detect',
            parameters=[config],
            output='screen',
        ),
        # 통신: LTE 텔레메트리 (시뮬에서는 GCS 모니터링 용도)
        Node(
            package='vtol_comm_lte',
            executable='telemetry_node',
            name='telemetry',
            parameters=[config],
            output='screen',
        ),
        # vtol_hw_gripper: 시뮬에서는 실행 안 함 (아두이노 없음)
        # → real_launch.py 에서만 실행됨
    ]

    return LaunchDescription([
        TimerAction(period=3.0, actions=ros_nodes),
    ])
