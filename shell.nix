{ pkgs ? import <nixpkgs> {} }:

pkgs.mkShell {
  name = "vtol-dev";

  buildInputs = with pkgs; [
    # 기본 유틸
    git
    curl
    wget
    vim

    # Python 환경
    (python3.withPackages (ps: with ps; [
      numpy
      pyserial
      opencv4
      # ultralytics는 nixpkgs 미지원 → setup.sh 또는 pip로 별도 설치
    ]))

    # ROS2 빌드 도구 (nixpkgs에 colcon이 없으면 pip fallback)
    # colcon-common-extensions → shellHook 에서 pip install
  ];

  shellHook = ''
    echo "──────────────────────────────────────"
    echo " VTOL 개발 환경 (Nix Shell)"
    echo "──────────────────────────────────────"

    # ROS2 Humble 소싱 (apt 설치되어 있을 경우)
    if [ -f /opt/ros/humble/setup.bash ]; then
      source /opt/ros/humble/setup.bash
      echo "[OK] ROS2 Humble sourced"
    else
      echo "[WARN] ROS2 Humble not found at /opt/ros/humble"
      echo "       setup.sh 를 먼저 실행하거나 Docker를 사용하세요."
    fi

    # 로컬 빌드 결과 소싱
    if [ -f install/setup.bash ]; then
      source install/setup.bash
      echo "[OK] vtol_ws install/setup.bash sourced"
    fi

    # colcon / pip 패키지가 없으면 venv에 설치
    if ! command -v colcon &>/dev/null; then
      echo "[INFO] colcon 없음 → pip install colcon-common-extensions"
      pip install --quiet colcon-common-extensions
    fi

    echo ""
    echo "빌드:  colcon build --symlink-install"
    echo "테스트: colcon test"
    echo "──────────────────────────────────────"
  '';
}
