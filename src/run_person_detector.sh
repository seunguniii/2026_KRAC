#!/usr/bin/env bash

set -e

WORKSPACE="$HOME/space_y/ws_KRAC"
VENV="$HOME/yolo"

PACKAGE="imagery_processing"
EXECUTABLE="following_recognition"

ENTRYPOINT="$WORKSPACE/install_yolo/$PACKAGE/lib/$PACKAGE/$EXECUTABLE"

echo "========== ROS YOLO START =========="

# ROS 2 Humble
if [ ! -f /opt/ros/humble/setup.bash ]; then
    echo "[ERROR] ROS 2 Humble setup.bash를 찾을 수 없습니다."
    exit 1
fi

source /opt/ros/humble/setup.bash

# 기존 워크스페이스 overlay
if [ -f "$WORKSPACE/install/setup.bash" ]; then
    source "$WORKSPACE/install/setup.bash"
fi

# YOLO Python 가상환경
if [ ! -f "$VENV/bin/activate" ]; then
    echo "[ERROR] YOLO 가상환경을 찾을 수 없습니다: $VENV"
    exit 1
fi

source "$VENV/bin/activate"

# YOLO 전용 ROS overlay
if [ ! -f "$WORKSPACE/install_yolo/setup.bash" ]; then
    echo "[ERROR] install_yolo/setup.bash가 없습니다."
    echo "먼저 imagery_processing 패키지를 build_yolo/install_yolo로 빌드하세요."
    exit 1
fi

source "$WORKSPACE/install_yolo/setup.bash"

echo
echo "Python       : $(which python3)"
echo "ROS package  : $(ros2 pkg prefix "$PACKAGE")"

# Ultralytics 확인
"$VENV/bin/python3" - <<'PY'
import sys
import torch
import ultralytics

print("Python exec  :", sys.executable)
print("Ultralytics  :", ultralytics.__version__)
print("CUDA enabled :", torch.cuda.is_available())

if torch.cuda.is_available():
    print("GPU          :", torch.cuda.get_device_name(0))
PY

# ros2 run 실행 파일 확인
if [ ! -f "$ENTRYPOINT" ]; then
    echo
    echo "[ERROR] ROS 실행 파일을 찾을 수 없습니다:"
    echo "$ENTRYPOINT"
    exit 1
fi

# 빌드하면서 /usr/bin/python3로 돌아간 경우 자동 수정
CURRENT_SHEBANG="$(head -n 1 "$ENTRYPOINT")"
EXPECTED_SHEBANG="#!$VENV/bin/python3"

if [ "$CURRENT_SHEBANG" != "$EXPECTED_SHEBANG" ]; then
    echo
    echo "Python shebang 수정:"
    echo "  기존: $CURRENT_SHEBANG"
    echo "  변경: $EXPECTED_SHEBANG"

    sed -i "1c\\$EXPECTED_SHEBANG" "$ENTRYPOINT"
fi

chmod +x "$ENTRYPOINT"

echo
echo "Entrypoint   : $(head -n 1 "$ENTRYPOINT")"
echo "Starting     : ros2 run $PACKAGE $EXECUTABLE"
echo "===================================="
echo

exec ros2 run "$PACKAGE" "$EXECUTABLE"
