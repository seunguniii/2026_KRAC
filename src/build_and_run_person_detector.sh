#!/usr/bin/env bash

set -e

WORKSPACE="$HOME/space_y/ws_KRAC"
VENV="$HOME/yolo"
PACKAGE="imagery_processing"

cd "$WORKSPACE"

source /opt/ros/humble/setup.bash

if [ -f "$WORKSPACE/install/setup.bash" ]; then
    source "$WORKSPACE/install/setup.bash"
fi

source "$VENV/bin/activate"

echo "Python: $(which python3)"
echo "Building $PACKAGE..."

colcon \
    --log-base log_yolo \
    build \
    --build-base build_yolo \
    --install-base install_yolo \
    --symlink-install \
    --packages-select "$PACKAGE"

exec "$HOME/bin/run_person_detector.sh"
