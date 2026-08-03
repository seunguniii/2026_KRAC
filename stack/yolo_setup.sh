#!/bin/bash
# setup_env.sh

echo "Creating YOLO virtual environment..."
python3 -m venv --system-site-packages ~/venvs/yolo

echo "Installing dependencies..."
~/venvs/yolo/bin/pip install -I --no-cache-dir \
  "numpy<2.0.0" \
  "setuptools<80" \
  typing_extensions \
  ultralytics

echo "Creating symlink for clean ROS 2 logging..."
ln -sf ~/venvs/yolo/bin/python3 ~/venvs/yolo/bin/yolo

echo "Environment setup complete."
