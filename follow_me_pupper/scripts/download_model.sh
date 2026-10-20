#!/bin/bash
# Fetch the Hailo model that viser_camera.py runs for detection + segmentation.
set -e
PROJECT_ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"
cd "$PROJECT_ROOT/robot"

echo "Downloading YOLOv8n-seg for Hailo (boxes + segmentation masks)..."
wget -nc https://hailo-model-zoo.s3.eu-west-2.amazonaws.com/ModelZoo/Compiled/v2.14.0/hailo8l/yolov8n_seg.hef

echo "Done -> $PROJECT_ROOT/robot/yolov8n_seg.hef"
