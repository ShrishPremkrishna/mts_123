#!/bin/bash
# Fetch the course-pinned Hailo8L detection/segmentation model; no robot activation.
set -euo pipefail
PROJECT_ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"
MODEL="$PROJECT_ROOT/robot/yolov8n_seg.hef"
URL='https://hailo-model-zoo.s3.eu-west-2.amazonaws.com/ModelZoo/Compiled/v2.14.0/hailo8l/yolov8n_seg.hef'
# Content observed from the course's versioned upstream URL on 2026-10-28.
EXPECTED='3e91fd4c8be77a560169185bd8cba7e6a6b9ab8be14ea118d5e9d054df51ca36'

digest() {
  if command -v sha256sum >/dev/null 2>&1; then
    sha256sum "$1" | awk '{print $1}'
  else
    shasum -a 256 "$1" | awk '{print $1}'
  fi
}
if [[ -e "$MODEL" ]]; then
  if [[ "$(digest "$MODEL")" != "$EXPECTED" ]]; then
    echo "Existing model differs from the course pin; preserve and inspect it before replacement: $MODEL" >&2
    exit 1
  fi
  echo "Verified existing model: $MODEL"
  exit 0
fi

TEMP_MODEL="$(mktemp "$PROJECT_ROOT/robot/.yolov8n_seg.XXXXXX")"
trap 'rm -f "$TEMP_MODEL"' EXIT
echo 'Downloading YOLOv8n-seg for Hailo8L (boxes + segmentation masks)...'
if command -v curl >/dev/null 2>&1; then
  curl --fail --location --max-time 180 --output "$TEMP_MODEL" "$URL"
else
  wget --timeout=60 --tries=2 -O "$TEMP_MODEL" "$URL"
fi
if [[ "$(digest "$TEMP_MODEL")" != "$EXPECTED" ]]; then
  echo 'Downloaded model does not match the pinned checksum; refusing installation.' >&2
  exit 1
fi
# Hard-link creation is atomic and refuses a destination created by another process.
ln "$TEMP_MODEL" "$MODEL"
echo "Verified model ready: $MODEL"
