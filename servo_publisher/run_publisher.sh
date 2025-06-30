#!/usr/bin/env bash

IMAGE="ghcr.io/screamlab/pros_base_image"

HOST_CODE_DIR="$(cd "$(dirname "$0")/../servo_publisher" && pwd)"
CONTAINER_CODE_DIR="/workspaces/src/servo_publisher"

docker run -it --rm \
  --network host \
  -v "${HOST_CODE_DIR}:${CONTAINER_CODE_DIR}" \
  -w "${CONTAINER_CODE_DIR}" \
  "${IMAGE}" \
  bash
