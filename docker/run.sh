#!/bin/bash
# Start the exam container with the repository mounted at /ws and X11 forwarded for Gazebo/RViz.
set -e
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE=exploration-exam:humble

docker image inspect "$IMAGE" >/dev/null 2>&1 || docker build -t "$IMAGE" "$REPO/docker"

xhost +local:docker >/dev/null
docker run -it --rm \
    --net=host \
    -e DISPLAY="$DISPLAY" \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v "$REPO":/ws \
    --device /dev/dri \
    "$IMAGE" bash
