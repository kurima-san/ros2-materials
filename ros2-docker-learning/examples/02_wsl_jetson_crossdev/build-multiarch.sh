#!/usr/bin/env bash
set -e
cd "$(dirname "$0")"
IMAGE=${1:-ros2-jetson-crossdev:arm64}
docker buildx build --platform linux/arm64 -f docker/Dockerfile.arm64 -t "$IMAGE" --load .
echo "Built local ARM64 image: $IMAGE"
echo "For a real multi-platform release, use --platform linux/amd64,linux/arm64 with --push to a registry."
