#!/usr/bin/env bash
set -e
cd "$(dirname "$0")"
IMAGE=${1:?Usage: $0 registry.example.com/name/image:tag}
docker buildx build --platform linux/amd64,linux/arm64 -f docker/Dockerfile.arm64 -t "$IMAGE" --push .
