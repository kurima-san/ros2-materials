#!/usr/bin/env bash
set -e
cd "$(dirname "$0")"
docker compose -f compose.jetson-sim.yaml up --build
