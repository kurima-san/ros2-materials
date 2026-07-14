#!/usr/bin/env bash
set -e
cd "$(dirname "$0")"
docker compose -f compose.runtime.yaml up --build
