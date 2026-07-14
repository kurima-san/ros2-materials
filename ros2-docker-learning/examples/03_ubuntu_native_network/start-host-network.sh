#!/usr/bin/env bash
set -e
cd "$(dirname "$0")"
docker compose -f compose.host.yaml up --build
