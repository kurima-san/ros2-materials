#!/usr/bin/env bash
set -e
cd "$(dirname "$0")"
docker compose -f compose.runtime.yaml up -d --build
docker compose -f compose.runtime.yaml ps
