#!/usr/bin/env bash
set -e
cd "$(dirname "$0")"
echo "ログ表示を終了するには Ctrl+C を押します。Containerは動作を続けます。"
docker compose -f compose.dev.yaml logs -f
