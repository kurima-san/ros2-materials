#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEST="$HOME/mola_ws/src/mola_mid360_tools"
mkdir -p "$HOME/mola_ws/src"
rm -rf "$DEST"
cp -a "$ROOT/mola_mid360_tools" "$DEST"

source /opt/ros/humble/setup.bash
if [[ -f "$HOME/mola_ws/install/setup.bash" ]]; then
  source "$HOME/mola_ws/install/setup.bash"
fi
cd "$HOME/mola_ws"
colcon build --symlink-install --packages-select mola_mid360_tools

echo
echo "Installed mola_mid360_tools into: $DEST"
echo "Now enter a fresh MOLA shell: mola_shell"
