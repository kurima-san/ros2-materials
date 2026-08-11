#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
mkdir -p "$HOME/mola_ws"
cp "$SCRIPT_DIR/mola.bashrc" "$HOME/mola_ws/mola.bashrc"

MARK_BEGIN='# >>> mola_shell managed by mola_mid360_complete_bundle >>>'
MARK_END='# <<< mola_shell managed by mola_mid360_complete_bundle <<<'
if ! grep -Fq "$MARK_BEGIN" "$HOME/.bashrc"; then
cat >> "$HOME/.bashrc" <<'BLOCK'

# >>> mola_shell managed by mola_mid360_complete_bundle >>>
mola_shell()
{
    env -i \
        HOME="$HOME" \
        USER="$USER" \
        LOGNAME="$LOGNAME" \
        SHELL=/bin/bash \
        TERM="$TERM" \
        DISPLAY="$DISPLAY" \
        XAUTHORITY="${XAUTHORITY:-}" \
        XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-}" \
        DBUS_SESSION_BUS_ADDRESS="${DBUS_SESSION_BUS_ADDRESS:-}" \
        PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
        bash --noprofile --rcfile "$HOME/mola_ws/mola.bashrc" -i
}
# <<< mola_shell managed by mola_mid360_complete_bundle <<<
BLOCK
  echo "Added mola_shell() to ~/.bashrc"
else
  echo "mola_shell() block already exists in ~/.bashrc; no duplicate added."
fi

echo "Installed: ~/mola_ws/mola.bashrc"
echo "Open a new normal shell or run: source ~/.bashrc"
