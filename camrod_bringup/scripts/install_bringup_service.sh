#!/usr/bin/env bash
# 261104 - Install CAMROD bringup as a *user* systemd unit.
#
# bringup opens camrod_ui_window, a browser window that must talk to the X
# server. A system unit has no DISPLAY/XAUTHORITY, so the stack would come up
# headless with no operator UI. A user unit ordered after graphical-session.target
# inherits both from the GNOME session, so the window appears.
#
# Run as the normal desktop user (nvidia), NOT with sudo.
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
pkg_dir="$(cd "${script_dir}/.." && pwd)"
launcher=/usr/local/bin/camrod-bringup
user_unit_dir="${HOME}/.config/systemd/user"

if [[ "${EUID}" -eq 0 ]]; then
  echo "Do NOT run this as root: user units live in \$HOME/.config/systemd/user." >&2
  echo "Run it as the desktop user:  ${BASH_SOURCE[0]}" >&2
  exit 1
fi

# --- 1. launcher (the only part needing root) ---------------------------------
if ! cmp -s "${pkg_dir}/scripts/launch_bringup.sh" "${launcher}" 2>/dev/null; then
  echo "!! ${launcher} is missing or out of date. Install it first:"
  echo "   sudo install -m 0755 ${pkg_dir}/scripts/launch_bringup.sh ${launcher}"
  exit 1
fi

# --- 2. user unit -------------------------------------------------------------
mkdir -p "${user_unit_dir}"
install -m 0644 "${pkg_dir}/systemd/camrod-bringup.user.service" \
                "${user_unit_dir}/camrod-bringup.service"
systemctl --user daemon-reload
systemctl --user enable camrod-bringup.service

# --- 3. report what still needs root ------------------------------------------
echo
echo "--- installed (user unit enabled) ---"

if systemctl is-enabled --quiet camrod-bringup.service 2>/dev/null; then
  echo
  echo "!! The OLD system-wide unit is still enabled and will race this one."
  echo "   Disable it:"
  echo "     sudo systemctl disable --now camrod-bringup.service"
  echo "     sudo rm /etc/systemd/system/camrod-bringup.service"
  echo "     sudo systemctl daemon-reload"
fi

if ! grep -qE '^\s*AutomaticLoginEnable\s*=\s*[Tt]rue' /etc/gdm3/custom.conf 2>/dev/null; then
  echo
  echo "!! GDM autologin is OFF, so nothing starts until someone logs in."
  echo "   Enable it (required for a hands-off boot):"
  echo "     sudo sed -i 's/^\\[daemon\\]/[daemon]\\nAutomaticLoginEnable=true\\nAutomaticLogin='\"${USER}\"'/' /etc/gdm3/custom.conf"
fi

cat <<'MSG'

  start now : systemctl --user start camrod-bringup
  stop      : systemctl --user stop camrod-bringup
  follow log: journalctl --user -u camrod-bringup -f
  disable   : systemctl --user disable camrod-bringup

Note: no startup delay. The stack launches as soon as the desktop session is up.
MSG
