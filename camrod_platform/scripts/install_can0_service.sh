#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
pkg_dir="$(cd "${script_dir}/.." && pwd)"

install -m 0755 "${pkg_dir}/scripts/setup_can0.sh" /usr/local/sbin/camrod-setup-can0
install -m 0644 "${pkg_dir}/systemd/camrod-can0.service" /etc/systemd/system/camrod-can0.service

systemctl daemon-reload
systemctl enable camrod-can0.service
systemctl restart camrod-can0.service
systemctl --no-pager --full status camrod-can0.service
