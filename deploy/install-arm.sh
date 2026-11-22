#!/usr/bin/env bash
# 在 ARM 开发板上安装 armd（需先在板上装好 Rust，或把交叉编译产物拷过来）
set -euo pipefail
BIN="${1:-target/release/armd}"
CONFIG="${2:-configs/koch-arm-board.json}"
sudo install -d /opt/armd /etc/armd
sudo install -m 755 "$BIN" /opt/armd/armd
[ -f /etc/armd/config.json ] || sudo install -m 600 "$CONFIG" /etc/armd/config.json
sudo install -m 644 deploy/systemd/armd.service /etc/systemd/system/armd.service
sudo systemctl daemon-reload
sudo systemctl enable --now armd
echo "armd 已安装：systemctl status armd；日志：journalctl -u armd -f"
