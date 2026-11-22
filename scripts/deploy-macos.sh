#!/usr/bin/env bash
# macOS（air.my）部署：构建 release → 安装 → 重签 → 重启 LaunchAgent。
#
# 注意：macOS 26 会给拷贝过的二进制打 provenance 标记，未重签会被 taskgated
# 以 "Code Signature Invalid" SIGKILL。所以复制后必须 ad-hoc 重签。
set -euo pipefail
cd "$(dirname "$0")/.."
cargo build --release
mkdir -p ~/.local/bin ~/.config/armd
cp /tmp/armd-target/release/armd ~/.local/bin/armd
codesign --force --sign - ~/.local/bin/armd >/dev/null 2>&1 || true
codesign -v ~/.local/bin/armd
if [ ! -f ~/.config/armd/config.json ]; then
  cp configs/koch.json ~/.config/armd/config.json
  echo "已生成默认配置 ~/.config/armd/config.json"
fi
launchctl kickstart -k gui/$(id -u)/com.armd 2>/dev/null || \
  echo "LaunchAgent com.armd 未加载：launchctl bootstrap gui/$(id -u) ~/Library/LaunchAgents/com.armd.plist"
sleep 2
~/.local/bin/armd --version
echo "≈ 部署完成；日志 /tmp/armd.log，identity /tmp/armd-identity.json"
