#!/usr/bin/env bash
# 交叉编译 armd 到 ARM Linux（64 位）。
#
# 方案 A（推荐）：cargo-zigbuild（需要 zig，跨平台链接最省事）
#   brew install zig && cargo install cargo-zigbuild
#   rustup target add aarch64-unknown-linux-gnu
#   ./scripts/build-arm.sh
#
# 方案 B：在 ARM 板上直接编译（最简单，板上 `rustup target add` 后 cargo build --release）
# 方案 C：CI/容器里用 aarch64 工具链
set -euo pipefail
TARGET="${TARGET:-aarch64-unknown-linux-gnu}"
echo "==> 目标: $TARGET"
rustup target list --installed | grep -q "$TARGET" || rustup target add "$TARGET"
if command -v cargo-zigbuild >/dev/null 2>&1 && command -v zig >/dev/null 2>&1; then
  echo "==> 使用 cargo-zigbuild"
  cargo zigbuild --release --target "$TARGET"
else
  echo "==> 尝试普通交叉编译（需要现成的 ${TARGET%%-*}-linux-gnu-gcc 链接器）"
  cargo build --release --target "$TARGET"
fi
BIN="target/$TARGET/release/armd"
[ -f "$BIN" ] || BIN="/tmp/armd-target/$TARGET/release/armd"
echo "==> 产物: $BIN"
file "$BIN" || true
echo "把它们拷到板上：scp $BIN arm-board:/tmp/armd && ssh arm-board 'sudo install -m755 /tmp/armd /opt/armd/armd'"
