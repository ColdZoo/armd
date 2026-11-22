#!/usr/bin/env bash
# 交叉编译 armd 到 ARM Linux（64 位 / 32 位）。
#
# 依赖（macOS 本机已装）：
#   brew install zig
#   curl -sSf https://sh.rustup.rs | sh -s -- -y --profile minimal   # 提供 rustup target
#   ~/.cargo/bin/rustup target add aarch64-unknown-linux-gnu armv7-unknown-linux-gnueabihf
#   ~/.cargo/bin/cargo install cargo-zigbuild --locked
#
# 产物：dist/armd-aarch64  dist/armd-armv7（同时保留在 target/<triple>/release/）
# 注：serialport 已关闭 libudev 默认特性，交叉编译不需要 ARM sysroot。
set -euo pipefail
cd "$(dirname "$0")/.."
CARGO="${CARGO:-$HOME/.cargo/bin/cargo}"
mkdir -p dist
for target in aarch64-unknown-linux-gnu armv7-unknown-linux-gnueabihf; do
  echo "==> $target"
  case "$target" in
    aarch64*) out="dist/armd-aarch64" ;;
    armv7*)   out="dist/armd-armv7" ;;
  esac
  "$CARGO" zigbuild --release --target "$target"
  cp "/tmp/armd-target/$target/release/armd" "$out"
  file "$out"
done
echo "完成：dist/armd-aarch64 dist/armd-armv7"
