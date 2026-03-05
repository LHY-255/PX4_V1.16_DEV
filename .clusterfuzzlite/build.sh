#!/usr/bin/env bash -eu

# 强制 Clang 使用 lld 链接器，防止 ld.gold 在处理 ASan 调试信息时崩溃
export CFLAGS="$CFLAGS -fuse-ld=lld"
export CXXFLAGS="$CXXFLAGS -fuse-ld=lld"
export LDFLAGS="$LDFLAGS -fuse-ld=lld"

PX4_FUZZ=1 make px4_sitl
cp build/px4_sitl_default/bin/px4 $OUT/px4