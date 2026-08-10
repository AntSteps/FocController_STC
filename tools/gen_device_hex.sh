#!/usr/bin/env bash
#
# gen_device_hex.sh — 批量生成不同设备ID的固件 HEX 文件
#
# 原理:
#   DEVICE_ID 定义于 Sources/inc/protocol_frame.h。编译后该值以 9 处立即数
#   常量分散在 hex 中(7 处回复帧写入 + 2 处地址过滤), 且地址随代码改动漂移,
#   因此不做 hex 层面补丁, 而是逐 id 改写头文件后调用 Keil 命令行批量构建。
#   实测单次增量构建约 6~7 秒 (仅重编译包含该头文件的 7 个源文件)。
#
# 用法:
#   ./tools/gen_device_hex.sh               # 生成 id 0..10 (共 11 个)
#   ./tools/gen_device_hex.sh 5 6 7         # 仅生成指定 id
#   IDS="0 2 4" ./tools/gen_device_hex.sh   # 同上, 通过环境变量
#
# 环境变量:
#   KEIL_UV4   UV4.exe 路径, 默认 /home/fuwaki/.wine/drive_c/Keil_v5/UV4/UV4.exe
#   OUT_DIR    输出目录, 默认 <项目根>/hex_output
#
# 注意: 运行期间请关闭 uVision GUI, 避免占用工程/输出文件。
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
HEADER="$PROJECT_DIR/Sources/inc/protocol_frame.h"
UVPROJ="$PROJECT_DIR/FocController_STC.uvproj"
HEX_SRC="$PROJECT_DIR/Objects/FocController_STC.hex"
UV4="${KEIL_UV4:-/home/fuwaki/.wine/drive_c/Keil_v5/UV4/UV4.exe}"
OUT_DIR="${OUT_DIR:-$PROJECT_DIR/hex_output}"
BUILD_LOG="$(mktemp /tmp/uv4_build_XXXXXX.log)"

# ---------- 前置检查 ----------
[ -f "$HEADER" ] || { echo "错误: 找不到 $HEADER" >&2; exit 1; }
[ -f "$UVPROJ" ] || { echo "错误: 找不到 $UVPROJ" >&2; exit 1; }
[ -f "$UV4" ]    || { echo "错误: 找不到 UV4.exe: $UV4 (可用 KEIL_UV4 指定)" >&2; exit 1; }
command -v wine   >/dev/null 2>&1 || { echo "错误: 未找到 wine" >&2; exit 1; }
command -v python3 >/dev/null 2>&1 || { echo "错误: 未找到 python3" >&2; exit 1; }

# 记录头文件当前 DEVICE_ID 值(如 0x03u), 结束后还原
ORIG_VALUE="$(grep -oP '^#define\s+DEVICE_ID\s+\K\S+' "$HEADER" || true)"
[ -n "$ORIG_VALUE" ] || { echo "错误: 头文件中未找到 DEVICE_ID 宏定义" >&2; exit 1; }

# ---------- 解析 id 列表 ----------
if [ $# -gt 0 ]; then
  IDS="$*"
elif [ -z "${IDS:-}" ]; then
  IDS="$(seq 0 10)"
fi
for id in $IDS; do
  case "$id" in
    ''|*[!0-9]*) echo "错误: 非法 id '$id'" >&2; exit 1;;
  esac
  [ "$id" -ge 0 ] && [ "$id" -le 255 ] || { echo "错误: id '$id' 超出 0..255" >&2; exit 1; }
done
mkdir -p "$OUT_DIR"

# ---------- 工具函数 ----------
# 改写头文件中的 DEVICE_ID 数值 token (保留原有缩进)
set_id() { # $1 = 新值, 如 0x05u
  # 字节级替换: 保留原文件的换行符(CRLF)与编码, 只改数值 token
  python3 - "$HEADER" "$1" <<'PY'
import re, sys
path, val = sys.argv[1], sys.argv[2]
src = open(path, 'rb').read()
new, n = re.subn(rb'^(\s*#define\s+DEVICE_ID\s+)\S+',
                 lambda m: m.group(1) + val.encode(), src, count=1, flags=re.M)
if n != 1:
    print(f'错误: 在 {path} 中未找到 DEVICE_ID 宏定义', file=sys.stderr)
    sys.exit(1)
open(path, 'wb').write(new)
PY
}

restore() {
  set_id "$ORIG_VALUE" 2>/dev/null || true
  echo "已还原 $HEADER 的 DEVICE_ID 为默认 $ORIG_VALUE"
}
trap restore EXIT

# 构建一次并校验: 退出码 / 日志 0 Error / hex 与上次必须不同
LAST_SUM=""
build() { # $1 = id (十进制)
  local id="$1" val nn new_sum
  val="$(printf '0x%02Xu' "$id")"

  set_id "$val"
  grep -q "^#define[[:space:]]\+DEVICE_ID[[:space:]]\+$val" "$HEADER" \
    || { echo "错误: 头文件改写未生效 ($val)" >&2; exit 1; }

  echo "==> 构建 id=$id ($val) ..."
  if ! timeout 300 env WINEDEBUG=-all wine "$UV4" -b "$UVPROJ" -o "$BUILD_LOG" >/dev/null 2>&1; then
    echo "错误: UV4 构建失败 (退出码 $?)" >&2; tail -30 "$BUILD_LOG" >&2; exit 1
  fi
  if ! grep -q "0 Error(s)" "$BUILD_LOG"; then
    echo "错误: 构建报告编译错误:" >&2; tail -30 "$BUILD_LOG" >&2; exit 1
  fi
  [ -f "$HEX_SRC" ] || { echo "错误: 未生成 $HEX_SRC" >&2; exit 1; }

  new_sum="$(sha256sum "$HEX_SRC" | cut -d' ' -f1)"
  if [ -n "$LAST_SUM" ] && [ "$new_sum" = "$LAST_SUM" ]; then
    echo "错误: hex 与上一次构建完全相同, DEVICE_ID 改写未生效" >&2; exit 1
  fi
  LAST_SUM="$new_sum"

  nn="$(printf '%02d' "$id")"
  cp "$HEX_SRC" "$OUT_DIR/FocController_STC_id${nn}.hex"
  echo "    -> $OUT_DIR/FocController_STC_id${nn}.hex"
}

# ---------- 主流程 ----------
echo "项目:      $PROJECT_DIR"
echo "UV4:       $UV4"
echo "输出目录:  $OUT_DIR"
echo "目标 id:   $IDS"
echo

start="$(date +%s)"
for id in $IDS; do
  build "$id"
done
elapsed="$(( $(date +%s) - start ))"

# 还原默认 id 并重建一次, 保持工作区 Objects/hex 与头文件一致
set_id "$ORIG_VALUE"
if timeout 300 env WINEDEBUG=-all wine "$UV4" -b "$UVPROJ" -o "$BUILD_LOG" >/dev/null 2>&1 \
   && grep -q "0 Error(s)" "$BUILD_LOG"; then
  echo "已还原默认构建 (DEVICE_ID=$ORIG_VALUE)"
else
  echo "警告: 默认 id 还原构建失败, 请稍后手动重新构建" >&2
fi

echo
echo "完成: $elapsed 秒, 输出 $OUT_DIR:"
ls -l "$OUT_DIR" | tail -n +2
