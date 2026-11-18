#!/usr/bin/env bash
# 一键看测试进度：编译 → 跑测试 → 列出还没过的用例 + 失败原因
#
# 用法：
#   ./check_progress.sh              # 完整跑（编译 + 测试）
#   ./check_progress.sh -t           # 只跑测试，不编译（改完代码后常用）
#   ./check_progress.sh -v           # 显示每个失败用例的详细断言输出
#
# 放在包根目录执行，或任意位置用绝对路径调。

set -uo pipefail

PKG=navbot_chassis
# 脚本所在目录视为包根目录
PKG_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$PKG_DIR/../.." && pwd)"   # src/<pkg> → 工作空间根

SKIP_BUILD=0
VERBOSE=0
for arg in "$@"; do
  case "$arg" in
    -t) SKIP_BUILD=1 ;;
    -v) VERBOSE=1 ;;
    *) echo "未知参数: $arg"; exit 2 ;;
  esac
done

if [ ! -f /opt/ros/humble/setup.bash ]; then
  echo "✘ 找不到 /opt/ros/humble/setup.bash，ROS 2 没装？"
  exit 1
fi
# shellcheck disable=SC1091
# ★ ROS 的 setup.bash 里有未定义变量，必须临时关掉 set -u 才能 source
set +u
source /opt/ros/humble/setup.bash
set -u

cd "$WS_DIR" || exit 1

if [ "$SKIP_BUILD" -eq 0 ]; then
  echo "=== 编译 $PKG ==="
  # ★ Python3_EXECUTABLE 防止 CMake 抓到非系统 python3
  if ! colcon build --packages-select "$PKG" \
       --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 2>&1 | tail -3; then
    echo "✘ 编译失败，先修编译"
    exit 1
  fi
  echo
fi

echo "=== 跑测试 ==="
colcon test --packages-select "$PKG" >/dev/null 2>&1

XML="$WS_DIR/build/$PKG/test_results/$PKG/test_protocol.gtest.xml"
if [ ! -f "$XML" ]; then
  echo "✘ 找不到测试结果 $XML"
  echo "  先跑一次完整编译（不带 -t）"
  exit 1
fi

python3 - "$XML" "$VERBOSE" << 'PY'
import sys
import xml.etree.ElementTree as ET

xml_path, verbose = sys.argv[1], sys.argv[2] == '1'
root = ET.parse(xml_path).getroot()

total = int(root.get('tests', 0))
fails = int(root.get('failures', 0)) + int(root.get('errors', 0))
passed = total - fails

bar = '█' * passed + '░' * fails
print(f"进度  {bar}")
print(f"      通过 {passed} / {total}    失败 {fails}")
print()

failed_cases = []
for case in root.iter('testcase'):
    cls = case.get('classname') or ''
    suite = cls.split('.')[-1] if cls else '(unknown)'
    name = case.get('name') or ''
    lines = []
    for bad in list(case.iter('failure')) + list(case.iter('error')):
        msg = (bad.get('message') or '').strip()
        # gtest 把「哪一行的断言挂了」放在 message 第一行，形如
        # /path/to/test_x.cpp:123 或 "Value of: ..."
        if msg:
            lines.append(msg.splitlines()[0].strip())
    if lines:
        failed_cases.append((suite, name, lines, (bad.text or '').strip()))

if not failed_cases:
    print("✔ 全部通过。四关都过了。")
    sys.exit(0)

print(f"还没过的用例（共 {len(failed_cases)} 个）：")
for suite, name, lines, _text in failed_cases:
    where = ''
    for ln in lines:
        if '.cpp:' in ln or '.hpp:' in ln:
            where = '  @ ' + ln.split('/')[-1]
            break
    print(f"  ✘ {suite}.{name}{where}")

# 按 suite 汇总，方便看还有几关没过
print()
suites = {}
for suite, name, *_ in failed_cases:
    suites.setdefault(suite, []).append(name)
level = {'Kinematics': '第 1 关', 'VelocityFormat': '第 2 关',
         'OdomParse': '第 3 关', 'FrameReader': '第 4 关'}
for suite in ('Kinematics', 'VelocityFormat', 'OdomParse', 'FrameReader'):
    if suite in suites:
        print(f"  {level.get(suite, suite):6s} {suite:16s} 还差 {len(suites[suite])} 个")
PY
