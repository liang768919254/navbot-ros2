#!/usr/bin/env bash
# ============================================================================
# audit_scripts.sh —— 脚本自审（专抓 bash -n 抓不到的那类错）
# ============================================================================
# 为什么需要这个文件？
#   `bash -n` 只能查**语法**，查不出**语义**。本项目在实测中反复栽过这一类：
#
#     echo" 少一个空格   → bash -n 通过！运行时才报
#                         "行495: echo xxx: 未找到命令"
#     cmd | tee log      → 退出码是 tee 的（恒 0），错误分支永不触发
#     if cmd | sed; then→ 同上，恒真
#
#   这三类错误的共同点：**语法合法、行为错误**，只能靠"运行"或"专门扫描"发现。
#   本脚本把后者自动化，改完脚本跑一下就能拦住。
#
# 用法：bash audit_scripts.sh
# 退出码：0 = 干净；1 = 有问题
# ============================================================================

set -u
cd "$(dirname "$0")" || exit 1

SCRIPTS="check_env.sh setup_env.sh diag_ros_apt_key.sh record_navigation.sh check_sim.sh"
ISSUES=0
say() { printf '%s\n' "$*"; }
ok()  { printf '  \033[32m✓\033[0m %s\n' "$*"; }
bad() { printf '  \033[31m✗\033[0m %s\n' "$*"; ISSUES=$((ISSUES+1)); }
warn(){ printf '  \033[33m!\033[0m %s\n' "$*"; }

say "=================================================="
say " 脚本自审（bash -n 抓不到的错误）"
say "=================================================="

for f in $SCRIPTS; do
  say ""
  say "【$f】"

  if [ ! -f "$f" ]; then
    warn "文件不存在，跳过"
    continue
  fi

  # ---- 1. 语法（能查的先查）----
  if bash -n "$f" 2>/dev/null; then
    ok "语法检查通过（bash -n）"
  else
    bad "有语法错误："
    bash -n "$f" 2>&1 | head -5 | sed 's/^/      /'
  fi

  # ---- 2. 命令名与引号之间缺空格（bash -n 查不出）----
  #     echo"xxx" 在 bash 里语法完全合法 —— 它被解释成"执行一个叫 echo"xxx" 的命令"
  #     判据交给 python：正则里同时含单双引号，bash 侧拼引号极易出错（已实测踩到）
  HITS=$(python - "$f" <<'PYEOF' 2>/dev/null
import sys, re
fn = sys.argv[1]
try:
    lines = open(fn, encoding='utf-8').read().split('\n')
except Exception:
    sys.exit(0)
# 命令名后【紧跟】引号（中间零空白）= 真错误；有 1 个空格就是合法的。
# 判据不能只搜「命令名+引号」，否则函数定义行 say() { printf '%s\n' "$*"; }
# 这类【命令名后紧跟圆括号】的正常写法会误报 —— 所以要求前面是行首或分隔符。
#   合法： ^echo "  |  ; echo "  |  && echo "  |  then echo "  |  ( echo "
#   错误： ^echo"  |  ; echo"  |  then echo"
pat = re.compile(r'(?:^|[;&|]|\bthen|\bdo|\)\s*&&)\s*(echo|printf|say|ok|bad|warn|tip)["\']')
out = []
for i, l in enumerate(lines, 1):
    if l.strip().startswith('#'):
        continue          # 注释里出现 "echo\"" 是文档，不是错误
    if pat.search(l):
        out.append(str(i) + ':' + l.strip()[:70])
print('\n'.join(out))
PYEOF
)
  if [ -n "$HITS" ]; then
    bad "命令名后缺空格（运行时才会炸：command not found）"
    printf '%s\n' "$HITS" | head -6 | sed 's/^/      行 /'
  else
    ok "无「命令缺空格」问题"
  fi
  # ---- 3. 管道退出码：末段是 tee/sed/grep/head 会吞掉真实退出码----
  #     若脚本用管道结果做判断，就必须显式开 pipefail
  #
  #     但不是所有管道都要 pipefail —— 要看【末段是什么】：
  #       末段 grep -q / test / diff -q这类「自己产出退出码」的 → 退出码可靠，不用管
  #       末段 tee / sed / head / cat / wc  → 退出码恒 0，必须开 pipefail
  #     本项目 record_navigation.sh 的 rcli() 就是靠末段 `|| true`兜的，属于另一类。
  RISKY=$(grep -nE '^\s*(if|while|until).*\|.*(tee|sed|head|cat|wc|sort|uniq|tail)[^|]*;\s*then' "$f" 2>/dev/null)
  ANY_PIPE=$(grep -cE '^\s*(if|while|until).*\|' "$f" 2>/dev/null)
  if [ "$ANY_PIPE" -eq 0 ]; then
    ok "无管道判断"
  elif [ -n "$RISKY" ]; then
    if grep -q 'set -o pipefail' "$f"; then
      ok "有风险管道（末段 tee/sed/head），且已开 pipefail"
    else
      bad "★ 管道末段是 tee/sed/head/cat 之类（退出码恒 0），但【全篇没有 pipefail】"
      printf '%s\n' "$RISKY" | head -4 | sed 's/^/      行 /'
      say   "         ⇒ 这些 if/while 永远走成功分支，错误处理形同虚设"
      say   "         修法：加 set -o pipefail，或改用重定向到文件再判断"
    fi
  else
    ok "管道末段是 grep -q / test 等（退出码可靠，无需 pipefail）"
  fi

  # ---- 4. set -e 下裸跑会崩的命令（给个提醒，不是错）----
  if grep -q '^set -e' "$f" 2>/dev/null; then
    BARE=$(grep -cE '^\s*(rosdep update|apt update|colcon build)\s*(\||$)' "$f" 2>/dev/null)
    if [ "${BARE:-0}" -gt 0 ]; then
      warn "有 set -e 脚本裸跑可能失败的命令（建议抓日志 + 判退出码）"
    else
      ok "set -e 下没有明显裸跑的危险命令"
    fi
  fi

  # ---- 5. 中英文之间漏空格（只提示，属排版）----
  SP=$(grep -nP '^[^#]*[\x{4e00}-\x{9fa5}][A-Za-z0-9]|^[^#]*[A-Za-z0-9][\x{4e00}-\x{9fa5}]' "$f" 2>/dev/null | head -3)
  if [ -n "$SP" ]; then
    warn "有中英文之间漏空格（不影响运行，属排版）"
    printf '%s\n' "$SP" | sed 's/^/      行 /'
  fi
done

# ---- 汇总 ----
say ""
say "=================================================="
if [ "$ISSUES" -eq 0 ]; then
  say " ✅ 全部通过"
  say "=================================================="
  exit 0
fi
say " ❌ 检出 $ISSUES 类问题"
say "=================================================="
say ""
say " ★ 重点：第 2、3 类是 bash -n 【查不出】的运行时错误，"
say "   本项目已在这两类上栽过多次，提交前务必跑本自审。"
exit 1
