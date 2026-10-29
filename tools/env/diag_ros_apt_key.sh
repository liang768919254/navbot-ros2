#!/usr/bin/env bash
# ============================================================================
# diag_ros_apt_key.sh —— 专治「配了 ROS2 源但 apt 说 NO_PUBKEY」
# ============================================================================
# 用途：setup_env.sh v2 打印「✅ 密钥指纹校验通过」，apt update 却报
#   由于没有公钥，无法验证下列签名： NO_PUBKEY F42ED6FBAB17C654
#   E: 仓库 "..." 没有数字签名。
#
#   为什么会矛盾：脚本用 `gpg --show-keys` 校验，那是【以 root 身份】读的，
#     读得到 600 的文件；而 apt 验签是【以 _apt 用户身份跑 gpgv】，
#     _apt 读不到 600 ⇒ 等于没装key。本脚本用和 apt 同一个视角来查。
#
# 用法：bash diag_ros_apt_key.sh
# 退出码：0 = 一切正常；1 = 检出问题（并给出修复命令）
# ============================================================================

set -u

KEYRING="/usr/share/keyrings/ros-archive-keyring.gpg"
ROSLIST="/etc/apt/sources.list.d/ros2.list"
FPR="C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654"
PROBLEMS=0

say()  { printf '%s\n' "$*"; }
ok()   { printf '  \033[32m%s\033[0m\n' "$*"; }
bad()  { printf '  \033[31m%s\033[0m\n' "$*"; PROBLEMS=$((PROBLEMS+1)); }
warn() { printf '  \033[33m%s\033[0m\n' "$*"; }

say "=================================================="
say " ROS2 apt 密钥诊断（用 apt 的视角查）"
say "=================================================="

# ---- 1. 密钥文件存在吗 ----
say ""
say "【1】密钥文件"
if [ -s "$KEYRING" ]; then
  ok "存在：$KEYRING"
else
  bad "不存在或为空：$KEYRING"
  say "     修：sudo curl -fsSL https://mirrors.ustc.edu.cn/rosdistro/ros.key -o $KEYRING"
  say "        sudo chmod 644 $KEYRING"
  exit 1
fi

# ---- 2. 权限（本bug 的真凶，重点看这项）----
say ""
say "【2】文件权限（★ 最可能的问题）"
PERM=$(stat -c '%a' "$KEYRING" 2>/dev/null || echo "?")
OWNER=$(stat -c '%U:%G' "$KEYRING" 2>/dev/null || echo "?")
say "     权限=$PERM  属主=$OWNER"
case "$PERM" in
  6??|4??|7??|2??)
    # _apt 属于 "other" 类，other 位(r=4)必须为 1
    OTHER_BITS=$(printf '%s' "$PERM" | tail -c 1)
    if [ "$OTHER_BITS" = "0" ]; then
      bad "★ 这就是 NO_PUBKEY 的原因：other 位是 0，_apt 用户读不到"
      say "     _apt 只按【other】权限读这个文件（它既不是 root 也不是属主）"
      say "     当前 $PERM ⇒ 属主可读、其他用户一律拒绝"
      say "     修：sudo chmod 644 $KEYRING"
    else
      ok "other 位可读（other=$OTHER_BITS）⇒ _apt 读得到"
    fi
    ;;
  *)
    bad "权限 $PERM 不是常规值，请人工确认：ls -l $KEYRING"
    ;;
esac

# ---- 3. 指纹对不对 ----
say ""
say "【3】密钥指纹（root 视角，注意：这步过了不代表 apt 能用）"
if gpg --show-keys "$KEYRING" 2>/dev/null | grep -q "$FPR"; then
  ok "指纹正确：$FPR"
else
  bad "指纹不对或读不出"
  say "     修：sudo rm -f $KEYRING"
  say "        sudo curl -fsSL https://mirrors.ustc.edu.cn/rosdistro/ros.key -o $KEYRING"
  say "        sudo chmod 644 $KEYRING"
fi

# ---- 4. 文件格式（armored 伪装成 .gpg 是经典坑）----
say ""
say "【4】文件格式"
MAGIC=$(od -A n -t x1 -N 2 "$KEYRING" 2>/dev/null | tr -d ' \n')
if [ "$MAGIC" = "2d2d" ] || [ "$MAGIC" = "2d2d2d" ]; then
  warn "文件是 ASCII armored，但扩展名是 .gpg"
  say "     apt 的 signed-by 期望二进制 keyring；多数版本能容忍，但为稳妥建议转成二进制："
  say "     修：gpg --dearmor < $KEYRING > /tmp/k.gpg && sudo cp /tmp/k.gpg $KEYRING && sudo chmod 644 $KEYRING"
elif [ -n "$MAGIC" ]; then
  ok "二进制 keyring（magic=$MAGIC）"
fi

# ---- 5. 源文件里的 signed-by 路径 ----
say ""
say "【5】ros2.list 的 signed-by 路径"
if [ -f "$ROSLIST" ]; then
  ok "ros2.list 存在"
  LINE=$(grep -vE '^\s*(#|$)' "$ROSLIST" | head -1)
  say "     内容：$LINE"
  SB=$(printf '%s' "$LINE" | sed -n 's/.*signed-by=\([^]]*\).*/\1/p')
  if [ -z "$SB" ]; then
    bad "这一行没有 signed-by=（若 keyring 权限不对就必然 NO_PUBKEY）"
    say "     修：sudo tee $ROSLIST > /dev/null <<'EOF'"
    say "     deb [arch=amd64 signed-by=$KEYRING] https://mirrors.ustc.edu.cn/ros2/ubuntu jammy main"
    say "     EOF"
  elif [ "$SB" != "$KEYRING" ]; then
    bad "signed-by 指向 '$SB'，但密钥在 '$KEYRING' —— 不一致"
    say "     修：sudo sed -i 's|signed-by=[^]]*|signed-by=$KEYRING|' $ROSLIST"
  else
    ok "signed-by 路径与密钥位置一致"
  fi
  # 路径含空格/换行等异常字符
  case "$LINE" in
    *" "*) warn "源行里有空格，确认不是换行被折断：$LINE" ;;
  esac
else
  bad "$ROSLIST 不存在（源没写成功）"
fi

# ---- 6. 终极验证：用 apt 内部同款 gpgv 真验一次 ----
#   这一项是本脚本的"决定性检查"，必须有明确结论：
#     验签通过         → 才算密钥真的可用
#     验签失败         → 报问题
#     拉不到 InRelease → 【不算通过】！降级为"网络问题，另行确认"（v1 初版栽在这：
#       网络失败时只warn 不计数，最后却打印"✅ 未发现密钥问题"，属误报）
say ""
say "【6】终极验证：gpgv 验签（这就是 apt 干的事）"
MIRROR=$(grep -oE 'https?://[^ ]+' "$ROSLIST" 2>/dev/null | head -1)
[ -z "$MIRROR" ] && MIRROR="https://mirrors.ustc.edu.cn/ros2/ubuntu"
CODENAME=$(lsb_release -cs 2>/dev/null || echo jammy)
say "     源：$MIRROR/dists/$CODENAME/InRelease"

# 多镜像 + 官方兜底：某个镜像拉不到就换下一个，别因单点网络问题下不了结论
VERIFY_STATE="未验证"
TRY_MIRRORS=("$MIRROR"
             "https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu"
             "https://mirrors.aliyun.com/ros2/ubuntu"
             "http://packages.ros.org/ros2/ubuntu")
for m in "${TRY_MIRRORS[@]}"; do
  TMP=$(mktemp)
  if curl -fsSL --max-time 25 "$m/dists/$CODENAME/InRelease" -o "$TMP" 2>/dev/null; then
    if gpgv --keyring "$KEYRING" "$TMP" >/dev/null 2>&1; then
      ok "验签通过（用 $(echo "$m" | cut -d/ -f3)的 InRelease）⇒ apt 能正常验签"
      VERIFY_STATE="通过"
    else
      bad "验签失败（$(echo "$m" | cut -d/ -f3)）⇒ 这就是 apt 报 NO_PUBKEY 的原因"
      say "     gpgv 读不到密钥或签名不匹配。手工复核："
      say "       ls -l $KEYRING        （必须 -rw-r--r--，other 位可读）"
      say "       gpgv --keyring $KEYRING <InRelease 文件>"
      VERIFY_STATE="失败"
    fi
    rm -f "$TMP"
    break        # 已拿到 InRelease，无论验签结果如何都算这一项有结论了
  fi
  rm -f "$TMP"
done

if [ "$VERIFY_STATE" = "未验证" ]; then
  # 关键：拉不到 InRelease 不能当成"没问题"，必须让用户知道这一项没验成
  warn "四个镜像都拉不到 InRelease ⇒【本项无法验证】（不代表密钥没问题）"
  say "     先排除网络：curl -I $MIRROR/dists/$CODENAME/InRelease"
  say "     网络通了再重跑本脚本；或直接跑 apt update 看是否还报 NO_PUBKEY。"
  say "     ★ 只要 apt update 不再出现 NO_PUBKEY，就算通过。"
  UNVERIFIED=1
else
  UNVERIFIED=0
fi

# ---- 结论 ----
say ""
say "=================================================="
if [ "$PROBLEMS" -eq 0 ] && [ "$UNVERIFIED" -eq 0 ]; then
  say " ✅ 全部检查通过（含 gpgv 真验签），可以直接：sudo apt update"
  say "=================================================="
  exit 0
fi
if [ "$PROBLEMS" -eq 0 ] && [ "$UNVERIFIED" -eq 1 ]; then
  say " ⚠️  前 5 项都没问题，但【第 6 项验签未能完成】（网络原因）"
  say "=================================================="
  say ""
  say " 密钥本身看不出毛病。请直接用下面这条做最终判定："
  say ""
  say "   sudo apt update 2>&1 | grep -E 'NO_PUBKEY|签名|ros2' "
  say ""
  say "   · 没有 NO_PUBKEY / 签名报错 → 一切正常，继续跑 setup_env.sh 即可"
  say "   · 仍有 NO_PUBKEY → 把上面 diag_ros_apt_key.sh 的完整输出发出来"
  exit 2
fi
say " ❌ 检出 $PROBLEMS 处问题"
say "=================================================="
say ""
say " 最省事的一次性修复（直接复制粘贴）："
say ""
say "   sudo rm -f $KEYRING"
say "   sudo curl -fsSL https://mirrors.ustc.edu.cn/rosdistro/ros.key \\"
say "     | sudo tee $KEYRING > /dev/null"
say "   sudo chmod 644 $KEYRING"
say "   sudo apt update"
say ""
say " 修完再跑一次本脚本确认（bash diag_ros_apt_key.sh），"
say " 没问题再回去跑 setup_env.sh。"
exit 1
