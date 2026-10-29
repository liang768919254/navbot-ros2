# G1 · 把本地代码推送到 GitHub 详细步骤

> 适用场景：本项目仓库位于 `C:\Users\Administrator\Desktop\具身智能学习路线\bridge\ros_study`，
> 远端为 `git@github.com:liang768919254/navbot-ros2.git`（SSH 方式）。

---

## 0. 开工前的体检结果（2026-10-01 已实测）

| 检查项 | 结果 | 含义 |
|---|---|---|
| SSH 认证 | ✅ `Hi liang768919254! You've successfully authenticated` | 钥匙已经配好，**不用输密码/token，直接能推** |
| 远端 main | `72b70bc`（= init 提交） | 远端还是初始提交，本地领先，属于最省事的情况 |
| 本地 HEAD | `72b70bc` | 与远端一致，没有"远端有新提交"的冲突 |
| 提交者身份 | `liang768919254` / `liang768919254@foxmail.com` | 已配置，提交记录不会是乱码/匿名 |
| 脚本换行符 | `record_navigation.sh`、`check_sim.sh`、`setup_env.sh`、`无硬件自测/*` **全是 LF** ✅ | 不会被 CRLF 坑（见 §5 第 2 条） |
| `.gitattributes` | ❌ 原本没有 | **本次已补上**，用来永久锁住 LF |

体检命令（自己复查时用）：

```bash
cd "C:/Users/Administrator/Desktop/具身智能学习路线/bridge/ros_study"

git config --global user.name && git config --global user.email   # 身份
ssh -T git@github.com                                            # SSH 通不通（回 Hi 用户名 就是通）
git ls-remote --heads origin                                      # 远端现在有什么
git status --short                                                # 本地有什么没提交
```

---

## 1. 先决定"提交什么"——别一把梭 `git add -A`

当前 9 个待处理文件，**建议按下面分类**：

| 文件 | 建议 | 理由 |
|---|---|---|
| `record_navigation.sh` | ✅ 提交 | 核心录制脚本，README 的动图就靠它 |
| `check_sim.sh` | ✅ 提交 | 仿真层单独体检脚本，和上面是配套的 |
| `docs/A5_知识概念全解.md` | ✅ 提交 | 知识文档，是仓库的价值项 |
| `src/navbot_bringup/launch/nav_bringup.launch.py`（修改） | ✅ 提交 | 启动配置改动，不提交别人跑不起来 |
| `.gitattributes` | ✅ 提交 | 本次新增，锁 LF |
| `record_navigation.sh.v1.bak` | ❌ 不提交 | 备份文件。**git 本身就是版本管理**，要留历史就靠 commit |
| `src/navbot_bringup/maps/navbot_room.pgm` | ⚠️ 你来定 | 只有 12 KB，**不是**"大文件"。提交后别人 clone 下来能直接复现导航；不提交则要自己重新建图 |
| `docs/_诊断_20261001_nav_gif/*.png` | ⚠️ 建议不提交 | 2.5 MB 的**失败录制**证据图，更适合放仓库外归档 |

> 说明：`.gitignore` 里那句 `maps/*.pgm` **不生效**（不带斜杠前缀的写法才匹配任意层级；
> `maps/*.pgm` 含斜杠 ⇒ 只匹配仓库根目录的 `maps/`），所以那份 pgm 才一直漂在未跟踪列表里。
> 要真正忽略它，应改成 `**/maps/*.pgm`。

---

## 2. 标准流程（6 步）

### Step 1 · 进仓库，看现状

```bash
cd "C:/Users/Administrator/Desktop/具身智能学习路线/bridge/ros_study"
git status --short
git log --oneline -3
```

### Step 2 · 预演（不写任何东西，先看清楚会提交哪些）

```bash
git add -A --dry-run
```

`--dry-run` 只打印 `add 'xxx'`，**不会真的暂存**。这一步是防"手滑把 2 GB 数据集推上去"最便宜的一道闸。

### Step 3 · 分批暂存（推荐分成两个提交，历史更好读）

**提交 1：录制 / 体检脚本链**

```bash
git add .gitattributes record_navigation.sh check_sim.sh docs/A5_知识概念全解.md
git commit -m "feat: 新增无头环境导航录制与仿真体检脚本

- record_navigation.sh：Xvfb + import 录 Navigation2 为 GIF，含四道仿真门、
  目标响应丢失重发、卡住检测、方形绕行
- check_sim.sh：只验仿真层，单点排查更快
- .gitattributes：锁定 .sh/.py/.yaml 为 LF，防 CRLF 破坏容器内执行"
```

**提交 2：启动配置修改**

```bash
git add src/navbot_bringup/launch/nav_bringup.launch.py
git commit -m "fix: 修正 nav_bringup 启动配置"
```

想把地图也纳入版本管理（12 KB，推荐，别人 clone 就能导航）：

```bash
git add src/navbot_bringup/maps/navbot_room.pgm
git commit -m "chore: 纳入建图产物 navbot_room.pgm（12KB，便于复现导航）"
```

不想让 `.bak` 和诊断图出现在 `git status` 里，就写进 `.gitignore`：

```bash
printf '\n# ==== 本地备份与诊断产物（不入库）====\n*.bak\n*.bak.*\ndocs/_诊断_*/\n' >> .gitignore
git add .gitignore && git commit -m "chore: 忽略本地备份与诊断产物"
```

### Step 4 · 推送到 GitHub

```bash
git push origin main
```

首次推送某个分支才需要带 `-u`（把本地分支和远端绑定，之后可只写 `git push`）：

```bash
git push -u origin main
```

### Step 5 · 验证（三处都要对得上）

```bash
git log --oneline -3                     # ① 本地有没有新提交
git ls-remote --heads origin             # ② 远端 main 的 hash 是否 = 本地 HEAD
git status -sb                           # ③ 应显示 ## main...origin/main（没有 ahead）
```

再打开 `https://github.com/liang768919254/navbot-ros2` 刷一下页面：文件列表、提交记录、文件内容都应在。

### Step 6 · 一次完整的"改动→上线"循环（以后照这个走）

```bash
git status --short          # 1 改了什么
git add -A --dry-run        # 2 预演
git add <具体文件>           # 3 暂存（别用 -A 图省事）
git commit -m "类型: 说明"   # 4 提交
git push                    # 5 推送
git log --oneline -3        # 6 复核
```

---

## 3. 把 GIF 放进 README（可选，但这是仓库的门面）

```bash
# ① 把录好的 GIF 放进仓库根目录（建议改个正经名字）
cp "/c/Users/Administrator/Downloads/navigation (1).gif" ./navigation.gif
git add navigation.gif
git commit -m "docs: 加入 Nav2 导航过程动图"
git push
```

② 编辑 `README.md`，把第 6 行的占位文字：

```markdown
> 📹 动图占位：录一段「slam_toolbox 建图 + Nav2 自主导航」的 10 秒 GIF 放这里。
```

换成：

```markdown
![Nav2 自主导航演示](navigation.gif)
```

> ⚠️ 当前手上这份 GIF 录的是**一次失败**（车原地抖 164 s、恢复行为 14 次、最后 aborted），
> 引用它会变成"负分演示"。建议标注清楚（例如 `![导航失败现场](navigation-fail.gif)`），
> 等成功版录出来再作为主图。

---

## 4. 推送命令速查（复制即用）

```bash
cd "C:/Users/Administrator/Desktop/具身智能学习路线/bridge/ros_study"

git status --short
git add -A --dry-run

git add .gitattributes record_navigation.sh check_sim.sh docs/A5_知识概念全解.md
git commit -m "feat: 新增无头环境导航录制与仿真体检脚本"

git add src/navbot_bringup/launch/nav_bringup.launch.py
git commit -m "fix: 修正 nav_bringup 启动配置"

git push origin main
git ls-remote --heads origin     # 复核：远端 hash 应等于本地 git rev-parse HEAD
```

---

## 5. 常见坑对照表（都会实打实撞上）

| 现象 | 真实原因 | 处理 |
|---|---|---|
| `Permission denied (publickey)` | 用的 key 不是 GitHub 上那把 | `ssh -T git@github.com` 测；`ssh-add -l` 看有没有加载 key |
| `bash: $'\r': command not found`（容器里跑脚本时） | 文件是 CRLF，Linux 把 `\r` 当成命令的一部分 | 已用 `.gitattributes` 锁 LF；急救：`sed -i 's/\r$//' xxx.sh` |
| `error: File xxx is 105.00 MB; exceeds GitHub's file size limit of 100.00 MB` | 单文件超 100 MB | `git rm --cached <file>` + 写进 `.gitignore`；真要存就用 Git LFS |
| `! [rejected] main -> main (non-fast-forward)` | 远端有本地没有的提交（多端/网页上改过） | `git pull --rebase origin main` 后再 push |
| 误 `git add` 了大文件 | 手滑或用了 `-A` | `git restore --staged <file>`（只是撤销暂存，不动文件） |
| 提交信息写错了、还没 push | | `git commit --amend -m "新信息"` |
| 已经 push 了想撤销 | | **别** `reset --hard` + 强推；用 `git revert <hash>` 生成一个反向提交 |
| 中文文件名变成 `\346\226\207...` 转义 | git 默认转义非 ASCII | `git config --global core.quotepath false`（只影响显示，不影响提交） |
| `git status` 一直显示"未跟踪"的生成物 | `.gitignore` 规则不匹配 | 规则带斜杠会被锚定：`maps/*.pgm` ≠ `**/maps/*.pgm` |

---

## 6. 工程映射（面试/复习时怎么讲这段）

- **能推上去 ≠ 推得干净**：`add -A` 一把梭在个人仓库只是脏历史，在团队仓库就是把数据/密钥推进主干。
  先 `--dry-run` 的肌肉记忆比任何 `.gitignore` 都可靠。
- **换行符是跨平台项目的隐形地雷**：Windows 写、Linux 跑的项目，`.gitattributes` 是"一次投入、永久免疫"。
- **git 不是网盘**：`.bak`、构建产物、大二进制不进仓库；`build/ install/ log/` 已在 `.gitignore` 里，
  这也是 ROS 项目的通行做法。
- **提交粒度**：一个提交只讲一件事（脚本链 / 启动配置分开），`git log` 才能当变更日志读。
