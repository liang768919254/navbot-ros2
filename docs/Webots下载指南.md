# Webots R2025a 下载指南（Windows 本机）

最新版是 **R2025a**，官方安装包全部托管在 GitHub Releases，官网本身只是个跳转页。

## 文件清单

| 平台 | 文件 | 大小 | 直链 |
|---|---|---|---|
| **Windows** | `webots-R2025a_setup.exe` | 250.7 MB | `https://github.com/cyberbotics/webots/releases/download/R2025a/webots-R2025a_setup.exe` |
| Ubuntu | `webots_2025a_amd64.deb` | — | `https://github.com/cyberbotics/webots/releases/download/R2025a/webots_2025a_amd64.deb` |
| Ubuntu 归档 | `webots-R2025a-x86-64.tar.bz2` | 143.45 MB | `https://github.com/cyberbotics/webots/releases/download/R2025a/webots-R2025a-x86-64.tar.bz2` |
| macOS | `webots-R2025a.dmg` | — | `https://github.com/cyberbotics/webots/releases/download/R2025a/webots-R2025a.dmg` |

**不需要注册登录**，官网点下载就是跳到上面这些 GitHub 链接。

## 方案一：浏览器 + 代理前缀（最省事）

实测 `gh-proxy.com` 和 `ghproxy.net` 都能下，把原始链接前面拼一段代理域名即可：

```
https://gh-proxy.com/https://github.com/cyberbotics/webots/releases/download/R2025a/webots-R2025a_setup.exe
```

浏览器里粘这个地址，下载器能接管，还能看到进度。

## 方案二：命令行 + 多线程（实测最稳）

单线程慢的话用 `aria2` 分片，能把速度拉起来：

```powershell
# Windows 上先装 aria2（用 winget）
winget install aria2.aria2

# 16 线程下载，断了可以 -c 续传
aria2c -x 16 -s 16 -k 1M -c `
  -o webots-R2025a_setup.exe `
  "https://gh-proxy.com/https://github.com/cyberbotics/webots/releases/download/R2025a/webots-R2025a_setup.exe"
```

`-x 16` 是连接数，`-k 1M` 是分片大小，`-c` 断点续传。中断了重跑同一条命令即可接着下。

## 方案三：下 143 MB 的归档包（省一半流量）

`.tar.bz2` 比 `.exe` 小 100 多 MB。解压后直接跑里面的 `webots` 目录：

```
webots-R2025a-x86-64/webots.exe
```

省流量，代价是不写注册表、菜单栏里没有快捷方式。要图形界面快捷方式的话自己建一个指向 `webots.exe` 的快捷方式。

## 方案四：官网的 Snap / 包管理器（Linux 侧才用得上）

官网提供 Snap：https://snapcraft.io/webots

```bash
sudo snap install webots
```

Snap 会走 Ubuntu 的镜像，比下deb 稳。但 Windows 上用不了这条路。

## 装完自检

```powershell
# 版本号
& "C:\Program Files\Webots\msys64/mingw64/bin/webots.exe" --version

# 或者直接启动，看 GUI 出来没有
& "C:\Program Files\Webots\msys64/mingw64/bin/webots.exe"
```

GUI 能打开、`--version` 打印 R2025a，就装好了。

## 如果要接ROS 2Humble

Webots 自带 ROS 接口，但它**不是 ROS 2 的一等公民**：

- Webots R2025a 官方文档的 ROS 接口主要面向 **ROS 1 Noetic**
- 想在 ROS 2 Humble 里用，需要自己写节点订阅 Webots 的topic

这个项目现在的方案是 **Gazebo Classic 11 + Nav2**，跟 Webots 是两条独立的路。Webots 更适合做机构仿真（多刚体、机械臂、接触丰富的场景），Nav2 生态还是 Gazebo 更全。

## 一句话总结

优先试方案一的浏览器 + 代理前缀；速度还不行就上方案二的 `aria2c -x 16`。250 MB 按 1 MB/s 算 4 分钟能下完。
