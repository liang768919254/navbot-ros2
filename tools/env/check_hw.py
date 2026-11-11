#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""navbot 硬件信息勘察（只读，不改任何配置）

在小车主控机上跑一遍，把这台车「有什么、能不能上路」一次看清：
板卡型号 / CPU / 内存 / 存储 / 温度 / 电源 / USB 外设 / 串口 / 总线节点 /
网络接口 / WiFi 详情 / ROS 2 环境，最后给一份「上路就绪」判据清单。

设计约束
--------
* 全程只读：不写 /etc、不装包、不动网卡、不 sudo。
* 零第三方依赖：只用标准库，Python 3.8+ 可用（Ubuntu 22.04 的 3.10 直接跑）。
* 命令全部带超时：`iw`/`nmcli`/`ros2` 卡住不会拖死脚本。
* 多路降级：本机没有 `iw`、NetworkManager 连不上时自动退到 sysfs 和
  `/proc/net/wireless`，不让「缺一个工具」变成「什么都拿不到」。

用法
----
    python3 check_hw.py                 # 终端报告
    python3 check_hw.py --quiet         # 只列异常项，快速体检
    python3 check_hw.py --json hw.json  # 报告 + 落一份机器可读 JSON
    python3 check_hw.py --json -        # 只吐 JSON（给上层程序吃）
    python3 check_hw.py --scan          # 追加周边 WiFi 扫描（需权限）
    python3 check_hw.py --link-test     # 追加 ping 链路质量测试
    python3 check_hw.py --wifi-module   # 探测底盘上的 ESP8266（自动找串口）
    python3 check_hw.py --wifi-module /dev/ttyUSB1   # 只测指定端口
    python3 check_hw.py --strict        # 有「上路就绪」项不达标则退出码 1

退出码
------
    0  正常（或 --strict 下全部达标）
    1  --strict 且有判据未达标
    2  脚本自身用错参数
"""

from __future__ import annotations

import argparse
import glob
import json
import os
import platform
import re
import socket
import statistics
import subprocess
import sys
import time
import unicodedata
from datetime import datetime
from typing import Any, Dict, List, Optional, Tuple

VERSION = "1.0.0"

# ---------------------------------------------------------------- 已知设备表
# USB VID:PID → (短名, 在小车项目里的角色)
# 这张表是给「插上就能认」用的：RPLIDAR 和 STM32 的转接芯片高度集中在这几个。
KNOWN_USB: Dict[Tuple[str, str], Tuple[str, str]] = {
    ("1a86", "7523"): ("CH340", "USB-TTL，STM32/ESP 常用"),
    ("1a86", "5523"): ("CH341", "USB-TTL / 并口，老款"),
    ("1a86", "55d4"): ("CH9102", "USB-TTL，CH340 的替代料"),
    ("10c4", "ea60"): ("CP210x", "RPLIDAR A1 转接板常用"),
    ("10c4", "ea70"): ("CP2105", "双串口 USB-TTL"),
    ("0403", "6001"): ("FT232", "USB-TTL，工控板常见"),
    ("0403", "6015"): ("FT231X", "USB-TTL"),
    ("0483", "5740"): ("STM32-VCP", "STM32 原生 USB 虚拟串口"),
    ("0483", "df11"): ("STM32-DFU", "STM32 下载模式（BOOT0 拉高时出现）"),
    ("2341", "0043"): ("Arduino Uno", "Arduino 串口"),
    ("1a86", "7522"): ("CH342", "双串口 USB-TTL"),
    ("8087", "0a2b"): ("Intel BT", "板载蓝牙"),
    ("8087", "0033"): ("Intel BT", "板载蓝牙 AX201/AX211"),
    ("0bda", "8153"): ("RTL8153", "USB 千兆网卡"),
    ("2e8a", "000a"): ("RP2040", "树莓派 Pico / 自研板"),
}

# 串口设备识别：路径前缀 → 说明
TTY_HINTS = [
    ("/dev/ttyUSB", "USB 转串口（CH340/CP210x/FTDI）"),
    ("/dev/ttyACM", "CDC-ACM（STM32 虚拟串口 / Arduino）"),
    ("/dev/ttyAMA", "SoC 原生 UART（树莓派 GPIO14/15）"),
    ("/dev/serial0", "树莓派 GPIO 串口别名（指向 ttyAMA0）"),
    ("/dev/ttyS", "板载串口（需确认是否启用 / 有无被 getty 占用）"),
]

SIGNAL_GRADES = [
    (-50, "极佳", "遥控/图传都稳"),
    (-60, "良好", "可用"),
    (-70, "一般", "大流量会掉"),
    (-80, "较差", "遥控有风险"),
    (-999, "很差", "别上路"),
]


# ------------------------------------------------------------------ 基础工具
def run(cmd: List[str], timeout: float = 3.0, merge_stderr: bool = False) -> Tuple[int, str]:
    """安全执行外部命令：永不抛异常，超时/不存在都返回 (rc, '')。

    统一 LC_ALL=C，避免本地化输出把数字格式和小数点弄成逗号。
    """
    env = dict(os.environ)
    env["LC_ALL"] = "C"
    env["LANG"] = "C"
    try:
        p = subprocess.run(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT if merge_stderr else subprocess.DEVNULL,
            timeout=timeout,
            env=env,
            text=True,
            errors="replace",
        )
        return p.returncode, (p.stdout or "").strip()
    except (FileNotFoundError, PermissionError, subprocess.TimeoutExpired, OSError):
        return 127, ""


def has(cmd: str) -> bool:
    from shutil import which

    return which(cmd) is not None


def rd(path: str) -> Optional[str]:
    """读文本文件，失败返回 None。自动去掉 NUL（device-tree 文件尾部带 \\x00）。"""
    try:
        with open(path, "r", encoding="utf-8", errors="replace") as f:
            return f.read().strip().strip("\x00").strip()
    except (OSError, IOError):
        return None


def rdi(path: str) -> Optional[int]:
    v = rd(path)
    if v is None:
        return None
    try:
        return int(re.sub(r"[^\d-]", "", v.split("\n")[0]) or "0")
    except ValueError:
        return None


def human_bytes(n: Optional[float]) -> str:
    if n is None:
        return "?"
    n = float(n)
    for unit in ("B", "KB", "MB", "GB", "TB"):
        if abs(n) < 1024.0:
            return f"{n:.0f} {unit}" if unit == "B" else f"{n:.1f} {unit}"
        n /= 1024.0
    return f"{n:.1f} PB"


def dwidth(s: str) -> int:
    """终端显示宽度：东亚宽字符算 2 列，否则表格必歪。"""
    return sum(2 if unicodedata.east_asian_width(ch) in ("W", "F") else 1 for ch in s)


def _fmt_list(items: List[str], max_show: int = 4, unit: str = "个") -> str:
    """设备节点列表：少就全列，多就报个数再列前几个——避免 i2c-0..14 刷屏折行。"""
    if not items:
        return "无"
    if len(items) <= max_show:
        return "、".join(items)
    return f"{len(items)} {unit}（{'、'.join(items[:max_show])} …）"


def pad(s: str, width: int) -> str:
    return s + " " * max(0, width - dwidth(s))


# ------------------------------------------------------------------ 采集：板卡
def probe_board() -> Dict[str, Any]:
    """板卡/机型。ARM 单板机读 device-tree，x86 读 DMI。"""
    d: Dict[str, Any] = {"arch": platform.machine()}

    dt_model = rd("/proc/device-tree/model") or rd("/sys/firmware/devicetree/base/model")
    if dt_model:
        d["model"] = dt_model
        d["source"] = "device-tree"
        d["kind"] = "ARM 单板机"
        # 树莓派顺手读一下内存总量，和型号对得上很有用
        if "raspberry pi" in dt_model.lower():
            d["kind"] = "树莓派"
    else:
        vendor = rd("/sys/class/dmi/id/sys_vendor")
        product = rd("/sys/class/dmi/id/product_name")
        board = rd("/sys/class/dmi/id/board_name")
        if vendor or product:
            d["model"] = " ".join(x for x in (vendor, product) if x) or "未知"
            d["board"] = board
            d["source"] = "DMI"
            d["kind"] = "x86 主机/笔记本"
        else:
            d["model"] = "未知"
            d["source"] = "无（容器或被裁剪的内核常见）"
            d["kind"] = "未知"

    # ARM 板上 /proc/cpuinfo 有 Hardware 字段（如 BCM2835）
    hw = None
    cpuinfo = rd("/proc/cpuinfo") or ""
    m = re.search(r"^(?:Hardware|Model)\s*:\s*(.+)$", cpuinfo, re.MULTILINE)
    if m:
        hw = m.group(1).strip()
    d["cpuinfo_model"] = hw
    return d


# ------------------------------------------------------------------ 采集：系统
def probe_system() -> Dict[str, Any]:
    d: Dict[str, Any] = {}
    osr = rd("/etc/os-release") or ""
    kv = dict(re.findall(r'^(\w+)="?([^"\n]*)"?$', osr, re.MULTILINE))
    d["distro"] = kv.get("PRETTY_NAME") or kv.get("NAME") or platform.platform()
    d["kernel"] = platform.release()
    d["hostname"] = socket.gethostname()
    d["python"] = platform.python_version()

    up = rd("/proc/uptime")
    if up:
        secs = float(up.split()[0])
        d["uptime_sec"] = int(secs)
        d["uptime_human"] = str(up_time(secs))
        d["boot_time"] = datetime.fromtimestamp(time.time() - secs).strftime("%Y-%m-%d %H:%M:%S")

    d["is_root"] = os.geteuid() == 0
    # 容器判定：/.dockerenv 或 cgroup 里带 docker/kubepods
    if os.path.exists("/.dockerenv"):
        d["container"] = "docker"
    else:
        cg = rd("/proc/1/cgroup") or ""
        if "kubepods" in cg:
            d["container"] = "k8s"
        elif "docker" in cg or "containerd" in cg:
            d["container"] = "docker"
        elif "lxc" in cg:
            d["container"] = "lxc"
    return d


def up_time(secs: float) -> str:
    s = int(secs)
    d, s = divmod(s, 86400)
    h, s = divmod(s, 3600)
    m, s = divmod(s, 60)
    parts = []
    if d:
        parts.append(f"{d} 天")
    if h:
        parts.append(f"{h} 小时")
    if m:
        parts.append(f"{m} 分")
    if not parts:
        parts.append(f"{s} 秒")
    return " ".join(parts)


# --------------------------------------------------------------- 采集：CPU/内存
def probe_cpu() -> Dict[str, Any]:
    d: Dict[str, Any] = {}
    cpuinfo = rd("/proc/cpuinfo") or ""
    m = re.search(r"^model name\s*:\s*(.+)$", cpuinfo, re.MULTILINE)
    if not m:
        m = re.search(r"^(?:Model|Hardware)\s*:\s*(.+)$", cpuinfo, re.MULTILINE)
    d["model"] = m.group(1).strip() if m else platform.processor() or "未知"

    # 物理核 vs 逻辑线程：/proc/cpuinfo 的 processor 行数是线程数，
    # 直接把它叫「核」会虚报一倍（i9-13900H 是 14 核 20 线程）
    threads = len(re.findall(r"^processor\s*:", cpuinfo, re.MULTILINE))
    mc = re.search(r"^cpu cores\s*:\s*(\d+)", cpuinfo, re.MULTILINE)
    d["threads"] = threads or os.cpu_count() or 0
    d["cores"] = int(mc.group(1)) if mc else d["threads"]

    # 当前频率（kHz → MHz）；ARM 上常见 cpufreq 缺失
    freqs = []
    for p in glob.glob("/sys/devices/system/cpu/cpu[0-9]*/cpufreq/scaling_cur_freq"):
        v = rdi(p)
        if v:
            freqs.append(v / 1000.0)
    if freqs:
        d["freq_mhz"] = round(statistics.mean(freqs))
    else:
        _, out = run(["vcgencmd", "measure_clock", "arm"])
        mm = re.search(r"=(\d+)", out or "")
        if mm:
            d["freq_mhz"] = round(int(mm.group(1)) / 1_000_000)

    # 负载
    load = rd("/proc/loadavg")
    if load:
        parts = load.split()
        d["load"] = [float(parts[0]), float(parts[1]), float(parts[2])]
        d["load_ratio"] = round(float(parts[0]) / max(1, d["threads"]), 2)
    return d


def probe_memory() -> Dict[str, Any]:
    d: Dict[str, Any] = {}
    mem = rd("/proc/meminfo") or ""
    kv = {k: int(v) * 1024 for k, v in re.findall(r"^(\w+):\s*(\d+) kB$", mem, re.MULTILINE)}
    total = kv.get("MemTotal")
    avail = kv.get("MemAvailable", kv.get("MemFree"))
    d["total"] = human_bytes(total)
    d["total_bytes"] = total
    if total and avail:
        d["available"] = human_bytes(avail)
        d["used_pct"] = round((total - avail) / total * 100, 1)
    swap_total = kv.get("SwapTotal") or 0
    d["swap"] = human_bytes(swap_total) if swap_total else "无"
    return d


# ------------------------------------------------------------------ 采集：温度
def probe_temp() -> Dict[str, Any]:
    """优先 CPU 封装温度；顺带抓 WiFi 芯片温度（板载无线贴题）。"""
    d: Dict[str, Any] = {"zones": []}
    cpu_val: Optional[float] = None
    wifi_val: Optional[float] = None
    cpu_keys = ("x86_pkg_temp", "cpu_thermal", "cpu-thermal", "soc_thermal", "soc-thermal", "acpitz")

    for zone in sorted(glob.glob("/sys/class/thermal/thermal_zone*")):
        t = rdi(os.path.join(zone, "temp"))
        typ = rd(os.path.join(zone, "type")) or "?"
        if t is None:
            continue
        c = t / 1000.0
        if c <= 0 or c > 200:
            continue
        d["zones"].append({"type": typ, "c": round(c, 1)})
        low = typ.lower()
        if "wifi" in low or "iwl" in low:
            if wifi_val is None or c > wifi_val:
                wifi_val = c
        elif any(k in low for k in cpu_keys):
            if cpu_val is None or c > cpu_val:
                cpu_val = c

    # 树莓派没有 thermal_zone 的型号走 vcgencmd
    if cpu_val is None:
        _, out = run(["vcgencmd", "measure_temp"])
        m = re.search(r"temp=([\d.]+)", out or "")
        if m:
            cpu_val = float(m.group(1))

    if cpu_val is not None:
        d["cpu_c"] = round(cpu_val, 1)
        d["cpu_state"] = (
            "正常" if cpu_val < 70 else "偏高" if cpu_val < 85 else "过热降频风险"
        )
    if wifi_val is not None:
        d["wifi_c"] = round(wifi_val, 1)

    # lm-sensors 兜底（只取第一条温度读数，不刷屏）
    if "cpu_c" not in d and has("sensors"):
        _, out = run(["sensors"], timeout=4)
        m = re.search(r":\s+\+?([\d.]+)°C", out or "")
        if m:
            d["cpu_c"] = float(m.group(1))
            d["cpu_state"] = "正常" if float(m.group(1)) < 70 else "偏高"
    return d


# ------------------------------------------------------------------ 采集：电源
def probe_power() -> Dict[str, Any]:
    d: Dict[str, Any] = {"supplies": []}
    for sup in sorted(glob.glob("/sys/class/power_supply/*")):
        name = os.path.basename(sup)
        typ = rd(os.path.join(sup, "type")) or "?"
        item: Dict[str, Any] = {"name": name, "type": typ}
        cap = rdi(os.path.join(sup, "capacity"))
        if cap is not None:
            item["capacity_pct"] = cap
        status = rd(os.path.join(sup, "status"))
        if status and status != "Unknown":
            item["status"] = status
        vnow = rdi(os.path.join(sup, "voltage_now"))
        if vnow:
            item["voltage_v"] = round(vnow / 1e6, 2)
        d["supplies"].append(item)

    batteries = [s for s in d["supplies"] if s["type"] == "Battery"]
    d["has_battery"] = bool(batteries)
    if batteries:
        d["battery_pct"] = batteries[0].get("capacity_pct")
        d["battery_status"] = batteries[0].get("status", "?")

    # 树莓派供电电压告警（欠压是「跑着跑着重启」的元凶）
    _, out = run(["vcgencmd", "get_throttled"])
    m = re.search(r"throttled=(0x[0-9a-fA-F]+)", out or "")
    if m:
        flags = int(m.group(1), 16)
        d["rpi_throttled"] = m.group(1)
        d["rpi_undervoltage"] = bool(flags & 0x1) or bool(flags & 0x10000)
    return d


# ------------------------------------------------------------------ 采集：存储
def probe_storage() -> Dict[str, Any]:
    d: Dict[str, Any] = {"disks": []}
    if has("lsblk"):
        rc, out = run(
            ["lsblk", "-J", "-b", "-o", "NAME,SIZE,TYPE,MOUNTPOINT,FSTYPE,MODEL,TRAN,ROTA"],
            timeout=5,
        )
        if rc == 0 and out:
            try:
                data = json.loads(out)
                for dev in data.get("blockdevices", []):
                    # loop 设备是 snap 的 squashfs 挂载，几十个，纯噪音
                    if dev.get("type") in ("loop", "rom"):
                        continue
                    d["disks"].append(_shrink_blk(dev))
                return d
            except json.JSONDecodeError:
                pass
    # 降级：df
    rc, out = run(["df", "-PB1"], timeout=4)
    if rc == 0:
        for line in out.splitlines()[1:]:
            f = line.split()
            if len(f) >= 6 and f[0].startswith("/dev/"):
                d["disks"].append(
                    {
                        "name": f[0],
                        "size": int(f[1]),
                        "mounted": f[5],
                        "used_pct": int(f[4].rstrip("%")),
                    }
                )
    return d


def _shrink_blk(node: Dict[str, Any]) -> Dict[str, Any]:
    """lsblk 树瘦身：只留关心的字段，递归 children。

    MODEL 字段被 lsblk 用空格补到固定宽度，必须压掉多余空白，否则渲染时被折行。
    """
    out: Dict[str, Any] = {
        "name": node.get("name"),
        "size": node.get("size"),
        "type": node.get("type"),
    }
    for k in ("mountpoint", "fstype", "tran"):
        if node.get(k):
            out[k] = node[k]
    if node.get("model"):
        out["model"] = " ".join(str(node["model"]).split())
    if node.get("rota") in (True, "1", 1):
        out["media"] = "机械"
    elif node.get("type") == "disk":
        out["media"] = "SSD/eMMC"
    kids = node.get("children") or []
    if kids:
        out["children"] = [_shrink_blk(k) for k in kids]
    return out


# ------------------------------------------------------------------ 采集：USB
def probe_usb() -> Dict[str, Any]:
    """遍历 sysfs，比解析 lsusb 稳（lsusb 可能没装）。"""
    d: Dict[str, Any] = {"devices": []}
    for vpath in sorted(glob.glob("/sys/bus/usb/devices/*/idVendor")):
        base = os.path.dirname(vpath)
        vid = (rd(vpath) or "").lower()
        pid = (rd(os.path.join(base, "idProduct")) or "").lower()
        if not vid or not pid:
            continue
        if vid == "1d6b":  # root hub，噪音
            continue
        item: Dict[str, Any] = {
            "vid_pid": f"{vid}:{pid}",
            "product": rd(os.path.join(base, "product")) or "",
            "manufacturer": rd(os.path.join(base, "manufacturer")) or "",
        }
        speed = rd(os.path.join(base, "speed"))
        if speed:
            item["speed"] = f"{speed} Mbps"
        serial = rd(os.path.join(base, "serial"))
        if serial:
            item["serial"] = serial
        if (vid, pid) in KNOWN_USB:
            name, role = KNOWN_USB[(vid, pid)]
            item["known"] = name
            item["role"] = role
        d["devices"].append(item)
    d["count"] = len(d["devices"])
    return d


# ------------------------------------------------------------------ 采集：串口
def probe_serial() -> Dict[str, Any]:
    """串口是底盘和雷达的命脉，重点看：有没有、是什么芯片、当前用户能不能开。"""
    d: Dict[str, Any] = {"ports": []}
    # ttyUSB/ttyACM 是 USB 转串口；ttyAMA/serial0 是树莓派 GPIO 串口（接 STM32 常走这条）；
    # ttyS0 是 x86 工控板板载串口
    for pat in ("/dev/ttyUSB*", "/dev/ttyACM*", "/dev/ttyAMA*", "/dev/serial0", "/dev/ttyS0"):
        for dev in sorted(glob.glob(pat)):
            item: Dict[str, Any] = {"dev": dev}
            hint = next((h for p, h in TTY_HINTS if dev.startswith(p)), "")
            if hint:
                item["kind"] = hint

            # 权限：能不能读写（ROS 串口桥最常见的坑就是没进 dialout 组）
            readable = os.access(dev, os.R_OK)
            writable = os.access(dev, os.W_OK)
            item["rw"] = "可读写" if (readable and writable) else (
                "只读" if readable else "无权限"
            )

            # udev 属性：拿芯片型号、序列号、稳定路径
            got_ids = False
            if has("udevadm"):
                rc, out = run(["udevadm", "info", "-q", "property", "-n", dev], timeout=3)
                if rc == 0:
                    props = dict(
                        line.split("=", 1) for line in out.splitlines() if "=" in line
                    )
                    vid = (props.get("ID_VENDOR_ID") or "").lower()
                    pid = (props.get("ID_MODEL_ID") or "").lower()
                    if vid and pid:
                        item["vid_pid"] = f"{vid}:{pid}"
                        got_ids = True
                        if (vid, pid) in KNOWN_USB:
                            item["chip"] = KNOWN_USB[(vid, pid)][0]
                    if props.get("ID_SERIAL_SHORT"):
                        item["serial"] = props["ID_SERIAL_SHORT"]
                    if props.get("ID_PATH"):
                        item["id_path"] = props["ID_PATH"]

            # udevadm 不可用 / 容器里没有 udev 数据库时，从 sysfs 反查
            if not got_ids:
                vid, pid = _sysfs_usb_ids(os.path.basename(dev))
                if vid and pid:
                    item["vid_pid"] = f"{vid}:{pid}"
                    if (vid, pid) in KNOWN_USB:
                        item["chip"] = KNOWN_USB[(vid, pid)][0]
            d["ports"].append(item)

    # by-id 稳定软链（推荐在 launch 里用它，别写 /dev/ttyUSB0）
    byid = []
    for p in sorted(glob.glob("/dev/serial/by-id/*")):
        try:
            byid.append({"link": p, "target": os.path.realpath(p)})
        except OSError:
            pass
    d["by_id"] = byid

    # 当前用户组：dialout 决定能不能不 sudo 开串口
    # 用 sudo 跑时 os.getgroups() 拿到的是 root 的组，会误判成「已加入」，
    # 所以优先按 SUDO_USER 反查真实登录用户的组。
    try:
        import grp

        probe_user = os.environ.get("SUDO_USER") or os.environ.get("USER")
        groups: List[str] = []
        if probe_user:
            rc, out = run(["id", "-nG", probe_user], timeout=2)
            if rc == 0 and out:
                groups = out.split()
        if not groups:
            groups = [grp.getgrgid(g).gr_name for g in os.getgroups()]
    except Exception:
        groups = []
        probe_user = None
    if probe_user and probe_user != os.environ.get("USER"):
        d["group_user"] = probe_user
    d["in_dialout"] = "dialout" in groups
    d["groups"] = groups
    d["count"] = len(d["ports"])
    return d


# ------------------------------------------------------------------ 采集：总线
def _sysfs_usb_ids(tty_name: str) -> Tuple[Optional[str], Optional[str]]:
    """从 sysfs 反查某个 tty 设备挂在哪颗 USB 芯片上，返回 (vid, pid)。"""
    if not tty_name:
        return None, None
    for p in glob.glob(f"/sys/bus/usb/devices/*/{tty_name}"):
        base = os.path.dirname(p)
        vid = rd(os.path.join(base, "idVendor"))
        pid = rd(os.path.join(base, "idProduct"))
        if vid and pid:
            return vid.lower(), pid.lower()
    return None, None


def probe_buses() -> Dict[str, Any]:
    d: Dict[str, Any] = {}
    d["i2c"] = sorted(glob.glob("/dev/i2c-*"))
    d["spi"] = sorted(glob.glob("/dev/spidev*"))
    d["video"] = sorted(glob.glob("/dev/video*"))
    d["gpio"] = [p for p in ("/dev/gpiomem", "/dev/gpiochip0") if os.path.exists(p)]
    d["pwm"] = sorted(glob.glob("/sys/class/pwm/pwmchip*"))
    if d["video"]:
        names = []
        if has("v4l2-ctl"):
            rc, out = run(["v4l2-ctl", "--list-devices"], timeout=4)
            if rc == 0:
                # 输出形如 "USB2.0 FHD UVC WebCam: USB2.0 FHD UVC WebCam (usb-...):"
                # 只取第一个冒号前的设备名，后面那截是重复信息，白占宽度
                names = [
                    line.split(":")[0].strip()
                    for line in out.splitlines()
                    if line.strip() and not line.startswith("\t")
                ]
        d["video_names"] = names
    return d


# ------------------------------------------------------------------ 采集：网络
def probe_net() -> Dict[str, Any]:
    d: Dict[str, Any] = {"interfaces": [], "default_route": None}

    rc, out = run(["ip", "-j", "addr", "show"], timeout=3)
    raw = None
    if rc == 0 and out:
        try:
            raw = json.loads(out)
        except json.JSONDecodeError:
            raw = None

    if raw is None:
        raw = _fallback_ip_addr()

    for iface in raw:
        name = iface.get("ifname")
        if not name:
            continue
        sysdir = f"/sys/class/net/{name}"
        item: Dict[str, Any] = {
            "name": name,
            "mac": iface.get("address") or rd(os.path.join(sysdir, "address")),
            "state": iface.get("operstate") or rd(os.path.join(sysdir, "operstate")) or "?",
        }
        if item["mac"] == "00:00:00:00:00:00":
            item["mac"] = None

        # 类型判定：有 wireless/phy80211 → WiFi；名字 lo → 回环
        is_wifi = os.path.isdir(os.path.join(sysdir, "wireless")) or os.path.isdir(
            os.path.join(sysdir, "phy80211")
        )
        if name == "lo":
            item["type"] = "回环"
        elif is_wifi:
            item["type"] = "WiFi"
        elif os.path.isdir(os.path.join(sysdir, "device")):
            item["type"] = "有线"
        else:
            item["type"] = "虚拟"
        item["is_wifi"] = is_wifi

        addrs = []
        for a in iface.get("addr_info", []) or []:
            if a.get("family") == "inet" and a.get("local"):
                addrs.append(f"{a['local']}/{a.get('prefixlen', '?')}")
        if addrs:
            item["ipv4"] = addrs

        if item["type"] == "有线":
            sp = rdi(os.path.join(sysdir, "speed"))
            if sp and sp > 0:
                item["link_speed"] = f"{sp} Mbps"

        drv = rd(os.path.join(sysdir, "device", "uevent"))
        if drv:
            m = re.search(r"^DRIVER=(.+)$", drv, re.MULTILINE)
            if m:
                item["driver"] = m.group(1).strip()
        d["interfaces"].append(item)

    # 默认路由
    rc, out = run(["ip", "-j", "route", "show", "default"], timeout=3)
    if rc == 0 and out:
        try:
            routes = json.loads(out)
            if routes:
                d["default_route"] = {
                    "gateway": routes[0].get("gateway"),
                    "dev": routes[0].get("dev"),
                }
        except json.JSONDecodeError:
            pass
    if d["default_route"] is None:
        rc, out = run(["ip", "route", "show", "default"], timeout=3)
        m = re.search(r"default via ([\d.]+) dev (\S+)", out or "")
        if m:
            d["default_route"] = {"gateway": m.group(1), "dev": m.group(2)}

    # DNS：systemd-resolved 环境下 /etc/resolv.conf 是 stub
    dns = []
    for p in ("/run/systemd/resolve/resolv.conf", "/etc/resolv.conf"):
        txt = rd(p)
        if txt:
            got = re.findall(r"^nameserver\s+(\S+)", txt, re.MULTILINE)
            got = [x for x in got if x != "127.0.0.53"]
            if got:
                dns = got
                break
    d["dns"] = dns
    return d


def _fallback_ip_addr() -> List[Dict[str, Any]]:
    """没 ip -j 时的降级：解析 ip -br addr。"""
    rc, out = run(["ip", "-br", "addr"], timeout=3)
    res = []
    if rc == 0:
        for line in out.splitlines():
            f = line.split()
            if len(f) >= 3:
                res.append(
                    {
                        "ifname": f[0],
                        "operstate": "up" if "UP" in f[1] else "down",
                        "addr_info": [
                            {"family": "inet", "local": a.split("/")[0], "prefixlen": a.split("/")[1]}
                            for a in f[2:]
                            if ":" not in a
                        ],
                    }
                )
    return res


# ------------------------------------------------------------------ 采集：WiFi
def probe_wifi(net: Dict[str, Any]) -> Dict[str, Any]:
    """WiFi 专项：接口能力 + 当前连接 + 信号 + rfkill + 监管域。

    信号值三条来源，按可靠性排序：
      1. iw dev X link      → signal: -48 dBm（最准）
      2. /proc/net/wireless → 第 4 列 level 就是 dBm（无需任何工具，最稳）
      3. nmcli SIGNAL(0-100) → 按 dBm ≈ q/2 - 100 估算，标注为估值
    """
    d: Dict[str, Any] = {"interfaces": [], "available": False}
    wifi_ifaces = [i for i in net.get("interfaces", []) if i.get("is_wifi")]
    if not wifi_ifaces:
        return d

    d["available"] = True
    for iface in wifi_ifaces:
        name = iface["name"]
        w: Dict[str, Any] = {"name": name, "driver": iface.get("driver"), "connected": False}

        # --- 接口能力：type / channel / txpower
        if has("iw"):
            rc, out = run(["iw", "dev", name, "info"], timeout=3)
            if rc == 0:
                m = re.search(r"^\s*type\s+(\S+)", out, re.MULTILINE)
                if m:
                    w["mode"] = m.group(1)
                m = re.search(r"channel\s+(\d+)\s*\((\d+)\s*MHz\)", out)
                if m:
                    w["channel"] = int(m.group(1))
                    w["freq_mhz"] = int(m.group(2))
                m = re.search(r"txpower\s+([\d.]+)\s*dBm", out)
                if m:
                    w["txpower_dbm"] = float(m.group(1))

            phy = rd(f"/sys/class/net/{name}/phy80211/name")
            if phy:
                w["phy"] = phy
                rc, out = run(["iw", "phy", phy, "info"], timeout=4)
                if rc == 0:
                    modes = re.search(
                        r"Supported interface modes:(.*?)(?:\n\s*Band|\n\s*valid|\Z)", out, re.S
                    )
                    if modes:
                        w["supported_modes"] = [
                            x.strip("* \t") for x in modes.group(1).splitlines() if x.strip()
                        ]
                        w["ap_capable"] = any("AP" == m for m in w["supported_modes"])

        # --- 当前连接（iw 优先）
        if has("iw"):
            rc, out = run(["iw", "dev", name, "link"], timeout=3)
            if rc == 0 and "Not connected" not in out and out:
                w["connected"] = True
                m = re.search(r"SSID:\s*(.+)", out)
                if m:
                    w["ssid"] = m.group(1).strip()
                m = re.search(r"Connected to ([\da-f:]{17})", out)
                if m:
                    w["bssid"] = m.group(1)
                m = re.search(r"freq:\s*([\d.]+)", out)
                if m:
                    f = float(m.group(1))
                    w["freq_mhz"] = int(f)
                    w["band"] = "5 GHz" if f > 4000 else "2.4 GHz"
                m = re.search(r"signal:\s*(-?\d+)\s*dBm", out)
                if m:
                    w["signal_dbm"] = int(m.group(1))
                m = re.search(r"tx bitrate:\s*([\d.]+)\s*MB/s", out)
                if m:
                    w["tx_rate_mbps"] = float(m.group(1)) * 8

        # --- 信号兜底：/proc/net/wireless（不依赖任何工具）
        if "signal_dbm" not in w:
            level, qual = _proc_wireless(name)
            if level is not None:
                w["signal_dbm"] = level
                w["signal_source"] = "/proc/net/wireless"
            if qual is not None:
                w["quality"] = qual

        # --- NetworkManager 兜底（拿 SSID / 速率 / 安全类型）
        if not w.get("ssid") and has("nmcli"):
            rc, out = run(
                ["nmcli", "-t", "-f", "GENERAL.CONNECTION", "dev", "show", name], timeout=4
            )
            if rc == 0 and ":" in out:
                conn = out.split(":", 1)[1].strip()
                if conn and conn != "--":
                    w["ssid"] = conn
                    w["connected"] = True
            rc, out = run(
                ["nmcli", "-t", "-f", "IN-USE,SSID,SIGNAL,RATE,SECURITY", "dev", "wifi", "list"],
                timeout=5,
            )
            if rc == 0:
                for line in out.splitlines():
                    if line.startswith("*"):
                        parts = _split_nmcli(line)
                        if len(parts) >= 5:
                            w.setdefault("ssid", parts[1])
                            if "signal_dbm" not in w:
                                try:
                                    q = int(parts[2])
                                    w["signal_dbm"] = round(q / 2 - 100)
                                    w["signal_source"] = "nmcli 估算"
                                except ValueError:
                                    pass
                            w.setdefault("rate", parts[3])
                            w.setdefault("security", parts[4])
                        break

        # --- wpa_supplicant 兜底（树莓派/精简系统上没有 NM 时的主力）
        if not w.get("ssid"):
            for cmd in (["wpa_cli", "-i", name, "status"], ["iwgetid", "-r", name]):
                if not has(cmd[0]):
                    continue
                rc, out = run(cmd, timeout=3)
                if rc != 0 or not out:
                    continue
                m = re.search(r"^ssid=(.+)$", out, re.MULTILINE)
                ssid = m.group(1).strip() if m else out.strip().splitlines()[0].strip()
                if ssid and ssid != "off/any":
                    w["ssid"] = ssid
                    w["ssid_source"] = cmd[0]
                    break

        w["ipv4"] = iface.get("ipv4", [])

        # --- 连接状态推断：iw/nmcli 都不可用时，靠「有信号 + 有 IP」判定
        #     （信号只能来自已关联的接口，/proc/net/wireless 不对未关联接口报 level）
        if not w.get("connected") and w.get("signal_dbm") is not None and w["ipv4"]:
            w["connected"] = True
            w["connected_inferred"] = True

        # --- 信号等级
        if "signal_dbm" in w:
            for thr, grade, note in SIGNAL_GRADES:
                if w["signal_dbm"] >= thr:
                    w["signal_grade"] = grade
                    w["signal_note"] = note
                    break
            w.setdefault("signal_source", "iw")

        # --- 频段兜底
        if "band" not in w and w.get("freq_mhz"):
            w["band"] = "5 GHz" if w["freq_mhz"] > 4000 else "2.4 GHz"

        # --- phy 名：部分驱动没有 phy80211 软链，别显示成 "phy ?"
        if not w.get("phy") and w.get("mode"):
            w.pop("phy", None)

        d["interfaces"].append(w)

    # --- rfkill：WiFi 打不开有一半是这里被 block 了
    d["rfkill"] = _probe_rfkill()
    # --- 监管域：把 5G 信道限制住会莫名连不上 149+ 信道
    if has("iw"):
        rc, out = run(["iw", "reg", "get"], timeout=3)
        m = re.search(r"country\s+(\w+)", out or "")
        if m:
            d["reg_country"] = m.group(1)
    return d


def _split_nmcli(line: str) -> List[str]:
    """nmcli -t 的 ':' 分隔，值里的 ':' 会被转义成 '\\:'。"""
    out, cur, esc = [], "", False
    for ch in line:
        if esc:
            cur += ch
            esc = False
        elif ch == "\\":
            esc = True
        elif ch == ":":
            out.append(cur)
            cur = ""
        else:
            cur += ch
    out.append(cur)
    return out


def _proc_wireless(ifname: str) -> Tuple[Optional[int], Optional[int]]:
    """从 /proc/net/wireless 取 (level_dBm, link_quality)。

    格式：wlo1: 0000  62.  -48.  -256 ...
             status  link  level noise
    注意末尾的点——用 float() 前要剥掉。
    """
    txt = rd("/proc/net/wireless")
    if not txt:
        return None, None
    for line in txt.splitlines():
        if not line.strip().startswith(ifname + ":"):
            continue
        body = line.split(":", 1)[1].strip()
        # 有些内核在 status 和数值之间有额外空白
        nums = re.findall(r"-?\d+\.?", body)
        if len(nums) < 3:
            continue
        try:
            qual = int(float(nums[1]))
            level = int(float(nums[2]))
        except (ValueError, IndexError):
            continue
        # level 偶见 -256（未就绪）或正值
        if level < -200 or level > 0:
            level = None
        return level, qual
    return None, None


def _probe_rfkill() -> Dict[str, Any]:
    res: Dict[str, Any] = {"available": False, "blocked": []}
    if has("rfkill"):
        rc, out = run(["rfkill", "list"], timeout=3)
        if rc == 0 and out:
            res["available"] = True
            cur = {}
            for line in out.splitlines():
                m = re.match(r"^\d+:\s+(\S+):\s+(\S+)", line)
                if m:
                    cur = {"type": m.group(1), "name": m.group(2)}
                ms = re.search(r"Soft blocked:\s+(\w+)", line)
                mh = re.search(r"Hard blocked:\s+(\w+)", line)
                if ms:
                    cur["soft_blocked"] = ms.group(1) == "yes"
                if mh:
                    cur["hard_blocked"] = mh.group(1) == "yes"
                    if cur.get("soft_blocked") or cur.get("hard_blocked"):
                        res["blocked"].append(cur.copy())
            return res
    # 降级：直接读 sysfs
    for p in sorted(glob.glob("/sys/class/rfkill/rfkill*")):
        typ = rd(os.path.join(p, "type"))
        if typ != "wlan":
            continue
        res["available"] = True
        soft = rd(os.path.join(p, "soft"))
        hard = rd(os.path.join(p, "hard"))
        if soft == "1" or hard == "1":
            res["blocked"].append(
                {"type": typ, "name": os.path.basename(p), "soft_blocked": soft == "1",
                 "hard_blocked": hard == "1"}
            )
    return res


def probe_scan() -> Dict[str, Any]:
    """周边 WiFi 扫描。需要 root 或 NetworkManager 权限，失败不算错。"""
    d: Dict[str, Any] = {"networks": [], "error": None}
    if has("nmcli"):
        rc, out = run(
            ["nmcli", "-t", "-f", "IN-USE,SSID,SIGNAL,CHAN,FREQ,SECURITY", "dev", "wifi", "list",
             "--rescan", "yes"],
            timeout=20,
        )
        if rc == 0 and out:
            for line in out.splitlines():
                p = _split_nmcli(line)
                if len(p) >= 6 and p[1]:
                    d["networks"].append(
                        {
                            "in_use": p[0].strip() == "*",
                            "ssid": p[1],
                            "signal_pct": p[2],
                            "channel": p[3],
                            "freq": p[4],
                            "security": p[5],
                        }
                    )
            d["networks"].sort(key=lambda x: -_int(x["signal_pct"]))
            return d
        d["error"] = "nmcli 扫描失败（NetworkManager 未运行或权限不足）"
        return d
    if has("iw"):
        ifaces = [i for i in glob.glob("/sys/class/net/*") if os.path.isdir(os.path.join(i, "wireless"))]
        if ifaces:
            name = os.path.basename(ifaces[0])
            rc, out = run(["iw", "dev", name, "scan"], timeout=20)
            if rc == 0:
                for blk in out.split("BSS ")[1:]:
                    m = re.search(r"SSID:\s*(.+)", blk)
                    s = re.search(r"signal:\s*(-?\d+)", blk)
                    f = re.search(r"freq:\s*(\d+)", blk)
                    if m:
                        d["networks"].append(
                            {
                                "ssid": m.group(1).strip(),
                                "signal_dbm": int(s.group(1)) if s else None,
                                "freq_mhz": int(f.group(1)) if f else None,
                            }
                        )
                return d
            d["error"] = "iw scan 失败（需要 root）"
            return d
    d["error"] = "没有可用的扫描工具"
    return d


def _int(v: Any) -> int:
    try:
        return int(str(v).strip())
    except (ValueError, TypeError):
        return 0


# ------------------------------------------------- 采集：底盘 WiFi 模块（ESP8266）
# ESP8266 AT 固件的标准查询指令。**只发查询，绝不发会改状态的**：
# 不发 AT+RST（重启模块）、AT+CWQAP（断开）、AT+CWJAP=（连 AP）、AT+CWMODE=（改模式）、
# AT+CIPSEND（发数据）。这是「只读」原则在 AT 层上的延续。
ESP_AT_QUERIES = [
    ("firmware", "AT+GMR", 1.2),
    ("mode", "AT+CWMODE?", 0.8),
    ("ap", "AT+CWJAP?", 0.9),
    ("ip", "AT+CIPSTA?", 0.9),
    ("iface", "AT+CIFSR", 0.9),
]

# 串口波特率候选，按可能性排序。ESP8266 的 AT 固件默认 115200，老模块 9600。
# 注意：74880（ESP 的 boot log 波特率）**故意不放进来**——它是非标准值，
# Linux 的 termios 没有 B74880 常量，普通 tcsetattr 根本设不了。
# 那要看 boot log 得用 termios2 + BOTHER ioctl，为它引入 ctypes 不划算；
# 而且 AT 通信从来不跑 74880，放进来只会产生一条误导性的「打不开」。
ESP_BAUDS = (115200, 9600, 57600, 38400, 230400, 460800)

# 静听窗口。太短会漏掉间歇性的数据流，太长则每端口每波特率都要等。
# 1.0 秒能覆盖 50Hz 的遥测帧，也能覆盖 1Hz 的慢速心跳。
ESP_LISTEN_SEC = 1.0


def _open_tty(port: str, baud: int) -> Optional[int]:
    """用 termios 把串口配成 raw 8N1 并返回 fd。

    故意不用 pyserial：这个脚本要求零第三方依赖，插到任何一台车机上都能跑。
    失败时返回 None —— 具体原因用 `_open_fail_reason()` 反查，不要在这里猜。
    """
    try:
        import termios
    except ImportError:  # 非 Unix
        return None
    try:
        fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    except OSError:
        return None
    try:
        attrs = termios.tcgetattr(fd)
        attrs[0] &= ~(termios.IXON | termios.IXOFF | termios.ICRNL | termios.IGNCR | termios.INLCR)
        attrs[1] &= ~termios.OPOST
        attrs[3] &= ~(termios.ECHO | termios.ICANON | termios.ISIG | termios.IEXTEN)
        attrs[2] &= ~(termios.CSIZE | termios.PARENB | termios.CSTOPB | termios.CRTSCTS)
        attrs[2] |= termios.CS8 | termios.CREAD | termios.CLOCAL
        speed = getattr(termios, f"B{baud}", None)
        if speed is None:
            os.close(fd)
            return None
        attrs[4] = attrs[5] = speed
        # cc 数组元素在部分 Python 版本是 bytes、部分是 int，两种都试一遍。
        # 全程 O_NONBLOCK，VMIN/VTIME 不会真的阻塞读，这里只是把状态摆正。
        try:
            attrs[6][termios.VMIN] = 0
            attrs[6][termios.VTIME] = 0
            termios.tcsetattr(fd, termios.TCSANOW, attrs)
        except TypeError:
            attrs[6][termios.VMIN] = b"\x00"
            attrs[6][termios.VTIME] = b"\x00"
            termios.tcsetattr(fd, termios.TCSANOW, attrs)
        termios.tcflush(fd, termios.TCIOFLUSH)
        return fd
    except Exception:  # noqa: BLE001
        try:
            os.close(fd)
        except OSError:
            pass
        return None


def _open_fail_reason(port: str, baud: int) -> str:
    """`_open_tty` 失败后反查原因，别笼统地说「打不开」。

    这一条很关键：真机上 `/dev/ttyS0` 明明 `os.access` 显示可读写，open 却失败
    （主板没有实际 UART 硬件时会 ENODEV）。笼统报「权限或设备不存在」会把
    排查方向带偏——用户会去查 udev 规则，而实际问题在硬件。
    """
    if not os.path.exists(port):
        return "设备节点不存在"
    if not os.access(port, os.R_OK | os.W_OK):
        return "无读写权限（把用户加进 dialout 组）"
    try:
        import termios

        if getattr(termios, f"B{baud}", None) is None:
            return f"Linux termios 不支持 {baud} 这个非标准波特率"
    except ImportError:
        pass
    # 前面三项都过，那就是 open 本身报错——把 errno 原样带出来
    try:
        fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        os.close(fd)
        return "能打开但 termios 配置失败（驱动可能不支持 raw 模式）"
    except OSError as e:
        return f"open 失败：{e.strerror}（errno {e.errno}）"


def _at_query(fd: int, cmd: str, wait: float = 0.8) -> str:
    """发一条 AT 指令并收集响应。收到数据后把等待窗口往后顺延，避免截断。"""
    try:
        os.write(fd, (cmd + "\r\n").encode("ascii", "ignore"))
    except OSError:
        return ""
    deadline = time.time() + wait
    buf = b""
    hard_stop = time.time() + wait + 2.0  # 防止模块一直吐数据把循环拖死
    while time.time() < deadline and time.time() < hard_stop:
        try:
            chunk = os.read(fd, 512)
        except BlockingIOError:
            chunk = b""
        except OSError:
            break
        if chunk:
            buf += chunk
            deadline = time.time() + 0.25
        else:
            time.sleep(0.03)
    return buf.decode("utf-8", "replace")


def _port_busy(port: str) -> Optional[str]:
    """端口被「别的进程」占着没有。串口能被多个进程同时打开，靠 open 失败判断不出来。

    直接扫 /proc/*/fd 找谁把这个 tty 打开着，不解析 fuser/lsof 的输出——
    它们的格式各版本不一致（实测 fuser 打印的是 `    4/path:`，PID 在**路径前面**，
    和文档里常见的 `/path: 1234` 正好相反，按后者写必然解析错）。

    - 比 realpath，软链（/dev/serial/by-id）也能正确匹配
    - 天然跳过自己：每次探测都要开一次 fd，不排除就是「自己把自己判成占用」
    - 读不到别人 fd 目录时静默跳过（权限不足宁可漏报，也不要误报）
    """
    try:
        target = os.path.realpath(port)
    except OSError:
        target = port
    me = os.getpid()
    holders: List[int] = []

    for fd_dir in glob.glob("/proc/[0-9]*/fd"):
        try:
            pid = int(fd_dir.split("/")[2])
        except (IndexError, ValueError):
            continue
        if pid == me:
            continue
        try:
            names = os.listdir(fd_dir)
        except (PermissionError, OSError):
            continue
        for fd in names:
            try:
                link = os.readlink(os.path.join(fd_dir, fd))
            except OSError:
                continue
            if link == port or link == target:
                holders.append(pid)
                break

    if holders:
        return "、".join(f"PID {p}" for p in holders[:3])
    return None


def probe_listen(port: str, baud: int = 115200, seconds: float = 15.0,
                 save_path: Optional[str] = None) -> Dict[str, Any]:
    """在指定波特率上长时间静听，把收到的原始数据 dump 出来。

    为什么需要长窗口：MCU 固件常常只在复位时打一次 banner，或者满足某个条件才发一帧。
    `--baudscan` 每档只给 2 秒，很容易全部错过——2026-10-31 就踩过一次：
    六档里只有一档撞到数据，而那一档还是错的波特率。

    正确姿势：已知波特率（比如固件里写死的 115200）时直接长听，
    期间**按一下板子的复位键**让固件重新说一次话。

    `save_path` 给定时把**完整原始字节**另存一份。终端里只显示开头几十行，
    固件一次打印上百行报告时，靠屏幕看必然漏。
    """
    d: Dict[str, Any] = {"port": port, "baud": baud, "seconds": seconds}
    if not os.path.exists(port):
        d["error"] = "设备节点不存在"
        return d

    fd = _open_tty(port, baud)
    if fd is None:
        d["error"] = _open_fail_reason(port, baud)
        return d
    try:
        t0 = time.time()
        text, raw = _listen(fd, seconds)
        d["seconds"] = round(time.time() - t0, 1)
        d["bytes"] = len(raw)
        d["classify"] = _classify_port_reply(text) if raw else None
        d["lines_total"] = len([ln for ln in text.splitlines() if ln.strip()])
        d["text"] = text[:8000]
        d["hex"] = raw[:96].hex(" ")
        d["rate_bps"] = round(len(raw) / max(0.001, d["seconds"]), 1)

        if save_path:
            try:
                with open(save_path, "wb") as f:
                    f.write(raw)
                d["saved"] = os.path.abspath(save_path)
            except OSError as e:
                d["save_error"] = f"{e.strerror}（{save_path}）"
    finally:
        try:
            os.close(fd)
        except OSError:
            pass
    return d


def probe_baud_scan(port: str, seconds: float = 2.0, bauds: Optional[List[int]] = None) -> Dict[str, Any]:
    """逐个波特率静听，报每个收到多少字节、像什么。

    为什么需要这个：很多 MCU 固件**不是持续输出**的——只在复位时打一行 banner，
    或者事件触发时才吐一帧。这种设备盲试 AT 永远试不出来，但换个思路：
    按正确波特率一定能收到可读文本，按错误波特率只会收到乱码或空。
    所以「扫一遍每个波特率收到什么」能直接反推出真实波特率。

    用法上是配合动作的：**跑扫描的同时按一下板子的复位键**，
    让固件重新打印一次启动日志，扫到正确波特率的那一栏就会冒出可读内容。
    """
    d: Dict[str, Any] = {"port": port, "seconds": seconds, "results": []}
    if not os.path.exists(port):
        d["error"] = "设备节点不存在"
        return d

    for baud in (bauds or ESP_BAUDS):
        fd = _open_tty(port, baud)
        if fd is None:
            d["results"].append({"baud": baud, "error": _open_fail_reason(port, baud)})
            continue
        try:
            text, raw = _listen(fd, seconds)
            d["results"].append(
                {
                    "baud": baud,
                    "bytes": len(raw),
                    "classify": _classify_port_reply(text) if raw else None,
                    "text": text.strip()[:120],
                    "hex": raw[:32].hex(" ") if raw else "",
                }
            )
        finally:
            try:
                os.close(fd)
            except OSError:
                pass

    # 谁收到的最像正常文本，谁就是最可能的波特率
    readable = [r for r in d["results"] if r.get("classify") in ("esp", "navbot_stm32",
                                                                 "stm32_debug", "text")]
    d["likely_baud"] = readable[0]["baud"] if readable else None
    return d


def _parse_esp_responses(raw: Dict[str, str]) -> Dict[str, Any]:
    """把 ESP8266 的 AT 原始响应解析成结构化字段。

    单独抽出来是为了能脱离串口单测——AT 解析写错的表现是「什么都没拿到」，
    和「模块没接」长得一模一样，不脱机测一次根本分不清。

    容错优先：解析不到就留空，绝不猜值。
    """
    d: Dict[str, Any] = {}

    gmr = raw.get("firmware", "")
    m = re.search(r"AT version:([\w.\-]+)", gmr)
    if m:
        d["at_version"] = m.group(1)
    m = re.search(r"SDK version:([\w.\-]+)", gmr)
    if m:
        d["sdk_version"] = m.group(1)
    m = re.search(r"Bin version[^:]*:([\w.\-]+)", gmr)
    if m:
        d["bin_version"] = m.group(1)

    m = re.search(r"\+CWMODE:(\d)", raw.get("mode", ""))
    if m:
        mode = int(m.group(1))
        d["cwm_mode"] = mode
        d["cwm_mode_text"] = {
            1: "Station（连路由器）",
            2: "SoftAP（自己开热点）",
            3: "Station+SoftAP（并存）",
        }.get(mode, f"未知({mode})")

    ap = raw.get("ap", "")
    m = re.search(r'\+CWJAP:"([^"]*)","([^"]*)",(\d+),(-?\d+)', ap)
    if m:
        d["ssid"] = m.group(1)
        d["bssid"] = m.group(2)
        d["channel"] = int(m.group(3))
        d["rssi_dbm"] = int(m.group(4))
        for thr, grade, note in SIGNAL_GRADES:
            if d["rssi_dbm"] >= thr:
                d["signal_grade"] = grade
                d["signal_note"] = note
                break
    elif "No AP" in ap:
        d["ap_state"] = "未连接任何 AP"

    # CIPSTA / CIFSR 两处都能拿 IP，谁有算谁
    for src in ("ip", "iface"):
        txt = raw.get(src, "")
        if "ip" not in d:
            m = re.search(r'STAIP,"([\d.]+)"', txt) or re.search(r'ip:"([\d.]+)"', txt)
            if m and m.group(1) != "0.0.0.0":
                d["ip"] = m.group(1)
        if "gateway" not in d:
            m = re.search(r'gateway:"([\d.]+)"', txt)
            if m:
                d["gateway"] = m.group(1)
        if "mac" not in d:
            m = re.search(r'STAMAC,"([\da-fA-F:]{17})"', txt)
            if m:
                d["mac"] = m.group(1)
    return d


def _listen(fd: int, seconds: float = ESP_LISTEN_SEC) -> Tuple[str, bytes]:
    """只听不发，返回 (解码文本, 原始字节)。

    这是整个探测里信息量最大的动作：MCU 类设备往往周期性主动吐遥测帧，
    而 ESP8266 是纯被动的，不问不说话。所以「静听有没有数据」直接区分了两者。

    同时返回原始字节是有原因的：判断「是噪声还是真数据」要看十六进制。
    连续重复的模式（如 `6b 6b 6b 6b`）几乎肯定是波特率不匹配造成的过采样，
    而真实帧的字节分布有明显结构。
    """
    deadline = time.time() + seconds
    buf = b""
    while time.time() < deadline:
        try:
            chunk = os.read(fd, 512)
        except BlockingIOError:
            chunk = b""
        except OSError:
            break
        if chunk:
            buf += chunk
            if len(buf) > 8192:
                break
        else:
            time.sleep(0.02)
    return buf.decode("utf-8", "replace"), buf


def _classify_port_reply(reply: str) -> Optional[str]:
    """根据对面回/发的内容判断这个串口上挂的是什么东西。

    比「AT 无应答」有用得多——它直接回答「我插的这个口到底是底盘还是 WiFi 模块」。
    """
    if not reply:
        return None
    if "OK" in reply and "AT" in reply:
        return "esp"
    # navbot O 帧：O,seq,enc_l,enc_r,vel_l,vel_r,pwm_l,pwm_r,vbat_mv,tick_ms,temp
    if re.search(r"(?:^|[\r\n])[OV],-?\d+,", reply):
        return "navbot_stm32"
    if re.search(r"(STM32|Alientek|正点原子|stm32)", reply):
        return "stm32_debug"

    # 乱码判定要按**码点**数，不能用 str.isprintable()：
    # decode("utf-8", "replace") 产生的 U+FFFD（�）isprintable() 竟然返回 True，
    # 于是波特率不对时的满屏乱码会被算成「可读文本」，把诊断彻底带偏。
    #
    # 两条判据，任一条成立即为乱码：
    #   ① 出现替换字符 → 原始字节不是合法 UTF-8，硬证据
    #   ② 控制字符（除 \r\n\t）占比过高 → 串口上的文本不会这样
    # 光看 ASCII 比例会把纯中文输出误判成乱码，所以不用它。
    total = len(reply)
    if total >= 4:
        if reply.count("\ufffd") / total > 0.08:
            return "garbled"
        ctrl = sum(1 for ch in reply if ord(ch) < 32 and ch not in "\r\n\t")
        if ctrl / total > 0.2:
            return "garbled"
    return "text"


KIND_LABEL = {
    "esp": "ESP8266（AT 固件正常应答）",
    "navbot_stm32": "STM32 底盘（正在发 O/V 帧）",
    "stm32_debug": "STM32 调试输出",
    "garbled": "乱码（波特率不匹配）",
    "text": "可读文本，但不是 AT 协议",
}


def probe_esp8266(ports: List[str], explicit: Optional[str] = None, scan_ap: bool = False) -> Dict[str, Any]:
    """探测底盘上的 ESP8266：先静听判身份，再用 AT 确认链路，最后逐条查状态。

    能不能探测成功，取决于 ESP 的 UART 在物理上通到哪：
      * 直连板载 USB-TTL  → 本脚本可直接探测
      * 接 STM32 的 USART2 → 必须先由 STM32 固件做 AT 透传，否则探测必然失败

    所以失败时**不能只报「没找到」**：要能说清对面到底是个什么东西。
    静听能认出 STM32 底盘这件事，本身就回答了「我插的这个口对不对」。
    """
    d: Dict[str, Any] = {"found": False, "attempts": [], "queried": False}
    cands = [explicit] if explicit else list(ports)
    if not cands:
        d["error"] = "没有候选串口可探测"
        return d

    d["candidates"] = cands

    for port in cands:
        busy = _port_busy(port)
        if busy:
            d["attempts"].append(
                {"port": port, "baud": None, "reply": "", "ok": False, "busy": busy}
            )
            continue
        for baud in ESP_BAUDS:
            fd = _open_tty(port, baud)
            if fd is None:
                d["attempts"].append(
                    {"port": port, "baud": baud, "reply": "", "ok": False,
                     "note": _open_fail_reason(port, baud)}
                )
                continue
            try:
                # 先只听不说：对面有主动数据流就不可能是「安静的 AT 模块」
                passive, raw = _listen(fd, ESP_LISTEN_SEC)
                att: Dict[str, Any] = {"port": port, "baud": baud, "reply": "", "ok": False}
                if passive:
                    att["passive"] = passive.strip()[:160]
                    # 十六进制要看原始字节，不能拿解码后的文本再 encode 回去
                    att["passive_hex"] = raw[:24].hex(" ")
                    kind = _classify_port_reply(passive)
                    if kind:
                        att["classify"] = kind
                    # 认出是底盘就没必要再发 AT 了，也不用试别的波特率
                    if kind in ("navbot_stm32", "stm32_debug"):
                        d["attempts"].append(att)
                        d["detected"] = kind
                        d["detected_port"] = port
                        d["detected_baud"] = baud
                        break

                hello = _at_query(fd, "AT", 0.6)
                ok = "OK" in hello
                att["reply"] = hello.strip()[:120]
                att["ok"] = ok
                if hello and "classify" not in att:
                    kind = _classify_port_reply(hello)
                    if kind:
                        att["classify"] = kind
                d["attempts"].append(att)
                if not ok:
                    continue

                # 链路通了，开始逐条查
                d.update({"found": True, "port": port, "baud": baud})
                d["queried"] = True
                raw: Dict[str, str] = {}
                for key, cmd, wait in ESP_AT_QUERIES:
                    raw[key] = _at_query(fd, cmd, wait)
                if scan_ap:
                    raw["cwlap"] = _at_query(fd, "AT+CWLAP", 6.0)
                d["raw"] = {k: v.strip()[:400] for k, v in raw.items()}

                # --- 解析交给独立函数，方便脱机单测（见 tools/verify/test_esp_at_probe.py）
                d.update(_parse_esp_responses(raw))
            finally:
                try:
                    os.close(fd)
                except OSError:
                    pass
            if d["found"] or d.get("detected"):
                break
        if d["found"] or d.get("detected"):
            break

    if not d["found"]:
        # 分情形给诊断，别让用户猜。「完全静默」和「有数据但不是 AT」是两回事，
        # 混成一句「AT 无应答」会把排查方向带偏。
        busy = [a for a in d["attempts"] if a.get("busy")]
        opened = [a for a in d["attempts"] if a.get("baud") and not a.get("note")]
        kinds = {a.get("classify") for a in d["attempts"] if a.get("classify")}
        with_data = [a for a in d["attempts"] if a.get("passive") or a.get("reply")]
        silent = [
            a for a in d["attempts"]
            if a.get("baud") and not a.get("note") and not a.get("busy")
            and not a.get("passive") and not a.get("reply")
        ]

        if d.get("detected") == "navbot_stm32":
            d["diagnosis"] = (
                f"{d.get('detected_port')} 上是 STM32 底盘（收到了 O/V 帧遥测），不是 ESP8266。"
                "ESP 挂在 STM32 的 USART2 后面，要读它得先由固件做 AT 透传。"
            )
        elif "stm32_debug" in kinds:
            d["diagnosis"] = (
                "对面在输出 STM32 调试信息，说明这个口是 MCU 的调试串口，不是 ESP8266。"
            )
        elif with_data:
            rates = "、".join(str(x) for x in sorted({a["baud"] for a in with_data}))
            silent_rates = "、".join(str(x) for x in sorted({a["baud"] for a in silent}))
            d["diagnosis"] = (
                f"线路上确实有数据（{rates} 下收到），但没有一句是 AT 应答。"
                + (f"{silent_rates} 下则完全静默。" if silent_rates else "")
                + "两种可能：① 波特率还没试对；② 对面不是 ESP——比如 MCU 在发自定义帧，"
                "当前波特率把它采成了乱码。看上面的十六进制：出现大量重复字节"
                "（如 6b 6b 6b）基本就是波特率不匹配。"
            )
        elif busy and len(busy) >= len(d["attempts"]):
            d["diagnosis"] = "串口被其他进程占用，先停掉 navbot_bridge / 串口工具再试"
        elif opened:
            d["diagnosis"] = (
                "所有波特率下都完全静默——既没有 AT 应答，也没有任何主动数据流。"
                "若 ESP 接在 STM32 的 USART2 后面且固件没做透传，就是这个表现；"
                "也可能是模块没供电、EN 脚没拉高，或 TX 线根本没接。"
            )
        else:
            d["diagnosis"] = "所有候选端口都没打开，往上翻「打不开」那几行看具体原因"
    return d


# ------------------------------------------------------------------ 采集：ROS2
def _home_dirs() -> List[str]:
    """家目录候选，按可靠性排序。

    ⚠️ 不能只用 `os.path.expanduser("~")`：受限环境里 `$HOME` 可能被设成 `/root`，
    而真正的工作空间在 `/home/xxx` 下，于是「明明装了却查不到」。
    最可靠的是**从脚本自身位置反推**——脚本就在 `<home>/<ws>/tools/env/` 里。
    """
    cands: List[str] = []
    # ① 脚本位置上溯三级：<home>/<ws>/tools/env/check_hw.py → <home>
    here = os.path.dirname(os.path.abspath(__file__))
    cands.append(os.path.dirname(os.path.dirname(os.path.dirname(here))))
    # ② 环境变量（可能不可信，作为补充）
    for key in ("HOME", "USERPROFILE"):
        v = os.environ.get(key)
        if v:
            cands.append(v)
    # ③ 真实账号信息
    try:
        import pwd

        cands.append(pwd.getpwuid(os.getuid()).pw_dir)
    except Exception:  # noqa: BLE001
        pass

    seen, out = set(), []
    for c in cands:
        if c and c not in seen and os.path.isdir(c):
            seen.add(c)
            out.append(c)
    return out


def _pkg_elsewhere(pkg: str) -> Optional[str]:
    """当前环境里找不到时，扫一遍家目录下其他工作空间。

    两种安装布局都要认（踩过：只认 merge 布局会把 isolated 布局的包全漏掉）：

        merge-install   <ws>/install/share/<pkg>              （colcon --merge-install）
        isolated        <ws>/install/<pkg>/share/<pkg>        （colcon 默认）

    「装了但没 source」和「真的没装」是两个完全不同的结论，不能让用户自己猜。
    """
    for home in _home_dirs():
        try:
            entries = sorted(os.listdir(home))
        except OSError:
            continue
        for name in entries:
            if name.startswith("."):
                continue
            base = os.path.join(home, name, "install")
            if not os.path.isdir(base):
                continue
            for cand in (
                os.path.join(base, "share", pkg),            # merge
                os.path.join(base, pkg, "share", pkg),       # isolated
            ):
                if os.path.isdir(cand):
                    return cand
    return None


def probe_ros() -> Dict[str, Any]:
    d: Dict[str, Any] = {}
    d["distro"] = os.environ.get("ROS_DISTRO") or "未 source"
    d["rmw"] = os.environ.get("RMW_IMPLEMENTATION") or "默认(fastrtps)"
    d["domain_id"] = os.environ.get("ROS_DOMAIN_ID") or "0"
    d["local_only"] = os.environ.get("ROS_LOCALHOST_ONLY") or "0"

    prefix_path = os.environ.get("AMENT_PREFIX_PATH") or ""
    prefixes = [p for p in prefix_path.split(":") if p]
    d["ament_prefix_count"] = len(prefixes)
    d["ament_prefixes"] = prefixes

    # 关键包是否装在任一 prefix 下（查目录比 ros2 pkg list 快得多，也不怕 daemon 卡住）
    wanted = ["rplidar_ros", "sllidar_ros2", "nav2_bringup", "slam_toolbox", "navbot_bridge"]
    found = {}
    elsewhere = {}
    for pkg in wanted:
        hit = None
        for pre in prefixes:
            if os.path.isdir(os.path.join(pre, "share", pkg)):
                hit = pre
                break
        found[pkg] = hit
        if hit is None:
            alt = _pkg_elsewhere(pkg)
            if alt:
                elsewhere[pkg] = alt
    d["packages"] = found
    d["packages_elsewhere"] = elsewhere

    if not prefixes:
        # 没 source 时兜底看 /opt/ros
        roots = sorted(glob.glob("/opt/ros/*/setup.bash"))
        d["installed_distros"] = [os.path.basename(os.path.dirname(r)) for r in roots]
    return d


# ------------------------------------------------------------------ 采集：链路
def probe_link(gateway: Optional[str] = None) -> Dict[str, Any]:
    """ping 网关与公网，判断这条 WiFi 够不够撑遥控。"""
    d: Dict[str, Any] = {"targets": []}
    if not has("ping"):
        d["error"] = "无 ping 命令"
        return d

    targets = []
    if gateway:
        targets.append(("网关", gateway))
    targets.append(("公网", "223.5.5.5"))

    for label, host in targets:
        rc, out = run(["ping", "-c", "10", "-i", "0.2", "-W", "1", host], timeout=15)
        item: Dict[str, Any] = {"label": label, "host": host}
        m = re.search(r"(\d+) packets transmitted, (\d+) received", out)
        if m:
            sent, recv = int(m.group(1)), int(m.group(2))
            item["loss_pct"] = round((sent - recv) / sent * 100, 1)
        m = re.search(r"rtt min/avg/max/mdev = ([\d.]+)/([\d.]+)/([\d.]+)", out)
        if m:
            item["rtt_min"] = float(m.group(1))
            item["rtt_avg"] = float(m.group(2))
            item["rtt_max"] = float(m.group(3))
        if "loss_pct" in item:
            lp = item["loss_pct"]
            avg = item.get("rtt_avg", 0)
            if lp == 0 and avg < 50:
                item["verdict"] = "可用"
            elif lp <= 5 and avg < 150:
                item["verdict"] = "勉强，遥控建议降速"
            else:
                item["verdict"] = "不可用，先解决链路"
        d["targets"].append(item)
    return d


# ------------------------------------------------------------------ 就绪判据
def readiness(hw: Dict[str, Any]) -> List[Dict[str, Any]]:
    """把「采集结果」变成「能不能上路」的判断。这是脚本存在的真正理由。"""
    items: List[Dict[str, Any]] = []

    serial = hw.get("serial", {})
    ports = serial.get("ports", [])
    ok_port = [p for p in ports if p.get("rw") == "可读写"]
    # USB 串口和树莓派 GPIO 串口都可能接底盘；板载 ttyS0 默认不算（多数没启用）
    chassis_ports = [
        p for p in ok_port
        if p["dev"].startswith(("/dev/ttyUSB", "/dev/ttyACM", "/dev/ttyAMA", "/dev/serial0"))
    ]
    if chassis_ports:
        chassis_detail = "、".join(p["dev"] for p in chassis_ports)
    elif ports:
        chassis_detail = "有设备但无权限，把用户加进 dialout 组"
    else:
        chassis_detail = "未发现 USB 串口或 GPIO 串口"

    items.append(
        {
            "name": "底盘串口",
            "ok": bool(chassis_ports),
            "detail": chassis_detail,
            "hint": "插入 STM32 底板 USB，检查 CH340/CP210x 是否枚举",
        }
    )

    # 雷达有两种接法，判据必须分开：
    #   ① 走 USB 转接板（CP210x/FT232）→ Linux 侧能直接看到 /dev/ttyUSB*
    #   ② 直接接 MCU 的 UART（如 STM32 的 USART3 PB10/PB11）→ **Linux 侧根本看不到**
    # 早期只认 ①，导致「② 的板子永远报『雷达未接』」——2026-10-31 实际踩到：
    # 固件打印 `RPLIDAR A1 (USART3)`，配置齐全，而判据还在报「只有 1 个 CH340，雷达未接？」。
    # 所以 ② 要报「待确认」（"?"），不能报 ✗。
    usb_lidar = [p for p in ports if p.get("chip") in ("CP210x", "CP2105", "FT232", "FT231X")]
    ch340 = [p for p in ports if p.get("chip") == "CH340"]
    if usb_lidar:
        lidar_ok: Any = True
        lidar_detail = "USB 转接板：" + "、".join(p["dev"] for p in usb_lidar)
    elif chassis_ports:
        lidar_ok = "?"
        lidar_detail = (
            "没见到 USB 雷达，但有 MCU 调试口 —— 雷达可能直接接在 MCU 的 UART 上，"
            "这条线 Linux 侧看不到，要用固件自检确认"
        )
    elif ch340:
        lidar_ok = "?"
        lidar_detail = f"只有 {len(ch340)} 个 CH340，雷达接线未确认"
    else:
        lidar_ok = False
        lidar_detail = "未发现任何雷达相关串口"

    items.append(
        {
            "name": "雷达链路",
            "ok": lidar_ok,
            "detail": lidar_detail,
            "hint": "A1 走 USB 转接板时是 CP210x；直接接 MCU 时要在固件里自检 USART3",
        }
    )

    wifi = hw.get("wifi", {})
    wifi_ok = any(w.get("connected") and w.get("ipv4") for w in wifi.get("interfaces", []))
    items.append(
        {
            "name": "WiFi 联网",
            "ok": wifi_ok,
            "detail": "已联网并拿到 IP" if wifi_ok else "未连接或无 IPv4",
            "hint": "检查 rfkill 是否 blocked、SSID/密码、DHCP",
        }
    )

    # 底盘 ESP8266：只有真的探测过才列这一项，没测不该报 ✗
    esp = hw.get("esp")
    if esp is not None:
        if esp.get("found"):
            bits = [f"{esp.get('port')} @{esp.get('baud')}"]
            if esp.get("ssid"):
                bits.append(f"SSID {esp['ssid']}")
            if esp.get("rssi_dbm") is not None:
                bits.append(f"{esp['rssi_dbm']} dBm")
            if esp.get("ip"):
                bits.append(f"IP {esp['ip']}")
            esp_detail = "，".join(bits)
        elif esp.get("detected") == "navbot_stm32":
            esp_detail = f"{esp.get('detected_port')} 上挂的是 STM32 底盘，不是 WiFi 模块"
        elif esp.get("detected"):
            esp_detail = KIND_LABEL.get(esp["detected"], esp["detected"])
        else:
            esp_detail = esp.get("diagnosis") or "AT 无应答"
        items.append(
            {
                "name": "WiFi模块",
                "ok": bool(esp.get("found")),
                "detail": esp_detail,
                "hint": "ESP 直连 USB-TTL 才能直接探测；接 STM32 USART2 需固件做 AT 透传",
            }
        )

    items.append(
        {
            "name": "串口权限",
            "ok": bool(serial.get("in_dialout")),
            "detail": "用户已在 dialout 组" if serial.get("in_dialout") else "用户不在 dialout 组，开串口要 sudo",
            "hint": "sudo usermod -aG dialout $USER，注销重登生效",
        }
    )

    ros = hw.get("ros", {})
    items.append(
        {
            "name": "ROS 2 环境",
            "ok": ros.get("distro") not in (None, "", "未 source"),
            "detail": ros.get("distro") or "未 source",
            "hint": "source /opt/ros/humble/setup.bash && source ~/navbot_ws/install/setup.bash",
        }
    )

    pkgs = ros.get("packages") or {}
    alt = ros.get("packages_elsewhere") or {}
    has_lidar_drv = bool(pkgs.get("rplidar_ros") or pkgs.get("sllidar_ros2"))
    alt_lidar = alt.get("rplidar_ros") or alt.get("sllidar_ros2")
    not_sourced = not ros.get("ament_prefix_count")
    if has_lidar_drv:
        lidar_drv_ok: Any = True
        lidar_drv_detail = "已安装"
    elif alt_lidar:
        lidar_drv_ok = "?"
        lidar_drv_detail = f"不在当前环境，但 {alt_lidar} 里有 —— source 那个工作空间就能用"
    elif not_sourced:
        lidar_drv_ok = "?"
        lidar_drv_detail = "未 source，查不到（不是真的没装）"
    else:
        lidar_drv_ok = False
        lidar_drv_detail = "rplidar_ros / sllidar_ros2 都没装"
    items.append(
        {
            "name": "雷达驱动包",
            "ok": lidar_drv_ok,
            "detail": lidar_drv_detail,
            "hint": "apt 装：sudo apt install ros-humble-rplidar-ros（2.1.4）；"
                    "注意它只对「雷达直连 PC 的 USB 转接板」有用",
        }
    )

    batt = hw.get("power", {}).get("battery_pct")
    items.append(
        {
            "name": "供电",
            "ok": True if batt is None else batt >= 30,
            "detail": "交流供电/无电池信息" if batt is None else f"电量 {batt}%",
            "hint": "电量低于 30% 时电机堵转会拉垮电压导致 STM32 复位",
        }
    )
    return items


# ------------------------------------------------------------------ 渲染
class Printer:
    KVW = 13  # key 列宽：容得下 navbot_bridge 这类长包名

    def __init__(self, quiet: bool = False):
        self.quiet = quiet
        self.lines: List[str] = []

    def section(self, title: str) -> None:
        self.lines.append("")
        self.lines.append(f"【{title}】")

    def kv(self, key: str, value: Any, mark: str = "") -> None:
        if self.quiet:
            return
        v = "-" if value in (None, "", [], {}) else str(value)
        prefix = f"  {pad(key, self.KVW)}: "
        width = max(20, 82 - dwidth(prefix))
        chunks = _wrap(v, width)
        self.lines.append(prefix + mark + chunks[0])
        for c in chunks[1:]:
            self.lines.append(" " * dwidth(prefix) + c)

    def raw(self, text: str = "") -> None:
        self.lines.append(text)

    def sub(self, text: str) -> None:
        """子项：缩进对齐到 kv 的值列（2 + KVW + 2）。"""
        if self.quiet:
            return
        self.lines.append(" " * (2 + self.KVW + 2) + text)

    def dump(self) -> str:
        return "\n".join(self.lines)


def _wrap(text: str, width: int) -> List[str]:
    """按显示宽度折行：中文不切半，英文单词尽量不切断。"""
    if dwidth(text) <= width:
        return [text]
    out: List[str] = []
    rest = text
    while dwidth(rest) > width:
        cur, curw = "", 0
        for ch in rest:
            w = 2 if unicodedata.east_asian_width(ch) in ("W", "F") else 1
            if curw + w > width:
                break
            cur += ch
            curw += w
        if not cur:  # 单字符就超宽，兜底硬切
            cur = rest[:1]
        # 优先断在空格，避免把 STM32 切成 "STM" + "32"；
        # 但空格太靠前（< 1/3 行宽）就不值得，会让输出变得碎
        sp = cur.rfind(" ")
        if sp > width // 3:
            cur = cur[:sp]
        out.append(cur.rstrip())
        rest = rest[len(cur):].lstrip()
    if rest:
        out.append(rest)
    return out


def signal_bar(dbm: Optional[int], slots: int = 16) -> str:
    if dbm is None:
        return ""
    # -90 → 0 格，-30 → 16 格
    ratio = (dbm + 90) / 60.0
    n = max(0, min(slots, round(ratio * slots)))
    return "█" * n + "░" * (slots - n)


def render_text(hw: Dict[str, Any], args: argparse.Namespace) -> str:
    p = Printer(args.quiet)
    now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    if not args.quiet:
        p.raw("=" * 78)
        p.raw(f" navbot 硬件勘察  ·  {now}  ·  v{VERSION}")
        p.raw(f" 主机 {hw['system'].get('hostname')}  ·  以 root 运行：{'是' if hw['system'].get('is_root') else '否'}")
        p.raw("=" * 78)

    # --- 板卡
    b, s = hw["board"], hw["system"]
    p.section("板卡与系统")
    p.kv("机型", b.get("model") or "未知")
    p.kv(
        "类型",
        f"{b.get('kind') or '未知'}  ·  {b.get('arch') or '?'}  ·  来源 {b.get('source') or '未知'}",
    )
    if b.get("board"):
        p.kv("主板", b["board"])
    p.kv("系统", f"{s.get('distro') or '未知'}  内核 {s.get('kernel') or '?'}")
    up = s.get("uptime_human")
    if up:
        p.kv("运行时长", f"{up}（开机于 {s.get('boot_time')}）")
    if s.get("container"):
        p.kv("容器", f"是（{s['container']}），部分硬件信息不可见")

    # --- CPU / 内存
    c, m = hw["cpu"], hw["memory"]
    p.section("处理器与内存")
    p.kv("CPU", c.get("model") or "未知")
    freq = f"{c.get('freq_mhz')} MHz" if c.get("freq_mhz") else "未知"
    p.kv(
        "核心/频率",
        f"{c.get('cores') or '?'} 核 / {c.get('threads') or '?'} 线程  ·  当前 {freq}",
    )
    if c.get("load"):
        ld = c["load"]
        p.kv("负载", f"1/5/15 分 {ld[0]:.2f} / {ld[1]:.2f} / {ld[2]:.2f}  （{c.get('load_ratio')} × 线程数）")
    mem_line = (
        f"{m.get('total') or '?'} 总量  ·  已用 {m.get('used_pct', '?')}%  ·  "
        f"{m.get('available') or '?'} 可用"
    )
    p.kv("内存", mem_line)
    p.kv("交换", m.get("swap") or "未知")

    # --- 温度 / 电源
    t, pw = hw["temp"], hw["power"]
    p.section("温度与供电")
    if t.get("cpu_c") is not None:
        p.kv("CPU 温度", f"{t['cpu_c']} °C  （{t.get('cpu_state')}）")
    else:
        p.kv("CPU 温度", "读不到（无 thermal_zone / 无 vcgencmd）")
    if t.get("wifi_c") is not None:
        p.kv("WiFi 芯片", f"{t['wifi_c']} °C")
    if pw.get("has_battery"):
        p.kv("电池", f"{pw.get('battery_pct')}%  {pw.get('battery_status')}")
    else:
        p.kv("供电", "无电池（交流/DC 供电）")
    if pw.get("rpi_undervoltage") is not None:
        p.kv(
            "欠压标志",
            "检测到欠压！换电源/换线" if pw["rpi_undervoltage"] else "正常",
        )

    # --- 存储
    p.section("存储")
    disks = hw["storage"].get("disks", [])
    if not disks:
        p.kv("磁盘", "未发现")
    for dsk in disks:
        line = f"{dsk.get('name')}  {human_bytes(dsk.get('size'))}"
        if dsk.get("model"):
            line += f"  {dsk['model']}"
        if dsk.get("tran"):
            line += f"  [{dsk['tran']}]"
        if dsk.get("media"):
            line += f"  {dsk['media']}"
        p.kv("磁盘", line)
        for kid in dsk.get("children", []) or []:
            sub = f"{kid.get('name')}  {human_bytes(kid.get('size'))}"
            if kid.get("fstype"):
                sub += f"  {kid['fstype']}"
            if kid.get("mountpoint"):
                sub += f"  → {kid['mountpoint']}"
            p.sub("└ " + sub)

    # --- USB
    p.section("USB 外设")
    usb = hw["usb"]
    if not usb.get("devices"):
        p.kv("设备", "无（容器里常见）")
    for dev in usb.get("devices", []):
        line = f"{dev['vid_pid']}  {dev.get('product') or dev.get('manufacturer') or '未知设备'}"
        if dev.get("known"):
            line += f"  ← {dev['known']}（{dev.get('role')}）"
        p.kv("设备", line)

    # --- 串口
    p.section("串口（底盘 / 雷达）")
    ser = hw["serial"]
    if not ser.get("ports"):
        p.kv("串口", "未发现（USB 转串口 / GPIO 串口 / 板载串口都没有）")
    for port in ser.get("ports", []):
        line = f"{port['dev']}  {port.get('rw')}"
        if port.get("chip"):
            line += f"  {port['chip']}"
        if port.get("vid_pid"):
            line += f"  [{port['vid_pid']}]"
        if port.get("kind"):
            line += f"  {port['kind']}"
        p.kv("端口", line)
    for link in ser.get("by_id", []):
        p.kv("稳定链接", f"{link['link']} → {link['target']}")
    who = ser.get("group_user") or os.environ.get("USER") or "当前用户"
    p.kv(
        "用户组",
        f"{who} {'已在' if ser.get('in_dialout') else '不在'} dialout 组"
        + ("" if ser.get("in_dialout") else "，开串口要 sudo"),
    )

    # --- 总线
    bus = hw["buses"]
    p.section("总线与外设节点")
    p.kv("I2C", _fmt_list(bus.get("i2c") or []))
    p.kv("SPI", _fmt_list(bus.get("spi") or []))
    p.kv("视频", _fmt_list(bus.get("video") or []))
    if bus.get("video_names"):
        p.kv("摄像头", "；".join(bus["video_names"][:2]))
    p.kv("GPIO", _fmt_list(bus.get("gpio") or []))

    # --- 网络
    net = hw["net"]
    p.section("网络接口")
    for it in net.get("interfaces", []):
        if it["name"] == "lo":
            continue
        line = f"{it['type']:<4} {it['state']:<6} {it.get('mac') or '-'}"
        if it.get("ipv4"):
            line += f"  {', '.join(it['ipv4'])}"
        if it.get("link_speed"):
            line += f"  {it['link_speed']}"
        if it.get("driver"):
            line += f"  [{it['driver']}]"
        p.kv(it["name"], line)
    rt = net.get("default_route")
    if rt:
        p.kv("默认路由", f"via {rt.get('gateway')}  dev {rt.get('dev')}")
    if net.get("dns"):
        p.kv("DNS", "、".join(net["dns"]))

    # --- WiFi（重点）
    wifi = hw["wifi"]
    p.section("WiFi 详情")
    if not wifi.get("available"):
        p.kv("无线接口", "未发现（本机可能没有 WiFi 网卡）")
    for w in wifi.get("interfaces", []):
        head = f"{w['name']}  驱动 {w.get('driver') or '?'}"
        if w.get("phy"):
            head += f"  phy {w['phy']}"
        p.kv("接口", head)
        if w.get("mode"):
            p.kv("工作模式", w["mode"])
        if w.get("connected_inferred"):
            p.kv("连接状态", "已连接（由信号+IP 推断，本机缺 iw/nmcli）")
        else:
            p.kv("连接状态", "已连接" if w.get("connected") else "未连接")
        if w.get("ssid"):
            p.kv("SSID", w["ssid"])
        if w.get("bssid"):
            p.kv("BSSID", w["bssid"])
        if w.get("channel") or w.get("freq_mhz"):
            p.kv("信道/频段", f"{w.get('channel', '?')}  {w.get('freq_mhz', '?')} MHz  {w.get('band', '')}")
        if w.get("signal_dbm") is not None:
            bar = signal_bar(w["signal_dbm"])
            p.kv(
                "信号强度",
                f"{w['signal_dbm']} dBm  {bar}  {w.get('signal_grade')}（{w.get('signal_note')}）",
            )
            if w.get("signal_source"):
                p.kv("信号来源", w["signal_source"])
        if w.get("quality") is not None:
            p.kv("链路质量", f"{w['quality']}/70")
        if w.get("tx_rate_mbps"):
            p.kv("发送速率", f"{w['tx_rate_mbps']:.0f} Mbps")
        elif w.get("rate"):
            p.kv("协商速率", w["rate"])
        if w.get("security"):
            p.kv("加密", w["security"])
        if w.get("txpower_dbm") is not None:
            p.kv("发射功率", f"{w['txpower_dbm']} dBm")
        if w.get("ap_capable") is not None:
            p.kv("AP 模式", "支持（可做热点）" if w["ap_capable"] else "不支持")
        if w.get("supported_modes"):
            p.kv("支持模式", "、".join(w["supported_modes"]))
        if w.get("ipv4"):
            p.kv("IP", "、".join(w["ipv4"]))

    rk = wifi.get("rfkill") or {}
    if rk.get("available"):
        if rk.get("blocked"):
            names = "、".join(f"{x['name']}(软{1 if x.get('soft_blocked') else 0}/硬{1 if x.get('hard_blocked') else 0})" for x in rk["blocked"])
            p.kv("rfkill", f"被阻塞：{names}  ← WiFi 打不开先看这里")
        else:
            p.kv("rfkill", "未阻塞")
    if wifi.get("reg_country"):
        p.kv("监管域", wifi["reg_country"])

    # --- 底盘 WiFi 模块（ESP8266，AT 探测）
    if hw.get("esp") is not None:
        esp = hw["esp"]
        p.section("底盘 WiFi 模块（ESP8266 / AT）")
        if esp.get("error"):
            p.kv("状态", esp["error"])
        elif esp.get("found"):
            p.kv("端口", f"{esp['port']}  @ {esp['baud']} 8N1")
            ver = []
            if esp.get("at_version"):
                ver.append(f"AT {esp['at_version']}")
            if esp.get("sdk_version"):
                ver.append(f"SDK {esp['sdk_version']}")
            if esp.get("bin_version"):
                ver.append(f"Bin {esp['bin_version']}")
            p.kv("固件", "  ·  ".join(ver) if ver else "未知（AT+GMR 无有效字段）")
            if esp.get("cwm_mode") is not None:
                p.kv("工作模式", f"{esp['cwm_mode']}  {esp.get('cwm_mode_text') or ''}")
            if esp.get("ssid"):
                p.kv("SSID", esp["ssid"])
            if esp.get("bssid"):
                p.kv("BSSID", esp["bssid"])
            if esp.get("channel"):
                p.kv("信道", esp["channel"])
            if esp.get("rssi_dbm") is not None:
                bar = signal_bar(esp["rssi_dbm"])
                p.kv(
                    "信号强度",
                    f"{esp['rssi_dbm']} dBm  {bar}  {esp.get('signal_grade')}"
                    f"（{esp.get('signal_note')}）",
                )
            if esp.get("ip"):
                p.kv("IP / 网关", f"{esp['ip']}  /  {esp.get('gateway') or '?'}")
            if esp.get("mac"):
                p.kv("MAC", esp["mac"])
            if esp.get("ap_state"):
                p.kv("AP 状态", esp["ap_state"])
            cwlap = (esp.get("raw") or {}).get("cwlap") or ""
            if cwlap:
                nets = re.findall(r'\+CWLAP:\((\d+),"([^"]*)",(-?\d+)', cwlap)
                top = sorted(nets, key=lambda x: -int(x[2]))[:5]
                p.kv("周边 AP", f"扫到 {len(nets)} 个，最强 5 个：" +
                     "、".join(f"{s} {r}dBm" for _e, s, r in top))
        else:
            atts = esp.get("attempts") or []
            if esp.get("detected"):
                p.kv(
                    "对面是",
                    f"{KIND_LABEL.get(esp['detected'], esp['detected'])}"
                    f"  —  {esp.get('detected_port')} @ {esp.get('detected_baud')}",
                )
            p.kv("结果", f"没探到 ESP8266（试过 {len(atts)} 组端口×波特率组合）")
            for a in atts[:6]:
                line = f"{a.get('port')} @ {a.get('baud') or '?'}"
                if a.get("busy"):
                    line += f"   被占用：{a['busy']}"
                elif a.get("note"):
                    line += f"   {a['note']}"
                elif a.get("classify"):
                    line += f"   {KIND_LABEL.get(a['classify'], a['classify'])}"
                elif a.get("reply"):
                    line += f"   回应：{a['reply'][:40]}"
                else:
                    line += "   静默（没有 AT 应答，也没有主动数据）"
                p.sub(line)
                if a.get("passive"):
                    p.sub(f"  └ 静听到：{a['passive'][:56]}")
                    if a.get("passive_hex"):
                        p.sub(f"     hex：{a['passive_hex']}")
            if len(atts) > 6:
                p.sub(f"…… 另有 {len(atts) - 6} 次尝试未列出")
            if esp.get("diagnosis"):
                p.kv("诊断", esp["diagnosis"])

    # --- 波特率扫描
    if hw.get("baudscan"):
        bs = hw["baudscan"]
        p.section("波特率扫描（只静听）")
        if bs.get("error"):
            p.kv("状态", bs["error"])
        else:
            p.kv("端口", f"{bs.get('port')}  每档静听 {bs.get('seconds')} 秒")
            for r in bs.get("results", []):
                if r.get("error"):
                    p.kv(str(r["baud"]), f"打不开：{r['error']}")
                    continue
                n = r.get("bytes", 0)
                if not n:
                    p.kv(str(r["baud"]), "0 字节（静默）")
                    continue
                kind = KIND_LABEL.get(r.get("classify"), r.get("classify") or "未分类")
                p.kv(str(r["baud"]), f"{n} 字节  →  {kind}")
                if r.get("text"):
                    p.sub(f"  └ {r['text'][:56]}")
                if r.get("hex"):
                    p.sub(f"     hex：{r['hex']}")
            if bs.get("likely_baud"):
                p.kv("最可能", f"{bs['likely_baud']}（这一档收到了可读内容）")
            else:
                # 全乱码但有数据量：说明对面在发，只是窗口/波特率没对上
                noisy = [r for r in bs.get("results", []) if (r.get("bytes") or 0) > 20]
                if noisy:
                    top = max(noisy, key=lambda r: r["bytes"])
                    p.kv("判断", f"{top['baud']} 收到 {top['bytes']} 字节但读不懂")
                    p.kv(
                        "下一步",
                        f"数据量说明对面确实在发，只是这档采样错了。若已知固件波特率，"
                        f"直接用它长听：--listen --baud <值> --sec 30，期间按复位键。",
                    )
                else:
                    p.kv("最可能", "没有一档收到内容 —— 按一下板子复位键再扫一次")

    # --- 长时间静听（dump 原始数据）
    if hw.get("listen"):
        ls = hw["listen"]
        p.section(f"静听 {ls.get('port', '?')} @ {ls.get('baud', '?')}")
        if ls.get("error"):
            p.kv("状态", ls["error"])
        else:
            n = ls.get("bytes", 0)
            kind = KIND_LABEL.get(ls.get("classify"), ls.get("classify") or "没有任何数据")
            p.kv("时长", f"{ls.get('seconds')} 秒")
            p.kv("收到", f"{n} 字节（约 {ls.get('rate_bps')} B/s）  →  {kind}"
                 if n else "0 字节 —— 这段时间里对面一句话都没说")
            if ls.get("text"):
                all_lines = [ln for ln in ls["text"].splitlines() if ln.strip()]
                show = all_lines[:40]
                if show:
                    p.kv("内容", show[0][:72])
                    for line in show[1:]:
                        p.sub(line[:78])
                total = ls.get("lines_total") or len(all_lines)
                if total > len(show):
                    p.sub(f"…… 共 {total} 行，屏幕只放前 {len(show)} 行；加 --save 文件 存全文")
            elif ls.get("hex"):
                p.kv("hex", ls["hex"][:88])
            if ls.get("saved"):
                p.kv("已存全文", ls["saved"])
            elif ls.get("save_error"):
                p.kv("保存失败", ls["save_error"])
            if not n:
                p.kv("提示", "按一下板子复位键再跑一次；或确认固件是否真的在 printf")

    # --- ROS2
    ros = hw["ros"]
    p.section("ROS 2 环境")
    if ros.get("error"):
        p.kv("状态", ros["error"])
    p.kv("发行版", ros.get("distro") or "未知")
    p.kv("RMW", f"{ros.get('rmw') or '默认'}  · domain {ros.get('domain_id') or 0}")
    p.kv("prefix 数", f"{ros.get('ament_prefix_count') or 0} 条")
    if not ros.get("ament_prefix_count"):
        p.kv("关键包", "当前 shell 未 source，查不到（不是真的没装）")
        if ros.get("installed_distros"):
            p.kv("已装发行版", "、".join(ros["installed_distros"]))
    else:
        alt_map = ros.get("packages_elsewhere") or {}
        for pkg, path in (ros.get("packages") or {}).items():
            if path:
                p.kv(pkg, path)
            elif pkg in alt_map:
                p.kv(pkg, f"不在当前环境 · 但 {alt_map[pkg]} 里有 —— source 对应工作空间即可")
            else:
                p.kv(pkg, "未安装")

    # --- 扫描
    if args.scan and hw.get("wifi_scan"):
        sc = hw["wifi_scan"]
        p.section("周边 WiFi")
        if sc.get("error"):
            p.kv("扫描", f"失败：{sc['error']}")
        for n in sc.get("networks", [])[:12]:
            mark = "*" if n.get("in_use") else " "
            sig = n.get("signal_pct") or n.get("signal_dbm")
            p.kv(
                "网络",
                f"{mark} {n['ssid']}  信号 {sig}  {n.get('channel') or ''}  {n.get('security') or ''}",
            )

    # --- 链路测试
    if args.link_test and hw.get("link_test"):
        lt = hw["link_test"]
        p.section("链路质量")
        if lt.get("error"):
            p.kv("状态", f"跳过：{lt['error']}")
        elif not lt.get("targets"):
            p.kv("状态", "无可用目标")
        for t in lt.get("targets", []):
            line = f"{t['host']}  丢包 {t.get('loss_pct', '?')}%"
            if "rtt_avg" in t:
                line += f"  RTT 均 {t['rtt_avg']} / 峰 {t.get('rtt_max', '?')} ms"
            line += f"  → {t.get('verdict', '?')}"
            p.kv(t["label"], line)

    # --- 采集告警：任何模块失败都在这里显形，绝不静默吞掉
    warn_keys = [
        ("board", "板卡"), ("system", "系统"), ("cpu", "CPU"), ("memory", "内存"),
        ("temp", "温度"), ("power", "供电"), ("storage", "存储"), ("usb", "USB"),
        ("serial", "串口"), ("buses", "总线"), ("net", "网络"), ("wifi", "WiFi"),
        ("ros", "ROS 2"), ("wifi_scan", "WiFi 扫描"), ("link_test", "链路测试"),
        ("esp", "ESP8266"),
    ]
    warns = []
    for key, label in warn_keys:
        blk = hw.get(key)
        if isinstance(blk, dict) and blk.get("error"):
            warns.append(f"{label} — {blk['error']}")
    if warns or hw.get("meta", {}).get("readiness_error"):
        p.section("采集告警")
        for w in warns:
            p.kv("失败", w)
        if hw.get("meta", {}).get("readiness_error"):
            p.kv("失败", f"就绪判据 — {hw['meta']['readiness_error']}")

    # --- 就绪判据（三态：✓ 达标 / ? 待确认 / ✗ 未达标）
    ready = hw.get("readiness", [])
    p.section("上路就绪检查")
    if not args.quiet:
        for it in ready:
            mark = "✓" if it["ok"] is True else ("?" if it["ok"] == "?" else "✗")
            p.raw(f"  {mark} {pad(it['name'], 10)} {it['detail']}")
            if it["ok"] is not True and it.get("hint"):
                p.raw(f"      → {it['hint']}")
        ok = sum(1 for x in ready if x["ok"] is True)
        pending = sum(1 for x in ready if x["ok"] == "?")
        p.raw("")
        p.raw(
            f"  达标 {ok}/{len(ready)}"
            + (f"（另有 {pending} 项待确认）" if pending else "")
        )

    # quiet 模式只列异常
    if args.quiet:
        bad = [x for x in ready if x["ok"] is not True]
        if not bad:
            return "全部达标（" + str(len(ready)) + " 项）\n"
        out = [f"未达标 {len(bad)}/{len(ready)} 项："]
        for it in bad:
            mark = "?" if it["ok"] == "?" else "✗"
            out.append(f"  {mark} {it['name']}: {it['detail']}")
            if it.get("hint"):
                out.append(f"      → {it['hint']}")
        return "\n".join(out) + "\n"

    p.raw("")
    return p.dump() + "\n"


# ---------------------------------------------------------------------- main
def collect(args: argparse.Namespace) -> Dict[str, Any]:
    """逐个采集，单个模块失败只记一笔，不拖垮整份报告。

    真机上 sysfs 千奇百怪（内核裁剪、容器限制、厂商魔改），
    一个模块抛异常就让整份报告空白是不可接受的。
    """
    t0 = time.time()

    def safe(name: str, fn, default, *a, **kw) -> Any:
        try:
            return fn(*a, **kw)
        except Exception as e:  # noqa: BLE001 — 这里就是要兜住一切
            d = dict(default) if default else {}
            d["error"] = f"{name} 采集失败：{type(e).__name__}: {e}"
            return d

    board = safe("板卡", probe_board, {})
    system = safe("系统", probe_system, {})
    cpu = safe("CPU", probe_cpu, {})
    memory = safe("内存", probe_memory, {})
    temp = safe("温度", probe_temp, {})
    power = safe("供电", probe_power, {})
    storage = safe("存储", probe_storage, {"disks": []})
    usb = safe("USB", probe_usb, {"devices": [], "count": 0})
    serial = safe("串口", probe_serial, {"ports": [], "by_id": [], "count": 0})
    buses = safe("总线", probe_buses, {})
    net = safe("网络", probe_net, {"interfaces": []})
    wifi = safe("WiFi", probe_wifi, {"interfaces": [], "available": False}, net)
    ros = safe("ROS 2", probe_ros, {})

    hw: Dict[str, Any] = {
        "meta": {
            "script_version": VERSION,
            "collected_at": datetime.now().isoformat(timespec="seconds"),
            "hostname": socket.gethostname(),
            "argv": " ".join(sys.argv),
            "elapsed_sec": None,
        },
        "board": board,
        "system": system,
        "cpu": cpu,
        "memory": memory,
        "temp": temp,
        "power": power,
        "storage": storage,
        "usb": usb,
        "serial": serial,
        "buses": buses,
        "net": net,
        "wifi": wifi,
        "ros": ros,
    }

    if args.scan:
        hw["wifi_scan"] = safe("WiFi 扫描", probe_scan, {"networks": []})
    if args.link_test:
        gw = (net.get("default_route") or {}).get("gateway")
        hw["link_test"] = safe("链路测试", probe_link, {"targets": []}, gw)
    if getattr(args, "wifi_module", None):
        cand = [p["dev"] for p in (serial.get("ports") or [])]
        explicit = None if args.wifi_module == "auto" else args.wifi_module
        hw["esp"] = safe(
            "ESP8266", probe_esp8266, {"found": False, "attempts": []},
            cand, explicit, args.scan,
        )
    if getattr(args, "baudscan", None):
        ports = [p["dev"] for p in (serial.get("ports") or [])]
        target = args.baudscan
        if target == "auto":
            target = ports[0] if ports else None
        if target:
            hw["baudscan"] = safe("波特率扫描", probe_baud_scan, {"results": []}, target)
    if getattr(args, "listen", None):
        ports = [p["dev"] for p in (serial.get("ports") or [])]
        target = args.listen
        if target == "auto":
            target = ports[0] if ports else None
        if target:
            hw["listen"] = safe(
                "静听", probe_listen, {},
                target, getattr(args, "baud", 115200), getattr(args, "sec", 15.0),
                getattr(args, "save", None),
            )

    try:
        hw["readiness"] = readiness(hw)
    except Exception as e:  # noqa: BLE001
        hw["readiness"] = []
        hw["meta"]["readiness_error"] = f"{type(e).__name__}: {e}"
    hw["meta"]["elapsed_sec"] = round(time.time() - t0, 2)
    return hw


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(
        description="navbot 硬件信息勘察（只读）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__.split("用法\n----")[1] if "用法\n----" in __doc__ else None,
    )
    ap.add_argument("--json", metavar="PATH", nargs="?", const="-",
                    help="输出 JSON；省略路径或写 - 表示只输出到 stdout")
    ap.add_argument("--quiet", action="store_true", help="只列未达标项")
    ap.add_argument("--scan", action="store_true", help="扫描周边 WiFi（需权限）")
    ap.add_argument(
        "--wifi-module", metavar="PORT", nargs="?", const="auto",
        help="探测底盘上的 ESP8266（只发 AT 查询，不改任何状态）；"
             "不带值＝自动遍历所有串口，带值＝只测指定端口如 /dev/ttyUSB1",
    )
    ap.add_argument(
        "--baudscan", metavar="PORT", nargs="?", const="auto",
        help="逐个波特率静听，反推真实波特率（跑的时候按一下板子复位键最有效）；"
             "不带值＝用第一个串口",
    )
    ap.add_argument(
        "--listen", metavar="PORT", nargs="?", const="auto",
        help="在指定波特率上长时间静听并 dump 原始数据；期间按一下板子复位键。"
             "配合 --baud / --sec",
    )
    ap.add_argument("--baud", type=int, default=115200,
                    help="--listen 用的波特率（默认 115200）")
    ap.add_argument("--sec", type=float, default=15.0,
                    help="--listen 静听时长，秒（默认 15）")
    ap.add_argument("--save", metavar="FILE",
                    help="--listen 时把完整原始字节另存为文件（屏幕只显示开头几十行）")
    ap.add_argument("--link-test", action="store_true", help="ping 网关/公网测链路质量")
    ap.add_argument("--strict", action="store_true", help="有判据未达标则退出码 1")
    ap.add_argument("--version", action="version", version=f"%(prog)s {VERSION}")
    args = ap.parse_args(argv)

    hw = collect(args)

    # --json - 时只吐 JSON，不混文本，方便管道
    json_only = args.json == "-"
    text = render_text(hw, args)
    if not json_only:
        sys.stdout.write(text)

    if args.json:
        payload = json.dumps(hw, ensure_ascii=False, indent=2)
        if json_only:
            sys.stdout.write(payload + "\n")
        else:
            path = os.path.abspath(args.json)
            try:
                with open(path, "w", encoding="utf-8") as f:
                    f.write(payload + "\n")
                sys.stdout.write(f"[JSON 已写入] {path}\n")
            except OSError as e:
                sys.stderr.write(f"[写 JSON 失败] {e}\n")

    if args.strict:
        # 只对明确的 ✗ 报错；「待确认」不算失败，否则门禁会被不可见项卡死
        bad = [x for x in hw["readiness"] if x["ok"] is False]
        return 1 if bad else 0
    return 0


if __name__ == "__main__":
    sys.exit(main())
