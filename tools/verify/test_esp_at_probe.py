#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""验证 check_hw.py 的 ESP8266 AT 探测。

分三层，能跑到哪层看你机器的条件：

  A. 解析层 —— 把「真实模块会回的那几段文本」直接喂给 `_parse_esp_responses`，
     断言每个字段都提对了。**任何环境都能跑**（含无串口的容器/CI）。
     这层是重点：AT 解析写错的表现是「什么都没拿到」，和「模块没接」一模一样。

  B. 注入层 —— 用 socketpair 冒充串口 fd（monkeypatch 掉 `_open_tty`），
     跑 probe_esp8266 的完整主流程：端口占用检测 → 逐波特率试探 → 找到模块
     → 逐条查询 → 解析 → found=True。**任何环境都能跑**，不需要 /dev/ptmx。
     盖不住 termios 那 20 行，交给 C 层。

  C. 收发层 —— 用真 pty 造一个必定应答的假 ESP8266，连 termios 一起验。
     需要 /dev/ptmx；容器里通常没有，拿不到就 SKIP（退出码不算失败），
     但会明确打印「termios 那段未验证」。

用法：
    python3 tools/verify/test_esp_at_probe.py
    python3 tools/verify/test_esp_at_probe.py --parse-only   # 只跑 A，最快
"""

from __future__ import annotations

import importlib.util
import os
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
CHECK_HW = os.path.join(HERE, "..", "env", "check_hw.py")


def load_module():
    spec = importlib.util.spec_from_file_location("check_hw", CHECK_HW)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


# 一个「标准 ESP8266 AT 固件 1.7.4」会怎么回话（含命令回显，和真模块一致）
REPLIES = {
    "AT": "AT\r\n\r\nOK\r\n",
    "AT+GMR": (
        "AT+GMR\r\n"
        "AT version:1.7.4.0(May 11 2020 19:13:04)\r\n"
        "SDK version:3.0.4(9532ceb)\r\n"
        "compile time:May 27 2020 10:12:17\r\n"
        "Bin version(Wroom 02):1.7.4\r\n"
        "OK\r\n"
    ),
    "AT+CWMODE?": "AT+CWMODE?\r\n+CWMODE:1\r\n\r\nOK\r\n",
    "AT+CWJAP?": (
        'AT+CWJAP?\r\n+CWJAP:"HAIT-Student-5G-13","aa:bb:cc:dd:ee:ff",44,-55\r\n\r\nOK\r\n'
    ),
    "AT+CIPSTA?": (
        "AT+CIPSTA?\r\n"
        '+CIPSTA:ip:"10.50.211.99"\r\n'
        '+CIPSTA:gateway:"10.50.128.1"\r\n'
        '+CIPSTA:netmask:"255.255.128.0"\r\n'
        "\r\nOK\r\n"
    ),
    "AT+CIFSR": (
        "AT+CIFSR\r\n"
        '+CIFSR:STAIP,"10.50.211.99"\r\n'
        '+CIFSR:STAMAC,"dc:4f:22:11:22:33"\r\n'
        "\r\nOK\r\n"
    ),
}

# 已连 AP 的期望结果
EXPECT_ONLINE = {
    "at_version": "1.7.4.0",
    "sdk_version": "3.0.4",
    "bin_version": "1.7.4",
    "cwm_mode": 1,
    "ssid": "HAIT-Student-5G-13",
    "bssid": "aa:bb:cc:dd:ee:ff",
    "channel": 44,
    "rssi_dbm": -55,
    "ip": "10.50.211.99",
    "gateway": "10.50.128.1",
    "mac": "dc:4f:22:11:22:33",
    "signal_grade": "良好",
}


# ---------------------------------------------------------------- A. 解析层
def test_parse(chk) -> int:
    raw = {
        "firmware": REPLIES["AT+GMR"],
        "mode": REPLIES["AT+CWMODE?"],
        "ap": REPLIES["AT+CWJAP?"],
        "ip": REPLIES["AT+CIPSTA?"],
        "iface": REPLIES["AT+CIFSR"],
    }
    got = chk._parse_esp_responses(raw)

    ok = 0
    total = 0
    for key, want in EXPECT_ONLINE.items():
        total += 1
        hit = got.get(key) == want
        ok += 1 if hit else 0
        if not hit:
            print(f"  FAIL  {key}: 期望 {want!r}，实际 {got.get(key)!r}")
        else:
            print(f"  PASS  {key} = {want!r}")

    # 边界 1：未连 AP
    total += 1
    r2 = chk._parse_esp_responses({"ap": "AT+CWJAP?\r\nNo AP\r\n\r\nOK\r\n"})
    if r2.get("ap_state") == "未连接任何 AP" and "ssid" not in r2:
        ok += 1
        print("  PASS  未连 AP 时给 ap_state，且不臆造 ssid")
    else:
        print(f"  FAIL  未连 AP 的解析：{r2!r}")

    # 边界 2：全空响应不能抛异常，也不能编造字段
    total += 1
    try:
        r3 = chk._parse_esp_responses({})
        if r3 == {}:
            ok += 1
            print("  PASS  空响应返回空 dict，不抛异常")
        else:
            print(f"  FAIL  空响应返回了 {r3!r}")
    except Exception as e:
        print(f"  FAIL  空响应抛异常：{e}")

    # 边界 3：报文被截断（模块只回了一半）
    total += 1
    r4 = chk._parse_esp_responses({"ap": 'AT+CWJAP?\r\n+CWJAP:"MyWiFi"'})
    if "ssid" not in r4:
        ok += 1
        print("  PASS  截断报文不被误解析成完整记录")
    else:
        print(f"  FAIL  截断报文被硬解析：{r4!r}")

    print(f"\n解析层：{ok}/{total} 通过")
    return 0 if ok == total else 1


# ------------------------------------------------------------ A2. 占用检测
def test_port_busy(chk) -> int:
    """端口占用检测：自己持有不算数、别人持有要能点出 PID。

    这条出过坑——早先用 fuser 并 findall 数字，把 `/dev/pts/1` 里的 `1` 当成 PID，
    也没排除自身，结果探测进程「自己把自己判成占用」，一条 AT 都没发就返回。
    改用扫 /proc/*/fd + 比 realpath 之后才可靠。
    """
    import subprocess
    import tempfile

    fd, path = tempfile.mkstemp(prefix="busy_probe_")
    os.close(fd)
    ok = total = 0

    # 1) 只有自己持有 → 必须判空闲，否则就是「自锁」
    total += 1
    with open(path, "r"):
        got = chk._port_busy(path)
    if got is None:
        ok += 1
        print("  PASS  只有自己持有 → 判空闲（不自锁）")
    else:
        print(f"  FAIL  自己持有被误判成占用：{got}")

    # 2) 另一个进程持有 → 必须点出来
    total += 1
    child = subprocess.Popen(
        [sys.executable, "-c", f"import time; f=open({path!r}); time.sleep(15)"]
    )
    try:
        time.sleep(0.6)
        got = chk._port_busy(path)
        if got and str(child.pid) in got:
            ok += 1
            print(f"  PASS  别的进程持有 → 检出 {got}")
        else:
            print(f"  FAIL  别的进程持有却没检出：{got!r}")
    finally:
        child.kill()
        child.wait()

    # 3) 释放后回到空闲
    total += 1
    if chk._port_busy(path) is None:
        ok += 1
        print("  PASS  释放后回到空闲")
    else:
        print("  FAIL  释放后仍判占用")

    # 4) 不存在的路径不抛异常
    total += 1
    try:
        if chk._port_busy("/dev/ttyUSB99-not-exist") is None:
            ok += 1
            print("  PASS  不存在的路径安全返回 None")
        else:
            print("  FAIL  不存在的路径返回了占用信息")
    except Exception as e:
        print(f"  FAIL  不存在的路径抛异常：{e}")

    try:
        os.unlink(path)
    except OSError:
        pass
    print(f"\n占用检测：{ok}/{total} 通过")
    return 0 if ok == total else 1


# ------------------------------------------------------------ A3. 身份识别
def test_classify_port(chk) -> int:
    """串口身份识别：对面是底盘还是 WiFi 模块，得说清楚。

    「AT 无应答」是句废话，用户拿它没法判断插错口没有。
    navbot 的 STM32 每 50Hz 主动吐 O 帧，静听一秒就能认出来——这才是有效信息。
    """
    import socket

    samples = [
        ("AT 应答", "AT\r\n\r\nOK\r\n", "esp"),
        ("O 帧", "O,128,1056,-1056,0.250,-0.250,120,120,11800,41235,3210\r\n", "navbot_stm32"),
        ("V 帧", "V,1,0.2,0.2\r\n", "navbot_stm32"),
        # 真机样本：这是 2026-10-31 在小梁机器上 @9600 实际收到的内容。
        # 旧判据用 str.isprintable()，可打印率 0.72 > 0.7，被误判成「可读文本」——
        # 因为 decode 出来的 U+FFFD 竟然 isprintable() == True。
        (
            "真机乱码（9600 收到）",
            b"l\x00ckkkk\x00k.&d\xff\xfe\xfdkk3Ff\x00#\x80gWF\x01Fx\x02F\x03f\x00\x04dF\x05F"
            b"\x06\x07\x08d\x00$g".decode("utf-8", "replace"),
            "garbled",
        ),
        # 注意样本一律用 bytes.decode 构造：直接写 str 字面量会绕过替换字符，
        # 测不出「decode 产生 U+FFFD」这条真实路径。
        ("短乱码", b"\x00\xff\xfe\x01\x02\x80".decode("utf-8", "replace"), "garbled"),
        ("纯控制字符", b"\x01\x02\x03\x04\x05\x06\x07\x08\x0b\x0c".decode("utf-8", "replace"), "garbled"),
        ("调试输出", "STM32F103 ready, build 20261022\r\n", "stm32_debug"),
        ("英文文本", "hello world this is a test", "text"),
        ("纯中文（不该判乱码）", "系统启动完成请等待\r\n", "text"),
        ("空", "", None),
    ]
    ok = total = 0
    for name, text, want in samples:
        total += 1
        got = chk._classify_port_reply(text)
        if got == want:
            ok += 1
            print(f"  PASS  {name} → {got}")
        else:
            print(f"  FAIL  {name} → {got}（期望 {want}）")

    # 端到端：对面持续发 O 帧，静听必须认成底盘，且不能误报 found=True
    a, b = socket.socketpair()
    a.setblocking(False)
    b.setblocking(False)
    stop = threading.Event()
    O_FRAME = b"O,128,1056,-1056,0.250,-0.250,120,120,11800,41235,3210\r\n"

    def emitter():
        while not stop.is_set():
            try:
                b.sendall(O_FRAME)
            except OSError:
                return
            time.sleep(0.1)

    threading.Thread(target=emitter, daemon=True).start()

    orig = chk._open_tty
    chk._open_tty = lambda port, baud: os.dup(a.fileno())
    try:
        r = chk.probe_esp8266(["/dev/fake-chassis"], explicit="/dev/fake-chassis")
    finally:
        chk._open_tty = orig
        stop.set()
        a.close()
        b.close()

    total += 1
    if r.get("detected") == "navbot_stm32" and r.get("found") is False:
        ok += 1
        print("  PASS  端到端：静听认出对面是 STM32 底盘，且不误判为 ESP")
    else:
        print(f"  FAIL  端到端识别：detected={r.get('detected')}, found={r.get('found')}")

    total += 1
    if "STM32" in (r.get("diagnosis") or ""):
        ok += 1
        print("  PASS  诊断里点明「这个口是 STM32，不是 ESP」")
    else:
        print(f"  FAIL  诊断没说到点上：{r.get('diagnosis')!r}")

    print(f"\n身份识别：{ok}/{total} 通过")
    return 0 if ok == total else 1


# ---------------------------------------------------------------- B. 注入层
def test_injected(chk) -> int:
    """用 socketpair 冒充串口 fd，绕开 termios，验 probe_esp8266 的主流程。

    这一层任何环境都能跑（不需要 /dev/ptmx）。它盖住的是：
      端口占用检测 → 逐波特率试探 → 找到模块 → 逐条查询 → 解析 → found=True
    盖不住 termios 那 20 行（真串口才走），那部分交给 C 层 pty 验。
    """
    import socket

    a, b = socket.socketpair()
    a.setblocking(False)
    b.setblocking(False)
    stop = threading.Event()

    def responder():
        buf = b""
        while not stop.is_set():
            try:
                data = b.recv(512)
            except BlockingIOError:
                time.sleep(0.01)
                continue
            except OSError:
                return
            if not data:
                time.sleep(0.01)
                continue
            buf += data
            while b"\r\n" in buf:
                line, buf = buf.split(b"\r\n", 1)
                cmd = line.decode("utf-8", "replace").strip()
                if not cmd:
                    continue
                try:
                    b.sendall(REPLIES.get(cmd, "\r\nERROR\r\n").encode())
                except OSError:
                    return

    threading.Thread(target=responder, daemon=True).start()

    orig_open = chk._open_tty
    chk._open_tty = lambda port, baud: os.dup(a.fileno())
    try:
        r = chk.probe_esp8266(["/dev/fake-esp"], explicit="/dev/fake-esp")
    finally:
        chk._open_tty = orig_open
        stop.set()
        a.close()
        b.close()

    ok = 0
    total = 0

    # _open_tty 的异常路径：路径不存在、文件不是 tty，都必须返回 None 而不是抛异常
    for bad_path in ("/dev/does-not-exist-xyz", "/dev/null"):
        total += 1
        try:
            fd = chk._open_tty(bad_path, 115200)
            if fd is None:
                ok += 1
                print(f"  PASS  _open_tty({bad_path}) → None")
            else:
                os.close(fd)
                print(f"  FAIL  _open_tty({bad_path}) 返回了 fd")
        except Exception as e:  # noqa: BLE001
            print(f"  FAIL  _open_tty({bad_path}) 抛异常：{type(e).__name__}: {e}")

    checks = [("found", r.get("found"), True)]
    checks += [(k, r.get(k), v) for k, v in EXPECT_ONLINE.items()]
    for name, got, want in checks:
        total += 1
        hit = got == want
        ok += 1 if hit else 0
        print(f"  {'PASS' if hit else 'FAIL'}  {name} = {got!r}" + ("" if hit else f"（期望 {want!r}）"))

    # 静默端口：对面没人应答，应该判未发现并给出诊断，而不是假装成功
    a2, b2 = socket.socketpair()
    a2.setblocking(False)
    b2.setblocking(False)
    chk._open_tty = lambda port, baud: os.dup(a2.fileno())
    try:
        r2 = chk.probe_esp8266(["/dev/fake-silent"], explicit="/dev/fake-silent")
    finally:
        chk._open_tty = orig_open
        a2.close()
        b2.close()
    total += 1
    if r2.get("found") is False and r2.get("diagnosis"):
        ok += 1
        print("  PASS  静默端口判为未发现，并给出诊断")
    else:
        print(f"  FAIL  静默端口处理异常：found={r2.get('found')}, diag={r2.get('diagnosis')!r}")

    print(f"\n注入层：{ok}/{total} 通过")
    if ok != total:
        print("\n原始响应（排查用）：")
        for k, v in (r.get("raw") or {}).items():
            print(f"  [{k}] {v!r}")
        print(f"尝试记录：{r.get('attempts')}")
    return 0 if ok == total else 1


# ---------------------------------------------------------------- C. 收发层
def fake_esp(master_fd: int, stop: threading.Event) -> None:
    """从 pty 主端读 AT 指令，按 REPLIES 表回话。"""
    buf = b""
    while not stop.is_set():
        try:
            data = os.read(master_fd, 512)
        except OSError:
            return
        if not data:
            time.sleep(0.02)
            continue
        buf += data
        while b"\r\n" in buf:
            line, buf = buf.split(b"\r\n", 1)
            cmd = line.decode("utf-8", "replace").strip()
            if not cmd:
                continue
            try:
                os.write(master_fd, REPLIES.get(cmd, "\r\nERROR\r\n").encode())
            except OSError:
                return


def test_pty(chk) -> int:
    try:
        import pty
    except ImportError:
        print("SKIP：没有 pty 模块")
        return 77
    try:
        master, slave = pty.openpty()
    except OSError as e:
        print(f"SKIP：拿不到 pty（{e}）——容器里常见，不算失败")
        return 77

    port = os.ttyname(slave)
    stop = threading.Event()
    threading.Thread(target=fake_esp, args=(master, stop), daemon=True).start()

    try:
        print(f"\n假 ESP8266 挂在 {port}，跑完整探测……")
        t0 = time.time()
        r = chk.probe_esp8266([port], explicit=port)
        dt = time.time() - t0

        ok = 0
        total = 0
        for key, want in EXPECT_ONLINE.items():
            total += 1
            hit = r.get(key) == want
            ok += 1 if hit else 0
            print(f"  {'PASS' if hit else 'FAIL'}  {key} = {r.get(key)!r}")
        total += 1
        if r.get("found") is True:
            ok += 1
            print("  PASS  found = True")
        else:
            print("  FAIL  found 不是 True")

        # 静默端口：应该判未发现并给诊断，而不是假装成功
        m2, s2 = pty.openpty()
        silent = os.ttyname(s2)
        r2 = chk.probe_esp8266([silent], explicit=silent)
        total += 1
        if r2.get("found") is False and r2.get("diagnosis"):
            ok += 1
            print("  PASS  静默端口判为未发现，并给出诊断")
        else:
            print(f"  FAIL  静默端口处理异常：found={r2.get('found')}, diag={r2.get('diagnosis')!r}")
        os.close(m2)
        os.close(s2)

        print(f"\n收发层：{ok}/{total} 通过，耗时 {dt:.2f}s")
        return 0 if ok == total else 1
    finally:
        stop.set()
        os.close(master)
        os.close(slave)


def main() -> int:
    parse_only = "--parse-only" in sys.argv
    chk = load_module()

    print("=" * 60)
    print("A. 解析层（不需要硬件）")
    print("=" * 60)
    rc_a = test_parse(chk)

    if parse_only:
        return rc_a

    print()
    print("=" * 60)
    print("A2. 端口占用检测（/proc 扫描）")
    print("=" * 60)
    rc_a2 = test_port_busy(chk)

    print()
    print("=" * 60)
    print("A3. 串口身份识别（对面是底盘还是 WiFi 模块）")
    print("=" * 60)
    rc_a3 = test_classify_port(chk)

    print()
    print("=" * 60)
    print("B. 注入层（socketpair 冒充串口，验主流程）")
    print("=" * 60)
    rc_b = test_injected(chk)

    print()
    print("=" * 60)
    print("C. 收发层（pty 真 fd，验 termios）")
    print("=" * 60)
    rc_c = test_pty(chk)

    print()
    print("=" * 60)
    verdicts = [
        (rc_a, "A 解析层"),
        (rc_a2, "A2 占用检测"),
        (rc_a3, "A3 身份识别"),
        (rc_b, "B 注入层"),
    ]
    if rc_c != 77:
        verdicts.append((rc_c, "C 收发层"))
    for rc, name in verdicts:
        print(f"  {name}: {'通过' if rc == 0 else '失败'}")
    if rc_c == 77:
        print("  C 收发层: 跳过（本环境无 pty；termios 那段未验证，需在真机跑一次）")
    bad = [n for rc, n in verdicts if rc != 0]
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
