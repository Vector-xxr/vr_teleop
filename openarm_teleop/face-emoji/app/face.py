# -*- coding: utf-8 -*-
"""
Face / Emoji socket 客户端

按照 doc/api.md 的协议，通过 TCP socket 向机器人发送 JSON 控制指令。

服务器:  192.168.2.199:22345
报文格式: JSON, 例如
    {"type": "expression", "intent": "sleepy", "tts": "你好呀"}

用法:
    # 发送一个表情
    python face.py --intent happy --tts "你好呀"

    # 发送一个场景
    python face.py --type scene --intent wave

    # 指定服务器地址/端口
    python face.py --host 192.168.2.199 --port 22345 --intent love

    # 交互模式 (循环输入指令)
    python face.py -i
"""

import argparse
import json
import socket
import sys

DEFAULT_HOST = "192.168.2.199"
DEFAULT_PORT = 22345
ENCODING = "utf-8"

# 表情 (type = expression)
EXPRESSION_INTENTS = {
    "neutral": "平静",
    "happy": "开心",
    "angry": "生气",
    "sad": "难过",
    "surprised": "惊讶",
    "sleepy": "困倦",
    "love": "喜欢",
    "think": "思考",
    "cool": "调皮",
    "dizzy": "眩晕",
}

# 场景 (type = scene)
SCENE_INTENTS = {
    "approach": "有人靠近",
    "known": "识别熟人",
    "listen": "开始聆听",
    "talk": "开始说话",
    "wave": "挥手打招呼",
    "idle": "进入待机",
    "wake": "模拟唤醒",
    "multi": "多声源",
}

VALID_INTENTS = {
    "expression": EXPRESSION_INTENTS,
    "scene": SCENE_INTENTS,
}


class FaceClient:
    """与机器人表情/场景服务的 socket 通讯客户端。"""

    def __init__(self, host=DEFAULT_HOST, port=DEFAULT_PORT, timeout=5.0):
        self.host = host
        self.port = port
        self.timeout = timeout
        self.sock = None

    def connect(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(self.timeout)
        self.sock.connect((self.host, self.port))
        return self

    def close(self):
        if self.sock is not None:
            try:
                self.sock.close()
            finally:
                self.sock = None

    def __enter__(self):
        return self.connect()

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()

    def send(self, payload):
        """发送一个 dict (会被序列化为 JSON) 并尝试读取返回。"""
        if self.sock is None:
            raise RuntimeError("socket 未连接, 请先调用 connect()")

        data = json.dumps(payload, ensure_ascii=False).encode(ENCODING)
        # 以换行作为报文分隔, 便于服务端按行解析
        self.sock.sendall(data + b"\n")

        # 尝试读取响应 (服务端可能不回包)
        try:
            resp = self.sock.recv(4096)
            if resp:
                return resp.decode(ENCODING, errors="replace").strip()
        except socket.timeout:
            return None
        return None


def build_payload(msg_type, intent, tts=None):
    """构造并校验请求报文。"""
    if msg_type not in VALID_INTENTS:
        raise ValueError(
            "未知 type: %r, 可选: %s" % (msg_type, ", ".join(VALID_INTENTS))
        )

    valid = VALID_INTENTS[msg_type]
    if intent not in valid:
        raise ValueError(
            "type=%s 下未知 intent: %r, 可选: %s"
            % (msg_type, intent, ", ".join(valid))
        )

    payload = {"type": msg_type, "intent": intent}
    if tts:
        payload["tts"] = tts
    return payload


def send_once(host, port, msg_type, intent, tts=None, timeout=5.0):
    """便捷函数: 连接 -> 发送一条 -> 关闭。"""
    payload = build_payload(msg_type, intent, tts)
    with FaceClient(host, port, timeout) as client:
        resp = client.send(payload)
    return payload, resp


def _print_intents():
    print("[表情] type = expression")
    for k, v in EXPRESSION_INTENTS.items():
        print("    %-10s %s" % (k, v))
    print("[场景] type = scene")
    for k, v in SCENE_INTENTS.items():
        print("    %-10s %s" % (k, v))


def interactive_loop(host, port, timeout):
    print("交互模式: 输入  <type> <intent> [tts]  发送; 输入 'list' 查看取值; 'q' 退出。")
    print("示例: expression happy 你好呀")
    with FaceClient(host, port, timeout) as client:
        print("已连接 %s:%d" % (host, port))
        while True:
            try:
                line = input("> ").strip()
            except (EOFError, KeyboardInterrupt):
                print()
                break
            if not line:
                continue
            if line in ("q", "quit", "exit"):
                break
            if line == "list":
                _print_intents()
                continue

            parts = line.split(None, 2)
            if len(parts) < 2:
                print("格式错误, 需要: <type> <intent> [tts]")
                continue
            msg_type, intent = parts[0], parts[1]
            tts = parts[2] if len(parts) > 2 else None
            try:
                payload = build_payload(msg_type, intent, tts)
            except ValueError as e:
                print("参数错误:", e)
                continue
            resp = client.send(payload)
            print("已发送:", json.dumps(payload, ensure_ascii=False))
            if resp is not None:
                print("返回:", resp)


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="机器人表情/场景 socket 客户端 (见 doc/api.md)"
    )
    parser.add_argument("--host", default=DEFAULT_HOST, help="服务器 IP (默认 %s)" % DEFAULT_HOST)
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help="端口 (默认 %d)" % DEFAULT_PORT)
    parser.add_argument("--type", dest="msg_type", default="expression",
                        choices=list(VALID_INTENTS), help="报文类型 (默认 expression)")
    parser.add_argument("--intent", help="意图取值, 见 --list")
    parser.add_argument("--tts", default=None, help="语音播报文本 (可选)")
    parser.add_argument("--timeout", type=float, default=5.0, help="连接/读取超时秒数")
    parser.add_argument("-i", "--interactive", action="store_true", help="进入交互模式")
    parser.add_argument("-l", "--list", action="store_true", help="列出所有 intent 取值")
    args = parser.parse_args(argv)

    if args.list:
        _print_intents()
        return 0

    if args.interactive:
        try:
            interactive_loop(args.host, args.port, args.timeout)
        except (socket.timeout, OSError) as e:
            print("连接失败:", e, file=sys.stderr)
            return 1
        return 0

    if not args.intent:
        parser.error("请提供 --intent (或使用 -i 交互模式 / -l 查看取值)")

    try:
        payload = build_payload(args.msg_type, args.intent, args.tts)
    except ValueError as e:
        print("参数错误:", e, file=sys.stderr)
        return 2

    try:
        _, resp = send_once(args.host, args.port, args.msg_type,
                            args.intent, args.tts, args.timeout)
    except (socket.timeout, OSError) as e:
        print("发送失败:", e, file=sys.stderr)
        return 1

    print("已发送:", json.dumps(payload, ensure_ascii=False))
    if resp is not None:
        print("返回:", resp)
    return 0


if __name__ == "__main__":
    sys.exit(main())
