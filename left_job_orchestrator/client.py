#!/usr/bin/env python3
"""TCP/TLS client for left_job_orchestrator.service (public PC = client end).

Default: plain TCP + JSON line + token (see SOCKET_API.md). TLS optional via OPENARM_LEFT_JOB_TLS=1.

  python -m left_job_orchestrator.client status
  python -m left_job_orchestrator.client 1
  python -m left_job_orchestrator.client subscribe --host <robot-ip>
"""

from __future__ import annotations

import argparse
import json
import socket
import sys
import time

from . import protocol, tls_util


def _recv_line(sock: socket.socket, buf: bytearray) -> str | None:
    while True:
        nl = buf.find(b"\n")
        if nl >= 0:
            raw = bytes(buf[:nl])
            del buf[: nl + 1]
            return raw.decode("utf-8", errors="replace").rstrip("\r")
        chunk = sock.recv(4096)
        if not chunk:
            return None
        buf.extend(chunk)


def _send_line(sock: socket.socket, text: str) -> None:
    sock.sendall((text.rstrip("\r\n") + "\n").encode("utf-8"))


def _connect(host: str, port: int, timeout: float):
    sock = socket.create_connection((host, port), timeout=timeout)
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    if tls_util.tls_enabled():
        sock = tls_util.wrap_client_socket(sock, server_hostname=host)
    return sock


def _command_to_request(command: str, token: str | None) -> dict:
    low = command.strip().lower()
    if low in {"status", "list"}:
        return protocol.build_request("status", token=token)
    if low in {"subscribe", "sub"}:
        return protocol.build_request("subscribe", token=token)
    if low in {"unsubscribe", "unsub"}:
        return protocol.build_request("unsubscribe", token=token)
    if low in {"abort", "stop"}:
        return protocol.build_request("abort", token=token)
    if low in {"shutdown", "quit", "exit"}:
        return protocol.build_request("shutdown", token=token)
    if low.startswith("run|"):
        return protocol.build_request("run", job=command.split("|", 1)[1], token=token)
    if low.startswith("run "):
        return protocol.build_request("run", job=command.split(None, 1)[1], token=token)
    return protocol.build_request("run", job=command, token=token)


def _send(
    host: str,
    port: int,
    command: str,
    *,
    timeout: float,
    token: str | None,
) -> str:
    sock = _connect(host, port, timeout)
    buf = bytearray()
    try:
        sock.settimeout(timeout)
        req = _command_to_request(command, token)
        _send_line(sock, protocol.encode_line(req))
        while True:
            line = _recv_line(sock, buf)
            if line is None:
                raise ConnectionError("connection closed before reply")
            try:
                obj = json.loads(line)
            except json.JSONDecodeError:
                if line.startswith("event|"):
                    continue
                return line
            if obj.get("event"):
                continue
            return protocol.encode_line(obj)
    finally:
        try:
            sock.close()
        except OSError:
            pass


def run_subscribe(
    host: str,
    port: int,
    *,
    timeout: float,
    heartbeat_miss_s: float,
    token: str | None,
) -> int:
    backoff = 1.0
    while True:
        sock = None
        buf = bytearray()
        try:
            print(f"[left-job-client] connect {host}:{port} tls={tls_util.tls_enabled()}", flush=True)
            sock = _connect(host, port, timeout)
            sock.settimeout(1.0)
            req = protocol.build_request("subscribe", token=token)
            _send_line(sock, protocol.encode_line(req))
            ack = _recv_line(sock, buf)
            if ack is None:
                raise ConnectionError("no subscribe ack")
            print(ack, flush=True)
            backoff = 1.0
            last_event = time.monotonic()
            while True:
                try:
                    line = _recv_line(sock, buf)
                except socket.timeout:
                    if time.monotonic() - last_event > heartbeat_miss_s:
                        print("[left-job-client] heartbeat miss; reconnecting", flush=True)
                        raise ConnectionError("heartbeat miss")
                    continue
                if line is None:
                    raise ConnectionError("disconnected")
                last_event = time.monotonic()
                print(line, flush=True)
        except KeyboardInterrupt:
            print("[left-job-client] Ctrl+C; unsubscribe", flush=True)
            if sock is not None:
                try:
                    sock.settimeout(timeout)
                    req = protocol.build_request("unsubscribe", token=token)
                    _send_line(sock, protocol.encode_line(req))
                    line = _recv_line(sock, buf)
                    if line:
                        print(line, flush=True)
                except (socket.timeout, OSError, ConnectionError):
                    pass
            return 0
        except (OSError, ConnectionError) as exc:
            print(f"[left-job-client] {exc}; retry in {backoff:.0f}s", file=sys.stderr)
            time.sleep(backoff)
            backoff = min(backoff * 2.0, 15.0)
        finally:
            if sock is not None:
                try:
                    sock.close()
                except OSError:
                    pass


def main() -> int:
    parser = argparse.ArgumentParser(description="Send a command to left job orchestrator")
    parser.add_argument(
        "command",
        nargs="?",
        default="status",
        help="1|2|3|status|subscribe|abort|shutdown|run|N",
    )
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=47925)
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument(
        "--token",
        default=None,
        help="API token (default: OPENARM_LEFT_JOB_API_TOKEN or openarm-job-api)",
    )
    parser.add_argument(
        "--heartbeat-miss-s",
        type=float,
        default=8.0,
        help="subscribe mode: reconnect if no line for this many seconds",
    )
    args = parser.parse_args()
    token = args.token if args.token is not None else protocol.resolve_api_token()

    if args.command.strip().lower() in {"subscribe", "sub"}:
        return run_subscribe(
            args.host,
            args.port,
            timeout=args.timeout,
            heartbeat_miss_s=float(args.heartbeat_miss_s),
            token=token,
        )

    try:
        print(_send(args.host, args.port, args.command, timeout=args.timeout, token=token))
        return 0
    except socket.timeout:
        print(
            f"[left-job-client] timeout waiting reply from {args.host}:{args.port}",
            file=sys.stderr,
        )
        return 1
    except (OSError, ConnectionError) as exc:
        print(f"[left-job-client] {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
