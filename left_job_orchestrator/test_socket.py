#!/usr/bin/env python3
"""Interactive / one-shot TCP JSON socket test for left_job_orchestrator.

Examples:
  # Against robot on LAN (service must be running)
  python -m left_job_orchestrator.test_socket --host 192.168.2.100 status
  python -m left_job_orchestrator.test_socket --host 192.168.2.100 run --job 1
  python -m left_job_orchestrator.test_socket --host 192.168.2.100 interactive

  # With shared token + HMAC (match server OPENARM_LEFT_JOB_API_TOKEN)
  export OPENARM_LEFT_JOB_API_TOKEN=my-secret
  python -m left_job_orchestrator.test_socket --host 192.168.2.100 status

Protocol: plain TCP (TLS default OFF) + one JSON object per line.
Auth: shared token (default ON, token=openarm-job-api). sig not required.
Enable TLS later: OPENARM_LEFT_JOB_TLS=1
Override token: OPENARM_LEFT_JOB_API_TOKEN=...
"""

from __future__ import annotations

import argparse
import json
import os
import socket
import sys
import time

from . import protocol


def _connect(host: str, port: int, timeout: float):
    from . import tls_util

    sock = socket.create_connection((host, port), timeout=timeout)
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    if tls_util.tls_enabled():
        sock = tls_util.wrap_client_socket(sock, server_hostname=host)
    return sock


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


def exchange(
    host: str,
    port: int,
    req: dict,
    *,
    timeout: float,
    verbose: bool,
) -> dict:
    line = protocol.encode_line(req)
    if verbose:
        print(f"→ {host}:{port}  {line}", flush=True)
    sock = _connect(host, port, timeout)
    buf = bytearray()
    try:
        sock.settimeout(timeout)
        _send_line(sock, line)
        while True:
            reply = _recv_line(sock, buf)
            if reply is None:
                raise ConnectionError("connection closed before reply")
            if verbose:
                print(f"← {reply}", flush=True)
            # Skip async events if any arrive early.
            try:
                obj = json.loads(reply)
            except json.JSONDecodeError:
                return {"ok": False, "error": "non_json_reply", "raw": reply}
            if obj.get("event"):
                continue
            return obj
    finally:
        try:
            sock.close()
        except OSError:
            pass


def cmd_to_request(command: str, job: str | None, token: str | None) -> dict:
    c = command.strip().lower()
    if c in {"status", "list", "subscribe", "sub", "unsubscribe", "unsub", "abort", "stop", "shutdown"}:
        mapped = {
            "list": "status",
            "sub": "subscribe",
            "unsub": "unsubscribe",
            "stop": "abort",
        }.get(c, c)
        return protocol.build_request(mapped, token=token)
    if c in {"run", "job"}:
        if not job:
            raise SystemExit("--job is required for run")
        return protocol.build_request("run", job=job, token=token)
    # Bare number => run that job
    if c.isdigit() or c.startswith("job"):
        return protocol.build_request("run", job=c, token=token)
    raise SystemExit(f"unknown command: {command}")


def run_subscribe(host: str, port: int, *, timeout: float, token: str | None, verbose: bool) -> int:
    backoff = 1.0
    while True:
        sock = None
        buf = bytearray()
        try:
            print(f"[test-socket] connect {host}:{port}", flush=True)
            sock = _connect(host, port, timeout)
            sock.settimeout(1.0)
            req = protocol.build_request("subscribe", token=token)
            line = protocol.encode_line(req)
            if verbose:
                print(f"→ {line}", flush=True)
            _send_line(sock, line)
            ack = _recv_line(sock, buf)
            if ack is None:
                raise ConnectionError("no subscribe ack")
            print(f"← {ack}", flush=True)
            backoff = 1.0
            last = time.monotonic()
            while True:
                try:
                    msg = _recv_line(sock, buf)
                except socket.timeout:
                    if time.monotonic() - last > 8.0:
                        raise ConnectionError("heartbeat miss")
                    continue
                if msg is None:
                    raise ConnectionError("disconnected")
                last = time.monotonic()
                print(f"← {msg}", flush=True)
        except KeyboardInterrupt:
            print("[test-socket] Ctrl+C", flush=True)
            if sock is not None:
                try:
                    sock.settimeout(timeout)
                    req = protocol.build_request("unsubscribe", token=token)
                    _send_line(sock, protocol.encode_line(req))
                    msg = _recv_line(sock, buf)
                    if msg:
                        print(f"← {msg}", flush=True)
                except (OSError, ConnectionError, socket.timeout):
                    pass
            return 0
        except (OSError, ConnectionError) as exc:
            print(f"[test-socket] {exc}; retry in {backoff:.0f}s", file=sys.stderr)
            time.sleep(backoff)
            backoff = min(backoff * 2.0, 15.0)
        finally:
            if sock is not None:
                try:
                    sock.close()
                except OSError:
                    pass


def run_interactive(host: str, port: int, *, timeout: float, token: str | None, verbose: bool) -> int:
    print(
        f"[test-socket] interactive → {host}:{port}\n"
        "  commands: status | run <job> | abort | subscribe | shutdown | quit\n"
        "  raw JSON:  {...}\n",
        flush=True,
    )
    while True:
        try:
            line = input("job> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            return 0
        if not line:
            continue
        if line.lower() in {"quit", "exit", "q"}:
            return 0
        try:
            if line.startswith("{"):
                req = json.loads(line)
            else:
                parts = line.split()
                head = parts[0].lower()
                job = parts[1] if len(parts) > 1 else None
                if head == "run" and job is None:
                    print("usage: run <job_id>", file=sys.stderr)
                    continue
                if head == "subscribe":
                    return run_subscribe(host, port, timeout=timeout, token=token, verbose=verbose)
                req = cmd_to_request(head if head != "run" else "run", job, token)
            resp = exchange(host, port, req, timeout=timeout, verbose=verbose)
            print(json.dumps(resp, ensure_ascii=False, indent=2), flush=True)
        except Exception as exc:  # noqa: BLE001
            print(f"[test-socket] error: {exc}", file=sys.stderr)


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description="Test left_job_orchestrator TCP JSON API")
    p.add_argument(
        "command",
        nargs="?",
        default="status",
        help="status|run|subscribe|abort|shutdown|interactive|1|2",
    )
    p.add_argument("--host", default=os.environ.get("OPENARM_LEFT_JOB_HOST", "127.0.0.1"))
    p.add_argument("--port", type=int, default=int(os.environ.get("OPENARM_SKEWER_JOB_LISTEN_PORT", "47925")))
    p.add_argument("--job", default=None, help="job id for run (e.g. 1)")
    p.add_argument(
        "--token",
        default=os.environ.get("OPENARM_LEFT_JOB_API_TOKEN", "") or None,
        help="shared API token (HMAC + token field)",
    )
    p.add_argument("--timeout", type=float, default=5.0)
    p.add_argument("-v", "--verbose", action="store_true", help="print raw request/response lines")
    args = p.parse_args(argv)

    cmd = args.command.strip().lower()
    if cmd in {"interactive", "i", "repl"}:
        return run_interactive(
            args.host, args.port, timeout=args.timeout, token=args.token, verbose=args.verbose
        )
    if cmd in {"subscribe", "sub"}:
        return run_subscribe(
            args.host, args.port, timeout=args.timeout, token=args.token, verbose=args.verbose
        )

    try:
        req = cmd_to_request(cmd, args.job, args.token)
        resp = exchange(args.host, args.port, req, timeout=args.timeout, verbose=True)
        print(json.dumps(resp, ensure_ascii=False, indent=2))
        return 0 if resp.get("ok") else 2
    except socket.timeout:
        print(f"[test-socket] timeout {args.host}:{args.port}", file=sys.stderr)
        return 1
    except (OSError, ConnectionError) as exc:
        print(f"[test-socket] {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
