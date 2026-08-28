#!/usr/bin/env python3
"""UDP client for skewer_job_service.py."""

from __future__ import annotations

import argparse
import socket
import sys


def main() -> int:
    parser = argparse.ArgumentParser(description="Send a command to skewer job service")
    parser.add_argument("command", nargs="?", default="status", help="1|2|3|status|abort|shutdown|run|N")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=47925)
    parser.add_argument("--timeout", type=float, default=2.0)
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(args.timeout)
    try:
        sock.sendto(args.command.encode("utf-8"), (args.host, args.port))
        data, _ = sock.recvfrom(4096)
        print(data.decode("utf-8", errors="replace"))
        return 0
    except socket.timeout:
        print(f"[skewer-job-client] timeout waiting reply from {args.host}:{args.port}", file=sys.stderr)
        return 1
    finally:
        sock.close()


if __name__ == "__main__":
    raise SystemExit(main())
