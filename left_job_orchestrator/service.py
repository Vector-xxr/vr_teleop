#!/usr/bin/env python3
"""Persistent left-arm job service.

Lifecycle:
  1) Start dual replay server once (VR motor config via openarm_vr_compat_env.sh)
  2) Play recordings/tasks/steady_action_on_table.csv once
  3) Idle; accept TCP job clients on :47925 (newline-framed commands)
  4) Aux thread pushes status on subscribed TCP connections
  5) On 1/2/job1/... run pipeline:
       - if jobN/visual_grasp*.txt exists:
           action1 -> quintic to begin -> tcp_forward_grasp -> quintic -> action2
       - else:
           action1 -> action2 (trajectory only)
"""

from __future__ import annotations

import argparse
import json
import socket
import ssl
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path

from . import defaults, paths, protocol, tls_util
from .flow_loader import build_flow_args
from .pipeline import preflight_job, run_job
from .replay import install_cleanup_handlers, shutdown_server, start_server_and_steady


def log(msg: str) -> None:
    print(f"[left-job] {msg}", flush=True)


@dataclass(eq=False)
class ClientConn:
    sock: socket.socket
    addr: tuple[str, int]
    write_lock: threading.Lock = field(default_factory=threading.Lock)
    subscribed: bool = False
    closed: bool = False
    prefer_json: bool = False
    recv_buf: bytearray = field(default_factory=bytearray)


@dataclass
class ServiceState:
    lock: threading.Lock = field(default_factory=threading.Lock)
    busy: bool = False
    current_job: str | None = None
    last_result: str = "idle"
    last_error: str = ""
    startup_done: bool = False
    abort_requested: bool = False
    stop_service: bool = False
    server_proc: object | None = None
    # TCP clients that sent "subscribe" (same connection receives event|...).
    subscribers: set[ClientConn] = field(default_factory=set)
    status_epoch: int = 0
    status_event: threading.Event = field(default_factory=threading.Event)


def _bump_status(state: ServiceState) -> None:
    with state.lock:
        state.status_epoch += 1
        state.status_event.set()


def status_dict(jobs_root: Path, state: ServiceState) -> dict:
    jobs = paths.discover_jobs(jobs_root)
    with state.lock:
        return {
            "busy": int(state.busy),
            "job": state.current_job,
            "startup_done": int(state.startup_done),
            "last": state.last_result,
            "error": state.last_error or None,
            "jobs": jobs,
            "subs": len(state.subscribers),
        }


def status_payload(jobs_root: Path, state: ServiceState) -> str:
    st = status_dict(jobs_root, state)
    jobs = ",".join(st["jobs"]) or "-"
    return (
        f"ok|busy={st['busy']}|job={st['job'] or '-'}|"
        f"startup_done={st['startup_done']}|last={st['last']}|"
        f"error={st['error'] or '-'}|jobs={jobs}|"
        f"subs={st['subs']}"
    )


def send_line(conn: ClientConn, text: str) -> bool:
    if conn.closed:
        return False
    payload = (text.rstrip("\r\n") + "\n").encode("utf-8")
    try:
        with conn.write_lock:
            conn.sock.sendall(payload)
        return True
    except OSError:
        return False


def send_response(conn: ClientConn, resp: dict, *, as_json: bool) -> bool:
    if as_json or conn.prefer_json:
        return send_line(conn, protocol.encode_line(resp))
    # Legacy pipe replies for plain-text clients.
    if not resp.get("ok"):
        err = resp.get("error") or "error"
        if err == "busy":
            return send_line(conn, f"busy|job={resp.get('job')}")
        return send_line(conn, f"error|{err}")
    cmd = str(resp.get("cmd") or "")
    if cmd == "status" or resp.get("event") == "status":
        jobs = resp.get("jobs") or []
        jobs_s = ",".join(str(j) for j in jobs) if isinstance(jobs, list) else str(jobs)
        return send_line(
            conn,
            f"ok|busy={int(resp.get('busy', 0))}|job={resp.get('job') or '-'}|"
            f"startup_done={int(resp.get('startup_done', 0))}|"
            f"last={resp.get('last') or '-'}|error={resp.get('error') or '-'}|"
            f"jobs={jobs_s or '-'}|subs={int(resp.get('subs', 0))}",
        )
    if cmd == "subscribe":
        return send_line(
            conn,
            f"ok|subscribed|from={resp.get('from')}|subs={resp.get('subs')}",
        )
    if cmd == "unsubscribe":
        return send_line(
            conn,
            f"ok|unsubscribed|from={resp.get('from')}|subs={resp.get('subs')}",
        )
    if cmd == "shutdown":
        return send_line(conn, "ok|shutdown_requested")
    if cmd == "abort":
        return send_line(conn, "ok|abort_requested")
    if cmd == "run":
        return send_line(
            conn,
            f"accepted|job={resp.get('job')}|mode={resp.get('mode')}|"
            f"from={resp.get('from')}",
        )
    return send_line(conn, protocol.encode_line(resp))


def recv_line(conn: ClientConn, *, timeout: float | None = None) -> str | None:
    """Read one newline-terminated line. Returns None on disconnect."""
    if timeout is not None:
        conn.sock.settimeout(timeout)
    while True:
        nl = conn.recv_buf.find(b"\n")
        if nl >= 0:
            raw = bytes(conn.recv_buf[:nl])
            del conn.recv_buf[: nl + 1]
            return raw.decode("utf-8", errors="replace").rstrip("\r")
        try:
            chunk = conn.sock.recv(4096)
        except socket.timeout:
            raise
        except OSError:
            return None
        if not chunk:
            return None
        conn.recv_buf.extend(chunk)


def close_conn(conn: ClientConn, state: ServiceState) -> None:
    with state.lock:
        state.subscribers.discard(conn)
        already = conn.closed
        conn.closed = True
        conn.subscribed = False
    if already:
        return
    try:
        conn.sock.shutdown(socket.SHUT_RDWR)
    except OSError:
        pass
    try:
        conn.sock.close()
    except OSError:
        pass


def _dispatch_request(
    req: dict,
    conn: ClientConn,
    *,
    jobs_root: Path,
    state: ServiceState,
    base_args,
    skip_action2_on_grasp_fail: bool,
) -> dict:
    cmd = str(req.get("cmd") or "").strip().lower()
    req_id = str(req.get("id") or "")
    addr = conn.addr
    from_s = f"{addr[0]}:{addr[1]}"

    auth_err = protocol.auth_check(req)
    if auth_err:
        return protocol.error_response(cmd=cmd, req_id=req_id, error=auth_err)

    if cmd in {"status", "list"}:
        return protocol.ok_response(cmd="status", req_id=req_id, **status_dict(jobs_root, state))

    if cmd in {"subscribe", "sub"}:
        with state.lock:
            conn.subscribed = True
            state.subscribers.add(conn)
            n = len(state.subscribers)
        _bump_status(state)
        log(f"subscribe +{from_s} (n={n})")
        return protocol.ok_response(
            cmd="subscribe",
            req_id=req_id,
            from_=None,
            **{"from": from_s, "subs": n},
        )

    if cmd in {"unsubscribe", "unsub"}:
        with state.lock:
            conn.subscribed = False
            state.subscribers.discard(conn)
            n = len(state.subscribers)
        log(f"unsubscribe -{from_s} (n={n})")
        return protocol.ok_response(
            cmd="unsubscribe",
            req_id=req_id,
            **{"from": from_s, "subs": n},
        )

    if cmd in {"shutdown", "quit", "exit"}:
        state.stop_service = True
        state.abort_requested = True
        _bump_status(state)
        return protocol.ok_response(cmd="shutdown", req_id=req_id)

    if cmd in {"abort", "stop"}:
        state.abort_requested = True
        _bump_status(state)
        return protocol.ok_response(cmd="abort", req_id=req_id)

    if cmd != "run":
        return protocol.error_response(cmd=cmd, req_id=req_id, error="bad_command")

    job_raw = str(req.get("job") or "").strip()
    job_id = paths.normalize_job_id(job_raw)
    if not job_id:
        return protocol.error_response(cmd="run", req_id=req_id, error="bad_job")

    job_folder = paths.job_dir(jobs_root, job_id)
    if not job_folder.is_dir():
        return protocol.error_response(
            cmd="run",
            req_id=req_id,
            error="unknown_job",
            job=job_id,
            path=str(job_folder),
        )
    try:
        steps = preflight_job(job_folder)
    except (FileNotFoundError, ValueError) as exc:
        return protocol.error_response(
            cmd="run", req_id=req_id, error="invalid_job_sequence", detail=str(exc)
        )

    mode = "sequence:" + "->".join(step.label for step in steps)

    with state.lock:
        if state.busy:
            return protocol.error_response(
                cmd="run", req_id=req_id, error="busy", job=state.current_job
            )
        if not state.startup_done:
            return protocol.error_response(cmd="run", req_id=req_id, error="startup_not_done")
        if state.server_proc is None:
            return protocol.error_response(cmd="run", req_id=req_id, error="no_server")
        state.busy = True
        state.current_job = job_id
        state.abort_requested = False
        state.last_result = "running"
        state.last_error = ""
    _bump_status(state)

    def worker() -> None:
        try:
            assert state.server_proc is not None
            log(f"job {job_id}: start ({mode})")
            run_job(
                state.server_proc,
                base_args,
                jobs_root,
                job_id,
                abort_check=lambda: state.abort_requested or state.stop_service,
                skip_action2_on_grasp_fail=skip_action2_on_grasp_fail,
            )
            with state.lock:
                state.last_result = "success"
                state.last_error = ""
            log(f"job {job_id}: success ({mode})")
        except Exception as exc:  # noqa: BLE001
            with state.lock:
                state.last_result = "failed"
                state.last_error = str(exc)
            log(f"job {job_id}: FAILED: {exc}")
        finally:
            with state.lock:
                state.busy = False
                state.current_job = None
            _bump_status(state)

    threading.Thread(target=worker, name=f"job-{job_id}", daemon=True).start()
    return protocol.ok_response(
        cmd="run",
        req_id=req_id,
        accepted=True,
        job=job_id,
        mode=mode,
        **{"from": from_s},
    )


def handle_command(
    text: str,
    conn: ClientConn,
    *,
    jobs_root: Path,
    state: ServiceState,
    base_args,
    skip_action2_on_grasp_fail: bool,
) -> tuple[dict, bool]:
    """Parse one line; return (response_dict, reply_as_json)."""
    raw = text.strip()
    if not raw:
        return protocol.error_response(error="empty"), conn.prefer_json

    as_json = protocol.looks_like_json_line(raw)
    if as_json:
        conn.prefer_json = True
        try:
            req = protocol.parse_line(raw)
        except json.JSONDecodeError as exc:
            return protocol.error_response(error=f"bad_json:{exc}"), True
        if int(req.get("v", protocol.PROTOCOL_VERSION)) != protocol.PROTOCOL_VERSION:
            return protocol.error_response(
                cmd=str(req.get("cmd") or ""),
                req_id=str(req.get("id") or ""),
                error="bad_version",
            ), True
    else:
        if protocol.resolve_require_auth():
            return protocol.error_response(error="auth_required_use_json"), False
        req = protocol.legacy_to_request(raw)
        if req is None:
            return protocol.error_response(error="empty"), False

    resp = _dispatch_request(
        req,
        conn,
        jobs_root=jobs_root,
        state=state,
        base_args=base_args,
        skip_action2_on_grasp_fail=skip_action2_on_grasp_fail,
    )
    return resp, as_json or conn.prefer_json


def _subscriber_push_loop(
    jobs_root: Path,
    state: ServiceState,
    *,
    heartbeat_s: float,
) -> None:
    """Aux thread: push status lines to subscribed TCP clients on change + heartbeat."""
    last_epoch = -1
    last_hb = 0.0
    log(f"subscriber push thread on (heartbeat={heartbeat_s:.1f}s)")
    while not state.stop_service:
        state.status_event.wait(timeout=min(0.5, max(0.1, heartbeat_s)))
        state.status_event.clear()
        if state.stop_service:
            break
        now = time.monotonic()
        with state.lock:
            epoch = state.status_epoch
            subs = [c for c in state.subscribers if c.subscribed and not c.closed]
        need = epoch != last_epoch or (now - last_hb) >= heartbeat_s
        if not need or not subs:
            continue
        st = status_dict(jobs_root, state)
        json_event = protocol.ok_response(cmd="status", event="status", **st)
        pipe_event = "event|" + status_payload(jobs_root, state)
        dead: list[ClientConn] = []
        for conn in subs:
            ok = (
                send_response(conn, json_event, as_json=True)
                if conn.prefer_json
                else send_line(conn, pipe_event)
            )
            if not ok:
                dead.append(conn)
        for conn in dead:
            close_conn(conn, state)
        last_epoch = epoch
        last_hb = now
    log("subscriber push thread stopped")


def _client_loop(
    conn: ClientConn,
    *,
    jobs_root: Path,
    state: ServiceState,
    base_args,
    skip_action2_on_grasp_fail: bool,
) -> None:
    addr = conn.addr
    log(f"client connected {addr[0]}:{addr[1]}")
    try:
        while not state.stop_service and not conn.closed:
            try:
                text = recv_line(conn, timeout=0.5)
            except socket.timeout:
                continue
            if text is None:
                break
            if not text.strip():
                continue
            if text.startswith("event|") or text.lstrip().startswith('{"event"'):
                continue
            log(f"cmd from {addr[0]}:{addr[1]}: {text!r}")
            try:
                resp, as_json = handle_command(
                    text,
                    conn,
                    jobs_root=jobs_root,
                    state=state,
                    base_args=base_args,
                    skip_action2_on_grasp_fail=skip_action2_on_grasp_fail,
                )
            except Exception as exc:  # noqa: BLE001
                resp = protocol.error_response(error=str(exc))
                as_json = conn.prefer_json
                log(f"handler error: {exc}")
            if not send_response(conn, resp, as_json=as_json):
                break
            if state.stop_service:
                break
    finally:
        close_conn(conn, state)
        log(f"client disconnected {addr[0]}:{addr[1]}")


def serve(
    *,
    jobs_root: Path,
    listen_port: int,
    steady_csv: Path,
    skip_action2_on_grasp_fail: bool,
    subscribe_heartbeat_s: float,
) -> int:
    install_cleanup_handlers()
    paths.OUTPUT_ROOT.mkdir(parents=True, exist_ok=True)
    paths.RUNTIME_ROOT.mkdir(parents=True, exist_ok=True)

    if not steady_csv.is_file():
        raise FileNotFoundError(f"startup CSV missing: {steady_csv}")
    if not paths.LEFT_CONFIG.is_file():
        raise FileNotFoundError(f"left config missing: {paths.LEFT_CONFIG}")
    begin_csv = defaults.resolve_observe_csv()
    if not begin_csv.is_file():
        raise FileNotFoundError(f"begin.csv missing: {begin_csv}")
    if not paths.DEFAULT_GRIPPER_GATE_JSON.is_file():
        raise FileNotFoundError(
            f"gripper gate missing: {paths.DEFAULT_GRIPPER_GATE_JSON} "
            "(run tcp_forward_grasp/run_calibrate_gate.sh)"
        )

    base_args = build_flow_args(output_dir=paths.OUTPUT_ROOT)
    base_args.steady_replay = steady_csv
    base_args.observe_replay = begin_csv
    base_args.execute = True

    state = ServiceState()
    startup_dir = paths.OUTPUT_ROOT / "startup"
    startup_dir.mkdir(parents=True, exist_ok=True)

    log(f"jobs_root={jobs_root}")
    log(f"steady={steady_csv}")
    log(f"begin_default={begin_csv}")
    log(f"grasp=tcp_forward_grasp gate={paths.DEFAULT_GRIPPER_GATE_JSON}")
    log("motor/gripper = VR teleop (openarm_vr_compat_env via replay server)")
    log(f"available jobs: {paths.discover_jobs(jobs_root)}")
    for jid in paths.discover_jobs(jobs_root):
        folder = paths.job_dir(jobs_root, jid)
        try:
            steps = preflight_job(folder)
            log(f"  job{jid}: " + " -> ".join(step.label for step in steps))
        except (FileNotFoundError, ValueError) as exc:
            log(f"  job{jid}: INVALID ({exc})")
    log("starting dual replay + playing steady once")
    state.server_proc = start_server_and_steady(base_args, startup_dir, steady_csv)
    state.startup_done = True
    _bump_status(state)
    use_tls = tls_util.tls_enabled()
    if use_tls:
        tls_util.ensure_self_signed_cert()
    log(
        f"ready. {'TLS' if use_tls else 'TCP'} :{listen_port} "
        f"auth={'on' if protocol.resolve_require_auth() else 'off'} "
        f"token={'set' if protocol.resolve_api_token() else 'none'}; "
        "JSON cmds=status|run|subscribe|abort|shutdown"
    )

    listen = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    listen.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    listen.bind(("0.0.0.0", listen_port))
    listen.listen(16)
    listen.settimeout(0.5)

    push_thread = threading.Thread(
        target=_subscriber_push_loop,
        name="job-subscribe-push",
        args=(jobs_root, state),
        kwargs={"heartbeat_s": float(subscribe_heartbeat_s)},
        daemon=True,
    )
    push_thread.start()

    client_threads: list[threading.Thread] = []
    try:
        while not state.stop_service:
            try:
                client_sock, addr = listen.accept()
            except socket.timeout:
                continue
            except OSError:
                if state.stop_service:
                    break
                raise
            client_sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            if use_tls:
                try:
                    client_sock = tls_util.wrap_server_socket(client_sock)
                except ssl.SSLError as exc:
                    log(f"TLS handshake failed from {addr[0]}:{addr[1]}: {exc}")
                    try:
                        client_sock.close()
                    except OSError:
                        pass
                    continue
            conn = ClientConn(sock=client_sock, addr=(addr[0], int(addr[1])))
            t = threading.Thread(
                target=_client_loop,
                name=f"job-client-{addr[0]}:{addr[1]}",
                args=(conn,),
                kwargs={
                    "jobs_root": jobs_root,
                    "state": state,
                    "base_args": base_args,
                    "skip_action2_on_grasp_fail": skip_action2_on_grasp_fail,
                },
                daemon=True,
            )
            t.start()
            client_threads.append(t)
            client_threads = [x for x in client_threads if x.is_alive()]
    finally:
        state.stop_service = True
        state.status_event.set()
        try:
            listen.close()
        except OSError:
            pass
        with state.lock:
            subs = list(state.subscribers)
        for conn in subs:
            close_conn(conn, state)
        deadline = time.time() + 5.0
        while state.busy and time.time() < deadline:
            time.sleep(0.2)
        shutdown_server("service shutdown")
        log("stopped")
    return 0


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Left-arm persistent job orchestrator")
    p.add_argument(
        "--jobs-root",
        type=Path,
        default=None,
        help="Directory containing job1/, job2/, ... (default: openarm_inplace/recordings)",
    )
    p.add_argument(
        "--job-listen-port",
        type=int,
        default=defaults.JOB_LISTEN_PORT,
        help="TCP port for job commands (default 47925)",
    )
    p.add_argument(
        "--steady-csv",
        type=Path,
        default=None,
        help="Startup dual CSV (default: recordings/tasks/steady_action_on_table.csv)",
    )
    p.add_argument(
        "--continue-action2-on-grasp-fail",
        action="store_true",
        help="Still play action2 if 2D grasp fails (default: skip action2)",
    )
    p.add_argument(
        "--subscribe-heartbeat-s",
        type=float,
        default=float(defaults.SUBSCRIBE_HEARTBEAT_S),
        help="Status push interval to subscribed clients (default 2s)",
    )
    return p


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    jobs_root = (args.jobs_root or defaults.resolve_jobs_root()).expanduser().resolve()
    steady = (args.steady_csv or defaults.resolve_steady_csv()).expanduser().resolve()
    return serve(
        jobs_root=jobs_root,
        listen_port=int(args.job_listen_port),
        steady_csv=steady,
        skip_action2_on_grasp_fail=not args.continue_action2_on_grasp_fail,
        subscribe_heartbeat_s=float(args.subscribe_heartbeat_s),
    )


if __name__ == "__main__":
    raise SystemExit(main())
