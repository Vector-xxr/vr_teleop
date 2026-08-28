"""Formal TCP line protocol for left_job_orchestrator.

Wire format: one UTF-8 JSON object per line (\\n terminated).

Request:
  {"v":1,"cmd":"status"|"run"|"subscribe"|"unsubscribe"|"abort"|"shutdown",
   "job":"1",          # required for run
   "id":"<client-id>", # optional correlation id
   "ts":1710000000.0,  # unix seconds
   "token":"...",      # shared secret (if server token configured)
   "sig":"<hex>"}      # optional HMAC-SHA256 over canonical payload

Canonical string for HMAC (UTF-8):
  "{v}|{cmd}|{job}|{id}|{ts:.3f}"
  job/id empty string if absent; ts formatted to 3 decimal places.

Response:
  {"v":1,"ok":true|false,"cmd":"...","id":"...","error":null|str, ...}

Event push (subscribe connections):
  {"v":1,"ok":true,"event":"status","busy":0|1, ...}

Legacy plain-text lines ("status", "1", ...) remain accepted only when
OPENARM_LEFT_JOB_REQUIRE_AUTH is off.

HMAC `sig` is optional for now (reserved for later); auth is token-only.
"""

from __future__ import annotations

import hashlib
import hmac
import json
import os
import time
import uuid
from typing import Any


PROTOCOL_VERSION = 1

# Default shared secret (change in production via OPENARM_LEFT_JOB_API_TOKEN).
DEFAULT_API_TOKEN = "openarm-job-api"


def resolve_api_token() -> str:
    env = os.environ.get("OPENARM_LEFT_JOB_API_TOKEN", "").strip()
    if env:
        return env
    # Empty string only if explicitly disabled with OPENARM_LEFT_JOB_REQUIRE_AUTH=0
    # and token cleared; otherwise use default token (auth on by default).
    return DEFAULT_API_TOKEN


def resolve_require_auth() -> bool:
    """Auth default ON. Set OPENARM_LEFT_JOB_REQUIRE_AUTH=0 to allow legacy plain text."""
    raw = os.environ.get("OPENARM_LEFT_JOB_REQUIRE_AUTH", "").strip()
    if raw in {"1", "true", "TRUE", "yes"}:
        return True
    if raw in {"0", "false", "FALSE", "no"}:
        return False
    return True


def canonical_sign_payload(cmd: str, *, job: str = "", req_id: str = "", ts: float = 0.0) -> str:
    return f"{PROTOCOL_VERSION}|{cmd}|{job}|{req_id}|{ts:.3f}"


def sign_request(secret: str, cmd: str, *, job: str = "", req_id: str = "", ts: float = 0.0) -> str:
    msg = canonical_sign_payload(cmd, job=job, req_id=req_id, ts=ts).encode("utf-8")
    return hmac.new(secret.encode("utf-8"), msg, hashlib.sha256).hexdigest()


def verify_sig(secret: str, sig: str, cmd: str, *, job: str = "", req_id: str = "", ts: float = 0.0) -> bool:
    if not secret or not sig:
        return False
    expect = sign_request(secret, cmd, job=job, req_id=req_id, ts=ts)
    return hmac.compare_digest(expect, sig.strip().lower())


def looks_like_json_line(text: str) -> bool:
    s = text.lstrip()
    return s.startswith("{") and s.endswith("}")


def build_request(
    cmd: str,
    *,
    job: str | None = None,
    req_id: str | None = None,
    token: str | None = None,
    sign: bool = False,
    ts: float | None = None,
) -> dict[str, Any]:
    secret = (token if token is not None else resolve_api_token()).strip()
    rid = req_id or uuid.uuid4().hex[:12]
    stamp = float(time.time() if ts is None else ts)
    job_s = "" if job is None else str(job)
    body: dict[str, Any] = {
        "v": PROTOCOL_VERSION,
        "cmd": cmd,
        "id": rid,
        "ts": round(stamp, 3),
    }
    if job_s:
        body["job"] = job_s
    if secret:
        body["token"] = secret
        # sig optional / reserved; enable with sign=True when needed later
        if sign:
            body["sig"] = sign_request(secret, cmd, job=job_s, req_id=rid, ts=body["ts"])
    return body


def encode_line(obj: dict[str, Any]) -> str:
    return json.dumps(obj, ensure_ascii=False, separators=(",", ":"))


def parse_line(text: str) -> dict[str, Any]:
    return json.loads(text)


def auth_check(req: dict[str, Any]) -> str | None:
    """Return error string if auth fails, else None.

    Current policy: require matching token only. HMAC `sig` is ignored if present
    (reserved for a later hardening pass).
    """
    secret = resolve_api_token()
    require = resolve_require_auth()
    if not require:
        return None
    if not secret:
        return "auth_misconfigured"
    token = str(req.get("token") or "").strip()
    if token and hmac.compare_digest(token, secret):
        return None
    return "unauthorized"


def error_response(
    *,
    cmd: str = "",
    req_id: str = "",
    error: str,
    **extra: Any,
) -> dict[str, Any]:
    out: dict[str, Any] = {
        "v": PROTOCOL_VERSION,
        "ok": False,
        "cmd": cmd,
        "id": req_id,
        "error": error,
    }
    out.update(extra)
    return out


def ok_response(
    *,
    cmd: str,
    req_id: str = "",
    **extra: Any,
) -> dict[str, Any]:
    out: dict[str, Any] = {
        "v": PROTOCOL_VERSION,
        "ok": True,
        "cmd": cmd,
        "id": req_id,
        "error": None,
    }
    out.update(extra)
    return out


def legacy_to_request(text: str) -> dict[str, Any] | None:
    """Map legacy pipe/plain commands into a request dict. None if empty."""
    cmd = text.strip()
    if not cmd:
        return None
    low = cmd.lower()
    if low in {"status", "list"}:
        return build_request("status", token="", sign=False)
    if low in {"subscribe", "sub"}:
        return build_request("subscribe", token="", sign=False)
    if low in {"unsubscribe", "unsub"}:
        return build_request("unsubscribe", token="", sign=False)
    if low in {"shutdown", "quit", "exit"}:
        return build_request("shutdown", token="", sign=False)
    if low in {"abort", "stop"}:
        return build_request("abort", token="", sign=False)
    job = None
    if low.startswith("run|"):
        job = cmd.split("|", 1)[1].strip()
    elif low.startswith("run "):
        job = cmd.split(None, 1)[1].strip()
    else:
        job = cmd
    # strip job prefix later in service via normalize
    return build_request("run", job=job, token="", sign=False)


def status_fields_from_pipe(payload: str) -> dict[str, Any]:
    """Best-effort parse of legacy ok|busy=... pipe string into JSON fields."""
    out: dict[str, Any] = {}
    if not payload.startswith("ok|"):
        out["raw"] = payload
        return out
    for part in payload.split("|")[1:]:
        if "=" not in part:
            continue
        k, v = part.split("=", 1)
        if k in {"busy", "startup_done", "subs"}:
            try:
                out[k] = int(v)
            except ValueError:
                out[k] = v
        elif k == "job":
            out["job"] = None if v in {"-", ""} else v
        elif k == "jobs":
            out["jobs"] = [] if v in {"-", ""} else v.split(",")
        elif k == "error":
            out["error"] = None if v in {"-", ""} else v
        elif k == "last":
            out["last"] = v
        else:
            out[k] = v
    return out
