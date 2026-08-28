"""TLS helpers for left_job_orchestrator (default ON)."""

from __future__ import annotations

import os
import ssl
import subprocess
from pathlib import Path

from . import paths

CERT_DIR = paths.ORCH_DIR / "certs"
CERT_FILE = CERT_DIR / "server.crt"
KEY_FILE = CERT_DIR / "server.key"


def _env_flag(name: str, default: bool) -> bool:
    raw = os.environ.get(name, "").strip()
    if raw in {"1", "true", "TRUE", "yes", "on"}:
        return True
    if raw in {"0", "false", "FALSE", "no", "off"}:
        return False
    return default


def tls_enabled() -> bool:
    """Transport encryption default OFF. Set OPENARM_LEFT_JOB_TLS=1 to enable."""
    return _env_flag("OPENARM_LEFT_JOB_TLS", False)


def ensure_self_signed_cert(*, force: bool = False) -> tuple[Path, Path]:
    """Create certs/server.{crt,key} with openssl if missing."""
    CERT_DIR.mkdir(parents=True, exist_ok=True)
    if CERT_FILE.is_file() and KEY_FILE.is_file() and not force:
        return CERT_FILE, KEY_FILE
    cmd = [
        "openssl",
        "req",
        "-x509",
        "-newkey",
        "rsa:2048",
        "-keyout",
        str(KEY_FILE),
        "-out",
        str(CERT_FILE),
        "-days",
        "3650",
        "-nodes",
        "-subj",
        "/CN=left-job-orchestrator/O=OpenArm/C=CN",
    ]
    subprocess.run(cmd, check=True, capture_output=True)
    KEY_FILE.chmod(0o600)
    return CERT_FILE, KEY_FILE


def server_ssl_context() -> ssl.SSLContext:
    ensure_self_signed_cert()
    ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    ctx.minimum_version = ssl.TLSVersion.TLSv1_2
    ctx.load_cert_chain(certfile=str(CERT_FILE), keyfile=str(KEY_FILE))
    return ctx


def client_ssl_context(*, insecure: bool = True) -> ssl.SSLContext:
    """Client TLS. Default insecure=True (trust self-signed robot cert)."""
    if insecure or _env_flag("OPENARM_LEFT_JOB_TLS_INSECURE", True):
        ctx = ssl._create_unverified_context()
        ctx.minimum_version = ssl.TLSVersion.TLSv1_2
        return ctx
    ctx = ssl.create_default_context()
    ca = os.environ.get("OPENARM_LEFT_JOB_CA", "").strip()
    if ca:
        ctx.load_verify_locations(cafile=ca)
    elif CERT_FILE.is_file():
        ctx.load_verify_locations(cafile=str(CERT_FILE))
    ctx.minimum_version = ssl.TLSVersion.TLSv1_2
    return ctx


def wrap_server_socket(sock):
    return server_ssl_context().wrap_socket(sock, server_side=True)


def wrap_client_socket(sock, *, server_hostname: str = "left-job-orchestrator"):
    return client_ssl_context().wrap_socket(sock, server_hostname=server_hostname)
