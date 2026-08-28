#!/usr/bin/env python3
"""Compatibility loader for the recovered Python 3.11 calibration module."""

from __future__ import annotations

import marshal
from pathlib import Path


RECOVERED_PYC = (
    Path(__file__).resolve().parent
    / "__pycache__"
    / "auto_calibrate_pixel_to_motion.recovered.cpython-311.pyc"
)


def _load_recovered_module() -> None:
    if not RECOVERED_PYC.is_file():
        raise RuntimeError(f"Recovered calibration bytecode is missing: {RECOVERED_PYC}")
    if RECOVERED_PYC.read_bytes()[:4] != __import__("importlib.util").util.MAGIC_NUMBER:
        raise RuntimeError(
            "Recovered calibration bytecode does not match this Python version; "
            "run it with Python 3.11 from openarm_teleop1."
        )
    with RECOVERED_PYC.open("rb") as stream:
        stream.read(16)
        code = marshal.load(stream)
    exec(code, globals(), globals())


_load_recovered_module()
