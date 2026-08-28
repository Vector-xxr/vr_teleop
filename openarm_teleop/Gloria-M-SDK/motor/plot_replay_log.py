#!/usr/bin/env python3
"""Plot telemetry recorded by replay_openarm_csv.py."""

from __future__ import annotations

import argparse
import csv
import math
import os
from glob import glob

try:
    import matplotlib.pyplot as plt
except ModuleNotFoundError:
    plt = None


def latest_log() -> str:
    pattern = os.path.join(os.path.dirname(__file__), "replay_logs", "*.csv")
    candidates = sorted(glob(pattern), key=os.path.getmtime)
    if not candidates:
        raise FileNotFoundError(f"no replay logs found: {pattern}")
    return candidates[-1]


def to_float(value: str) -> float:
    if value is None or value == "":
        return math.nan
    try:
        return float(value)
    except ValueError:
        return math.nan


def load_log(path: str) -> dict[str, list]:
    rows: dict[str, list] = {}
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for name in reader.fieldnames or []:
            rows[name] = []
        for row in reader:
            for name in rows:
                if name == "phase":
                    rows[name].append(row.get(name, ""))
                else:
                    rows[name].append(to_float(row.get(name, "")))

    if not rows or not rows.get("wall_time"):
        raise ValueError("empty or invalid telemetry CSV")

    t0 = rows["wall_time"][0]
    rows["time_s"] = [v - t0 for v in rows["wall_time"]]
    return rows


def series(rows: dict[str, list], name: str) -> list[float]:
    return rows.get(name, [math.nan] * len(rows["time_s"]))


def crop_rows(rows: dict[str, list], start_s: float, phase: str | None = None) -> dict[str, list]:
    if start_s < 0:
        raise ValueError("--ignore-start must be non-negative")
    if start_s <= 0 and not phase:
        return rows

    phases = rows.get("phase", [""] * len(rows["time_s"]))
    phase_indices = [
        i for i, t in enumerate(rows["time_s"])
        if math.isfinite(t) and (not phase or phases[i] == phase)
    ]
    if not phase_indices:
        raise ValueError("no samples remain after applying --phase")

    base_time = rows["time_s"][phase_indices[0]]
    keep = [
        i for i in phase_indices
        if rows["time_s"][i] - base_time >= start_s
    ]
    if not keep:
        raise ValueError("no samples remain after applying --ignore-start")

    cropped = {}
    for name, values in rows.items():
        if isinstance(values, list) and len(values) == len(rows["time_s"]):
            cropped[name] = [values[i] for i in keep]
        else:
            cropped[name] = values
    return cropped


def rms(values: list[float]) -> float:
    finite = [v for v in values if math.isfinite(v)]
    if not finite:
        return math.nan
    return math.sqrt(sum(v * v for v in finite) / len(finite))


def build_summary(rows: dict[str, list], label: str) -> dict[str, float | str]:
    def stat(name: str) -> tuple[float, float]:
        values = [abs(v) for v in series(rows, name) if math.isfinite(v)]
        if not values:
            return math.nan, math.nan
        return max(values), rms(values)

    tau_max, tau_rms = stat("tau_fb")
    current_max, current_rms = stat("current_est_a")
    dq_max, dq_rms = stat("dq_fb")
    return {
        "label": label,
        "samples": len(rows["time_s"]),
        "tau_max": tau_max,
        "tau_rms": tau_rms,
        "current_max": current_max,
        "current_rms": current_rms,
        "dq_max": dq_max,
        "dq_rms": dq_rms,
    }


def summary_lines(summary: dict[str, float | str]) -> list[str]:
    return [
        f"window={summary['label']} samples={summary['samples']}",
        f"abs(tau_fb) max/rms={summary['tau_max']:.6g}/{summary['tau_rms']:.6g} Nm",
        f"abs(current_est_a) max/rms={summary['current_max']:.6g}/{summary['current_rms']:.6g} A",
        f"abs(dq_fb) max/rms={summary['dq_max']:.6g}/{summary['dq_rms']:.6g} rad/s",
    ]


def print_summary(summary: dict[str, float | str]) -> None:
    for line in summary_lines(summary):
        print(f"[SUMMARY] {line}")


def output_suffix(ignore_start: float, phase: str | None) -> str:
    parts = []
    if phase:
        parts.append(f"phase_{phase}")
    if ignore_start > 0:
        text = f"{ignore_start:g}".replace(".", "p")
        parts.append(f"ignore_start_{text}s")
    return "_" + "_".join(parts) if parts else ""


def plot_log(path: str, output: str | None, show: bool, ignore_start: float = 0.0, phase: str | None = None) -> str:
    raw_rows = load_log(path)
    rows = crop_rows(raw_rows, ignore_start, phase)
    label_parts = []
    if ignore_start > 0:
        label_parts.append(f"t>={ignore_start:.3f}s")
    if phase:
        label_parts.append(f"phase={phase}")
    window_label = ", ".join(label_parts) if label_parts else "full"
    summary = build_summary(rows, window_label)
    print_summary(summary)
    t = rows["time_s"]

    if output is None:
        base, _ = os.path.splitext(path)
        output = base + output_suffix(ignore_start, phase) + (".png" if plt is not None else ".svg")

    if plt is None:
        if show:
            print("[WARN] matplotlib is not installed; --show is unavailable with SVG fallback")
        return plot_log_svg(path, output, rows, summary)

    fig, axes = plt.subplots(5, 1, figsize=(14, 15.8), sharex=True)
    fig.suptitle(f"{os.path.basename(path)}  [{window_label}]", fontsize=13, y=0.992)
    fig.text(
        0.5,
        0.966,
        " | ".join(summary_lines(summary)[1:]),
        ha="center",
        va="top",
        fontsize=9,
    )

    ax = axes[0]
    ax.plot(t, series(rows, "q_cmd"), label="q_cmd", linewidth=1.2)
    ax.plot(t, series(rows, "q_fb"), label="q_fb", linewidth=1.0)
    ax.set_ylabel("position rad")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")

    ax = axes[1]
    ax.plot(t, series(rows, "dq_cmd"), label="dq_cmd", linewidth=1.2)
    ax.plot(t, series(rows, "dq_fb"), label="dq_fb", linewidth=1.0)
    ax.set_ylabel("velocity rad/s")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")

    ax = axes[2]
    ax.plot(t, series(rows, "tau_cmd"), label="tau_cmd", linewidth=1.0)
    ax.plot(t, series(rows, "tau_fb"), label="tau_fb", linewidth=1.0)
    ax.set_ylabel("torque Nm")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper left")
    ax2 = ax.twinx()
    ax2.plot(t, series(rows, "current_est_a"), label="current_est_a", color="tab:red", linewidth=0.9, alpha=0.75)
    ax2.set_ylabel("current est A")
    ax2.legend(loc="upper right")

    ax = axes[3]
    ax.plot(t, series(rows, "t_mos_c"), label="t_mos_c", linewidth=1.0)
    ax.plot(t, series(rows, "t_rotor_c"), label="t_rotor_c", linewidth=1.0)
    ax.set_ylabel("temperature C")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")

    ax = axes[4]
    send_late_ms = [v * 1000.0 if math.isfinite(v) else math.nan for v in series(rows, "send_late_s")]
    ax.plot(t, send_late_ms, label="send_late_ms", linewidth=0.9)
    ax.set_ylabel("late ms")
    ax.set_xlabel("time s")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")

    fig.tight_layout(rect=(0, 0, 1, 0.94))
    os.makedirs(os.path.dirname(os.path.abspath(output)), exist_ok=True)
    fig.savefig(output, dpi=160)
    print(f"[PLOT] saved {output}")

    if show:
        plt.show()
    else:
        plt.close(fig)
    return output


def finite_pairs(x_values: list[float], y_values: list[float]) -> list[tuple[float, float]]:
    return [(x, y) for x, y in zip(x_values, y_values) if math.isfinite(x) and math.isfinite(y)]


def nice_range(values: list[float]) -> tuple[float, float]:
    finite = [v for v in values if math.isfinite(v)]
    if not finite:
        return 0.0, 1.0
    lo, hi = min(finite), max(finite)
    if abs(hi - lo) < 1e-12:
        pad = max(1.0, abs(lo) * 0.1)
    else:
        pad = (hi - lo) * 0.08
    return lo - pad, hi + pad


def svg_polyline(
    pairs: list[tuple[float, float]],
    x_range: tuple[float, float],
    y_range: tuple[float, float],
    x0: float,
    y0: float,
    w: float,
    h: float,
) -> str:
    if not pairs:
        return ""
    xmin, xmax = x_range
    ymin, ymax = y_range
    xspan = max(1e-12, xmax - xmin)
    yspan = max(1e-12, ymax - ymin)
    points = []
    for x, y in pairs:
        px = x0 + (x - xmin) / xspan * w
        py = y0 + h - (y - ymin) / yspan * h
        points.append(f"{px:.2f},{py:.2f}")
    return " ".join(points)


def plot_log_svg(path: str, output: str, rows: dict[str, list], summary: dict[str, float | str]) -> str:
    t = rows["time_s"]
    width = 1400
    panel_h = 210
    left = 90
    right = 40
    top = 118
    gap = 45
    plot_w = width - left - right
    panels = [
        ("position rad", [("q_cmd", "#1f77b4"), ("q_fb", "#ff7f0e")]),
        ("velocity rad/s", [("dq_cmd", "#1f77b4"), ("dq_fb", "#ff7f0e")]),
        ("torque/current", [("tau_cmd", "#2ca02c"), ("tau_fb", "#9467bd"), ("current_est_a", "#d62728")]),
        ("temperature C", [("t_mos_c", "#8c564b"), ("t_rotor_c", "#e377c2")]),
        ("send late ms", [("send_late_ms", "#7f7f7f")]),
    ]
    height = top + len(panels) * panel_h + (len(panels) - 1) * gap + 70
    x_range = nice_range(t)

    derived = dict(rows)
    derived["send_late_ms"] = [
        v * 1000.0 if math.isfinite(v) else math.nan for v in series(rows, "send_late_s")
    ]

    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="white"/>',
        f'<text x="{width/2:.1f}" y="30" text-anchor="middle" font-family="sans-serif" font-size="18">{os.path.basename(path)}</text>',
        f'<text x="{width/2:.1f}" y="54" text-anchor="middle" font-family="sans-serif" font-size="13">{summary_lines(summary)[0]}</text>',
        f'<text x="{width/2:.1f}" y="76" text-anchor="middle" font-family="sans-serif" font-size="12">{summary_lines(summary)[1]}</text>',
        f'<text x="{width/2:.1f}" y="96" text-anchor="middle" font-family="sans-serif" font-size="12">{summary_lines(summary)[2]} | {summary_lines(summary)[3]}</text>',
    ]

    for idx, (ylabel, lines) in enumerate(panels):
        y_top = top + idx * (panel_h + gap)
        all_values: list[float] = []
        for name, _ in lines:
            all_values.extend(derived.get(name, []))
        y_range = nice_range(all_values)
        parts.append(f'<rect x="{left}" y="{y_top}" width="{plot_w}" height="{panel_h}" fill="#fafafa" stroke="#ccc"/>')
        parts.append(f'<text x="18" y="{y_top + panel_h/2:.1f}" font-family="sans-serif" font-size="13" transform="rotate(-90 18,{y_top + panel_h/2:.1f})">{ylabel}</text>')
        parts.append(f'<text x="{left - 8}" y="{y_top + 14}" text-anchor="end" font-family="monospace" font-size="11">{y_range[1]:.3g}</text>')
        parts.append(f'<text x="{left - 8}" y="{y_top + panel_h}" text-anchor="end" font-family="monospace" font-size="11">{y_range[0]:.3g}</text>')
        for grid_i in range(1, 4):
            gy = y_top + panel_h * grid_i / 4.0
            parts.append(f'<line x1="{left}" y1="{gy:.2f}" x2="{left + plot_w}" y2="{gy:.2f}" stroke="#e4e4e4"/>')
        legend_x = left + 12
        for name, color in lines:
            values = derived.get(name, [math.nan] * len(t))
            points = svg_polyline(finite_pairs(t, values), x_range, y_range, left, y_top, plot_w, panel_h)
            if points:
                parts.append(f'<polyline points="{points}" fill="none" stroke="{color}" stroke-width="1.4"/>')
            parts.append(f'<text x="{legend_x}" y="{y_top + 18}" font-family="sans-serif" font-size="12" fill="{color}">{name}</text>')
            legend_x += max(80, len(name) * 8 + 20)

    x_axis_y = top + len(panels) * panel_h + (len(panels) - 1) * gap + 25
    parts.append(f'<text x="{left}" y="{x_axis_y}" font-family="monospace" font-size="12">{x_range[0]:.2f}s</text>')
    parts.append(f'<text x="{left + plot_w}" y="{x_axis_y}" text-anchor="end" font-family="monospace" font-size="12">{x_range[1]:.2f}s</text>')
    parts.append(f'<text x="{width/2:.1f}" y="{x_axis_y + 25}" text-anchor="middle" font-family="sans-serif" font-size="13">time s</text>')
    parts.append("</svg>")

    os.makedirs(os.path.dirname(os.path.abspath(output)), exist_ok=True)
    with open(output, "w", encoding="utf-8") as f:
        f.write("\n".join(parts))
    print(f"[PLOT] matplotlib unavailable; saved SVG {output}")
    return output


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Plot replay_openarm_csv telemetry log.")
    parser.add_argument("log", nargs="?", default=None, help="telemetry CSV; default uses latest replay_logs/*.csv")
    parser.add_argument("-o", "--output", default=None, help="output PNG path; default is LOG.png")
    parser.add_argument("--show", action="store_true", help="also open an interactive matplotlib window")
    parser.add_argument("--ignore-start", type=float, default=0.0, help="ignore the first N seconds when plotting/statistics")
    parser.add_argument("--phase", default=None, help="only plot/statistics for a phase, e.g. replay")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    path = os.path.abspath(args.log or latest_log())
    plot_log(path, args.output, args.show, args.ignore_start, args.phase)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
