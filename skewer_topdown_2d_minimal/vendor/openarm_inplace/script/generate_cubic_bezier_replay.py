#!/usr/bin/env python3
import argparse
import csv
from pathlib import Path


def parse_vec(text):
    vals = [float(x.strip()) for x in text.split(",") if x.strip()]
    if len(vals) != 16:
        raise argparse.ArgumentTypeError(f"expected 16 comma-separated values, got {len(vals)}")
    return vals


def load_vec_from_csv(path, row_index):
    with Path(path).open(newline="") as f:
        reader = csv.reader(f)
        next(reader)
        for i, row in enumerate(reader):
            if i == row_index:
                if len(row) != 33:
                    raise ValueError(f"{path}: expected 33 columns, got {len(row)} at data row {i}")
                return [float(row[1 + 2 * j]) for j in range(16)]
    raise ValueError(f"{path}: row {row_index} not found")


def cubic_bezier(q0, q1, q2, q3, s):
    a = (1.0 - s) ** 3
    b = 3.0 * (1.0 - s) ** 2 * s
    c = 3.0 * (1.0 - s) * s ** 2
    d = s ** 3
    return [a * q0[i] + b * q1[i] + c * q2[i] + d * q3[i] for i in range(16)]


def cubic_bezier_derivative(q0, q1, q2, q3, s, duration):
    a = 3.0 * (1.0 - s) ** 2
    b = 6.0 * (1.0 - s) * s
    c = 3.0 * s ** 2
    return [
        (a * (q1[i] - q0[i]) + b * (q2[i] - q1[i]) + c * (q3[i] - q2[i])) / duration
        for i in range(16)
    ]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--q-start", type=parse_vec)
    parser.add_argument("--q-goal", type=parse_vec)
    parser.add_argument("--start-csv", help="Read q_start from a replay CSV")
    parser.add_argument("--goal-csv", help="Read q_goal from a replay CSV")
    parser.add_argument("--start-row", type=int, default=0)
    parser.add_argument("--goal-row", type=int, default=-1)
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--hz", type=float, default=200.0)
    parser.add_argument("--output", default="recordings/bezier_start_to_goal_5s_200hz.csv")
    args = parser.parse_args()

    if args.q_start is not None:
        q_start = args.q_start
    elif args.start_csv:
        q_start = load_vec_from_csv(args.start_csv, args.start_row)
    else:
        raise SystemExit("Provide --q-start or --start-csv")

    if args.q_goal is not None:
        q_goal = args.q_goal
    elif args.goal_csv:
        goal_row = args.goal_row
        if goal_row < 0:
            with Path(args.goal_csv).open(newline="") as f:
                rows = list(csv.reader(f))
            goal_row = len(rows) - 2
        q_goal = load_vec_from_csv(args.goal_csv, goal_row)
    else:
        raise SystemExit("Provide --q-goal or --goal-csv")

    n = int(round(args.duration * args.hz))
    if n <= 0:
        raise SystemExit("--duration * --hz must be positive")

    out = Path(args.output)
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("w", newline="") as f:
        writer = csv.writer(f)
        header = ["time"]
        for i in range(16):
            header += [f"q{i}", f"dq{i}"]
        writer.writerow(header)
        for k in range(n + 1):
            t = k / args.hz
            s = min(1.0, t / args.duration)
            q = cubic_bezier(q_start, q_start, q_goal, q_goal, s)
            dq = cubic_bezier_derivative(q_start, q_start, q_goal, q_goal, s, args.duration)
            row = [f"{t:.9f}"]
            for qi, dqi in zip(q, dq):
                row += [f"{qi:.9f}", f"{dqi:.9f}"]
            writer.writerow(row)
    print(f"[bezier] wrote {n + 1} frames to {out}")


if __name__ == "__main__":
    main()
