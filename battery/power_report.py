#!/usr/bin/env python3
"""
Power report for one battery log (T10 / T14).

Reads a CSV from battery_logger.py and prints mean power per group:
  * per activity label (lying / standing / walking / ...), by default
  * per named time block, with --blocks, for tests where the label does
    not change (e.g. lying with different programs running on PC2)

Rows marked gap = 1 are skipped (the data there is stale). The first
--skip seconds of every block are skipped too, so a block measures the
settled state, not the moment a program was started.

Optional --plot draws load power (1-minute mean) and SoC over time, with
the blocks shaded, as a PNG for the logbook.

Run (no ROS needed):
    python3 power_report.py logs/battery_20261020_150544.csv
    python3 power_report.py logs/battery_20261022_152118.csv \
        --blocks "A=15:21-15:50,B=15:50-16:15,C=16:15-16:40" --plot fig.png
"""

import argparse
import csv
import statistics
from datetime import datetime


def read_rows(path):
    rows = []
    with open(path) as f:
        for r in csv.DictReader(f):
            if r["gap"] == "1" or not r["load_w"]:
                continue
            rows.append({
                "t": datetime.fromisoformat(r["time_iso"]),
                "w": float(r["load_w"]),
                "soc": (int(r["p1_soc"]) + int(r["p2_soc"])) / 2,
                "temp": max(float(r["p1_temp_max_c"]), float(r["p2_temp_max_c"])),
                "activity": r["activity"],
            })
    return rows


def parse_blocks(text, day):
    """'A=15:21-15:50,B=15:50-16:15' -> [(name, start, end), ...] on the log's day."""
    blocks = []
    for part in text.split(","):
        name, span = part.split("=")
        a, b = span.split("-")
        at = lambda hm: datetime.combine(day, datetime.strptime(hm.strip(), "%H:%M").time())
        blocks.append((name.strip(), at(a), at(b)))
    return blocks


def group(rows, blocks, skip_s):
    """Returns [(name, rows)] by block, or by activity label when no blocks given."""
    if blocks:
        return [(n, [r for r in rows if (r["t"] - a).total_seconds() >= skip_s and r["t"] < b])
                for n, a, b in blocks]
    names = sorted(set(r["activity"] for r in rows))
    return [(n, [r for r in rows if r["activity"] == n]) for n in names]


def report(groups):
    print(f"{'group':<10}{'minutes':>8}{'mean W':>9}{'sd W':>7}{'median W':>10}{'SoC start-end':>15}{'max degC':>10}")
    for name, g in groups:
        if len(g) < 2:
            print(f"{name:<10}{'no data':>8}")
            continue
        w = [r["w"] for r in g]
        # time actually spent in this group = rows x normal row spacing
        # (an activity label comes and goes, so first-to-last time would overcount)
        step = statistics.median((b["t"] - a["t"]).total_seconds() for a, b in zip(g, g[1:]))
        mins = len(g) * step / 60
        soc = f"{g[0]['soc']:.0f} -> {g[-1]['soc']:.0f} %"
        print(f"{name:<10}{mins:>8.1f}{statistics.mean(w):>9.1f}{statistics.pstdev(w):>7.1f}"
              f"{statistics.median(w):>10.1f}{soc:>15}{max(r['temp'] for r in g):>10.0f}")


def plot(rows, blocks, out):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import matplotlib.dates as mdates

    # 1-minute mean: the raw 10 Hz power jumps by +-10 W, which hides the level
    t, w, soc = [], [], []
    for i in range(0, len(rows), 600):
        chunk = rows[i:i + 600]
        t.append(chunk[len(chunk) // 2]["t"])
        w.append(statistics.mean(r["w"] for r in chunk))
        soc.append(chunk[-1]["soc"])

    fig, ax1 = plt.subplots(figsize=(8, 4))
    ax1.plot(t, w, color="tab:blue", label="Load power (1-min mean)")
    ax1.set_ylabel("Power from both packs (W)")
    ax1.set_ylim(0, max(w) * 1.25)  # from 0, so small changes don't look huge
    ax1.set_xlabel("Time")
    ax1.xaxis.set_major_formatter(mdates.DateFormatter("%H:%M"))
    ax2 = ax1.twinx()
    ax2.step(t, soc, where="post", color="tab:orange", label="State of charge")
    ax2.set_ylabel("State of charge (%)")
    for k, (name, a, b) in enumerate(blocks or []):
        ax1.axvspan(a, b, color="grey", alpha=0.08 if k % 2 else 0.18)
        ax1.text(a, ax1.get_ylim()[1], f" {name}", va="top", fontsize=9)
    lines = ax1.get_legend_handles_labels()
    more = ax2.get_legend_handles_labels()
    ax1.legend(lines[0] + more[0], lines[1] + more[1], loc="lower left", fontsize=8)
    ax1.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(out, dpi=150)
    print(f"plot saved: {out}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv")
    ap.add_argument("--blocks", help='named time blocks, e.g. "A=15:21-15:50,B=15:50-16:15"')
    ap.add_argument("--skip", type=float, default=60, help="seconds skipped at the start of each block (default 60)")
    ap.add_argument("--plot", help="save a power + SoC plot to this PNG")
    args = ap.parse_args()

    rows = read_rows(args.csv)
    blocks = parse_blocks(args.blocks, rows[0]["t"].date()) if args.blocks else None
    report(group(rows, blocks, args.skip))
    if args.plot:
        plot(rows, blocks, args.plot)


if __name__ == "__main__":
    main()
