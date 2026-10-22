"""Regenerate docs/figures/radar_timing.png (schematic; offsets from radar06, one sensor).

Run: uv run python docs/figures/make_timing.py
"""
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

OFFSET = 0.112  # s; radar06 radar_adc recv - header, mean (range 0.105-0.126)
PERIOD = 0.5    # s; schematic frame spacing (radar06 is ~0.51 s)

fig, ax = plt.subplots(figsize=(8, 2.9))
hdr = [i * PERIOD for i in range(4)]
rec = [h + OFFSET for h in hdr]
ax.eventplot([hdr], lineoffsets=2, linelengths=0.5, colors="#1b6ca8")
ax.eventplot([rec], lineoffsets=1, linelengths=0.5, colors="#c0392b")
odom = [0.05 + 0.1 * k for k in range(18)]  # recv stamps only, ~10 per radar frame here
ax.eventplot([odom], lineoffsets=0, linelengths=0.5, colors="#555555")
for h, r in zip(hdr[:2], rec[:2]):
    ax.annotate("", xy=(r, 1.5), xytext=(h, 1.5),
                arrowprops=dict(arrowstyle="->", color="#333"))
ax.text(hdr[0] + 0.005, 1.58, "recv - header\n~112 ms (radar06)", fontsize=8, va="bottom")
t0, t1 = rec[1], rec[2]
ax.axvspan(t0, t1, ymin=0.0, ymax=0.12, color="#f0b429", alpha=0.6)
ax.text((t0 + t1) / 2, -0.5, "odometry window for frame 2:\n(recv[1], recv[2]]",
        ha="center", va="top", fontsize=8)
ax.axvline(t1, color="#c0392b", ls=":", lw=1)
ax.text(t1 + 0.01, 2.55, "reference stamp\nrecv[2] (default clock)", fontsize=8, va="top")
ax.set_yticks([2, 1, 0])
ax.set_yticklabels(["radar header stamp", "radar recv stamp", "odom recv stamp"])
ax.set_xlabel("time (s), schematic spacing")
ax.set_ylim(-1.0, 2.8)
ax.set_xlim(-0.05, 1.8)
fig.tight_layout()
fig.savefig(Path(__file__).with_name("radar_timing.png"), dpi=130)
