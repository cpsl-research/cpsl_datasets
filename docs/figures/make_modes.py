"""Regenerate docs/figures/capture_modes.png (schematic; spacing is illustrative).

Run: uv run python docs/figures/make_modes.py
"""
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

fig, axs = plt.subplots(2, 1, figsize=(8, 3.6), sharex=True)
ref = [0.10, 0.35, 0.60, 0.85]          # reference (radar) stamps
all_rows = [0.02 + 0.067 * k for k in range(15)]            # every message, ~15 Hz
ticks = [0.0 + 0.1 * k for k in range(10)]                  # sampled: tick grid (not stored)
samp_rows = [0.02, 0.09, 0.2, 0.27, 0.41, 0.5, 0.62, 0.72, 0.8, 0.93]   # chosen messages' recv stamps

for ax, rows, title in ((axs[0], all_rows, "mode all: every message is a row"),
                        (axs[1], samp_rows, "mode sampled: at most one row per tick; the row keeps the message's own stamp")):
    ax.eventplot([ref], lineoffsets=1, linelengths=0.5, colors="#1b6ca8")
    ax.eventplot([rows], lineoffsets=0, linelengths=0.5, colors="#555555")
    t = ref[2]
    j = max(r for r in rows if r <= t)
    ax.plot([j, t], [0.45, 0.45], color="#c0392b", lw=2)
    ax.text((j + t) / 2, 0.55, "dt = t_sample - t_ref", ha="center", fontsize=8, color="#c0392b")
    ax.plot([j], [0], "o", color="#c0392b", ms=6)
    ax.set_yticks([1, 0])
    ax.set_yticklabels(["reference stamp", "odometry rows"], fontsize=8)
    ax.set_title(title, fontsize=9, loc="left")
    ax.set_ylim(-0.5, 1.5)
for k in ticks:
    axs[1].axvline(k, color="#bbbbbb", lw=0.6, ls=":", zorder=0)
axs[1].text(0.0, -0.45, "dotted: tick grid (not stored)", fontsize=7, color="#888888", va="bottom")
axs[1].set_xlabel("time (s), schematic; red dot = sample served for the third reference stamp")
axs[1].set_xlim(-0.03, 1.0)
fig.tight_layout()
fig.savefig(Path(__file__).with_name("capture_modes.png"), dpi=130)
