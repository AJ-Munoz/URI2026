#!/usr/bin/python3
# ============================================================
# URI 2026 — Experiment Plotter
# Reads URI2026_Data.txt and produces publication-quality plots
#
# Usage: python URI2026_Plot.py
# ============================================================

import numpy as np
import matplotlib.pyplot as plt
import matplotlib
from pathlib import Path
import sys

# === Style ===
matplotlib.rcParams.update({
    'font.family': 'serif',
    'font.serif': ['Times New Roman', 'DejaVu Serif'],
    'font.size': 11,
    'axes.labelsize': 13,
    'axes.titlesize': 14,
    'legend.fontsize': 10,
    'xtick.labelsize': 10,
    'ytick.labelsize': 10,
    'figure.dpi': 150,
    'savefig.dpi': 300,
    'savefig.bbox': 'tight',
    'axes.grid': True,
    'grid.alpha': 0.3,
    'grid.linewidth': 0.5,
    'lines.linewidth': 1.5,
    'axes.spines.top': False,
    'axes.spines.right': False,
})

# === Colors ===
C_POS  = '#2E86AB'
C_REF  = '#E84855'
C_ERR  = '#F18F01'
C_CTRL = '#44BBA4'

COUNTS_PER_INCH = 4115

# === Load Data ===
datafile = Path("URI2026_Data.txt")
if not datafile.exists():
    print("URI2026_Data.txt not found. Run the experiment first.")
    sys.exit(1)

# Read header
header_lines = []
with open(datafile, 'r') as f:
    for line in f:
        if line.startswith('#'):
            header_lines.append(line.strip('# \n'))
        else:
            break

print("Experiment info:")
for h in header_lines:
    if h:
        print(f"  {h}")

# Load data
data = np.loadtxt(datafile, comments='#')
t  = data[:, 0]
e  = data[:, 1] / COUNTS_PER_INCH
x  = data[:, 2] / COUNTS_PER_INCH
xd = data[:, 3] / COUNTS_PER_INCH
u  = data[:, 4]
dt = data[:, 5]
Ts = np.median(dt)

# === Metrics ===
ISE  = np.trapz(e**2, t)
ISC  = np.trapz(u**2, t)
ITAE = np.trapz(t * np.abs(e), t)
e_ss = np.mean(np.abs(e[-100:]))

# Detect mode and controller from header
is_regulation = any('Regulation' in h for h in header_lines)
ctrl_name = "Unknown"
for h in header_lines:
    if 'Controller:' in h:
        ctrl_name = h.split(':')[-1].strip()
mode_name = "Step Response" if is_regulation else "Sine Tracking"

# === Figure 1: Time Series ===
fig, axes = plt.subplots(3, 1, figsize=(8, 6), sharex=True,
                          gridspec_kw={'height_ratios': [3, 2, 1.5]})

ax1 = axes[0]
ax1.plot(t, xd, color=C_REF, linewidth=2, label=r'Reference $x_d(t)$',
         linestyle='--', zorder=3)
ax1.plot(t, x, color=C_POS, linewidth=1.2, label=r'Actual $x(t)$',
         alpha=0.9, zorder=4)
ax1.set_ylabel('Position [in]')
ax1.legend(loc='upper right', framealpha=0.9, edgecolor='none')
ax1.set_title(f'{mode_name} — {ctrl_name} Controller', fontweight='bold')

ax2 = axes[1]
ax2.fill_between(t, e, 0, color=C_ERR, alpha=0.15)
ax2.plot(t, e, color=C_ERR, linewidth=1.0, label=r'Error $e(t)$')
ax2.axhline(0, color='gray', linewidth=0.5)
ax2.set_ylabel('Error [in]')
ax2.legend(loc='upper right', framealpha=0.9, edgecolor='none')

ax3 = axes[2]
ax3.fill_between(t, u, 0, color=C_CTRL, alpha=0.15)
ax3.plot(t, u, color=C_CTRL, linewidth=0.8, label=r'Control $u(t)$')
ax3.axhline(0, color='gray', linewidth=0.5)
ax3.set_ylabel('Control')
ax3.set_xlabel('Time [s]')
ax3.legend(loc='upper right', framealpha=0.9, edgecolor='none')
ax3.set_ylim([-1.1, 1.1])

plt.tight_layout(h_pad=0.5)
plt.savefig('URI2026_TimeSeries.png')
print("\nSaved: URI2026_TimeSeries.png")

# === Figure 3: Timing ===
fig3, (ax_dt, ax_hist) = plt.subplots(1, 2, figsize=(10, 3.5))

ax_dt.plot(t, dt * 1000, color=C_POS, linewidth=0.5, alpha=0.7)
ax_dt.axhline(Ts * 1000, color=C_REF, linewidth=1, linestyle='--',
              label=f'Median Ts = {Ts*1000:.2f} ms')
ax_dt.set_xlabel('Time [s]')
ax_dt.set_ylabel('Sample Interval [ms]')
ax_dt.set_title('Sample Timing', fontweight='bold')
ax_dt.legend(framealpha=0.9, edgecolor='none')

ax_hist.hist(dt * 1000, bins=50, color=C_POS, alpha=0.7, edgecolor='white')
ax_hist.axvline(Ts * 1000, color=C_REF, linewidth=1.5, linestyle='--')
ax_hist.set_xlabel('Sample Interval [ms]')
ax_hist.set_ylabel('Count')
ax_hist.set_title('Timing Distribution', fontweight='bold')

plt.tight_layout()
plt.savefig('URI2026_Timing.png')
print("Saved: URI2026_Timing.png")

plt.show()
print("\nDone. =^..^=")