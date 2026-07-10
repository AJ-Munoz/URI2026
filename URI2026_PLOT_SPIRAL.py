#!/usr/bin/python3
# ============================================================
# URI 2026 — Dual Actuator Plotter
# Reads URI2026_Data_Dual.txt and produces publication plots.
# Columns: t e1 e2 x1 x2 xd1 xd2 u1 u2 phi theta tilt dt
# ============================================================

import numpy as np
import matplotlib.pyplot as plt
import matplotlib
from pathlib import Path
import sys

matplotlib.rcParams.update({
    'font.family': 'serif',
    'font.serif': ['Times New Roman', 'DejaVu Serif'],
    'font.size': 11,
    'axes.labelsize': 13, 'axes.titlesize': 14, 'legend.fontsize': 10,
    'xtick.labelsize': 10, 'ytick.labelsize': 10,
    'figure.dpi': 150, 'savefig.dpi': 300, 'savefig.bbox': 'tight',
    'axes.grid': True, 'grid.alpha': 0.3, 'grid.linewidth': 0.5,
    'lines.linewidth': 1.5,
    'axes.spines.top': False, 'axes.spines.right': False,
})

C_Q1, C_Q2 = '#2E86AB', '#E84855'
C_REF      = '#888888'
C_ERR      = '#F18F01'
C_CTRL     = '#44BBA4'
C_TILT     = '#6A4C93'

COUNTS_PER_INCH = 4115

datafile = Path("URI2026_Data_Dual.txt")
if not datafile.exists():
    print("URI2026_Data_Dual.txt not found. Run the experiment first.")
    sys.exit(1)

header_lines = []
with open(datafile) as fh:
    for line in fh:
        if line.startswith('#'):
            header_lines.append(line.strip('# \n'))
        else:
            break
print("Experiment info:")
for h in header_lines:
    if h:
        print(f"  {h}")

ctrl_name, mode_name, tilt_limit = "Unknown", "Run", None
for h in header_lines:
    if 'Controller:' in h:
        ctrl_name = h.split(':')[-1].strip()
    if 'Mode:' in h:
        mode_name = h.split(':')[-1].strip()
    if 'TiltLimit:' in h:
        try:
            tilt_limit = float(h.split(':')[1].split('deg')[0])
        except Exception:
            pass

d = np.loadtxt(datafile, comments='#')
t   = d[:, 0]
e   = d[:, 1:3] / COUNTS_PER_INCH
x   = d[:, 3:5] / COUNTS_PER_INCH
xd  = d[:, 5:7] / COUNTS_PER_INCH
u   = d[:, 7:9]
phi = np.degrees(d[:, 9])
th  = np.degrees(d[:, 10])
tilt = d[:, 11]
dt  = d[:, 12]
Ts  = np.median(dt)

# Metrics
ISE = np.trapz(np.sum(e**2, axis=1), t)
ISC = np.trapz(np.sum(u**2, axis=1), t)
print(f"\nISE = {ISE:.5f} in^2 s   ISC = {ISC:.3f}   peak tilt = {tilt.max():.1f} deg")

# === Figure 1: actuator tracking + error + control ===
fig, ax = plt.subplots(3, 1, figsize=(8, 7), sharex=True,
                       gridspec_kw={'height_ratios': [3, 2, 2]})

ax[0].plot(t, xd[:, 0], color=C_Q1, ls='--', alpha=0.6, label=r'$q_{1,d}$')
ax[0].plot(t, xd[:, 1], color=C_Q2, ls='--', alpha=0.6, label=r'$q_{2,d}$')
ax[0].plot(t, x[:, 0], color=C_Q1, label=r'$q_1$')
ax[0].plot(t, x[:, 1], color=C_Q2, label=r'$q_2$')
ax[0].axhline(2.0, color='r', ls=':', lw=0.8, alpha=0.5)
ax[0].axhline(-2.0, color='r', ls=':', lw=0.8, alpha=0.5, label='stroke')
ax[0].set_ylabel('Extension [in]')
ax[0].legend(loc='upper right', ncol=3, framealpha=0.9, edgecolor='none')
ax[0].set_title(f'{mode_name} — {ctrl_name} (per-actuator)', fontweight='bold')

ax[1].plot(t, e[:, 0], color=C_Q1, lw=1.0, label=r'$e_1$')
ax[1].plot(t, e[:, 1], color=C_Q2, lw=1.0, label=r'$e_2$')
ax[1].axhline(0, color='gray', lw=0.5)
ax[1].set_ylabel('Error [in]')
ax[1].legend(loc='upper right', framealpha=0.9, edgecolor='none')

ax[2].plot(t, u[:, 0], color=C_Q1, lw=0.8, label=r'$u_1$')
ax[2].plot(t, u[:, 1], color=C_Q2, lw=0.8, label=r'$u_2$')
ax[2].axhline(0, color='gray', lw=0.5)
ax[2].set_ylim([-1.1, 1.1])
ax[2].set_ylabel('Control')
ax[2].set_xlabel('Time [s]')
ax[2].legend(loc='upper right', framealpha=0.9, edgecolor='none')

plt.tight_layout(h_pad=0.5)
plt.savefig('URI2026_Dual_TimeSeries.png')
print("Saved: URI2026_Dual_TimeSeries.png")

# === Figure 2: orientation + tilt safety ===
fig2, (axa, axt) = plt.subplots(2, 1, figsize=(8, 5), sharex=True)
axa.plot(t, phi, color=C_Q1, label=r'$\phi$')
axa.plot(t, th, color=C_Q2, label=r'$\theta$')
axa.set_ylabel('Angle [deg]')
axa.legend(loc='upper right', framealpha=0.9, edgecolor='none')
axa.set_title('Orientation & Tilt Safety', fontweight='bold')

axt.plot(t, tilt, color=C_TILT, label='total tilt')
if tilt_limit is not None:
    axt.axhline(tilt_limit, color='r', ls='--', lw=1.2,
                label=f'limit = {tilt_limit:.0f}°')
axt.set_ylabel('Tilt about z [deg]')
axt.set_xlabel('Time [s]')
axt.legend(loc='upper right', framealpha=0.9, edgecolor='none')

plt.tight_layout(h_pad=0.5)
plt.savefig('URI2026_Dual_Tilt.png')
print("Saved: URI2026_Dual_Tilt.png")

# === Figure 3: timing ===
fig3, (axd, axh) = plt.subplots(1, 2, figsize=(10, 3.5))
axd.plot(t, dt * 1000, color=C_Q1, lw=0.5, alpha=0.7)
axd.axhline(Ts * 1000, color=C_REF, ls='--', lw=1, label=f'median Ts = {Ts*1000:.2f} ms')
axd.set_xlabel('Time [s]'); axd.set_ylabel('Sample Interval [ms]')
axd.set_title('Sample Timing', fontweight='bold')
axd.legend(framealpha=0.9, edgecolor='none')
axh.hist(dt * 1000, bins=50, color=C_Q1, alpha=0.7, edgecolor='white')
axh.axvline(Ts * 1000, color=C_REF, ls='--', lw=1.5)
axh.set_xlabel('Sample Interval [ms]'); axh.set_ylabel('Count')
axh.set_title('Timing Distribution', fontweight='bold')
plt.tight_layout()
plt.savefig('URI2026_Dual_Timing.png')
print("Saved: URI2026_Dual_Timing.png")

# ====== Figure 4: Spiral Visualization =====
fig4, (p1) = plt.subplots(1, 1)
p1.plot(th, phi, color=C_Q1)
p1.set_title('Spiral', fontweight='bold')
p1.set_aspect('equal')

plt.tight_layout(h_pad=0.5)
plt.savefig('URI2026_Spiral.png')
print("Saved: URI2026_Spiral.png")

# ====== Figure 5: Plot Lambda =====
phi_rad = np.radians(phi)
tht_rad = np.radians(th)

lam1 = np.cos(phi_rad) * np.sin(tht_rad)
lam2 = -np.sin(phi_rad)
lam3 = np.cos(phi_rad) * np.cos(tht_rad)

fig5 = plt.figure()
p5 = fig5.add_subplot(111, projection='3d')

p5.plot(lam1, lam2, lam3, color=C_Q1, label='Lambda Path')

p5.set_title('3D Lambda Plot', fontweight='bold')
p5.set_xlabel('Lam 1')
p5.set_ylabel('Lam 2')
p5.set_zlabel('Lam 3')

p5.set_xlim([-1, 1])
p5.set_ylim([-1, 1])
p5.set_zlim([-1, 1])
p5.set_box_aspect((1, 1, 1))

plt.tight_layout(h_pad=0.5)
plt.savefig('URI2026_Lambda_3D.png')
print("Saved: URI2026_Lambda_3D.png")

plt.show()
