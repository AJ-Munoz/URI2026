#!/usr/bin/python3
# ============================================================
# URI 2026 — Dual Linear Actuator Control (2DoF TVC rig)
# 2x PA-HD2-4-2000-HS-12VDC via Teensy 4.1 + 2x BTS7960
#
# Pipeline (per the IEEE paper):
#   lambda_des(t)  ->  (phi, theta)  ->  (q1_des, q2_des)  ->  controller  ->  PWM
#
# Modes:
#   1. Regulation — step the thrust vector to a fixed tilt and hold
#   2. Tracking   — figure-eight on the unit sphere (paper, Eq. 40)
#
# Controllers (applied PER ACTUATOR, two independent SISO channels):
#   1. PID
#   2. Sliding Mode (tanh)
#   3. Super Twisting
#
# SAFETY (45 deg total tilt about z):
#   - Stroke trip   : any actuator past +/-2"            -> ABORT (geometry-free)
#   - Tilt trip     : arccos(cos phi cos theta) > 45 deg -> ABORT (uses geometry)
#   - Reference trip: commanded tilt > 45 deg            -> ABORT (bad trajectory)
#   ABORT = drive both actuators to HOME (the 2" zero), then RESET the Teensy.
#
# Output: URI2026_Data_Dual.txt
# Usage:  python URI2026_PID_CONTROL_DUAL.py
# ============================================================

import serial
import time
import numpy as np
import subprocess

# =====================================================================
# Configuration
# =====================================================================
PORT = '/dev/ttyACM0'
BAUD = 115200
COUNTS_PER_INCH = 4115
Ts = 0.002
DURATION = 60

STROKE_IN   = 2.0                          # +/- travel about home [in]
STROKE_CNT  = STROKE_IN * COUNTS_PER_INCH  # +/- travel about home [counts]
TILT_LIMIT_DEG = 45.0                      # safety line, total tilt about z

# ---------------------------------------------------------------------
# GEOMETRY  ***  MEASURE THESE ON THE REAL RIG  ***
# Units: inches. The tilt-trip and the q_des mapping both depend on them.
# Until they are measured, the STROKE trip (geometry-free) is the
# authoritative safety; the tilt trip uses these placeholders.
# ---------------------------------------------------------------------
H1  = 13.28     # base plane -> U-joint center   [in]  <-- MEASURE
H2  = 9.51      # U-joint -> top-plate pin plane  [in]  <-- MEASURE
R   = 6.2      # pin-circle radius (both plates) [in]  <-- MEASURE (~5)
ELL = 0.6      # actuator rest length [in] (cancels in control; only logging)
GEOMETRY_MEASURED = True   # set True once H1,H2,R are real -> trust tilt trip

# ---------------------------------------------------------------------
# POLARITY  ***  SET AFTER A BENCH TEST  ***
# EXTEND_DIR : which BTS input extends the actuator (1 = RIGHT, 0 = LEFT).
#              Drive RIGHT (cmd 1/3) and watch the encoder; if counts FALL,
#              that input retracts, so set EXTEND_DIR = 0.
# ENC_SIGN   : +1 if extending makes counts INCREASE, else -1.
# ---------------------------------------------------------------------
EXTEND_DIR_X = 1
EXTEND_DIR_Y = 1
ENC_SIGN_X   = +1
ENC_SIGN_Y   = +1

# =====================================================================
# Controller gains  (per actuator; same structure as the single-actuator rig)
# =====================================================================
# PID
Kp  = 5e-3
Ki  = 2e-4
Kd  = 2e-4
Kaw = 0.5 * Ki

# PIDNet
Kd_PN     = 0.9
ALPHA_PN  = 10.0
vartheta = 1/700
BETA_PN   = np.zeros(6)
PHI_PN    = np.zeros((6,2))
gamma_0   = 0.4
GAMMA_PN  = np.array([0.05, 1.0, 0.2, 0.07, 0.2, 0.07])
sigma = 0.2 * 4115 # Larger value gives troubles
center0 = np.array([ 0.0, 0.0]) * 4115
center1 = np.array([ 0.2, 0.2]) * 4115
center2 = np.array([-0.2, 0.2]) * 4115
center3 = np.array([-0.2,-0.2]) * 4115
center4 = np.array([ 0.2,-0.2]) * 4115
sc  = 0.5 * ALPHA_PN * 4115 # Below this value there is a kick in the actuators


# Super Twisting
ST_K1 = 2e-2
ST_K2 = 1.5e-2
ALPHA_ST = 10.0


# Random
# rand_theta = (np.random.rand(3)-(0.5*np.ones(3)))*20 * np.pi / 180 
# rand_phi   = (np.random.rand(3)-(0.5*np.ones(3)))*20 * np.pi / 180 
rand_w0 = np.pi / 180.0 * np.array([5.0, 1.0, -5.0]) #-10 -> 10
rand_w1  = np.pi / 180.0 * np.array([90, -90, 45]) # -180 -> 180

print(rand_w0 * 180/np.pi)
print(rand_w1 * 180/np.pi)


# Paper unit-vector SMC (Eq. 29-35): coupled law on integral surface
#   s = e_dot + 2*alpha*e + alpha^2*Integral(e)   [s0 offset so s(0)=0]
#   u = g_q - K * s / (||s|| + eps)               [||s|| is the 2-vector norm]
# alpha (= ALPHA_SM = 5.0 /s) carries over from the paper directly.
# K and EPS are in NORMALIZED units here (u in [-1,1], e in counts),
# NOT the paper's Newtons/meters -> tune at the bench.
UV_K    = 1.0        # feedback gain (normalized) <-- BENCH TUNE
UV_EPS  = 200.0        # boundary layer [counts]    <-- BENCH TUNE
ALPHA_SM = 100.0

# Gravity feedforward. The PA-HD2 leadscrew is NON-BACKDRIVABLE: it holds
# position at zero command, so static gravity FF is largely inert on this
# hardware (unlike the force-actuator sim). Default OFF; flip on only to
# TEST whether the paper's FF-sensitivity result appears on the bench.
USE_GRAVITY_FF = False
UV_GFF = 0.0          # normalized feedforward magnitude if enabled

# Levant differentiator gain (per channel)
Ld = 1.0e4

# =====================================================================
# Reference trajectory parameters
# =====================================================================
W0 = np.deg2rad(10.0)   # tilt amplitude (cone half-angle of the figure-eight)
W1 = 0.2               # rad/s (slow)

# =====================================================================
# Geometry helpers (forward kinematics)
# =====================================================================
_ex = np.array([1.0, 0.0, 0.0])
_ey = np.array([0.0, 1.0, 0.0])
_ez = np.array([0.0, 0.0, 1.0])

def _Rx(p):
    c, s = np.cos(p), np.sin(p)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])

def _Ry(t):
    c, s = np.cos(t), np.sin(t)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])

# R_ is the scalar pin radius (named to avoid clashing with the rotation R)
R_ = R

def norm_a(phi, theta):
    R = _Ry(theta) @ _Rx(phi)
    a = H1 * _ez + H2 * (R @ _ez) + R_ * (R @ _ex) - R_ * _ex
    return np.linalg.norm(a)

def norm_b(phi, theta):
    R = _Ry(theta) @ _Rx(phi)
    b = H1 * _ez + H2 * (R @ _ez) + R_ * (R @ _ey) - R_ * _ey
    return np.linalg.norm(b)

A0 = norm_a(0.0, 0.0)   # actuator a length at home (= H1 + H2)
B0 = norm_b(0.0, 0.0)   # actuator b length at home (= H1 + H2)

def lambda_to_eta(lam):
    """Thrust-vector direction -> (phi, theta). Paper Eq. 28."""
    l1, l2, l3 = lam
    phi   = -np.arctan2(l2, np.sqrt(l1**2 + l3**2))
    theta =  np.arctan2(l1, l3)
    return phi, theta

def eta_to_qdes_counts(phi, theta):
    """(phi, theta) -> commanded actuator deltas from home, in COUNTS."""
    dq1 = (norm_a(phi, theta) - A0) * COUNTS_PER_INCH
    dq2 = (norm_b(phi, theta) - B0) * COUNTS_PER_INCH
    return np.array([dq1, dq2])

def total_tilt_deg(phi, theta):
    """Angle of the plate normal lambda = R*ez from the z-axis."""
    lam_z = np.cos(phi) * np.cos(theta)
    lam_z = np.clip(lam_z, -1.0, 1.0)
    return np.degrees(np.arccos(lam_z))

def eta_from_q_counts(x1_cnt, x2_cnt, guess):
    """Inverse kinematics: measured encoder counts -> (phi, theta).
    Solves norm_a(eta)=A0+dq1, norm_b(eta)=B0+dq2 by Newton iteration."""
    dq1_in = x1_cnt / COUNTS_PER_INCH
    dq2_in = x2_cnt / COUNTS_PER_INCH
    target = np.array([A0 + dq1_in, B0 + dq2_in])
    eta = np.array(guess, dtype=float)
    eps = 1e-5
    for _ in range(5):
        f0 = np.array([norm_a(*eta), norm_b(*eta)]) - target
        # finite-difference Jacobian
        ap = np.array([norm_a(eta[0] + eps, eta[1]), norm_b(eta[0] + eps, eta[1])])
        at = np.array([norm_a(eta[0], eta[1] + eps), norm_b(eta[0], eta[1] + eps)])
        J = np.column_stack(((ap - (f0 + target)) / eps, (at - (f0 + target)) / eps))
        try:
            deta = np.linalg.solve(J, -f0)
        except np.linalg.LinAlgError:
            break
        eta = eta + deta
        if np.linalg.norm(deta) < 1e-9:
            break
    return eta

# =====================================================================
# Serial helpers
# =====================================================================
def read_encoders(ser):
    """Returns (x1, x2) signed counts, or None on a bad line."""
    ser.reset_input_buffer()
    ser.write(bytes([5]))
    line = ser.readline().decode('utf-8', 'ignore').strip()
    parts = line.split()
    if len(parts) != 2:
        return None
    try:
        return ENC_SIGN_X * int(parts[0]), ENC_SIGN_Y * int(parts[1])
    except ValueError:
        return None

def drive_dual(ser, u1, u2):
    """Send control in [-1,1] per channel atomically; returns (x1, x2)."""
    p1 = int(255 * abs(np.clip(u1, -1.0, 1.0)))
    p2 = int(255 * abs(np.clip(u2, -1.0, 1.0)))
    p1 = 0 if p1 < 10 else p1
    p2 = 0 if p2 < 10 else p2
    # control-positive (u>0) means EXTEND -> map to the wiring's extend input
    dx = EXTEND_DIR_X if u1 >= 0 else (1 - EXTEND_DIR_X)
    dy = EXTEND_DIR_Y if u2 >= 0 else (1 - EXTEND_DIR_Y)
    ser.write(bytes([9, dx, p1, dy, p2]))
    line = ser.readline().decode('utf-8', 'ignore').strip()
    parts = line.split()
    if len(parts) != 2:
        return None
    try:
        return ENC_SIGN_X * int(parts[0]), ENC_SIGN_Y * int(parts[1])
    except ValueError:
        return None

def stop(ser):
    ser.write(bytes([7]))

def reset_teensy(ser):
    ser.write(bytes([8]))

def move_home(ser, pwm=120, tol=60):
    """Drive both actuators back to the home (0-count) position."""
    while True:
        enc = read_encoders(ser)
        if enc is None:
            continue
        x1, x2 = enc
        d1 = -x1   # want 0
        d2 = -x2
        if abs(d1) < tol and abs(d2) < tol:
            break
        dx = EXTEND_DIR_X if d1 >= 0 else (1 - EXTEND_DIR_X)
        dy = EXTEND_DIR_Y if d2 >= 0 else (1 - EXTEND_DIR_Y)
        p1 = 0 if abs(d1) < tol else pwm
        p2 = 0 if abs(d2) < tol else pwm
        ser.write(bytes([9, dx, p1, dy, p2]))
        ser.readline()
        time.sleep(0.01)
    stop(ser)
    time.sleep(0.2)

def safe_abort(ser, reason):
    """Trip: stop, return to home, reset the Teensy, close out."""
    print(f"\n\n*** SAFETY ABORT: {reason} ***")
    stop(ser)
    time.sleep(0.05)
    print("  Returning to home...", end="", flush=True)
    try:
        move_home(ser)
    except Exception as ex:
        print(f" (home failed: {ex})", end="")
    print(" done.")
    print("  Resetting Teensy...")
    reset_teensy(ser)
    time.sleep(0.2)

# =====================================================================
# Homing  (PLATE MUST BE DISCONNECTED — drives both legs to their stops)
# =====================================================================
SEEK_PWM = 180   # retract toward the full-retract hard stop
POS_PWM  = 150   # extend to the 2" home pose
STALL_S  = 1.5   # counts unchanged this long under power = stalled
HOME_EXTEND_CNT = int(STROKE_IN * COUNTS_PER_INCH)   # +2" from full retract

def _retract_to_stops(ser):
    """Retract BOTH legs to their hard stops. Per-leg stall detection;
    each leg cuts power independently the instant it stalls."""
    dxr = 1 - EXTEND_DIR_X      # retract direction (opposite of extend)
    dyr = 1 - EXTEND_DIR_Y
    last = [None, None]
    stall_t = [time.time(), time.time()]
    stalled = [False, False]
    while not all(stalled):
        pxr = 0 if stalled[0] else SEEK_PWM
        pyr = 0 if stalled[1] else SEEK_PWM
        ser.write(bytes([9, dxr, pxr, dyr, pyr]))
        parts = ser.readline().decode('utf-8', 'ignore').strip().split()
        if len(parts) != 2:
            continue
        try:
            pos = [ENC_SIGN_X * int(parts[0]), ENC_SIGN_Y * int(parts[1])]
        except ValueError:
            continue
        for i in (0, 1):
            if stalled[i]:
                continue
            if last[i] is not None and abs(pos[i] - last[i]) <= 2:
                if time.time() - stall_t[i] > STALL_S:
                    stalled[i] = True
            else:
                stall_t[i] = time.time()
            last[i] = pos[i]
        time.sleep(0.05)
    stop(ser)
    time.sleep(0.1)

def _extend_both_to(ser, target_cnt, tol=60):
    """Extend both legs together to +target_cnt from the current zero."""
    while True:
        enc = read_encoders(ser)
        if enc is None:
            continue
        d1 = target_cnt - enc[0]
        d2 = target_cnt - enc[1]
        if abs(d1) < tol and abs(d2) < tol:
            break
        dx = EXTEND_DIR_X if d1 >= 0 else (1 - EXTEND_DIR_X)
        dy = EXTEND_DIR_Y if d2 >= 0 else (1 - EXTEND_DIR_Y)
        p1 = 0 if abs(d1) < tol else POS_PWM
        p2 = 0 if abs(d2) < tol else POS_PWM
        ser.write(bytes([9, dx, p1, dy, p2]))
        ser.readline()
        time.sleep(0.01)
    stop(ser)
    time.sleep(0.2)

def home_routine(ser):
    """Establish home = 2" extended = eta 0, as a repeatable datum.
    Retract to stops -> zero -> extend +2" -> zero. Plate OFF only."""
    print("\n--- HOMING ---")
    ans = input("  Is the TOP PLATE DISCONNECTED? Homing drives BOTH legs "
                "to their stops. [y/N]: ").strip().lower()
    if ans != 'y':
        # safe fallback: hand-level-and-zero (no stop-seeking)
        print("  Stop-seek skipped. Set the plate LEVEL at the 2\" pose by hand,")
        input("  then press Enter to zero the encoders here (home / eta=0)... ")
        ser.write(bytes([6]))
        time.sleep(0.2)
        print(f"  Home set by hand. Encoders: {read_encoders(ser)}")
        return

    print("  Retracting both legs to full-retract stops...", flush=True)
    _retract_to_stops(ser)
    ser.write(bytes([6]))           # datum: full retract = 0
    time.sleep(0.2)
    print(f"  At retract datum. Extending +2\" ({HOME_EXTEND_CNT} cnt)...", flush=True)
    _extend_both_to(ser, HOME_EXTEND_CNT)
    ser.write(bytes([6]))           # zero here -> home, eta = 0
    time.sleep(0.2)
    print(f"  HOME established at 2\" (eta=0). Encoders: {read_encoders(ser)}")
    input("  >>> RECONNECT THE TOP PLATE, then press Enter to continue... ")

# =====================================================================
# Connect
# =====================================================================
try:
    ser = serial.Serial(port=PORT, baudrate=BAUD, timeout=0.1)
    time.sleep(2)
    ser.readline()
except Exception as e:
    print(f"Connection failed: {e}")
    exit()

print("\n" + "=" * 52)
print("  URI 2026 — Dual Actuator TVC Experiment")
print("=" * 52)
if not GEOMETRY_MEASURED:
    print("  [!] GEOMETRY_MEASURED = False")
    print("      Tilt trip uses placeholder H1/H2/R. The +/-2\" stroke")
    print("      trip is geometry-free and protects you meanwhile.")

# =====================================================================
# User setup
# =====================================================================
print("\nExperiment mode:")
print("  1 - Regulation   (step the thrust vector to a fixed tilt)")
print("  2 - Circle       (constant-tilt cone, traced at steady rate)")
print("  3 - Figure-eight (paper, Eq. 40)")
print("  4 - Random        (3 random points on the unit sphere, SLERP between)")
mode = int(input("Selection: "))

print("\nController (applied per actuator):")
print("  1 - PID")
print("  2 - PIDNet")
print("  3 - Super Twisting")
print("  4 - Unit-vector SMC (paper, coupled, integral surface)")
ctr_sel = int(input("Selection: "))

if mode == 1:
    tilt_deg = float(input("\nTarget tilt about z [deg] (<=45): "))
    azi_deg  = float(input("Azimuth of tilt [deg] (0 = +x): "))
    if tilt_deg > TILT_LIMIT_DEG:
        print(f"  Clamped to {TILT_LIMIT_DEG} deg.")
        tilt_deg = TILT_LIMIT_DEG
    step_time = 2.0
    # fixed target lambda on the cone
    lam_tgt = np.array([
        np.sin(np.deg2rad(tilt_deg)) * np.cos(np.deg2rad(azi_deg)),
        np.sin(np.deg2rad(tilt_deg)) * np.sin(np.deg2rad(azi_deg)),
        np.cos(np.deg2rad(tilt_deg)),
    ])

# =====================================================================
# Reference generator
# =====================================================================
def reference_lambda(t):
    if mode == 1:
        if t < step_time:
            return np.array([0.0, 0.0, 1.0])     # level until the step
        return lam_tgt
    elif mode == 2:                              # circle: constant tilt W0
        return np.array([
            np.sin(W0) * np.cos(W1 * t),
            np.sin(W0) * np.sin(W1 * t),
            np.cos(W0),
        ])
    elif mode == 3:                                        # figure-eight (paper Eq. 40)
        return np.array([
            np.sin(W0) * np.cos(W1 * t),
            np.sin(W0) * np.sin(2.0 * W1 * t),
            np.cos(W0),
        ])
    else:
        return SLERP(t)

def SLERP(t):
	if t < DURATION/3:
		W0 = rand_w0[0]
		W1 = rand_w1[0]
	elif t < 2*DURATION/3:
		W0 = rand_w0[1]
		W1 = rand_w1[1]
	else:
		W0 = rand_w0[2]
		W1 = rand_w1[2]
	
	return np.array([
            np.sin(W0) * np.cos(W1),
            np.sin(W0) * np.sin(W1),
            np.cos(W0),
        ])
# =====================================================================
# Startup: home / zero
# =====================================================================
mode_names = {1: "Regulation", 2: "Circle", 3: "FigureEight", 4: "Random"}
ctrl_names = {1: "PID", 2: "RBF-PIDNet", 3: "SuperTwisting", 4: "UnitVectorSMC"}

home_routine(ser)
time.sleep(0.3)

# =====================================================================
# State (per channel, length-2 numpy)
# =====================================================================
z1  = np.zeros(2)
z2  = np.zeros(2)
Iz  = np.zeros(2)
Isq = np.zeros(2)
Aw  = np.zeros(2)
s0  = np.zeros(2)       # surface offset so s(0)=0 (unit-vector SMC)
s0_set = False
eta_est = np.zeros(2)   # warm-start for inverse kinematics
u_sat = np.zeros(2)     # control output, defined before first use
first_tick = True       # first loop iteration only reads (no u yet)

ISE = np.zeros(2)
ISC = np.zeros(2)

# =====================================================================
# Data file
# =====================================================================
f = open("URI2026_Data_Dual.txt", "w")
f.write("# URI 2026 Dual Actuator TVC Experiment\n")
f.write(f"# Mode: {mode_names[mode]}\n")
f.write(f"# Controller: {ctrl_names[ctr_sel]}\n")
if mode == 1:
    f.write(f"# Step: tilt {tilt_deg} deg, azimuth {azi_deg} deg at t={step_time}s\n")
elif mode == 2:
    f.write(f"# Circle: tilt {np.rad2deg(W0):.1f} deg, W1={W1} rad/s\n")
else:
    f.write(f"# Figure-eight: W0={np.rad2deg(W0)} deg, W1={W1} rad/s\n")
f.write(f"# Geometry[in]: H1={H1} H2={H2} R={R} ELL={ELL} measured={GEOMETRY_MEASURED}\n")
f.write(f"# TiltLimit: {TILT_LIMIT_DEG} deg   Ts: {Ts}s   Duration: {DURATION}s\n")
f.write("# t\te1\te2\tx1\tx2\txd1\txd2\tu1\tu2\tphi\ttheta\ttilt\tdt\n")

# =====================================================================
# Run
# =====================================================================
print(f"\nRunning {DURATION}s ...", flush=True)
aborted = False

try:
    start_time = time.perf_counter()
    last_sample_time = start_time
    next_sample_time = start_time + Ts
    t = 0.0

    while t < DURATION:
        while time.perf_counter() < next_sample_time:
            pass
        now = time.perf_counter()
        dt = now - last_sample_time
        last_sample_time = now
        t = now - start_time
        next_sample_time += Ts

        # --- reference: lambda -> eta -> q_des ---
        lam = reference_lambda(t)
        phi_d, theta_d = lambda_to_eta(lam)

        # reference-side safety: never command past the line
        if total_tilt_deg(phi_d, theta_d) > TILT_LIMIT_DEG + 1e-6:
            safe_abort(ser, f"commanded tilt {total_tilt_deg(phi_d,theta_d):.1f} deg > {TILT_LIMIT_DEG}")
            aborted = True
            break

        xd = eta_to_qdes_counts(phi_d, theta_d)   # [counts, counts]

        # --- atomic command + feedback ---
        # (first tick we have no u yet -> send zero, just read)
        if first_tick:
            enc = read_encoders(ser)
            first_tick = False
        else:
            enc = drive_dual(ser, u_sat[0], u_sat[1])
        if enc is None:
            continue
        x = np.array(enc, dtype=float)            # measured counts [2]

        # --- SAFETY TRIPS ---
        # 1) stroke trip (geometry-free, always valid)
        if np.any(np.abs(x) > STROKE_CNT):
            safe_abort(ser, f"stroke limit (x={x.astype(int)} cnt, |limit|={int(STROKE_CNT)})")
            aborted = True
            break
        # 2) tilt trip (forward kinematics from encoders)
        eta_est = eta_from_q_counts(x[0], x[1], eta_est)
        tilt_meas = total_tilt_deg(eta_est[0], eta_est[1])
        if tilt_meas > TILT_LIMIT_DEG:
            safe_abort(ser, f"measured tilt {tilt_meas:.1f} deg > {TILT_LIMIT_DEG}")
            aborted = True
            break

        # --- error + Levant differentiator ---
        e = (x - xd) * (np.tanh(.5*t)**2) # 1*t
        ez = e - z1
        dz1 = 1.5 * np.sqrt(Ld) * np.sqrt(np.abs(ez) + 1e-6) * np.sign(ez) + z2
        dz2 = 1.1 * Ld * np.sign(ez)
        z1 += dt * dz1
        z2 += dt * dz2
        Iz += dt * z1

        # --- control law (vectorised over the two channels) ---
        if ctr_sel == 1:                                   # PID
            u = -Kp * z1 - Kd * z2 - Ki * Iz + Aw
            Aw += Kaw * (np.clip(u, -1.0, 1.0) - u) * dt
        elif ctr_sel == 2:                                 # PIDNet
            s   = z2 + ALPHA_PN*z1
            sv1 = 0 if np.linalg.norm(s[0]) >= sc else 1
            sv2 = 0 if np.linalg.norm(s[1]) >= sc else 1
            sv  = np.diag(np.array([sv1, sv2]))
            xi1 = np.array([z1[0],z2[0]])
            xi2 = np.array([z1[1],z2[1]])
            phi01 = np.exp(-np.linalg.norm(xi1 - center0)**2 / (2*sigma**2))
            phi11 = np.exp(-np.linalg.norm(xi1 - center1)**2 / (2*sigma**2))
            phi21 = np.exp(-np.linalg.norm(xi1 - center2)**2 / (2*sigma**2))
            phi31 = np.exp(-np.linalg.norm(xi1 - center3)**2 / (2*sigma**2))  
            phi41 = np.exp(-np.linalg.norm(xi1 - center4)**2 / (2*sigma**2))  
            phi02 = np.exp(-np.linalg.norm(xi2 - center0)**2 / (2*sigma**2))  
            phi12 = np.exp(-np.linalg.norm(xi2 - center1)**2 / (2*sigma**2))  
            phi22 = np.exp(-np.linalg.norm(xi2 - center2)**2 / (2*sigma**2))  
            phi32 = np.exp(-np.linalg.norm(xi2 - center3)**2 / (2*sigma**2))  
            phi42 = np.exp(-np.linalg.norm(xi2 - center4)**2 / (2*sigma**2))  
            PHI_PN = np.array([
				[1    , 1],
				[phi01, phi02],
				[phi11, phi12],
				[phi21, phi22],
				[phi31, phi32],
				[phi41, phi42]
            ]) @ sv
            adap_fnc = np.tanh(vartheta * s)
            BETA_PN += dt * GAMMA_PN * ( PHI_PN @ adap_fnc) \
            - gamma_0 * GAMMA_PN * BETA_PN
            u = - Kd_PN * adap_fnc - PHI_PN.T @ BETA_PN
        elif ctr_sel == 3:                                 # Super twisting
            s = z2 + ALPHA_ST * z1
            Isq += dt * np.sign(s)
            u = -ST_K1 * np.sqrt(np.abs(s)) * np.sign(s) - ST_K2 * Isq
        else:                                              # Unit-vector SMC (paper, coupled)
            # integral surface: s = e_dot + 2a e + a^2 * Int(e)  (z1~e, z2~e_dot, Iz~Int e)
            ALPHA_UV_PD = 100
            sPD = z2 + ALPHA_UV_PD * z1
            #ALPHA_UV_PID = 10
            #sPID = z2 + 10 * ALPHA_UV_PID * z1 + 0.01*(ALPHA_UV_PID**2) * Iz
            u = - UV_K * sPD / (np.linalg.norm(sPD) + UV_EPS)   # COUPLED: shared ||s||

        u_sat = np.clip(u, -1.0, 1.0)

        # --- metrics + log ---
        ISE += dt * e ** 2
        ISC += dt * u_sat ** 2
        f.write(f"{t:.4f}\t{e[0]:.2f}\t{e[1]:.2f}\t{x[0]:.1f}\t{x[1]:.1f}\t"
                f"{xd[0]:.1f}\t{xd[1]:.1f}\t{u_sat[0]:.4f}\t{u_sat[1]:.4f}\t"
                f"{eta_est[0]:.5f}\t{eta_est[1]:.5f}\t{tilt_meas:.3f}\t{dt:.6f}\n")

    if not aborted:
        print(" Done.")

except KeyboardInterrupt:
    print("\n\nInterrupted by user.")

finally:
    if not aborted:
        stop(ser)
        time.sleep(0.05)
        try:
            move_home(ser)
        except Exception:
            pass
        reset_teensy(ser)
        time.sleep(0.1)
    if 'ser' in locals() and ser.is_open:
        ser.close()
    f.close()
    print(f"\nISE = [{ISE[0]:.2f}, {ISE[1]:.2f}] cnt^2 s,  "
          f"ISC = [{ISC[0]:.4f}, {ISC[1]:.4f}]")
    print("Data saved to URI2026_Data_Dual.txt")
    print("Run URI2026_PLOT_DUAL.py for plots.")
    print("=^..^=")
    subprocess.run(["python3","URI2026_PLOT_DUAL.py"])

