#!/usr/bin/python3
# ============================================================
# URI 2026 — Dual Linear Actuator Control (2DoF TVC rig)
# 2x PA-HD2-4-2000-HS-12VDC via Teensy 4.1 + 2x BTS7960
#
# Controllers:
#   1. PID
#   2. DD-DOB (data-driven disturbance observer, adaptive Gamma)
#   3. Super Twisting
#   4. Unit-vector SMC (PD surface)
#
# Output: URI2026_Data_Dual.txt
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

STROKE_IN   = 2.0
STROKE_CNT  = STROKE_IN * COUNTS_PER_INCH
TILT_LIMIT_DEG = 45.0

# ---------------------------------------------------------------------
# GEOMETRY
# ---------------------------------------------------------------------
H1  = 13.28
H2  = 9.51
R   = 6.2
ELL = 0.6
GEOMETRY_MEASURED = True

# ---------------------------------------------------------------------
# POLARITY
# ---------------------------------------------------------------------
EXTEND_DIR_X = 1
EXTEND_DIR_Y = 1
ENC_SIGN_X   = +1
ENC_SIGN_Y   = +1

# =====================================================================
# Controller gains
# =====================================================================

# --- PID ---
Kp  = 5e-3
Ki  = 2e-4
Kd  = 2e-4
Kaw = 0.5 * Ki

# --- DD-DOB ---
ALPHA_DOB    = 10.0                   # sliding surface rate
RHO_DOB      = 10.0 * Ts             # = 0.02, conservative first run
RHO_S_DOB    = 0.0                    # tanh OFF
VARSIGMA_DOB = 1.0 / 800             # tanh slope (unused when RHO_S=0)
SIGMA_REG    = 0.1 * 80000           # = 8000, damped pinv (10% of Gamma0)

GAMMA_0      = 80000                  # m * alpha * T
ETA_0        = Ts**2                  # = 4e-6
MU_DOB       = Ts                     # = 0.002
DELTA_DOB    = Ts**2                  # = 4e-6

GAMMA_MIN    = 0.1 * GAMMA_0         # = 8000
GAMMA_MAX    = 10.0 * GAMMA_0        # = 800000
GAMMA_SIGN   = +1

# Chatter protection: ABORT if tripped
CHATTER_WINDOW = 50
CHATTER_THRESH = 1.0                  # 2x worst healthy controller (STA=0.42)

# --- Super Twisting ---
ST_K1 = 2e-2
ST_K2 = 1.5e-2
ALPHA_ST = 10.0

# --- Unit-vector SMC ---
UV_K    = 1.0
UV_EPS  = 400.0
ALPHA_SM = 10.0

USE_GRAVITY_FF = False
UV_GFF = 0.0

# =====================================================================
# Reference trajectory
# =====================================================================
W0 = np.deg2rad(10.0)
W1 = 0.2

# =====================================================================
# Geometry helpers
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

R_ = R

def norm_a(phi, theta):
    Rm = _Ry(theta) @ _Rx(phi)
    a = H1 * _ez + H2 * (Rm @ _ez) + R_ * (Rm @ _ex) - R_ * _ex
    return np.linalg.norm(a)

def norm_b(phi, theta):
    Rm = _Ry(theta) @ _Rx(phi)
    b = H1 * _ez + H2 * (Rm @ _ez) + R_ * (Rm @ _ey) - R_ * _ey
    return np.linalg.norm(b)

A0 = norm_a(0.0, 0.0)
B0 = norm_b(0.0, 0.0)

def lambda_to_eta(lam):
    l1, l2, l3 = lam
    phi   = -np.arctan2(l2, np.sqrt(l1**2 + l3**2))
    theta =  np.arctan2(l1, l3)
    return phi, theta

def eta_to_qdes_counts(phi, theta):
    dq1 = (norm_a(phi, theta) - A0) * COUNTS_PER_INCH
    dq2 = (norm_b(phi, theta) - B0) * COUNTS_PER_INCH
    return np.array([dq1, dq2])

def total_tilt_deg(phi, theta):
    lam_z = np.cos(phi) * np.cos(theta)
    lam_z = np.clip(lam_z, -1.0, 1.0)
    return np.degrees(np.arccos(lam_z))

def eta_from_q_counts(x1_cnt, x2_cnt, guess):
    dq1_in = x1_cnt / COUNTS_PER_INCH
    dq2_in = x2_cnt / COUNTS_PER_INCH
    target = np.array([A0 + dq1_in, B0 + dq2_in])
    eta = np.array(guess, dtype=float)
    eps = 1e-5
    for _ in range(5):
        f0 = np.array([norm_a(*eta), norm_b(*eta)]) - target
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
    p1 = int(255 * abs(np.clip(u1, -1.0, 1.0)))
    p2 = int(255 * abs(np.clip(u2, -1.0, 1.0)))
    p1 = 0 if p1 < 10 else p1
    p2 = 0 if p2 < 10 else p2
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
    while True:
        enc = read_encoders(ser)
        if enc is None:
            continue
        x1, x2 = enc
        d1 = -x1
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
# Homing
# =====================================================================
SEEK_PWM = 180
POS_PWM  = 150
STALL_S  = 1.5
HOME_EXTEND_CNT = int(STROKE_IN * COUNTS_PER_INCH)

def _retract_to_stops(ser):
    dxr = 1 - EXTEND_DIR_X
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
    print("\n--- HOMING ---")
    ans = input("  Is the TOP PLATE DISCONNECTED? Homing drives BOTH legs "
                "to their stops. [y/N]: ").strip().lower()
    if ans != 'y':
        print("  Stop-seek skipped. Set the plate LEVEL at the 2\" pose by hand,")
        input("  then press Enter to zero the encoders here (home / eta=0)... ")
        ser.write(bytes([6]))
        time.sleep(0.2)
        print(f"  Home set by hand. Encoders: {read_encoders(ser)}")
        return

    print("  Retracting both legs to full-retract stops...", flush=True)
    _retract_to_stops(ser)
    ser.write(bytes([6]))
    time.sleep(0.2)
    print(f"  At retract datum. Extending +2\" ({HOME_EXTEND_CNT} cnt)...", flush=True)
    _extend_both_to(ser, HOME_EXTEND_CNT)
    ser.write(bytes([6]))
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

# =====================================================================
# User setup
# =====================================================================
print("\nExperiment mode:")
print("  1 - Regulation   (step to fixed tilt)")
print("  2 - Circle       (constant-tilt cone)")
print("  3 - Figure-eight")
mode = int(input("Selection: "))

print("\nController:")
print("  1 - PID")
print("  2 - DD-DOB (data-driven disturbance observer)")
print("  3 - Super Twisting")
print("  4 - Unit-vector SMC")
ctr_sel = int(input("Selection: "))

if mode == 1:
    tilt_deg = float(input("\nTarget tilt [deg] (<=45): "))
    azi_deg  = float(input("Azimuth [deg] (0 = +x): "))
    if tilt_deg > TILT_LIMIT_DEG:
        print(f"  Clamped to {TILT_LIMIT_DEG} deg.")
        tilt_deg = TILT_LIMIT_DEG
    step_time = 2.0
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
            return np.array([0.0, 0.0, 1.0])
        return lam_tgt
    elif mode == 2:
        return np.array([
            np.sin(W0) * np.cos(W1 * t),
            np.sin(W0) * np.sin(W1 * t),
            np.cos(W0),
        ])
    else:
        return np.array([
            np.sin(W0) * np.cos(W1 * t),
            np.sin(W0) * np.sin(2.0 * W1 * t),
            np.cos(W0),
        ])

# =====================================================================
# Startup
# =====================================================================
mode_names = {1: "Regulation", 2: "Circle", 3: "FigureEight"}
ctrl_names = {1: "PID", 2: "DD-DOB", 3: "SuperTwisting", 4: "UnitVectorSMC"}

home_routine(ser)
time.sleep(0.3)

# =====================================================================
# State
# =====================================================================
Ie  = np.zeros(2)
Isq = np.zeros(2)
Aw  = np.zeros(2)
s0  = np.zeros(2)
s0_set = False
eta_est = np.zeros(2)
u_sat = np.zeros(2)
first_tick = True

# DD-DOB state
Gamma_k     = GAMMA_0 * np.eye(2)
Gamma_prev  = GAMMA_0 * np.eye(2)
d_hat       = np.zeros(2)
y_prev      = np.zeros(2)
e_prev      = np.zeros(2)
edot_prev   = np.zeros(2)
u_dob_prev  = np.zeros(2)
dob_ready   = False
chatter_buf = np.zeros(CHATTER_WINDOW)
chatter_idx = 0

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
f.write("# t\te1\te2\tx1\tx2\txd1\txd2\tu1\tu2\tphi\ttheta\ttilt\tdt"
        "\tG11\tG22\td1\td2\ty1\ty2\n")

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

        # --- reference ---
        lam = reference_lambda(t)
        phi_d, theta_d = lambda_to_eta(lam)

        if total_tilt_deg(phi_d, theta_d) > TILT_LIMIT_DEG + 1e-6:
            safe_abort(ser, f"commanded tilt {total_tilt_deg(phi_d,theta_d):.1f} > {TILT_LIMIT_DEG}")
            aborted = True
            break

        xd = eta_to_qdes_counts(phi_d, theta_d)

        # --- atomic command + feedback ---
        if first_tick:
            enc = read_encoders(ser)
            first_tick = False
        else:
            enc = drive_dual(ser, u_sat[0], u_sat[1])
        if enc is None:
            continue
        x = np.array(enc, dtype=float)

        # --- SAFETY ---
        if np.any(np.abs(x) > STROKE_CNT):
            safe_abort(ser, f"stroke limit (x={x.astype(int)}, limit={int(STROKE_CNT)})")
            aborted = True
            break
        eta_est = eta_from_q_counts(x[0], x[1], eta_est)
        tilt_meas = total_tilt_deg(eta_est[0], eta_est[1])
        if tilt_meas > TILT_LIMIT_DEG:
            safe_abort(ser, f"tilt {tilt_meas:.1f} > {TILT_LIMIT_DEG}")
            aborted = True
            break

        # --- error + Levant differentiator ---
        e = (x - xd) * (np.tanh(0.5 * t)**2)
        edot = -edot_prev + (2.0 / dt) * (e - e_prev) if t > Ts else np.zeros(2)
        Ie += dt * e

        # =============================================================
        # CONTROLLER
        # =============================================================
        log_G11 = Gamma_k[0, 0]
        log_G22 = Gamma_k[1, 1]
        log_d   = d_hat.copy()
        log_y   = np.zeros(2)

        if ctr_sel == 1:  # ----------------------------------------- PID
            u = -Kp * e - Kd * edot - Ki * Ie + Aw
            Aw += Kaw * (np.clip(u, -1.0, 1.0) - u) * dt

        elif ctr_sel == 2:  # --------------------------------------- DD-DOB
            # Tustin differentiation
            y = edot + ALPHA_DOB * e

            if dob_ready:
                Delta_y = y - y_prev
                psi = Delta_y - Gamma_prev @ u_dob_prev

                # Adaptive vartheta: aggressive when calm, conservative during transients
                Gu_norm_sq = np.linalg.norm(Gamma_prev @ u_dob_prev)**2
                denom_vt = Ts + np.linalg.norm(Delta_y)**2 + Gu_norm_sq
                vt = Ts / denom_vt

                # DOB update: (1-vt)*d_hat + vt*psi
                # vt->0 during transients: holds d_hat
                # vt->1 during calm: tracks residual
                d_hat = (1.0 - vt) * d_hat + vt * psi

                # Gamma update (skip at zero input)
                if np.linalg.norm(u_dob_prev) > 1e-6:
                    res = Delta_y - d_hat
                    eta_G = ETA_0 / (MU_DOB + np.dot(res, res)
                                     + np.dot(u_dob_prev, u_dob_prev))
                    B_k = (1.0 - DELTA_DOB * eta_G) * np.eye(2) \
                        - eta_G * np.outer(u_dob_prev, u_dob_prev)
                    Gamma_k = Gamma_k @ B_k + eta_G * np.outer(res, u_dob_prev)

                # Projection / reset
                for idx in range(2):
                    if abs(Gamma_k[idx, idx]) < GAMMA_MIN:
                        Gamma_k[idx, idx] = GAMMA_SIGN * GAMMA_MIN
                    elif abs(Gamma_k[idx, idx]) > GAMMA_MAX:
                        Gamma_k[idx, idx] = GAMMA_SIGN * GAMMA_MAX
                    if np.sign(Gamma_k[idx, idx]) != GAMMA_SIGN:
                        Gamma_k[idx, idx] = GAMMA_SIGN * GAMMA_0

            # Control law
            GtG = Gamma_k.T @ Gamma_k + SIGMA_REG * np.eye(2)
            Gamma_pinv = np.linalg.solve(GtG, Gamma_k.T)
            correction = RHO_DOB * y + d_hat
            if RHO_S_DOB > 0:
                correction += RHO_S_DOB * np.tanh(VARSIGMA_DOB * y)
            u = -Gamma_pinv @ correction

            # Chatter protection: ABORT if tripped
            if t > Ts:
                du = u - u_dob_prev
                chatter_buf[chatter_idx % CHATTER_WINDOW] = np.dot(du, du)
                chatter_idx += 1
                if np.sum(chatter_buf) > CHATTER_THRESH:
                    safe_abort(ser, f"chatter monitor: C={np.sum(chatter_buf):.4f} > {CHATTER_THRESH}")
                    aborted = True
                    break

            # Store for next tick
            y_prev     = y.copy()
            Gamma_prev = Gamma_k.copy()
            if not dob_ready:
                dob_ready = True

            log_G11 = Gamma_k[0, 0]
            log_G22 = Gamma_k[1, 1]
            log_d   = d_hat.copy()
            log_y   = y.copy()

        elif ctr_sel == 3:  # --------------------------------------- Super twisting
            s = edot + ALPHA_ST * e
            Isq += dt * np.sign(s)
            u = -ST_K1 * np.sqrt(np.abs(s)) * np.sign(s) - ST_K2 * Isq

        else:  # ---------------------------------------------------- Unit-vector SMC
            sPD = edot + ALPHA_SM * e
            u = -UV_K * sPD / (np.linalg.norm(sPD) + UV_EPS)

        u_sat = np.clip(u, -1.0, 1.0)

        # Store previous values for next cycle
        e_prev     = e.copy()
        edot_prev  = edot.copy()
        
        # Store u for DD-DOB
        if ctr_sel == 2:
            u_dob_prev = u_sat.copy()

        # --- metrics + log ---
        ISE += dt * e ** 2
        ISC += dt * u_sat ** 2
        f.write(f"{t:.4f}\t{e[0]:.2f}\t{e[1]:.2f}\t{x[0]:.1f}\t{x[1]:.1f}\t"
                f"{xd[0]:.1f}\t{xd[1]:.1f}\t{u_sat[0]:.4f}\t{u_sat[1]:.4f}\t"
                f"{eta_est[0]:.5f}\t{eta_est[1]:.5f}\t{tilt_meas:.3f}\t{dt:.6f}\t"
                f"{log_G11:.6f}\t{log_G22:.6f}\t"
                f"{log_d[0]:.4f}\t{log_d[1]:.4f}\t"
                f"{log_y[0]:.4f}\t{log_y[1]:.4f}\n")

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
    print("=^..^=")
    subprocess.run(["python3", "URI2026_PLOT_DUAL.py"])
