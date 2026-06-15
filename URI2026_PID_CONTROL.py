#!/usr/bin/python3
# ============================================================
# URI 2026 — Linear Actuator Control Experiment
# PA-HD2-4-2000-HS-12VDC via Teensy 4.1 + BTS7960
#
# Modes:
#   1. Regulation — step to a position and hold
#   2. Tracking   — follow a sine wave
#
# Controllers:
#   1. P only
#   2. Sliding Mode (tanh)
#   3. Super Twisting
#
# Output: URI2026_Data.txt
# Usage:  python URI2026_Experiment.py
# ============================================================

import serial
import time
import numpy as np

# === Configuration ===
PORT = '/dev/ttyACM0'
BAUD = 115200
COUNTS_PER_INCH = 4115
TOTAL_COUNTS = 16460
Ts = 0.002
DURATION = 20.0

# === Sliding Mode Parameters ===
K_SM = 220.0
LAMBDA_SM = 0.001
ALPHA_SM = 5.0

# === Super Twisting Parameters ===
ST_K1 = 2.5
ST_K2 = 2.0

# === PID Gain ===
Kp = 0.025
Ki = 1e-3
Kd = 1e-4
Kaw = 0.5 * Ki

# === Anti-Wingup ===
Aw = 0.0

# === Levant Differentiator ===
Ld = 1.0e4

# === Helper ===
def sat(x):
    return np.clip(x, -1.0, 1.0)

def read_encoder(ser):
    ser.reset_input_buffer()
    ser.write(bytes([3]))
    line = ser.readline().decode('utf-8').strip()
    if not line:
        return None
    try:
        return int(line)
    except ValueError:
        return None

def home_actuator(ser):
    print("  Homing... ", end="", flush=True)
    ser.write(bytes([4]))
    time.sleep(0.1)
    ser.write(bytes([2, 180]))

    last_pos = None
    stall_start = time.time()
    while True:
        pos = read_encoder(ser)
        if pos is None:
            continue
        if last_pos is not None and pos == last_pos:
            if time.time() - stall_start > 1.5:
                break
        else:
            stall_start = time.time()
        last_pos = pos
        time.sleep(0.05)

    ser.write(bytes([5]))
    time.sleep(0.1)
    ser.write(bytes([4]))
    time.sleep(0.1)
    print("Done. Home = 0")

def move_to(ser, target, pwm=150):
    while True:
        pos = read_encoder(ser)
        if pos is None:
            continue
        error = target - pos
        if abs(error) < 50:
            break
        if error > 0:
            ser.write(bytes([1, pwm]))
        else:
            ser.write(bytes([2, pwm]))
        time.sleep(0.01)
    ser.write(bytes([5]))
    time.sleep(0.3)

# === Connect ===
try:
    ser = serial.Serial(port=PORT, baudrate=BAUD, timeout=0.1)
    time.sleep(2)
    ser.readline()
except Exception as e:
    print(f"Connection failed: {e}")
    exit()

# === User Setup ===
print("\n" + "=" * 50)
print("  URI 2026 — Linear Actuator Experiment")
print("=" * 50)

print("\nExperiment mode:")
print("  1 - Regulation (step response)")
print("  2 - Tracking (sine wave)")
mode = int(input("Selection: "))

print("\nController:")
print("  1 - PID")
print("  2 - Sliding Mode (tanh)")
print("  3 - Super Twisting")
ctr_sel = int(input("Selection: "))

if mode == 1:
    step_inches = float(input("\nStep size in inches (e.g. 1.5): "))
    step_counts = step_inches * COUNTS_PER_INCH
    step_time = 2.0
elif mode == 2:
    amp_inches = float(input("\nSine amplitude in inches (e.g. 1.0): "))
    period = float(input("Sine period in seconds (e.g. 8.0): "))
    amp_counts = amp_inches * COUNTS_PER_INCH

# === Home and position ===
home_actuator(ser)
one_inch = COUNTS_PER_INCH
print(f"  Moving to 1 inch ({one_inch} counts)... ", end="", flush=True)
move_to(ser, one_inch)
print("Done.")
time.sleep(0.5)

# === State Variables ===
t = 0.0
x0 = None
z1, z2, Iz = 0.0, 0.0, 0.0
Isq = 0.0
ISE, ISC = 0.0, 0.0

# === Labels ===
mode_names = {1: "Regulation", 2: "Tracking"}
ctrl_names = {1: "PID", 2: "SlidingMode", 3: "SuperTwisting"}

# === Write header ===
f = open("URI2026_Data.txt", "w")
f.write(f"# URI 2026 Linear Actuator Experiment\n")
f.write(f"# Mode: {mode_names[mode]}\n")
f.write(f"# Controller: {ctrl_names[ctr_sel]}\n")
if mode == 1:
    f.write(f"# Step: {step_inches} inches at t={step_time}s\n")
elif mode == 2:
    f.write(f"# Sine: {amp_inches} in amplitude, {period}s period\n")
f.write(f"# Ts: {Ts}s  Duration: {DURATION}s\n")
f.write(f"# t\te\tx\txd\tu\tdt\n")

# === Run ===
print(f"Running {DURATION}s...", end="", flush=True)

try:
    start_time = time.perf_counter()
    last_sample_time = start_time
    next_sample_time = start_time + Ts

    while t < DURATION:
        while time.perf_counter() < next_sample_time:
            pass

        current_time = time.perf_counter()
        dt = current_time - last_sample_time
        last_sample_time = current_time
        t = current_time - start_time
        next_sample_time += Ts

        ser.write(bytes([3]))
        line = ser.readline().decode('utf-8').strip()
        if not line:
            continue
        try:
            raw_x = int(line)
        except ValueError:
            continue

        if x0 is None:
            x0 = raw_x
        x = float(raw_x - x0)

        if mode == 1:
            xd = step_counts if t >= step_time else 0.0
        elif mode == 2:
            xd = amp_counts * np.sin(2.0 * np.pi * t / period)

        e = x - xd

        ez = e - z1
        dz1 = 1.5 * np.sqrt(Ld) * np.sqrt(abs(ez) + 1e-6) * np.sign(ez) + z2
        dz2 = 1.1 * Ld * np.sign(ez)
        z1 += dt * dz1
        z2 += dt * dz2
        Iz += dt * z1

        u = 0.0
        if ctr_sel == 1:
            u = -Kp * z1 - Kd * z2 - Ki * Iz + Aw
            Aw += Kaw * (np.clip(u, -1.0, 1.0) - u) * dt
			
        elif ctr_sel == 2:
            s = z2 + ALPHA_SM * z1
            u = -K_SM * np.tanh(LAMBDA_SM * s) / 255.0

        elif ctr_sel == 3:
            s = z2 + ALPHA_SM * z1
            Isq += dt * np.sign(s)
            u = -(ST_K1 * np.sqrt(abs(s)) * np.sign(s) + ST_K2 * Isq)
            u = u / 255.0

        u_sat = np.clip(u, -1.0, 1.0)

        pwm = int(255 * abs(u_sat))
        pwm = 0 if pwm < 10 else pwm
        if u_sat >= 0:
            ser.write(bytes([1, pwm]))
        else:
            ser.write(bytes([2, pwm]))

        ISE += dt * e ** 2
        ISC += dt * u ** 2
        f.write(f"{t:.4f}\t{e:.4f}\t{x:.4f}\t{xd:.4f}\t{u:.4f}\t{dt:.6f}\n")

    print(" Done.")

except KeyboardInterrupt:
    print("\n\nInterrupted by user.")

finally:
    home_actuator(ser)
    time.sleep(0.1)
    if 'ser' in locals() and ser.is_open:
        try:
            ser.write(bytes([5]))
            time.sleep(0.05)
            ser.write(bytes([6]))
        except:
            pass
        ser.close()
    f.close()
    print(f"\nResults: ISE = {ISE:.4f},  ISC = {ISC:.4f}")
    print("Data saved to URI2026_Data.txt")
    print("Run URI2026_Plot.py for plots.")
    print("=^..^=")
