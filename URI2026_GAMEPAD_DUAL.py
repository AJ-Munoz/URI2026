#!/usr/bin/python3
# ============================================================
# URI 2026 — Dual Actuator GAMEPAD JOG (Xbox controller)
# Teensy 4.1 + 2x BTS7960 + 2x PA-HD2-4-2000-HS-12VDC
#
# CONTINUOUS velocity-style jogging: stick deflection sets PWM,
# release = stop (sent every frame). NOT a REPL -- the loop polls
# the controller and drives the motors every cycle, so "stop" is
# automatic (neutral sticks -> zero PWM that same frame) and B/A
# respond instantly even mid-motion.
#
#   Left  stick (vertical)  -> Actuator X (q1)   up = extend
#   Right stick (vertical)  -> Actuator Y (q2)   up = extend
#   A  -> HOMING sequence (hold to confirm; PLATE MUST BE OFF)
#   B  -> quit (stops first)
#
# Runs from a terminal on the Pi desktop via SDL dummy video driver
# (no GUI window). Reuses the SAME polarity flags as the jog tool.
# ============================================================

import os
os.environ.setdefault("SDL_VIDEODRIVER", "dummy")   # joystick without a window

import sys
import time
import serial
import pygame

# === Config (identical to URI2026_JOG_DUAL.py) ===
PORT = '/dev/ttyACM0'
BAUD = 115200
COUNTS_PER_INCH = 4115
STROKE_IN  = 2.0
STROKE_CNT = STROKE_IN * COUNTS_PER_INCH

# === Polarity (set these to match your locked jog-tool values) ===
EXTEND_DIR = [1, 1]    # [X, Y] BTS input that EXTENDS: 1=RIGHT, 0=LEFT
ENC_SIGN   = [+1, +1]  # [X, Y] +1 if extending makes counts INCREASE

# === Homing (PLATE OFF) ===
SEEK_PWM = 180
POS_PWM  = 150
STALL_S  = 1.5
HOME_EXTEND_CNT = int(2.0 * COUNTS_PER_INCH)

# === Gamepad / loop tuning ===
LOOP_HZ      = 60          # poll/drive rate
DEADZONE     = 0.12        # stick neutral band (fraction)
MAX_PWM      = 200         # full-stick PWM
MIN_PWM      = 25          # below this -> treat as zero (stiction floor)
INVERT_STICK = True        # most pads: up = -1; True makes up = extend
HOLD_A_S     = 1.0         # hold A this long to trigger homing
SLOWZONE_CNT = int(0.25 * COUNTS_PER_INCH)  # taper PWM near the stroke limit

# Xbox axis indices (SDL): 0=LX 1=LY 2=RX 3=RY (common); adjust if needed
AX_LY = 3
AX_RY = 2
BTN_A = 0
BTN_B = 1

# =====================================================================
def connect():
    try:
        s = serial.Serial(port=PORT, baudrate=BAUD, timeout=0.1)
        time.sleep(2)
        s.readline()
        return s
    except Exception as e:
        print(f"Connection failed: {e}")
        sys.exit(1)

def read_positions(ser):
    """cmd 5 -> [x1, x2] signed counts, or None."""
    ser.reset_input_buffer()
    ser.write(bytes([5]))
    parts = ser.readline().decode('utf-8', 'ignore').strip().split()
    if len(parts) != 2:
        return None
    try:
        return [ENC_SIGN[0]*int(parts[0]), ENC_SIGN[1]*int(parts[1])]
    except ValueError:
        return None

def set_motors(ser, pwmX, dirX, pwmY, dirY):
    """cmd 9 atomic set both + read; returns [x1, x2] or None."""
    ser.write(bytes([9, dirX, int(pwmX), dirY, int(pwmY)]))
    parts = ser.readline().decode('utf-8', 'ignore').strip().split()
    if len(parts) != 2:
        return None
    try:
        return [ENC_SIGN[0]*int(parts[0]), ENC_SIGN[1]*int(parts[1])]
    except ValueError:
        return None

def stop(ser):
    ser.write(bytes([7])); time.sleep(0.02)

def axis_to_pwm(val):
    """Stick value in [-1,1] -> (pwm, extend_bool). Deadzone + stiction floor."""
    if INVERT_STICK:
        val = -val
    if abs(val) < DEADZONE:
        return 0, True
    # rescale past deadzone so motion starts smoothly
    mag = (abs(val) - DEADZONE) / (1.0 - DEADZONE)
    pwm = int(MIN_PWM + mag * (MAX_PWM - MIN_PWM))
    return pwm, (val > 0)   # positive (up) = extend

def stroke_guard(pwm, extend, pos, ax):
    """Clamp PWM near/at the +/-2in limit, every frame."""
    if pwm == 0:
        return 0
    # hard stop at the wall
    if extend and pos >= STROKE_CNT:
        return 0
    if (not extend) and pos <= -STROKE_CNT:
        return 0
    # taper inside the slow zone
    if extend and pos > STROKE_CNT - SLOWZONE_CNT:
        frac = max(0.0, (STROKE_CNT - pos) / SLOWZONE_CNT)
        pwm = int(pwm * frac)
    if (not extend) and pos < -STROKE_CNT + SLOWZONE_CNT:
        frac = max(0.0, (pos + STROKE_CNT) / SLOWZONE_CNT)
        pwm = int(pwm * frac)
    return pwm if pwm >= MIN_PWM else 0

def home_sequence(ser):
    """Retract both to stops -> zero -> +2in -> zero. PLATE OFF only.
    Blocking; the jog loop is paused while this runs."""
    print("\n  [A] HOMING requested.")
    ans = input("  TOP PLATE DISCONNECTED? [y/N]: ").strip().lower()
    if ans != 'y':
        print("  Homing aborted.")
        return
    print("  Retracting both legs to stops...", flush=True)
    dxr, dyr = 1 - EXTEND_DIR[0], 1 - EXTEND_DIR[1]
    last = [None, None]; st = [time.time(), time.time()]; stalled = [False, False]
    while not all(stalled):
        pxr = 0 if stalled[0] else SEEK_PWM
        pyr = 0 if stalled[1] else SEEK_PWM
        pos = set_motors(ser, pxr, dxr, pyr, dyr)
        if pos is None:
            continue
        for i in (0, 1):
            if stalled[i]:
                continue
            if last[i] is not None and abs(pos[i] - last[i]) <= 2:
                if time.time() - st[i] > STALL_S:
                    stalled[i] = True
                    print(f"    leg {i+1} stalled.")
            else:
                st[i] = time.time()
            last[i] = pos[i]
        time.sleep(0.05)
    stop(ser); time.sleep(0.1)
    ser.write(bytes([6])); time.sleep(0.2)   # zero at retract datum
    print(f"  Extending +2\" ({HOME_EXTEND_CNT} cnt)...", flush=True)
    while True:
        pos = read_positions(ser)
        if pos is None:
            continue
        d1 = HOME_EXTEND_CNT - pos[0]; d2 = HOME_EXTEND_CNT - pos[1]
        if abs(d1) < 60 and abs(d2) < 60:
            break
        dx = EXTEND_DIR[0] if d1 >= 0 else (1 - EXTEND_DIR[0])
        dy = EXTEND_DIR[1] if d2 >= 0 else (1 - EXTEND_DIR[1])
        p1 = 0 if abs(d1) < 60 else POS_PWM
        p2 = 0 if abs(d2) < 60 else POS_PWM
        set_motors(ser, p1, dx, p2, dy)
        time.sleep(0.01)
    stop(ser); time.sleep(0.2)
    ser.write(bytes([6])); time.sleep(0.2)   # zero -> HOME
    print("  HOME set at 2\" (0.0 in). >>> RECONNECT PLATE before experiments.\n")

def status_line(pos, pwmX, pwmY):
    def bar(cnt):
        frac = max(-1.0, min(1.0, cnt / STROKE_CNT))
        n = int(abs(frac) * 5)
        fill = ('#' * n).ljust(5)
        warn = "!" if abs(cnt) > STROKE_CNT * 0.97 else " "
        return f"{cnt/COUNTS_PER_INCH:+.2f}in [{fill}]{warn}"
    sys.stdout.write(
        f"\r  X {bar(pos[0])}  PWM{pwmX:3d}   |   "
        f"Y {bar(pos[1])}  PWM{pwmY:3d}   | A=home B=quit   ")
    sys.stdout.flush()

# =====================================================================
def main():
    ser = connect()
    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        print("No gamepad detected. Plug in the Xbox controller.")
        ser.close(); sys.exit(1)
    js = pygame.joystick.Joystick(0)
    js.init()
    print("\n" + "=" * 52)
    print(f"  URI 2026 — GAMEPAD JOG  ({js.get_name()})")
    print("=" * 52)
    print("  Left stick = X actuator, Right stick = Y actuator")
    print("  A = homing (plate off),  B = quit\n")

    dt = 1.0 / LOOP_HZ
    a_down_since = None
    try:
        while True:
            t0 = time.perf_counter()
            pygame.event.pump()

            # --- buttons ---
            if js.get_button(BTN_B):
                break
            if js.get_button(BTN_A):
                if a_down_since is None:
                    a_down_since = time.time()
                elif time.time() - a_down_since >= HOLD_A_S:
                    stop(ser)
                    home_sequence(ser)      # blocking; loop paused
                    a_down_since = None
            else:
                a_down_since = None

            # --- sticks -> pwm ---
            pwmX, extX = axis_to_pwm(js.get_axis(AX_LY))
            pwmY, extY = axis_to_pwm(js.get_axis(AX_RY))

            # --- drive + read, with per-frame stroke clamp ---
            # first read to clamp against current position
            pos = read_positions(ser)
            if pos is None:
                continue
            pwmX = stroke_guard(pwmX, extX, pos[0], 0)
            pwmY = stroke_guard(pwmY, extY, pos[1], 1)
            dX = EXTEND_DIR[0] if extX else (1 - EXTEND_DIR[0])
            dY = EXTEND_DIR[1] if extY else (1 - EXTEND_DIR[1])
            newpos = set_motors(ser, pwmX, dX, pwmY, dY)
            if newpos is not None:
                pos = newpos

            status_line(pos, pwmX, pwmY)

            # --- pace the loop ---
            rem = dt - (time.perf_counter() - t0)
            if rem > 0:
                time.sleep(rem)

    except KeyboardInterrupt:
        pass
    finally:
        stop(ser)
        if ser.is_open:
            ser.close()
        pygame.quit()
        print("\n  stopped. =^..^=")

if __name__ == "__main__":
    main()
