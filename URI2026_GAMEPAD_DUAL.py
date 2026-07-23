#!/usr/bin/python3
# ============================================================
# URI 2026 — Dual Actuator GAMEPAD JOG (Xbox, right stick)
# Teensy 4.1 + 2x BTS7960 + 2x PA-HD2-4-2000-HS-12VDC
#
# Simple velocity-style jog. Read stick -> scale to +/-255 ->
# sign picks direction. Release stick = stop (sent every frame).
#
#   Right stick vertical (axis 3) -> X actuator   (inverted)
#   Right stick horizontal(axis 4) -> Y actuator
#   A -> homing (PLATE OFF)        B -> quit
#
# Runs from a terminal (SDL dummy video). Set EXTEND_DIR/ENC_SIGN
# to your locked values.
# ============================================================

import os
os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
import sys, time, serial, pygame

# === Serial / calibration ===
PORT = '/dev/ttyACM0'
BAUD = 115200
COUNTS_PER_INCH = 4115
STROKE_CNT = int(2.0 * COUNTS_PER_INCH)        # +/- travel about home

# === Polarity (your locked values) ===
EXTEND_DIR = [1, 1]    # [X, Y] BTS input that EXTENDS: 1=RIGHT, 0=LEFT
ENC_SIGN   = [+1, +1]  # [X, Y] +1 if extending raises counts

# === Gamepad mapping ===
AXIS_X = 1              # right-stick vertical   -> X actuator
AXIS_Y = 0             # right-stick horizontal -> Y actuator
SIGN_X = -1            # X is inverted (up should extend)
SIGN_Y = +1
MAX_PWM = 255          # full-stick PWM (lower for gentler jogging)
FLOOR   = 15           # ignore tiny stick noise below this PWM
BTN_A   = 0            # homing
BTN_B   = 1            # quit
LOOP_HZ = 60

# === Homing (PLATE OFF) ===
SEEK_PWM, POS_PWM, STALL_S = 180, 150, 1.5
HOME_EXTEND_CNT = int(2.0 * COUNTS_PER_INCH)

# =====================================================================
def connect():
    try:
        s = serial.Serial(PORT, BAUD, timeout=0.1)
        time.sleep(2); s.readline()
        return s
    except Exception as e:
        print(f"Connection failed: {e}"); sys.exit(1)

def read_pos(ser):
    ser.reset_input_buffer(); ser.write(bytes([5]))
    p = ser.readline().decode('utf-8', 'ignore').strip().split()
    if len(p) != 2: return None
    try: return [ENC_SIGN[0]*int(p[0]), ENC_SIGN[1]*int(p[1])]
    except ValueError: return None

def drive(ser, pwmX, dirX, pwmY, dirY):
    ser.write(bytes([9, dirX, int(pwmX), dirY, int(pwmY)]))
    p = ser.readline().decode('utf-8', 'ignore').strip().split()
    if len(p) != 2: return None
    try: return [ENC_SIGN[0]*int(p[0]), ENC_SIGN[1]*int(p[1])]
    except ValueError: return None

def stop(ser):
    ser.write(bytes([7])); time.sleep(0.02)

def home(ser):
    print("\n  [A] HOMING.")
    if input("  TOP PLATE DISCONNECTED? [y/N]: ").strip().lower() != 'y':
        print("  aborted."); return
    print("  retracting to stops...", flush=True)
    dxr, dyr = 1-EXTEND_DIR[0], 1-EXTEND_DIR[1]
    last=[None,None]; t=[time.time(),time.time()]; done=[False,False]
    while not all(done):
        pos = drive(ser, 0 if done[0] else SEEK_PWM, dxr,
                         0 if done[1] else SEEK_PWM, dyr)
        if pos is None: continue
        for i in (0,1):
            if done[i]: continue
            if last[i] is not None and abs(pos[i]-last[i])<=2:
                if time.time()-t[i] > STALL_S: done[i]=True
            else: t[i]=time.time()
            last[i]=pos[i]
        time.sleep(0.05)
    stop(ser); time.sleep(0.1)
    ser.write(bytes([6])); time.sleep(0.2)         # zero at retract datum
    print(f"  extending +2\" ({HOME_EXTEND_CNT} cnt)...", flush=True)
    while True:
        pos = read_pos(ser)
        if pos is None: continue
        d1, d2 = HOME_EXTEND_CNT-pos[0], HOME_EXTEND_CNT-pos[1]
        if abs(d1)<60 and abs(d2)<60: break
        drive(ser, 0 if abs(d1)<60 else POS_PWM, EXTEND_DIR[0] if d1>=0 else 1-EXTEND_DIR[0],
                   0 if abs(d2)<60 else POS_PWM, EXTEND_DIR[1] if d2>=0 else 1-EXTEND_DIR[1])
        time.sleep(0.01)
    stop(ser); time.sleep(0.2)
    ser.write(bytes([6])); time.sleep(0.2)         # zero -> HOME
    print("  HOME set at 2\" (0.0 in). >>> RECONNECT PLATE.\n")

# =====================================================================
def main():
    ser = connect()
    pygame.init(); pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        print("No controller detected."); ser.close(); sys.exit(1)
    js = pygame.joystick.Joystick(0); js.init()
    print("\n" + "="*52)
    print(f"  URI 2026 — GAMEPAD JOG  ({js.get_name()})")
    print("  Right stick: vert=X (inverted), horiz=Y |  A=home  B=quit")
    print("="*52 + "\n")

    pos = read_pos(ser) or [0, 0]
    dt = 1.0 / LOOP_HZ
    try:
        while True:
            t0 = time.perf_counter()
            pygame.event.pump()

            if js.get_button(BTN_B):
                break
            if js.get_button(BTN_A):
                stop(ser); home(ser)
                # resume-safe: wait until stick released so it doesn't lurch
                while abs(js.get_axis(AXIS_X)) > 0.2 or abs(js.get_axis(AXIS_Y)) > 0.2:
                    pygame.event.pump(); time.sleep(0.05)
                pos = read_pos(ser) or pos
                continue

            # --- stick -> signed magnitude -> pwm + direction ---
            vx = js.get_axis(AXIS_X) * SIGN_X      # >0 means extend
            vy = js.get_axis(AXIS_Y) * SIGN_Y
            pwmX = int(abs(vx) * MAX_PWM); pwmX = 0 if pwmX < FLOOR else min(pwmX, MAX_PWM)
            pwmY = int(abs(vy) * MAX_PWM); pwmY = 0 if pwmY < FLOOR else min(pwmY, MAX_PWM)
            extX, extY = (vx > 0), (vy > 0)

            # --- stroke hard-stop at +/-2" (uses last position) ---
            if pwmX and ((extX and pos[0] >= STROKE_CNT) or (not extX and pos[0] <= -STROKE_CNT)):
                pwmX = 0
            if pwmY and ((extY and pos[1] >= STROKE_CNT) or (not extY and pos[1] <= -STROKE_CNT)):
                pwmY = 0

            dirX = EXTEND_DIR[0] if extX else 1-EXTEND_DIR[0]
            dirY = EXTEND_DIR[1] if extY else 1-EXTEND_DIR[1]
            newpos = drive(ser, pwmX, dirX, pwmY, dirY)
            if newpos: pos = newpos

            sys.stdout.write(f"\r  X {pos[0]/COUNTS_PER_INCH:+.2f}in pwm{pwmX:3d}   "
                             f"Y {pos[1]/COUNTS_PER_INCH:+.2f}in pwm{pwmY:3d}   A=home B=quit ")
            sys.stdout.flush()

            rem = dt - (time.perf_counter() - t0)
            if rem > 0: time.sleep(rem)

    except KeyboardInterrupt:
        pass
    finally:
        stop(ser)
        if ser.is_open: ser.close()
        pygame.quit()
        print("\n  stopped. =^..^=")

if __name__ == "__main__":
    main()
