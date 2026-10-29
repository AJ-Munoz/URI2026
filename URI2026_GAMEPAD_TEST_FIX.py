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
#
# CHANGE: added a per-axis breakaway "kick" so a stick push from
# rest overcomes actuator static friction instead of requiring a
# double-tap. See KICK_PWM / KICK_TIME and the kick logic in the
# main loop.
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

# === Breakaway kick (fixes "sticky"/double-tap start) ===
# When an axis goes from stopped to commanded motion, PWM is forced
# up to KICK_PWM for KICK_TIME seconds to overcome the actuator's
# static friction, then it drops back to whatever the stick asks for.
KICK_PWM  = 140        # tune: lowest PWM that reliably breaks the actuator loose
KICK_TIME = 0.08       # seconds to hold the kick

# === Reversal cooldown (fixes stall on simultaneous direction flips) ===
# Reversing a loaded actuator's polarity draws a big current spike. Doing
# it on BOTH channels in the same instant (e.g. flicking the stick through
# a diagonal) can sag the shared 12V rail enough to glitch comms. When an
# axis's commanded direction flips while it was actively driving, force
# that axis to 0 PWM for REV_HOLD seconds before letting it drive the new
# direction -- this staggers the current draw instead of spiking both
# channels at once, and also re-arms the breakaway kick above.
REV_HOLD = 0.06        # seconds to force-stop an axis before flipping it

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

    # kick-start state, one entry per axis (0=X, 1=Y)
    was_zero = [True, True]
    kick_end = [0.0, 0.0]

    # reversal-cooldown state, one entry per axis (0=X, 1=Y)
    last_dir      = [None, None]   # last direction bit actually sent
    last_pwm_sent = [0, 0]         # last pwm actually sent
    rev_hold_end  = [0.0, 0.0]

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
                was_zero = [True, True]
                kick_end = [0.0, 0.0]
                last_dir = [None, None]
                last_pwm_sent = [0, 0]
                rev_hold_end = [0.0, 0.0]
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

            now = time.perf_counter()

            # --- reversal cooldown: never flip a loaded axis's direction
            #     instantly; force a brief stop first so both channels
            #     can't spike current at the same moment ---
            for i, (d, pwm) in enumerate(((dirX, pwmX), (dirY, pwmY))):
                if last_dir[i] is not None and d != last_dir[i] and last_pwm_sent[i] > 0:
                    rev_hold_end[i] = now + REV_HOLD
                last_dir[i] = d

            if now < rev_hold_end[0]:
                pwmX = 0
            if now < rev_hold_end[1]:
                pwmY = 0

            # --- breakaway kick: force higher pwm briefly on 0 -> moving edge ---
            for i, pwm in enumerate((pwmX, pwmY)):
                if pwm > 0 and was_zero[i]:
                    kick_end[i] = now + KICK_TIME
                was_zero[i] = (pwm == 0)

            if pwmX > 0 and now < kick_end[0]:
                pwmX = max(pwmX, KICK_PWM)
            if pwmY > 0 and now < kick_end[1]:
                pwmY = max(pwmY, KICK_PWM)

            newpos = drive(ser, pwmX, dirX, pwmY, dirY)
            if newpos: pos = newpos
            last_pwm_sent = [pwmX, pwmY]

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
