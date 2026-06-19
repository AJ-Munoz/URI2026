#!/usr/bin/python3
# ============================================================
# URI 2026 — Dual Actuator JOG / GO-TO-POSITION bench tool
# Teensy 4.1 + 2x BTS7960 + 2x PA-HD2-4-2000-HS-12VDC
#
# Prompt-driven. Move one actuator at a time to a target position
# and hold. Positions are in INCHES relative to HOME, where
#   home (0.0 in) == the 2" extended pose, travel +/- 2".
#
# Because the actuators may be anywhere at startup, you can DECLARE
# the current position of each ("assume") before commanding moves,
# or zero both here to define home.
#
# Commands at the > prompt:
#   p              print both positions (counts + inches)
#   z              zero both encoders HERE  -> defines home (0.0 in)
#   a <ax> <in>    ASSUME axis ax (1|2) is currently at <in> inches
#   <ax> <in>      GO axis ax (1|2) to <in> inches, then hold
#   t <ax>         polarity TEST: pulse axis both ways, report deltas
#   s              stop both motors
#   j <ax> <in>    jog: move axis by a RELATIVE <in> from where it is
#   r              reset Teensy
#   q              quit (stops first)
#
# Uses firmware as-is: cmd 9 (atomic set + read), 5 (read), 6 (zero), 8 (reset).
# ============================================================

import serial
import time
import numpy as np

# === Config ===
PORT = '/dev/ttyACM0'
BAUD = 115200
COUNTS_PER_INCH = 4115
STROKE_IN  = 2.0
STROKE_CNT = STROKE_IN * COUNTS_PER_INCH

# === Polarity (the things this tool helps you discover; use 't <ax>') ===
EXTEND_DIR = [1, 1]    # [X, Y] BTS input that EXTENDS: 1=RIGHT, 0=LEFT
ENC_SIGN   = [+1, +1]  # [X, Y] +1 if extending makes counts INCREASE

# === Go-to loop ===
GOTO_TOL_CNT = 40       # within this -> done
KP_GOTO      = 0.06     # counts -> pwm (proportional)
PWM_MIN      = 60        # overcome stiction
PWM_MAX      = 200
RUNAWAY_CNT  = 400      # if we move this far the WRONG way -> abort (bad polarity)

# === Homing (PLATE MUST BE DISCONNECTED) ===
SEEK_PWM = 180          # retract toward the full-retract hard stop
POS_PWM  = 150          # extend to the 2" home pose
STALL_S  = 1.5          # counts unchanged this long under power = stalled
HOME_EXTEND_CNT = int(2.0 * COUNTS_PER_INCH)   # +2" from full retract

# software offset so reported = ENC_SIGN*raw + offset  [counts]
offset = [0, 0]

# =====================================================================
def connect():
    try:
        s = serial.Serial(port=PORT, baudrate=BAUD, timeout=0.1)
        time.sleep(2)
        s.readline()
        return s
    except Exception as e:
        print(f"Connection failed: {e}")
        raise SystemExit(1)

def read_raw(ser):
    ser.reset_input_buffer()
    ser.write(bytes([5]))
    parts = ser.readline().decode('utf-8', 'ignore').strip().split()
    if len(parts) != 2:
        return None
    try:
        return int(parts[0]), int(parts[1])
    except ValueError:
        return None

def positions(ser):
    """Return [pos1_cnt, pos2_cnt] with sign + offset applied, or None."""
    raw = read_raw(ser)
    if raw is None:
        return None
    return [ENC_SIGN[0]*raw[0] + offset[0],
            ENC_SIGN[1]*raw[1] + offset[1]]

def set_motors(ser, pwmX, dirX, pwmY, dirY):
    """Atomic set; returns positions after."""
    ser.write(bytes([9, dirX, int(pwmX), dirY, int(pwmY)]))
    parts = ser.readline().decode('utf-8', 'ignore').strip().split()
    if len(parts) != 2:
        return None
    try:
        raw = (int(parts[0]), int(parts[1]))
    except ValueError:
        return None
    return [ENC_SIGN[0]*raw[0] + offset[0],
            ENC_SIGN[1]*raw[1] + offset[1]]

def stop(ser):
    ser.write(bytes([7]))
    time.sleep(0.02)

def show(ser):
    p = positions(ser)
    if p is None:
        print("  (read error)")
        return
    for i in (0, 1):
        in_ = p[i] / COUNTS_PER_INCH
        flag = "  <-- PAST +/-2in!" if abs(p[i]) > STROKE_CNT else ""
        print(f"  Actuator {i+1}: {p[i]:+7d} cnt  ({in_:+.3f} in){flag}")

def assume(ax, val_in, ser):
    """Declare axis ax currently at val_in inches (set software offset)."""
    raw = read_raw(ser)
    if raw is None:
        print("  read error"); return
    target_cnt = val_in * COUNTS_PER_INCH
    offset[ax] = target_cnt - ENC_SIGN[ax]*raw[ax]
    print(f"  Actuator {ax+1} declared at {val_in:+.3f} in.")

def goto(ser, ax, target_in):
    """Drive ONE actuator to target_in (other held at 0), then hold."""
    target_cnt = np.clip(target_in * COUNTS_PER_INCH, -STROKE_CNT, STROKE_CNT)
    if abs(target_in) > STROKE_IN + 1e-6:
        print(f"  target clamped to +/-{STROKE_IN}in")

    p = positions(ser)
    if p is None:
        print("  read error"); return
    start_err = target_cnt - p[ax]
    best_abs = abs(start_err)

    print(f"  Going A{ax+1} -> {target_cnt/COUNTS_PER_INCH:+.3f} in ...", end="", flush=True)
    t0 = time.time()
    while True:
        err = target_cnt - p[ax]
        if abs(err) < GOTO_TOL_CNT:
            break
        # runaway / wrong-polarity guard
        if abs(err) > best_abs + RUNAWAY_CNT:
            stop(ser)
            print(f"\n  !! moving AWAY from target (err {err:+.0f} cnt).")
            print(f"     Likely EXTEND_DIR/ENC_SIGN wrong for axis {ax+1}. Aborted.")
            return
        best_abs = min(best_abs, abs(err))
        if time.time() - t0 > 15:
            stop(ser)
            print("\n  !! timeout, aborted."); return

        pwm = int(np.clip(KP_GOTO*abs(err), PWM_MIN, PWM_MAX))
        extend = err > 0                                   # need more length
        d = EXTEND_DIR[ax] if extend else (1 - EXTEND_DIR[ax])
        # build atomic command: moving axis active, other axis 0
        if ax == 0:
            p = set_motors(ser, pwm, d, 0, EXTEND_DIR[1])
        else:
            p = set_motors(ser, 0, EXTEND_DIR[0], pwm, d)
        if p is None:
            continue
        # stroke safety on the moving axis
        if abs(p[ax]) > STROKE_CNT:
            stop(ser)
            print(f"\n  !! stroke limit hit ({p[ax]:+d} cnt). Aborted."); return
        time.sleep(0.005)

    stop(ser)
    print(f" done ({p[ax]/COUNTS_PER_INCH:+.3f} in). Holding (leadscrew self-locks).")

def polarity_test(ser, ax, pulse_pwm=110, pulse_s=0.25):
    """Pulse axis each way; report encoder delta so you can set the flags."""
    print(f"  Polarity test, axis {ax+1} (pulses {pulse_s*1000:.0f} ms each way)...")
    for label, d in [("RIGHT-input(cmd extend dir)", EXTEND_DIR[ax]),
                     ("LEFT-input", 1 - EXTEND_DIR[ax])]:
        p0 = positions(ser)[ax]
        t0 = time.time()
        while time.time() - t0 < pulse_s:
            if ax == 0:
                set_motors(ser, pulse_pwm, d, 0, EXTEND_DIR[1])
            else:
                set_motors(ser, 0, EXTEND_DIR[0], pulse_pwm, d)
        stop(ser)
        time.sleep(0.2)
        p1 = positions(ser)[ax]
        dlt = p1 - p0
        print(f"    {label}: delta = {dlt:+d} cnt "
              f"({'counts up' if dlt>0 else 'counts down'})")
    print("    -> EXTEND should give +counts. If reversed, flip EXTEND_DIR or ENC_SIGN.")

def home_sequence(ser):
    """Retract BOTH legs to their stops -> zero -> extend +2" -> zero.
    Defines a repeatable home (2" extended = 0.0 in). PLATE OFF only."""
    ans = input("  TOP PLATE DISCONNECTED? Homing drives BOTH legs to their "
                "stops. [y/N]: ").strip().lower()
    if ans != 'y':
        print("  Homing aborted (reconnect-safe).")
        return

    # --- retract both to hard stops, per-leg stall detection ---
    print("  Retracting both legs to full-retract stops...", flush=True)
    dxr, dyr = 1 - EXTEND_DIR[0], 1 - EXTEND_DIR[1]
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
            pos = [ENC_SIGN[0]*int(parts[0]), ENC_SIGN[1]*int(parts[1])]
        except ValueError:
            continue
        for i in (0, 1):
            if stalled[i]:
                continue
            if last[i] is not None and abs(pos[i] - last[i]) <= 2:
                if time.time() - stall_t[i] > STALL_S:
                    stalled[i] = True
                    print(f"    leg {i+1} stalled at stop.")
            else:
                stall_t[i] = time.time()
            last[i] = pos[i]
        time.sleep(0.05)
    stop(ser)
    time.sleep(0.1)

    # --- zero at retract datum, then extend both +2" ---
    ser.write(bytes([6])); time.sleep(0.2)
    offset[0] = offset[1] = 0
    print(f"  Retract datum = 0. Extending +2\" ({HOME_EXTEND_CNT} cnt)...", flush=True)
    while True:
        enc = positions(ser)
        if enc is None:
            continue
        d1 = HOME_EXTEND_CNT - enc[0]
        d2 = HOME_EXTEND_CNT - enc[1]
        if abs(d1) < 60 and abs(d2) < 60:
            break
        dx = EXTEND_DIR[0] if d1 >= 0 else (1 - EXTEND_DIR[0])
        dy = EXTEND_DIR[1] if d2 >= 0 else (1 - EXTEND_DIR[1])
        p1 = 0 if abs(d1) < 60 else POS_PWM
        p2 = 0 if abs(d2) < 60 else POS_PWM
        ser.write(bytes([9, dx, p1, dy, p2]))
        ser.readline()
        time.sleep(0.01)
    stop(ser)
    time.sleep(0.2)

    # --- zero here -> this is HOME (0.0 in) ---
    ser.write(bytes([6])); time.sleep(0.2)
    offset[0] = offset[1] = 0
    print("  HOME established at 2\" (0.0 in). New zero set.")
    print("  >>> RECONNECT THE TOP PLATE before running experiments.")
    show(ser)

# =====================================================================
def main():
    ser = connect()
    print("\n" + "="*52)
    print("  URI 2026 — Dual Actuator GO-TO-POSITION")
    print("="*52)
    print("  Home (0.0 in) = 2\" extended.  Travel +/- 2 in.")
    print("  Type 'help' for commands.\n")
    show(ser)

    HELP = (
        "  p            print positions\n"
        "  home         FULL HOMING: retract both to stops, +2\", set new zero (PLATE OFF)\n"
        "  z            zero both here (define home = 0.0 in)\n"
        "  a <ax> <in>  ASSUME axis is currently at <in> inches\n"
        "  <ax> <in>    GO axis (1|2) to <in> inches\n"
        "  j <ax> <in>  JOG axis by relative <in>\n"
        "  t <ax>       polarity test on axis\n"
        "  s            stop both\n"
        "  r            reset Teensy\n"
        "  q            quit"
    )

    try:
        while True:
            cmd = input("> ").strip().split()
            if not cmd:
                continue
            c = cmd[0].lower()

            if c in ('q', 'quit', 'exit'):
                break
            elif c in ('help', 'h', '?'):
                print(HELP)
            elif c == 'p':
                show(ser)
            elif c == 's':
                stop(ser); print("  stopped.")
            elif c == 'z':
                ser.write(bytes([6])); time.sleep(0.2)
                offset[0] = offset[1] = 0
                print("  zeroed both. Home set here (0.0 in)."); show(ser)
            elif c in ('home', 'hm'):
                home_sequence(ser)
            elif c == 'r':
                stop(ser); ser.write(bytes([8])); time.sleep(0.3)
                print("  Teensy reset (encoders -> 0). Re-declare positions if needed.")
                offset[0] = offset[1] = 0
            elif c == 'a' and len(cmd) == 3:
                assume(int(cmd[1])-1, float(cmd[2]), ser)
            elif c == 't' and len(cmd) == 2:
                polarity_test(ser, int(cmd[1])-1)
            elif c == 'j' and len(cmd) == 3:
                ax = int(cmd[1])-1
                p = positions(ser)
                if p: goto(ser, ax, p[ax]/COUNTS_PER_INCH + float(cmd[2]))
            elif c in ('1', '2') and len(cmd) == 2:
                goto(ser, int(c)-1, float(cmd[1]))
            else:
                print("  ? type 'help'")
    except KeyboardInterrupt:
        print("\n  interrupted.")
    finally:
        stop(ser)
        if ser.is_open:
            ser.close()
        print("  closed. =^..^=")

if __name__ == "__main__":
    main()
