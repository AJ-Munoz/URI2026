#!/usr/bin/python3
# ============================================================
# URI 2026 — Xbox Controller Axis/Button Finder
# Standalone. No motors, no serial. Just prints live values so
# you can read off the indices for URI2026_GAMEPAD_DUAL.py.
#
# Run:  python3 URI2026_GAMEPAD_TEST.py
# Quit: Ctrl-C
#
# How to use:
#   - Push the LEFT stick up/down  -> the axis whose value swings
#     is AX_LY   (up should read negative on a normal Xbox pad)
#   - Push the RIGHT stick up/down -> that axis is AX_RY
#   - Press A / B -> the buttons that light up are BTN_A / BTN_B
#   - Triggers REST at -1.0 (not 0). If a "stick" axis rests at
#     -1.0, it's a trigger -- do NOT use it for a stick.
# ============================================================

import os
os.environ.setdefault("SDL_VIDEODRIVER", "dummy")   # no window needed
import sys
import time
import pygame

def main():
    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        print("No controller detected. Plug in the Xbox pad and rerun.")
        sys.exit(1)

    js = pygame.joystick.Joystick(0)
    js.init()
    n_ax = js.get_numaxes()
    n_bt = js.get_numbuttons()
    n_hat = js.get_numhats()

    print("\n" + "=" * 56)
    print(f"  Controller : {js.get_name()}")
    print(f"  Axes: {n_ax}   Buttons: {n_bt}   Hats: {n_hat}")
    print("=" * 56)
    print("  Wiggle each stick and press each button.")
    print("  Watch which AXIS number changes / which BUTTON turns 1.")
    print("  (Triggers rest at -1.00 -- don't use those for sticks.)")
    print("  Ctrl-C to quit.\n")

    try:
        while True:
            pygame.event.pump()

            axes = [js.get_axis(i) for i in range(n_ax)]
            btns = [js.get_button(b) for b in range(n_bt)]

            ax_str = "  ".join(f"A{i}:{v:+.2f}" for i, v in enumerate(axes))
            pressed = [str(b) for b, on in enumerate(btns) if on]
            bt_str = ("BTN " + ",".join(pressed)) if pressed else "BTN -"

            sys.stdout.write("\r  " + ax_str + "   | " + bt_str + "        ")
            sys.stdout.flush()
            time.sleep(0.08)

    except KeyboardInterrupt:
        print("\n\n  Done. Copy the indices into URI2026_GAMEPAD_DUAL.py:")
        print("    AX_LY  = <left-stick-vertical axis>")
        print("    AX_RY  = <right-stick-vertical axis>")
        print("    BTN_A  = <A button>")
        print("    BTN_B  = <B button>\n")
    finally:
        pygame.quit()

if __name__ == "__main__":
    main()
