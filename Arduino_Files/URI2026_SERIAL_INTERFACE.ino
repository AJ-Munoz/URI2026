// ============================================================
// URI 2026 — Linear Actuator Firmware
// Teensy 4.1 + BTS7960 + PA-HD2-4-2000-HS-12VDC
//
// Binary protocol over USB Serial (115200 baud):
//   1, pwm  → Extend at pwm (0-255)
//   2, pwm  → Retract at pwm (0-255)
//   3       → Print encoder position (ASCII line)
//   4       → Zero encoder
//   5       → Stop motor
//   6       → Software reset (ARM system reset)
//
// Encoder: Quadrature Hall Effect
//   Pin 4 → Brown  (Hall A)
//   Pin 5 → Green  (Hall B)
//   3.3V  → Red
//   GND   → Black
//
// Motor Driver:
//   Pin 2 → BTS7960 RPWM (extend)
//   Pin 3 → BTS7960 LPWM (retract)
//   R_EN + L_EN → 3.3V
//   VCC → 3.3V, GND → shared
//   B+/B- → Mean Well LRS-350-12
//   M+/M- → Actuator motor leads
//
// Calibration:
//   Full stroke:   ~16460 counts (4 inches)
//   Counts/inch:   ~4115
//   Counts/mm:     ~162
// ============================================================

#include <Encoder.h>

// --- Pin assignments ---
constexpr uint8_t  PIN_RPWM  = 2;
constexpr uint8_t  PIN_LPWM  = 3;
constexpr uint8_t  PIN_ENC_A = 4;
constexpr uint8_t  PIN_ENC_B = 5;
constexpr uint16_t PWM_FREQ  = 20000;

// --- Encoder (quadrature, handled by library + hardware interrupts) ---
Encoder encoder(PIN_ENC_A, PIN_ENC_B);

void setup() {
  Serial.begin(115200);

  pinMode(PIN_RPWM, OUTPUT);
  pinMode(PIN_LPWM, OUTPUT);
  analogWrite(PIN_RPWM, 0);
  analogWrite(PIN_LPWM, 0);

  analogWriteFrequency(PIN_RPWM, PWM_FREQ);
  analogWriteFrequency(PIN_LPWM, PWM_FREQ);
  analogWriteResolution(8);

  Serial.println(0);  // signal ready to host
}

void loop() {
  if (Serial.available() > 0) {
    int sel = Serial.read();

    // 1 = Extend
    if (sel == 1) {
      while (!Serial.available());
      int pwm = Serial.read();
      analogWrite(PIN_LPWM, 0);
      analogWrite(PIN_RPWM, pwm);
    }
    // 2 = Retract
    else if (sel == 2) {
      while (!Serial.available());
      int pwm = Serial.read();
      analogWrite(PIN_RPWM, 0);
      analogWrite(PIN_LPWM, pwm);
    }
    // 3 = Read encoder position
    else if (sel == 3) {
      noInterrupts();
      long pos = encoder.read();
      interrupts();
      Serial.println(pos);
    }
    // 4 = Zero encoder
    else if (sel == 4) {
      encoder.write(0);
    }
    // 5 = Stop motor
    else if (sel == 5) {
      analogWrite(PIN_RPWM, 0);
      analogWrite(PIN_LPWM, 0);
    }
    // 6 = Software reset (Teensy 4.1 ARM Cortex-M7)
    else if (sel == 6) {
      analogWrite(PIN_RPWM, 0);
      analogWrite(PIN_LPWM, 0);
      SCB_AIRCR = 0x05FA0004;
    }
  }
}