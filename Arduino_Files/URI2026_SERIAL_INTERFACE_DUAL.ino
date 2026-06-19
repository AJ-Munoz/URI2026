// ============================================================
// URI 2026 — Dual Linear Actuator Firmware (2DoF TVC rig)
// Teensy 4.1 + 2x BTS7960 + 2x PA-HD2-4-2000-HS-12VDC
//
// PINOUT (from whiteboard):
//   Actuator X (q1, GREEN bundle)
//     pin 5  -> BTS RIGHT input
//     pin 6  -> BTS LEFT  input
//     pin 2  -> Enc A (green)
//     pin 3  -> Enc B (brown)
//   Actuator Y (q2, YELLOW bundle)
//     pin 9  -> BTS RIGHT input
//     pin 10 -> BTS LEFT  input
//     pin 7  -> Enc A (green)
//     pin 8  -> Enc B (brown)
//
// NOTE: "RIGHT" / "LEFT" are just the two BTS half-bridge inputs.
//       Which one is physically EXTEND is decided on the host after
//       a bench test (drive RIGHT, watch whether counts rise).
//
// Binary protocol over USB Serial (115200 baud):
//   1, pwm                 -> X drive RIGHT input at pwm (0-255)
//   2, pwm                 -> X drive LEFT  input at pwm
//   3, pwm                 -> Y drive RIGHT input at pwm
//   4, pwm                 -> Y drive LEFT  input at pwm
//   5                      -> print "encX encY" (one line, space sep)
//   6                      -> zero both encoders
//   7                      -> stop both motors
//   8                      -> software reset (ARM system reset)
//   9, dX, pX, dY, pY      -> ATOMIC: set both motors AND print "encX encY"
//                             dX/dY: 1 = RIGHT input, 0 = LEFT input
//                             pX/pY: pwm magnitude 0-255
//                             (this is what the control loop uses every tick)
//
// SAFETY: deadman timer. If no byte is received for > DEADMAN_MS,
//         both drivers are forced to 0. Protects against a hung host
//         or dropped USB while the top-heavy plate is commanded.
//
// Calibration: ~4115 counts/inch, ~16460 counts full 4" stroke.
// ============================================================

#include <Encoder.h>

// --- Motor (BTS7960) inputs ---
constexpr uint8_t PIN_X_R = 5;    // X right input
constexpr uint8_t PIN_X_L = 6;    // X left  input
constexpr uint8_t PIN_Y_R = 9;    // Y right input
constexpr uint8_t PIN_Y_L = 10;   // Y left  input

// --- Encoder inputs ---
constexpr uint8_t PIN_X_ENC_A = 2;   // green
constexpr uint8_t PIN_X_ENC_B = 3;   // brown
constexpr uint8_t PIN_Y_ENC_A = 7;   // green
constexpr uint8_t PIN_Y_ENC_B = 8;   // brown

constexpr uint16_t PWM_FREQ   = 20000;
constexpr uint32_t DEADMAN_MS = 100;  // stop if host silent this long

Encoder encX(PIN_X_ENC_A, PIN_X_ENC_B);
Encoder encY(PIN_Y_ENC_A, PIN_Y_ENC_B);

uint32_t lastCmdMs = 0;
bool stoppedByDeadman = false;

// --- Low-level drive ---
void motorStop() {
  analogWrite(PIN_X_R, 0); analogWrite(PIN_X_L, 0);
  analogWrite(PIN_Y_R, 0); analogWrite(PIN_Y_L, 0);
}

void driveX(uint8_t dir, uint8_t pwm) {
  if (dir) { analogWrite(PIN_X_L, 0); analogWrite(PIN_X_R, pwm); }
  else     { analogWrite(PIN_X_R, 0); analogWrite(PIN_X_L, pwm); }
}

void driveY(uint8_t dir, uint8_t pwm) {
  if (dir) { analogWrite(PIN_Y_L, 0); analogWrite(PIN_Y_R, pwm); }
  else     { analogWrite(PIN_Y_R, 0); analogWrite(PIN_Y_L, pwm); }
}

void printEncoders() {
  long x = encX.read();
  long y = encY.read();
  Serial.print(x); Serial.print(' '); Serial.println(y);
}

// blocking read of one byte (used for command arguments)
uint8_t readByte() {
  while (!Serial.available());
  return (uint8_t)Serial.read();
}

void setup() {
  Serial.begin(115200);

  pinMode(PIN_X_R, OUTPUT); pinMode(PIN_X_L, OUTPUT);
  pinMode(PIN_Y_R, OUTPUT); pinMode(PIN_Y_L, OUTPUT);
  motorStop();

  analogWriteFrequency(PIN_X_R, PWM_FREQ);
  analogWriteFrequency(PIN_X_L, PWM_FREQ);
  analogWriteFrequency(PIN_Y_R, PWM_FREQ);
  analogWriteFrequency(PIN_Y_L, PWM_FREQ);
  analogWriteResolution(8);

  lastCmdMs = millis();
  Serial.println(0);  // signal ready to host
}

void loop() {
  if (Serial.available() > 0) {
    int sel = Serial.read();
    lastCmdMs = millis();
    stoppedByDeadman = false;

    switch (sel) {
      case 1: { uint8_t p = readByte(); driveX(1, p); } break;  // X right
      case 2: { uint8_t p = readByte(); driveX(0, p); } break;  // X left
      case 3: { uint8_t p = readByte(); driveY(1, p); } break;  // Y right
      case 4: { uint8_t p = readByte(); driveY(0, p); } break;  // Y left

      case 5: printEncoders(); break;

      case 6: encX.write(0); encY.write(0); break;

      case 7: motorStop(); break;

      case 8:                                   // software reset
        motorStop();
        delay(2);
        SCB_AIRCR = 0x05FA0004;
        break;

      case 9: {                                 // atomic dual set + feedback
        uint8_t dX = readByte();
        uint8_t pX = readByte();
        uint8_t dY = readByte();
        uint8_t pY = readByte();
        driveX(dX, pX);
        driveY(dY, pY);
        printEncoders();
      } break;

      default: break;
    }
  }

  // --- Deadman: cut power if host has gone quiet ---
  if (!stoppedByDeadman && (millis() - lastCmdMs > DEADMAN_MS)) {
    motorStop();
    stoppedByDeadman = true;
  }
}
