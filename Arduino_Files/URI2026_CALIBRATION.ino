// ============================================================
// Linear Actuator Calibration — Teensy 4.1 + BTS7960
// PA-HD2-4-2000-HS-12VDC (Quadrature Hall Effect Encoder)
//
// What this does:
//   1. Retracts the actuator until it hits the limit switch
//   2. Records the encoder count at that limit
//   3. Extends the actuator until it hits the other limit
//   4. Records the encoder count at that limit
//   5. Prints the difference (total counts over 4" stroke)
//   6. Retracts back home
//
// How to use:
//   Open Serial Monitor at 115200 baud
//   Send 'g' to start calibration
//   Keep hands clear — it moves on its own!
//
// Wiring:
//   Pin 2 → BTS7960 RPWM  (extend direction)
//   Pin 3 → BTS7960 LPWM  (retract direction)
//   Pin 4 → Encoder Brown  (Hall A)
//   Pin 5 → Encoder Green  (Hall B)
//   Encoder Red → 3.3V
//   Encoder Black → GND
// ============================================================

#include <Encoder.h>

// --- Pin assignments ---
constexpr uint8_t  PIN_RPWM  = 2;
constexpr uint8_t  PIN_LPWM  = 3;
constexpr uint8_t  PIN_ENC_A = 4;
constexpr uint8_t  PIN_ENC_B = 5;

// --- Settings ---
constexpr uint16_t PWM_FREQ  = 20000;  // 20kHz — above human hearing
constexpr uint8_t  CAL_PWM   = 180;    // drive strength during calibration
constexpr uint16_t STALL_MS  = 1500;   // if encoder doesn't change for this long, we hit a limit

// --- Encoder object (handles quadrature counting automatically) ---
Encoder encoder(PIN_ENC_A, PIN_ENC_B);

// --- State tracking ---
long lastPos = 0;
unsigned long lastMoveTime = 0;
long retractCount = 0;
long extendCount = 0;

// --- State machine for the calibration sequence ---
enum State { WAITING, RETRACTING, EXTENDING, HOMING };
State state = WAITING;

// --- Motor control: positive = extend, negative = retract, 0 = stop ---
void driveMotor(int pwm) {
  pwm = constrain(pwm, -255, 255);
  if (pwm > 0) {
    analogWrite(PIN_LPWM, 0);
    analogWrite(PIN_RPWM, pwm);
  } else if (pwm < 0) {
    analogWrite(PIN_RPWM, 0);
    analogWrite(PIN_LPWM, -pwm);
  } else {
    analogWrite(PIN_RPWM, 0);
    analogWrite(PIN_LPWM, 0);
  }
}

// --- Returns true if the encoder hasn't moved for STALL_MS ---
bool stalled() {
  long pos = encoder.read();
  if (pos != lastPos) {
    lastPos = pos;
    lastMoveTime = millis();
    return false;
  }
  return (millis() - lastMoveTime > STALL_MS);
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}

  pinMode(PIN_RPWM, OUTPUT);
  pinMode(PIN_LPWM, OUTPUT);
  driveMotor(0);

  analogWriteFrequency(PIN_RPWM, PWM_FREQ);
  analogWriteFrequency(PIN_LPWM, PWM_FREQ);
  analogWriteResolution(8);

  Serial.println("Calibration. Send 'g' to start.");
}

void loop() {
  // --- Wait for 'g' command to begin ---
  if (Serial.available()) {
    char c = Serial.read();
    while (Serial.available()) Serial.read();
    if ((c == 'g' || c == 'G') && state == WAITING) {
      encoder.write(0);
      state = RETRACTING;
      driveMotor(-CAL_PWM);
      lastMoveTime = millis();
      lastPos = 0;
      Serial.println("Retracting to limit...");
    }
  }

  // --- Step 1: Retract until stall ---
  if (state == RETRACTING && stalled()) {
    driveMotor(0);
    retractCount = encoder.read();
    Serial.println("Retract limit reached.");
    delay(500);
    state = EXTENDING;
    driveMotor(CAL_PWM);
    lastMoveTime = millis();
    lastPos = encoder.read();
    Serial.println("Extending to limit...");
  }

  // --- Step 2: Extend until stall ---
  if (state == EXTENDING && stalled()) {
    driveMotor(0);
    extendCount = encoder.read();
    long total = extendCount - retractCount;
    Serial.println("Extend limit reached.");

    // --- Print results ---
    Serial.println("\n===== RESULTS =====");
    Serial.printf("Retract count: %ld\n", retractCount);
    Serial.printf("Extend count:  %ld\n", extendCount);
    Serial.printf("Total range:   %ld counts\n", total);
    Serial.printf("Counts/inch:   %ld\n", total / 4);
    Serial.printf("Counts/mm:     %.1f\n", total / 101.6);
    Serial.println("===================\n");

    delay(500);
    state = HOMING;
    driveMotor(-CAL_PWM);
    lastMoveTime = millis();
    lastPos = encoder.read();
    Serial.println("Returning home...");
  }

  // --- Step 3: Return to retract limit ---
  if (state == HOMING && stalled()) {
    driveMotor(0);
    Serial.printf("Home. Encoder: %ld\n", encoder.read());
    Serial.println("Done. Send 'g' to run again.");
    state = WAITING;
  }
}