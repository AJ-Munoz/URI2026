// ============================================================
// PWM Test — Teensy 4.1 + BTS7960
// PA-HD2-4-2000-HS-12VDC
//
// What this does:
//   Sends a PWM signal to the motor driver to move the actuator.
//   Positive values extend, negative values retract.
//
// How to use:
//   Open Serial Monitor at 115200 baud, line ending: Newline
//   Type a number from -255 to 255 and hit enter
//     100  → extend at 100/255 power
//    -35   → retract at 35/255 power
//      0   → stop
//
// Wiring:
//   Pin 2 → BTS7960 RPWM  (extend direction)
//   Pin 3 → BTS7960 LPWM  (retract direction)
//   BTS7960 R_EN + L_EN → 3.3V (enables the driver)
//   BTS7960 VCC → 3.3V, GND → shared with Teensy
//   BTS7960 B+/B- → 12V power supply
//   BTS7960 M+/M- → actuator motor leads
// ============================================================

// --- Pin assignments ---
constexpr uint8_t  PIN_RPWM = 2;
constexpr uint8_t  PIN_LPWM = 3;

// --- PWM frequency: 20kHz is above human hearing, so no whine ---
constexpr uint16_t PWM_FREQ = 20000;

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

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}  // wait for USB serial connection

  pinMode(PIN_RPWM, OUTPUT);
  pinMode(PIN_LPWM, OUTPUT);
  driveMotor(0);  // start with motor stopped

  // Set PWM frequency and resolution
  analogWriteFrequency(PIN_RPWM, PWM_FREQ);
  analogWriteFrequency(PIN_LPWM, PWM_FREQ);
  analogWriteResolution(8);  // 0-255 range

  Serial.println("PWM Test Ready. Send value -255 to 255.");
}

void loop() {
  if (Serial.available()) {
    // Read the number the user typed
    int val = Serial.parseInt();
    // Clear any leftover characters (like newline)
    while (Serial.available()) Serial.read();

    driveMotor(val);
    Serial.printf("PWM set to: %d\n", val);
  }
}