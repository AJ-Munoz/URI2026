// ============================================================
// Sine Wave + Encoder Test — Teensy 4.1 + BTS7960
// PA-HD2-4-2000-HS-12VDC (Quadrature Hall Effect Encoder)
//
// What this does:
//   Drives the actuator with a sinusoidal PWM pattern so it
//   smoothly extends and retracts. Prints the encoder position
//   so you can verify the sensor tracks the motion.
//
// How to use:
//   Open Serial Monitor at 115200 baud, line ending: Newline
//   g       → start the sine wave
//   s       → stop
//   a150    → set amplitude to 150 (range 0-255)
//   p6000   → set period to 6000ms (one full cycle)
//   z       → zero the encoder count
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
constexpr uint16_t PWM_FREQ = 20000;

// --- Encoder object ---
Encoder encoder(PIN_ENC_A, PIN_ENC_B);

// --- Sine wave parameters (adjustable at runtime) ---
int amplitude = 150;           // peak PWM value (0-255)
unsigned long periodMs = 6000; // one full sine cycle in milliseconds
bool running = false;
unsigned long startTime = 0;

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
  while (!Serial && millis() < 3000) {}

  pinMode(PIN_RPWM, OUTPUT);
  pinMode(PIN_LPWM, OUTPUT);
  driveMotor(0);

  analogWriteFrequency(PIN_RPWM, PWM_FREQ);
  analogWriteFrequency(PIN_LPWM, PWM_FREQ);
  analogWriteResolution(8);

  Serial.println("Sine+Encoder Test. g=go s=stop a<amp> p<ms> z=zero");
}

void loop() {
  // --- Handle serial commands ---
  if (Serial.available()) {
    char c = Serial.peek();

    if (c == 'g' || c == 'G') {
      Serial.read();
      while (Serial.available()) Serial.read();
      running = true;
      startTime = millis();
      Serial.printf("Running: amplitude=%d period=%dms\n", amplitude, periodMs);

    } else if (c == 's' || c == 'S') {
      Serial.read();
      while (Serial.available()) Serial.read();
      running = false;
      driveMotor(0);
      Serial.println("Stopped.");

    } else if (c == 'a' || c == 'A') {
      Serial.read();
      amplitude = constrain(Serial.parseInt(), 0, 255);
      while (Serial.available()) Serial.read();
      Serial.printf("Amplitude: %d\n", amplitude);

    } else if (c == 'p' || c == 'P') {
      Serial.read();
      periodMs = max(500, Serial.parseInt());  // minimum 500ms to protect the actuator
      while (Serial.available()) Serial.read();
      Serial.printf("Period: %dms\n", periodMs);

    } else if (c == 'z' || c == 'Z') {
      Serial.read();
      while (Serial.available()) Serial.read();
      encoder.write(0);
      Serial.println("Encoder zeroed.");

    } else {
      while (Serial.available()) Serial.read();
    }
  }

  // --- Sine wave generation ---
  if (running) {
    // Calculate where we are in the sine cycle
    float t = (millis() - startTime) / (float)periodMs;
    float sineVal = sin(2.0 * PI * t);
    int pwm = (int)(amplitude * sineVal);
    driveMotor(pwm);

    // Print PWM and encoder position every 100ms
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 100) {
      Serial.printf("pwm: %4d  enc: %ld\n", pwm, encoder.read());
      lastPrint = millis();
    }

  } else {
    // Print encoder position every 500ms when idle
    static unsigned long lastIdle = 0;
    if (millis() - lastIdle > 500) {
      Serial.printf("enc: %ld\n", encoder.read());
      lastIdle = millis();
    }
  }
}