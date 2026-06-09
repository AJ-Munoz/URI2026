// ============================================================
// PID Position Control — Teensy 4.1 + BTS7960
// PA-HD2-4-2000-HS-12VDC (Quadrature Hall Effect Encoder)
//
// What this does:
//   Drives the actuator to a target encoder position and holds
//   it there using a PID controller. The controller calculates
//   the error (target - current), and adjusts the motor PWM
//   to minimize that error.
//
//   PID breakdown:
//     P (proportional) — reacts to current error
//     I (integral)     — reacts to accumulated past error
//     D (derivative)   — reacts to rate of change of error
//
// How to use:
//   Open Serial Monitor at 115200 baud, line ending: Newline
//   g 8000   → go to encoder position 8000
//   g 0      → go back to zero
//   s        → stop and disable PID
//   z        → zero encoder at current position
//   h        → home: retract to limit, zero, ready
//   kp 0.05  → set proportional gain
//   ki 0.001 → set integral gain
//   kd 0.1   → set derivative gain
//   ?        → print current gains and position
//
// Calibration data (from our measurements):
//   Full stroke:    ~16460 counts (4 inches)
//   Counts/inch:    ~4115
//   Counts/mm:      ~162
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

// --- Motor settings ---
constexpr uint16_t PWM_FREQ   = 20000;
constexpr uint8_t  MIN_PWM    = 50;    // minimum PWM to overcome stiction
constexpr uint8_t  MAX_PWM    = 255;

// --- Stall detection ---
constexpr uint16_t STALL_MS   = 1500;

// --- PID gains (tune these!) ---
float Kp = 0.02;    // start conservative
float Ki = 0.0001;
float Kd = 0.001;

// --- PID state ---
long  targetPos     = 0;
float integral      = 0;
long  prevError     = 0;
unsigned long prevTime = 0;
bool  pidActive     = false;
constexpr int POS_TOLERANCE = 15;    // close enough (encoder counts)
constexpr float INTEGRAL_MAX = 50000; // anti-windup clamp

// --- Homing state ---
bool homing = false;
long lastHomingPos = 0;
unsigned long lastHomingMove = 0;

// --- Encoder ---
Encoder encoder(PIN_ENC_A, PIN_ENC_B);

// --- Motor control ---
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

// --- Parse a float after a prefix like "kp " ---
float parseFloat() {
  return Serial.parseFloat();
}

void printStatus() {
  long pos = encoder.read();
  Serial.println("\n--- Status ---");
  Serial.printf("Position:  %ld\n", pos);
  Serial.printf("Target:    %ld\n", targetPos);
  Serial.printf("PID:       %s\n", pidActive ? "ACTIVE" : "off");
  Serial.printf("Kp=%.4f  Ki=%.5f  Kd=%.3f\n", Kp, Ki, Kd);
  Serial.printf("Counts/inch: 4115\n");
  Serial.printf("Position in inches: %.2f\n", pos / 4115.0);
  Serial.println("--------------\n");
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

  prevTime = millis();

  Serial.println("PID Controller Ready.");
  Serial.println("Commands: g<pos> s z h kp<val> ki<val> kd<val> ?");
}

void loop() {
  // --- Handle serial commands ---
  if (Serial.available()) {
    char c = Serial.peek();

    if (c == 'g' || c == 'G') {
      // Go to position
      Serial.read();
      targetPos = Serial.parseInt();
      while (Serial.available()) Serial.read();
      integral = 0;       // reset integral on new target
      prevError = 0;
      prevTime = millis();
      pidActive = true;
      homing = false;
      Serial.printf("Target: %ld\n", targetPos);

    } else if (c == 's' || c == 'S') {
      // Stop
      Serial.read();
      while (Serial.available()) Serial.read();
      pidActive = false;
      homing = false;
      driveMotor(0);
      Serial.println("Stopped.");

    } else if (c == 'z' || c == 'Z') {
      // Zero encoder
      Serial.read();
      while (Serial.available()) Serial.read();
      encoder.write(0);
      targetPos = 0;
      integral = 0;
      prevError = 0;
      Serial.println("Encoder zeroed.");

    } else if (c == 'h' || c == 'H') {
      // Home
      Serial.read();
      while (Serial.available()) Serial.read();
      pidActive = false;
      homing = true;
      driveMotor(-180);
      lastHomingPos = encoder.read();
      lastHomingMove = millis();
      Serial.println("Homing...");

    } else if (c == 'k' || c == 'K') {
      // Gain adjustment: kp, ki, or kd
      Serial.read();  // consume 'k'
      char which = Serial.read();  // 'p', 'i', or 'd'
      float val = Serial.parseFloat();
      while (Serial.available()) Serial.read();
      if (which == 'p' || which == 'P') {
        Kp = val;
        Serial.printf("Kp = %.4f\n", Kp);
      } else if (which == 'i' || which == 'I') {
        Ki = val;
        integral = 0;  // reset integral when gain changes
        Serial.printf("Ki = %.5f\n", Ki);
      } else if (which == 'd' || which == 'D') {
        Kd = val;
        Serial.printf("Kd = %.3f\n", Kd);
      }

    } else if (c == '?') {
      Serial.read();
      while (Serial.available()) Serial.read();
      printStatus();

    } else {
      while (Serial.available()) Serial.read();
    }
  }

  // --- Homing logic ---
  if (homing) {
    long pos = encoder.read();
    if (pos != lastHomingPos) {
      lastHomingPos = pos;
      lastHomingMove = millis();
    }
    if (millis() - lastHomingMove > STALL_MS) {
      driveMotor(0);
      encoder.write(0);
      targetPos = 0;
      integral = 0;
      prevError = 0;
      homing = false;
      Serial.println("Home set. Encoder zeroed.");
    }
    return;
  }

  // --- PID control loop ---
  if (pidActive) {
    unsigned long now = millis();
    float dt = (now - prevTime) / 1000.0;  // delta time in seconds
    prevTime = now;

    // Avoid division by zero on first pass
    if (dt <= 0) return;

    long pos = encoder.read();
    long error = targetPos - pos;

    // --- If close enough, stop and hold ---
    if (abs(error) <= POS_TOLERANCE) {
      driveMotor(0);
      integral = 0;

      // Print arrival once
      static bool arrived = false;
      if (!arrived) {
        Serial.printf("Arrived at %ld (target %ld)\n", pos, targetPos);
        arrived = true;
      }
      return;
    }

    // Reset arrived flag when moving
    static bool arrived = false;
    arrived = false;

    // --- P term ---
    float pTerm = Kp * error;

    // --- I term with anti-windup ---
    integral += error * dt;
    integral = constrain(integral, -INTEGRAL_MAX, INTEGRAL_MAX);
    float iTerm = Ki * integral;

    // --- D term ---
    float dTerm = Kd * (error - prevError) / dt;
    prevError = error;

    // --- Sum and clamp ---
    float output = pTerm + iTerm + dTerm;
    int pwm = constrain(abs((int)output), MIN_PWM, MAX_PWM);

    // Apply direction
    if (output > 0) {
      driveMotor(pwm);
    } else {
      driveMotor(-pwm);
    }

    // --- Print every 100ms ---
    static unsigned long lastPrint = 0;
    if (now - lastPrint > 100) {
      Serial.printf("pos:%ld err:%ld pwm:%d P:%.1f I:%.1f D:%.1f\n",
                     pos, error, (output > 0 ? pwm : -pwm),
                     pTerm, iTerm, dTerm);
      lastPrint = now;
    }
  }
}