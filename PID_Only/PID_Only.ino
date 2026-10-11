#include <Arduino.h>

// --- PIN MAPPING ---
const int PIN_RPM_INT = 2;   // LM393 DO on D2 (Interrupt)
const int PIN_MOTOR_PWM = 3; // L293D Enable
const int PIN_MOTOR_IN1 = 6; // L293D IN1 (Swapped to D6)
const int PIN_MOTOR_IN2 = 4; // L293D IN2

// --- PID CONSTANTS (From Lab 6) ---
const unsigned long Ts_ms = 50; // 50ms sample time
const float Ts = Ts_ms / 1000.0f;
float Kp = 0.f;  // Start lower for motor vs LED
float Ki = 0.0f;
float Kd = 0.00f;
float alpha = 0.80f; // EMA filter for RPM noise

// --- CONTROL VARIABLES ---
volatile unsigned long pulseCount = 0;
float rpm_measured = 0;
float rpm_filtered = 0;
float rpm_prev_filtered = 0;
int setPt_RPM = 19000; // Example test setpoint (RPM) [cite: 5134]

// --- PID STATE ---
float Iacc = 0.0f;
float lastControlMs = 0;

// Interrupt Service Routine (ISR)
void countPulse() {
  pulseCount++; // [cite: 5257]
}

void setup() {
  Serial.begin(115200);
  
  pinMode(PIN_MOTOR_PWM, OUTPUT);
  pinMode(PIN_MOTOR_IN1, OUTPUT);
  pinMode(PIN_MOTOR_IN2, OUTPUT);
  
  // Set initial direction: Forward
  digitalWrite(PIN_MOTOR_IN1, HIGH);
  digitalWrite(PIN_MOTOR_IN2, LOW);

  pinMode(PIN_RPM_INT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_RPM_INT), countPulse, FALLING); 

  Serial.println("Motor PID Test Starting...");
}

void loop() {
  unsigned long now = millis();

  // Fixed-rate control loop
  if (now - lastControlMs >= Ts_ms) {
    lastControlMs += Ts_ms;

    // 1) Calculate RPM
    // N = number of slots in your encoder disk
    int N = 3; 
    noInterrupts();    
    float currentPulses = pulseCount;
    pulseCount = 0;
    interrupts();    
    // Formula: (pulses / time_sec) * 60 / N_slots [cite: 5280]
    rpm_measured = (currentPulses / Ts) * 60.0f / N;

    // 2) Filter RPM (EMA) [cite: 4943]
    rpm_filtered = (alpha * rpm_filtered) + ((1.0f - alpha) * rpm_measured);

    // 3) Error
    float error = (float)setPt_RPM - rpm_filtered;

    // 4) PID terms 
    float P = Kp * error;
    float D = -Kd * (rpm_filtered - rpm_prev_filtered) / Ts; // Derivative on measurement [cite: 4949]

    // Integral with Anti-Windup (Conditional Integration) [cite: 4945]
    float I_candidate = Iacc + (Ki * Ts * error);
    float u_unsat = P + I_candidate + D;

    // Only update I if we aren't saturated or if error is moving us out of saturation
    if (u_unsat >= 0 && u_unsat <= 255) {
        Iacc = I_candidate;
    }
    Iacc = constrain(Iacc, -255, 255);

    int u = constrain((int)u_unsat, 0, 255);

    // 5) Output to Motor
    analogWrite(PIN_MOTOR_PWM, u);

    // Save state for next loop
    rpm_prev_filtered = rpm_filtered;

    // Serial Plotter output
    Serial.print("RPM_SP:");
    Serial.print(setPt_RPM); Serial.print(",");
    Serial.print("RPM_PV:");
    Serial.print(rpm_filtered); Serial.print(",");
    Serial.print("RPM_Error:");
    Serial.println(u);
  }
}