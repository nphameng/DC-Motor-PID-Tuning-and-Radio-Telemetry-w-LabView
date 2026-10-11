#include <SPI.h>
#include <RF24.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>

// ---------- PIN MAPPING (Verified) ----------
#define PIN_RPM_INT    2   
#define PIN_MOTOR_PWM  3   
#define PIN_MOTOR_IN2  4   
#define PIN_DHT        5   
#define PIN_MOTOR_IN1  6   
#define PIN_BTN_STOP   7   // D7: Local Stop [cite: 668]
#define PIN_BTN_PID    8   // D8: Local PID Toggle [cite: 669]
#define PIN_RF_CE      9   
#define PIN_RF_CSN     10  
#define PIN_LDR        A0  
#define PIN_BTN_START  A3  // A3: Local Start [cite: 672]

// ---------- CONFIG & ADDRESSES ----------
#define TC74_ADDR      0x48
#define LCD_ADDR       0x27
#define DHT_TYPE       DHT11
const byte pipeCMD[6] = "L9CMD"; 
const byte pipeTLM[6] = "L9TLM"; 

// ---------- STRUCTURES ----------
struct CommandPacket {
  int setpoint;    
  bool motorStop;  
  bool reverse;    
  bool pidEnabled; 
};

struct TelemetryPacket {
  float currentRPM;
  int temperature;
  int humidity;
  bool isPidOn;
};

// ---------- OBJECTS & GLOBALS ----------
RF24 radio(PIN_RF_CE, PIN_RF_CSN);
LiquidCrystal_I2C lcd(LCD_ADDR, 16, 2);
DHT dht(PIN_DHT, DHT_TYPE);

CommandPacket incoming = {0, true, false, false};
TelemetryPacket tlm = {0.0, 0, 0, false};

volatile unsigned long pulseCount = 0;
const unsigned long Ts_ms = 50; 
const float Ts = Ts_ms / 1000.0f;
float Kp = 0.8f, Ki = 0.3f, Kd = 0.05f; 
float Iacc = 0, rpm_filtered = 0, rpm_prev_filtered = 0;
float alpha = 0.80f; 

unsigned long lastControlMs = 0;
unsigned long lastDisplayMs = 0;

void countPulse() { pulseCount++; } // [cite: 474]

void setup() {
  Serial.begin(115200); 
  Wire.begin();
  lcd.init();
  lcd.backlight();
  dht.begin();

  pinMode(PIN_MOTOR_PWM, OUTPUT);
  pinMode(PIN_MOTOR_IN1, OUTPUT);
  pinMode(PIN_MOTOR_IN2, OUTPUT);
  
  // Initialize Active-Low Buttons [cite: 50, 130]
  pinMode(PIN_BTN_START, INPUT_PULLUP);
  pinMode(PIN_BTN_STOP, INPUT_PULLUP);
  pinMode(PIN_BTN_PID, INPUT_PULLUP);

  pinMode(PIN_RPM_INT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_RPM_INT), countPulse, FALLING);

  radio.begin();
  radio.setDataRate(RF24_250KBPS);
  radio.openReadingPipe(1, pipeCMD);
  radio.openWritingPipe(pipeTLM);
  radio.startListening();
}

void loop() {
  unsigned long now = millis();

  // 1. WIRELESS RECEIVE
  if (radio.available()) {
    radio.read(&incoming, sizeof(incoming));
  }

  // 2. PHYSICAL BUTTON OVERRIDES (Local overrides LabVIEW) [cite: 171]
  if (digitalRead(PIN_BTN_START) == LOW) incoming.motorStop = false;
  if (digitalRead(PIN_BTN_STOP) == LOW)  incoming.motorStop = true;
  
  // Toggle PID with simple debounce
  static bool lastPidBtn = HIGH;
  bool pidBtn = digitalRead(PIN_BTN_PID);
  if (pidBtn == LOW && lastPidBtn == HIGH) {
    incoming.pidEnabled = !incoming.pidEnabled;
    delay(50); // Small debounce
  }
  lastPidBtn = pidBtn;

  // 3. PID CONTROL LOOP (50ms) [cite: 291, 582]
  if (now - lastControlMs >= Ts_ms) {
    lastControlMs += Ts_ms;

    noInterrupts(); // Atomic capture for high-speed pulses [cite: 134, 596]
    float currentPulses = pulseCount;
    pulseCount = 0;
    interrupts(); 
    
    // N=3 for your custom disk [cite: 135, 233]
    float rpm_measured = (currentPulses / Ts) * 60.0f / 3.0f; 
    rpm_filtered = (alpha * rpm_filtered) + ((1.0f - alpha) * rpm_measured); // EMA Filter [cite: 135, 173]

    int u = 0; // Final PWM value

    if (incoming.motorStop) {
      analogWrite(PIN_MOTOR_PWM, 0);
      Iacc = 0; // Reset integral to prevent jump on restart [cite: 73, 156]
    } else {
      digitalWrite(PIN_MOTOR_IN1, !incoming.reverse);
      digitalWrite(PIN_MOTOR_IN2, incoming.reverse);

      if (incoming.pidEnabled) {
        float error = (float)incoming.setpoint - rpm_filtered;
        float P = Kp * error;
        float D = -Kd * (rpm_filtered - rpm_prev_filtered) / Ts; // Derivative on Measurement [cite: 170, 203]
        float I_candidate = Iacc + (Ki * Ts * error);
        
        // Anti-Windup: Only update I if output is within 0-255 [cite: 44, 140, 587]
        if (P + I_candidate + D >= 0 && P + I_candidate + D <= 255) {
          Iacc = I_candidate;
        }
        u = constrain((int)(P + Iacc + D), 0, 255);
        analogWrite(PIN_MOTOR_PWM, u);
      } else {
        // Manual Mode: Setpoint 0-255 maps directly to PWM [cite: 71, 151]
        u = constrain(incoming.setpoint, 0, 255);
        analogWrite(PIN_MOTOR_PWM, u);
      }
    }
    
    // ---------- SERIAL PLOTTER OUTPUT ----------
    // Format: Setpoint, Actual_RPM, PWM_Output(0-255)
    Serial.print("RMP_SP:");
    Serial.print(incoming.setpoint); 
    Serial.print(",");
    Serial.print("RMP_PV:");
    Serial.print(rpm_filtered);
    Serial.print(",");
    Serial.print("RMP_Error:");
    Serial.println(u); // New line triggers the plotter [cite: 48, 572]

    rpm_prev_filtered = rpm_filtered;
  }

  // 4. TELEMETRY & DISPLAY (500ms) [cite: 211, 230]
  if (now - lastDisplayMs >= 500) {
    lastDisplayMs = now;

    tlm.currentRPM = rpm_filtered;
    tlm.humidity = (int)dht.readHumidity();
    tlm.isPidOn = incoming.pidEnabled;
    
    Wire.beginTransmission(TC74_ADDR);
    Wire.write(0x00);
    if (Wire.endTransmission() == 0) {
      Wire.requestFrom(TC74_ADDR, (byte)1);
      if (Wire.available()) tlm.temperature = (int8_t)Wire.read();
    }

    lcd.setCursor(0, 0);
    lcd.print("RPM:"); lcd.print((int)tlm.currentRPM);
    lcd.print(" "); lcd.print(tlm.isPidOn ? "PID" : "MANL"); // Added PID status [cite: 508]
    lcd.print(incoming.motorStop ? " STOP" : " RUN ");

    lcd.setCursor(0, 1);
    lcd.print("T:"); lcd.print(tlm.temperature);
    lcd.print("C H:"); lcd.print(tlm.humidity);
    lcd.print("% L"); lcd.print(map(analogRead(PIN_LDR), 0, 1023, 0, 100));

    // Send Telemetry back to TX [cite: 341]
    radio.stopListening();
    radio.write(&tlm, sizeof(tlm));
    radio.startListening();
  }
}