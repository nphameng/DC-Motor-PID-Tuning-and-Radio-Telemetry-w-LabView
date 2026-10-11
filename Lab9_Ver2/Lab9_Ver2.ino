#include <SPI.h>
#include <RF24.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>

// ---------- PIN MAPPING ----------
#define PIN_RPM_INT     2   
#define PIN_MOTOR_PWM   3   
#define PIN_MOTOR_IN2   4   
#define PIN_DHT         5   
#define PIN_MOTOR_IN1   6   
#define PIN_BTN_STOP    7   
#define PIN_BTN_PID     8   
#define PIN_RF_CE       9   
#define PIN_RF_CSN      10  
#define PIN_LDR         A0  
#define PIN_BTN_START   A3  

// ---------- CONFIG ----------
#define TC74_ADDR       0x48
#define LCD_ADDR        0x27
#define DHT_TYPE        DHT11

// ---------- GLOBALS ----------
RF24 radio(PIN_RF_CE, PIN_RF_CSN);
LiquidCrystal_I2C lcd(LCD_ADDR, 16, 2);
DHT dht(PIN_DHT, DHT_TYPE);

const byte pipeCMD[6] = "L9CMD"; 
const byte pipeTLM[6] = "L9TLM"; 

// Master State Structure
struct InternalState {
  int setpoint;
  bool motorStop;
  bool reverse;
  bool pidEnabled;
} sys = {150, true, false, false}; //0 first

// PID & Math Variables
volatile unsigned long pulseCount = 0;
const unsigned long Ts_ms = 50; 
const float Ts = Ts_ms / 1000.0f;

// Note: Tune Kp and Ki as needed based on previous 12VDC testing
float Kp = 0.8f, Ki = 0.3f, Kd = 0.05f; 
float Iacc = 0, rpm_filtered = 0, rpm_prev_filtered = 0;
float alpha = 0.80f; 
int current_u = 0;

unsigned long lastControlMs = 0;
unsigned long lastDisplayMs = 0;

void countPulse() { pulseCount++; }

void setup() {
  Serial.begin(115200); 
  Wire.begin();
  
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print("System Boot...");
  
  dht.begin();

  pinMode(PIN_MOTOR_PWM, OUTPUT);
  pinMode(PIN_MOTOR_IN1, OUTPUT);
  pinMode(PIN_MOTOR_IN2, OUTPUT);
  
  // Physical Override Buttons
  pinMode(PIN_BTN_START, INPUT_PULLUP);
  pinMode(PIN_BTN_STOP, INPUT_PULLUP);
  pinMode(PIN_BTN_PID, INPUT_PULLUP);
  
  // High-Speed Interrupt
  pinMode(PIN_RPM_INT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_RPM_INT), countPulse, FALLING);

  radio.begin();
  radio.setDataRate(RF24_250KBPS);
  radio.openReadingPipe(1, pipeCMD);
  radio.openWritingPipe(pipeTLM);
  radio.startListening();
  
  delay(1000);
  lcd.clear();
}

void loop() {
  unsigned long now = millis();

  // 1. WIRELESS RECEIVE (String-based parsing for LabVIEW TX)
  if (radio.available()) {
    char abc[32] = {0};
    radio.read(&abc, sizeof(abc));
    char cmdChar = abc[0]; 
    
    if (cmdChar == 'S') {
      sys.motorStop = true;
    } else {
      sys.motorStop = false;
      sys.reverse = (cmdChar == 'R');
      int sp, p_en;
      if (sscanf(abc + 2, "%d,%d", &sp, &p_en) == 2) {
        sys.pidEnabled = (p_en == 1);
        // Map 0-100% slider from LabVIEW to RPM (PID) or PWM (Manual)
        sys.setpoint = sys.pidEnabled ? map(sp, 0, 100, 0, 25000) : map(sp, 0, 100, 0, 255);
      }
    }
  }

  // 2. PHYSICAL BUTTON OVERRIDES (Takes priority over radio)
  if (digitalRead(PIN_BTN_START) == LOW) sys.motorStop = false;
  if (digitalRead(PIN_BTN_STOP) == LOW)  sys.motorStop = true;
  
  static bool lastPidBtn = HIGH;
  bool pidBtn = digitalRead(PIN_BTN_PID);
  if (pidBtn == LOW && lastPidBtn == HIGH) {
    sys.pidEnabled = !sys.pidEnabled;
    delay(50); // Debounce
  }
  lastPidBtn = pidBtn;

 // 3. PID CONTROL LOOP (50ms) [cite: 909, 931]
  if (now - lastControlMs >= Ts_ms) {
    lastControlMs += Ts_ms;
    
    // Atomic pulse capture [cite: 948, 1179]
    noInterrupts();
    float currentPulses = pulseCount;
    pulseCount = 0;
    interrupts(); 
    
    // N=3 Custom Encoder Math [cite: 948, 1254]
    float rpm_measured = (currentPulses / Ts) * 60.0f / 3.0f; 
    rpm_filtered = (alpha * rpm_filtered) + ((1.0f - alpha) * rpm_measured); // EMA Filter [cite: 1195]

    float error_rpm = 0; // Declare true error here for the plotter

    if (sys.motorStop) {
      analogWrite(PIN_MOTOR_PWM, 0);
      Iacc = 0; 
      current_u = 0;
      error_rpm = 0; // Error is 0 when intentionally stopped
    } else {
      digitalWrite(PIN_MOTOR_IN1, !sys.reverse);
      digitalWrite(PIN_MOTOR_IN2, sys.reverse);
      
      if (sys.pidEnabled) {
        // Calculate the true RPM Error [cite: 962, 1340]
        error_rpm = (float)sys.setpoint - rpm_filtered; 
        
        float P = Kp * error_rpm;
        float D = -Kd * (rpm_filtered - rpm_prev_filtered) / Ts; // Derivative on Measurement [cite: 1192, 1630]
        float I_candidate = Iacc + (Ki * Ts * error_rpm);
        
        // Anti-Windup Logic [cite: 1188, 1629]
        if (P + I_candidate + D >= 0 && P + I_candidate + D <= 255) {
          Iacc = I_candidate;
        }
        current_u = constrain((int)(P + Iacc + D), 0, 255);
      } else {
        // In Manual Mode, setpoint is just 0-255 PWM juice [cite: 977, 1010]
        current_u = constrain(sys.setpoint, 0, 255);
        // Note: error_rpm isn't strictly applicable in manual mode, 
        // but you can leave it as 0 or calculate it if you want to see the difference.
      }
      analogWrite(PIN_MOTOR_PWM, current_u);
    }
    
    rpm_prev_filtered = rpm_filtered;

    // ---------- SERIAL PLOTTER OUTPUT ----------
    // Format: Setpoint, Actual_RPM, Error_RPM
    Serial.print("Setpoint:"); 
    Serial.print(sys.setpoint); 
    Serial.print(",");
    
    Serial.print("Current_RPM:");
    Serial.print(rpm_filtered);
    Serial.print(",");
    
    Serial.print("Error_RPM:");
    Serial.println(error_rpm); // New line triggers the graph update
  }
  
  // 4. TELEMETRY & DISPLAY (500ms)
  if (now - lastDisplayMs >= 500) {
    lastDisplayMs = now;
    
    // Sensor Acquisitions
    int8_t temp = -127;
    Wire.beginTransmission(TC74_ADDR);
    Wire.write(0x00);
    if (Wire.endTransmission() == 0) {
      Wire.requestFrom(TC74_ADDR, (byte)1);
      if (Wire.available()) temp = (int8_t)Wire.read();
    }
    int humid = (int)dht.readHumidity();
    int light = map(analogRead(PIN_LDR), 0, 1023, 0, 100);

    // OPTIMIZED LCD UPDATE (16x2 Character Fit)
    // Line 1: "R:25000 PID RUN "
    lcd.setCursor(0, 0);
    lcd.print("R:"); 
    lcd.print((int)rpm_filtered);
    lcd.print("    "); // Pad spaces to clear old trailing numbers
    
    lcd.setCursor(8, 0); // Lock to column 8 for mode status
    lcd.print(sys.pidEnabled ? "PID " : "MAN ");
    lcd.print(sys.motorStop ? "STP" : "RUN");

    // Line 2: "T:25C H:45% L:99"
    lcd.setCursor(0, 1);
    lcd.print("T:"); 
    if (temp == -127) lcd.print("--"); else lcd.print(temp); 
    lcd.print("C ");
    
    lcd.print("H:"); 
    if (isnan(humid)) lcd.print("--"); else lcd.print(humid); 
    lcd.print("% ");
    
    lcd.print("L:"); lcd.print(light);
    lcd.print("  "); // Wipe trailing chars

    // FORMAT TELEMETRY STRING FOR LABVIEW
    // Format: T,Temp,RPM,Humid,Light,PWM,PID,Status
    char tlmBuffer[64];
    sprintf(tlmBuffer, "T,%d,%.1f,%.1f,%d,%d,%d,%s", 
            temp, rpm_filtered, (float)humid, light, current_u, 
            sys.pidEnabled ? 1 : 0, sys.motorStop ? "STOP" : "RUN");
    
    // Wireless Transmit back to TX
    radio.stopListening();
    radio.write(&tlmBuffer, sizeof(tlmBuffer));
    radio.startListening();
    
    // Print to Serial for debugging
    Serial.println(tlmBuffer);
  }
}