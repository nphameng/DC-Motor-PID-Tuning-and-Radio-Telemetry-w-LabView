#include <SPI.h>
#include <nRF24L01.h>
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

// ---------- STRUCTS (MATCHES PARTNER TX EXACTLY) ----------
struct ControlPacket {
  bool motorStop;      
  char command;        // 'F', 'R', or 'S'
  uint8_t setpoint;    // 0 to 100 percent
  bool pidEnable;      
  uint16_t seq;        // packet sequence number
};

struct TelemetryPacket {
  uint16_t rpm;
  float tempC;
  float humidity;
  uint16_t light;
  bool pidMode;
  uint8_t motorPWM;
  char status[12];
};

// ---------- GLOBALS ----------
RF24 radio(PIN_RF_CE, PIN_RF_CSN);
LiquidCrystal_I2C lcd(LCD_ADDR, 16, 2);
DHT dht(PIN_DHT, DHT_TYPE);

const byte pipeCMD[6] = "L9CMD"; 
const byte pipeTLM[6] = "L9TLM"; 

ControlPacket incoming;
TelemetryPacket tlmOut;

// Master State Structure
struct InternalState {
  int setpoint;
  bool motorStop;
  bool reverse;
  bool pidEnabled;
} sys = {0, true, false, false}; 

// Connection Timeout Variables
unsigned long lastPacketMs = 0;
const unsigned long RX_TIMEOUT_MS = 2000; // 2 seconds without data = disconnected
bool isConnected = false;

// PID & Math Variables
volatile unsigned long pulseCount = 0;
const unsigned long Ts_ms = 50; 
const float Ts = Ts_ms / 1000.0f;

// PID Tuning Parameters
float Kp = 0.1f, Ki = 0.1f, Kd = 0.00f; 
float Iacc = 0, rpm_filtered = 0, rpm_prev_filtered = 0;
float alpha = 0.80f; 
int current_u = 0;

unsigned long lastControlMs = 0;
unsigned long lastDisplayMs = 0;

void countPulse() { 
  pulseCount++; 
}

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

  // Radio Setup
  if (!radio.begin()) {
    Serial.println("Radio Hardware Not Responding!");
  }
  radio.setDataRate(RF24_1MBPS);
  radio.setChannel(108); // CRITICAL: Matches Partner TX
  radio.setPALevel(RF24_PA_LOW);
  radio.openReadingPipe(1, pipeCMD);
  radio.openWritingPipe(pipeTLM);
  radio.startListening();
  
  delay(1000);
  lcd.clear();
}

void loop() {
  unsigned long now = millis();

  // ==========================================
  // 1. WIRELESS RECEIVE (Binary Struct Parsing)
  // ==========================================
  if (radio.available()) {
    radio.read(&incoming, sizeof(incoming)); 
    
    lastPacketMs = now;
    isConnected = true;

    // Map incoming struct to local system state
    sys.motorStop = incoming.motorStop;
    sys.reverse = (incoming.command == 'R');
    sys.pidEnabled = incoming.pidEnable;
    
    // Map 0-100% slider from LabVIEW to units based on mode
    if (sys.pidEnabled) {
      sys.setpoint = map(incoming.setpoint, 0, 100, 0, 25000); // Target RPM
    } else {
      sys.setpoint = map(incoming.setpoint, 0, 100, 0, 255);   // Target PWM
    }
  }

  // ==========================================
  // 2. CONNECTION TIMEOUT LOGIC
  // ==========================================
  if (now - lastPacketMs > RX_TIMEOUT_MS) {
    isConnected = false;
  }

  // ==========================================
  // 3. WAITING STATE (Blocks regular execution)
  // ==========================================
  if (!isConnected) {
    // Failsafe: Cut motor power if radio dies
    analogWrite(PIN_MOTOR_PWM, 0);
    Iacc = 0; 
    current_u = 0;
    
    if (now - lastDisplayMs >= 5000) {
      lastDisplayMs = now;
      lcd.setCursor(0, 0);
      lcd.print("Status: WAITING ");
      lcd.setCursor(0, 1);
      lcd.print("Connect to TX...");
    }
    return; // Exit loop until connected
  }

  // ==========================================
  // 4. PHYSICAL BUTTON OVERRIDES
  // ==========================================
  if (digitalRead(PIN_BTN_START) == LOW) sys.motorStop = false;
  if (digitalRead(PIN_BTN_STOP) == LOW)  sys.motorStop = true;
  
  static bool lastPidBtn = HIGH;
  bool pidBtn = digitalRead(PIN_BTN_PID);
  if (pidBtn == LOW && lastPidBtn == HIGH) {
    sys.pidEnabled = !sys.pidEnabled;
    delay(50); // Debounce
  }
  lastPidBtn = pidBtn;

  // ==========================================
  // 5. PID CONTROL LOOP (50ms)
  // ==========================================
  if (now - lastControlMs >= Ts_ms) {
    lastControlMs += Ts_ms;
    
    // Atomic pulse capture
    noInterrupts();
    float currentPulses = pulseCount;
    pulseCount = 0;
    interrupts(); 
    
    // N=3 Custom Encoder Math
    float rpm_measured = (currentPulses / Ts) * 60.0f / 3.0f; 
    rpm_filtered = (alpha * rpm_filtered) + ((1.0f - alpha) * rpm_measured); 

    float error_rpm = 0; // Moved here so the plotter can always see it

    if (sys.motorStop) {
      analogWrite(PIN_MOTOR_PWM, 0);
      Iacc = 0; 
      current_u = 0;
    } else {
      digitalWrite(PIN_MOTOR_IN1, !sys.reverse);
      digitalWrite(PIN_MOTOR_IN2, sys.reverse);
      
      if (sys.pidEnabled) {
        error_rpm = (float)sys.setpoint - rpm_filtered; 
        float P = Kp * error_rpm;
        float D = -Kd * (rpm_filtered - rpm_prev_filtered) / Ts; 
        float I_candidate = Iacc + (Ki * Ts * error_rpm);
        
        // Anti-Windup Logic
        if (P + I_candidate + D >= 0 && P + I_candidate + D <= 255) {
          Iacc = I_candidate;
        }
        current_u = constrain((int)(P + Iacc + D), 0, 255);
      } else {
        current_u = constrain(sys.setpoint, 0, 255);
      }
      analogWrite(PIN_MOTOR_PWM, current_u);
    }
    
    rpm_prev_filtered = rpm_filtered;

    // ---------- SERIAL PLOTTER OUTPUT ----------
    Serial.print("Setpoint:"); 
    Serial.print(sys.setpoint); 
    Serial.print(",");
    Serial.print("Current_RPM:");
    Serial.print(rpm_filtered);
    Serial.print(",");
    Serial.print("Error_RPM:");
    Serial.println(error_rpm); 
  }

  // ==========================================
  // 6. TELEMETRY & DISPLAY (500ms)
  // ==========================================
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

    // OPTIMIZED LCD UPDATE 
    lcd.setCursor(0, 0);
    lcd.print("R:"); 
    lcd.print((int)rpm_filtered);
    lcd.print("    "); 
    
    lcd.setCursor(8, 0); 
    lcd.print(sys.pidEnabled ? "PID " : "MAN ");
    lcd.print(sys.motorStop ? "STP" : "RUN");

    lcd.setCursor(0, 1);
    lcd.print("T:"); 
    if (temp == -127) lcd.print("--"); else lcd.print(temp); 
    lcd.print("C ");
    
    lcd.print("H:"); 
    if (isnan(humid)) lcd.print("--"); else lcd.print(humid); 
    lcd.print("% ");
    
    lcd.print("L:"); lcd.print(light);
    lcd.print("  "); 

    // POPULATE TELEMETRY STRUCT 
    tlmOut.rpm = (uint16_t)rpm_filtered;
    tlmOut.tempC = (float)temp;
    tlmOut.humidity = isnan(humid) ? 0.0f : (float)humid;
    tlmOut.light = (uint16_t)light;
    tlmOut.pidMode = sys.pidEnabled;
    tlmOut.motorPWM = (uint8_t)current_u;
    
    if (sys.motorStop) {
      strcpy(tlmOut.status, "STOP");
    } else {
      strcpy(tlmOut.status, "RUN");
    }

    // Wireless Transmit back to TX
    radio.stopListening();
    radio.write(&tlmOut, sizeof(tlmOut));
    radio.startListening();
  }
}