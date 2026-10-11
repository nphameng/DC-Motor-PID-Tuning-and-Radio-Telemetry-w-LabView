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
#define N_SLOTS         4      

// ---------- STRUCTS ----------
struct ControlPacket {
  bool motorStop;      
  char command;        
  uint8_t setpoint;    
  bool pidEnable;      
  uint16_t seq;        
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

struct InternalState {
  int setpoint;
  bool motorStop;
  bool reverse;
  bool pidEnabled;
} sys = {10000, true, false, true}; 

unsigned long lastPacketMs = 0;
const unsigned long RX_TIMEOUT_MS = 2000; 
bool isConnected = false;

volatile unsigned long pulseCount = 0;
const unsigned long Ts_ms = 200; // Stabilized window //100
const float Ts = Ts_ms / 1000.0f;

// PID Tuning Parameters - Optimized for 4-blade resolution
float Kp = 0.0165f, Ki = 0.0001f, Kd = 0.0f; //Ki=0.0000122f Kp = 0.0131 or 0.0160
float Iacc = 0, rpm_filtered = 0, rpm_prev_filtered = 0;
float alpha = 0.92f; 
int current_u = 0;

unsigned long lastControlMs = 0;
unsigned long lastDisplayMs = 0;

// ---------- DEBOUNCING GLOBALS ----------
volatile unsigned long lastPulseTime = 0; 
const unsigned long debounceTime_us = 500; 

void countPulse() {
  unsigned long now_us = micros();
  if (now_us - lastPulseTime > debounceTime_us) {
    pulseCount++;
    lastPulseTime = now_us;
  }
}

void setup() {
  Serial.begin(115200); 
  Wire.begin();
  lcd.init();
  lcd.backlight();
  dht.begin();

  pinMode(PIN_MOTOR_PWM, OUTPUT);
  pinMode(PIN_MOTOR_IN1, OUTPUT);
  pinMode(PIN_MOTOR_IN2, OUTPUT);
  pinMode(PIN_BTN_START, INPUT_PULLUP);
  pinMode(PIN_BTN_STOP, INPUT_PULLUP);
  pinMode(PIN_BTN_PID, INPUT_PULLUP);
  pinMode(PIN_RPM_INT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_RPM_INT), countPulse, FALLING);

  radio.begin();
  radio.setDataRate(RF24_1MBPS);
  radio.setChannel(108); 
  radio.setPALevel(RF24_PA_LOW);
  radio.openReadingPipe(1, pipeCMD);
  radio.openWritingPipe(pipeTLM);
  radio.startListening();
  
  lcd.print("PID READY");
  delay(1000);
  lcd.clear();
}

void loop() {
  unsigned long now = millis();

  // 1. WIRELESS RECEIVE (Bypassed logic for Offline Tuning)
  if (radio.available()) {
    radio.read(&incoming, sizeof(incoming)); 
    lastPacketMs = now;
    isConnected = true;
    // Note: Radio setpoint logic is commented out to prioritize buttons
  }

  // 3. PHYSICAL BUTTON OVERRIDES
  if (digitalRead(PIN_BTN_START) == LOW) sys.motorStop = false;
  if (digitalRead(PIN_BTN_STOP) == LOW)  sys.motorStop = true;
  
  static bool lastPidBtn = HIGH;
  bool pidBtn = digitalRead(PIN_BTN_PID);
  if (pidBtn == LOW && lastPidBtn == HIGH) {
    sys.pidEnabled = !sys.pidEnabled;
    if (!sys.pidEnabled) sys.setpoint = 150; 
    else sys.setpoint = 10000;              
    delay(50); 
  }
  lastPidBtn = pidBtn;

  // 4. PID CONTROL LOOP (100ms)
  if (now - lastControlMs >= Ts_ms) {
    lastControlMs += Ts_ms;
    
    noInterrupts();
    float currentPulses = pulseCount;
    pulseCount = 0;
    interrupts(); 
    
    float rpm_measured = (currentPulses / Ts) * 60.0f / (float)N_SLOTS; 
    rpm_filtered = (alpha * rpm_filtered) + ((1.0f - alpha) * rpm_measured); 

    float error_rpm = 0; 

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
        
        // Anti-Saturation & Anti-Windup Logic
        float total_u = P + I_candidate + D;
        if (total_u >= 0 && total_u <= 255) {
            Iacc = I_candidate;
        }
        current_u = constrain((int)total_u, 0, 255);

      } else {
        current_u = constrain(sys.setpoint, 0, 255);
        error_rpm = 0; 
      }
      analogWrite(PIN_MOTOR_PWM, current_u);
    }
    rpm_prev_filtered = rpm_filtered;

    // SERIAL PLOTTER OUTPUT
    Serial.print("Setpoint:"); Serial.print(sys.setpoint); Serial.print(",");
    Serial.print("Current_RPM:"); Serial.print(rpm_filtered); Serial.print(",");
    Serial.print("Error_RPM:"); Serial.println(error_rpm); 
  }

  // 5. TELEMETRY & DISPLAY (500ms)
  if (now - lastDisplayMs >= 500) {
    lastDisplayMs = now;
    
    int8_t temp = -127;
    Wire.beginTransmission(TC74_ADDR);
    Wire.write(0x00);
    if (Wire.endTransmission() == 0) {
      Wire.requestFrom(TC74_ADDR, (byte)1);
      if (Wire.available()) temp = (int8_t)Wire.read();
    }

    lcd.setCursor(0, 0);
    lcd.print("R:"); lcd.print((int)rpm_filtered); lcd.print("    "); 
    lcd.setCursor(8, 0); 
    lcd.print(sys.pidEnabled ? "PID " : "MAN ");
    lcd.print(sys.motorStop ? "STP" : "RUN");

    lcd.setCursor(0, 1);
    lcd.print("T:"); if (temp == -127) lcd.print("--"); else lcd.print(temp); 
    lcd.print("C  SP:"); lcd.print(sys.setpoint); lcd.print("   ");
  }
}