#include <SPI.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include <nRF24L01.h>
#include <RF24.h>

// ----- PIN MAP ---
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

// ---- CONFIUGURATIONS ---
#define TC74_ADDR       0x48
#define LCD_ADDR        0x27
#define DHT_TYPE        DHT11
#define N_SLOTS         4 // 4 blade fan

// ----- STRUCTURES (Receive from TX) ------
struct ControlPacket 
{
  bool motorStop;      // stop command
  char command;        // char 'F', 'R', 'S'
  uint8_t setpoint;    // 0-100%
  bool pidEnable;      // PID enable or not
  uint16_t seq;        // packet has a sequence number
};

//send back to TX (telemetry)
struct TelemetryPacket 
{
  uint16_t rpm;
  float tempC;
  float humidity;
  uint16_t light;
  bool pidMode;
  uint8_t motorPWM;
  char status[12];
};

// ---GLOBALS VARUIABLES ---
RF24 radio(PIN_RF_CE, PIN_RF_CSN);
LiquidCrystal_I2C lcd(LCD_ADDR, 16, 2);
DHT dht(PIN_DHT, DHT_TYPE);
// address of receving and sending back
const byte pipeCMD[6] = "L9CMD"; 
const byte pipeTLM[6] = "L9TLM"; 

ControlPacket incoming;
TelemetryPacket tlmOut;

// Master Structure that controls states
struct InternalState 
{
  int setpoint;
  bool motorStop;
  bool reverse;
  bool pidEnabled;
} 
sys = {0, true, false, false}; 

// Variables to control connection lost / timeout
unsigned long lastPacketMs = 0;
const unsigned long RX_TIMEOUT_MS = 2000; // 2 seconds without data from TX ---> disconnected
bool isConnected = false;

// sample window time
volatile unsigned long pulseCount = 0;
const unsigned long Ts_ms = 200; 
const float Ts = Ts_ms / 1000.0f;

// Tuning Params
float Kp = 0.0165f, Ki = 0.0000f, Kd = 0.0f; 
float Iacc = 0, rpm_filtered = 0, rpm_prev_filtered = 0;
float alpha = 0.92f; 
int current_u = 0;

unsigned long lastControlMs = 0;
unsigned long lastDisplayMs = 0;

// variables to debounce (each fan blade can read 1-2 pules if no debounce)
volatile unsigned long lastPulseTime = 0; 
const unsigned long debounceTime_us = 500; // 0.5ms lockout

void countPulse() 
{ 
  unsigned long now_us = micros();
  if (now_us - lastPulseTime > debounceTime_us) 
  {
    pulseCount++;
    lastPulseTime = now_us;
  }
}
//-------------------------------------------------------------------------------------------------------------
void setup() 
{
  Serial.begin(115200); 
  Wire.begin();
  
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print("Starting RX Module");
  
  dht.begin();

  pinMode(PIN_MOTOR_PWM, OUTPUT);
  pinMode(PIN_MOTOR_IN1, OUTPUT);
  pinMode(PIN_MOTOR_IN2, OUTPUT);
  
  // Physical Buttons to override LabView commands
  pinMode(PIN_BTN_START, INPUT_PULLUP);
  pinMode(PIN_BTN_STOP, INPUT_PULLUP);
  pinMode(PIN_BTN_PID, INPUT_PULLUP);
  
  // HInterrupt. I use interrupt to measure RPM from speed sensor
  pinMode(PIN_RPM_INT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_RPM_INT), countPulse, FALLING);

  // Radio Config
  if (!radio.begin()) 
  {
    Serial.println("Hardware not Responding");
  }
  // notice, all of these must match TX
  radio.setDataRate(RF24_1MBPS);
  radio.setChannel(108); // CRITICAL: Matches LabVIEW TX
  radio.setPALevel(RF24_PA_LOW);
  radio.openReadingPipe(1, pipeCMD);
  radio.openWritingPipe(pipeTLM);
  radio.startListening();
  
  delay(800);
  lcd.clear();
}

void loop() 
{
  unsigned long now = millis();

  // 1. Recevie data from TX
  if (radio.available()) 
  {
    radio.read(&incoming, sizeof(incoming)); 
    
    lastPacketMs = now;
    isConnected = true;

    // We map the incoming data packet structure from TX to the RX states
    sys.motorStop = incoming.motorStop;
    sys.reverse = (incoming.command == 'R');
    sys.pidEnabled = incoming.pidEnable;
    
    // Map 0-100% setpoint from LabVIEW to units 
    if (sys.pidEnabled) 
    {
      sys.setpoint = map(incoming.setpoint, 0, 100, 0, 25000); // RPM SP
    } else {
      sys.setpoint = map(incoming.setpoint, 0, 100, 0, 255);   // PWM SP
    }
  }

  // 2. Logic for connection timeout
  if (now - lastPacketMs > RX_TIMEOUT_MS) 
  {
    isConnected = false;
  }

  // 3. waiting state, execute if dont see TX signal. Let us know if we are connected or not
  if (!isConnected) 
  {
    // Safety: cut motor power if cannot seeTX
    analogWrite(PIN_MOTOR_PWM, 0);
    Iacc = 0; 
    current_u = 0;
    
    if (now - lastDisplayMs >= 800) 
    { 
      lastDisplayMs = now;
      lcd.setCursor(0, 0);
      lcd.print("Waiting to Conn");
    }
    return; // Exit loop until connected to TX
  }

  // 4.local buttons on RX to override Labview
  if (digitalRead(PIN_BTN_START) == LOW) sys.motorStop = false;
  if (digitalRead(PIN_BTN_STOP) == LOW)  sys.motorStop = true;
  
  static bool lastPidBtn = HIGH;
  bool pidBtn = digitalRead(PIN_BTN_PID);
  if (pidBtn == LOW && lastPidBtn == HIGH)
  {
    sys.pidEnabled = !sys.pidEnabled;
    delay(50); // Debounce
  }
  lastPidBtn = pidBtn;

  // 5. PID CONTROL LOOP 
  if (now - lastControlMs >= Ts_ms) {
    lastControlMs += Ts_ms;
    
    // pulse capture
    noInterrupts();
    float currentPulses = pulseCount;
    pulseCount = 0;
    interrupts(); 
    
    // calculate RPM. N_SLOTS = 4
    float rpm_measured = (currentPulses / Ts) * 60.0f / (float)N_SLOTS; 
    rpm_filtered = (alpha * rpm_filtered) + ((1.0f - alpha) * rpm_measured); 

    float error_rpm = 0; 

    if (sys.motorStop) 
    {
      analogWrite(PIN_MOTOR_PWM, 0);
      Iacc = 0; 
      current_u = 0;
    } 
    else 
    {
      digitalWrite(PIN_MOTOR_IN1, !sys.reverse);
      digitalWrite(PIN_MOTOR_IN2, sys.reverse);
      
      if (sys.pidEnabled) {
        error_rpm = (float)sys.setpoint - rpm_filtered; 
        float P = Kp * error_rpm;
        float D = -Kd * (rpm_filtered - rpm_prev_filtered) / Ts; 
        float I_candidate = Iacc + (Ki * Ts * error_rpm);
        
        float total_u = P + I_candidate + D;
        
        // -logic for Anti Windup (I)
        // Only freeze the integral if we hit the ceiling AND error is positive, OR if we hit the floor AND error is negative.
        bool saturatedHigh = (total_u >= 255) && (error_rpm > 0);
        bool saturatedLow = (total_u <= 0) && (error_rpm < 0);
        
        if (!saturatedHigh && !saturatedLow) 
        {
            Iacc = I_candidate;
        }

        // Calculate where PID needs to go (Target error u)
        int target_u = constrain((int)total_u, 0, 255);
        
        // lOGIC for Anti Brake Slamming 
        //(our motor was slamming on the brake and stop if error between curent and sp RPM was too big)
        // Limit the max PWM change per 100ms cycle. Lower = smoother but slower reaction.
        int max_pwm_change = 15; 
        
        if (target_u > current_u + max_pwm_change) 
        {
            current_u += max_pwm_change; // Smooth acceleration
        } 
        else if (target_u < current_u - max_pwm_change) 
        {
            current_u -= max_pwm_change; // Smooth deceleration (Motor doesnt slam stop)
        } 
        else 
        {
            current_u = target_u;
        }
        
      } 
      else 
      {
        current_u = constrain(sys.setpoint, 0, 255);
      }
      analogWrite(PIN_MOTOR_PWM, current_u);
      
    }
    
    rpm_prev_filtered = rpm_filtered;

    // Serial Plotter
    Serial.print("Setpoint:"); Serial.print(sys.setpoint); Serial.print(",");
    Serial.print("Current_RPM:"); Serial.print(rpm_filtered); Serial.print(",");
    Serial.print("Error_RPM:"); Serial.println(error_rpm); 
  }

  // 6. Telemetry, display on LCD
  if (now - lastDisplayMs >= 500) 
  {
    lastDisplayMs = now;
    
    // getting dat from sensors
    int8_t temp = -127;
    Wire.beginTransmission(TC74_ADDR);
    Wire.write(0x00);
    if (Wire.endTransmission() == 0) 
    {
      Wire.requestFrom(TC74_ADDR, (byte)1);
      if (Wire.available()) temp = (int8_t)Wire.read();
    }
    int humid = (int)dht.readHumidity();
    int light = map(analogRead(PIN_LDR), 0, 1023, 0, 100);

    // LCD 
    lcd.setCursor(0, 0);
    lcd.print("R:"); lcd.print((int)rpm_filtered); lcd.print("    "); 
    
    lcd.setCursor(8, 0); 
    lcd.print(sys.pidEnabled ? "PID " : "MAN ");
    lcd.print(sys.motorStop ? "STP" : "RUN");

    lcd.setCursor(0, 1);
    lcd.print("T:"); 
    if (temp == -127) lcd.print("--"); 
    else lcd.print(temp); 
    lcd.print("C ");
    
    lcd.print("H:"); 
    if (isnan(humid)) lcd.print("--"); 
    else lcd.print(humid); 
    lcd.print("% ");
    
    lcd.print("L:"); lcd.print(light); lcd.print("  "); 

    // Fill in values of Telemetry structure to send back to Labview (TX)
    tlmOut.rpm = (uint16_t)rpm_filtered;
    tlmOut.tempC = (float)temp;
    tlmOut.humidity = isnan(humid) ? 0.0f : (float)humid;
    tlmOut.light = (uint16_t)light;
    tlmOut.pidMode = sys.pidEnabled;
    tlmOut.motorPWM = (uint8_t)current_u;
    
    if (sys.motorStop) 
    {
      strcpy(tlmOut.status, "STOP");
    } 
    else 
    {
      strcpy(tlmOut.status, "RUN");
    }

    // Wireless Transmit back to TX
    radio.stopListening();
    radio.write(&tlmOut, sizeof(tlmOut));
    radio.startListening();
  }
}