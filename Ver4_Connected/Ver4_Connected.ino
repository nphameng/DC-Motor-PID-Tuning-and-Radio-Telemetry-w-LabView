#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include <math.h>

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

// ---------- RF SETUP ----------
RF24 radio(PIN_RF_CE, PIN_RF_CSN);

const byte pipeToReceiver[6] = "L9CMD";
const byte pipeToTransmitter[6] = "L9TLM";

// ---------- LCD + DHT ----------
LiquidCrystal_I2C lcd(LCD_ADDR, 16, 2);
DHT dht(PIN_DHT, DHT_TYPE);

// ---------- STRUCTS ----------
struct ControlPacket {
  bool motorStop;
  char command;        // 'F', 'R', or 'S'
  uint8_t setpoint;    // 0 to 100 percent
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

ControlPacket incoming;
TelemetryPacket tlmOut;

// ---------- SYSTEM STATE ----------
struct InternalState {
  int setpoint;
  bool motorStop;
  bool reverse;
  bool pidEnabled;
} sys = {0, true, false, false};

// ---------- CONNECTION ----------
unsigned long lastPacketMs = 0;
const unsigned long RX_TIMEOUT_MS = 2000;
bool isConnected = false;

uint16_t lastSeq = 0;
char lastCommand = 'S';
uint8_t lastSetpointPercent = 0;

// ---------- RPM / PID ----------
volatile unsigned long pulseCount = 0;

const unsigned long Ts_ms = 50;
const float Ts = Ts_ms / 1000.0f;

float Kp = 0.4f;
float Ki = 0.2f;
float Kd = 0.05f;

float Iacc = 0;
float rpm_filtered = 0;
float rpm_prev_filtered = 0;
float alpha = 0.80f;

int current_u = 0;

unsigned long lastControlMs = 0;
unsigned long lastDisplayMs = 0;
unsigned long lastSensorMs = 0;

// ---------- SENSOR VALUES ----------
float lastTempC = -127.0f;
float lastHumidity = 0.0f;
uint16_t lastLight = 0;

bool lastTlmOk = false;

// ---------- FUNCTION PROTOTYPES ----------
void countPulse();
void readWireless(unsigned long now);
void handleConnectionTimeout(unsigned long now);
void showWaitingScreen(unsigned long now);
void handleButtons();
void runControlLoop(unsigned long now);
void updateSensors();
void updateLCD();
void buildTelemetryPacket();
void sendTelemetryToTransmitter();
void stopMotorOutput();
void clearLCDRow(uint8_t row);

void setup() {
  Serial.begin(115200);
  Wire.begin();

  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Receiver Boot");
  lcd.setCursor(0, 1);
  lcd.print("Starting NRF...");

  dht.begin();

  pinMode(PIN_MOTOR_PWM, OUTPUT);
  pinMode(PIN_MOTOR_IN1, OUTPUT);
  pinMode(PIN_MOTOR_IN2, OUTPUT);

  pinMode(PIN_BTN_START, INPUT_PULLUP);
  pinMode(PIN_BTN_STOP, INPUT_PULLUP);
  pinMode(PIN_BTN_PID, INPUT_PULLUP);

  pinMode(PIN_RPM_INT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_RPM_INT), countPulse, FALLING);

  stopMotorOutput();

  if (!radio.begin()) {
    Serial.println("ERR,NRF24_NOT_FOUND");

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("NRF NOT FOUND");
    lcd.setCursor(0, 1);
    lcd.print("Check wiring");

    while (1);
  }

  radio.setAutoAck(true);
  radio.setRetries(5, 15);
  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(108);

  // Receiver listens for commands on L9CMD
  radio.openReadingPipe(1, pipeToReceiver);

  // Receiver sends telemetry back on L9TLM
  radio.openWritingPipe(pipeToTransmitter);

  radio.startListening();

  Serial.println("READY,RECEIVER");
  Serial.println("WAITING_FOR_COMMANDS");

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Receiver Ready");
  lcd.setCursor(0, 1);
  lcd.print("Waiting CMD...");
}

void loop() {
  unsigned long now = millis();

  readWireless(now);
  handleConnectionTimeout(now);

  if (!isConnected) {
    showWaitingScreen(now);
    return;
  }

  handleButtons();
  runControlLoop(now);

  if (now - lastSensorMs >= 1000) {
    lastSensorMs = now;
    updateSensors();
  }

  if (now - lastDisplayMs >= 500) {
    lastDisplayMs = now;
    updateLCD();
  }
}

void countPulse() {
  pulseCount++;
}

void readWireless(unsigned long now) {
  while (radio.available()) {
    radio.read(&incoming, sizeof(incoming));

    lastPacketMs = now;
    isConnected = true;

    lastSeq = incoming.seq;
    lastCommand = incoming.command;
    lastSetpointPercent = incoming.setpoint;

    if (incoming.command != 'F' && incoming.command != 'R' && incoming.command != 'S') {
      sys.motorStop = true;
      sys.reverse = false;
      sys.setpoint = 0;
      lastCommand = '?';
    } else {
      sys.motorStop = incoming.motorStop || incoming.command == 'S';
      sys.reverse = incoming.command == 'R';
      sys.pidEnabled = incoming.pidEnable;

      if (sys.pidEnabled) {
        sys.setpoint = map(incoming.setpoint, 0, 100, 0, 25000);
      } else {
        sys.setpoint = map(incoming.setpoint, 0, 100, 0, 255);
      }
    }

    Serial.print("RX_CMD,");
    Serial.print(incoming.seq);
    Serial.print(",");
    Serial.print(incoming.command);
    Serial.print(",");
    Serial.print(incoming.setpoint);
    Serial.print(",");
    Serial.print(incoming.pidEnable ? 1 : 0);
    Serial.print(",");
    Serial.println(sys.motorStop ? "STOP" : "RUN");

    // Same working structure as your test:
    // receiver replies right after receiving a command/heartbeat.
    sendTelemetryToTransmitter();
  }
}

void handleConnectionTimeout(unsigned long now) {
  if (isConnected && (now - lastPacketMs > RX_TIMEOUT_MS)) {
    isConnected = false;
    stopMotorOutput();

    Serial.println("CONNECTION_TIMEOUT");
  }
}

void showWaitingScreen(unsigned long now) {
  stopMotorOutput();

  if (now - lastDisplayMs >= 500) {
    lastDisplayMs = now;

    clearLCDRow(0);
    lcd.setCursor(0, 0);
    lcd.print("Status: WAITING");

    clearLCDRow(1);
    lcd.setCursor(0, 1);
    lcd.print("Connect to TX");
  }
}

void handleButtons() {
  if (digitalRead(PIN_BTN_START) == LOW) {
    sys.motorStop = false;
  }

  if (digitalRead(PIN_BTN_STOP) == LOW) {
    sys.motorStop = true;
  }

  static bool lastPidBtn = HIGH;
  bool pidBtn = digitalRead(PIN_BTN_PID);

  if (pidBtn == LOW && lastPidBtn == HIGH) {
    sys.pidEnabled = !sys.pidEnabled;
    delay(50);
  }

  lastPidBtn = pidBtn;
}

void runControlLoop(unsigned long now) {
  if (now - lastControlMs >= Ts_ms) {
    lastControlMs += Ts_ms;

    noInterrupts();
    float currentPulses = pulseCount;
    pulseCount = 0;
    interrupts();

    float rpm_measured = (currentPulses / Ts) * 60.0f / 3.0f;
    rpm_filtered = (alpha * rpm_filtered) + ((1.0f - alpha) * rpm_measured);

    if (sys.motorStop) {
      stopMotorOutput();
    } else {
      digitalWrite(PIN_MOTOR_IN1, sys.reverse ? LOW : HIGH);
      digitalWrite(PIN_MOTOR_IN2, sys.reverse ? HIGH : LOW);

      if (sys.pidEnabled) {
        float error_rpm = (float)sys.setpoint - rpm_filtered;

        float P = Kp * error_rpm;
        float D = -Kd * (rpm_filtered - rpm_prev_filtered) / Ts;
        float I_candidate = Iacc + (Ki * Ts * error_rpm);

        float u_candidate = P + I_candidate + D;

        if (u_candidate >= 0 && u_candidate <= 255) {
          Iacc = I_candidate;
        }

        current_u = constrain((int)(P + Iacc + D), 0, 255);
      } else {
        current_u = constrain(sys.setpoint, 0, 255);
      }

      analogWrite(PIN_MOTOR_PWM, current_u);
    }

    rpm_prev_filtered = rpm_filtered;
  }
}

void updateSensors() {
  int8_t tempRaw = -127;

  Wire.beginTransmission(TC74_ADDR);
  Wire.write(0x00);

  if (Wire.endTransmission() == 0) {
    Wire.requestFrom(TC74_ADDR, (byte)1);

    if (Wire.available()) {
      tempRaw = (int8_t)Wire.read();
    }
  }

  lastTempC = (float)tempRaw;

  float h = dht.readHumidity();
  if (!isnan(h)) {
    lastHumidity = h;
  }

  lastLight = (uint16_t)map(analogRead(PIN_LDR), 0, 1023, 0, 100);
}

void updateLCD() {
  clearLCDRow(0);
  lcd.setCursor(0, 0);
  lcd.print("R:");
  lcd.print((int)rpm_filtered);

  lcd.setCursor(8, 0);
  lcd.print(sys.pidEnabled ? "PID " : "MAN ");
  lcd.print(sys.motorStop ? "STP" : "RUN");

  clearLCDRow(1);
  lcd.setCursor(0, 1);
  lcd.print("C:");
  if (lastTempC <= -100) {
    lcd.print("--");
  } else {
    lcd.print((int)lastTempC);
  }

  lcd.print(" H:");
  lcd.print((int)lastHumidity);

  lcd.print(" L:");
  lcd.print(lastLight);
}

void buildTelemetryPacket() {
  int rpmInt = (int)rpm_filtered;

  if (rpmInt < 0) {
    rpmInt = 0;
  }

  if (rpmInt > 65535) {
    rpmInt = 65535;
  }

  tlmOut.rpm = (uint16_t)rpmInt;
  tlmOut.tempC = lastTempC;
  tlmOut.humidity = lastHumidity;
  tlmOut.light = lastLight;
  tlmOut.pidMode = sys.pidEnabled;
  tlmOut.motorPWM = (uint8_t)current_u;

  if (sys.motorStop) {
    strncpy(tlmOut.status, "STOP", sizeof(tlmOut.status));
  } else if (sys.reverse) {
    strncpy(tlmOut.status, "REVERSE", sizeof(tlmOut.status));
  } else {
    strncpy(tlmOut.status, "FORWARD", sizeof(tlmOut.status));
  }

  tlmOut.status[sizeof(tlmOut.status) - 1] = '\0';
}

void sendTelemetryToTransmitter() {
  buildTelemetryPacket();

  radio.stopListening();

  lastTlmOk = radio.write(&tlmOut, sizeof(tlmOut));

  radio.startListening();

  Serial.print("TLM_REPLY,");
  Serial.println(lastTlmOk ? "RF_OK" : "RF_FAIL");
}

void stopMotorOutput() {
  analogWrite(PIN_MOTOR_PWM, 0);
  digitalWrite(PIN_MOTOR_IN1, LOW);
  digitalWrite(PIN_MOTOR_IN2, LOW);

  Iacc = 0;
  current_u = 0;
}

void clearLCDRow(uint8_t row) {
  lcd.setCursor(0, row);
  lcd.print("                ");
}