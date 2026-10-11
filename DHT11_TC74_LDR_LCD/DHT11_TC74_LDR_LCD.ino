#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>

// ---------- Pin Definitions ----------
#define LDR_PIN A0          // Light Sensor 
#define DHT_PIN 5           // DHT11 Data Pin 
#define DHT_TYPE DHT11

// ---------- I2C Addresses ----------
#define TC74_ADDR 0x48      // TC74 Temperature Sensor [cite: 247]
#define LCD_ADDR 0x27       // 16x2 LCD [cite: 192]

// ---------- Objects & Variables ----------
LiquidCrystal_I2C lcd(LCD_ADDR, 16, 2);
DHT dht(DHT_PIN, DHT_TYPE);

unsigned long lastDisplayMs = 0;
const unsigned long displayInterval = 500; // 500ms update cycle [cite: 211]

void setup() {
  Serial.begin(115200); // Faster baud rate as established in Lab 6 [cite: 250]
  
  // Initialize I2C and LCD
  Wire.begin(); 
  lcd.init();
  lcd.backlight();
  
  // Initialize DHT11
  dht.begin();
  
  lcd.setCursor(0,0);
  lcd.print("Sensor System");
  lcd.setCursor(0,1);
  lcd.print("Initializing...");
  delay(1000); 
  lcd.clear();
}

void loop() {
  unsigned long now = millis();

  // Update Display every 500ms to avoid blocking the CPU [cite: 211, 230]
  if (now - lastDisplayMs >= displayInterval) {
    lastDisplayMs = now;

    // 1. Read LDR (Light)
    int ldrVal = analogRead(LDR_PIN); // Returns 0-1023 [cite: 99]

    // 2. Read TC74A0 (Temperature) via I2C
    int8_t tempTC74 = -127; 
    Wire.beginTransmission(TC74_ADDR);
    Wire.write(0x00);
    if (Wire.endTransmission() == 0) {
      Wire.requestFrom(TC74_ADDR, (byte)1);
      if (Wire.available()) tempTC74 = (int8_t)Wire.read();
    }

    // 3. Read DHT11 (Humidity)
    float humidity = dht.readHumidity();

    // ---------- Update LCD Display ----------
    lcd.setCursor(0, 0);
    // Line 1: Light and TC74 Temp
    lcd.print("L:");
    lcd.print(ldrVal);
    lcd.print(" T1:");
    if (tempTC74 == -127) lcd.print("--");
    else lcd.print(tempTC74);
    lcd.print("C  "); // Extra spaces to clear old digits

    lcd.setCursor(0, 1);
    // Line 2: DHT11 Humidity
    lcd.print("H:");
    if (isnan(humidity)) {
      lcd.print("--%");
    } else {
      lcd.print((int)humidity);
      lcd.print("%");
    }
    
    // Optional: Add some spacing or status info
    lcd.print(" System OK ");

    // ---------- Serial Telemetry for LabVIEW [cite: 337, 339] ----------
    Serial.print("LDR:"); Serial.print(ldrVal);
    Serial.print(",TC74:"); Serial.print(tempTC74);
    Serial.print(",HUM:"); Serial.println(humidity);
  }
}