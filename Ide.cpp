#include <WiFi.h>
#include <FirebaseESP32.h>
#include <Wire.h>
#include <MPU6050.h>
#include <math.h>


// ===== WIFI =====
#define WIFI_SSID "SKYFiber_2.4GHz_6cZ4"
#define WIFI_PASSWORD "D9m3yGgv"


// ===== FIREBASE =====
#define FIREBASE_HOST "https://geopulse-khs2025-default-rtdb.asia-southeast1.firebasedatabase.app/"
#define FIREBASE_AUTH "YOUR_DATABASE_SECRET"


FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;


// ===== MPU6050 =====
MPU6050 mpu;
int16_t ax_raw, ay_raw, az_raw;
int16_t gx_raw, gy_raw, gz_raw;
float pitch = 0, roll = 0, yaw = 0;
float gx_bias = 0, gy_bias = 0, gz_bias = 0;
unsigned long lastTime = 0;
const float alpha = 0.98;
const int CALIBRATION_SAMPLES = 200;


// ===== Vibration =====
const int Vibration_signal = 23;
float vibration_total = 0;
const float VIB_MIN_THRESH = 0.1;   // cm/s² (lower bound)
const float VIB_MAX_THRESH = 0.3;   // cm/s² (upper bound of interest)
const float VIB_INCREMENT = 0.004; // cm/s² increment per detection
unsigned long lastVibrationTime = 0;
const unsigned long RESET_TIMEOUT = 10000;


// ===== Soil Moisture =====
#define SOIL_PIN 32
int airValue = 3200;
int waterValue = 1200;
int soilValue = 0;
int moisturePercent = 0;
const int SOIL_THRESH = 75; // ~75% VWC


// ===== Rain Gauge =====
const int RAIN_PIN = 27;
float mmPerTip = 0.2794;
int lastState = HIGH;
unsigned long lastDebounce = 0;
unsigned long lastTipMillis = 0;
float totalRain = 0.0;
float currentRain = 0.0;


// ===== New Rain Per Minute =====
float perMinRain = 0.0;
unsigned long perMinStart = 0;
const float RAIN_HOURLY_THRESH = 10.0;   // mm/h
const float RAIN_DAILY_THRESH  = 60.0;   // mm/day


// ===== Gyro Thresholding (no baseline) =====
const float GYRO_THRESH_TRIGGER = 25.0;  // trigger if abs(angle) >= 25°
const float GYRO_THRESH_CLEAR   = 20.0;  // clear only when falls below 20° (hysteresis)
const unsigned long GYRO_HOLD_TIME = 30000UL; // must persist for 30 seconds


// Small smoothing for angles (EMA)
const float ANGLE_EMA_ALPHA = 0.1; // between 0 (slow) and 1 (no smoothing)
float pitch_smoothed = 0.0;
float roll_smoothed  = 0.0;


// Timer for sustained detection
unsigned long gyroTriggerStart = 0;
bool gyroCurrentlyCounting = false;
bool gyroTriggered = false;


// ===== Buzzer =====
const int buzzerPin = 2;


// ===== Timing =====
unsigned long lastPrintTime = 0;
const unsigned long PRINT_INTERVAL = 5000;


// ===== Smart Trigger Flags =====
bool vibTriggered = false;
bool soilTriggered = false;
bool rainTriggered = false;


// ===== Functions =====
bool evaluateAlarm() {
  // Vibration triggered if above lower bound
  vibTriggered  = (vibration_total >= VIB_MIN_THRESH && vibration_total <= VIB_MAX_THRESH);


  // Soil threshold
  soilTriggered = (moisturePercent >= SOIL_THRESH);


  // Rain threshold
  rainTriggered = (perMinRain * 60 >= RAIN_HOURLY_THRESH) || (totalRain >= RAIN_DAILY_THRESH);


  // Gyro trigger needs both pitch and roll above threshold for sustained time
  bool pitchBad = fabs(pitch_smoothed) >= GYRO_THRESH_TRIGGER;
  bool rollBad  = fabs(roll_smoothed)  >= GYRO_THRESH_TRIGGER;


  if (pitchBad && rollBad) {
    if (!gyroCurrentlyCounting) {
      gyroTriggerStart = millis();
      gyroCurrentlyCounting = true;
    } else {
      if (millis() - gyroTriggerStart >= GYRO_HOLD_TIME) {
        gyroTriggered = true;
      }
    }
  } else {
    if (fabs(pitch_smoothed) < GYRO_THRESH_CLEAR && fabs(roll_smoothed) < GYRO_THRESH_CLEAR) {
      gyroCurrentlyCounting = false;
      gyroTriggerStart = 0;
      gyroTriggered = false;
    }
  }


  int triggerCount = vibTriggered + soilTriggered + rainTriggered + (gyroTriggered ? 1 : 0);
  return (triggerCount >= 2);
}


// ===== SETUP =====
void setup() {
  Serial.begin(9600);
  Wire.begin();


  pinMode(buzzerPin, OUTPUT);
  pinMode(Vibration_signal, INPUT);
  pinMode(SOIL_PIN, INPUT);
  pinMode(RAIN_PIN, INPUT_PULLUP);


  // Connect WiFi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" CONNECTED!");


  // Firebase
  config.host = FIREBASE_HOST;
  config.signer.tokens.legacy_token = FIREBASE_AUTH;
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);


  // MPU6050 init
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    while (1);
  }


  // Gyro calibration
  long gx_sum = 0, gy_sum = 0, gz_sum = 0;
  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    mpu.getRotation(&gx_raw, &gy_raw, &gz_raw);
    gx_sum += gx_raw;
    gy_sum += gy_raw;
    gz_sum += gz_raw;
    delay(5);
  }
  gx_bias = gx_sum / (float)CALIBRATION_SAMPLES;
  gy_bias = gy_sum / (float)CALIBRATION_SAMPLES;
  gz_bias = gz_sum / (float)CALIBRATION_SAMPLES;


  lastTime = millis();
  lastPrintTime = millis();
  perMinStart = millis();


  Serial.println("System Initialized!");
}


// ===== LOOP =====
void loop() {
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  if (dt <= 0) dt = 0.001;
  lastTime = now;


  // MPU6050 readings
  mpu.getAcceleration(&ax_raw, &ay_raw, &az_raw);
  mpu.getRotation(&gx_raw, &gy_raw, &gz_raw);


  float gx_deg = (gx_raw - gx_bias) / 131.0;
  float gy_deg = (gy_raw - gy_bias) / 131.0;
  float gz_deg = (gz_raw - gz_bias) / 131.0;


  float ax_g = ax_raw / 16384.0;
  float ay_g = ay_raw / 16384.0;
  float az_g = az_raw / 16384.0;


  float pitch_acc = atan2(ax_g, sqrt(ay_g*ay_g + az_g*az_g)) * 180.0 / PI;
  float roll_acc  = atan2(ay_g, sqrt(ax_g*ax_g + az_g*az_g)) * 180.0 / PI;


  pitch = alpha * (pitch + gy_deg * dt) + (1 - alpha) * pitch_acc;
  roll  = alpha * (roll + gx_deg * dt) + (1 - alpha) * roll_acc;
  yaw   += gz_deg * dt;
  if(yaw > 360) yaw -= 360;
  if(yaw < 0) yaw += 360;


  pitch_smoothed = (ANGLE_EMA_ALPHA * pitch) + ((1.0 - ANGLE_EMA_ALPHA) * pitch_smoothed);
  roll_smoothed  = (ANGLE_EMA_ALPHA * roll)  + ((1.0 - ANGLE_EMA_ALPHA) * roll_smoothed);


  // Vibration
  int sensorState = digitalRead(Vibration_signal);
  if(sensorState == HIGH){
    vibration_total += VIB_INCREMENT;
    lastVibrationTime = now;
    delay(200);
  } else if(now - lastVibrationTime >= RESET_TIMEOUT){
    vibration_total = 0;
  }


  // Soil moisture
  long sum = 0;
  for(int i=0;i<10;i++) sum += analogRead(SOIL_PIN);
  soilValue = sum/10;
  moisturePercent = map(soilValue, airValue, waterValue,0,100);
  moisturePercent = constrain(moisturePercent,0,100);


  // Rain gauge
  int state = digitalRead(RAIN_PIN);
  if(lastState==HIGH && state==LOW && now-lastDebounce>200){
    totalRain += mmPerTip;        
    currentRain = mmPerTip;      
    perMinRain += mmPerTip;      
    lastTipMillis = now;
    lastDebounce = now;
  }
  if(currentRain>0 && now-lastTipMillis>5000) currentRain = 0;
  lastState = state;


  if(now - perMinStart >= 60000){
    perMinRain = 0;
    perMinStart = now;
  }


  // Multi-sensor alarm
  bool alarm = evaluateAlarm();
  digitalWrite(buzzerPin, alarm ? HIGH : LOW);


  // ===== Alarm indicator =====
  static bool lastAlarmState = false;
  if (alarm && !lastAlarmState) {
    Serial.println("=== ALARM TRIGGERED! ===");
  }
  if (!alarm && lastAlarmState) {
    Serial.println("=== Alarm Cleared ===");
  }
  lastAlarmState = alarm;


  // Print & Firebase upload
  if(now - lastPrintTime >= PRINT_INTERVAL){
    lastPrintTime = now;


    Serial.print("Pitch: "); Serial.print(pitch,1);
    Serial.print(" | Roll: "); Serial.print(roll,1);
    Serial.print(" | PS: "); Serial.print(pitch_smoothed,2);
    Serial.print(" | RS: "); Serial.print(roll_smoothed,2);
    Serial.print(" | Vib: "); Serial.print(vibration_total,3);
    Serial.print(" | Soil: "); Serial.print(moisturePercent); Serial.print("%");
    Serial.print(" | Rain/min: "); Serial.print(perMinRain,3);
    Serial.print(" | Total Rain: "); Serial.println(totalRain,3);
    Serial.print(" | GyroTrig: "); Serial.print(gyroTriggered ? "YES" : "NO");
    Serial.print(" | Alarm: "); Serial.println(alarm ? "YES" : "NO");


    Firebase.setFloat(fbdo,"/Pitch",pitch);
    Firebase.setFloat(fbdo,"/Roll",roll);
    Firebase.setFloat(fbdo,"/Yaw",yaw);
    Firebase.setFloat(fbdo,"/Vibration",vibration_total);
    Firebase.setInt(fbdo,"/Moisture",moisturePercent);
    Firebase.setFloat(fbdo,"/RainPerMinute",perMinRain);
    Firebase.setFloat(fbdo,"/RainTotal",totalRain);
    Firebase.setBool(fbdo,"/GyroTriggered",gyroTriggered);
    Firebase.setBool(fbdo,"/Alarm",alarm);
  }
}