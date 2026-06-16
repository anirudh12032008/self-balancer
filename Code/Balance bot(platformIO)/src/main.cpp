#include <Arduino.h>

#include <Wire.h>
#include <MPU6050.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Preferences.h>

// MPU6050 I2C Pins (ESP32 WROOM default I2C pins)
#define SDA_PIN 21
#define SCL_PIN 22

// TB6612FNG Motor Driver Pins (Optimized for ESP32 WROOM, avoiding strapping pins)
#define PWMA_PIN 25
#define PWMB_PIN 26
#define AIN1_PIN 27
#define AIN2_PIN 14
#define BIN1_PIN 32
#define BIN2_PIN 33
#define STBY_PIN 13

// PWM Settings
#define PWM_FREQ 20000
#define PWM_RESOLUTION 8
#define PWM_CHANNEL_A 0
#define PWM_CHANNEL_B 1

// MPU6050 Object
MPU6050 mpu;

// MPU6050 Data
int16_t ax, ay, az;
int16_t gx, gy, gz;
float accAngleX = 0, accAngleY = 0;
float roll = 0, pitch = 0;

// Gyro offsets for calibration
int16_t gxOffset = 0, gyOffset = 0, gzOffset = 0;

// Motor control variables
int motorSpeedA = 0;
int motorSpeedB = 0;

// PID Control Variables
float Kp = 40.0;    // Proportional gain (tune this first)
float Ki = 0.5;     // Integral gain
float Kd = 1.2;     // Derivative gain

float targetAngle = 0.0;  // Target balance angle (adjustable offset)
float lastError = 0.0;
float integral = 0.0;
unsigned long lastTime = 0;

// Balance mode
bool balanceMode = false;  // Auto-balance disabled by default
int manualSpeedA = 0;      // Manual control override
int manualSpeedB = 0;
int maxSpeed = 255;        // Maximum motor speed (adjust reaction speed)
int minSpeed = 40;         // Minimum effective motor speed to overcome deadband

// BLE Server
BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
BLECharacteristic* pControlCharacteristic = NULL;
bool deviceConnected = false;

Preferences prefs;

// BLE UUIDs
#define SERVICE_UUID        "19b10000-e8f2-537e-4f6c-d104768a1214"
#define CHAR_UUID           "19b10001-e8f2-537e-4f6c-d104768a1214"
#define CONTROL_CHAR_UUID   "19b10002-e8f2-537e-4f6c-d104768a1214"

// Forward declarations
void setMotorSpeed(int speedA, int speedB);
void loadSettings();
void saveSettings();

void loadSettings() {
  prefs.begin("balance", true);
  Kp = prefs.getFloat("Kp", Kp);
  Ki = prefs.getFloat("Ki", Ki);
  Kd = prefs.getFloat("Kd", Kd);
  targetAngle = prefs.getFloat("target", targetAngle);
  maxSpeed = prefs.getInt("maxSpeed", maxSpeed);
  minSpeed = prefs.getInt("minSpeed", minSpeed);
  balanceMode = prefs.getBool("balance", balanceMode);
  prefs.end();

  Serial.println("Loaded settings from NVS");
}

void saveSettings() {
  prefs.begin("balance", false);
  prefs.putFloat("Kp", Kp);
  prefs.putFloat("Ki", Ki);
  prefs.putFloat("Kd", Kd);
  prefs.putFloat("target", targetAngle);
  prefs.putInt("maxSpeed", maxSpeed);
  prefs.putInt("minSpeed", minSpeed);
  prefs.putBool("balance", balanceMode);
  prefs.end();
}

class MyCharacteristicCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string value = pCharacteristic->getValue();
      if (value.length() > 0) {
        String command = String(value.c_str());
        
        // Check for PID tuning commands
        if (command.startsWith("KP:")) {
          Kp = command.substring(3).toFloat();
          Serial.print("Kp set to: "); Serial.println(Kp);
          saveSettings();
        } 
        else if (command.startsWith("KI:")) {
          Ki = command.substring(3).toFloat();
          Serial.print("Ki set to: "); Serial.println(Ki);
          saveSettings();
        }
        else if (command.startsWith("KD:")) {
          Kd = command.substring(3).toFloat();
          Serial.print("Kd set to: "); Serial.println(Kd);
          saveSettings();
        }
        else if (command.startsWith("TARGET:")) {
          targetAngle = command.substring(7).toFloat();
          Serial.print("Target angle set to: "); Serial.println(targetAngle);
          saveSettings();
        }
        else if (command.startsWith("SPEED:")) {
          maxSpeed = command.substring(6).toInt();
          maxSpeed = constrain(maxSpeed, 50, 255);
          Serial.print("Max speed set to: "); Serial.println(maxSpeed);
          saveSettings();
        }
        else if (command.startsWith("MIN:")) {
          minSpeed = command.substring(4).toInt();
          minSpeed = constrain(minSpeed, 40, 200);
          Serial.print("Min speed set to: "); Serial.println(minSpeed);
          saveSettings();
        }
        else if (command.startsWith("BALANCE:")) {
          balanceMode = command.substring(8).toInt();
          if (balanceMode) {
            integral = 0; // Reset integral when enabling balance mode
            lastError = 0;
          }
          Serial.print("Balance mode: "); Serial.println(balanceMode ? "ON" : "OFF");
          saveSettings();
        }
        else {
          // Manual motor control (for testing)
          int commaIndex = command.indexOf(',');
          if (commaIndex > 0) {
            manualSpeedA = command.substring(0, commaIndex).toInt();
            manualSpeedB = command.substring(commaIndex + 1).toInt();
            balanceMode = false; // Disable auto-balance in manual mode
            Serial.print("Manual control: A=");
            Serial.print(manualSpeedA);
            Serial.print(", B=");
            Serial.println(manualSpeedB);
          }
        }
      }
    }
};

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      Serial.println("Device connected via BLE");
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      Serial.println("Device disconnected");
      BLEDevice::startAdvertising();
    }
};

void setup() {
  Serial.begin(115200);
  loadSettings();
  
  // CRITICAL: Disable motor driver first (STBY LOW)
  pinMode(STBY_PIN, OUTPUT);
  digitalWrite(STBY_PIN, LOW);
  Serial.println("Motor driver disabled (STBY LOW)");
  
  // Setup motor pins to prevent floating
  pinMode(AIN1_PIN, OUTPUT);
  pinMode(AIN2_PIN, OUTPUT);
  pinMode(BIN1_PIN, OUTPUT);
  pinMode(BIN2_PIN, OUTPUT);
  digitalWrite(AIN1_PIN, LOW);
  digitalWrite(AIN2_PIN, LOW);
  digitalWrite(BIN1_PIN, LOW);
  digitalWrite(BIN2_PIN, LOW);
  
  // Setup PWM for motor speed control
  ledcSetup(PWM_CHANNEL_A, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_B, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(PWMA_PIN, PWM_CHANNEL_A);
  ledcAttachPin(PWMB_PIN, PWM_CHANNEL_B);
  ledcWrite(PWM_CHANNEL_A, 0);
  ledcWrite(PWM_CHANNEL_B, 0);
  
  Serial.println("Motor pins initialized and stopped");
  
  // Initialize I2C for MPU6050 with internal pull-ups and slower clock
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000); // 100kHz - slower, more reliable
  
  Serial.println("Scanning I2C bus...");
  byte error, address;
  int nDevices = 0;
  for(address = 1; address < 127; address++ ) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address<16) Serial.print("0");
      Serial.println(address,HEX);
      nDevices++;
    }
  }
  if (nDevices == 0) {
    Serial.println("No I2C devices found! Check wiring:");
    Serial.println("  MPU6050 VCC -> ESP32 3.3V");
    Serial.println("  MPU6050 GND -> ESP32 GND");
    Serial.println("  MPU6050 SDA -> ESP32 GPIO18");
    Serial.println("  MPU6050 SCL -> ESP32 GPIO19");
    Serial.println("  Add 4.7K pull-up resistors on SDA/SCL if not present on module");
    while(1) { delay(1000); }
  }
  
  // Initialize MPU6050
  Serial.println("Initializing MPU6050...");
  mpu.initialize();
  
  if (mpu.testConnection()) {
    Serial.println("MPU6050 connection successful");
  } else {
    Serial.println("MPU6050 connection failed");
    while(1);
  }
  
  // Configure MPU6050
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  
  // Calibrate gyro
  long gxSum = 0, gySum = 0, gzSum = 0;
  for (int i = 0; i < 100; i++) {
    int16_t gxTemp, gyTemp, gzTemp;
    mpu.getRotation(&gxTemp, &gyTemp, &gzTemp);
    gxSum += gxTemp;
    gySum += gyTemp;
    gzSum += gzTemp;
    delay(10);
  }
  
  gxOffset = gxSum / 100;
  gyOffset = gySum / 100;
  gzOffset = gzSum / 100;
  
  Serial.println("MPU6050 Ready!");
  
  // Create BLE Device
  BLEDevice::init("ESP32_BalanceBot");
  
  // Create BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  
  // Create BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);
  
  // Create BLE Characteristic
  pCharacteristic = pService->createCharacteristic(
                      CHAR_UUID,
                      BLECharacteristic::PROPERTY_READ |
                      BLECharacteristic::PROPERTY_NOTIFY
                    );
  
  pCharacteristic->addDescriptor(new BLE2902());
  pCharacteristic->setValue("Hello World!");
  
  // Create BLE Control Characteristic (for motor commands)
  pControlCharacteristic = pService->createCharacteristic(
                      CONTROL_CHAR_UUID,
                      BLECharacteristic::PROPERTY_WRITE
                    );
  pControlCharacteristic->setCallbacks(new MyCharacteristicCallbacks());
  
  // Start service
  pService->start();
  
  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);
  BLEDevice::startAdvertising();
  
  Serial.println("BLE device started!");
  Serial.println("Device name: ESP32_BalanceBot");
  Serial.println("Open hello.html in Chrome to see MPU6050 data!");
  
  // Wait 5 seconds before enabling motor driver
  Serial.println("\nWaiting 5 seconds before enabling motor driver...");
  for (int i = 5; i > 0; i--) {
    Serial.print(i);
    Serial.println(" seconds...");
    delay(1000);
  }
  
  // Enable motor driver (STBY HIGH)
  digitalWrite(STBY_PIN, HIGH);
  Serial.println("\n✓ Motor driver ENABLED! Ready to control motors.");
}

void setMotorA(int speed) {
  if (speed > 0) {
    digitalWrite(AIN1_PIN, HIGH);
    digitalWrite(AIN2_PIN, LOW);
    ledcWrite(PWM_CHANNEL_A, speed);
  } else if (speed < 0) {
    digitalWrite(AIN1_PIN, LOW);
    digitalWrite(AIN2_PIN, HIGH);
    ledcWrite(PWM_CHANNEL_A, -speed);
  } else {
    digitalWrite(AIN1_PIN, LOW);
    digitalWrite(AIN2_PIN, LOW);
    ledcWrite(PWM_CHANNEL_A, 0);
  }
}

void setMotorB(int speed) {
  if (speed > 0) {
    digitalWrite(BIN1_PIN, HIGH);
    digitalWrite(BIN2_PIN, LOW);
    ledcWrite(PWM_CHANNEL_B, speed);
  } else if (speed < 0) {
    digitalWrite(BIN1_PIN, LOW);
    digitalWrite(BIN2_PIN, HIGH);
    ledcWrite(PWM_CHANNEL_B, -speed);
  } else {
    digitalWrite(BIN1_PIN, LOW);
    digitalWrite(BIN2_PIN, LOW);
    ledcWrite(PWM_CHANNEL_B, 0);
  }
}

void setMotorSpeed(int speedA, int speedB) {
  speedA = constrain(speedA, -255, 255);
  speedB = constrain(speedB, -255, 255);
  motorSpeedA = speedA;
  motorSpeedB = speedB;
  setMotorA(speedA);
  setMotorB(speedB);
}

void calculateAngles() {
  // Calculate accelerometer angles
  accAngleX = atan2(ay, sqrt(ax * ax + az * az)) * 180 / PI;
  accAngleY = atan2(-ax, sqrt(ay * ay + az * az)) * 180 / PI;
  
  // Calculate gyro angles (integrate gyro data)
  float dt = 0.01;  // 10ms sampling time
  
  // Complementary filter (combines accelerometer and gyro data)
  float alpha = 0.98;
  roll = alpha * (roll + (gx / 131.0) * dt) + (1 - alpha) * accAngleX;
  pitch = alpha * (pitch + (gy / 131.0) * dt) + (1 - alpha) * accAngleY;
}

float calculatePID(float currentAngle) {
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0; // Convert to seconds
  lastTime = currentTime;
  
  if (dt > 0.1) dt = 0.01; // Limit dt to prevent spikes on first run
  
  // Calculate error
  float error = targetAngle - currentAngle;
  
  // Proportional term
  float P = Kp * error;
  
  // Integral term (with anti-windup)
  integral += error * dt;
  integral = constrain(integral, -100, 100); // Prevent integral windup
  float I = Ki * integral;
  
  // Derivative term
  float derivative = (error - lastError) / dt;
  float D = Kd * derivative;
  lastError = error;
  
  // Calculate output
  float output = P + I + D;

  // Apply deadband: if below minSpeed, stop motors
  if (abs(output) < minSpeed) {
    output = 0;
  }
  
  return constrain(output, -maxSpeed, maxSpeed);
}

void balanceRobot() {
  // Calculate angles
  calculateAngles();
  
  if (balanceMode) {
    // PID control for balance
    float pidOutput = calculatePID(pitch);
    
    // If bot falls over too much, stop motors
    if (abs(pitch) > 45) {
      setMotorSpeed(0, 0);
      integral = 0; // Reset integral
      return;
    }
    
    // Apply PID output to motors
    int speed = (int)pidOutput;
    setMotorSpeed(speed, speed);
  } else {
    // Manual control mode
    setMotorSpeed(manualSpeedA, manualSpeedB);
  }
}

void loop() {
  // Read MPU6050 data
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  // Apply gyro calibration
  gx -= gxOffset;
  gy -= gyOffset;
  gz -= gzOffset;
  
  // Run balance control
  balanceRobot();
  
  // Send data via BLE (every 100ms to reduce load)
  static unsigned long lastBLEUpdate = 0;
  if (deviceConnected && (millis() - lastBLEUpdate > 100)) {
    lastBLEUpdate = millis();
    
    String jsonData = "{";
    jsonData += "\"ax\":" + String(ax) + ",";
    jsonData += "\"ay\":" + String(ay) + ",";
    jsonData += "\"az\":" + String(az) + ",";
    jsonData += "\"gx\":" + String(gx) + ",";
    jsonData += "\"gy\":" + String(gy) + ",";
    jsonData += "\"gz\":" + String(gz) + ",";
    jsonData += "\"roll\":" + String(roll, 2) + ",";
    jsonData += "\"pitch\":" + String(pitch, 2) + ",";
    jsonData += "\"motorA\":" + String(motorSpeedA) + ",";
    jsonData += "\"motorB\":" + String(motorSpeedB) + ",";
    jsonData += "\"Kp\":" + String(Kp, 2) + ",";
    jsonData += "\"Ki\":" + String(Ki, 3) + ",";
    jsonData += "\"Kd\":" + String(Kd, 2) + ",";
    jsonData += "\"target\":" + String(targetAngle, 2) + ",";
    jsonData += "\"maxSpeed\":" + String(maxSpeed) + ",";
    jsonData += "\"minSpeed\":" + String(minSpeed) + ",";
    jsonData += "\"balance\":" + String(balanceMode);
    jsonData += "}";
    
    pCharacteristic->setValue(jsonData.c_str());
    pCharacteristic->notify();
  }
  
  delay(10); // 100Hz control loop
}
