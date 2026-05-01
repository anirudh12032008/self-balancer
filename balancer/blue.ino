#include <Wire.h>
#include <MPU6050.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// MPU6050 I2C Pins
#define SDA_PIN 4
#define SCL_PIN 5

// MPU6050 Object
MPU6050 mpu;

// MPU6050 Data
int16_t ax, ay, az;
int16_t gx, gy, gz;
float accAngleX = 0, accAngleY = 0;
float roll = 0, pitch = 0;

// Gyro offsets for calibration
int16_t gxOffset = 0, gyOffset = 0, gzOffset = 0;

// BLE Server
BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
bool deviceConnected = false;

// BLE UUIDs
#define SERVICE_UUID        "19b10000-e8f2-537e-4f6c-d104768a1214"
#define CHAR_UUID           "19b10001-e8f2-537e-4f6c-d104768a1214"

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
  
  // Initialize I2C for MPU6050
  Wire.begin(SDA_PIN, SCL_PIN);
  
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
  pCharacteristic->addDescriptor(new BLE2902());
  
  // Start service
  pService->start();
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
  BLEDevice::init("ESP32");
  
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
  
  // Start service
  pService->start();
  
  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);
  BLEDevice::startAdvertising();
  
  Serial.println("BLE device started!");
  Serial.println("BLE device started!");
  Serial.println("Device name: ESP32");
  Serial.println("Open hello.html in Chrome to see MPU6050 data!");
}

void loop() {
  // Read MPU6050 data
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  // Apply gyro calibration
  gx -= gxOffset;
  gy -= gyOffset;
  gz -= gzOffset;
  
  // Calculate angles
  calculateAngles();
  
  // Send data via BLE
  if (deviceConnected) {
    String jsonData = "{";
    jsonData += "\"ax\":" + String(ax) + ",";
    jsonData += "\"ay\":" + String(ay) + ",";
    jsonData += "\"az\":" + String(az) + ",";
    jsonData += "\"gx\":" + String(gx) + ",";
    jsonData += "\"gy\":" + String(gy) + ",";
    jsonData += "\"gz\":" + String(gz) + ",";
    jsonData += "\"roll\":" + String(roll, 2) + ",";
    jsonData += "\"pitch\":" + String(pitch, 2);
    jsonData += "}";
    
    pCharacteristic->setValue(jsonData.c_str());
    pCharacteristic->notify();
  }
  
  delay(100);
}

void calculateAngles() {
  // Calculate accelerometer angles
  accAngleX = atan2(ay, sqrt(ax * ax + az * az)) * 180 / PI;
  accAngleY = atan2(-ax, sqrt(ay * ay + az * az)) * 180 / PI;
  
  // Calculate gyro angles (integrate gyro data)
  float dt = 0.1;  // 100ms sampling time
  
  // Complementary filter (combines accelerometer and gyro data)
  float alpha = 0.96;
  roll = alpha * (roll + (gx / 131.0) * dt) + (1 - alpha) * accAngleX;
  pitch = alpha * (pitch + (gy / 131.0) * dt) + (1 - alpha) * accAngleY;
}

