#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include <esp_now.h>
#include <WiFi.h>

MPU6050 mpu;

float delta_thresholds[6] = {0.5, 0.5, 0.5, 25.0, 25.0, 25.0}; // Thresholds for x, y, z, roll, pitch, yaw

// MPU6050 offsets (adjust according to your sensor)
int16_t accelOffsets[3] = {0, 0, 0};   // Accelerometer offsets
int16_t gyroOffsets[3] = {0, 0, 0};    // Gyroscope offsets
float lastLoop[6];

// Variables for storing sensor data
int16_t accelRaw[3];  // Raw accelerometer values
int16_t gyroRaw[3];   // Raw gyroscope values

// Variables for derived data (acceleration in g's and angular velocity in °/s)
float accelG[3];  // Acceleration in g's (converted from raw accelerometer values)
float gyroDPS[3]; // Gyroscope angular velocity in degrees per second

// Threshold exceedance direction variables
int accelXThresholdDir;
int accelYThresholdDir;
int accelZThresholdDir;
int gyroXThresholdDir;
int gyroYThresholdDir;
int gyroZThresholdDir;


float avg_acx = 0.0;
float avg_acy = 0.0;
float avg_acz = 0.0;
float avg_gyx = 0.0;
float avg_gyy = 0.0;
float avg_gyz = 0.0;

// Structure for storing sensor values
typedef struct {
  float x;
  float y;
  float z;
  float pitch;
  float roll;
  float yaw;
} sensor_values;

sensor_values info;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  char macStr[18];
  Serial.print("Packet to: ");
  // Copies the sender mac address to a string
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print(macStr);
  Serial.print(" send status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// Function to determine threshold exceedance direction
int getThresholdDirection(float currentValue, float lastValue, float threshold) {
  if (currentValue > lastValue + threshold) {
    return 1; // Exceeded in positive direction
  } else if (currentValue < lastValue - threshold) {
    return 2; // Exceeded in negative direction
  }
  return 0; // Not exceeded
}

void latch(int arg, float &store) {
    if ((arg == 2 && store ==1)||(arg == 1 && store == 2)) {
        store = 0;
        delay(100);
    }
    else if (store == 0){
      store = arg;
    }
}

void setup() {
  Wire.begin();
  Serial.begin(9600);

  mpu.initialize();

  WiFi.mode(WIFI_STA);
 
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  esp_now_register_send_cb(OnDataSent);
   
  uint8_t broadcastAddress1[] = {0xA8,0x42,0xE3,0x59,0xA5,0xD4};

  esp_now_peer_info_t peerInfo;
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  memcpy(peerInfo.peer_addr, broadcastAddress1, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  
  // Calibrate MPU6050 offsets if needed
  mpu.CalibrateAccel(6);
  mpu.CalibrateGyro(6);

  // Store the offsets for later use
  mpu.getAcceleration(&accelRaw[0], &accelRaw[1], &accelRaw[2]);
  accelOffsets[0] = accelRaw[0];
  accelOffsets[1] = accelRaw[1];
  accelOffsets[2] = accelRaw[2];
  mpu.getRotation(&gyroRaw[0], &gyroRaw[1], &gyroRaw[2]);
  gyroOffsets[0] = gyroRaw[0];
  gyroOffsets[1] = gyroRaw[1];
  gyroOffsets[2] = gyroRaw[2];

  //set initial movement to 0
  info.x = 0;
  info.y = 0;
  info.z = 0;
  info.pitch = 0;
  info.roll =  0;
  info.yaw = 0;

}

void loop() {

  mpu.getMotion6(&accelRaw[0], &accelRaw[1], &accelRaw[2], &gyroRaw[0], &gyroRaw[1], &gyroRaw[2]);

    for (int j = 0; j < 3; ++j) {
      lastLoop[j] = (float)(accelRaw[j] - accelOffsets[j]) / 16384.0;  // 16384 LSB/g for MPU6050
      lastLoop[j+3] = (float)(gyroRaw[j] - gyroOffsets[j]) / 131.0;    // 131 LSB/(°/s) for MPU6050
    }

  delay(10);
  // Accumulate readings for averaging
  for (int i = 0; i < 25; ++i) {
    mpu.getMotion6(&accelRaw[0], &accelRaw[1], &accelRaw[2], &gyroRaw[0], &gyroRaw[1], &gyroRaw[2]);

    for (int j = 0; j < 3; ++j) {
      accelG[j] = (float)(accelRaw[j] - accelOffsets[j]) / 16384.0;  // 16384 LSB/g for MPU6050
      gyroDPS[j] = (float)(gyroRaw[j] - gyroOffsets[j]) / 131.0;    // 131 LSB/(°/s) for MPU6050
    }

    avg_acx += accelG[0];
    avg_acy += accelG[1];
    avg_acz += accelG[2];
    avg_gyx += gyroDPS[0];
    avg_gyy += gyroDPS[1];
    avg_gyz += gyroDPS[2];

    // Threshold exceedance direction variables
    accelXThresholdDir = getThresholdDirection(accelG[0], lastLoop[0], delta_thresholds[0]);
    accelYThresholdDir = getThresholdDirection(accelG[1], lastLoop[1], delta_thresholds[1]);
    accelZThresholdDir = getThresholdDirection(accelG[2], lastLoop[2], delta_thresholds[2]);
    gyroXThresholdDir = getThresholdDirection(gyroDPS[0], lastLoop[3], delta_thresholds[3]);
    gyroYThresholdDir = getThresholdDirection(gyroDPS[1], lastLoop[4], delta_thresholds[4]);
    gyroZThresholdDir = getThresholdDirection(gyroDPS[2], lastLoop[5], delta_thresholds[5]);

    latch(accelXThresholdDir, info.x);
    latch(accelYThresholdDir, info.y);
    latch(accelZThresholdDir, info.z);
    latch(gyroXThresholdDir, info.pitch);
    latch(gyroYThresholdDir, info.roll);
    latch(gyroZThresholdDir, info.yaw);
    

    lastLoop[0] = accelG[0];
    lastLoop[1] = accelG[1];
    lastLoop[2] = accelG[2];
    lastLoop[3] = gyroDPS[0];
    lastLoop[4] = gyroDPS[1];
    lastLoop[5] = gyroDPS[2];

    delay(10); // Adjust delay as needed
  }

  // Calculate averages
  avg_acx /= 25.0;
  avg_acy /= 25.0;
  avg_acz /= 25.0;
  avg_gyx /= 25.0;
  avg_gyy /= 25.0;
  avg_gyz /= 25.0;

  // Print data
  Serial.print("X: ");
  Serial.print(avg_acx);
  Serial.print("   Y: ");
  Serial.print(avg_acy);
  Serial.print("   Z: ");
  Serial.print(avg_acz);
  Serial.print("   GX: ");
  Serial.print(avg_gyx);
  Serial.print("   GY: ");
  Serial.print(avg_gyy);
  Serial.print("   GZ: ");
  Serial.println(avg_gyz);

    // Print motion
  Serial.print("Xmotion: ");
  Serial.print(info.x);
  Serial.print("   Ymotion: ");
  Serial.print(info.y);
  Serial.print("   Zmotion: ");
  Serial.print(info.z);
  Serial.print("   GXmotion: ");
  Serial.print(info.pitch);
  Serial.print("   GYmotion: ");
  Serial.print(info.roll);
  Serial.print("   GZmotion: ");
  Serial.println(info.yaw);

  // Send data using ESP-NOW
  esp_err_t result = esp_now_send(0, (uint8_t *)&info, sizeof(sensor_values));
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  } else {
    Serial.println("Error sending the data");
  }

}
