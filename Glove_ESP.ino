#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include <esp_now.h>
#include <WiFi.h>

MPU6050 mpu;

// Selectable I2C address
#define MPU6050_ADDRESS_1 0x68
#define MPU6050_ADDRESS_2 0x69
uint8_t mpu_address = MPU6050_ADDRESS_1; // Default address, change if needed

// Finger declaration
const int ringFinger = 39; // GPIO 13 corresponds to the EN pin
const int middleFinger = 34; // GPIO 36 corresponds to the VP pin
const int pointerFinger = 35; // GPIO 39 corresponds to the VN pin

int ring;
int middle;
int pointer;

float delta_thresholds[6] = {0.3, 0.3, 0.3, 6000.0, 70.0, 120.0}; // Thresholds for x, y, z, roll, pitch, yaw

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
int ThresholdDir[6] = {0};

//very rough way of skipping latch statements
bool pauseBuffer; 

float avg_acx = 0.0;
float avg_acy = 0.0;
float avg_acz = 0.0;
float avg_gyx = 0.0;
float avg_gyy = 0.0;
float avg_gyz = 0.0;

// Structure for storing sensor values
typedef struct {
  int fingers;
  float rawx;
  float rawy;
  float rawz;
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

void chooseDirection(int a, int b, int thresholdDir[]){
  if(thresholdDir[b+3]==0 && thresholdDir[a]!=0){
    for(int i=0; i<6; i++){
      if(i!=a){
        thresholdDir[i]=0;
      }
    }
  }
  else if(thresholdDir[b+3]!=0){
    for(int i=0; i<6; i++){
      if(i!=(b+3)){
        thresholdDir[i]=0;
      }
    }
  }
}

void latch(int arg, float &store, sensor_values info, bool &ret) {
    
      if ((arg == 2 && store ==1)||(arg == 1 && store == 2)) {
          store = 0;
          ret = true;
      }
      else if (store == 0 && info.x == 0 && info.y == 0 && info.z == 0 && info.roll == 0 && info.pitch == 0 && info.yaw == 0){
        store = arg;
        ret = true;
      }
      else{
        ret = false;
      }
    
}

int greatestDiff(float acc[], float last[]){
  int ret = 0;
  float biggest = abs(acc[0]-last[0]); 
  float compare;

  for(int i=1; i<3; i++){
    compare = abs(acc[i]-last[i]);
    if(biggest<compare){
      biggest = compare;
      ret = i;
    }
  }
  return ret;
}

int finglation(int ring, int middle, int pointer){

  //int ringdiff = abs(ring-1700);
  //int middlediff = abs(middle-1700);
  //int pointerdiff = abs(pointer-1700);

  if (ring>2000 || ring<1400){
    return 1;
  }
  else if(pointer>2000 || pointer<1400){
    return 2;
  }
  else{
    return 0;
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

  pauseBuffer = false;

  ring = analogRead(ringFinger);
  middle = analogRead(middleFinger);
  pointer = analogRead(pointerFinger);

  info.fingers = finglation(ring, middle, pointer);

  mpu.getMotion6(&accelRaw[0], &accelRaw[1], &accelRaw[2], &gyroRaw[0], &gyroRaw[1], &gyroRaw[2]);

    for (int j = 0; j < 3; ++j) {
      lastLoop[j] = (float)(accelRaw[j]) / 16384.0;  // 16384 LSB/g for MPU6050
      lastLoop[j+3] = (float)(gyroRaw[j] - gyroOffsets[j]) / 131.0;    // 131 LSB/(°/s) for MPU6050
    }

  delay(10);

  // Accumulate readings for averaging
  for (int i = 0; i < 25; ++i) {
    mpu.getMotion6(&accelRaw[0], &accelRaw[1], &accelRaw[2], &gyroRaw[0], &gyroRaw[1], &gyroRaw[2]);

    for (int j = 0; j < 3; ++j) {
      accelG[j] = (float)(accelRaw[j]) / 16384.0;  // 16384 LSB/g for MPU6050
      gyroDPS[j] = (float)(gyroRaw[j] - gyroOffsets[j]) / 131.0;    // 131 LSB/(°/s) for MPU6050
    }

    avg_acx += accelG[0];
    avg_acy += accelG[1];
    avg_acz += accelG[2];
    avg_gyx += gyroDPS[0];
    avg_gyy += gyroDPS[1];
    avg_gyz += gyroDPS[2];

        // Threshold exceedance direction variables
    ThresholdDir[0] = getThresholdDirection(accelG[0], lastLoop[0], delta_thresholds[0]);
    ThresholdDir[1] = getThresholdDirection(accelG[1], lastLoop[1], delta_thresholds[1]);
    ThresholdDir[2] = getThresholdDirection(accelG[2], lastLoop[2], delta_thresholds[2]);
    ThresholdDir[3] = getThresholdDirection(gyroDPS[0], lastLoop[3], delta_thresholds[3]);
    ThresholdDir[4] = getThresholdDirection(gyroDPS[1], lastLoop[4], delta_thresholds[4]);
    ThresholdDir[5] = getThresholdDirection(gyroDPS[2], lastLoop[5], delta_thresholds[5]);

    int gda = greatestDiff(accelG, lastLoop);
    int gdg = greatestDiff(gyroDPS, lastLoop);

    chooseDirection(gda, gdg, ThresholdDir);

    //Serial.print("ThresholdDir[0]: "); Serial.print(ThresholdDir[0]); Serial.print("ThresholdDir[1]: "); Serial.print(ThresholdDir[1]); Serial.print("ThresholdDir[2]: "); Serial.print(ThresholdDir[2]); Serial.print("ThresholdDir[3]: "); Serial.print(ThresholdDir[3]); Serial.print("ThresholdDir[4]: "); Serial.print(ThresholdDir[4]); Serial.print("ThresholdDir[5]: "); Serial.println(ThresholdDir[5]);
    latch(ThresholdDir[0], info.x, info, pauseBuffer);
    latch(ThresholdDir[1], info.y, info, pauseBuffer);
    latch(ThresholdDir[2], info.z, info, pauseBuffer);
    latch(ThresholdDir[3], info.pitch, info, pauseBuffer);
    latch(ThresholdDir[4], info.roll, info, pauseBuffer);
    latch(ThresholdDir[5], info.yaw, info, pauseBuffer);
    

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


  //store avg info in espnow message
  info.rawx = avg_acx;
  info.rawy = avg_acy;
  info.rawz = avg_acz;

  // Print data
  
  Serial.print("X: ");
  Serial.print(info.rawx);
  Serial.print("   Y: ");
  Serial.print(info.rawy);
  Serial.print("   Z: ");
  Serial.print(info.rawz);
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

  
  Serial.print("Ring Pin Value: ");
  Serial.print(ring);
  Serial.print("   Middle Pin Value: ");
  Serial.print(middle);
  Serial.print("   Pointer Pin Value: ");
  Serial.println(pointer);
  

  // Send data using ESP-NOW
  esp_err_t result = esp_now_send(0, (uint8_t *)&info, sizeof(sensor_values));
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  } else {
    Serial.println("Error sending the data");
  }

}