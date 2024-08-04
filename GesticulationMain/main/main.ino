// Define all the included libraries
// ==========================================================
#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
// ==========================================================

// Define classes
// ==========================================================
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// ==========================================================

// Define global variables
// ==========================================================
#define YAWSERVOMIN  130
#define YAWSERVOMAX  475
#define PITCHSERVOMIN 70
#define PITCHSERVOMAX 500
#define SHOULDERSERVOMIN 250
#define SHOULDERSERVOMAX 500
#define SERVOCOUNTERCW 205  // PWM length to go continuous counter clockwise
#define SERVOCW  410 // PWM length to go continuous clockwise
#define SERVOSTOP  600  // PWM length to stop continuous movement
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates
uint8_t servoNum = 0;
bool cartMode = false;
bool armMode = false;
// ==========================================================

// Define structure to hold joint state values
// ==========================================================
typedef struct joint_state_values {
  int fingers;
  int rawx;
  int rawy;
  int rawz;
  float joint1;
  float joint2;
  float joint3;
  float joint4;
  float joint5;
  float joint6; 
} joint_state_values;
joint_state_values jointinfo;
// ==========================================================

// ==========================================================
// Public methods to control servo motors
void moveForward() {
  Serial.println("Moving Forward");
  pwm.setPWM(0, 1, SERVOCW);
  pwm.setPWM(1, 0, SERVOCOUNTERCW);
}

void turnRight() {
  Serial.println("Turning Right");
  pwm.setPWM(0, 0, SERVOCW);
  pwm.setPWM(1, 0, SERVOCW);
}

void turnLeft() {
  Serial.println("Turning Left");
  pwm.setPWM(0, 0, SERVOCOUNTERCW);
  pwm.setPWM(1, 0, SERVOCOUNTERCW);
}

void moveStop() {
  Serial.println("Stopping Movement");
  pwm.setPWM(0, 0, SERVOSTOP);
  pwm.setPWM(1, 0, SERVOSTOP);
}

int mapYAWAngleToPWM(int degrees) {
  Serial.print("mapping YAW Servo: ");
  int pulselen = map(degrees, 0, 270, YAWSERVOMIN, YAWSERVOMAX);
  Serial.print(degrees);
  Serial.print(" --> ");
  Serial.print(pulselen);
  Serial.println();
  return pulselen;
}

int mapPITCHAngleToPWM(int degrees) {
  Serial.println("Mapping PITCH Servo: ");
  int pulselen = map(degrees, 0, 270, PITCHSERVOMIN, PITCHSERVOMAX);
  Serial.print(degrees);
  Serial.print(" --> ");
  Serial.print(pulselen);
  Serial.println();
  return pulselen;
}

int mapSHOULDERAngleToPWM(int degrees) {
    Serial.println("Mapping SHOULDER Servo: ");
  int pulselen = map(degrees, 0, 270, SHOULDERSERVOMIN, SHOULDERSERVOMAX);
  Serial.print(degrees);
  Serial.print(" --> ");
  Serial.print(pulselen);
  Serial.println();
  return pulselen;
}
// ==========================================================

// Main function of the script, upon interrupt receives data and computes arm/cart function
void OnDataRecv(const esp_now_recv_info *info, const uint8_t *incomingData, int len) {
  // Copy the received data into the joint_state_values structure
  memcpy(&jointinfo, incomingData, len);

  // Print the received joint values
  Serial.println("Received joint state values:");
    Serial.print("Joint 1");
    Serial.print(": ");
    Serial.println(jointinfo.joint1);

    Serial.print("Joint 2");
    Serial.print(": ");
    Serial.println(jointinfo.joint2);

    Serial.print("Joint 3");
    Serial.print(": ");
    Serial.println(jointinfo.joint3);

    Serial.print("Joint 4");
    Serial.print(": ");
    Serial.println(jointinfo.joint4);

    Serial.print("Joint 5");
    Serial.print(": ");
    Serial.println(jointinfo.joint5);

    Serial.print("Joint 6");
    Serial.print(": ");
    Serial.println(jointinfo.joint6);

   Serial.print("Control Finger: Finger ");
   Serial.print(jointinfo.fingers);
   Serial.println();
   Serial.println("Raw Accel Positions: ");
   Serial.print("X: ");
   Serial.print(jointinfo.rawx);
   Serial.println();
   Serial.print("Y: ");
   Serial.print(jointinfo.rawy);
   Serial.println();
   Serial.print("Z: ");
   Serial.print(jointinfo.rawz);
   Serial.println();
   Serial.println("Completed Info Read");

  if (jointinfo.fingers == 1) {
    cartMode = true;
    armMode = false;
  } else if (jointinfo.fingers == 2) {
    cartMode = false;
    armMode = true;
  } else {
    Serial.println("Unrecognized/No finger input, defaulting to: NONE");
    cartMode = false;
    armMode = false;
  }

  if (cartMode) {
    Serial.println("Cart Mode!");
    // Do something here with cart mode
    // use gravity with gyro and whichever axis corresponds to a direction

  } else if (armMode) {
    Serial.println("Arm Mode!");
    
    Serial.println("Moving Shoulder.");
    // Shoulder YAW
    mapYAWAngleToPWM(jointinfo.joint1);
    // Shoulder PITCH
    mapSHOULDERAngleToPWM(jointinfo.joint2);
    delay(333);

    Serial.println("Moving Elbow.");
    // Elbow YAW
    mapYAWAngleToPWM(jointinfo.joint3);
    // Elbow PITCH
    mapPITCHAngleToPWM(jointinfo.joint4);
    delay(333);

    Serial.println("Moving Wrist.");
    // Wrist YAW
    mapYAWAngleToPWM(jointinfo.joint5);
    // Wrist PITCH
    mapPITCHAngleToPWM(jointinfo.joint6);

    Serial.println("Movement Complete!");
  }
}

void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);

  // Initialize pwm for servo motors
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
  delay(200);

  // Set device as a Wi-Fi station
  WiFi.mode(WIFI_STA);
  delay(100);

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register the receive callback function (runs recursively so "loop()" is not needed)
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
}

void loop() {
  // Nothing to do here, as everything is handled in the OnDataRecv callback
} 