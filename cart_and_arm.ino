#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

// Define a structure to hold the joint state values
typedef struct joint_state_values {
  float positions[6]; // Adjust the size based on the number of joints
} joint_state_values;

joint_state_values jointinfo;

void OnDataRecv(const esp_now_recv_info *info, const uint8_t *incomingData, int len) {
  // Copy the received data into the joint_state_values structure
  memcpy(&jointinfo, incomingData, sizeof(joint_state_values));

  // Print the received joint values
  Serial.println("Received joint state values:");
  for (int i = 0; i < 6; i++) {
    Serial.print("Joint ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(jointinfo.positions[i]);
  }
}

void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);

  // Set device as a Wi-Fi station
  WiFi.mode(WIFI_STA);
  //WiFi.disconnect();
  delay(100);

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register the receive callback function
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
}

void loop() {
  // Nothing to do here, as everything is handled in the OnDataRecv callback
  //Serial.println("test");
}
