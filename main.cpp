#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <esp_now.h>
#include <WiFi.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/pose.h>
#include <geometry_msgs/msg/wrench_stamped.h>

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only available for Arduino framework with serial transport.
#endif

// ROS 2 variables
rcl_publisher_t publisher;
geometry_msgs__msg__Pose pose_msg;

rcl_subscription_t subscriber;
geometry_msgs__msg__WrenchStamped wrench_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
double inc = 0.02;

// ESP-NOW variables
uint8_t receiverMacAddress[] = {0x40, 0x91, 0x51, 0x2D, 0x60, 0xC4};
bool espNowDataReceived = false; // Flag to track ESP-NOW message reception

// Error checking macros
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if(temp_rc != RCL_RET_OK){Serial.print("Error in "#fn": "); Serial.println(rcl_get_error_string().str); rcl_reset_error();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if(temp_rc != RCL_RET_OK){Serial.print("Soft error in "#fn": "); Serial.println(rcl_get_error_string().str); rcl_reset_error();}}

typedef struct accel_values {
  float x;
  float y;
  float z;
  float pitch;
  float roll;
  float yaw;
} accel_values;

typedef struct joint_state_values {
  float positions[6]; // Adjust the size based on the number of joints
} joint_state_values;

joint_state_values jointinfo;

accel_values info;

esp_now_peer_info_t peerInfo;

void update_position(double &x, double y){
  if(y==1){
    x -= inc;
  }
  else if (y==2)
  {
    x += inc;
  }
}

void joint_state_callback(const void * msgin) {
  const geometry_msgs__msg__WrenchStamped * msg = (const geometry_msgs__msg__WrenchStamped *)msgin;

  // Process the received message
  Serial.println("Received joint state message.");

  // Extract values from the received WrenchStamped message
  jointinfo.positions[0] = msg->wrench.force.x;
  jointinfo.positions[1] = msg->wrench.force.y;
  jointinfo.positions[2] = msg->wrench.force.z;
  jointinfo.positions[3] = msg->wrench.torque.x;
  jointinfo.positions[4] = msg->wrench.torque.y;
  jointinfo.positions[5] = msg->wrench.torque.z;

  pose_msg.position.y += 0.001;

  // Transmit joint states over ESP-NOW
  esp_err_t result = esp_now_send(receiverMacAddress, (uint8_t *)&jointinfo, sizeof(jointinfo));
  if (result == ESP_OK) {
    Serial.println("Sent with success");
    pose_msg.position.x -= 0.001;
  } else {
    Serial.println("Error sending the data");
    pose_msg.position.z -= 0.001;
  }
}

void OnInput(const uint8_t * info, const uint8_t *incomingData, int length) {
    accel_values data;

    // Copy incoming data into accel_values structure
    memcpy(&data, incomingData, sizeof(accel_values));

    // Update pose_msg with received data
    update_position(pose_msg.position.x, data.x);
    update_position(pose_msg.position.y, data.y);
    update_position(pose_msg.position.z, data.z);

    Serial.println("Data received from ESP-NOW");
    espNowDataReceived = true; // Set flag to indicate data received

    // Publish the updated pose_msg
    RCSOFTCHECK(rcl_publish(&publisher, &pose_msg, NULL));
}

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  WiFi.mode(WIFI_STA);
  delay(2000);

  allocator = rcl_get_default_allocator();

  // Initialize micro-ROS support
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_platformio_node", "", &support));

  // Create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Pose),
    "pose_to_moveit"));

  // Create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, WrenchStamped),
    "joint_positions"));

  // Create executor with 1 handle (subscriber)
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &wrench_msg, &joint_state_callback, ON_NEW_DATA));

  // Initialize Pose message data
  pose_msg.position.x = 0.5;
  pose_msg.position.y = 0.0;
  pose_msg.position.z = 0.8;

  // Initialize esp-now
  if (esp_now_init() != ESP_OK) {
    Serial.println("error starting esp now");
  }
  esp_now_register_recv_cb(OnInput);

  memcpy(peerInfo.peer_addr, receiverMacAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    pose_msg.orientation.x = 0.001;
    return;
  }
}

void loop() {
  // Spin the executor to handle pending events
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}
