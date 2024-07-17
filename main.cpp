#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <esp_now.h>
#include <WiFi.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/pose.h>
#include <sensor_msgs/msg/joint_state.h>

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only available for Arduino framework with serial transport.
#endif

// ROS 2 variables
rcl_publisher_t publisher;
geometry_msgs__msg__Pose pose_msg;

rcl_subscription_t subscriber;
sensor_msgs__msg__JointState joint_state_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
double inc = 0.02;

// ESP-NOW variables
uint8_t receiverMacAddress[] = {0xA8, 0x42, 0xE3, 0x59, 0xA4, 0xE0};
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

accel_values info;

void update_position(double &x, double y){
  if(y==1){
    x-=inc;
  }
  else if (y==2)
  {
    x+=inc;
  }
  
}

void joint_state_callback(const void * msgin) {
  const sensor_msgs__msg__JointState * msg = (const sensor_msgs__msg__JointState *)msgin;
  // Process the received JointState message
  Serial.print("Received JointState message with ");
  Serial.print(msg->name.size);
  Serial.println(" joints.");
}

void OnInput(const uint8_t * info, const uint8_t *incomingData, int length) {
    accel_values data;

    // Copy incoming data into accel_values structure
    memcpy(&data, incomingData, sizeof(accel_values));
//    Serial.print("Xin");
//    Serial.println(data.acx);
    // Update pose_msg with received data
    update_position(pose_msg.position.x, data.x);
    update_position(pose_msg.position.y, data.y);
    update_position(pose_msg.position.z, data.z);
    //pose_msg.orientation.x = 0;//data.gyx;
    //pose_msg.orientation.y = 0;//data.gyy;
    //.orientation.z = 0;//data.gyz;

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
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
    "joint_states"));

  // Create executor with 1 handle (subscriber)
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &joint_state_msg, &joint_state_callback, ON_NEW_DATA));

  // Initialize Pose message data
  pose_msg.position.x = 0.5;
  pose_msg.position.y = 0.0;
  pose_msg.position.z = 0.8;

  //pose_msg.orientation.x = 0.0;
  //pose_msg.orientation.y = 0.0;
  //pose_msg.orientation.z = 0.0;
  //pose_msg.orientation.w = 1.0;  // Typically, a valid quaternion starts with w = 1


  // Initialize esp-now
  if(esp_now_init() != ESP_OK){
  Serial.println("error starting esp now");
  }
  esp_now_register_recv_cb(OnInput);
}

void loop() {
  // Spin the executor to handle pending events
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}
