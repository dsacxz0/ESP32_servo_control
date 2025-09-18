#include <Arduino.h>
#include <WiFi.h>
#include <micro_ros_utils.hpp>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rosidl_runtime_c/string_functions.h>
#include <trajectory_msgs/msg/joint_trajectory.h>
#include <trajectory_msgs/msg/joint_trajectory_point.h>

char ssid[] = "monakS"; // Please set your WiFi SSID
char password[] = "        "; // Please set your WiFi Password
IPAddress agent_ip = IPAddress(192, 168, 215, 102); // Please set your micro-ROS agent IP address
int agent_port = 8888;

#define NUM_SERVOS 12

#define RX_PIN 18
#define TX_PIN 19
HardwareSerial BusSerial(2);
#define HDR      0x55
#define CMD_MOVE 0x01
#define CMD_LOAD 0x1F

uint8_t calcCHK(const uint8_t *b) {
  uint16_t s = 0;
  for (uint8_t i = 2; i < b[3] + 2; i++) s += b[i];
  return ~s;
}

void sendPack(uint8_t id, uint8_t cmd, const uint8_t *p, uint8_t n) {
  uint8_t buf[6 + n];
  buf[0] = buf[1] = HDR;
  buf[2] = id; 
  buf[3] = n + 3;
  buf[4] = cmd;
  for (uint8_t i = 0; i < n; i++) buf[5 + i] = p[i];
  buf[5 + n] = calcCHK(buf);
  BusSerial.write(buf, 6 + n);
}

void enableTorque(uint8_t id) {
  uint8_t on = 1;
  sendPack(id, CMD_LOAD, &on, 1);
}

void moveServoDeg(uint8_t id, float deg) {
  deg = constrain(deg, 0.0f, 240.0f);
  uint16_t pos = (uint16_t)(deg / 240.0f * 1000.0f);
  uint8_t p[4] = {
    (uint8_t)(pos & 0xFF),
    (uint8_t)(pos >> 8),
    0x64, 0x00 
  };
  sendPack(id, CMD_MOVE, p, 4);
}

void connectToWiFi() {
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  
  Serial.println();
  Serial.print("Connected to WiFi network with IP Address: ");
  Serial.println(WiFi.localIP());
}

rcl_subscription_t         subscription;
rclc_executor_t            executor;
rcl_node_t                 node;
rclc_support_t             support;
rcl_allocator_t            allocator;
trajectory_msgs__msg__JointTrajectory traj_msg;

void traj_callback(const void * msgin) {
  Serial.println("=== Trajectory Callback Triggered ===");
  
  auto *t = (const trajectory_msgs__msg__JointTrajectory *)msgin;
  
  Serial.printf("Joint names count: %u\n", (unsigned)t->joint_names.size);
  for (size_t i = 0; i < t->joint_names.size && i < 5; i++) {
    Serial.printf("Joint[%u]: %s\n", (unsigned)i, t->joint_names.data[i].data);
  }
  
  Serial.printf("Points count: %u\n", (unsigned)t->points.size);
  
  if (t->points.size == 0) {
    Serial.println("WARNING: No trajectory points received!");
    return;
  }
  
  auto &pt = t->points.data[0];
  size_t n = pt.positions.size < NUM_SERVOS ? pt.positions.size : NUM_SERVOS;
  Serial.printf("Point 0 has %u positions (will use %u)\n", (unsigned)pt.positions.size, (unsigned)n);
  
  for (size_t i = 0; i < n; i++) {
    float angle = (float)pt.positions.data[i];
    Serial.printf("Moving servo %u to %.2f degrees\n", (unsigned)(i + 1), angle);
    moveServoDeg(i + 1, angle);
    delay(10);
  }
  
  Serial.println("=== Callback Complete ===");
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  connectToWiFi();
  
  BusSerial.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
  for (uint8_t id = 1; id <= NUM_SERVOS; id++) {
    enableTorque(id);
    delay(20);
  }
  delay(200);
  
  Serial.printf("Setting up micro-ROS WiFi transport to %s:%d\n", agent_ip.toString().c_str(), agent_port);
  
  if (strlen(agent_ip.toString().c_str()) == 0) {
    Serial.println("ERROR: agent_ip is empty! Please set the correct IP address.");
    while(1) delay(1000);
  }
  
  set_microros_wifi_transports(ssid, password, agent_ip, agent_port);
  
  delay(2000);
  
  allocator = rcl_get_default_allocator();
  
  Serial.println("Attempting to connect to micro-ROS agent...");
  rcl_ret_t ret;
  int retry_count = 0;
  const int max_retries = 10;
  
  do {
    ret = rclc_support_init(&support, 0, NULL, &allocator);
    if (ret != RCL_RET_OK) {
      retry_count++;
      Serial.printf("Connection attempt %d/%d failed, retrying in 2 seconds...\n", retry_count, max_retries);
      delay(2000);
    }
  } while (ret != RCL_RET_OK && retry_count < max_retries);
  
  if (ret != RCL_RET_OK) {
    Serial.printf("Failed to initialize micro-ROS support after %d attempts\n", max_retries);
    Serial.println("Check network connectivity and agent status");
    while(1) {
      delay(5000);
      Serial.println("Retrying connection...");
      if (rclc_support_init(&support, 0, NULL, &allocator) == RCL_RET_OK) {
        Serial.println("Connection established!");
        break;
      }
    }
  }
  
  ret = rclc_node_init_default(&node, "servo_node", "", &support);
  if (ret != RCL_RET_OK) {
    Serial.println("Failed to initialize micro-ROS node");
    while(1) {
      delay(1000);
    }
  }
  
  Serial.println("micro-ROS node initialized successfully");
  
  trajectory_msgs__msg__JointTrajectory__init(&traj_msg);
  
  rosidl_runtime_c__String__Sequence__init(&traj_msg.joint_names, NUM_SERVOS);
  char tmp[16];
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    sprintf(tmp, "servo_%u", i + 1);
    rosidl_runtime_c__String__assign(&traj_msg.joint_names.data[i], tmp);
  }
  
  trajectory_msgs__msg__JointTrajectoryPoint__Sequence__init(&traj_msg.points, 1);
  traj_msg.points.data[0].positions.data =
    (double *)malloc(NUM_SERVOS * sizeof(double));
  traj_msg.points.data[0].positions.size =
    traj_msg.points.data[0].positions.capacity = NUM_SERVOS;
  traj_msg.points.data[0].time_from_start.sec = 0;
  traj_msg.points.data[0].time_from_start.nanosec = 0;
  
  ret = rclc_subscription_init_default(
    &subscription, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(trajectory_msgs, msg, JointTrajectory),
    "/servo_trajectory"
  );
  
  if (ret != RCL_RET_OK) {
    Serial.println("Failed to initialize subscription");
    while(1) {
      delay(1000);
    }
  }
  
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(
    &executor, &subscription, &traj_msg,
    traj_callback, ON_NEW_DATA
  );
  
  Serial.println("Setup complete, ready to receive trajectory commands");
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi disconnected, attempting to reconnect...");
    connectToWiFi();
  }
  
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
}