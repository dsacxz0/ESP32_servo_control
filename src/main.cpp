#include <Arduino.h>
#include <WiFi.h>
#include <micro_ros_utils.hpp>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rosidl_runtime_c/string_functions.h>
#include <trajectory_msgs/msg/joint_trajectory.h>
#include <trajectory_msgs/msg/joint_trajectory_point.h>

// ===================== WiFi & micro-ROS config =====================
char ssid[] = "";             // set your WiFi SSID
char password[] = "";       // set your WiFi Password
IPAddress agent_ip(192, 168, 215, 102); // micro-ROS agent IP
int agent_port = 8888;

// ===================== Servo bus (ESP32 UART2) =====================
#define RX_PIN 18
#define TX_PIN 19
HardwareSerial BusSerial(2);

// ===================== App config =====================
#define NUM_SERVOS 12

// ===================== LX-16A protocol constants =====================
static const uint8_t SERVO_ID_ALL = 0xFE;

static const uint8_t SERVO_MOVE_TIME_WRITE         = 1;
static const uint8_t SERVO_MOVE_TIME_READ          = 2;
static const uint8_t SERVO_MOVE_TIME_WAIT_WRITE    = 7;
static const uint8_t SERVO_MOVE_TIME_WAIT_READ     = 8;
static const uint8_t SERVO_MOVE_START              = 11;
static const uint8_t SERVO_MOVE_STOP               = 12;
static const uint8_t SERVO_ID_WRITE                = 13;
static const uint8_t SERVO_ID_READ                 = 14;
static const uint8_t SERVO_ANGLE_OFFSET_ADJUST     = 17;
static const uint8_t SERVO_ANGLE_OFFSET_WRITE      = 18;
static const uint8_t SERVO_ANGLE_OFFSET_READ       = 19;
static const uint8_t SERVO_ANGLE_LIMIT_WRITE       = 20;
static const uint8_t SERVO_ANGLE_LIMIT_READ        = 21;
static const uint8_t SERVO_VIN_LIMIT_WRITE         = 22;
static const uint8_t SERVO_VIN_LIMIT_READ          = 23;
static const uint8_t SERVO_TEMP_MAX_LIMIT_WRITE    = 24;
static const uint8_t SERVO_TEMP_MAX_LIMIT_READ     = 25;
static const uint8_t SERVO_TEMP_READ               = 26;
static const uint8_t SERVO_VIN_READ                = 27;
static const uint8_t SERVO_POS_READ                = 28;
static const uint8_t SERVO_OR_MOTOR_MODE_WRITE     = 29;
static const uint8_t SERVO_OR_MOTOR_MODE_READ      = 30;
static const uint8_t SERVO_LOAD_OR_UNLOAD_WRITE    = 31;
static const uint8_t SERVO_LOAD_OR_UNLOAD_READ     = 32;
static const uint8_t SERVO_LED_CTRL_WRITE          = 33;
static const uint8_t SERVO_LED_CTRL_READ           = 34;
static const uint8_t SERVO_LED_ERROR_WRITE         = 35;
static const uint8_t SERVO_LED_ERROR_READ          = 36;

static const uint8_t SERVO_ERROR_OVER_TEMPERATURE  = 1;
static const uint8_t SERVO_ERROR_OVER_VOLTAGE      = 2;
static const uint8_t SERVO_ERROR_LOCKED_ROTOR      = 4;

// ===================== Helpers =====================
static inline uint8_t lb(uint16_t v){ return (uint8_t)(v & 0xFF); }
static inline uint8_t hb(uint16_t v){ return (uint8_t)((v >> 8) & 0xFF); }
static inline uint16_t makeWord16(uint8_t lo, uint8_t hi){ return (uint16_t)lo + (((uint16_t)hi) << 8); }

template<typename T>
static inline T clampT(T lo, T hi, T v){ return v < lo ? lo : (v > hi ? hi : v); }

// ===================== LX16A ServoController (Arduino/ESP32) =====================
class ServoController {
public:
  explicit ServoController(Stream& ser, uint32_t defaultTimeoutMs = 1000)
  : ser_(ser), defaultTimeoutMs_(defaultTimeoutMs) {}

  // ---- High-level API ----
  void move(uint8_t id, int position, int timeMs = 0){
    position = clampT(0, 1000, position);
    timeMs   = clampT(0, 30000, timeMs);
    uint8_t p[4] = { lb(position), hb(position), lb(timeMs), hb(timeMs) };
    command_(id, SERVO_MOVE_TIME_WRITE, p, 4);
  }
  void move_prepare(uint8_t id, int position, int timeMs = 0){
    position = clampT(0, 1000, position);
    timeMs   = clampT(0, 30000, timeMs);
    uint8_t p[4] = { lb(position), hb(position), lb(timeMs), hb(timeMs) };
    command_(id, SERVO_MOVE_TIME_WAIT_WRITE, p, 4);
  }
  void move_start(uint8_t id = SERVO_ID_ALL){ command_(id, SERVO_MOVE_START, nullptr, 0); }
  void move_stop(uint8_t id  = SERVO_ID_ALL){ command_(id, SERVO_MOVE_STOP,  nullptr, 0); }

  bool get_prepared_move(uint8_t id, uint16_t& out_pos, uint16_t& out_time, uint32_t toMs=0){
    uint8_t buf[10]; uint8_t n=0; if(!query_(id, SERVO_MOVE_TIME_WAIT_READ, buf,n,toMs)) return false; if(n<6) return false;
    out_pos = makeWord16(buf[2], buf[3]); out_time = makeWord16(buf[4], buf[5]); return true;
  }

  bool get_servo_id(uint8_t& out_id, uint8_t id = SERVO_ID_ALL, uint32_t toMs=0){
    uint8_t b[10]; uint8_t n=0; if(!query_(id, SERVO_ID_READ, b,n,toMs)) return false; if(n<3) return false; out_id=b[2]; return true; }
  void set_servo_id(uint8_t id, uint8_t newId){ command_(id, SERVO_ID_WRITE, &newId, 1); }

  bool get_position(uint8_t id, int16_t& out_pos, uint32_t toMs=0){
    uint8_t b[10]; uint8_t n=0; if(!query_(id, SERVO_POS_READ, b,n,toMs)) return false; if(n<4) return false; int32_t p=makeWord16(b[2],b[3]); if(p>32767) p-=65536; out_pos=(int16_t)p; return true; }

  bool get_voltage(uint8_t id, uint16_t& out_mV, uint32_t toMs=0){
    uint8_t b[10]; uint8_t n=0; if(!query_(id, SERVO_VIN_READ, b,n,toMs)) return false; if(n<4) return false; out_mV=makeWord16(b[2],b[3]); return true; }
  bool get_temperature(uint8_t id, uint8_t& out_C, uint32_t toMs=0){
    uint8_t b[10]; uint8_t n=0; if(!query_(id, SERVO_TEMP_READ, b,n,toMs)) return false; if(n<3) return false; out_C=b[2]; return true; }

  bool get_position_limits(uint8_t id, uint16_t& out_min, uint16_t& out_max, uint32_t toMs=0){
    uint8_t b[10]; uint8_t n=0; if(!query_(id, SERVO_ANGLE_LIMIT_READ, b,n,toMs)) return false; if(n<6) return false; out_min=makeWord16(b[2],b[3]); out_max=makeWord16(b[4],b[5]); return true; }
  void set_position_limits(uint8_t id, int minp, int maxp){
    minp=clampT(0,1000,minp); maxp=clampT(0,1000,maxp);
    uint8_t p[4]={lb(minp),hb(minp),lb(maxp),hb(maxp)}; command_(id,SERVO_ANGLE_LIMIT_WRITE,p,4);
  }

  bool get_voltage_limits(uint8_t id, uint16_t& out_minMv, uint16_t& out_maxMv, uint32_t toMs=0){
    uint8_t b[10]; uint8_t n=0; if(!query_(id, SERVO_VIN_LIMIT_READ, b,n,toMs)) return false; if(n<6) return false; out_minMv=makeWord16(b[2],b[3]); out_maxMv=makeWord16(b[4],b[5]); return true; }
  void set_voltage_limits(uint8_t id, int minMv, int maxMv){
    minMv=clampT(4500,12000,minMv); maxMv=clampT(4500,12000,maxMv);
    uint8_t p[4]={lb(minMv),hb(minMv),lb(maxMv),hb(maxMv)}; command_(id,SERVO_VIN_LIMIT_WRITE,p,4);
  }

  bool get_max_temperature_limit(uint8_t id, uint8_t& out_C, uint32_t toMs=0){
    uint8_t b[10]; uint8_t n=0; if(!query_(id, SERVO_TEMP_MAX_LIMIT_READ, b,n,toMs)) return false; if(n<3) return false; out_C=b[2]; return true; }
  void set_max_temperature_limit(uint8_t id, int C){ C=clampT(50,100,C); uint8_t p=(uint8_t)C; command_(id,SERVO_TEMP_MAX_LIMIT_WRITE,&p,1); }

  bool get_mode(uint8_t id, uint8_t& out_mode, uint32_t toMs=0){
    uint8_t b[10]; uint8_t n=0; if(!query_(id, SERVO_OR_MOTOR_MODE_READ, b,n,toMs)) return false; if(n<3) return false; out_mode=b[2]; return true; }
  bool get_motor_speed(uint8_t id, int16_t& out_speed, uint32_t toMs=0){
    uint8_t b[10]; uint8_t n=0; if(!query_(id, SERVO_OR_MOTOR_MODE_READ, b,n,toMs)) return false; if(n<6) return false; if(b[2]!=1){ out_speed=0; return true; } int32_t s=makeWord16(b[4],b[5]); if(s>32767) s-=65536; out_speed=(int16_t)s; return true; }
  void set_servo_mode(uint8_t id){ uint8_t p[4]={0,0,0,0}; command_(id,SERVO_OR_MOTOR_MODE_WRITE,p,4); }
  void set_motor_mode(uint8_t id, int speed=0){ speed=clampT(-1000,1000,speed); uint16_t u=(speed<0)?(uint16_t)(speed+65536):(uint16_t)speed; uint8_t p[4]={1,0,lb(u),hb(u)}; command_(id,SERVO_OR_MOTOR_MODE_WRITE,p,4); }

  bool is_motor_on(uint8_t id, bool& out_on, uint32_t toMs=0){ uint8_t b[10]; uint8_t n=0; if(!query_(id,SERVO_LOAD_OR_UNLOAD_READ,b,n,toMs)) return false; if(n<3) return false; out_on=(b[2]==1); return true; }
  void motor_on(uint8_t id){ uint8_t v=1; command_(id,SERVO_LOAD_OR_UNLOAD_WRITE,&v,1); }
  void motor_off(uint8_t id){ uint8_t v=0; command_(id,SERVO_LOAD_OR_UNLOAD_WRITE,&v,1); }

  bool is_led_on(uint8_t id, bool& out_on, uint32_t toMs=0){ uint8_t b[10]; uint8_t n=0; if(!query_(id,SERVO_LED_CTRL_READ,b,n,toMs)) return false; if(n<3) return false; out_on=(b[2]==0); return true; }
  void led_on(uint8_t id){ uint8_t v=0; command_(id,SERVO_LED_CTRL_WRITE,&v,1); }
  void led_off(uint8_t id){ uint8_t v=1; command_(id,SERVO_LED_CTRL_WRITE,&v,1); }

  bool get_led_errors(uint8_t id, uint8_t& out_bits, uint32_t toMs=0){ uint8_t b[10]; uint8_t n=0; if(!query_(id,SERVO_LED_ERROR_READ,b,n,toMs)) return false; if(n<3) return false; out_bits=b[2]; return true; }
  void set_led_errors(uint8_t id, uint8_t bits){ bits=clampT<uint8_t>(0,7,bits); command_(id,SERVO_LED_ERROR_WRITE,&bits,1); }


private:
  Stream& ser_;
  uint32_t defaultTimeoutMs_;

  static uint8_t checksum_(uint8_t id, uint8_t len, uint8_t cmd, const uint8_t* p, uint8_t n){
    uint16_t sum = id + len + cmd;
    for(uint8_t i=0;i<n;++i) sum += p[i];
    return (uint8_t)(0xFF - (sum & 0xFF));
  }

  void command_(uint8_t id, uint8_t cmd, const uint8_t* p=nullptr, uint8_t n=0){
    uint8_t len = 3 + n;
    uint8_t chk = checksum_(id,len,cmd,p,n);
    ser_.write((uint8_t)0x55); ser_.write((uint8_t)0x55);
    ser_.write(id); ser_.write(len); ser_.write(cmd);
    if(n && p) ser_.write(p, n);
    ser_.write(chk);
  }

  bool readByteUntil_(uint8_t& out, uint32_t deadline){
    while (millis() < deadline){
      if (ser_.available() > 0){ int v = ser_.read(); if (v >= 0){ out = (uint8_t)v; return true; } }
      delayMicroseconds(200);
    }
    return false;
  }

  bool readN_(uint8_t* buf, uint8_t n, uint32_t deadline){
    for(uint8_t i=0;i<n;++i){ if(!readByteUntil_(buf[i], deadline)) return false; }
    return true;
  }

  bool wait_response_(uint8_t expectId, uint8_t expectCmd, uint8_t* out, uint8_t& outLen, uint32_t toMs){
    const uint32_t deadline = millis() + (toMs ? toMs : defaultTimeoutMs_);
    while (millis() < deadline){
      uint8_t b=0; if(!readByteUntil_(b, deadline)) return false; if (b!=0x55) continue;
      if(!readByteUntil_(b, deadline)) return false; if (b!=0x55) continue;
      uint8_t head[3]; if(!readN_(head,3,deadline)) return false; // id,len,cmd
      uint8_t sid=head[0], len=head[1], cmd=head[2];
      if (len > 7){ // skip invalid frame (plus checksum)
        uint8_t dump = (len>3)? (uint8_t)(len-3) : 0; uint8_t tmp;
        for(uint8_t i=0;i<dump+1;++i){ if(!readByteUntil_(tmp, deadline)) break; }
        continue;
      }
      uint8_t nParams = (len>3)? (uint8_t)(len-3) : 0;
      uint8_t params[8];
      if (nParams>0 && !readN_(params, nParams, deadline)) return false;
      uint8_t chk=0; if(!readByteUntil_(chk, deadline)) return false;
      if (checksum_(sid,len,cmd,params,nParams) != chk) continue;
      if (cmd != expectCmd) continue;
      if (expectId != SERVO_ID_ALL && sid != expectId) continue;
      out[0]=sid; out[1]=cmd; for(uint8_t i=0;i<nParams;++i) out[2+i]=params[i]; outLen=(uint8_t)(2+nParams);
      return true;
    }
    return false;
  }

  bool query_(uint8_t id, uint8_t cmd, uint8_t* out, uint8_t& outLen, uint32_t toMs){
    command_(id, cmd, nullptr, 0);
    return wait_response_(id, cmd, out, outLen, toMs);
  }
};

// Global controller instance
ServoController servoCtl(BusSerial, 1000);

// ===================== WiFi helper =====================
void connectToWiFi(){
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED){ delay(1000); Serial.print("."); }
  Serial.println();
  Serial.print("Connected, IP: "); Serial.println(WiFi.localIP());
}

// ===================== micro-ROS globals =====================
const char* servo_angle_topic = "/servo_angle";
const char* motor_speed_topic = "/motor_speed";
rcl_subscription_t         servo_angle_sub;
rcl_subscription_t         motor_speed_sub;
rclc_executor_t            executor;
rcl_node_t                 node;
rclc_support_t             support;
rcl_allocator_t            allocator;
trajectory_msgs__msg__JointTrajectory servo_angle_msg;
trajectory_msgs__msg__JointTrajectory motor_speed_msg;


// ===================== Speed trajectory callback =====================
void motor_speed_callback(const void * msgin){
  const auto *t = (const trajectory_msgs__msg__JointTrajectory*)msgin;
  if (t->points.size == 0) return;
  const auto &pt = t->points.data[0];
  size_t n = pt.velocities.size;
  if (n == 0) return;
  if (n > NUM_SERVOS) n = NUM_SERVOS;
  for (size_t i = 0; i < n; ++i){
    int spd = (int)lrint(pt.velocities.data[i]);
    if (spd < -1000) spd = -1000; else if (spd > 1000) spd = 1000;
    servoCtl.set_motor_mode((uint8_t)(i+1), spd);
    delay(2);
  }
}

// ===================== Trajectory callback =====================
static inline uint16_t degToTicks(float deg){
  if (deg < 0.0f) deg = 0.0f; if (deg > 240.0f) deg = 240.0f;
  return (uint16_t)(deg / 240.0f * 1000.0f);
}

void servo_angle_callback(const void * msgin){
  Serial.println("=== Trajectory Callback ===");
  auto *t = (const trajectory_msgs__msg__JointTrajectory *)msgin;
  if (t->points.size == 0){ Serial.println("No points"); return; }

  const auto &pt = t->points.data[0];
  // Optional: use time_from_start if provided (per-point total duration)
  uint32_t timeMs = 100; // default
  if (pt.time_from_start.sec > 0 || pt.time_from_start.nanosec > 0){
    uint32_t ms = (uint32_t)pt.time_from_start.sec * 1000u + (pt.time_from_start.nanosec / 1000000u);
    if (ms == 0) ms = 1; // avoid zero-time
    timeMs = clampT<uint32_t>(0, 30000, ms);
  }

  size_t n = pt.positions.size < NUM_SERVOS ? pt.positions.size : NUM_SERVOS;
  for (size_t i = 0; i < n; ++i){
    float angle_deg = (float)pt.positions.data[i];
    uint16_t ticks = degToTicks(angle_deg);
    Serial.printf("Servo %u -> %.2f deg (%u) in %ums\n", (unsigned)(i+1), angle_deg, ticks, (unsigned)timeMs);
    servoCtl.set_servo_mode((uint8_t)(i+1));
    servoCtl.move((uint8_t)(i+1), ticks, (int)timeMs);
    delay(5);
  }
  Serial.println("=== Trajectory Done ===");
}

// ===================== Setup =====================
void setup(){
  Serial.begin(115200);
  delay(500);

  connectToWiFi();

  BusSerial.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
  delay(50);
  // Enable torque on all servos
  for (uint8_t id = 1; id <= NUM_SERVOS; ++id){ servoCtl.motor_on(id); delay(10); }

  Serial.printf("Setting up micro-ROS WiFi transport to %s:%d\n", agent_ip.toString().c_str(), agent_port);
  if (strlen(agent_ip.toString().c_str()) == 0){
    Serial.println("ERROR: agent_ip is empty!");
    while(1) delay(1000);
  }
  set_microros_wifi_transports(ssid, password, agent_ip, agent_port);
  delay(1000);

  allocator = rcl_get_default_allocator();

  Serial.println("Connecting to micro-ROS agent...");
  rcl_ret_t ret; int retry=0; const int max_retries=10;
  do{
    ret = rclc_support_init(&support, 0, NULL, &allocator);
    if (ret != RCL_RET_OK){ ++retry; Serial.printf("Support init failed (%d/%d)\n", retry, max_retries); delay(1500); }
  } while (ret != RCL_RET_OK && retry < max_retries);

  if (ret != RCL_RET_OK){
    Serial.println("Failed to init support; retrying forever...");
    while (ret != RCL_RET_OK){ delay(3000); ret = rclc_support_init(&support, 0, NULL, &allocator); }
  }

  ret = rclc_node_init_default(&node, "servo_node", "", &support);
  if (ret != RCL_RET_OK){ Serial.println("Node init failed"); while(1) delay(1000); }

  trajectory_msgs__msg__JointTrajectory__init(&servo_angle_msg);
  rosidl_runtime_c__String__Sequence__init(&servo_angle_msg.joint_names, NUM_SERVOS);
  char tmp[16];
  for (uint8_t i=0;i<NUM_SERVOS;++i){ snprintf(tmp, sizeof(tmp), "servo_%u", (unsigned)(i+1)); rosidl_runtime_c__String__assign(&servo_angle_msg.joint_names.data[i], tmp); }

  trajectory_msgs__msg__JointTrajectoryPoint__Sequence__init(&servo_angle_msg.points, 1);
  servo_angle_msg.points.data[0].positions.data = (double*)malloc(NUM_SERVOS * sizeof(double));
  servo_angle_msg.points.data[0].positions.size = servo_angle_msg.points.data[0].positions.capacity = NUM_SERVOS;
  servo_angle_msg.points.data[0].time_from_start.sec = 0; servo_angle_msg.points.data[0].time_from_start.nanosec = 0;

  ret = rclc_subscription_init_default(
    &servo_angle_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(trajectory_msgs, msg, JointTrajectory),
    servo_angle_topic
  );
  if (ret != RCL_RET_OK){ Serial.println("servo_angle_sub init failed"); while(1) delay(1000); }

  trajectory_msgs__msg__JointTrajectory__init(&motor_speed_msg);
  rosidl_runtime_c__String__Sequence__init(&motor_speed_msg.joint_names, NUM_SERVOS);
  for (uint8_t i=0;i<NUM_SERVOS;++i){ char nm[16]; snprintf(nm,sizeof(nm),"servo_%u", (unsigned)(i+1)); rosidl_runtime_c__String__assign(&motor_speed_msg.joint_names.data[i], nm); }
  trajectory_msgs__msg__JointTrajectoryPoint__Sequence__init(&motor_speed_msg.points, 1);
  motor_speed_msg.points.data[0].velocities.data = (double*)malloc(NUM_SERVOS * sizeof(double));
  motor_speed_msg.points.data[0].velocities.size = motor_speed_msg.points.data[0].velocities.capacity = NUM_SERVOS;
  motor_speed_msg.points.data[0].time_from_start.sec = 0;
  motor_speed_msg.points.data[0].time_from_start.nanosec = 0;
  ret = rclc_subscription_init_default(
    &motor_speed_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(trajectory_msgs, msg, JointTrajectory),
    motor_speed_topic
  );
  if (ret != RCL_RET_OK){ Serial.println("motor_speed_sub init failed"); while(1) delay(1000); }

  rclc_executor_init(&executor, &support.context, 2, &allocator);
  rclc_executor_add_subscription(
    &executor, &servo_angle_sub, &servo_angle_msg,
    servo_angle_callback, ON_NEW_DATA
  );
  rclc_executor_add_subscription(
    &executor, &motor_speed_sub, &motor_speed_msg,
    motor_speed_callback, ON_NEW_DATA
  );

  Serial.println("Setup complete");
}

// ===================== Loop =====================
void loop(){
  if (WiFi.status() != WL_CONNECTED){ Serial.println("WiFi lost; reconnecting..."); connectToWiFi(); }
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));  
}
