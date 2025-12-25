#include <Arduino.h>
#include <WiFi.h>
#include <micro_ros_platformio.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float64.h>
#include <std_msgs/msg/float64_multi_array.h>
#include <navbot_interfaces/msg/velocity.h>
#include <navbot_interfaces/msg/pose.h>
#include <navbot_interfaces/msg/multi_values.h>

#include <MPU9250_WE.h>
#include <Wire.h>

#include "Odometry.h"
#include "EncoderClass.h"
#include "MotorClass.h"
#include "PIDControlClass.h"

#define MPU9250_ADDR 0x68
#define ACCEL_SENSITIVITY 16384.0  
#define G 9.80665
#define DEG_TO_RAD 0.01745

#define WHEEL_RADIUS 0.043
#define WHEEL_BASE 0.175
#define LEFT_WHEEL_TICKS 224
#define RIGHT_WHEEL_TICKS 224

const char* ssid = "ESP32-Access-Point";
const char* password = "ts123456";

WiFiServer server(80);
MPU9250_WE myMPU9250 = MPU9250_WE(MPU9250_ADDR);

// ---------------------- Hardware & PID ----------------------
int left_sp = 0, right_sp = 0;  // Target speed from ROS
int left_rpm, right_rpm;
int controlActionLeft = 0, controlActionRight = 0;

float kp = 0.5;
float ki = 5;
float kd = 0.01;
float h = 0.1; // PID sample period in seconds

// Motor & Encoder pins
uint8_t MotorIN3 = 23, MotorIN4 = 19, MotorENB = 15;
uint8_t MotorIN1 = 33, MotorIN2 = 32, MotorENA = 4;
uint8_t LEFT_ENCODER_A = 18, LEFT_ENCODER_B = 5;
uint8_t RIGHT_ENCODER_A = 16, RIGHT_ENCODER_B = 17;

PIDControlClass PIDControllerLeft(kp, ki, kd, h);
PIDControlClass PIDControllerRight(kp, ki, kd, h);
EncoderClass* EncoderLeft = new EncoderClass(LEFT_ENCODER_A, LEFT_ENCODER_B);
EncoderClass* EncoderRight = new EncoderClass(RIGHT_ENCODER_A, RIGHT_ENCODER_B);
MotorClass MotorLeft(MotorIN1, MotorIN2, MotorENA, LEFT_WHEEL_TICKS);
MotorClass MotorRight(MotorIN3, MotorIN4, MotorENB, RIGHT_WHEEL_TICKS);
Odometry RobotOdom(WHEEL_RADIUS, WHEEL_BASE, LEFT_WHEEL_TICKS, RIGHT_WHEEL_TICKS);

// ---------------------- micro-ROS ----------------------
rcl_subscription_t set_velocity_sub;
navbot_interfaces__msg__Velocity velocity_sub_msg;

rcl_publisher_t get_velocity_pub;
navbot_interfaces__msg__Velocity velocity_pub_msg;

rcl_publisher_t get_position_pub;
navbot_interfaces__msg__MultiValues position_pub_msg;

rcl_publisher_t odom_pub;
navbot_interfaces__msg__Pose odom_pub_msg;

rcl_publisher_t imu_pub;
std_msgs__msg__Float64MultiArray imu_pub_msg;

rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;
rcl_timer_t timer;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){while(1){};}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void publish_imu() 
{
  // xyzFloat accRaw = myMPU9250.getAccRawValues();
  // xyzFloat accCorrRaw = myMPU9250.getCorrectedAccRawValues();
  // xyzFloat gValue = myMPU9250.getGValues();
  // float resultantG = myMPU9250.getResultantG(gValue);
  xyzFloat gValue = myMPU9250.getCorrectedAccRawValues();
  xyzFloat gyr_raw = myMPU9250.getCorrectedGyrRawValues();
  xyzFloat magValue = myMPU9250.getMagValues();
  
  xyzFloat a_mps2;
  a_mps2.x = (gValue.x/ ACCEL_SENSITIVITY) * G;
  a_mps2.y = (gValue.y/ ACCEL_SENSITIVITY) * G;
  a_mps2.z = (gValue.z/ ACCEL_SENSITIVITY) * G;

  xyzFloat gyr;
  gyr.x = gyr_raw.x / 131.0;
  gyr.y = gyr_raw.y / 131.0;
  gyr.z = gyr_raw.z / 131.0;

  Serial.print(a_mps2.x);
  Serial.print("   ");
  Serial.print(a_mps2.y);
  Serial.print("   ");
  Serial.println(a_mps2.z);

  double data[9] = {
    a_mps2.x, a_mps2.y, a_mps2.z,
    gyr.x, gyr.y, gyr.z,
    magValue.x, magValue.y, magValue.z
  };

  imu_pub_msg.data.data = data;
  imu_pub_msg.data.size = 9;
  imu_pub_msg.data.capacity = 9;
  RCSOFTCHECK(rcl_publish(&imu_pub, &imu_pub_msg, NULL));

}

// ---------------------- ROS callback ----------------------
void set_velocity_sub_callback(const void * msgin)
{
    const navbot_interfaces__msg__Velocity * msg = (const navbot_interfaces__msg__Velocity *)msgin;
    left_sp = msg->left_velocity;
    right_sp = msg->right_velocity;
}

void odom_pub_callback(rcl_timer_t * timer, int64_t last_call_time) 
{
    RobotOdom.update(EncoderLeft->getPulseCount(), EncoderRight->getPulseCount());

    odom_pub_msg.x = RobotOdom.getX();
    odom_pub_msg.y = RobotOdom.getY();
    odom_pub_msg.theta = RobotOdom.getTheta();
    RCSOFTCHECK(rcl_publish(&odom_pub, &odom_pub_msg, NULL));

    velocity_pub_msg.left_velocity = MotorLeft.getMotorRPM(EncoderLeft->getPulseCount());
    velocity_pub_msg.right_velocity = MotorRight.getMotorRPM(EncoderRight->getPulseCount());
    RCSOFTCHECK(rcl_publish(&get_velocity_pub, &velocity_pub_msg, NULL));

    position_pub_msg.left_value = MotorLeft.getMotorPosition(EncoderLeft->getPulseCount());
    position_pub_msg.right_value = MotorRight.getMotorPosition(EncoderRight->getPulseCount());
    RCSOFTCHECK(rcl_publish(&get_position_pub, &position_pub_msg, NULL));

    publish_imu();
}

// ---------------------- FreeRTOS PID Task ----------------------
void PIDTask(void * parameter)
{
    const TickType_t xFrequency = pdMS_TO_TICKS(h * 1000); // Convert h seconds to ms
    TickType_t xLastWakeTime = xTaskGetTickCount();

    unsigned long last_time = millis();

    for (;;)
    {
        // Wait for next cycle
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        // -------------------- Read encoders --------------------
        left_rpm = MotorLeft.getMotorRPM(EncoderLeft->getPulseCount());
        right_rpm = MotorRight.getMotorRPM(EncoderRight->getPulseCount());
        Serial.print(EncoderLeft->getPulseCount());
        Serial.print(", ");
        Serial.print(EncoderRight->getPulseCount());

        // -------------------- Compute PID ----------------------
        unsigned long now = millis();
        float dt = (now - last_time) / 1000.0;
        last_time = now;

        // Compute PID with dynamic dt
        controlActionLeft = PIDControllerLeft.ComputeControlAction(left_sp, left_rpm);
        controlActionRight = PIDControllerRight.ComputeControlAction(right_sp, right_rpm);

        if (left_sp == 0) controlActionLeft = 0;
        if (right_sp == 0) controlActionRight = 0;

        // -------------------- Control motors ------------------
        MotorLeft.MotorControl(controlActionLeft);
        MotorRight.MotorControl(controlActionRight);

    }
}

// ---------------------- Setup ----------------------
void setup()
{
    Serial.begin(115200);
    // set_microros_serial_transports(Serial);

    Wire.begin();
    if(!myMPU9250.init()){
        Serial.println("MPU9250 does not respond");
    }
    else{
        Serial.println("MPU9250 is connected");
    }
    if(!myMPU9250.initMagnetometer()){
        Serial.println("Magnetometer does not respond");
    }
    else{
        Serial.println("Magnetometer is connected");
    }

    Serial.println("Position you MPU9250 flat and don't move it - calibrating...");
    delay(1000);
    myMPU9250.autoOffsets();
    Serial.println("Done!");

    myMPU9250.enableGyrDLPF();

    myMPU9250.setGyrDLPF(MPU9250_DLPF_6);

    myMPU9250.setSampleRateDivider(5);

    myMPU9250.setGyrRange(MPU9250_GYRO_RANGE_250);

    myMPU9250.setAccRange(MPU9250_ACC_RANGE_2G);

    myMPU9250.enableAccDLPF(true);

    myMPU9250.setAccDLPF(MPU9250_DLPF_6);

    myMPU9250.setMagOpMode(AK8963_CONT_MODE_100HZ);
    delay(200);

    IPAddress agent_ip(10, 42, 0, 1);
    size_t agent_port = 8888;
    char name[] = "taiso";
    char psk[]= "ts123456";
    set_microros_wifi_transports(name, psk, agent_ip, agent_port);


    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

    MotorLeft.MotorInitialize();
    MotorRight.MotorInitialize();
    delay(500);

    EncoderLeft->initEncoder([]{ EncoderLeft->countPulsesLowResolution(); }, 1);
    EncoderRight->initEncoder([]{ EncoderRight->countPulsesLowResolution(); }, 1);
    delay(500);

    PIDControllerLeft.PIDControllerInitialize(0, 0);
    PIDControllerRight.PIDControllerInitialize(0, 0);

    // ---------------------- micro-ROS ----------------------
    allocator = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "micro_ros_esp32", "", &support));

    RCCHECK(rclc_subscription_init_default(
        &set_velocity_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(navbot_interfaces, msg, Velocity),
        "set_velocity"));

    RCCHECK(rclc_publisher_init_default(
        &get_velocity_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(navbot_interfaces, msg, Velocity),
        "get_velocity"));

    RCCHECK(rclc_publisher_init_default(
        &get_position_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(navbot_interfaces, msg, MultiValues),
        "get_position"));

    RCCHECK(rclc_publisher_init_default(
        &odom_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(navbot_interfaces, msg, Pose),
        "odom"));

    RCCHECK(rclc_publisher_init_default(
        &imu_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
        "imu"));

    const unsigned int timer_timeout = 50;
    RCCHECK(rclc_timer_init_default2(
        &timer,
        &support,
        RCL_MS_TO_NS(timer_timeout),
        odom_pub_callback, true));

    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));
    RCCHECK(rclc_executor_add_subscription(&executor, &set_velocity_sub, &velocity_sub_msg,
                                           &set_velocity_sub_callback, ON_NEW_DATA));

    // ---------------------- Create PID task ----------------------
    xTaskCreatePinnedToCore(
        PIDTask,
        "PID Task",
        4096,
        NULL,
        2,       // High priority
        NULL,
        1        // Run on Core 1 (keep micro-ROS on Core 0)
    );
}

// ---------------------- Main loop ----------------------
void loop()
{
    // Spin executor on Core 0
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)));
    delay(1);  // prevent 100% CPU
}
