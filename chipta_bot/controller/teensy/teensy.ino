#include <micro_ros_arduino.h>
#include "esp_task_wdt.h"  // watch dog timer library
#include <stdio.h>
#include <rcl/rcl.h>             // core ros client library nodes , publisher , subscriber
#include <rcl/error_handling.h>  // error handling from the rcl layer
#include <rclc/rclc.h>           //  The RCL Convenience layer. It simplifies the use of rcl for C developers by providing easier-to-use functions for common tasks.
#include <rclc/executor.h>       // spinning node, triggering the callback
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/int64.h>

// define is the preprocessive directive , it is the macro
#define RPWM 18
#define LPWM 19
#define RPWM1 14
#define LPWM1 15
#define RPWM2 4
#define LPWM2 3
#define RPWM3 8
#define LPWM3 7

// these are the ros objects like in the oops
rcl_subscription_t subscriber;  // subscriber configured to listen
std_msgs__msg__Int64 msg;       // __ after msg is the naming convention
rclc_executor_t executor;       // manages the node's spinning and callbacks
rcl_allocator_t allocator;      // allocates the memory for the callbacks memory for the nodes and publishers
rclc_support_t support;         // support structure that holds context information for the ros2 client library
rcl_node_t node;                //
rcl_timer_t timer;

bool micro_ros_init_successful;

int16_t br_pwm;
int16_t bl_pwm;
int16_t fr_pwm;
int16_t fl_pwm;
int64_t combined_value;

#define LED_PIN 13

// Macros and the Error Handling
#define RCCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) { error_loop(); } \
  }
#define RCSOFTCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) { error_loop(); } \
  }

void error_loop() {
  while (1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

//twist message cb
// void subscription_callback(const void *msgin) {
//   const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
//   // if velocity in x direction is 0 turn off LED, if 1 turn on LED
//   digitalWrite(LED_PIN, (msg->linear.x == 0) ? LOW : HIGH);
// }

void setMotorSpeed(int rpwm, int lpwm, int pwm) {

  if (pwm > 0) {
    analogWrite(rpwm, pwm);
    analogWrite(lpwm, 0);
  } else if (pwm < 0) {
    analogWrite(rpwm, 0);
    analogWrite(lpwm, abs(pwm));
  } else {
    analogWrite(rpwm, 0);
    analogWrite(lpwm, 0);
  }
}

void pwm_decode(int64_t combined_value) {

  br_pwm = (combined_value % 1000) - 255;
  combined_value /= 1000;
  bl_pwm = (combined_value % 1000) - 255;
  combined_value /= 1000;
  fr_pwm = (combined_value % 1000) - 255;
  combined_value /= 1000;
  fl_pwm = (combined_value % 1000) - 255;

  Serial.println(br_pwm);
  Serial.println(bl_pwm);
  Serial.println(fr_pwm);
  Serial.println(fl_pwm);

  setMotorSpeed(RPWM, LPWM, fl_pwm);
  setMotorSpeed(RPWM1, LPWM1, fr_pwm);
  setMotorSpeed(RPWM2, LPWM2, bl_pwm);
  setMotorSpeed(RPWM3, LPWM3, br_pwm);
}


void subscription_callback(const void *msgin) {                           // the msg i spassed as the generic void pointer this allows the executor to use one function signature for all different msg type
  const std_msgs__msg__Int64 *msg = (const std_msgs__msg__Int64 *)msgin;  // casting

  combined_value = msg->data;

  pwm_decode(combined_value);
}

void setup() {

  Serial.begin(115200);

  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);
  pinMode(RPWM1, OUTPUT);
  pinMode(LPWM1, OUTPUT);
  pinMode(RPWM2, OUTPUT);
  pinMode(LPWM2, OUTPUT);
  pinMode(RPWM3, OUTPUT);
  pinMode(LPWM3, OUTPUT);

  Serial.println("aagya idhar pench");
  set_microros_wifi_transports("OPPO", "sudodeva", "10.149.190.233", 8888);  // ip of the micro ros agent
  Serial.print("wifi chalagya pencho\n\n");
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  // delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options  Initializes the ROS client library support structure. This is the first step.
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int64),
    "pwm"));


  // create executor and  configure it
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));

  digitalWrite(LED_PIN, LOW);
}

void loop() {
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}