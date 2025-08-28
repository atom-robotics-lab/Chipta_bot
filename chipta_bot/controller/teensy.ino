#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/int64.h>

#define RPWM 18
#define LPWM 19
#define RPWM1 14
#define LPWM1 15
#define RPWM2 4
#define LPWM2 3
#define RPWM3 8  
#define LPWM3 7

rcl_subscription_t subscriber;
std_msgs__msg__Int64 msg;   
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_timer_t timer;

bool micro_ros_init_successful;


#define LED_PIN 13
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}

void error_loop(){
  while(1){
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
  } 
  else if (pwm < 0) {
    analogWrite(rpwm, 0);     
    analogWrite(lpwm, abs(pwm));
  } 
  else {
    analogWrite(rpwm, LOW);     
    analogWrite(lpwm, LOW);     
  }
}

void subscription_callback(const void * msgin) {
  const std_msgs__msg__Int64 * msg = (const std_msgs__msg__Int64 *)msgin;
  
  int64_t combined_value = msg->data;     

  int16_t br_pwm = (combined_value % 1000)-255;           
  combined_value /= 1000;
  int16_t bl_pwm = (combined_value % 1000)-255;           
  combined_value /= 1000;
  int16_t fr_pwm = (combined_value % 1000)-255;           
  combined_value /= 1000;
  int16_t fl_pwm = (combined_value % 1000)-255;

  
  setMotorSpeed(RPWM, LPWM, fl_pwm);
  setMotorSpeed(RPWM1, LPWM1, fr_pwm);
  setMotorSpeed(RPWM2, LPWM2, bl_pwm);
  setMotorSpeed(RPWM3, LPWM3, br_pwm);
}

void setup() {

  Serial.begin(115200);

  pinMode(RPWM,OUTPUT);
  pinMode(LPWM,OUTPUT);
  pinMode(RPWM1,OUTPUT);
  pinMode(LPWM1,OUTPUT);
  pinMode(RPWM2,OUTPUT);
  pinMode(LPWM2,OUTPUT);
  pinMode(RPWM3,OUTPUT);
  pinMode(LPWM3,OUTPUT);

  set_microros_wifi_transports("A.T.O.M robotics", "atom281121", "192.168.0.104", 8888);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  
  
  delay(2000);

  allocator = rcl_get_default_allocator();

   //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int64),
    "pwm"));
  

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));

  digitalWrite(LED_PIN, LOW);
}

void loop() {
  delay(100);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
