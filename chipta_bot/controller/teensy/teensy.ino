#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/int64.h>

#define RPWM1 23   // fr    
#define LPWM1 22     
#define RPWM2 2     //fl
#define LPWM2 4      
#define DIR3  19    // rr
#define PWM3 21
#define DIR4  5    // rl
#define PWM4 13

rcl_subscription_t subscriber;
std_msgs__msg__Int64 msg;   
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_timer_t timer;

bool micro_ros_init_successful;

int16_t rr_pwm;
int16_t rl_pwm;
int16_t fr_pwm;
int16_t fl_pwm;
int64_t combined_value;

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

void bts_driver(int rpwm, int lpwm, int pwm) {

  if (pwm > 0) {
    analogWrite(rpwm, pwm);   
    analogWrite(lpwm, 0);     
  } 
  else if (pwm < 0) {
    analogWrite(rpwm, 0);     
    analogWrite(lpwm, abs(pwm));
  } 
  else {
    analogWrite(rpwm, 0);     
    analogWrite(lpwm, 0);     
  }
}

void smartElex_driver(int dir, int pwm_pin, int value) {

  if (value > 0) {
    digitalWrite(dir, LOW);
    analogWrite(pwm_pin, value);   
  } 
  else if (value < 0) {
    digitalWrite(dir, HIGH);
    analogWrite(pwm_pin, abs(value));   
  } 
  else {
    analogWrite(pwm_pin, 0); 
  }
}

void pwm_decode(int64_t combined_value) {
  
  rr_pwm = (combined_value % 1000)-255;           
  combined_value /= 1000;
  rl_pwm = (combined_value % 1000)-255;           
  combined_value /= 1000;
  fr_pwm = (combined_value % 1000)-255;           
  combined_value /= 1000;
  fl_pwm = (combined_value % 1000)-255;

  Serial.println(rr_pwm);
  Serial.println(rl_pwm);
  Serial.println(fr_pwm);
  Serial.println(fl_pwm);  

  bts_driver(RPWM1, LPWM1, fr_pwm);    // fr
  bts_driver(RPWM2, LPWM2, fl_pwm);   // fl
  bts_driver(DIR3, PWM3, rr_pwm);    // rr
  smartElex_driver(DIR4, PWM4, rl_pwm);    // rl
}


void subscription_callback(const void * msgin) {
  const std_msgs__msg__Int64 * msg = (const std_msgs__msg__Int64 *)msgin;
  
  combined_value = msg->data;   

  Serial.println(combined_value);
  pwm_decode(combined_value);

}

void setup() {

  Serial.begin(9600);

  pinMode(RPWM1,OUTPUT);
  pinMode(LPWM1,OUTPUT);
  pinMode(RPWM2,OUTPUT);
  pinMode(LPWM2,OUTPUT);
  pinMode(DIR3,OUTPUT);
  pinMode(PWM3,OUTPUT);
  pinMode(DIR4,OUTPUT);
  pinMode(PWM4,OUTPUT);
  digitalWrite(DIR3, LOW);
  analogWrite(PWM3, 0); 
  digitalWrite(DIR4, LOW);
  analogWrite(PWM4, 0); 

  Serial.println("aagya idhar pench");
  set_microros_transports();
  // set_microros_serial_transports();

  // set_microros_wifi_transports("Laxmipg", "atom281121", "192.168.100.23", 8888);
  Serial.print("wifi chalgya pencho\n\n");
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  
  
  // delay(2000);I

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
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}