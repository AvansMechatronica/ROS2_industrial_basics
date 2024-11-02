#include <Arduino.h>

#include <EEPROM.h>

#include <micro_ros_platformio.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include "rosidl_runtime_c/string_functions.h"  // Header for string assignment functions


#include <std_msgs/msg/bool.h>
#include <sensor_msgs/msg/range.h>
#include <range_sensors_interfaces/msg/sensor_information.h>

#include <HCSR04.h>


#define WIFI_SSID "VRV9517724283"
#define PASSWORD "@AYCwXhz976C"

//#define WIFI_SSID "BirdsBoven"
//#define PASSWORD "Highway12!"
#define NODE_NAME "sensor_info_publisher"
#define PORT 8888
int ip_address[] = {192, 168, 2, 150};

#define SR04_TRIG_PIN   2
#define SR04_ECHO_PIN   3

rcl_publisher_t sensor_information_publisher;

rclc_executor_t executor;

range_sensors_interfaces__msg__SensorInformation sensor_information;

rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

#define STATUS_LED 3


IPAddress agent_ip(ip_address[0], ip_address[1], ip_address[2], ip_address[3]);


rcl_timer_t timer;
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

int scan_index = 0;

bool errorLedState = false;


#define STATUS_LED_PIN    8

#define MAX_RANGE  1.00
#define MIN_RANGE  0.10

void error_loop(){
  Serial.printf("Ultrasonic Sensor\nError\nSystem halted");
  while(1){
      
        if(errorLedState){
           digitalWrite(STATUS_LED_PIN, HIGH);
           errorLedState = false;
        }
        else{
            digitalWrite(STATUS_LED_PIN, LOW);
            errorLedState = true;
        }
    delay(100);
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {

    struct timespec ts;
    extern int clock_gettime(clockid_t unused, struct timespec *tp);
    clock_gettime(CLOCK_REALTIME, &ts);
    sensor_information.sensor_data.header.stamp.sec = ts.tv_sec;
    sensor_information.sensor_data.header.stamp.nanosec = ts.tv_nsec;

    double* distances = HCSR04.measureDistanceCm();
    //Serial.printf("Distance = %f\n", (float)(distances[0]/100.0));
    sensor_information.sensor_data.range= (float)(distances[0]/100.0);

    RCSOFTCHECK(rcl_publish(&sensor_information_publisher, &sensor_information, NULL));
  }
}


void setup() {

  Serial.begin(115200);
  while (!Serial);
  delay(2000);
  Serial.print("Light-controller started, node: ");
  Serial.println(NODE_NAME);

  HCSR04.begin(SR04_TRIG_PIN, SR04_ECHO_PIN);

  pinMode(STATUS_LED_PIN, OUTPUT); 
  digitalWrite(STATUS_LED_PIN, HIGH);

#ifdef WIFI

  Serial.printf("hostname :%s\n", NODE_NAME);
  WiFi.setHostname(NODE_NAME);

    // Set WiFi to station mode and disconnect from an AP if it was previously connected.
    //WiFi.effect(WIFI_STA);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, PASSWORD);
  //WiFi.setTxPower(WIFI_POWER_5dBm);
  delay(100);
#if defined(ARDUINO_ESP32S3_DEV)
  WiFi.effect(WIFI_STA);
  WiFi.begin(WIFI_SSID, PASSWORD);
  //WiFi.setTxPower(WIFI_POWER_8_5dBm);
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(1000);
  }
  Serial.println();
  Serial.print("Ip adress: ");
  Serial.println(WiFi.localIP());
  Serial.print("MAC adress: ");
  Serial.println(WiFi.macAddress());
#endif

  set_microros_wifi_transports(WIFI_SSID, PASSWORD, agent_ip, (size_t)PORT);
#else
  #if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
  #error This example is only avaliable for Arduino framework with serial transport.
  #endif
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  delay(2000);
#endif

  Serial.printf(" Rangesensor WiFi Connected\n");

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, NODE_NAME, "", &support));

  // setup static values for topic
  rosidl_runtime_c__String__assign(&sensor_information.sensor_data.header.frame_id, "distance_sensor_frame");
  rosidl_runtime_c__String__assign(&sensor_information.maker_name, "Avans");
  sensor_information.part_number = 20241102;

  // Fill in the sensor data information.
  sensor_information.sensor_data.radiation_type = sensor_msgs__msg__Range__ULTRASOUND;
  sensor_information.sensor_data.field_of_view = 0.5; // Field of view of the sensor in rad.
  sensor_information.sensor_data.min_range = (float)MIN_RANGE; // Minimum distance range of the sensor in m.
  sensor_information.sensor_data.max_range = MAX_RANGE; // Maximum distance range of the sensor in m.

  // create sensor_information_publisher
  RCCHECK(rclc_publisher_init_default(
    &sensor_information_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(range_sensors_interfaces, msg, SensorInformation),
    "sensor_info"));

  // create timer,

  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(100),
    timer_callback));

  // create executor
  int number_of_executors = 1;
  RCCHECK(rclc_executor_init(&executor, &support.context, number_of_executors, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
 
  Serial.println("Light-controller ready");
  digitalWrite(STATUS_LED_PIN, LOW);

}

void loop() {
  delay(10);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));

}
