#include <Arduino.h>

#include <EEPROM.h>

#include <micro_ros_platformio.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/bool.h>


rcl_publisher_t sensor_information_publisher;

rclc_executor_t executor;


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




void error_loop(){
  Serial.printf("Light RGB controller\nError\nSystem halted");
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
    RCSOFTCHECK(rcl_publish(&sensor_information_publisher, &scenery_light_status[scenery_light_state_index], NULL));
  }
}




void setup() {

  Serial.begin(115200);
  while (!Serial);
  delay(2000);
  Serial.print("Light-controller started, node: ");
  Serial.println(NODE_NAME);

  EEPROM.begin(sizeof(EEPROM_STORE) * NUMBER_OF_SCENERY_LIGHTS);


  pinMode(STATUS_LED_PIN, OUTPUT); 
  digitalWrite(STATUS_LED_PIN, HIGH);



  const char *host_name = convertToCamelCase(NODE_NAME);
  Serial.printf("hostname :%s\n", host_name);
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
  //neopixelWrite(RGB_BUILTIN,0,0,RGB_BRIGHTNESS);
#if defined(ARDUINO_ESP32C3_DEV)
#elif defined(ARDUINO_ESP32S3_DEV)
  ws2812fxStatus.setColor(0, 0, RGB_BRIGHTNESS);
  ws2812fxStatus.service();
#else
    "Unknown Platform"
#endif


  Serial.printf(" Scenery Light Control WiFi Connected\n");


  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, NODE_NAME, "", &support));

  char topic_name[40];


  sprintf(topic_name, "railtrack/scenery/status");
  // create sensor_information_publisher
  RCCHECK(rclc_publisher_init_default(
    &sensor_information_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(railway_interfaces, msg, SceneryState),
    topic_name));

  // create timer,

  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(100),
    timer_callback));

  // create executor
  int number_of_executors = 2;
  RCCHECK(rclc_executor_init(&executor, &support.context, number_of_executors, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
 
    Serial.println("Light-controller ready");
    digitalWrite(STATUS_LED_PIN, LOW);

}

void loop() {
  delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));

}
