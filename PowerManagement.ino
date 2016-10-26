
#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <cb1_msgs/PowerState.h>

/////////// DEFINES ///////////
#define ESTOP 12
#define ACTUATORS 11 // HIGH == On, LOW == Off
#define PC 10   // LOW == On, HIGH == Off
#define POWER_OFF 9
#define POWER_BTN 8
#define VOLT_SENSE 0
#define VOLT_DIV 0.346 // EQUATION - R2 / (R1 + R2) 
#define SHUTDOWN_TIMER_LENGTH 30000 // 30 seconds
#define DEBOUNCE_DELAY 200

/////////// GLOBAL VARS ///////////
//ros::NodeHandle nh;
ros::NodeHandle_<ArduinoHardware, 15, 15, 200, 200> nh;
cb1_msgs::PowerState power_state;
bool shutdown_timer_started = false;
unsigned long shutdown_time_start = 0;
unsigned long last_state_msg = 0;

unsigned long last_estop_debounce_time = 0;
byte last_estop_state = LOW;
unsigned long last_power_debounce_time = 0;
byte last_power_state = LOW;

/////////// READ INPUT FUNCTIONS ///////////
void read_voltage() {
  float voltage = analogRead(VOLT_SENSE);
  voltage = (voltage / 1024) * 5.0;
  voltage = voltage / VOLT_DIV;
  power_state.battery_voltage = (voltage * 100);
}

void read_button_states() {

  byte power_reading = digitalRead(POWER_BTN);
  if (power_reading != last_power_state){
    last_power_debounce_time = millis();
  }
  if ((millis() - last_power_debounce_time) > DEBOUNCE_DELAY){
    power_state.shutdown_btn_state = power_reading;
  }
  last_power_state = power_reading;

  byte estop_reading = digitalRead(ESTOP);
  if (estop_reading != last_estop_state){
    last_estop_debounce_time = millis();
  }
  if ((millis() - last_estop_debounce_time) > DEBOUNCE_DELAY){
    power_state.estop_triggered = estop_reading;
  }
  last_estop_state = estop_reading;

  if(power_state.estop_triggered){
    switch_actuators(false);
  } else {
    switch_actuators(true);
  }
}

/////////// CONTROL OUTPUT FUNCTIONS ///////////

void switch_pc(bool on) {
  if (on) {
    digitalWrite(PC, LOW);
    power_state.pc_breaker_state = true;
  } else {
    digitalWrite(PC, HIGH);
    power_state.pc_breaker_state = false;
  }
}

void switch_actuators(bool on) {
  if (on && !power_state.estop_triggered) {
    digitalWrite(ACTUATORS, HIGH);
    power_state.actuator_breaker_state = true;
  } else {
    digitalWrite(ACTUATORS, LOW);
    power_state.actuator_breaker_state = false;
  }
}

/////////// ROS CALLBACK FUNCTIONS ///////////

void switch_actuators_cb(const std_msgs::Bool& bool_msg) {
  switch_actuators(bool_msg.data);
}

void shutdown_timer_cb(const std_msgs::Empty& empty_msg) {
  if (shutdown_timer_started) {
    // if we've already started the timer cancel it
    shutdown_timer_started = false;
    shutdown_time_start = 0;
  } else {
    // start the shutdown timer
    shutdown_timer_started = true;
    shutdown_time_start = millis();
  }
}

/////////// PUBS & SUBS ///////////
ros::Publisher power_state_pub("power_state", &power_state);
ros::Subscriber<std_msgs::Empty> shutdown_sub("shutdown", shutdown_timer_cb);
ros::Subscriber<std_msgs::Bool> acutuator_breaker_sub("actuator_breaker", switch_actuators_cb);

void setup() {
  // setup pins
  pinMode(ESTOP, INPUT);
  pinMode(ACTUATORS, OUTPUT);
  pinMode(PC, OUTPUT);
  pinMode(POWER_OFF, OUTPUT);
  pinMode(POWER_BTN, INPUT);
  switch_actuators(false);
  switch_pc(true);
  delay(100);
  nh.getHardware()->setBaud(57600);
  delay(100);
  nh.initNode();
  nh.advertise(power_state_pub);
  nh.subscribe(shutdown_sub);
  nh.subscribe(acutuator_breaker_sub);
}

void loop() {
  read_button_states();
    
  if (shutdown_timer_started && (millis() > (shutdown_time_start + SHUTDOWN_TIMER_LENGTH))) {
    digitalWrite(POWER_OFF, HIGH);
  }

  if (millis() > (last_state_msg + 500)) {
    read_voltage();
    power_state_pub.publish(&power_state);
    last_state_msg = millis();
  }
  nh.spinOnce();
  
}

