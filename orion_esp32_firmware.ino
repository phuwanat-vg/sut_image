#include <ros.h>
#include "Arduino.h"
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Range.h>
#include <WiFi.h>

ros::NodeHandle nh;

//=========================================================
const float wheel_radius = 0.0321;
const float wheel_base = 0.1175;
const float min_speed = 6.23;
const float max_speed = 9.345;

const int freq = 30000;
const int pwmChannel = 0;
const int pwmChannel2 = 1;
const int resolution = 8;
int dutyCycle = 200;
float v = 0.0;
float w = 0.0;
float vr = 0.0;
float vl = 0.0;

float range = 0.0;

#define LED_BUILTIN 13

const int trig = 13;
const int echo = 12;

sensor_msgs::Range range_msg;
ros::Publisher pub_range( "/ultrasound", &range_msg);

char frameid[] = "/ultrasound";


#define L_MOTOR_PWM 14
#define L_MOTOR_DIR1 27
#define L_MOTOR_DIR2 26
#define R_MOTOR_PWM 32
#define R_MOTOR_DIR1 33
#define R_MOTOR_DIR2 25

const char* ssid     = "WIFI SSID";
const char* password = "WIFI PASSWORD";
// Set the rosserial socket server IP address
IPAddress server(192, 168, 1, xxx);
// Set the rosserial socket server port
const uint16_t serverPort = 11411;


void turnWheel( const float wheel_power,
                unsigned int pwm_pin,
                unsigned int dir1_pin,
                unsigned int dir2_pin) {
  float factor = max(min(wheel_power, 1.0f), -1.0f);
  //Serial.print(wheel_power.data);
  if ( factor >= 0 ) {
    digitalWrite(dir1_pin, LOW);
    digitalWrite(dir2_pin, HIGH);
    ledcWrite(pwm_pin, (unsigned int)(255 * factor));
  } else {
    digitalWrite(dir1_pin, HIGH);
    digitalWrite(dir2_pin, LOW);
    ledcWrite(pwm_pin, (unsigned int)(255 * (-1.0f * factor)));
  }
}

void ultrasonic() {
  long duration;

  pinMode(trig, OUTPUT);
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(5);
  digitalWrite(trig, LOW);
  pinMode(echo, INPUT);
  duration = pulseIn(echo, HIGH);

  range = microsecondsToCentimeters(duration);

  Serial.println(range);

  range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg.header.frame_id =  frameid;
  range_msg.field_of_view = 0.1;  // fake
  range_msg.min_range = 0.0;
  range_msg.max_range = 6.47;
  range_msg.range = range*0.01;
  range_msg.header.stamp = nh.now();
  pub_range.publish(&range_msg);
  delay(5);



}

float microsecondsToCentimeters(long microseconds)
{
  return microseconds / 29.0 / 2.0;
}

void rightWheelCb( const std_msgs::Float32 &wheel_power ) {

  turnWheel( wheel_power.data, pwmChannel2, R_MOTOR_DIR1, R_MOTOR_DIR2 );

}

void leftWheelCb( const std_msgs::Float32 &wheel_power ) {

  turnWheel( wheel_power.data, pwmChannel, L_MOTOR_DIR1, L_MOTOR_DIR2 );
}


float vel_to_power(float vel) {
  float factor = 0.0;
  float power = 0.0;



  factor = (abs(vel) - min_speed) / (max_speed - min_speed);
  power = (factor * (1.0 - 0.75)) + 0.75;

  if (vel < 0) {
    power = power * -1;
  }
  return power;
}

void velCb( const geometry_msgs::Twist &vel_data ) {

  float R = wheel_radius;
  float L = wheel_base;

  v = vel_data.linear.x;
  w = vel_data.angular.z;

  vr = ((2.0 * v) + (2.0 * w)) / (2.0 * R);
  vl = ((2.0 * v) - (2.0 * w)) / (2.0 * R);

  vr = max(min(vr, max_speed), -max_speed);
  vl = max(min(vl, max_speed), -max_speed);

  turnWheel(vel_to_power(vl), pwmChannel, L_MOTOR_DIR1, L_MOTOR_DIR2);
  turnWheel(vel_to_power(vr), pwmChannel2, R_MOTOR_DIR1, R_MOTOR_DIR2);
}

ros::Subscriber<std_msgs::Float32> sub_right("wheel_power_right",
    &rightWheelCb );
ros::Subscriber<std_msgs::Float32> sub_left("wheel_power_left",
    &leftWheelCb );
ros::Subscriber<geometry_msgs::Twist> sub_vel("cmd_vel",
    &velCb );


void setup()
{

  ledcAttachPin(L_MOTOR_PWM, pwmChannel);
  ledcAttachPin(R_MOTOR_PWM, pwmChannel2);
  ledcSetup(pwmChannel, freq, resolution);
  ledcSetup(pwmChannel2, freq, resolution);


  // initialize LED digital pin as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  //pinMode(L_MOTOR_PWM, OUTPUT);
  pinMode(L_MOTOR_DIR1, OUTPUT);
  pinMode(L_MOTOR_DIR2, OUTPUT);

 // pinMode(R_MOTOR_PWM, OUTPUT);
  pinMode(R_MOTOR_DIR1, OUTPUT);
  pinMode(R_MOTOR_DIR2, OUTPUT);



  // Init motors to stop
  digitalWrite(L_MOTOR_DIR1, LOW);
  digitalWrite(L_MOTOR_DIR2, LOW);
  digitalWrite(R_MOTOR_DIR1, LOW);
  digitalWrite(R_MOTOR_DIR2, LOW);
  ledcWrite(L_MOTOR_PWM, 0);
  ledcWrite(R_MOTOR_PWM, 0);


  Serial.begin(115200);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  // Connect the ESP8266 the the wifi AP
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  // Set the connection to rosserial socket server
  nh.getHardware()->setConnection(server, serverPort);
  nh.initNode();

  // Another way to get IP
  Serial.print("IP = ");
  Serial.println(nh.getHardware()->getLocalIP());
  //Serial.begin(57600);
  nh.subscribe(sub_right);
  nh.subscribe(sub_left);
  nh.subscribe(sub_vel);
  nh.advertise(pub_range);

  delay(20);
}

void loop()
{
  //nh.loginfo("Log Me");
  ultrasonic();
  nh.spinOnce();

  // wait for a second
  delay(20);

}
