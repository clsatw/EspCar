#include <ESP8266WiFi.h>
// #define LOGGING
// has to be included on 1st line
#include "logging.h"

// Import required libraries
// Libs for connecting to any available wifi
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>         //https://github.com/tzapu/WiFiManager

// aRest
#include <PubSubClient.h>
#include <aREST.h>

// Clients
WiFiClient espClient;
PubSubClient client(espClient);

// Create aREST instance
aREST rest = aREST(client);

// Unique ID to identify the device for cloud.arest.io
// char* device_id = "clsa0916";
char* device_id = "107929";

// #define CLSA
// WiFi parameters
#ifdef CLSA
const char* ssid = "CLSA";
const char* password = "1234Abcd";
#else
const char* ssid = "c1225";
const char* password = "c0918321359";
#endif

char * key = "1obqzch8x3e7e626";
// Functions
void callback(char* topic, byte* payload, unsigned int length);

// enable one of the motor shields below
//#define ENABLE_ADAFRUIT_MOTOR_DRIVER
#ifdef ENABLE_ADAFRUIT_MOTOR_DRIVER
#include "adafruit_motor_driver.h"
#define LEFT_MOTOR_INIT 3 // M3 and M4
#define RIGHT_MOTOR_INIT 4
#endif

#define ENABLE_NODEMCU_V1_MOTOR_DRIVER
#ifdef ENABLE_NODEMCU_V1_MOTOR_DRIVER
#include "nodemcu_v1_motor_driver.h"
#define LEFT_MOTOR_INIT D1, D3   // A+ and A- pwm:D1, direction: D3
#define RIGHT_MOTOR_INIT D2, D4  // B+ and B-
#endif

#include "nodemcu_ultrasonic_sensor.h"
const int trigPin = D6;  //D4
const int echoPin = D8;  //D3
#define MAX_DISTANCE 200
Roach:: DistanceSensor dSensor(trigPin, echoPin, MAX_DISTANCE);

const uint8_t MIN_SPEED = 00; // no less than 30%
const uint8_t MAX_SPEED = 255; // no greater than 50%
const uint8_t FIX_SPEED = 170; // right motor fix at 40% speed.

// Roach is a namespace
Roach::Motor leftMotor(LEFT_MOTOR_INIT);
Roach::Motor rightMotor(RIGHT_MOTOR_INIT);

// auto mode
unsigned long startTime;
#define irLeft D5 // A0
#define irRight D7 // A2

// Declare functions to be exposed to the API
int stop(String command);
int forward(String command);
int left(String command);
int right(String command);
int backward(String command);
// operation mode: self driving or manual
int op(String command);
int speed(String command);
bool selfDriving = false; // default manual mode
int motorSpeed = 150;
uint8_t maxDist2Wall = 8; // 3cm.
int fsmDelay = 30; // default 20ms

enum state_t { F, L, R, B};
state_t s;
// Linked data structure
struct State
{
  int (*cmdPtr)(String); // output func, param String is only for aRest
  unsigned long delay;  // time to delay in ms
  //unsigned long next[8];
  state_t next[8];
}; // Next if 3-bit input is 0-7
typedef const struct State StateType;
typedef StateType *StatePtr;
StateType fsm[4] = {
  {&forward, fsmDelay, {B, B, R, L, B, B, R, F}},   // Center
  {&left,    fsmDelay, {B, B, L, F, B, B, R, F}},   // off to the Left
  {&right,   fsmDelay, {B, B, R, L, B, B, F, F}},   // off to the Right state, we need to turn left
  {&backward, random(fsmDelay * 3, fsmDelay * 4), {B, B, L, L, B, B, R, R}}
  // {&backward, random(30, 40), {B, B, L, L, B, B, R, !random(2) ? L : R}}
};

uint8_t irSensorInput() {
  uint8_t irL0, irL1, irR0, center;
  int irR1;
  center = (dSensor.getDistance()) < maxDist2Wall ? 0 : 1; // distance to center < 8cm
  irL0 = digitalRead(irLeft);
  irL1 = digitalRead(LED_BUILTIN) && irL0;

  irR1 = analogRead(A0);
  // in fact it outputs 1024 when no obstacle ahead
  irR0 = (irR1 < 1000) ? 0 : 1;
  irR1 = digitalRead(irRight) && irR0;
  // log("irL1: %u\n", irL1);
  return irL1 << 2 | center << 1 | irR1;
  //return digitalRead(irLeft) << 2 | center << 1 | digitalRead(irRight);
}

// Functions
void callback(char* topic, byte* payload, unsigned int length);

void setup() {
  Serial.begin(115200);           // set up Serial library at 115200 bps
  //WiFiManager
  //Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;
  //sets timeout until configuration portal gets turned off
  //useful to make it all retry or go to sleep
  //in seconds
  wifiManager.setTimeout(180);

  //or use this for auto generated name ESP + ChipID
  //fetches ssid and pass and tries to connect
  //if it does not connect it starts an access point with the specified name
  //here  "AutoConnectAP"
  //and goes into a blocking loop awaiting configuration
  if (!wifiManager.autoConnect()) {
    log("failed to connect and hit timeout");
    delay(3000);
    //reset and try again, or maybe put it to deep sleep
    ESP.reset();
    delay(5000);
  }
  //if you get here you have connected to the WiFi
  log("connected...yeey :)");

  /* Connect to WiFi
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println(".");
    }
    Serial.println("");
    Serial.println("WiFi connected");
  */

  // Set aREST key
  rest.setKey(key, client);
  // Set callback
  client.setCallback(callback);

  // Give name and ID to device
  rest.set_id(device_id);
  rest.set_name("clsa0916");

  // Function to be exposed
  rest.function("forward", forward);
  rest.function("stop", stop);
  rest.function("right", right);
  rest.function("left", left);
  rest.function("backward", backward);
  rest.function("op", op);
  rest.function("speed", setSpeed);
  //log("ip addr: %u\n", WiFi.localIP());

  // Set output topic
  // char* out_topic = rest.get_topic();

  // auto mode
  pinMode(irLeft, INPUT);
  // LED_BULLTIN is D0
  pinMode(LED_BUILTIN, INPUT);
  pinMode(irRight, INPUT);

  s = F; // state initialization
  startTime = millis();
}

void loop() {
  uint8_t input;
  unsigned long currentTime = millis();
  unsigned long elapsedTime = currentTime - startTime;

  // Connect to the cloud
  rest.handle(client);

  if (selfDriving == true) {
    (fsm[s].cmdPtr)("NULL");  // aRest requires a String for param
    input = irSensorInput(); // read sensors
    // log("input: %u\n", input);
    if (elapsedTime >= fsm[s].delay) {
      //delay(fsm[s].delay);     // wait
      s = fsm[s].next[input];  // next state depends on input and current state
      //log("next state: %u\n", s);
      startTime = currentTime;
    }
  }
}

// Handles message arrived on subscribed topic(s)
void callback(char* topic, byte * payload, unsigned int length) {
  // Handle
  rest.handle_callback(client, topic, payload, length);
}

// Functions to control the robot
int forward(String command) {
  // log("motorSpeed: %u\n", motorSpeed);
  rightMotor.setSpeed(motorSpeed);
  leftMotor.setSpeed(motorSpeed + 10);
}
int backward(String command) {
  log("back");
  leftMotor.setSpeed(-motorSpeed);
  rightMotor.setSpeed(-motorSpeed);
}
// coz i'm lazy to change wiring
int right(String command) {
  log("left");
  rightMotor.setSpeed(motorSpeed);
  leftMotor.setSpeed(-motorSpeed);
  //leftMotor.setSpeed(-FIX_SPEED);
}
int left(String command) {
  log("right");
  leftMotor.setSpeed(motorSpeed);
  rightMotor.setSpeed(-motorSpeed);
  // rightMotor.setSpeed(MIN_SPEED);
}
int stop(String command) {
  // log(command);
  leftMotor.setSpeed(0);
  rightMotor.setSpeed(0);
}
int op(String command) {
  log("driving mode: %u\n", selfDriving);
  selfDriving = !selfDriving;
  if (!selfDriving) {
    stop("stop");
  }
}
int setSpeed(String command) {
  int params[3], i = 0, from = 0, idx = 0;
  // 3 input params
  Serial.println("Robot params");
  idx = command.indexOf(",");
  for (i = 0; i < 3; i++) {
    params[i] = command.substring(from, idx).toInt();
    from = idx + 1;
    idx = command.indexOf(",", from);
  }

  motorSpeed = params[0];
  maxDist2Wall = params[1];
  fsmDelay = params[2];
  log("speed: %u\n", motorSpeed);
  log("dist: %u\n", maxDist2Wall);
  log("delay: %u\n", fsmDelay);
}



