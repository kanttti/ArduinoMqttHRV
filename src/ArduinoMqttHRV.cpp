// Megalle koodi joka ottaa Serial1 vastaan viestejä toiselta arduinolta ja kasvattaa FanSpeed laskuria
// ja lähettää fanspeed tiedon takaisin toiselle arduinolle

/*
Commands from Nano to Mega on Serial

+   Increase fan speed
-   Decrease fan speed
A   Timer OFF
B   Timer level 1
C   Timer level 2
D   Timer level 3

Commands from Mega to Nano on serial

1   Fan speed 1 (turn 1 led on)
2   Fan speed 2 (turn 2 leds on)
3   Fan speed 3 (turn 3 leds on)
4   Fan speed 4 (turn 4 leds on)
N   Timer Off (no Timer leds on)
O   Timer level 1 (1 led on)
P   Timer level 2 (2 leds on)
Q   Timer level 3 (3 leds on)

*/

#include <Arduino.h>
#include <SPI.h>
#include <Ethernet.h>
#include <PubSubClient.h>
#include <EEPROM.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <ArduinoJson.h>
#include <CmdParser.hpp>

//---------------------------------------------------------------------------

#define FanSpeed_EEADDR 33  // EEPROM address to store current fan speed
#define STATES 20           // STATE loops per second
#define MinFanSpeed 1       // Minimum value for fanSpeed
#define MaxFanSpeed 8       // Maximum value for fanSpeed
#define ExhaustFanOffPin 38 // Pin to use for exhaust fan relay
#define IntakeFanOffPin 42 // Pin to use for exhaust fan relay
#define AirHeaterPin 40 // Pin to use for exhaust fan relay
#define TimerLevel1 300     // Seconds for exhaust fan timer for each level from Arduino Nano (Default:300)
#define TimerLevel2 900     // Seconds for exhaust fan timer for each level from Arduino Nano (Default:900)
#define TimerLevel3 1800    // Seconds for exhaust fan timer for each level from Arduino Nano (Default:1800)
#define TEMPERATURE_MEAS_PERIOD 60 // Interval for temperature measurement in seconds
#define MQTT_CLIENTNAME "ArduinoLTO"
#define MQTT_USERNAME ""
#define MQTT_PASSWORD ""

// Sensors
#define ONE_WIRE_BUS_PIN 44 // Pin to use for DS18B20 temperature sensors
#define SENSOR_RESOLUTION 12
#define MAXIMUM_NUMBER_OF_SENSORS 20

// Global variables
int fanSpeed = 1;
int fanSpeedMemory = 255;
int exhFanOffTime = 0;
int timerLevel = 0;
int temp_meas_period = 0;
unsigned long state_loop_timer = 0;
unsigned long reconnect_loop_timer = 0;
unsigned long exh_fan_off_timer = 0;
unsigned long lastTempRequest = 0;
int state = 0;  
int fanSpeedPins[] = { 22, 36, 34, 32, 30, 28, 26, 24 };   // Pins used for fanspeeds in speed order
int numOfFanSpeeds = 8;                    // Number of pins declared above
bool fanSpeedChanged = false;
bool exhFanOff = false;
int numberOfSensors = 0;
bool debug = false;

// Define array of structs for OneWire sensor addresses
typedef struct 
  {
      DeviceAddress SensorAddress;
  } Addresses;

Addresses SensorAddresses[MAXIMUM_NUMBER_OF_SENSORS];

CmdParser cmdParser;

// Declare functions
void mqttCallback(char* topic, byte* payload, unsigned int length);
void StateLoop();
void reconnect();
void handleSerial();
void communicateTimer();
void communicateFanSpeed();
void handleFanSpeed();
void setExhFanOff();
void setExhFanOn();
void increaseFanSpeed();
void decreaseFanSpeed();
void increaseTimer();
void decreaseTimer();
void printTemperature(float tempC, byte address[8]);
void mqttPublishTemperature(float tempC, byte address[8]);

// MAC addrees and IP address
byte mac[]    = {  0xDE, 0xED, 0xBA, 0xFE, 0xFE, 0xED };
IPAddress ip(10, 0, 0, 222);

// IP Address of your MQTT broker - change to adapt to your network
byte server[] = { 10, 0, 0, 200 }; // not shown

// Initiate instances -----------------------------------

EthernetClient ethClient;
PubSubClient client(server, 1883, mqttCallback, ethClient);
OneWire oneWire(ONE_WIRE_BUS_PIN);  // Setup a OneWire instance to communicate with OneWire devices
DallasTemperature sensors(&oneWire);  // Pass OneWire reference to Dallas Temperature

//-------------------------------------------------------

void setup() {
  // Begin the Serial1 at 19200 Baud for other arduino and Serial at 19200 for USB debug
  Serial.begin(19200);
  Serial1.begin(19200);
  pinMode(LED_BUILTIN, OUTPUT);
  delay(3000);   // wait for 3 seconds to let Ethernet board startup properly
  pinMode(ExhaustFanOffPin, OUTPUT);
  digitalWrite(ExhaustFanOffPin, HIGH);
  for (int i = 0; i < numOfFanSpeeds; i++) {
    pinMode(fanSpeedPins[i], OUTPUT); // turn fan pins OUTPUT
  }
  for (int i = 0; i < numOfFanSpeeds; i++) {
    digitalWrite(fanSpeedPins[i], HIGH); // turn fan pins LOW
  }
  // If Eeprom is corrupted. Set fanspeed to 1
  fanSpeed = EEPROM.read(FanSpeed_EEADDR);
  if (fanSpeed < MinFanSpeed || fanSpeed > MaxFanSpeed){
    EEPROM.write(FanSpeed_EEADDR, 1);
  }
  
  // Setup ethernet connection to MQTT broker
  Ethernet.begin(mac, ip);
  // Initialize Dallas Temperature measurement library
  sensors.begin();
  int i = 0;
  while(oneWire.search(SensorAddresses[i].SensorAddress)) {
    i++;
  }
  numberOfSensors = i;
  for (i = 0; i < numberOfSensors; i = i +1) {
    Serial.print("Sensor ");
    Serial.print(i+1);
    Serial.print(": ");
    char writedisplay[20];
    sprintf (writedisplay, "%02X%02X%02X%02X%02X%02X%02X%02X",
              SensorAddresses[i].SensorAddress [0],
              SensorAddresses[i].SensorAddress [1],
              SensorAddresses[i].SensorAddress [2],
              SensorAddresses[i].SensorAddress [3],
              SensorAddresses[i].SensorAddress [4],
              SensorAddresses[i].SensorAddress [5],
              SensorAddresses[i].SensorAddress [6],
              SensorAddresses[i].SensorAddress [7]);
    Serial.println(writedisplay);
    }

    // set the resolution to 10 bit (Can be 9 to 12 bits .. lower is faster)
  for (i = 0; i < numberOfSensors; i = i + 1) {
    sensors.setResolution(SensorAddresses[i].SensorAddress, SENSOR_RESOLUTION);
  }
  sensors.setWaitForConversion(false); // Set to ASYNC mode
    // Command all devices on bus to read temperature  
  sensors.requestTemperatures();  
  lastTempRequest = millis();

  Serial1.print("Speed ");
  Serial1.println(fanSpeed);

}  //--(end setup )---




void loop() {
//  if (!client.connected()) {      // IF MQTT not connected try to reconnect
//    reconnect();
//  }
  if (millis() >= state_loop_timer) StateLoop();
  handleSerial();
  client.loop();
  if (exhFanOffTime > 0) setExhFanOff();
  if (millis() >= exh_fan_off_timer && exhFanOff) setExhFanOn();
}

/*********************************************************************************************\
 * State loop
\*********************************************************************************************/

void StateLoop() {
  state_loop_timer = millis() + (1000 / STATES);
  state++;

/*-------------------------------------------------------------------------------------------*\
 * Every second
\*-------------------------------------------------------------------------------------------*/

  if (STATES == state) {
    state = 0;
    handleFanSpeed();
    
    temp_meas_period++;
    if (temp_meas_period == TEMPERATURE_MEAS_PERIOD -2) {
      sensors.requestTemperatures();
      Serial.println("Getting temperatures... ");
    }
    if (temp_meas_period >= TEMPERATURE_MEAS_PERIOD) {
      temp_meas_period = 0;
      int i;
      for (i = 0; i < numberOfSensors; i = i + 2) {
        float tempC = sensors.getTempC(SensorAddresses[i].SensorAddress);
        printTemperature(tempC, SensorAddresses[i].SensorAddress);
        if (client.connected()) {      // IF MQTT connected publish temperatures
          mqttPublishTemperature(tempC, SensorAddresses[i].SensorAddress);
        }
        Serial.println();
      }
    }
    if (temp_meas_period == (TEMPERATURE_MEAS_PERIOD/2) -2) {
      sensors.requestTemperatures();
      Serial.println("Getting temperatures... ");
    }
    if (temp_meas_period == (TEMPERATURE_MEAS_PERIOD/2)) {
      int i;
      for (i = 1; i < numberOfSensors; i = i + 2) {
        float tempC = sensors.getTempC(SensorAddresses[i].SensorAddress);
        printTemperature(tempC, SensorAddresses[i].SensorAddress);
        if (client.connected()) {      // IF MQTT connected publish temperatures
          mqttPublishTemperature(tempC, SensorAddresses[i].SensorAddress);
        }
        Serial.println();
      }
    }

  }
 

/*-------------------------------------------------------------------------------------------*\
 * Every 0.05 second
\*-------------------------------------------------------------------------------------------*/

  if (!(state % (STATES/20))) {
    communicateFanSpeed();
    if (millis() < exh_fan_off_timer && exhFanOff) communicateTimer();
  }
}

void reconnect() {
  if (millis() >= reconnect_loop_timer) {          // Try every 10 seconds 
    reconnect_loop_timer = millis() + (10000);
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(MQTT_CLIENTNAME,MQTT_USERNAME,MQTT_PASSWORD)) {    // change as desired - clientname must be unique for MQTT broker
      client.publish("stat/LTO/connected","Online");
      Serial.println("connected");
      byte pubArray[] = { byte(fanSpeed+48) };
      client.publish("stat/LTO/fanspeed", pubArray, 1);
      client.subscribe("cmnd/LTO/fanspeed");          // subscribe to topic
      client.subscribe("cmnd/LTO/exhFanOffTimer");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 10 seconds");
    }
  }
}

// Handle serial messages from other arduino
 
void handleSerial() {

  CmdBuffer<32> myBuffer;
  if (Serial1.available()) {
    if (myBuffer.readFromSerial(&Serial1,500)) {
          
        if (cmdParser.parseCmd(&myBuffer) != CMDPARSER_ERROR) {
            // SERIAL COMMANDS
            
            // Speed
            if (cmdParser.equalCommand_P(PSTR("Speed"))) {
                Serial.print(cmdParser.getCmdParam(0));
                Serial.print(" ");
                Serial.println(cmdParser.getCmdParam(1));
                if (cmdParser.equalCmdParam(1,"up")) {
                  increaseFanSpeed();
                } else if (cmdParser.equalCmdParam(1,"down")) {
                  decreaseFanSpeed();
                }
            }

            // Timer
            if (cmdParser.equalCommand_P(PSTR("Timer"))) {
                Serial.print(cmdParser.getCmdParam(0));
                Serial.print(" ");
                Serial.println(cmdParser.getCmdParam(1));
                if (cmdParser.equalCmdParam(1,"up")) {
                  increaseTimer();
                } else if (cmdParser.equalCmdParam(1,"down")) {
                  decreaseTimer();
                }
            }

        }
      }
    }

}

// Handle timer messages to other arduino and MQTT

void communicateTimer() {
  int timeLeft = (exh_fan_off_timer - millis()) / 1000;  // Calculate remaining time
   int level = 0;                                         // define levels based on seconds to communicate to other Arduino via Serial 
     if (timeLeft > 0) {
       level = 1;
     }
     if (timeLeft > TimerLevel1) {
       level = 2;
     }
     if (timeLeft > TimerLevel2) {
       level = 3;
     }
     if (timerLevel != level) {
       Serial1.print("Timer ");
       Serial1.println(level);                       // Send O,P,Q... to other arduino to tell the level.
       Serial.print("Timer level = ");               // Just for DEBUG
       Serial.print(level);                          // Just for DEBUG
       Serial.print(" (");                           // Just for DEBUG
       Serial.write(level+78);                       // Just for DEBUG
       Serial.println(")");                          // Just for DEBUG
       timerLevel = level;
       byte pubArray[] = { byte(level+48) };            // Publish level as numbers 1-3 to MQTT
       client.publish("stat/LTO/exhFanOffTimer", pubArray, 1);
     }
     
}

// Handle fan speed messages to other arduino and MQTT

void communicateFanSpeed() {
   if (fanSpeed != fanSpeedMemory) {
      Serial1.print("Speed ");
      Serial1.println(fanSpeed);
      Serial.print("Communicated fan speed is ");
      Serial.println(fanSpeed);
      if (client.connected()) {      // IF MQTT connected publish speed
        byte pubArray[] = { byte(fanSpeed+48) };
        client.publish("stat/LTO/fanspeed", pubArray, 1);
      }
      fanSpeedMemory = fanSpeed;
      fanSpeedChanged = true;
    }
}

// Set fanspeed relay field according to fanspeed setting

void handleFanSpeed() {
  if (fanSpeedChanged) {
    for (int i = 0; i < numOfFanSpeeds; i++) {  // Loop all fanSpeedPins
      if (i+1 == fanSpeed) {                    // If iteration is current speed set pin LOW
        digitalWrite(fanSpeedPins[i], LOW);     // turn ON current speed relay
      }
      if (i+1 != fanSpeed) {                    // If iteration is NOT current speed set pin HIGH
        digitalWrite(fanSpeedPins[i], HIGH);    // turn OFF current speed relay
      }
    }
    Serial.print("Fan Speed Changed to ");
    Serial.println(fanSpeed);
    EEPROM.write(FanSpeed_EEADDR, fanSpeed);
    fanSpeedChanged = false;
  }
}

// Set exhaust fan OFF if timer set --------------------------------------------------------

void setExhFanOff() {
  exh_fan_off_timer = millis() + (long(exhFanOffTime)*1000);
  digitalWrite(ExhaustFanOffPin, LOW);
  Serial.print("Setting Exhaust Fan Off for ");
  Serial.print(exhFanOffTime);
  Serial.println(" seconds");
  exhFanOff = true;
  exhFanOffTime = 0;
}

// Set exhaust fan back on after timer done ------------------------------------------------

void setExhFanOn() {
  digitalWrite(ExhaustFanOffPin, HIGH);
  Serial.println("Setting Exhaust Fan On");
//  Serial1.write("N");                            // Send "N" (Normal mode) to other arduino
//  int level = 0;
//  byte pubArray[] = { byte(level+48) };              // Send 0 to MQTT to tell timer is off
//  client.publish("stat/LTO/timer", pubArray, 1);
  exhFanOff = false;
}

// Increase Fan speed
 
void increaseFanSpeed() {
  if (fanSpeed < MaxFanSpeed){
        fanSpeed++;
  }
}

// Decrease Fan speed
 
void decreaseFanSpeed() {
  if (fanSpeed > MinFanSpeed){
        fanSpeed--;
  }
}


// Handle and convert incoming MQTT messages ----------------------------------------

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  // handle message arrived
  char content[10] = "";
  for (unsigned int num=0;num<length;num++) {
      content[num] = payload[num];
  }  

// Print to serial for Debug
  Serial.print("Received MQTT on topic ");
  Serial.print(topic);
  Serial.print(" ");
  Serial.print(content);
  Serial.println();
  
// Set variables based on received MQTT payloads

 if (strcmp(topic,"cmnd/LTO/fanspeed") == 0){
   if (content[0]-48 >= MinFanSpeed && content[0]-48 <= MaxFanSpeed) {
     fanSpeed = content[0]-48;
   }
 }
 if (strcmp(topic,"cmnd/LTO/exhFanOffTimer") == 0){
   exhFanOffTime = atoi(content);
   if (exhFanOffTime <= 0 && exhFanOff){
     exhFanOffTime = 2;    // If exhFanOffTime is 0 and Fan is OFF, use 3 seconds to turn fan on
   }
   }
 }


void printTemperature(float tempC, byte address[8])
{
   if (tempC == -127.00) 
   {
   Serial.print("Error getting temperature  ");
   } 
   else
  {
   Serial.print("Probe ");
   char tempAddress[20];
   sprintf (tempAddress, "%02X%02X%02X%02X%02X%02X%02X%02X",address[0],address[1],address[2],address[3],address[4],address[5],address[6],address[7]);
   Serial.print(tempAddress);
   Serial.print(" temperature is:  ");
   Serial.print(tempC);
   Serial.print(" ");
   }
}
// End printTemperature


void mqttPublishTemperature(float tempC, byte address[8])
{
   if (tempC == -127.00) 
   {
    // Add mqtt error message here
   } 
   else
  {
    // Create Topic for MQTT publish (format /topic/LTO/SENSOR/sensoraddress)
    char tempAddress[20];
    sprintf (tempAddress, "%02X%02X%02X%02X%02X%02X%02X%02X",address[0],address[1],address[2],address[3],address[4],address[5],address[6],address[7]);
    char pubTopic[40];
    sprintf(pubTopic,"tele/LTO/SENSOR/%s",tempAddress);
    
    // Create Temperature Json object for MQTT payload    
    StaticJsonBuffer<200> jsonBuffer;
    JsonObject& message = jsonBuffer.createObject();
    message["Temperature"] = tempC;
    char pubMessage[40];
    message.printTo(pubMessage, sizeof(pubMessage));
    // Serial printing for debugging only
    if (debug) {
    Serial.println();
    Serial.print("Publishing to topic: ");
    Serial.println(pubTopic);
    Serial.print("Payload: ");
    Serial.println(pubMessage);
    }
    client.publish(pubTopic, pubMessage);
  }
}