// BUDDY SYSTEM
#include <ArduinoMqttClient.h>
#if defined(ARDUINO_SAMD_MKRWIFI1010) || defined(ARDUINO_SAMD_NANO_33_IOT) || defined(ARDUINO_AVR_UNO_WIFI_REV2)
  #include <WiFiNINA.h>
#elif defined(ARDUINO_SAMD_MKR1000)
  #include <WiFi101.h>
#elif defined(ARDUINO_ESP8266_ESP12)
  #include <ESP8266WiFi.h>
#endif

#include "arduino_secrets.h"

char ssid[] = SECRET_SSID;        
char pass[] = SECRET_PASS;    

WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

const char broker[] = "broker.emqx.io";
int        port     = 1883;
const char topic[]  = "SIT210/wave";
const char Topic[]  = "SIT210/Wave";

//PIN DECLARATIONS 

//pin declaration for the Ultrasonic sensor
int trig = 4;
int echo = 3;
//For distance mesurement by ultrasonic
double duration=0;
double distance=0;
double constant = 0.0344/2;

int led = 2;

//---------------- SETUP -------------------//

void setup() {
  //Initialize serial and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // attempt to connect to Wifi network:
  Serial.print("Attempting to connect to WPA SSID: ");
  Serial.println(ssid);
  while (WiFi.begin(ssid, pass) != WL_CONNECTED) {
    // failed, retry
    Serial.print(".");
    delay(5000);
  }

  Serial.println("You're connected to the network");
  Serial.println();

  Serial.print("Attempting to connect to the MQTT broker: ");
  Serial.println(broker);

  if (!mqttClient.connect(broker, port)) {
    Serial.print("MQTT connection failed! Error code = ");
    Serial.println(mqttClient.connectError());

    while (1);
  }

  Serial.println("You're connected to the MQTT broker!");
  Serial.println();

  Serial.print("Subscribing to topic: ");
  Serial.println(topic);
  Serial.println();

  // subscribe to a topic
  mqttClient.subscribe(Topic);

  pinMode(led, OUTPUT);
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT); 
}

//---------------- LOOP -------------------//

void loop() {

//sending the wave to client 

  delay(2000);
  digitalWrite(trig,LOW);
  delayMicroseconds(4);
  digitalWrite(trig,HIGH);
  delayMicroseconds(10);
  
  duration = pulseIn(echo,HIGH);

  distance = duration * constant;

if (distance < 25)
{
    Serial.println("Sending a message....");
  // send message, the Print interface can be used to set the message contents
    mqttClient.beginMessage(topic);
    mqttClient.print("Nandini waved at ");
    mqttClient.println(distance);
    mqttClient.endMessage();
}


//recieving the message from client and the blinking
  int messageSize = mqttClient.parseMessage();
  
  if (messageSize) {
    
    // we received a message, print out the topic and contents
    Serial.print("Received a message with topic '");
    Serial.println(mqttClient.messageTopic());
    Serial.println("Client has sent message:");
    
    // use the Stream interface to print the contents
    while (mqttClient.available()) {
      Serial.print((char)mqttClient.read());
    }
    blink_times(3);
    Serial.println();
  }

  
}

/*

  for blinking number of times when message is recieved
  
*/
void blink_times(int i )
{
  while (i > 0)
  {
    blink(); 
    i--;
  }  
}

//setting up blink duration 
void blink()
{
  digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(500);                       // wait for a second
  digitalWrite(led, LOW);    // turn the LED off by making the voltage LOW
  delay(500); 
}
