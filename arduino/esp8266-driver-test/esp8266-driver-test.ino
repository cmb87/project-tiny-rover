#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include "config.h"

// https://arduino.stackexchange.com/questions/50311/how-to-send-float-numbers-from-python3-with-struct-to-arduino
// Update these with values suitable for your network.

const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWORD;

const char* mqttServer = MQTT_BROKER_NAME;
const int mqttPort = 1883;
const char* mqttUser = MQTT_BROKER_USER;

const char* mqttPassword = MQTT_BROKER_PASSWORD;
const char* id = ID;


WiFiClient espClient;
PubSubClient client(espClient);
unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE	(50)
char msg[MSG_BUFFER_SIZE];
int value = 0;



struct __attribute__((packed)) Controll {
  long x1;
  long x2;
};

Controll myControll;   


void setup_wifi() {

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  myControll.x1 = 1500;
  myControll.x2 = 1500;

}



void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    //String clientId = "ESP8266Client-";
    //clientId += String(random(0xffff), HEX);
    
    // Attempt to connect
    if (client.connect(id, mqttUser, mqttPassword )) {
    //if (client.connect(clientId.c_str())) {



      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish(String("esp/" + String(id) + "/hello").c_str(), "I'm back");
      // ... and resubscribe
      client.subscribe("control");


    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}


void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");

  memcpy(&myControll, payload, length);

  Serial.print(myControll.x1);
  Serial.print(" ");
  Serial.print(myControll.x2);

  // for (int i = 0; i < length; i++) {
  //   Serial.print((char)payload[i]);
  // }

  Serial.println();

  // Switch on the LED if an 1 was received as first character
  if ((char)payload[0] == '1') {
    digitalWrite(BUILTIN_LED, LOW);   // Turn the LED on (Note that LOW is the voltage level
    // but actually the LED is on; this is because
    // it is active low on the ESP-01)
  } else {
    digitalWrite(BUILTIN_LED, HIGH);  // Turn the LED off by making the voltage HIGH
  }

}


void setup() {
  pinMode(BUILTIN_LED, OUTPUT);     // Initialize the BUILTIN_LED pin as an output
  Serial.begin(115200);
  setup_wifi();

  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);

  if (!client.connected()) {
    reconnect();
  }
  client.subscribe("control");


}

void loop() {

  if (!client.connected()) {
    reconnect();
  }
  client.loop();




  // unsigned long now = millis();
  // if (now - lastMsg > 2000) {
  //   lastMsg = now;
  //   ++value;
  //   snprintf (msg, MSG_BUFFER_SIZE, "hello world #%ld", value);
  //   Serial.print("Publish message: ");
  //   Serial.println(msg);
  //   client.publish("outTopic", msg);
  // }

}
