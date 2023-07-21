/*
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp32-cam-post-image-photo-server/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*/
#include <Arduino.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "esp_camera.h"
#include <PubSubClient.h>
#include "config.h"

const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWORD;

const char* mqttServer = MQTT_BROKER_NAME;
const int mqttPort = 1883;
const char* mqttUser = MQTT_BROKER_USER;

const char* mqttPassword = MQTT_BROKER_PASSWORD;
const char* id = ID;

String serverName = "workmasterx";   // REPLACE WITH YOUR Raspberry Pi IP ADDRESS
//String serverName = "example.com";   // OR REPLACE WITH YOUR DOMAIN NAME
String serverPath = "/upload";     // The default serverPath 
const int serverPort = 5006;

// CAMERA_MODEL_AI_THINKER
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

#define BUILTIN_LED  4

// Timers
unsigned long t0;
unsigned long t1;
unsigned long t2;

// Motor
// #define B1B 15 // Motor B pins
// #define B1A 13
// #define A1B 2 // Motor B pins
// #define A1A 14

#define A1B 15 // Motor B pins
#define B1A 13
#define B1B 2 // Motor B pins
#define A1A 14

bool fwdEnabled = true;
bool bwdEnabled = true;

int speedA = 254;
int speedB = 254;


// More MQTT
WiFiClient espClient;
PubSubClient client(espClient);


// ======================================================
// Motor
// ======================================================
// ======================================================
// Setup
// ======================================================
void callback(char* topic, byte* payload, unsigned int length) {
  

  // ---------------------------------------
  if (String(topic) == String("esp/" + String(id) + "/control")) {


    StaticJsonDocument<100> doc;
    deserializeJson(doc, payload, length);

    speedA = int(doc["x"][0]);
    speedB = int(doc["x"][1]);



    
  }

  // ---------------------------------------
  if (String(topic) == String("esp/" + String(id) + "/photo")) {
    digitalWrite(BUILTIN_LED, HIGH); 
    delay(1000);
    sendPhoto();
    delay(1000); 
    digitalWrite(BUILTIN_LED, LOW); 
  }

  // ---------------------------------------
  if (String(topic) == String("esp/" + String(id) + "/stop")) {
    stop();
  }

  // ---------------------------------------
  if (String(topic) == String("esp/" + String(id) + "/forward")) {
    forward();
  }

  // ---------------------------------------
  if (String(topic) == String("esp/" + String(id) + "/backward")) {
    backward();
  }
  // ---------------------------------------
  if (String(topic) == String("esp/" + String(id) + "/left")) {
    left();
  }
  // ---------------------------------------
  if (String(topic) == String("esp/" + String(id) + "/right")) {
    right();
  }
}



// ======================================================
// MQTT
// ======================================================
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
      client.subscribe(String("esp/" + String(id) + "/photo").c_str());
      client.subscribe(String("esp/" + String(id) + "/stop").c_str());
      client.subscribe(String("esp/" + String(id) + "/forward").c_str());
      client.subscribe(String("esp/" + String(id) + "/backward").c_str());
      client.subscribe(String("esp/" + String(id) + "/right").c_str());  
      client.subscribe(String("esp/" + String(id) + "/left").c_str());  
      client.subscribe(String("esp/" + String(id) + "/control").c_str());  

    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}



// ======================================================
// Setup
// ======================================================
void setup() {
  
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); 
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);  
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println();
  Serial.print("ESP32-CAM IP Address: ");
  Serial.println(WiFi.localIP());

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  // init with high specs to pre-allocate larger buffers
  if(psramFound()){
    config.frame_size = FRAMESIZE_SXGA;
    config.jpeg_quality = 10;  //0-63 lower number means higher quality
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;  //0-63 lower number means higher quality
    config.fb_count = 1;
  }
  
  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    delay(1000);
    ESP.restart();
  }

  camera_fb_t * fb = NULL;
  fb = esp_camera_fb_get();
  if(!fb) {
    Serial.println("Camera capture failed");
    delay(1000);
    ESP.restart();
  }

  // Configure
  sensor_t * s = esp_camera_sensor_get();
  s->set_brightness(s, 0);     // -2 to 2
  s->set_contrast(s, 0);       // -2 to 2
  s->set_saturation(s, 0);     // -2 to 2
  s->set_special_effect(s, 0); // 0 to 6 (0 - No Effect, 1 - Negative, 2 - Grayscale, 3 - Red Tint, 4 - Green Tint, 5 - Blue Tint, 6 - Sepia)
  s->set_whitebal(s, 1);       // 0 = disable , 1 = enable
  s->set_awb_gain(s, 1);       // 0 = disable , 1 = enable
  s->set_wb_mode(s, 1);        // 0 to 4 - if awb_gain enabled (0 - Auto, 1 - Sunny, 2 - Cloudy, 3 - Office, 4 - Home)
  s->set_exposure_ctrl(s, 1);  // 0 = disable , 1 = enable
  s->set_aec2(s, 0);           // 0 = disable , 1 = enable
  s->set_ae_level(s, 0);       // -2 to 2
  s->set_aec_value(s, 300);    // 0 to 1200
  s->set_gain_ctrl(s, 1);      // 0 = disable , 1 = enable
  s->set_agc_gain(s, 0);       // 0 to 30
  s->set_gainceiling(s, (gainceiling_t)0);  // 0 to 6
  s->set_bpc(s, 0);            // 0 = disable , 1 = enable
  s->set_wpc(s, 1);            // 0 = disable , 1 = enable
  s->set_raw_gma(s, 1);        // 0 = disable , 1 = enable
  s->set_lenc(s, 1);           // 0 = disable , 1 = enable
  s->set_hmirror(s, 0);        // 0 = disable , 1 = enable
  s->set_vflip(s, 0);          // 0 = disable , 1 = enable
  s->set_dcw(s, 1);            // 0 = disable , 1 = enable
  s->set_colorbar(s, 0);       // 0 = disable , 1 = enable

  // Timers
  t0 = millis();
  t1 = millis();
  t2 = millis();

  // Flashlight
  pinMode(BUILTIN_LED, OUTPUT);     // Initialize the BUILTIN_LED pin as an output

  // Motor
  pinMode(A1A, OUTPUT);
  pinMode(A1B, OUTPUT);
  pinMode(B1A, OUTPUT);
  pinMode(B1B, OUTPUT);
  digitalWrite(A1A, LOW);
  digitalWrite(A1B, LOW);
  digitalWrite(B1A, LOW);
  digitalWrite(B1B, LOW);

  // MQTT Connect
  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);

  if (!client.connected()) {
    reconnect();
  }
  client.subscribe(String("esp/" + String(id) + "/photo").c_str());
  client.subscribe(String("esp/" + String(id) + "/stop").c_str());
  client.subscribe(String("esp/" + String(id) + "/forward").c_str());
  client.subscribe(String("esp/" + String(id) + "/backward").c_str());
  client.subscribe(String("esp/" + String(id) + "/right").c_str());
  client.subscribe(String("esp/" + String(id) + "/left").c_str());
  client.subscribe(String("esp/" + String(id) + "/control").c_str());
}


// ======================================================
// Loop
// ======================================================
void loop() {

  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  

  // if ((millis()-t1) > 5000) {
  //   // Humidity and Temperature measurement
  //   humidity = dht.readHumidity();
  //   temperature = dht.readTemperature(); 

  //   if (isnan(humidity)) humidity = -1;
  //   if (isnan(temperature)) temperature = -1;

  //   client.publish(String("esp/" + String(id) + "/humidity").c_str(), dtostrf(humidity, 6, 2, buffer));  
  //   client.publish(String("esp/" + String(id) + "/temperature").c_str(), dtostrf(temperature, 6, 2, buffer));  

  //   t1 = millis();
  // }

}

// ======================================================
// Send Photo
// ======================================================


void left() {          //function of forward

  analogWrite(A1A, 0);
  analogWrite(A1B, speedA);
  analogWrite(B1A, 0);
  analogWrite(B1B, speedB);

  client.publish(String("esp/" + String(id) + "/status").c_str(), "left");  
  //client.publish(String("esp/" + String(id) + "/status").c_str(), "forward");  

}

void right() {          //function of forward

    analogWrite(A1A, speedA);
    analogWrite(A1B, 0);
    analogWrite(B1A, speedB);
    analogWrite(B1B, 0);

    client.publish(String("esp/" + String(id) + "/status").c_str(), "right");  
  
}


void forward() {          //function of forward

  analogWrite(A1A, 0);
  analogWrite(A1B, speedA);
  analogWrite(B1A, speedB);
  analogWrite(B1B, 0);

  client.publish(String("esp/" + String(id) + "/status").c_str(), "forward");  
  
}


void backward() {         //function of backward
  
  analogWrite(A1A, speedA);
  analogWrite(A1B, 0);

  analogWrite(B1A, 0);
  analogWrite(B1B, speedB);

  client.publish(String("esp/" + String(id) + "/status").c_str(), "forward");  
}

void stop() {              //function of stop
  analogWrite(A1A, 0);
  analogWrite(A1B, 0);
  digitalWrite(A1A, LOW);
  digitalWrite(A1B, LOW);
  analogWrite(B1A, 0);
  analogWrite(B1B, 0);
  digitalWrite(B1A, LOW);
  digitalWrite(B1B, LOW);

    client.publish(String("esp/" + String(id) + "/status").c_str(), "stop");  
}




    
// ======================================================
// Send Photo
// ======================================================
String sendPhoto() {
  String getAll;
  String getBody;

  camera_fb_t * fb = NULL;
  fb = esp_camera_fb_get();
  if(!fb) {
    Serial.println("Camera capture failed");
    delay(1000);
    ESP.restart();
  }
  
  Serial.println("Connecting to server: " + serverName);

  if (espClient.connect(serverName.c_str(), serverPort)) {
    Serial.println("Connection successful!");    
    String head = "--RandomNerdTutorials\r\nContent-Disposition: form-data; name=\"imageFile\"; filename=\"esp32-cam.jpg\"\r\nContent-Type: image/jpeg\r\n\r\n";
    String tail = "\r\n--RandomNerdTutorials--\r\n";

    uint32_t imageLen = fb->len;
    uint32_t extraLen = head.length() + tail.length();
    uint32_t totalLen = imageLen + extraLen;
  
    espClient.println("POST " + serverPath + " HTTP/1.1");
    espClient.println("Host: " + serverName);
    espClient.println("Content-Length: " + String(totalLen));
    espClient.println("Content-Type: multipart/form-data; boundary=RandomNerdTutorials");
    espClient.println();
    espClient.print(head);
  
    uint8_t *fbBuf = fb->buf;
    size_t fbLen = fb->len;
    for (size_t n=0; n<fbLen; n=n+1024) {
      if (n+1024 < fbLen) {
        espClient.write(fbBuf, 1024);
        fbBuf += 1024;
      }
      else if (fbLen%1024>0) {
        size_t remainder = fbLen%1024;
        espClient.write(fbBuf, remainder);
      }
    }   
    espClient.print(tail);
    
    esp_camera_fb_return(fb);
    
    int timoutTimer = 10000;
    long startTimer = millis();
    boolean state = false;
    
    while ((startTimer + timoutTimer) > millis()) {
      Serial.print(".");
      delay(100);      
      while (espClient.available()) {
        char c = espClient.read();
        if (c == '\n') {
          if (getAll.length()==0) { state=true; }
          getAll = "";
        }
        else if (c != '\r') { getAll += String(c); }
        if (state==true) { getBody += String(c); }
        startTimer = millis();
      }
      if (getBody.length()>0) { break; }
    }
    Serial.println();
    espClient.stop();
    Serial.println(getBody);
  }
  else {
    getBody = "Connection to " + serverName +  " failed.";
    Serial.println(getBody);
  }
  return getBody;
}

