/////////////////////////////////////////////////////////////////
/*
  AWS IoT | ESP32CAM working as a publisher on MQTT
  Video Tutorial: https://youtu.be/7_3qbou_keg
  Created by Eric N. (ThatProject)
*/
/////////////////////////////////////////////////////////////////

#include "config.h"
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include "WiFi.h"
#include "esp_camera.h"

// =======================
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

#define ESP32CAM_PUBLISH_TOPIC   "esp32/cam_0"
#define BUILTIN_LED  4

const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWORD;

const char* mqttServer = MQTT_BROKER_NAME;
const int mqttPort = 1883;
const char* mqttUser = MQTT_BROKER_USER;

const char* mqttPassword = MQTT_BROKER_PASSWORD;
const char* id = ID;


// VGA Resolution JPEG ==> 20kbytes
const int bufferSize = 51200; //1024 * 23; // 23552 bytes



WiFiClient espClient;
PubSubClient client(espClient);



// =======================
struct __attribute__((packed)) Controll {
  long x1;
  long x2;
};

#define A1B 15 // Motor B pins
#define B1A 13
#define B1B 2 // Motor B pins
#define A1A 14

boolean light = false;

Controll myControll;   

// =======================
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


}
// =======================
void cameraInit(){
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
  config.frame_size = FRAMESIZE_VGA; // 640x480
  config.jpeg_quality = 10;
  config.fb_count = 2;

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    ESP.restart();
    return;
  }
}
// =======================
void cameraSettings(){
  // Configure
  sensor_t * s = esp_camera_sensor_get();
  s->set_brightness(s, 0);     // -2 to 2
  s->set_contrast(s, 0);       // -2 to 2
  s->set_saturation(s, 0);     // -2 to 2
  s->set_special_effect(s, 2); // 0 to 6 (0 - No Effect, 1 - Negative, 2 - Grayscale, 3 - Red Tint, 4 - Green Tint, 5 - Blue Tint, 6 - Sepia)
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
}

// =======================
void grabImage(){

  camera_fb_t * fb = NULL;
  fb = esp_camera_fb_get();

  //                                                dont send if photo is larger then buffersize
  if(fb != NULL && fb->format == PIXFORMAT_JPEG && fb->len < bufferSize){
    Serial.print("Image Length: ");
    Serial.print(fb->len);
    Serial.print("\t Publish Image: ");
    bool result = client.publish(ESP32CAM_PUBLISH_TOPIC, fb->buf, fb->len, false);
    Serial.println(result);

    if(!result){
      ESP.restart();
    }
  }
  esp_camera_fb_return(fb);
  delay(1);
}
// =======================
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
      client.subscribe(String("esp/" + String(id) + "/control").c_str());  
      client.subscribe(String("esp/" + String(id) + "/light").c_str());  

    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
// =======================
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");


  if (String(topic) == String("esp/" + String(id) + "/control")) {
    memcpy(&myControll, payload, length);

    if (myControll.x1>1500 && myControll.x2>1500) {
      // Forward
      analogWrite(A1A, 0);
      analogWrite(A1B, map(myControll.x1, 1500, 2000, 0, 255));
      analogWrite(B1A, map(myControll.x1, 1500, 2000, 0, 255));
      analogWrite(B1B, 0);

    } else if (myControll.x1<1500 && myControll.x2<1500) {
      // Backward
      analogWrite(A1A, map(myControll.x1, 1500, 1000, 0, 255));
      analogWrite(A1B, 0);
      analogWrite(B1A, 0);
      analogWrite(B1B, map(myControll.x1, 1500, 1000, 0, 255));

    } else if (myControll.x1>1500 && myControll.x2<1500) {
      // Right
      analogWrite(A1A, map(myControll.x1, 1500, 2000, 0, 255));
      analogWrite(A1B, 0);
      analogWrite(B1A, map(myControll.x1, 1500, 1500, 0, 255));
      analogWrite(B1B, 0);

    } else if (myControll.x1<1500 && myControll.x2>1500) {
      // Left
      analogWrite(A1A, 0);
      analogWrite(A1B, map(myControll.x1, 1500, 1000, 0, 255));
      analogWrite(B1A, 0);
      analogWrite(B1B, map(myControll.x1, 1500, 2000, 0, 255));
    } else {
      // Stop
      analogWrite(A1A, 0);
      analogWrite(A1B, 0);
      analogWrite(B1A, 0);
      analogWrite(B1B, 0);
    }
  } else if (String(topic) == String("esp/" + String(id) + "/light")) {
    if (light) {
      digitalWrite(BUILTIN_LED, HIGH); 
    } else {
      digitalWrite(BUILTIN_LED, LOW); 
    }
    light = !light;
  }

}


// =======================
void setup() {
  Serial.begin(115200);
  cameraInit();
  setup_wifi();
  cameraSettings();

  
  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);
  client.setBufferSize(bufferSize);

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

  myControll.x1 = 1500;
  myControll.x2 = 1500;


}
// =======================
void loop() {
  if (!client.connected()) {
    reconnect();
  } else {
    grabImage();
  }
  client.loop();
}
// =======================