/*
https://wokwi.com/projects/360194707275211777
https://github.com/gilmaimon/ArduinoWebsockets/issues/101
https://stackoverflow.com/questions/64175514/esp32-cam-websocket-wifimulti-reconnect
https://how2electronics.com/connecting-esp32-to-amazon-aws-iot-core-using-mqtt/
https://randomnerdtutorials.com/esp32-cam-ov2640-camera-settings/

// Websockets...
// I finallyfound the answer thanks to this comment. I was using the first certificate, but the second was needed. Sorry, my bad.
// The right command is:
// $ openssl s_client -showcerts -connect websocket.org:443
openssl s_client -showcerts -connect rosmqtt.tda-x.siemens-energy.cloud:443 ==> use the second displayed certificate for wss!

Note: When using MQTT and Websockets together without a sufficient delay the WSS server seems to cause a problem. Either the image is to big or the delay????

*/
#include <Arduino.h>
#include <ArduinoJson.h>
//#include <WiFi.h>
#include <WiFiClientSecure.h>
//#include <WiFiMulti.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "esp_camera.h"
#include <PubSubClient.h>
#include <ArduinoWebsockets.h>

#include "config.h"

// Timers
unsigned long t0;
unsigned long t1;
unsigned long t2;

// Motor
#define A1B 15 // Motor B pins
#define B1A 13
#define B1B 2 // Motor B pins
#define A1A 14


// Wifi
WiFiClientSecure net = WiFiClientSecure();
PubSubClient client(net);

// Websocket
using namespace websockets;
WebsocketsClient webSocket;


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



// ======================================================
// Steering Commands
// =====================================================

void moveForwardAndTurn(int leftSpeed, int rightSpeed) {  
  analogWrite(A1A, 0);  
  analogWrite(A1B, leftSpeed);  
  analogWrite(B1A, rightSpeed);  
  analogWrite(B1B, 0);  
}  

void moveBackwardAndTurn(int leftSpeed, int rightSpeed) {  
  analogWrite(A1A, leftSpeed);  
  analogWrite(A1B, 0);  
  analogWrite(B1A, 0);  
  analogWrite(B1B, rightSpeed);  
}  

void stopMotors() {  
  analogWrite(A1A, 0);  
  analogWrite(A1B, 0);  
  analogWrite(B1A, 0);  
  analogWrite(B1B, 0);  
}

void getMotorSpeedsFromJoystick(float joystickX, float joystickY, int &leftSpeed, int &rightSpeed) {  
  // Constrain the joystick values to the range of -1 to 1  
  joystickX = constrain(joystickX, -1, 1);  
  joystickY = constrain(joystickY, -1, 1);  
  
  // Calculate the raw motor speeds  
  float v = (1 - abs(joystickX)) * joystickY + joystickY;  
  float w = (1 - abs(joystickY)) * joystickX + joystickX;  
    
  // Map the motor speeds to the range of -255 to 255  
  leftSpeed = int(255 * (v - w) / 2);  
  rightSpeed = int(255 * (v + w) / 2);  
  
  // Constrain the motor speeds to the range of -255 to 255  
  leftSpeed = constrain(leftSpeed, 0, 255);  
  rightSpeed = constrain(rightSpeed, 0, 255);  
}  

// ======================================================
// Setup
// ======================================================
void callback(char* topic, byte* payload, unsigned int length) {
  
  // ---------------------------------------
  if (String(topic) == String("controller")) {

    Serial.println("Controller Command!");

    StaticJsonDocument<100> doc;
    deserializeJson(doc, payload, length);

    float turn =  doc["controller"]["x"]; // TurnRate
    float speed = doc["controller"]["y"];  // Forward
    
    int leftSpeed, rightSpeed;  
    getMotorSpeedsFromJoystick(turn, fabs(speed), leftSpeed, rightSpeed);  



    // Forward
    if ( speed >= 0.0) {

        Serial.print("BWD: ");
        Serial.print(leftSpeed);
        Serial.print(", ");
        Serial.print(rightSpeed);
        Serial.println();

        moveBackwardAndTurn(
          leftSpeed,
          rightSpeed
        );



    // Backward
    } else {
        Serial.print("FWD: ");
        Serial.print(leftSpeed);
        Serial.print(", ");
        Serial.print(rightSpeed);
        Serial.println();

        moveForwardAndTurn(
          leftSpeed,
          rightSpeed
        );

    }
  }
  // ---------------------------------------
  if (String(topic) == String("light_on")) {
    digitalWrite(BUILTIN_LED, HIGH); 
    Serial.println("Lights on");
  }
  // ---------------------------------------
  if (String(topic) == String("light_off")) {
    digitalWrite(BUILTIN_LED, LOW); 
    Serial.println("Lights off");
  }

}

// ======================================================
// MQTT
// ======================================================
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");

    // Attempt to connect
    if (client.connect(THINGNAME)) {

      Serial.println("connected");
      // Once connected, publish an announcement...
      //client.publish(String("esp/" + String(id) + "/hello").c_str(), "I'm back");
      // ... and resubscribe
      client.publish(String("esp").c_str(), "HelloFormEsp");

      client.subscribe("controller");
      client.subscribe("light_on");
      client.subscribe("light_off");

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
// Websocket
// ======================================================
void onMessageCallback(WebsocketsMessage message) {
    Serial.print("Got Message: ");
    Serial.println(message.data());
}

void onEventsCallback(WebsocketsEvent event, String data) {
    if(event == WebsocketsEvent::ConnectionOpened) {
        Serial.println("Connnection Opened");
    } else if(event == WebsocketsEvent::ConnectionClosed) {
        Serial.println("Connnection Closed");
    } else if(event == WebsocketsEvent::GotPing) {
        Serial.println("Got a Ping!");
    } else if(event == WebsocketsEvent::GotPong) {
        Serial.println("Got a Pong!");
    }
}


// ======================================================
// Send camera frame
// ======================================================
void sendFrame(){
  //capture a frame
  camera_fb_t * fb = esp_camera_fb_get();
  if (!fb) {
      Serial.println("Frame buffer could not be acquired");
      return;
  }
  //replace this with your own function
  webSocket.sendBinary((const char *)fb->buf, fb->len);

  //return the frame buffer back to be reused
  esp_camera_fb_return(fb);
}


// ======================================================
// Setup
// ======================================================
void setup() {
  
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); 

  Serial.begin(115200);

  // ---------------- Wifi ----------------
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
 
  Serial.println("Connecting to Wi-Fi");
 
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  
  Serial.println();
  Serial.print("ESP32-CAM IP Address: ");
  Serial.println(WiFi.localIP());
  Serial.println("Wifi Connection ready :)");

  // ---------------- Websocket ----------------

  Serial.print("Connecting to ");
  Serial.println(WEBSOCKET_SERVER);

  webSocket.onMessage(onMessageCallback);
  webSocket.onEvent(onEventsCallback);

  webSocket.setCACert(SSL_CERT);
  bool connected = webSocket.connect(WEBSOCKET_SERVER);  

  if (connected) {
    Serial.println("Connected");
  } else {
    Serial.println("Connection failed.");
  }


  Serial.println("Websocket ready :)");

  // ---------------- MQTT ----------------
  // Configure WiFiClientSecure to use the AWS IoT device credentials
  net.setCACert(AWS_CERT_CA);
  net.setCertificate(AWS_CERT_CRT);
  net.setPrivateKey(AWS_CERT_PRIVATE);

  // Connect to the MQTT broker on the AWS endpoint we defined earlier
  client.setServer(AWS_IOT_ENDPOINT, 8883);
  client.setCallback(callback);

  if (!client.connected()) {
    reconnect();
  }

  Serial.println("MQTT ready :)");

  // ---------------- Camera ----------------
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
  // FRAMESIZE_UXGA (1600 x 1200)
  // FRAMESIZE_QVGA (320 x 240)
  // FRAMESIZE_CIF (352 x 288)
  // FRAMESIZE_VGA (640 x 480)
  // FRAMESIZE_SVGA (800 x 600)
  // FRAMESIZE_XGA (1024 x 768)
  // FRAMESIZE_SXGA (1280 x 1024)

  if(psramFound()){
    Serial.println("PSRAM found!");
    config.frame_size = FRAMESIZE_QVGA; // FRAMESIZE_SXGA;
    config.jpeg_quality = 10;  //0-63 lower number means higher quality
    config.fb_count = 2;
  } else {
    Serial.println("Using Default");
    config.frame_size = FRAMESIZE_QVGA; //FRAMESIZE_SVGA;
    config.jpeg_quality = 10;  //0-63 lower number means higher quality
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

  Serial.println("Camera setup :)");

  // ---------------- Timers ----------------
  t0 = millis();
  t1 = millis();
  t2 = millis();

  // ---------------- Light ----------------
  pinMode(BUILTIN_LED, OUTPUT);     // Initialize the BUILTIN_LED pin as an output

  // ---------------- Motors ----------------
  pinMode(A1A, OUTPUT);
  pinMode(A1B, OUTPUT);
  pinMode(B1A, OUTPUT);
  pinMode(B1B, OUTPUT);
  digitalWrite(A1A, LOW);
  digitalWrite(A1B, LOW);
  digitalWrite(B1A, LOW);
  digitalWrite(B1B, LOW);

  Serial.println("System ready :)");
}


// ======================================================
// Loop
// ======================================================
void loop() {

  if (!client.connected()) {
    reconnect();
  }

  client.loop();

  // let the websockets client check for incoming messages
  if(webSocket.available()) {
    webSocket.poll();
    sendFrame();
    delay(10);
  }

}


