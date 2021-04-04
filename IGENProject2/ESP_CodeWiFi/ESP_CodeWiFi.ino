#include "WiFi.h"
#include "SPIFFS.h"
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
 
const char* ssid = "yourNetworkName";
const char* password =  "yourNetworkPassword";
 
WebServer server;
WebSocketsServer webSocket = WebSocketsServer(81);
Adafruit_MPU6050 mpu;

/*-------------------------------------------------*/
/*-------------------------------------------------*/

//Define Pins
    //Fingers: Hand's Thumb, Pointer, and Middle
        int Finger_1 = 34;  //Pointer
        int Finger_2 = 35;  //Middle
        int Finger_3 = 32;  //Thumb
    //Buttons: Each serve different functions
        int Cali = 19;  //Calibration
        int Map1 = 18;  //Mappable Input 1
        int Map2 = 17;  //Mappable Input 2
        int Map3 = 16;  //Mappable Input 3

//Inputs
    int accel[3] = {0,0,0};
    int rotate[3] = {0,0,0};

//Outputs
    //LEDs
        int LED = 2; //Calibration Lights

    int count;
    bool calibrated;

/*--------------------------------------------------*/
/*--------------------------------------------------*/
/*
//Function to handle incoming data

// Callback for incoming event
void onEvent(AsyncWebSocket * server, AsyncWebSocketClient * client, AwsEventType type, 
             void * arg, uint8_t *data, size_t len){
   switch(type) {
      case WS_EVT_CONNECT:
        Serial.printf("Client connected: \n\tClient id:%u\n\tClient IP:%s\n", 
             client->id(), client->remoteIP().toString().c_str());
        break;
      case WS_EVT_DISCONNECT:
         Serial.printf("Client disconnected:\n\tClient id:%u\n", client->id());
         break;
      case WS_EVT_DATA:
         handlingIncomingData(arg, data, len);
         break;
      case WS_EVT_PONG:
          Serial.printf("Pong:\n\tClient id:%u\n", client->id());
          break;
      case WS_EVT_ERROR:
          Serial.printf("Error:\n\tClient id:%u\n", client->id());
          break;     
   }
  
}
//Initialize the WebSocket
void initWebSocket() {
  ws.onEvent(onEvent);
  server.addHandler(&ws);
}
*/
/*--------------------------------------------------*/
/*--------------------------------------------------*/

void setup(){
   //Assigning Inputs
   pinMode(Finger_1, INPUT);
   pinMode(Finger_2, INPUT);
   pinMode(Finger_3, INPUT);
   pinMode(Cali, INPUT);
   pinMode(Map1, INPUT);
   pinMode(Map2, INPUT);
   pinMode(Map3, INPUT);
   //Assigning Outputs
   pinMode(LED, OUTPUT);
    
  Serial.begin(115200);
  
  void readData() {
    // should only be used to set/unset flags
    read_data = true;
  }
 
  if(!SPIFFS.begin()){
     Serial.println("An Error has occurred while mounting SPIFFS");
     return;
  }
 
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi..");
  }
 
  Serial.println(WiFi.localIP());
 
  server.on("/htmlTest", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/htmlTest.html", "text/html");});
  server.on("/cssTest.css", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/cssTest.css", "text/css");});
  server.on("/javaTest.js", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/cssTest.css", "text/javascript");});
 
  server.begin(); //In search bar type http://#yourIP#/htmlTest
  webSocket.begin();

  //MPU 6050 setup
    while (!Serial)
        delay(10); //Pause Zero, Leonardo, etc until serial console opens
        Serial.println("Adafruit MPU6050 test!");
        // Try to initialize!
            if (!mpu.begin()) {
                Serial.println("Failed to find MPU6050 chip");
                while (1) {
                    delay(10);
                }
            }
        Serial.println("MPU6050 Found!");
}
 
void loop() {
  //Get Finger Sensor readings
     int Pointer = analogRead(Finger_1);
     int Middle = analogRead(Finger_2);
     int Thumb = analogRead(Finger_3);
    
  //Get Acceleration / Rotation readings
      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);
      
  webSocket.loop();
  server.handleClient();  
    String json = a.acceration.x
    json += " ";
    json += a.acceleration.y;
    json += " ";
    json += a.acceleration.z;
    json += " ";
    json += Pointer;
    json += " ";
    json += Middle;
    json += " ";
    json += Thumb;
    json += " ";
    
//    Serial.println(json); // DEBUGGING
    webSocket.broadcastTXT(json.c_str(), json.length());
  }
}
