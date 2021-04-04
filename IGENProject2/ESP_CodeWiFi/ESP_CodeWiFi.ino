#include "WiFi.h"
#include "SPIFFS.h"
#include "ESPAsyncWebServer.h"
 
const char* ssid = "yourNetworkName";
const char* password =  "yourNetworkPassword";
 
AsyncWebServer server(80);

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
 
void setup(){
  Serial.begin(115200);
 
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
 
  server.begin();
  //In search bar type http://#yourIP#/htmlTest
}
 
void loop(){
}
