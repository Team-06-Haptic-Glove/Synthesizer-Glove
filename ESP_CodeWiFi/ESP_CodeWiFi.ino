#include <WiFi.h>
#include <Wire.h>
#include <SPIFFS.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

const char* ssid = "yourNetworkName";
const char* password =  "yourNetworkPassword";
 
AsyncWebServer server(80);
Adafruit_MPU6050 mpu;

/*-------------------------------------------------*/
/*----------------DECLARE VARIABLES----------------*/

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
    //Richard Testing
        int Switch = 13;

//Inputs
//Outputs
    //LEDs
        int LED = 2; //Calibration Lights

/*--------------------------------------------------*/
/*----------------DECLARE FUNCTIONS-----------------*/
//Create a string using sensor data
String sensorReadings(){
  //Get Flex Sensor inputs
     int Pointer = analogRead(Finger_1);
     int Middle = analogRead(Finger_2);
     int Thumb = analogRead(Finger_3);
  //Get Button inputs
     int Button1 = digitalRead(Map1);
     int Button2 = digitalRead(Map2);
     int Button3 = digitalRead(Map3);

  //Get Acceleration / Rotation readings
      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);
  //Return sensor readings
  //Serial.println(Pointer + " " + Middle + " " + Thumb + " " + a.acceleration.x + " " + a.acceleration.y + " " + a.acceleration.z);
  //return String(Pointer + " " + Middle + " " + Thumb + " " + a.acceleration.x + " " + a.acceleration.y + " " + a.acceleration.z);
  return String("YEAAAA BABBYYYYY);
}

/*--------------------------------------------------*/
/*-------------------BEGIN SETUP--------------------*/

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

 //Make sure we can read the file system
  if(!SPIFFS.begin()){
     Serial.println("An Error has occurred while mounting SPIFFS");
     return;
  }
 /*
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
  */
  //Begin WiFi web server
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi..");
  }
  Serial.println(WiFi.localIP());

  //On HTTP request for root, provide htmlTest.html
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/index.html", "text/html");
  });;
  //On HTTP request for stylesheet, provide cssTest.html
  server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/style.css", "text/css");
  });
  //On HTTP request for javascript, provide javaTest.js
  server.on("/script.js", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/script.js", "text/javascript");
  });
  //On HTTP request for Tone, provide Tone.js
  server.on("/Tone.js", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/Tone.js", "text/javascript");
  });
  //On HTTP request for sensor readings, provide string
  server.on("/sensors", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", sensorReadings().c_str());
  });


  //Images
  server.on("/zWood", HTTP_GET, [](AsyncWebServerRequest *request){
  request->send(SPIFFS, "/zWood.jpeg", "image/jpeg");
  });
  server.on("/zWood2", HTTP_GET, [](AsyncWebServerRequest *request){
  request->send(SPIFFS, "/zWood2.jpeg", "image/jpeg");
  });
  server.on("/zBlackground", HTTP_GET, [](AsyncWebServerRequest *request){
  request->send(SPIFFS, "/zBlackground.jpg", "image/jpg");
  });
  server.on("/zMetal", HTTP_GET, [](AsyncWebServerRequest *request){
  request->send(SPIFFS, "/zMetal.jpeg", "image/jpeg");
  });
  server.on("/zBlackground", HTTP_GET, [](AsyncWebServerRequest *request){
  request->send(SPIFFS, "/zSpaceground.jpg", "image/jpg");
  });
  server.on("/zDrip", HTTP_GET, [](AsyncWebServerRequest *request){
  request->send(SPIFFS, "/zDrip.png", "image/png");
  });
  server.on("/zSetting", HTTP_GET, [](AsyncWebServerRequest *request){
  request->send(SPIFFS, "/zSetting.png", "image/png");
  });
  server.on("/zGlov", HTTP_GET, [](AsyncWebServerRequest *request){
  request->send(SPIFFS, "/zGlov.jpg", "image/jpg");
  });
  server.on("/zFB", HTTP_GET, [](AsyncWebServerRequest *request){
  request->send(SPIFFS, "/zFB.png", "image/png");
  });
  server.on("/zIG", HTTP_GET, [](AsyncWebServerRequest *request){
  request->send(SPIFFS, "/zIG.png", "image/png");
  });
  server.on("/zLI", HTTP_GET, [](AsyncWebServerRequest *request){
  request->send(SPIFFS, "/zLI.png", "image/png");
  });
  server.on("/zPerson1", HTTP_GET, [](AsyncWebServerRequest *request){
  request->send(SPIFFS, "/zPerson1.jpeg", "image/jpeg");
  });
  server.on("/zPerson2", HTTP_GET, [](AsyncWebServerRequest *request){
  request->send(SPIFFS, "/zPerson2.jpe", "image/jpg");
  });
  server.on("/zPerson3", HTTP_GET, [](AsyncWebServerRequest *request){
  request->send(SPIFFS, "/zPerson3.jpeg", "image/jpeg");
  });
  server.on("/zPerson4", HTTP_GET, [](AsyncWebServerRequest *request){
  request->send(SPIFFS, "/zPerson4.jpeg", "image/jpeg");
  });
  server.on("/zPerson5", HTTP_GET, [](AsyncWebServerRequest *request){
  request->send(SPIFFS, "/zPerson5.jpg", "image/jpg");
  });
  server.on("/zPerson6", HTTP_GET, [](AsyncWebServerRequest *request){
  request->send(SPIFFS, "/zPerson6.jpg", "image/jpg");
  });
  
  //Begin Webserver
  server.begin(); //In search bar type http://#yourIP#/htmlTest
}
/*--------------------------------------------------*/
/*------------------FOREVER LOOP--------------------*/
void loop() {
}
