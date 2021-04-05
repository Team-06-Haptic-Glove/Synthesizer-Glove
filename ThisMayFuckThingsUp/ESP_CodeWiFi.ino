#include <WiFi.h>
#include <Wire.h>
#include <SPIFFS.h>
#include <ESPAsyncTCP.h>
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

//Inputs
//Outputs
    //LEDs
        int LED = 2; //Calibration Lights

/*--------------------------------------------------*/
/*----------------DECLARE FUNCTIONS-----------------*/
//Callback: send webpages
//Send homepage
void onIndexRequest(AsyncWebServerRequest *request){
  IPAddress remote_ip = request->client()->remoteIP();
  Serial.println("[" + remote_ip.toString() + "] HTTP GET request of " + request->url());
  request->send(SPIFFS, "/htmlTest.html", "text/html");
}
//Send stylesheet
void onCSSRequest(AsyncWebServerRequest *request){
  IPAddress remote_ip = request->client()->remoteIP();
  Serial.println("[" + remote_ip.toString() + "] HTTP GET request of " + request->url());
  request->send(SPIFFS, "/cssTest.css", "text/css");
}
//Send javaScript
void onJavaRequest(AsyncWebServerRequest *request){
  IPAddress remote_ip = request->client()->remoteIP();
  Serial.println("[" + remote_ip.toString() + "] HTTP GET request of " + request->url());
  request->send(SPIFFS, "/javaTest.js", "text/javascript");
}

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
  Serial.println(Pointer + " " + Middle + " " + Thumb + " " + a.acceleration.x + " " + a.acceleration.y + " " + a.acceleration.z);
  return String(Pointer + " " + Middle + " " + Thumb + " " + a.acceleration.x + " " + a.acceleration.y + " " + a.acceleration.z);
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

  //Begin WiFi web server
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi..");
  }
  Serial.println(WiFi.localIP());

  //On HTTP request for root, provide htmlTest.html
  server.on("/", HTTP_GET, onIndexRequest);
  //On HTTP request for stylesheet, provide cssTest.html
  server.on("/cssTest.css", HTTP_GET, onCSSRequest);
  //On HTTP request for javascript, provide javaTest.html
  server.on("/javaTest.js", HTTP_GET, onJavaRequest);
  //On HTTP request for sensor readings, provide string
  server.on("/sensors", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", sensorReadings().c_str());
  });

  //Begin Webserver
  server.begin(); //In search bar type http://#yourIP#/htmlTest
}
/*--------------------------------------------------*/
/*------------------FOREVER LOOP--------------------*/
void loop() {
}
