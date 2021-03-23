/*Code Information
Date: Arbitrary Day
Purpose: Take sensor readings from ESP32, send to da computer fo da processin
*/

/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/

//Define Libraries
    //Arduino
#include "Arduino.h"
    //ESP-32
#include "WiFi.h"
#include "BluetoothSerial.h"
    //MPU-6050
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

BluetoothSerial SerialBT;
Adafruit_MPU6050 mpu;

//Bluetooth setup
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/

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
        
//Additional Constants
int Resistor = 1000; //Resistance in Ohms

/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/

//Functions
    //Calibration
        //Glove Calibrates after 3s --> Have LEDs as visual feedback
    void calibrate(){
        //Reset local variables
            calibrated = false;
            count = 0;
        //While Calibrating
        while (calibrated = false){
            //If moving, reset timer
            while(accel != {0,0,0} || rotate != {0,0,0}{
                digitalWrite(LED, HIGH);
                delay(300);
                digitalWrite(LED, LOW);
                delay(1000);
                calibrated = false;
                count = 0;
            }
            //If still, begin counting calibration
            while (count <= 2){
                digitalWrite(LED, HIGH);
                delay(300);
                digitalWrite(LED, LOW);
                delay(1000);
                count = count + 1;
            }
            digitalWrite(LED, HIGH);
            delay(2000);
            digitalWrite(LED, LOW);
            calibrated = true;
        }
    }
    
    //Finger Flex
        //Value of given aspect changes with the flex of the finger
    void fingerflex(finger){
        newVal = analogRead(finger);
        return newVal;
    }

/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/

//Body
    //Setup Code, run once
void setup() {
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
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
        Serial.print("Accelerometer range set to: ");
        switch (mpu.getAccelerometerRange()) {
            case MPU6050_RANGE_2_G:
                Serial.println("+-2G");
                break;
            case MPU6050_RANGE_4_G:
                Serial.println("+-4G");
                break;
            case MPU6050_RANGE_8_G:
                Serial.println("+-8G");
                break;
            case MPU6050_RANGE_16_G:
                Serial.println("+-16G");
                break;
        }
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
        Serial.print("Gyro range set to: ");
        switch (mpu.getGyroRange()) {
            case MPU6050_RANGE_250_DEG:
                Serial.println("+- 250 deg/s");
                break;
            case MPU6050_RANGE_500_DEG:
                Serial.println("+- 500 deg/s");
                break;
            case MPU6050_RANGE_1000_DEG:
                Serial.println("+- 1000 deg/s");
                break;
            case MPU6050_RANGE_2000_DEG:
                Serial.println("+- 2000 deg/s");
                break;
        }
    mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
        Serial.print("Filter bandwidth set to: ")
        switch (mpu.getFilterBandwidth()) {
            case MPU6050_BAND_260_HZ:
                Serial.println("260 Hz");
                break;
            case MPU6050_BAND_184_HZ:
                Serial.println("184 Hz");
                break;
            case MPU6050_BAND_94_HZ:
                Serial.println("94 Hz");
                break;
            case MPU6050_BAND_44_HZ:
                Serial.println("44 Hz");
                break;
            case MPU6050_BAND_21_HZ:
                Serial.println("21 Hz");
                break;
            case MPU6050_BAND_10_HZ:
                Serial.println("10 Hz");
                break;
            case MPU6050_BAND_5_HZ:
                Serial.println("5 Hz");
                break;
        }
    
    //Bluetooth setup
    if(!SerialBT.begin("ESP32")){
        Serial.println("An error occurred initializing Bluetooth");
    }
    else {
        Serial.println("The device started, now you can pair it with bluetooth!");
    }
}

/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/

    //Running code, run eternally
void loop() {
    //Calibrate when button pressed
    if (digitalRead(CALI)==HIGH) {
        calibrate();
    } 
    
    //Get Finger Sensor readings
        int Pointer = fingerflex(Finger_1);
        int Middle = fingerflex(Finger_2);
        int Thumb = fingerflex(Finger_3);
    
    //Get Acceleration / Rotation readings
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);
        
        accel = {a.acceleration.x, a.acceleration.y, a.acceleration.z};
        rotate = {g.gyro.x, g.gyro.y, g.gyro.z};
    
    //Display sensor readings
        //Sends Acceleration / Rotation
        Serial.print("Acceleration (x,y,z): "); Serial.println(accel);
        Serial.print("Rotation: "); Serial.println(rotation);
        //Sends Finger Bend
        Serial.print("Pointer Bend: "); Serial.println(Pointer);
        Serial.print("Middle Bend: "); Serial.println(Middle);
        Serial.print("Thumb Bend: "); Serial.println(Thumb);

    //Reads the serial monitor line for information
        //Sends whatever is read as a bluetooth packet
    if (Serial.available()) {
        SerialBT.write(Serial.read());
    }
    delay(20);
}

/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/

//Installing ESP32 for Arduino_IDE: https://randomnerdtutorials.com/installing-the-esp32-board-in-arduino-ide-windows-instructions/

//Uploading Code to ESP32: https://dronebotworkshop.com/esp32-intro/

//Bluetooth code: https://www.instructables.com/ESP32-Bluetooth-Tutorial-How-to-Use-Inbuilt-Blueto/
//MPU6050 code: https://raw.githubusercontent.com/RuiSantosdotme/Random-Nerd-Tutorials/master/Projects/ESP/ESP_MPU6050_Basic_Demo.ino
