
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <rocketry_lib.h>
#include <utility/quat.h>

/* Set the delay between fresh samples */
// 10 for 100 hz
// 5 for 200 hz
#define BNO055_SAMPLERATE_DELAY_MS (10)

Adafruit_BNO055 bno = Adafruit_BNO055(55);
#include "SD.h"
#include "SPI.h"

const int chipSelect = 23;
File dataFile;

bool initQuatFound = false;
imu::Quaternion quat_init;

AHRS ahrs;
// ahrsUtil::QuatUtil util = ahrsUtil::QuatUtil();



void setup() {
  Serial.begin(115200);
  Wire.setClock(1000000L);
  delay(1000);
  Serial.println("Orientation Sensor Test"); Serial.println("");
  BNOinit();
  bno.restoreDefults();
  
  delay(3000);

}

void loop() {
  if(!initQuatFound){
      quat_init = bno.getQuat();
      if(quat_init.x() != 0 && quat_init.y()  != 0 && quat_init.z()  != 0){
          initQuatFound = true;
          bno.changeToAccGyro();
          //bno.set16Grange();
          bno.set2000dps523HZ();
          bno.set16Gand1000HZ();
      
        
      }
    }else{

        // float timeStep = 0.01;
        long T1 = micros();

        imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
        imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
        
        long T2 = micros();
        //Serial.println(T2-T1);
        
        // Serial.println(bno.modee());
        // Serial.println(bno.AccConfig());
        // Serial.println(bno.GyroConfig());


        // Serial.println("-----");
        float timeStep = 0.0065;
        imu::Quaternion gyroIntedQuat;
        if(gyro.magnitude() != 0){
          gyroIntedQuat = ahrs.integrateGyro(gyro, accel, quat_init, timeStep);
        }else{
          gyroIntedQuat = quat_init;
        }
        
        
        // imu::Quaternion ASD = bno.getQuat();
        // float tileAngleFromSensor = ahrs.tilt(ASD);
        // //util.printQuat(ASD);
        // Serial.println(tileAngleFromSensor);
        // Serial.println(F("~~~~~~~~"));
        imu::Quaternion quat = gyroIntedQuat;
        quat_init = quat;
        float tiltAngleFromMath = ahrs.tilt(quat);
        Serial.println(tiltAngleFromMath);
        Serial.println(F("----"));


        // long time = millis();
        // File file;
        // String data = "";
        // data += String(time);
        // data += ",";
        // data += String(tiltAngleFromMath);
        // data += ",";
        // data += String(tileAngleFromSensor);
        // data += ",";
        // data += String(tileAngleFromSensor-tiltAngleFromMath);

        // file = SD.open("bnodrift.txt", FILE_WRITE);
        // file.println(data);
        // file.close();

        //delay(BNO055_SAMPLERATE_DELAY_MS);

        
          
    }

}

void BNOinit(){
      /* Initialise the sensor */
    if (!bno.begin())
    {
        /* There was a problem detecting the BNO055 ... check your connections */
        Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
        while (1);
    }

    bool foundCalib = false;
    bool sdavailable = false;

   if (SD.begin(BUILTIN_SDCARD)) {        
        sdavailable = true;        
        Serial.println("card initialized.");
        if (SD.exists("data.dat")){
                Serial.println("Calibration File found");
                
                dataFile = SD.open("data.dat", FILE_READ);
    
                adafruit_bno055_offsets_t calibrationData;
                sensor_t sensor;
    
                bno.getSensor(&sensor);
                dataFile.read((uint8_t *)&calibrationData, sizeof(calibrationData));
    
                dataFile.close();  
                ahrs.displaySensorOffsets(calibrationData);

                
    
                Serial.println("\n\nRestoring Calibration data to the BNO055...");
                bno.setSensorOffsets(calibrationData);
    
                Serial.println("\n\nCalibration data loaded into BNO055");
                foundCalib = true;

                /*---------Check Calibration-------------*/ 
                
                Serial.println("Checking Offset__________________");           
                    adafruit_bno055_offsets_t checkOffset;
                       bno.getSensorOffsets(checkOffset);
    
    
                ahrs.displaySensorOffsets(checkOffset);
                Serial.println("\nChecking Offset end_______________"); 
                /*--------------------------------------*/ 
        }
        else {
           Serial.println("No Calibration File found");
          }}
    else {
          Serial.println("Card failed, or not present");
          }
          

    delay(1000);

    /* Display some basic information on this sensor */
    ahrs.displaySensorDetails(bno);

    /* Optional: Display current status */
    ahrs.displaySensorStatus(bno);

   //Crystal must be configured AFTER loading calibration data into BNO055.
    bno.setExtCrystalUse(true);

    sensors_event_t event;
    bno.getEvent(&event);
    if (foundCalib){
        Serial.println("Move sensor slightly to calibrate magnetometers");
        while (!bno.isFullyCalibrated())
        {
            bno.getEvent(&event);
            ahrs.displayCalStatus(bno);
            Serial.println(" ");
            delay(BNO055_SAMPLERATE_DELAY_MS);
        }
    }
    else
    {
        Serial.println("Please Calibrate Sensor: ");
        while (!bno.isFullyCalibrated())
        {
            bno.getEvent(&event);

            Serial.print("X: ");
            Serial.print(event.orientation.x, 4);
            Serial.print("\tY: ");
            Serial.print(event.orientation.y, 4);
            Serial.print("\tZ: ");
            Serial.print(event.orientation.z, 4);

            /* Optional: Display calibration status */
            ahrs.displayCalStatus(bno);

            /* New line for the next sample */
            Serial.println("");

            /* Wait the specified delay before requesting new data */
            delay(BNO055_SAMPLERATE_DELAY_MS);
        }
    }

    Serial.println("\nFully calibrated!");
    Serial.println("--------------------------------");
    Serial.println("Calibration Results: ");
    adafruit_bno055_offsets_t newCalib;
    bno.getSensorOffsets(newCalib);
    ahrs.displaySensorOffsets(newCalib);

    Serial.println("\n\nStoring calibration data to SDCARD...");

     if (!sdavailable) {
        Serial.println("Card failed, or not present");
        return;
    }

    Serial.println("card initialized.");
    dataFile = SD.open("data.dat", FILE_WRITE);



    dataFile.write((const uint8_t *)&newCalib, sizeof(newCalib));
    dataFile.close();
    Serial.println("Data stored to SD-Card.");
    Serial.println("\n--------------------------------\n");
}
void LogData(float accelX, float accelY, float accelZ, float gyroX, float gyroY, float gyroZ){
    
    // long T1 = micros();
    
    long time = micros();
    
    String data = "";
    data.concat(time);
    data.concat(",");
    data.concat(accelX);
    data.concat(",");
    data.concat(accelY);
    data.concat(",");
    data.concat(accelZ);
    data.concat(",");
    data.concat(gyroX);
    data.concat(",");
    data.concat(gyroY);
    data.concat(",");
    data.concat(gyroZ);
    data.concat(",");

    File file;
    file = SD.open("data.csv", FILE_WRITE);
    file.println(data);
    file.close();
}
