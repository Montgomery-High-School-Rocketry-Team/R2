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
const calfileName = "data.dat";
AHRS ahrs(3);
ahrsUtil::QuatUtil util = ahrsUtil::QuatUtil();

void setup(void)
{
    Serial.begin(115200);
    Wire.setClock(1000000);
    delay(1000);
    Serial.println("Orientation Sensor Test"); Serial.println("");

    BNOinit();
}

void loop() {
    if(!initQuatFound){
      quat_init = bno.getQuat();
      if(quat_init.x() != 0 && quat_init.y()  != 0 && quat_init.z()  != 0){
        initQuatFound = true;
      }
    }else{
      

          /* Get a new sensor event */
          // sensors_event_t event;
          // bno.getEvent(&event);


        /* Board layout:
              +----------+
              |         *| RST   PITCH  ROLL  HEADING
          ADR |*        *| SCL
          INT |*        *| SDA     ^            /->
          PS1 |*        *| GND     |            |
          PS0 |*        *| 3VO     Y    Z-->    \-X
              |         *| VIN
              +----------+
        */

        /* The WebSerial 3D Model Viewer expects data as heading, pitch, roll */
        // Serial.print(F("Orientation: "));
        // Serial.print(360 - (float)event.orientation.x);
        // Serial.print(F(", "));
        // Serial.print((float)event.orientation.y);
        // Serial.print(F(", "));
        // Serial.print((float)event.orientation.z);
        // Serial.println(F(""));


       


       

        //long T1 = micros();
      
        //const float timeStep = (T2-T1)*1e6;
        //data collection....
        float timeStep = 0.01;
        imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
        imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
        
        //logic
        imu::Quaternion gyroIntedQuat;
        if(gyro.magnitude() != 0){
          gyroIntedQuat = ahrs.integrateGyro(gyro, accel, quat_init, timeStep);
        }else{
          gyroIntedQuat = quat_init;
        }
        
        

        imu::Quaternion ASD = bno.getQuat();
        float tileAngleFromSensor = ahrs.tilt(ASD);
        //util.printQuat(ASD);
        // Serial.println(tileAngleFromSensor);
        // Serial.println(F("~~~~~~~~"));
        imu::Quaternion quat = gyroIntedQuat;
        quat_init = quat;
        float tiltAngleFromMath = ahrs.tilt(quat);
        // Serial.println(tiltAngleFromMath);
        // Serial.println(F("----"));
          
        //long T2 = micros();
        //Serial.println(T2-T1);
        long time = micros();

        File file;
        String data = "";
        data += String(time);
        data += ",";
        data += String(tiltAngleFromMath);
        data += ",";
        data += String(tileAngleFromSensor);
        data += ",";
        data += String(tileAngleFromSensor-tiltAngleFromMath);

        file = SD.open("bnodrift.txt", FILE_WRITE);
        file.println(data);
        file.close();


        
        // Serial.print(F("Quaternion: "));
        // Serial.print((float)quat.w(), 4);
        // Serial.print(F(", "));
        // Serial.print((float)quat.x(), 4);
        // Serial.print(F(", "));
        // Serial.print((float)quat.y(), 4);
        // Serial.print(F(", "));
        // Serial.print((float)quat.z(), 4);
        // Serial.println(F(""));

          /* Optional: Display calibration status */
          //displayCalStatus();

          /* Optional: Display sensor status (debug only) */
          //displaySensorStatus();

          /* New line for the next sample */
          // Serial.println("");

          /* Wait the specified delay before requesting new data */
          delay(BNO055_SAMPLERATE_DELAY_MS);
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
        if (SD.exists("bnodrift.txt")){
          SD.remove("bnodrift.txt");
        }
        if (SD.exists(calfileName)){
                Serial.println("Calibration File found");
                
                dataFile = SD.open(calfileName, FILE_READ);
    
                adafruit_bno055_offsets_t calibrationData;
                sensor_t sensor;
    
                bno.getSensor(&sensor);
                dataFile.read((uint8_t *)&calibrationData, sizeof(calibrationData));
    
                dataFile.close();  
                displaySensorOffsets(calibrationData);

                
    
                Serial.println("\n\nRestoring Calibration data to the BNO055...");
                bno.setSensorOffsets(calibrationData);
    
                Serial.println("\n\nCalibration data loaded into BNO055");
                foundCalib = true;

                /*---------Check Calibration-------------*/ 
                
                Serial.println("Checking Offset__________________");           
                    adafruit_bno055_offsets_t checkOffset;
                       bno.getSensorOffsets(checkOffset);
    
    
                displaySensorOffsets(checkOffset);
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
    displaySensorDetails();

    /* Optional: Display current status */
    displaySensorStatus();

   //Crystal must be configured AFTER loading calibration data into BNO055.
    bno.setExtCrystalUse(true);

    sensors_event_t event;
    bno.getEvent(&event);
    if (foundCalib){
        Serial.println("Move sensor slightly to calibrate magnetometers");
        /*


        */
        while (!bno.isFullyCalibrated())
        {
            bno.getEvent(&event);
            displayCalStatus();
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
            displayCalStatus();

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
    displaySensorOffsets(newCalib);

    Serial.println("\n\nStoring calibration data to SDCARD...");
/////////////
     if (!sdavailable) {
        Serial.println("Card failed, or not present");
        return;
    }

    Serial.println("card initialized.");
    dataFile = SD.open(calfileName, FILE_WRITE);



    dataFile.write((const uint8_t *)&newCalib, sizeof(newCalib));
    dataFile.close();
    Serial.println("Data stored to SD-Card.");
    Serial.println("\n--------------------------------\n");
}
/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
    */
/**************************************************************************/
void displaySensorDetails(void)
{
    sensor_t sensor;
    bno.getSensor(&sensor);
    Serial.println("------------------------------------");
    Serial.print("Sensor:       "); Serial.println(sensor.name);
    Serial.print("Driver Ver:   "); Serial.println(sensor.version);
    Serial.print("Unique ID:    "); Serial.println(sensor.sensor_id);
    Serial.print("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
    Serial.print("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
    Serial.print("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
    Serial.println("------------------------------------");
    Serial.println("");
    delay(500);
}

/**************************************************************************/
/*
    Display some basic info about the sensor status
    */
/**************************************************************************/
void displaySensorStatus(void)
{
    /* Get the system status values (mostly for debugging purposes) */
    uint8_t system_status, self_test_results, system_error;
    system_status = self_test_results = system_error = 0;
    bno.getSystemStatus(&system_status, &self_test_results, &system_error);

    /* Display the results in the Serial Monitor */
    Serial.println("");
    Serial.print("System Status: 0x");
    Serial.println(system_status, HEX);
    Serial.print("Self Test:     0x");
    Serial.println(self_test_results, HEX);
    Serial.print("System Error:  0x");
    Serial.println(system_error, HEX);
    Serial.println("");
    delay(500);
}

/**************************************************************************/
/*
    Display sensor calibration status
    */
/**************************************************************************/
void displayCalStatus(void)
{
    /* Get the four calibration values (0..3) */
    /* Any sensor data reporting 0 should be ignored, */
    /* 3 means 'fully calibrated" */
    uint8_t system, gyro, accel, mag;
    system = gyro = accel = mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);

    /* The data should be ignored until the system calibration is > 0 */
    Serial.print("\t");
    if (!system)
    {
        Serial.print("! ");
    }

    /* Display the individual values */
    Serial.print("Sys:");
    Serial.print(system, DEC);
    Serial.print(" G:");
    Serial.print(gyro, DEC);
    Serial.print(" A:");
    Serial.print(accel, DEC);
    Serial.print(" M:");
    Serial.print(mag, DEC);
}

/**************************************************************************/
/*
    Display the raw calibration offset and radius data
    */
/**************************************************************************/
void displaySensorOffsets(const adafruit_bno055_offsets_t &calibData)
{
    Serial.print("Accelerometer: ");
    Serial.print(calibData.accel_offset_x); Serial.print(" ");
    Serial.print(calibData.accel_offset_y); Serial.print(" ");
    Serial.print(calibData.accel_offset_z); Serial.print(" ");

    Serial.print("\nGyro: ");
    Serial.print(calibData.gyro_offset_x); Serial.print(" ");
    Serial.print(calibData.gyro_offset_y); Serial.print(" ");
    Serial.print(calibData.gyro_offset_z); Serial.print(" ");

    Serial.print("\nMag: ");
    Serial.print(calibData.mag_offset_x); Serial.print(" ");
    Serial.print(calibData.mag_offset_y); Serial.print(" ");
    Serial.print(calibData.mag_offset_z); Serial.print(" ");

    Serial.print("\nAccel Radius: ");
    Serial.print(calibData.accel_radius);

    Serial.print("\nMag Radius: ");
    Serial.print(calibData.mag_radius);
}