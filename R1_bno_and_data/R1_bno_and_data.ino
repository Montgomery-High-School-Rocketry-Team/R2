#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <rocketry_lib.h>
#include <utility/quat.h>
#include "Adafruit_BMP3XX.h"

#include <TimeLib.h>
#include <SPI.h>
#include "SD.h"

/*********************** START----- DATA COLLECTION SET UP GLOBAL VALS***********************/
// montgomery sea level pressure
#define SEALEVELPRESSURE_HPA (1009)
#define BMP_CS 10


#define button 6
#define buzzer 7
// Define list of tone frequencies to play.
int toneFreq[] = { 262,   // C4
                   294,   // D4
                   330,   // E4
                   349,   // F4
                   392,   // G4
                   440,   // A4
                   494 };  // B4
int toneCount = sizeof(toneFreq)/sizeof(int);


File file;
Adafruit_BMP3XX bmp;
// seconds till apogee * targetted packaets per sec
const int secondsTillApogee = 10;
const int SIZE =  secondsTillApogee*500;
String *Data = new String[SIZE];
long startTime;
int idxx = 0;
bool Apogee = false;
/******************************************* END -----  DATA COLLECTION SET UP GLOBAL VALS **********************************/

/*********************** START ALGO GLOBAL VALUES ***********************/
/* Set the delay between fresh samples */
// 10 for 100 hz
// 5 for 200 hz
#define BNO055_SAMPLERATE_DELAY_MS (10)

Adafruit_BNO055 bno = Adafruit_BNO055(55);

const int chipSelect = 23;
File dataFile;

bool initQuatFound = false;
imu::Quaternion quat_init;
const calfileName = "data.dat";
AHRS ahrs;
//ahrsUtil::QuatUtil util = ahrsUtil::QuatUtil();


/*********************** END ALGO GLOBAL VALUES ***********************/

void setup(void)
{
    Serial.begin(115200);
    Wire.setClock(1000000);
    delay(1000);
    Serial.println("Orientation Sensor Test"); Serial.println("");

    BMPinit();
    SDinit();

    BNOinit();

    startTime = millis();
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
        if (SD.exists(calfileName)){
                Serial.println("Calibration File found");
                
                dataFile = SD.open(calfileName, FILE_READ);
    
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


void BMPinit(){
  if(!bmp.begin_SPI(BMP_CS)){
    Serial.println(F("bmp failed"));
    while (1) { delay(10); }
  }
  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_100_HZ);
}


void SDinit(){
  if(!SD.begin(BUILTIN_SDCARD)){
    Serial.println(F("SD failed"));
    while (1) { delay(10); }
  }


  pinMode(button, INPUT_PULLUP); 

  pinMode(buzzer, OUTPUT);

  if(SD.exists("data.csv")){

    PlayBuzzerUP();

    while (true){
          if (digitalRead(button) == LOW)
          {
              
              break;
          }

      }

    Serial.println("#data start");


    file = SD.open("data.csv", FILE_READ);
    while (file.available()){
      Serial.write(file.read());
    }
    file.close();

    PlayBuzzerDown();

    
    while (true){
        if (digitalRead(button) == LOW)
        {
            noTone(buzzer);
            break;
        }

    }

    SD.remove("data.csv");
  }

  // eraseFiles();

  file = SD.open("data.csv", FILE_WRITE);
  file.println("time,ax,ay,az,gx,gy,gz,mx,my,mz,temp,press,alt"); //,predApp(m)
  file.close();
  
}
