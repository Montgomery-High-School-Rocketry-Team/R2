#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <rocketry_lib.h>
#include <utility/quat.h>
#include "Adafruit_BMP3XX.h"
#include <Stepper.h>
#include <SPI.h>
#include "SD.h"

/*********************** START----- DATA COLLECTION SET UP GLOBAL VALS***********************/
// montgomery sea level pressure
#define SEALEVELPRESSURE_HPA (1009)
#define BMP_CS 10

File file;
Adafruit_BMP3XX bmp;
// seconds till apogee * targetted packaets per sec
const int secondsTillApogee = 10;
const int SIZE =  secondsTillApogee*400;
String *Data = new String[SIZE];
long startTime;
int idxx = 0;
bool Apogee = false;
//in in seconds
//float GLOB_DT = 0.0145;
float GLOB_DT = 0.01;
/******************************************* END -----  DATA COLLECTION SET UP GLOBAL VALS **********************************/

/*********************** START ALGO GLOBAL VALUES ***********************/




/*************** START BNO STUFF***************/
/* Set the delay between fresh samples */
// 10 for 100 hz
// 5 for 200 hz
#define BNO055_SAMPLERATE_DELAY_MS (10)

Adafruit_BNO055 bno = Adafruit_BNO055(55);

File dataFile;

bool initQuatFound = false;
imu::Quaternion quat_init;
AHRS ahrs(SEALEVELPRESSURE_HPA);
//ahrsUtil::QuatUtil util = ahrsUtil::QuatUtil();
/***************  END BNO STUFF***************/

/*************** START BRIAN STUFF***************/
// determine size later, i think we go with the algo's stuff
// OOPSP NVM
float axx[SIZE];
float ayy[SIZE];
float azz[SIZE];
float altitude[SIZE];

/*************** END BRIAN STUFF***************/

/*********************** END ALGO GLOBAL VALUES ***********************/

void setup(void)
{
    Serial.begin(115200);
    Wire.setClock(1000000);
    //Serial.println("Orientation Sensor Test"); Serial.println("");

    BMPinit();
    ahrs.SDinit(BUILTIN_SDCARD);

    BNOinit();
    bno.restoreDefults();
    
    quat_init = ahrs.loop_find_quat_init(bno);

    //GROUND_ALT = ahrs.get_ground_alt(bmp);

    ahrs.before_launch_detection(bno,bmp);

    startTime = millis();
}

void loop() {
    imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
      

    float* dataPtr;
    dataPtr = ahrs.GetData(accel,gyro, bno, bmp);

    float* v;
    v = accel_to_v();
    // float vx = v[0];
    // float vy = v[1];
    // float vz = v[2];

    imu::Quaternion ASD = bno.getQuat();
    float tileAngleFromSensor = ahrs.tilt(ASD);

    LogData(dataPtr[0],dataPtr[1],dataPtr[2],dataPtr[3],dataPtr[4],dataPtr[5],dataPtr[6],dataPtr[7],dataPtr[8], tileAngleFromSensor);
    
    float alt = altitude[idxx];

    idxx = ahrs.motor_logic(startTime, secondsTillApogee, Data, idxx, alt,bno,bmp);
    
 }

    
void LogData(float accelX, float accelY, float accelZ, float gyroX, float gyroY, float gyroZ, float board_temperature, float press, float alt, float angle){
    
    // long T1 = micros();
    
    long time = millis();
    
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
    // data.concat(magx);
    // data.concat(",");
    // data.concat(magy);
    // data.concat(",");
    // data.concat(magz);
    // data.concat(",");
    data.concat(String(board_temperature,0));
    data.concat(",");
    data.concat(String(press,0));
    data.concat(",");
    data.concat(String(alt,0));
    data.concat(",");
    data.concat(String(angle,3));


    /// BIG ERROR - I NEED TO MAKE SURE THE  DATA [ ] ARRAY IS LARGE ENOUGH AND THEN CHECK IF THE SIZE==IDX OR SMTH, THEN I NEED TO FIND A WAY TO DELETE/CLEAR THE ARRAY OR DATA[] = [] or smth
    // nvm update ^^ 10 minutes later - i think my old code solved this issue by just making a super duper big array that can store all the flight's data until apogee or smth... 
    // TODO: i still need to implement the data in the else statement - i think just uncoment the stuff in the else statement and just go from there.....
    // greate
    if(time - startTime > secondsTillApogee*1000 && !Apogee){
      //Serial.println("WRITING TO data.csv");
      
      file = SD.open("data.csv", FILE_WRITE);
     
      for(int i=0; i<idxx; i++ ){
        file.println(Data[i]);
        
      }
      file.close();
     // Serial.println("WRITING TO data.csv - DONE");
      //delay(10000);
      Apogee = true;

      // this global dt is found from uncommenting the code under the else statement
      // time it takes to run the void loop
      GLOB_DT = 0.025;
      
    }else if(!Apogee){
      Data[idxx] = data;
      altitude[idxx] = alt;
      // Serial.println(Data[idxx]);
      // Serial.println(idxx);
      axx[idxx] = accelX;
      ayy[idxx] = accelY;
      azz[idxx] = accelZ;
      idxx ++;
    }else{
      
      // file = SD.open("data.csv", FILE_WRITE);
      // file.println(data);
      // file.close();
    
    }
    

    
    
    // long T2 = micros();
    // Serial.println(T2-T1);


}

void BMPinit(){
  if(!bmp.begin_SPI(BMP_CS)){
    //Serial.println(F("bmp failed"));
    while (1) { delay(10); }
  }
  // if(!bmp.begin_I2C()){
  //   Serial.println(F("bmp failed"));
  //   while (1) { delay(10); }
  // }
  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_200_HZ);
}

void BNOinit(){
      /* Initialise the sensor */
    if (!bno.begin())
    {
        /* There was a problem detecting the BNO055 ... check your connections */
       // Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
        while (1);
    }

    bool foundCalib = false;
    bool sdavailable = false;

   if (SD.begin(BUILTIN_SDCARD)) {        
        sdavailable = true;        
        //Serial.println("card initialized.");
        if (SD.exists("data.dat")){
               // Serial.println("Calibration File found");
                
                dataFile = SD.open("data.dat", FILE_READ);
    
                adafruit_bno055_offsets_t calibrationData;
                sensor_t sensor;
    
                bno.getSensor(&sensor);
                dataFile.read((uint8_t *)&calibrationData, sizeof(calibrationData));
    
                dataFile.close();  
              //  ahrs.displaySensorOffsets(calibrationData);

                
    
               // Serial.println("\n\nRestoring Calibration data to the BNO055...");
                bno.setSensorOffsets(calibrationData);
    
               // Serial.println("\n\nCalibration data loaded into BNO055");
                foundCalib = true;

                /*---------Check Calibration-------------*/ 
                
              //  Serial.println("Checking Offset__________________");           
                adafruit_bno055_offsets_t checkOffset;
                bno.getSensorOffsets(checkOffset);
    
    
               // ahrs.displaySensorOffsets(checkOffset);
              //  Serial.println("\nChecking Offset end_______________"); 
                /*--------------------------------------*/ 
        }
        else {
          // Serial.println("No Calibration File found");
          }}
    else {
         // Serial.println("Card failed, or not present");
          }
          

    delay(1000);

    /* Display some basic information on this sensor */
    //ahrs.displaySensorDetails(bno);

    /* Optional: Display current status */
   // ahrs.displaySensorStatus(bno);

   //Crystal must be configured AFTER loading calibration data into BNO055.
    bno.setExtCrystalUse(true);

    sensors_event_t event;
    bno.getEvent(&event);

    ahrs.play_diff_sound_for_diff_cali(bno, BNO055_SAMPLERATE_DELAY_MS);

    // Serial.println("\nFully calibrated!");
    // Serial.println("--------------------------------");
    // Serial.println("Calibration Results: ");
    adafruit_bno055_offsets_t newCalib;
    bno.getSensorOffsets(newCalib);
    //ahrs.displaySensorOffsets(newCalib);

    //Serial.println("\n\nStoring calibration data to SDCARD...");
/////////////
     if (!sdavailable) {
       // Serial.println("Card failed, or not present");
        return;
    }

    //Serial.println("card initialized.");
    dataFile = SD.open("data.dat", FILE_WRITE);



    dataFile.write((const uint8_t *)&newCalib, sizeof(newCalib));
    dataFile.close();


   // Serial.println("Data stored to SD-Card.");
   // Serial.println("\n--------------------------------\n");
}




float* accel_to_v(){
  int a = 0;
  int b = idxx;
  float dt = GLOB_DT;
  
  float vx = ahrs.integrate(a,b,axx,dt);
  float vy = ahrs.integrate(a,b,ayy,dt);
  float vz = ahrs.integrate(a,b,azz,dt);

  //ahrs.sqrt10(const double number)
  // |v|  = sqrt10(vx*vx + vy*vy + vz*vz);
  static float v[3];
  v[0] = vx;
  v[1] = vy;
  v[2] = vz;

  return v;


}

