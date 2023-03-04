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

//TODO: CHANGE THIS LATER LOL
#define button 23
#define buzzer 21
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
const int SIZE =  secondsTillApogee*400;
String *Data = new String[SIZE];
long startTime;
int idxx = 0;
bool Apogee = false;
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
AHRS ahrs;
//ahrsUtil::QuatUtil util = ahrsUtil::QuatUtil();
/***************  END BNO STUFF***************/

/*************** START APOGEE PRED STUFF***************/
// determine size later, i think we go with the algo's stuff
// OOPSP NVM
float ax[SIZE];
float ay[SIZE];
float az[SIZE];
/*
  data[0] = accelX;
  data[1] = accelY;
  data[2] = accelZ;
  data[3] = gyroX;
  data[4] = gyroY;
  data[5] = gyroZ;
*/



/***************  END APOGEE PRED STUFF***************/

/*********************** END ALGO GLOBAL VALUES ***********************/

void setup(void)
{
    Serial.begin(115200);
    Wire.setClock(1000000);
    delay(1000);
    Serial.println("Orientation Sensor Test"); Serial.println("");

    //bmp is ACTUALLY ACTUALLY SPI
    BMPinit();
    SDinit();

    BNOinit();
    bno.restoreDefults();

    delay(3000);

    startTime = millis();
}

void loop() {
    if(!initQuatFound){
      quat_init = bno.getQuat();
      if(quat_init.x() != 0 && quat_init.y()  != 0 && quat_init.z()  != 0){
        initQuatFound = true;
        bno.changeToAccGyro();
        // bno.set16Grange();
        bno.set2000dps523HZ();
        bno.set16Gand1000HZ();
      }
    }else{
        long T1 = micros();
      
        float timeStep = 0.006;
        imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
        imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
        
        
        float* dataPtr;
        dataPtr = GetData(accel,gyro);
        LogData(dataPtr[0],dataPtr[1],dataPtr[2],dataPtr[3],dataPtr[4],dataPtr[5],dataPtr[6],dataPtr[7],dataPtr[8]);
      
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


        
      
        //const float timeStep = (T2-T1)*1e6;
        //data collection....
        
        
        //logic
        imu::Quaternion gyroIntedQuat;
        if(gyro.magnitude() != 0){
          gyroIntedQuat = ahrs.integrateGyro(gyro, accel, quat_init, timeStep);
        }else{
          gyroIntedQuat = quat_init;
        }
      

        // imu::Quaternion ASD = bno.getQuat();
        // float tileAngleFromSensor = ahrs.tilt(ASD);
        //util.printQuat(ASD);
        // Serial.println(tileAngleFromSensor);
        // Serial.println(F("~~~~~~~~"));
        imu::Quaternion quat = gyroIntedQuat;
        quat_init = quat;
        float tiltAngleFromMath = ahrs.tilt(quat);
        // Serial.println(tiltAngleFromMath);
        // Serial.println(F("----"));
          
        

         // Brian's Values
        float alt = bmp.readAltitude(SEALEVELPRESSURE_HPA);
        
         //tiltAngleFromMath
        // accelX = ax[idxx] ;
        // accelY = ay[idxx];
        // accelZ = az[idxx];
        
        float* v;
        v = accel_to_v();
        float vx = v[0];
        float vy = v[1];
        float vz = v[2];

        long T2 = micros();
        Serial.println(T2-T1);
    }

    
}



// change to spi later 
void BMPinit(){
  // if(!bmp.begin_SPI(BMP_CS)){
  //   Serial.println(F("bmp failed"));
  //   while (1) { delay(10); }
  // }
  if(!bmp.begin_I2C()){
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

    ahrs.PlayBuzzerUP(buzzer, toneFreq, toneCount);

    while (true){
          if (digitalRead(button) == LOW)
          {
              
              break;
          }

      }

    Serial.println("##data start");


    file = SD.open("data.csv", FILE_READ);
    while (file.available()){
      Serial.write(file.read());
    }
    file.close();

    Serial.println("##data end");

    ahrs.PlayBuzzerDown(buzzer, toneFreq, toneCount);

    
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
  //file.println("time,ax,ay,az,gx,gy,gz,mx,my,mz,temp,press,alt"); //,predApp(m)
  file.println("time,ax,ay,az,gx,gy,gz,temp,press,alt"); 
  file.close();
  
}

float* GetData(imu::Vector<3> accel, imu::Vector<3> gyro ){

  
  // We are not doing mag cuz like its so slow and not useful for our algo
  // sensors_event_t accel;
  // sensors_event_t gyro;
  // sensors_event_t temp;
  // sox.getEvent(&accel, &gyro, &temp);

  //   //uTesla
  // sensors_event_t mag; 
  // lis3mdl.getEvent(&mag);
  // float magx = mag.magnetic.x;
  // float magy = mag.magnetic.y;
  // float magz = mag.magnetic.z;

  float press = bmp.pressure / 100.0;
  float alt = bmp.readAltitude(SEALEVELPRESSURE_HPA);
  float board_temperature = (bmp.temperature + bno.getTemp())/2;


  float accelX = accel.x();
  float accelY = accel.y();
  float accelZ = accel.z();

  

  float gyroX = gyro.x();
  float gyroY = gyro.y();
  float gyroZ = gyro.z();

  static float data[9];

  data[0] = accelX;
  data[1] = accelY;
  data[2] = accelZ;
  data[3] = gyroX;
  data[4] = gyroY;
  data[5] = gyroZ;
  data[6] = board_temperature; //magx
  data[7] = press; //magy
  data[8] = alt; //magz
  // data[9] = board_temperature;
  // data[10] = press;
  // data[11] = alt;


	return data; //address of structure member returned
}

void LogData(float accelX, float accelY, float accelZ, float gyroX, float gyroY, float gyroZ, float board_temperature, float press, float alt){
    
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


    /// BIG ERROR - I NEED TO MAKE SURE THE  DATA [ ] ARRAY IS LARGE ENOUGH AND THEN CHECK IF THE SIZE==IDX OR SMTH, THEN I NEED TO FIND A WAY TO DELETE/CLEAR THE ARRAY OR DATA[] = [] or smth
    // nvm update ^^ 10 minutes later - i think my old code solved this issue by just making a super duper big array that can store all the flight's data until apogee or smth... 
    // TODO: i still need to implement the data in the else statement - i think just uncoment the stuff in the else statement and just go from there.....
    // greate
    if(time - startTime > secondsTillApogee*1000 && !Apogee){
      Serial.println("WRITING TO data.csv");
      
      file = SD.open("data.csv", FILE_WRITE);
     
      for(int i=0; i<idxx; i++ ){
        file.println(Data[i]);
        
      }
      file.close();
      Serial.println("WRITING TO data.csv - DONE");
      //delay(10000);
      Apogee = true;
      
    }else if(!Apogee){
      Data[idxx] = data;
      // Serial.println(Data[idxx]);
      // Serial.println(idxx);
      // ax[idxx] = accelX;
      // ay[idxx] = accelY;
      // az[idxx] = accelZ;
      idxx ++;
    }else{
      // file = SD.open("data.csv", FILE_WRITE);
      // file.println(data);
      // file.close();
    
    }
    

    
    
    // long T2 = micros();
    // Serial.println(T2-T1);


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
/////////////
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

float* accel_to_v(){
  //float integrate(int a, int b, float arr[], float dt);
  int a = 0;
  int b = idxx;
  float dt = 0.0001;
  // currently i think dt = 0.006
  //float dt = gimedt();
  
  float vx = ahrs.integrate(a,b,ax,dt);
  float vy = ahrs.integrate(a,b,ay,dt);
  float vz = ahrs.integrate(a,b,az,dt);

  //ahrs.sqrt10(const double number)
  // |v|  = sqrt10(vx*vx + vy*vy + vz*vz);
  static float v[3];
  v[0] = vx;
  v[1] = vy;
  v[2] = vz;

  return v;


}

void update_a_s(){
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  ax[idxx] = accel.x();
  ay[idxx] = accel.y();    
  az[idxx] = accel.z();
  
}



