#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <rocketry_lib.h>
#include <utility/quat.h>
#include "Adafruit_BMP3XX.h"
#include <Stepper.h>
#include <TimeLib.h>
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

//**motor stuff**
const int stepsPerRevolution = 2038;
Stepper _stepper = Stepper(stepsPerRevolution, 8,9,10,11);
// **end motor stuff**

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
float axx[SIZE];
float ayy[SIZE];
float azz[SIZE];
float altitude[SIZE];


/***************  END APOGEE PRED STUFF***************/

/* BRIAN'S ALGO */



// Global Constants - Do not change
const float R = 8.314462618;     
const float M = 0.0289652;
//-----------------------------------------------//
// Global Evniornment Variables - User Set at Launch Site
const float g = 9.801;       // Graviational Acceleration at launch site (m/s^2)
const float T = 293.15;      // Temperature at launch site (K [C + 273.15])
const float rho = 101325;    // Air pressure at launch site (Pa)

const float density = (rho * M) / (R * T);     // Air density (kg/m^3)

//-----------------------------------------------//
// Global Rocket Characteristic Variables - User Set 
const float m = 0.4711;               // Mass of rocket in coast phase (kg)
const float area_cm2 = 25.88;         // Characterstic area of rocket or frontal area (cm^2)
const float A = area_cm2 * 0.0001;    // Characterstic area of rocket (m^2)
const float cd = 0.435;               // Drag Coefficient, Dimensionless
const float C = 0.5 * cd * A * density;         // Drag constant (for cv^2 term)
// print(C)

//-----------------------------------------------//


// Initial State of Simulation - User Set - Set to immediately after burnout as default
float s0 = 45.36;    // Initial position (m)
float v0 = 95.84;      // Initial velocity (m/s)
float theta0 = 17.98;      // Initial angle from vertical (degrees)
float timestep = 0.02;    // Timestep of simulation (s)
int iters = 700;

float arr[1000];
float sx[1000];
float sy[1000];
float vx[1000];
float vy[1000];
float ax[1000];
float ay[1000];
float t[1000];


// *END BRIANS ALGO /






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
    
}

void loop() {
      quat_init = ahrs.loop_find_quat_init(bno);
      if(!initQuatFound){
        startTime = millis();
        initQuatFound = true;
      }
        //long T1 = micros();
      
      float timeStep = GLOB_DT;
      imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
      imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
        
        
      float* dataPtr;
      dataPtr = GetData(accel,gyro);
      LogData(dataPtr[0],dataPtr[1],dataPtr[2],dataPtr[3],dataPtr[4],dataPtr[5],dataPtr[6],dataPtr[7],dataPtr[8]);
        
      float* v;
      v = accel_to_v();
      float vx = v[0];
      float vy = v[1];
      float vz = v[2];

        //*****logic****
        // imu::Quaternion gyroIntedQuat;
        // if(gyro.magnitude() != 0){
        //   gyroIntedQuat = ahrs.integrateGyro(gyro, accel, quat_init, timeStep);
        // }else{
        //   gyroIntedQuat = quat_init;

          
        // }

      

      imu::Quaternion ASD = bno.getQuat();
      float tileAngleFromSensor = ahrs.tilt(ASD);
        // Serial.println(tileAngleFromSensor);
        // Serial.println(F("~~~~~~~~"));
        // imu::Quaternion quat = gyroIntedQuat;
        // quat_init = quat;
        // float tiltAngleFromMath = ahrs.tilt(quat);
        // Serial.println(tiltAngleFromMath);
        // Serial.println(F("----"));
          
        

       // Brian's Values
      float alt = altitude[idxx];
        
       //long T2 = micros();
      // Serial.println(T2-T1);
      // GLOB_DT = (T2-T1)/1000000;
      float predApog = 0;
      // run it after burnout...
      boolean runLoop = false;
      if(runLoop){
              Apogee_PRED_INIT();
              for (int n=1; n<iters; n++){
              
                  
                if(iterate(n) == false){ 
                        predApog = sy[n];
                        
                }
                
                
              }
                    
      }
 }

    




void BMPinit(){
  if(!bmp.begin_SPI(BMP_CS)){
    Serial.println(F("bmp failed"));
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

  //float press = bmp.pressure / 100.0;
  float press = 6969;
  float alt = bmp.readAltitude(SEALEVELPRESSURE_HPA);
  //float board_temperature = (bmp.temperature + bno.getTemp())/2;
  float board_temperature=bno.getTemp();


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

/*
moves stepper
@param angle
make sure the angle is in radians 
*/
void moveStepper(int rpm, float angle){
  _stepper.setSpeed(rpm);
  float steps_everyrev =  stepsPerRevolution/(2*PI);
  float calc_steps = steps_everyrev * angle;
  _stepper.step(calc_steps);

}





float diff_vx(float vxc, float vyc){
  return -C/m * ahrs.sqrt10(vxc*vxc + vyc*vxc) * vxc;
}

float diff_vy(float vxc, float vyc){
  return -g - (C/m * ahrs.sqrt10(vxc*vxc + vyc*vyc) * vyc);
}

void Apogee_PRED_INIT(){
   float vx0 = sin(theta0*1000 / 57296) * v0;
   float vy0 = cos(theta0*1000 / 57296) * v0;
   float t0 = 0;
   t[0] = t0;
   vx[0] = vx0;
   vy[0]=vy0;
   sx[0]=0;
   sy[0]=s0;
   ax[0]=0;
   ay[0]=0;
}


boolean iterate(int n){
    float vx_dot = diff_vx(vx[n-1], vy[n-1]);
    float vy_dot = diff_vy(vx[n-1], vy[n-1]);
    float delta_vx = vx_dot * timestep;
    float delta_vy = vy_dot * timestep;
    float delta_sx = (2 * vx[n-1] + delta_vx) / 2 * timestep;
    float delta_sy = (2 * vy[n-1] + delta_vy) / 2 * timestep;
    t[n] = t[n-1] + timestep;
    vx[n] = vx[n-1] + delta_vx;
    vy[n] = vy[n-1] + delta_vy;
    sx[n] = sx[n-1] + delta_sx;
    sy[n] = sy[n-1] + delta_sy;
    ax[n] = vx_dot;
    ay[n] = vy_dot;
    
    if(vy[n-1] > 0){
      return true;
    }else{
      return false; 
    }
        
}



