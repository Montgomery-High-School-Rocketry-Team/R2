#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <rocketry_lib.h>
#include <utility/quat.h>
#include "Adafruit_BMP3XX.h"
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
const int SIZE = secondsTillApogee * 400;
String* Data = new String[SIZE];
long startTime;
int idxx = 0;
bool Apogee = false;
//in in seconds
float GLOB_DT = 0.0145;
/******************************************* END -----  DATA COLLECTION SET UP GLOBAL VALS **********************************/

/*********************** START ALGO GLOBAL VALUES ***********************/
Stepper stepper = Stepper(2038, 5, 6, 7, 8);
// angle stuff for step motor
int motor_rpm = 10;
float LAST_ANGLE = 0.349066;
bool already_apogee_time_passed = false;

bool already_twenty_rotated = false;
bool already_forty_rotated = false;
bool already_sixty_rotated = false;
bool already_eighty_rotated = false;
bool already_rotated_back = false;

//end angle stuff
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

/*************** END BRIAN STUFF***************/

/*********************** END ALGO GLOBAL VALUES ***********************/

void setup(void) {
  Serial.begin(115200);
  Wire.setClock(1000000);
  //Serial.println("Orientation Sensor Test"); Serial.println("");

  BMPinit();
  ahrs.SDinit(BUILTIN_SDCARD);

  BNOinit();
  bno.restoreDefults();

  quat_init = ahrs.loop_find_quat_init(bno);

  //UNCOMMENT BEFORE LAUNCH
  //ahrs.before_launch_detection(bno,bmp);

  stepper.setSpeed(10);

  startTime = millis();
}

void loop() {

  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);


  float* dataPtr;
  dataPtr = ahrs.GetData(accel, gyro, bno, bmp);

  float* v;
  v = accel_to_v(dataPtr[0], dataPtr[1],dataPtr[2]);
  // float vx = v[0];
  // float vy = v[1];
  // float vz = v[2];

  imu::Quaternion ASD = bno.getQuat();
  float tiltAngleFromSensor = ahrs.tilt(ASD);

  float alt = dataPtr[8];
  //Serial.println(alt);

  LogData(dataPtr[0], dataPtr[1], dataPtr[2], dataPtr[3], dataPtr[4], dataPtr[5], dataPtr[6], dataPtr[7], dataPtr[8], tiltAngleFromSensor);

  

  long time = millis();
  if (!(time - startTime > secondsTillApogee * 1000)) {
    bool ran = false;
    if (!already_twenty_rotated) {

      Data[idxx] = "# START MOTOR ROTATE - 20DEG";
      idxx++;

      Serial.println("start 20");

      ahrs.moveStepper(stepper, 2038, LAST_ANGLE);

      Data[idxx] = "# END MOTOR ROTATE - 20DEG";
      idxx++;
      Serial.println("end 20");
      already_twenty_rotated = true;
      ran = true;

    } else if ( !already_forty_rotated) {

      Data[idxx] = "# START MOTOR ROTATE - 40DEG";
      idxx++;
      Serial.println("start 40");

      ahrs.moveStepper(stepper, 2038, LAST_ANGLE);

      Data[idxx] = "# END MOTOR ROTATE - 40DEG";
      idxx++;
      Serial.println("end 40");
      already_forty_rotated = true;
      ran = true;

    } else if ( !already_sixty_rotated) {

      Data[idxx] = "# START MOTOR ROTATE - 60DEG";
      idxx++;
      Serial.println("start 60");
      ahrs.moveStepper(stepper, 2038, LAST_ANGLE);

      Data[idxx] = "# END MOTOR ROTATE - 60DEG";
      idxx++;
      Serial.println("end 60");
      already_sixty_rotated = true;
      ran = true;

    } else if ( !already_eighty_rotated) {

      Data[idxx] = "# START MOTOR ROTATE - 80DEG";
      idxx++;
      Serial.println("start 80");
      ahrs.moveStepper(stepper, 2038, LAST_ANGLE);

      Data[idxx] = "# END MOTOR ROTATE - 80DEG";
      idxx++;
      Serial.println("end 80");
      already_eighty_rotated = true;
      ran = true;
    }

    if (ran) {
      imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
      imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

      float* dataPtr;
      dataPtr = ahrs.GetData(accel, gyro, bno, bmp);

      imu::Quaternion ASD = bno.getQuat();
      float tiltAngleFromSensor = ahrs.tilt(ASD);

      axx[idxx] = accel.x();
      ayy[idxx] = accel.y();
      azz[idxx] = accel.z();


      long time = millis();
      String data = "";
      data.concat(time);
      data.concat(",");
      data.concat(dataPtr[0]);
      data.concat(",");
      data.concat(dataPtr[1]);
      data.concat(",");
      data.concat(dataPtr[2]);
      data.concat(",");
      data.concat(dataPtr[3]);
      data.concat(",");
      data.concat(dataPtr[4]);
      data.concat(",");
      data.concat(dataPtr[5]);
      data.concat(",");
      data.concat(String(dataPtr[6], 0));
      data.concat(",");
      data.concat(String(dataPtr[7], 0));
      data.concat(",");
      data.concat(String(dataPtr[8], 0));
      data.concat(",");
      data.concat(String(tiltAngleFromSensor, 3));

      Data[idxx] = data;
      idxx++;
    }
  } else {
    //run this loop once like for example retract the air breaks
    if (!already_apogee_time_passed && !already_rotated_back) {
      // retract air breaks
      ahrs.moveStepper(stepper, 2038, -LAST_ANGLE * 4);
      already_rotated_back = true;
    }
  }

}


void LogData(float accelX, float accelY, float accelZ, float gyroX, float gyroY, float gyroZ, float board_temperature, float press, float alt, float angle) {

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
  data.concat(String(board_temperature, 0));
  data.concat(",");
  data.concat(String(press, 2));
  data.concat(",");
  data.concat(String(alt, 2));
  data.concat(",");
  data.concat(String(angle, 3));


  /// BIG ERROR - I NEED TO MAKE SURE THE  DATA [ ] ARRAY IS LARGE ENOUGH AND THEN CHECK IF THE SIZE==IDX OR SMTH, THEN I NEED TO FIND A WAY TO DELETE/CLEAR THE ARRAY OR DATA[] = [] or smth
  // nvm update ^^ 10 minutes later - i think my old code solved this issue by just making a super duper big array that can store all the flight's data until apogee or smth...
  // TODO: i still need to implement the data in the else statement - i think just uncoment the stuff in the else statement and just go from there.....
  // greate
  if (time - startTime > secondsTillApogee * 1000 && !Apogee) {
    //Serial.println("WRITING TO data.csv");

    file = SD.open("data.csv", FILE_WRITE);

    for (int i = 0; i < idxx; i++) {
      file.println(Data[i]);
    }
    file.close();
    // Serial.println("WRITING TO data.csv - DONE");
    //delay(10000);
    Apogee = true;

    // this global dt is found from uncommenting the code under the else statement
    // time it takes to run the void loop
    GLOB_DT = 0.025;

  } else if (!Apogee) {
    Data[idxx] = data;
    // Serial.println(Data[idxx]);
    // Serial.println(idxx);
    
    idxx++;
  } else {

    // file = SD.open("data.csv", FILE_WRITE);
    // file.println(data);
    // file.close();
  }




  // long T2 = micros();
  // Serial.println(T2-T1);
}

void BMPinit() {
  if (!bmp.begin_SPI(BMP_CS)) {
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

void BNOinit() {
  /* Initialise the sensor */
  if (!bno.begin()) {
    /* There was a problem detecting the BNO055 ... check your connections */
    // Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1)
      ;
  }

  //bool foundCalib = false;
  bool sdavailable = false;

  if (SD.begin(BUILTIN_SDCARD)) {
    sdavailable = true;
    Serial.println("card initialized.");
    if (SD.exists("data.dat")) {
      Serial.println("Calibration File found");

      dataFile = SD.open("data.dat", FILE_READ);

      adafruit_bno055_offsets_t calibrationData;
      sensor_t sensor;

      bno.getSensor(&sensor);
      dataFile.read((uint8_t*)&calibrationData, sizeof(calibrationData));

      dataFile.close();
      //  ahrs.displaySensorOffsets(calibrationData);



      // Serial.println("\n\nRestoring Calibration data to the BNO055...");
      bno.setSensorOffsets(calibrationData);

      // Serial.println("\n\nCalibration data loaded into BNO055");
      //foundCalib = true;

      /*---------Check Calibration-------------*/

      //  Serial.println("Checking Offset__________________");
      adafruit_bno055_offsets_t checkOffset;
      bno.getSensorOffsets(checkOffset);


      // ahrs.displaySensorOffsets(checkOffset);
      //  Serial.println("\nChecking Offset end_______________");
      /*--------------------------------------*/
    } else {
      Serial.println("No Calibration File found");
    }
  } else {
    Serial.println("Card failed, or not present");
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

  Serial.println("\nFully calibrated!");
  Serial.println("--------------------------------");
  Serial.println("Calibration Results: ");
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



  dataFile.write((const uint8_t*)&newCalib, sizeof(newCalib));
  dataFile.close();


  // Serial.println("Data stored to SD-Card.");
  // Serial.println("\n--------------------------------\n");
}




float* accel_to_v(float ax,float ay, float az) {
  
  axx[idxx] = ax;
  ayy[idxx] = ay;
  azz[idxx] = az;
  int a = 0;
  int b = idxx;
  float dt = GLOB_DT;

  float vx = ahrs.integrate(a, b, axx, dt);
  float vy = ahrs.integrate(a, b, ayy, dt);
  float vz = ahrs.integrate(a, b, azz, dt);

  //float vv  = ahrs.sqrt10(vx*vx + vy*vy + vz*vz);
  //Serial.println(vv);
  static float v[3];
  v[0] = vx;
  v[1] = vy;
  v[2] = vz;
  // Serial.println(v[0]);
  // Serial.println(v[1]);
  // Serial.println(v[2]);
  // Serial.println("---");
  return v;
}
