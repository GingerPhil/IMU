

//IMU
//MPU-9150
//28-09-2017
//
//Main file

#include <Wire.h>
#include <Servo.h>
#include "config.h"

//****************************************CONSTANTS****************************************
//IMU constants
const int MPU = 0x68;  //I2C address of the MPU9150
const float gyroScaleFactor = 16.4; //sensitivity scale factor from datasheet 0-131 1-65.6 2-32.8 3-16.4
const int acclScaleFactor = 2048; //sensitivity scale factor from datasheet 0-16384 1-8192 2-4096 3-2048

//****************************************VARIABLES****************************************
//timing variables
unsigned long loopTime; //TESTING
unsigned long loopStartTime; //TESTING

//mode variables
int mode;

//IMU calc variables
unsigned long pastMicros = 0, currentMicros = 0; //time since program started in us
float dt; //change in time for calculations

int16_t acclX, acclY, acclZ, IMUtemperature, gyroX, gyroY, gyroZ; //16 byte variables for accelerometer,gyro and temperature readings

double acclAngleX = 0, acclAngleY = 0, gyroRateX, gyroRateY, gyroAngleX = 0, gyroAngleY = 0; //variables for angles and rates
float currentPitchAngle = 0, currentRollAngle = 0; //variables for the final angles

float pitchAngle, elevatorOutput, pitchSetpoint; //pitch input to PID, output of PID, setpoint for PID
float pitchErrorSum, pitchErrorLast; //pitch sum for integral, last error for dirivative

float rollAngle, alrnOutput, rollSetpoint; //roll 
float rollErrorSum, rollErrorLast; //roll

float heading, rudderOutput, headingSetpoint; //heading
float headingErrorSum, headingErrorLast; //heading

//servo variables
Servo servo;

//****************************************SETUP****************************************
//setup function
void setup()
{
  //start serial for debug
  Serial.begin(250000);

  //start serial for GPS
  Serial1.begin(9600);

  //start serial for telemetry
  Serial2.begin(9600);
  Serial2.println("Telemetry OK");
  
  //set up ports

  //set up GPS and wait for fix


  //set upI2C for MPU
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B); //PWR_MGMT_1 register
  Wire.write(0); //set to zero, turn off sleep mode
  Wire.endTransmission(true);
  
  Wire.beginTransmission(MPU);
  Wire.write(0x1B); //GYRO_CONFIG register FS_SEL
  Wire.write(B00011000); //change range of gyro dps 0-250 1-500 2-1000 3-2000
  Wire.endTransmission(true);
  Wire.beginTransmission(MPU);
  Wire.write(0x1C); //ACCL_CONFIG register AFS_SEL
  Wire.write(B00011000); //change range of gyro dps 0-2 1-4 2-8 3-16
  Wire.endTransmission(true);


  //calibrate IMU using averaged accl value

  //get current angle
  

  Serial.println("Setup OK");
}

//****************************************FUNCTIONS****************************************
//read raw IMU values
void rawInert()
{
  //setup I2C
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); //start with register 0x3B (ACCEL_XOUT_H)
  if(Wire.endTransmission(false) == 0) //return 0 = no faults
  {
    Wire.requestFrom(MPU, 14, true); //14 registers - LOCKS HERE IF GPS CONNECTED

    //get accl and gyro values
    acclX = Wire.read() <<8|Wire.read(); //0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
    acclY = Wire.read() <<8|Wire.read(); //0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    acclZ = Wire.read() <<8|Wire.read(); //0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    IMUtemperature = Wire.read() <<8|Wire.read(); //0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    gyroX = Wire.read() <<8|Wire.read(); //0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    gyroY = Wire.read() <<8|Wire.read(); //0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    gyroZ = Wire.read() <<8|Wire.read(); //0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  
    IMUtemperature = IMUtemperature/340.00+36.53; //temperature in degC from datasheet
  }
  
  //print to serial - comment out
  //Serial.print("AcclX = "); Serial.print(acclX);
  //Serial.print(" | AcclY = "); Serial.print(acclY);
  //Serial.print(" | AcclZ = "); Serial.print(acclZ);
  //Serial.print(" | Temp = "); Serial.print(IMUtemperature);
  //Serial.print(" | GyroX = "); Serial.print(gyroX);
  //Serial.print(" | GyroY = "); Serial.print(gyroY);
  //Serial.print(" | GyroZ = "); Serial.println(gyroZ);
}


//calculate accelerometer angles in degrees
void acclAngles()
{
  //replace with atan2
  acclAngleX = RAD_TO_DEG * atan((float)acclY / sqrt(pow((float)acclZ, 2) + pow((float)acclX, 2))); //convert to angles in X plane
  acclAngleY = RAD_TO_DEG * atan((float) - acclX / sqrt(pow((float)acclZ, 2) + pow((float)acclY, 2))); //convert to angle in Y plane

  //correct range to 360deg
  if(acclZ < 0)
  {
    if(acclAngleX < 0)
      acclAngleX = -180 - acclAngleX;
    else
      acclAngleX = 180 - acclAngleX;
    
    if(acclAngleY < 0)
      acclAngleY = -180 - acclAngleY;
    else
      acclAngleY = 180 - acclAngleY;
  }

  
  //print to serial - comment out
  //Serial.print("X accl angle "); Serial.println(acclAngleX);
  //Serial.print("Y accl angle "); Serial.println(acclAngleY);
}


//get gyro rates in degrees per second
void gyroRates()
{
  gyroRateX = (float)gyroX / gyroScaleFactor; //calculate gyro rate in X plane
  gyroRateY = (float)gyroY / gyroScaleFactor; //calculate gyro rate in Y plane
  
  //print to serial - comment out
  //Serial.print("X gyro rate "); Serial.println(gyroRateX);
  //Serial.print("Y gyro rate "); Serial.println(gyroRateY);
}


//get current angle using gyro rate
void gyroAngles()
{
  //get gyro rates
  gyroRates();
   
  pastMicros = currentMicros; //time since program started for last loop
  currentMicros = micros(); //find time since program started

  //wrap micros()
  if (currentMicros > pastMicros) //micros overflows after ~70 minutes
    dt = (float)(currentMicros - pastMicros)/1000000;
  else
    dt = (float)((4294967295 - pastMicros) + currentMicros)/1000000;

  //integrate gyro rate
  gyroAngleX += (gyroRateX * dt);
  if (gyroAngleX > 360) //wrap around
    gyroAngleX -= 360;
    
  gyroAngleY += (gyroRateY * dt);
  if (gyroAngleY > 360) //wrap around
    gyroAngleY -= 360;

  //print to serial - comment out
  //Serial.print("dt "); Serial.println(dt);
  //Serial.print("gyro X angle "); Serial.println(gyroAngleX);
  //Serial.print("gyro Y angle "); Serial.println(gyroAngleY);
}


void complimentaryFilter()
{

  //calcuate current angle - integrate gyro
  currentPitchAngle += gyroRateX * dt;
  currentRollAngle += gyroRateY * dt;

  //check G value
  int forceMagnitude;
  forceMagnitude = abs(acclX) + abs(acclY) + abs(acclZ);

  //factor in accl when stable (0.5 - 2G)
  if (forceMagnitude >= (float)acclScaleFactor/2 && forceMagnitude <= (float)acclScaleFactor*2)
  {
    currentPitchAngle = (0.98 * currentPitchAngle) + (0.02 * acclAngleX); //X filter: factor in accl
    currentRollAngle = (0.98 * currentRollAngle) + (0.02 * acclAngleY); //Y filter: factor in accl
  } 
}


//calculate the current angles from gyro and accelerometer using complementary filter
void currentAngles()
{
  acclAngles(); //get angles from accelerometer
  gyroAngles(); //get angles from gyroscope

  //filter pitch and roll - store globally
  complimentaryFilter();
  
  
  //store fault if NAN
  
  //print to serial - TESTING
  //Serial.print("CURRENT ANGLE X "); Serial.println(currentPitchAngle);
  //Serial.print("CURRENT ANGLE Y "); Serial.println(currentRollAngle);
  //Serial.println("");

  //Serial.print(gyroAngleX); Serial.print(",");
  //Serial.print(acclAngleX); Serial.print(",");
  Serial.print(currentPitchAngle); Serial.print(",");
}

//****************************************MAIN LOOP****************************************
void loop()
{
  //inertial measurements
  rawInert(); //update raw values from MPU  
  currentAngles(); //calc and filter pitch/roll angles - stored globally

  //FOR TESTING
  //Serial.print("Loop Time: ");
  //Serial.println(loopTime);

  //force a 5ms loop (-8us)
  while(micros()-loopStartTime <= 4992)
  {}

  loopTime = micros() - loopStartTime; //calc loop time - TESTING
  loopStartTime = micros(); //find time loop start
}

