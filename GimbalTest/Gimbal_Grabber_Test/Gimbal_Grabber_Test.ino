#define GIMBAL_Y_PIN 5
#define GIMBAL_X_PIN 6
#include <Wire.h>
#include <stdint.h>
#include <Servo.h>

struct imuResult
{
  int16_t gyroX = 0, gyroY = 0, gyroZ = 0;
};

Servo gimbalY;
Servo gimbalX;

void setup() {
  // put your setup code here, to run once:
  gimbalY.attach(GIMBAL_Y_PIN);
  gimbalX.attach(GIMBAL_X_PIN);
  Serial.begin(9600); //9600 Baud Rate
  Serial.setTimeout(15);
  initIMU();

}

const int defaultInterval = 50, interval = 4;
int rollingX = defaultInterval, rollingY = defaultInterval;

void loop() {
  // put your main code here, to run repeatedly:
  const int W = 1;
  const int S = 2;
  const int A = 3;
  const int D = 4;

  if(true)
  {
    int code = Serial.parseInt();

    switch(code)
    {
      case W:
        rollingY += interval;
        break;
      case S:
        rollingY -= interval;
        break;
      case A:
        rollingX += interval;
        break;
      case D:
        rollingX -= interval;
        break;
      default:
        break;       
    }

    //Serial.println(code);
    //Serial.print(" ");
    
  }
  setGimbalY(rollingY);
  setGimbalX(rollingX);

  struct imuResult result = readFromIMU();
  Serial.print(result.gyroX);
  Serial.print(" ");
  Serial.print(result.gyroY);
  Serial.print(" ");
  Serial.println(result.gyroZ);
  
  //Serial.flush();  

}

const int SERVO_MIN_MS = 900;
const int SERVO_MAX_MS = 2000;

void setGimbalY(int &percent)
{
  if(percent < 0) {percent = 0;}
  if(percent > 100) {percent = 100;}
  int microseconds = ((SERVO_MAX_MS-SERVO_MIN_MS)*(percent/100.0) + SERVO_MIN_MS);
  #ifdef LOG_GIMBAL_PERCENT
  Serial.print(" Gimbal Y ");
  Serial.print(percent);
  Serial.print(" ");
  Serial.println(microseconds);
  #endif
  

  gimbalY.writeMicroseconds(microseconds);
}

void setGimbalX(int &percent)
{
  if(percent < 0) {percent = 0;}
  if(percent > 100) {percent = 100;}
  int microseconds = ((SERVO_MAX_MS-SERVO_MIN_MS)*(percent/100.0) + SERVO_MIN_MS);
  #ifdef LOG_GIMBAL_PERCENT
  Serial.print(" Gimbal X ");
  Serial.print(percent);
  Serial.print(" ");
  Serial.println(microseconds);
  #endif
  
  gimbalX.writeMicroseconds(microseconds);
}

//IMU Code
#define IMU_ADDR 0x68
#define IMU_INIT 0x6B

#define GYRO_X_H 0x3B
#define GYRO_X_L 0x44
#define GYRO_Y_H 0x45
#define GYRO_Y_L 0x46
#define GYRO_Z_H 0x47
#define GYRO_Z_L 0x48

void initIMU()
{
  Wire.begin();
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(IMU_INIT);
  Wire.endTransmission(true);
}

struct imuResult readFromIMU()
{
  struct imuResult returnStruct;
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(GYRO_X_H);
  Wire.endTransmission(false);
  Wire.requestFrom(IMU_ADDR, 6, true); //We want to get the next 6 bytes. 
  returnStruct.gyroX = Wire.read() << 8 | Wire.read(); //We get the high and low registers and put them in an int with 16 bits
  returnStruct.gyroY = Wire.read() << 8 | Wire.read();
  returnStruct.gyroZ = Wire.read() << 8 | Wire.read();
  
  return returnStruct;
  
} 
