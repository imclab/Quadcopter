#include "IMU.h"
#include "Sensors.h"

IMU::IMU()
{
    eulerAngles.x = 0;
    eulerAngles.y = 0;
    eulerAngles.z = 0;
    
    tau=0.075;
    
    Q_angle  =  0.01; //0.001
    Q_gyro   =  0.0003;  //0.003
    R_angle  =  0.01;  //0.03
    x_bias = 0;
    P_00 = 0;
    P_01 = 0;
    P_10 = 0;
    P_11 = 0;
    
    accFilterConstant = 0.6;
}

IMU::~IMU()
{
  
}

void IMU::Update(vector3f gyroData, vector3f accData)
{
  
  currentTime = millis();
  looptime = currentTime-previousTime;
  previousTime = currentTime;
  
  //On filtre x
  //float accAngleX = - _atan2(Sensor.GetAccData().x,Sensor.GetAccData().z);
  //eulerAngles.x = ComplementaryFilter(accAngleX,gyroData.y,eulerAngles.x,looptime);
  
  //On filtre y
  float accAngleY = - _atan2(accData.y,accData.z);
  eulerAngles.y = ComplementaryFilter(accAngleY,gyroData.x,eulerAngles.y,looptime);
  
  //eulerAngles.y = Sensor.GetGyroAngles().y;
  //float accAngleY = eulerAngles.y*accFilterConstant + _atan2(Sensor.GetAccData().y,Sensor.GetAccData().z)*(accFilterConstant-1);
  //eulerAngles.x = KalmanFilter(accAngleX,Sensor.GetGyroData().x,eulerAngles.x,looptime);
  
  delay(20);
}

vector3f IMU::GetEulerAngles()
{
  return eulerAngles;
}

// a=tau / (tau + loop time)
// newAngle = angle measured with atan2 using the accelerometer
// newRate = angle measured using the gyro
// looptime = loop time in millis()

float IMU::ComplementaryFilter(float accAngle, float gyroData, float previousAngle, float looptime) 
{
  looptime /= 1000.0;
  float a = tau / (tau + looptime);
  return a * (previousAngle + gyroData * looptime) + (1-a) * (accAngle);
}

// newAngle = angle measured with atan2 using the accelerometer
// newRate = angle measured using the gyro
// looptime = loop time in millis()

float IMU::KalmanFilter(float accAngle, float gyroData, float previousAngle, float looptime)
{
  looptime /= 1000.0;
  previousAngle += looptime * (gyroData - x_bias);
  P_00 +=  - looptime * (P_10 + P_01) + Q_angle * looptime;
  P_01 +=  - looptime * P_11;
  P_10 +=  - looptime * P_11;
  P_11 +=  + Q_gyro * looptime;
  
  y = accAngle - previousAngle;
  S = P_00 + R_angle;
  K_0 = P_00 / S;
  K_1 = P_10 / S;
  
  previousAngle +=  K_0 * y;
  x_bias  +=  K_1 * y;
  P_00 -= K_0 * P_00;
  P_01 -= K_0 * P_01;
  P_10 -= K_1 * P_00;
  P_11 -= K_1 * P_01;
  
  return previousAngle;
}

float IMU::_atan2(float y, float x){
  #define fp_is_neg(val) ((((uint8_t*)&val)[3] & 0x80) != 0)
  float z = y / x;
  int16_t zi = abs(int16_t(z * 100)); 
  int8_t y_neg = fp_is_neg(y);
  if ( zi < 100 ){
    if (zi > 10) 
     z = z / (1.0f + 0.28f * z * z);
   if (fp_is_neg(x)) {
     if (y_neg) z -= PI;
     else z += PI;
   }
  } else {
   z = (PI / 2.0f) - z / (z * z + 0.28f);
   if (y_neg) z -= PI;
  }
  z *= (180.0f / PI); 
  return z;
}

IMU imu = IMU();
