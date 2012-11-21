#include "IMU.h"

IMU::IMU()
{
    eulerAngles.x = 0;
    eulerAngles.y = 0;
    eulerAngles.z = 0;
    
    velocity.x = 0;
    velocity.y = 0;
    velocity.z = 0;
    
    speedloop_time1 = millis();
    speedloop_time2 = speedloop_time1;
    
    
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
  
  float sign = 1;
  if(accData.z < 0.f)
    sign = -1;
    
  //Calcul du roll - on utilise un z normalisé pour éliminer l'influence du pitch
  //En effet si l'on augmente le pitch, z va diminuer, y (du roll) va rester constant
  //et atan(y,z) va varier alors qu'il ne devrait pas
  //Pour cela on "normalise" le z avec la valeur du x 
  //La normalisation ne conservant pas le signe de z, on le rajoute manuellement
  
  float accAngleX = - _atan2(accData.y,sign * sqrt(accData.z*accData.z + accData.x*accData.x));
  eulerAngles.x = accAngleX;//ComplementaryFilter(accAngleX,gyroData.y,eulerAngles.x,looptime);
  
  //Calcul du pitch
  float accAngleY = - _atan2(accData.x,sign * sqrt(accData.z*accData.z + accData.y*accData.y));
  eulerAngles.y = accAngleY;//ComplementaryFilter(accAngleY,gyroData.x,eulerAngles.y,looptime);
  
  //eulerAngles.y = Sensor.GetGyroAngles().y;
  //float accAngleY = eulerAngles.y*accFilterConstant + _atan2(Sensor.GetAccData().y,Sensor.GetAccData().z)*(accFilterConstant-1);
  //eulerAngles.x = KalmanFilter(accAngleX,Sensor.GetGyroData().x,eulerAngles.x,looptime);
  
}

void IMU::ComputeSpeed(vector3f accData)
{
  accDataNoG[1].x = accDataNoG[0].x;
  accDataNoG[1].y = accDataNoG[0].y;
  accDataNoG[1].z = accDataNoG[0].z;
  
  //On calcule les valeurs théoriques produites par la gravité sur l'accéléromètre
  vector3f accGravity;
  //Rappel : G = 1000.f
  accGravity.x = sin(1000.f * eulerAngles.y);
  accGravity.y = sin(1000.f * eulerAngles.x);
  accGravity.z = sqrt(1000000 - accGravity.x*accGravity.x - accGravity.y*accGravity.y);
  
  //On élimine la gravité avec les valeurs d'angles d'euler
  accDataNoG[0].x = accData.x - accGravity.x;
  accDataNoG[0].y = accData.y - accGravity.y;
  accDataNoG[0].z = accData.z - accGravity.z;
  
  //On intègre les valeurs
  speedloop_time1 = millis();
  speedloop_time = speedloop_time1 - speedloop_time2;
  
  velocity.x += (accDataNoG[0].x - accDataNoG[1].x)*speedloop_time/1000.f;
  velocity.y += (accDataNoG[0].y - accDataNoG[1].y)*speedloop_time/1000.f;
  velocity.z += (accDataNoG[0].z - accDataNoG[1].z)*speedloop_time/1000.f;
}


vector3f IMU::GetVelocity()
{
  //return velocity;
  return accDataNoG[0];
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
  
  if ( zi < 100 )
  {
    if (zi > 10)
       z = z / (1.0f + 0.28f * z * z);
       
    if (fp_is_neg(x)) 
    {
     if (y_neg)
       z -= PI;
     else
       z += PI;
    }
  } 
  else 
  {
   z = (PI / 2.0f) - z / (z * z + 0.28f);
   
   if (y_neg) 
     z -= PI;
  }
  
  z *= (180.0f / PI); 
  return z;
}
