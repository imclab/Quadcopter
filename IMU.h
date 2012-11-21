#ifndef IMU_H
#define IMU_H

#include "structures.h"
#include <Arduino.h>

class IMU
{
  public:
    IMU();
    ~IMU();
    
    /*** A appeler à chaque loop ***/
    void Update(vector3f gyroData, vector3f accData);
    /*** Calcule la vitesse selon les 3 axes ***/
    void ComputeSpeed(vector3f accData);
    /*** Retourne les valeurs d'angle d'euler ***/
    vector3f GetEulerAngles();
    /*** Renvoie la vitesse actuelle selon les 3 axes ***/
    vector3f GetVelocity();
    
    /*** Kalman Filtering ***/
    float KalmanFilter(float accAngle, float gyroData, float previousAngle, float looptime);
    /*** Complementary filter ***/
    float ComplementaryFilter(float accAngle, float gyroData, float previousAngle, float looptime);
    
    float _atan2(float x, float y);
    
    //Conversion acc to euler
    //vector3f accToEuler(vector3f a);
  private:
    //Valeurs finales d'angle
    vector3f eulerAngles;
    
    int previousTime;
    int currentTime;
    int looptime;
    
    float tau;
    
    float accFilterConstant;
    
    //Pour calculer la vitesse
     vector3f velocity;
     vector3f accDataNoG[2];
     int speedloop_time1;
     int speedloop_time2;
     int speedloop_time;
  
  
  /*
  //Données du gyro [plus recente au rang 0]
  angles gdata[4];
  
  //Données de l'acc
  vector3f adata;
  vector3f preAdata;
  vector3f accZero;
  
  
  //Valeurs d'angle produites par l'acc
  
  vector3f preAccAngles;
  float rotationsCount;
  //Valeurs d'angle produites par le gyro
  vector3f gyroAngles;

  //Coefficient du filtre passe bas de l'accélérometre
  float AccFilterCoeff;
  
  //Complementary filter
  float a;
  float tau;*/
  
  // KasBot V1 - Kalman filter module
  float Q_angle;
  float Q_gyro;
  float R_angle;
  
  float x_bias;
  float P_00, P_01, P_10, P_11;
  float  y, S;
  float K_0, K_1;
};

#endif
