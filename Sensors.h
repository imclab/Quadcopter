#ifndef SENSORS_H
#define SENSORS_H

#include "structures.h"
#include <Arduino.h>

class Sensors
{
  public :
  Sensors();
  ~Sensors();
  /*** Initialise les capteurs ***/
  void Init();
  /*** Pour calibrer le gyro ***/
  void CalibrateGyro();
  /*** Pour calibrer l'accelerometre ***/
  void CalibrateAcc();
  /*** Pour tester si les capteurs sont actifs ***/
  byte IsActive();
  
  /*** A appeller à chaque loop pour maj les données ***/
  void UpdateData();
  
  /*** Renvoie la dernière valeur connue des données du gyro ***/
  vector3f GetGyroData();
  /*** Renvoie la valeur d'angle produite par intégration ***/
  vector3f GetGyroAngles();
  /*** Renvoie la dernière valeur connue des données de l'accelero ***/
  vector3f GetAccData();
  
  /*** Integration runge kutta 4 ***/
  float RK4Integrate(int data4, int data3, int data2, int data1, int deltaTmillis);
  
  
  
  private :
  //Pour lire les valeurs du gyro
  void ReadGyros();
  //Pour lire les données de l'acc
  void ReadAccs();
  
  vector3f gyroData[4];
  vector3f gyroZero;
  vector3f gyroAngles;
  
  //Les coordonnées x et y sont des offsets, le z n'est normalement pas utilisé
  vector3f accZero;
  float accGain;
  
  vector3f accData;
  
  int previousTime;
  int currentTime;
  int looptime;
  
  //Le buffer de reception
  byte outbuf[6];
  
  long int measuresCountGyro;
  long int measuresCountAcc;

};

extern Sensors Sensor;

#endif
