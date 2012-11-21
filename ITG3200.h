#ifndef SENSORS_H
#define SENSORS_H

#include "structures.h"
#include <Arduino.h>

class ITG3200
{
  public :
  ITG3200();
  ~ITG3200();
  /*** Initialise les capteurs ***/
  void Configure();
  /*** Pour calibrer le gyro ***/
  void Calibrate();
  /*** Pour tester si les capteurs sont actifs ***/
  byte IsActive();
  
  /*** A appeller à chaque loop pour maj les données ***/
  void UpdateData();
  
  /*** Renvoie la dernière valeur connue des données du gyro ***/
  vector3f GetGyroData();
  /*** Renvoie la valeur d'angle produite par intégration ***/
  vector3f GetGyroAngles();
  
  /*** Integration runge kutta 4 ***/
  float RK4Integrate(int data4, int data3, int data2, int data1, int deltaTmillis);
  
  //Pour lire les valeurs brutes du gyro
  void ReadRawAngularRotation();
  
  private :
  
  vector3f gyroData[4];
  vector3f gyroZero;
  vector3f gyroAngles;
  
  int previousTime;
  int currentTime;
  int looptime;
  
  //Le buffer de reception
  byte outbuf[6];
  
  long int measuresCount;
};

#endif
