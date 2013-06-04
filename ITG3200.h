#ifndef SENSORS_H
#define SENSORS_H

#include "structures.h"
#include <Arduino.h>
#include "GyroscopeAbstractBase.h"

/// Invensense ITG3200 3 axis I2C Accelerometer

class ITG3200 : public GyroscopeAbstractBase
{
  public :
  ITG3200();
  ~ITG3200();
 
  void Configure();
  void Calibrate();
  boolean IsActive();
  void ProcessData();
  void Read();

  vector3f GetAngularVelocity();
  vector3f GetIntegratedAngles();
  
  /*** Integration runge kutta 4 ***/
  // TODO : A VIRER DE LA
  // options : 1 seule méthode d'appel à l'integrateur (possible faire functor pour conserver état dernière intégration)
  // float Integrator(int data*, int timeIntervals*);
  float RK4Integrate(int data4, int data3, int data2, int data1, int deltaTmillis);
 
  
  private :
  
  //vector3f gyroData[4];
  vector3f m_angularVelocityZero;
  //vector3f gyroAngles;
  
  int previousTime;
  int currentTime;
  int looptime;
  
  //Le buffer de reception
  byte outbuf[6];//declarer static ds methode directement
  
  long int measuresCount;
};

#endif
