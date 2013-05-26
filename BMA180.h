//BMA180.h - Library for using BMA180 accelerometer
//Copyright (C) 2012 Rémi Bèges
//This file is part of the "Overdrivr/Quadcopter autopilot"
//For conditions of distribution and use, see copyright notice in Config.hpp


#ifndef BMA180_H
#define BMA180_H

#include "structures.h"
#include <Arduino.h>
#include "AccelerometerAbstractBase.h"

class BMA180 : public SensorAbstractBase
{
  public :
  BMA180();
  ~BMA180();
 
  void Configure();
  void Calibrate();
  boolean IsAlive() const;
  void ProcessData();
  void Read();
  
  /*** Renvoie les dernières valeurs connues d'accélération ***/
  vector3f GetAcceleration();
  /*** Renvoie la température du capteur ***/
  int GetTemperature();
  /*** Pour lire les données brutes d'accélération ***/
  void ReadRawAccelerations();
  /*** Pour lire la temperature ***/
  void ReadRawTemperature();
  
  private :
  
  //Les coordonnées x et y sont des offsets, le z n'est normalement pas utilisé
  vector3f accZero;
  float accGain;
  
  //Les données utiles
  vector3f accData;
  float temperature;
  
  //La gestion du temps
  int previousTime;
  int currentTime;
  int loopTime;
    
  //Le buffer de reception
  byte outbuf[6];

  long int measuresCount;

};

#endif
