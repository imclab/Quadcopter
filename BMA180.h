//BMA180.h - Library for using BMA180 accelerometer
//Copyright (C) 2012 Rémi Bèges
//This file is part of the "Overdrivr/Quadcopter autopilot"
//For conditions of distribution and use, see copyright notice in Config.hpp


#ifndef BMA180_H
#define BMA180_H

#include "structures.h"
#include <Arduino.h>
#include "AccelerometerAbstractBase.h"

/// Bosch BMA180 3 axis I2C accelerometer

class BMA180 : public AccelerometerAbstractBase
{
  public :
  BMA180();
  ~BMA180();
 
  void Configure();
  void Calibrate();
  boolean IsAlive() const;
  void ProcessData();
  void Read();
  
  void ReadTemperature();
  float GetTemperature() const;
    
  private :
  
  //Les coordonnées x et y sont des offsets, le z n'est normalement pas utilisé
  vector3f m_accelerationZero;
  float m_accelerationGain;
  
  float m_temperature;
  
  //La gestion du temps
  int previousTime;
  int currentTime;
  int loopTime;

};

#endif
