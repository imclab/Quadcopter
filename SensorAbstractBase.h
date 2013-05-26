//Copyright (C) 2012 Rémi Bèges
//This file is part of the "Overdrivr/Quadcopter autopilot"
//For conditions of distribution and use, see copyright notice in Config.hpp


#ifndef SENSORABSTRACTBASE_H
#define SENSORABSTRACTBASE_H

#include "structures.h"
#include <Arduino.h>

class SensorAbstractBase
{
  public :

  virtual void Configure() = 0;
  virtual void Calibrate() = 0;
  virtual long int Get ReadingsCount() const;
  virtual boolean IsAlive() = 0 const;
  virtual void ProcessData() = 0;
  virtual void Read() = 0;

  private :

  long int readingsCount;

};

#endif
