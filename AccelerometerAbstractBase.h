//Copyright (C) 2012 Rémi Bèges
//This file is part of the "Overdrivr/Quadcopter autopilot"
//For conditions of distribution and use, see copyright notice in Config.hpp

#ifndef ACCELEROMETERABSTRACTBASE_H
#define ACCELEROMETERABSTRACTBASE_H

#include "structures.h"
#include <Arduino.h>
#include "SensorAbstractBase.h"

/// Base class for Accelerometer sensor
/// Inherits Sensor Base class
/// Should be inherited by any Accelerometer implementation, 
/// and the following inherited member functions should be implemented in that inherited class :
/// - void Configure();
/// - void Calibrate();
/// - long int GetReadingsCount() const;
/// - boolean IsAlive() const;
/// - void ProcessData();
/// - void Read();
/// - vector3f GetAcceleration() const;

/// This way the standard IMU class provided in this library
/// can work with any accelerometer, regardless of internal implementation details

class AccelerometerAbstractBase : public SensorAbstractBase
{
  public :

  virtual vector3f GetAcceleration() const;

  protected :
  
  vector3f m_acceleration;

};

#endif
