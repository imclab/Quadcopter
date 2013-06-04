//Copyright (C) 2012 Rémi Bèges
//This file is part of the "Overdrivr/Quadcopter autopilot"
//For conditions of distribution and use, see copyright notice in Config.hpp

#ifndef GYROSCOPEABSTRACTBASE_H
#define GYROSCOPEABSTRACTBASE_H

#include "structures.h"
#include <Arduino.h>
#include "SensorAbstractBase.h"

/// Base class for any gyroscope sensor manager
/// Inherits Sensor Base class
/// Should be inherited by any gyroscope implementation, 
/// and the following inherited member functions should be implemented in that inherited class :
/// - void Configure();
/// - void Calibrate();
/// - long int GetReadingsCount() const;
/// - boolean IsAlive() const;
/// - void ProcessData();
/// - void Read();
/// - vector3f GetAngularRotation() const;

/// This way the standard IMU class provided in this library
/// can work with any accelerometer, regardless of internal implementation details

class GyroscopeAbstractBase : public SensorAbstractBase
{
  public :
  
  // Unit : deg/s
  virtual vector3f GetAngularRotation() const;

  protected :
  
  vector3f m_angularRotation;

};

#endif
