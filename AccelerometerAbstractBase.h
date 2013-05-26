//Copyright (C) 2012 Rémi Bèges
//This file is part of the "Overdrivr/Quadcopter autopilot"
//For conditions of distribution and use, see copyright notice in Config.hpp

#ifndef ACCELEROMETERABSTRACTBASE_H
#define ACCELEROMETERABSTRACTBASE_H

#include "structures.h"
#include <Arduino.h>
#include "SensorAbstractBase.h"

class AccelerometerAbstractBase : public SensorAbstractBase
{
  public :

  virtual vector3f GetAcceleration() const;

  protected :
  
  vector3f m_acceleration;

};

#endif
