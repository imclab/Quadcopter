#include "GyroscopeAbstractBase.h"

vector3f GyroscopeAbstractBase::GetAngularVelocity() const
{
 return m_angularVelocity.top(); 
}

vector3f GyroscopeAbstractBase::GetIntegratedAngles() const
{
 return m_integratedAngles;
}
