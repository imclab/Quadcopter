#include <Wire.h>
#include "structures.h"
#include "BMA180.h"
#include "ITG3200.h"
#include "IMU.h"

BMA180 accelerometers;
ITG3200 gyroscopes;
IMU imu;

void setup()
{
  Serial.begin(115200);
  Wire.begin();
  
  accelerometers.Configure();
  //accelerometers.Calibrate();
  
  gyroscopes.Configure();
  gyroscopes.Calibrate();
}

void loop()
{
  vector3f gyrodata;
  vector3f eulerAngles;
  vector3f accdata;
  vector3f velocity;
  
  //1 - On récupère les valeurs des capteurs
  accelerometers.UpdateData(true,false);
  gyroscopes.UpdateData();
  
  //La valeur de l'accéléromètre orienté verticalement au repos doit être 1000 (=1G)
  //Les valeurs des accéléromètres sont en mg
  accdata = accelerometers.GetAcceleration();
  //Les valeurs des gyroscopes au repos doivent être 0
  //Les valeurs des gyros doivent être en degrees/s
  gyrodata = gyroscopes.GetGyroData();
    
  //2 - On met à jour les valeurs calculées par la centrale inertielle
  imu.Update(gyrodata,accdata);
  //imu.ComputeSpeed(accdata);
  
  //3 - On récupère les valeurs des commandes provenant de la télécommande
  
  //4 - On calcule le terme d'erreur, et on pondère le calcul du mixage avec les valeurs des commandes
  
  //5 - On envoi les valeurs aux moteurs
  
  
  
  eulerAngles = imu.GetEulerAngles();
  velocity = imu.GetVelocity();
  /*
  Serial.print(velocity.x);
  Serial.print(",");
  Serial.print(velocity.y);
  Serial.print(",");
  Serial.print(velocity.z);*/

  Serial.print(gyrodata.x);
  Serial.print(",");
  Serial.print(gyrodata.y);
  Serial.print(",");
  Serial.print(gyrodata.z);
  Serial.print(",");
  Serial.print(accdata.x);
  Serial.print(",");
  Serial.print(accdata.y);
  Serial.print(",");
  Serial.print(accdata.z);
  Serial.print(",");
  Serial.print(eulerAngles.x);
  Serial.print(",");
  Serial.print(eulerAngles.y);
  Serial.print(",");
  Serial.println(eulerAngles.z);
  delay(30);
  
}
