#include <Wire.h>
#include "structures.h"
#include "Sensors.h"
#include "IMU.h"

void setup()
{
  Serial.begin(115200);
  Wire.begin();
  Sensor.Init();
}

void loop()
{
  vector3f gyrodata;
  vector3f eulerAngles;
  vector3f accdata;
  
  //1 - On récupère les valeurs des capteurs
  Sensor.UpdateData();
  
  //La valeur de l'accéléromètre orienté verticalement au repos doit être 1000 (=1G)
  //Les valeurs des accéléromètres sont en mg
  
  //Les valeurs des gyroscopes au repos doivent être 0
  //Les valeurs des gyros doivent être en degrees/s
  gyrodata = Sensor.GetGyroData();
  accdata = Sensor.GetAccData();
  
  //2 - On met à jour les valeurs calculées d'angle
  imu.Update(gyrodata,accdata);
  
  //3 - On récupère les valeurs des commandes provenant de la télécommande
  
  //4 - On calcule le terme d'erreur, et on pondère le calcul du mixage avec les valeurs des commandes
  
  //5 - On envoi les valeurs aux moteurs
  
  
  
  eulerAngles = imu.GetEulerAngles();
  
  
  Serial.print( gyrodata.x);
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
  delay(10);
  
}
