#include <Wire.h>
#include "structures.h"
#include "BMA180.h"
#include "ITG3200.h"
#include "IMU.h"

BMA180 accelerometer;
ITG3200 gyroscope;
IMU imu;

void setup()
{
  Serial.begin(115200);
  Wire.begin();
  
  accelerometer.Configure();
  //accelerometer.Calibrate();
  
  gyroscope.Configure();
  gyroscope.Calibrate();
}

void loop()
{
  vector3f gyrodata;
  vector3f eulerAngles;
  vector3f accdata;
  vector3f velocity;
  
  //1 - On récupère les valeurs des capteurs
  accelerometer.Read();
  gyroscope.UpdateData();
  
  //2 - Nettoyage des données
  accelerometer.ProcessData();
  
  //La valeur de l'accéléromètre orienté verticalement au repos doit être 1000 (=1G)
  //Les valeurs des accéléromètres sont en mg
  accdata = accelerometer.GetAcceleration();
  //Les valeurs des gyroscopes au repos doivent être 0
  //Les valeurs des gyros doivent être en degrees/s
  gyrodata = gyroscope.GetGyroData();
    
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
