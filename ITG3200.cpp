#include "ITG3200.h"
#include "Wire.h"

ITG3200::ITG3200()
{
  for(int i(0) ; i < 4 ; ++i)
  {
  gyroData[i].x = 0;
  gyroData[i].y = 0;
  gyroData[i].z = 0;
  }
  gyroZero.x = 0;
  gyroZero.y = 0;
  gyroZero.z = 0;
  gyroAngles.x = 0;
  gyroAngles.y = 0;
  gyroAngles.z = 0;
  
  currentTime = millis();
  previousTime = currentTime;
}

ITG3200::~ITG3200()
{
}

void ITG3200::Configure()
{
  /******************  Initialisation du gyro  *******************/
  //1) Reset
  //2) setup filtre passe bas
  //3) referencement de la pll
  
  //Adresse d'ecriture du gyro
  Wire.beginTransmission(0x69);
  //On indique l'adresse du registre Power Management
  Wire.write(0x3E);
  //On met le bit 8 à 1 pour resetter le gyro
  Wire.write(0x80);
  Wire.endTransmission();
   
  Wire.beginTransmission(0x69);
  //On indique l'adresse du registre DLPF
  Wire.write(0x16);
  //On règle le filtre passe bas à 20 Hz (DLPF_CFG = 4)
  Wire.write(0x19);
  Wire.endTransmission();
  
  Wire.beginTransmission(0x69);
  //Registre Power Management
  Wire.write(0x3E);
  //La PLL est référencée sur l'axe Z du gyro (+ stable d'après Invensense)
  Wire.write(0x03);
  Wire.endTransmission();
  
  delay(20);
}

void ITG3200::UpdateData()
{
  //preAdata = adata;
  //preAccAngles = accAngles;
  currentTime = millis();
  looptime = currentTime-previousTime;
  previousTime = currentTime;
  //Lecture du Gyro
  this->ReadRawAngularRotation();
  //Zéro du gyro
  gyroData[0].x -= gyroZero.x;
  gyroData[0].y -= gyroZero.y;
  gyroData[0].z -= gyroZero.z;
  //Integration
  gyroAngles.x += RK4Integrate(gyroData[3].x, gyroData[2].x, gyroData[1].x, gyroData[0].x, looptime);
  gyroAngles.y += RK4Integrate(gyroData[3].y, gyroData[2].y, gyroData[1].y, gyroData[0].y, looptime);
  gyroAngles.z += RK4Integrate(gyroData[3].z, gyroData[2].z, gyroData[1].z, gyroData[0].z, looptime);
}

byte ITG3200::IsActive()
{
  
  //adresse du port i2c en lecture
  Wire.beginTransmission(0x69);
  //On set l'adresse du registre WHO_AM_I
  Wire.write(uint8_t(0x00));
  Wire.endTransmission();
  
  //On lit 1 octet depuis l'adresse 0x68
  Wire.requestFrom(0x69,1);
  outbuf[0] = Wire.read();
  //Serial.println(outbuf[0],HEX);
  
  return outbuf[0];
}

void ITG3200::Calibrate()
{
  
  long int sumx = 0, sumy = 0, sumz = 0;
  int nbsamples = 100;
  for(int i(0) ; i < nbsamples ; ++i)
  {
     this->ReadRawAngularRotation();
     sumx += gyroData[0].x;
     sumy += gyroData[0].y;
     sumz += gyroData[0].z;
     delay(10);
  }
  
  gyroZero.x = sumx/nbsamples;
  gyroZero.y = sumy/nbsamples;
  gyroZero.z = sumz/nbsamples;
  
  Serial.print("Cal Gyr :");
  Serial.print(gyroZero.x);
  Serial.print(",");
  Serial.print(gyroZero.y);
  Serial.print(",");
  Serial.println(gyroZero.z);
}

vector3f ITG3200::GetGyroData()
{
  return gyroData[0];
}

vector3f ITG3200::GetGyroAngles()
{
  return gyroAngles;
}

float ITG3200::RK4Integrate(int data4, int data3, int data2, int data1, int deltaTmillis)
{
  float area = (((data4+2*data3+2*data2+data1)/6)*deltaTmillis/1000.f);
  return area;
}

void ITG3200::ReadRawAngularRotation()
{
  TWBR = ((16000000L / 400000L) - 16) / 2; // change the I2C clock rate to 400kHz
  //Gyro en lecture
  Wire.beginTransmission(0x69);
  //Adresse de debut des donnees gyro
  Wire.write(0x1D);
  Wire.endTransmission();
  
  //On demande 6 octets au gyro
  Wire.requestFrom(0x69,6);
  outbuf[0] = Wire.read();
  outbuf[1] = Wire.read();
  outbuf[2] = Wire.read();
  outbuf[3] = Wire.read();
  outbuf[4] = Wire.read();
  outbuf[5] = Wire.read();
  
  gyroData[3].x = gyroData[2].x;
  gyroData[3].y = gyroData[2].y;
  gyroData[3].z = gyroData[2].z;
  
  gyroData[2].x = gyroData[1].x;
  gyroData[2].y = gyroData[1].y;
  gyroData[2].z = gyroData[1].z;
  
  gyroData[1].x = gyroData[0].x;
  gyroData[1].y = gyroData[0].y;
  gyroData[1].z = gyroData[0].z;
  
  //ERREUR MULTIWII ICI ?
  //16 bits -> +- 32768 -> -+ 2000°/s
  gyroData[0].x =  ((outbuf[0] << 8) | outbuf[1])/14;
  gyroData[0].y = -((outbuf[2] << 8) | outbuf[3])/14;
  gyroData[0].z =  ((outbuf[4] << 8) | outbuf[5])/14;
  
  measuresCount++;
  
}
