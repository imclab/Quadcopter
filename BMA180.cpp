#include "BMA180.h"
#include "Wire.h"

BMA180::BMA180()
{  
  accData.x = 0;
  accData.y = 0;
  accData.z = 0;
  
  accZero.x = 0;
  accZero.y = 0;
  accZero.z = 0;
  
  accGain = 1;
}

BMA180::~BMA180()
{
}

void BMA180::Configure()
{  
  /*----------------- Initialisation des accéléromètres  ---------------------*/
  delay(10);
  //default range 2G: 1G = 4096 unit. On changera cette valeur pour 4 G plus tard
  Wire.beginTransmission(0x40);
  Wire.write(0x0D);
  Wire.write(0x10); // register: ctrl_reg0  -- value: set bit ee_w to 1 to enable writing
  Wire.endTransmission();
  
  //---------------  On indique le registre à lire (tcs)
  Wire.beginTransmission(0x40);
  Wire.write(0x20);
  Wire.endTransmission();
  
  Wire.requestFrom(0x40,1);
  uint8_t control = Wire.read();
  
  control &= 0x0F; // save tcs register
  control |= 0x10; // register: bw_tcs reg: bits 4-7 to set bw -- value: set low pass filter to 20Hz
  
  Wire.beginTransmission(0x40);
  Wire.write(0x20);
  Wire.write(control);
  Wire.endTransmission();
    
  //------------------   On indique le registre à lire (tco_z)
  Wire.beginTransmission(0x40);
  Wire.write(0x30);
  Wire.endTransmission();
  
  Wire.requestFrom(0x40,1);
  control = Wire.read();
  
  control &= 0xFC;        // save tco_z register
  control |= 0x00;        // set mode_config to 0
  
  Wire.beginTransmission(0x40);
  Wire.write(0x30);
  Wire.write(control);
  Wire.endTransmission();
  
  //-------------------  On définit la plage d'accélération à détecter, en l'occurence 4G
  // On doit donc lire le registre 0x35 et changer le paramètre range.
  // On ne touche pas aux autres paramètres du registre (offset_x, smp_skip)
  Wire.beginTransmission(0x40);
  Wire.write(0x35);
  Wire.endTransmission();
  
  Wire.requestFrom(0x40,1);
  control = Wire.read();
  control &= 0xF1;        // save offset_x and smp_skip
  control |= 0x04<<1; // set range to 4G
  Wire.beginTransmission(0x40);
  Wire.write(0x35);
  Wire.write(control);
  Wire.endTransmission();
  
  delay(20);
}

void BMA180::UpdateData(boolean readAcceleration, boolean readTemperature)
{
  if(readAcceleration)
  {
    //Lecture de l'accélérometre
    this->ReadRawAccelerations();
    //Zéro de l'accéléromètre
    accData.x *= accGain;
    accData.y *= accGain;
    accData.z *= accGain;
    
    accData.x += accZero.x;
    accData.y += accZero.y;
    accData.z += accZero.z;
  }
  if(readTemperature)
  {
    this->ReadRawTemperature();
  }
}


boolean BMA180::IsActive()
{
  //On vérifie que le capteur est bien actif en cherchant à lire son id, normalement : 11
  //Pour cela on lit le registre chip_id (2 LSB du registre 0x00)
  
  Wire.beginTransmission(0x40);//Adresse i2c du capteur
  Wire.write(uint8_t(0x00));//Adresse i2c du registre
  Wire.endTransmission();
  
  //On lit 1 octet
  Wire.requestFrom(0x40,1);
  outbuf[0] = Wire.read();
  
  if(outbuf[0] & 0x03 == 0x03)
    return true;
  
  return false;
}

void BMA180::Calibrate()
{
  float sumx = 0, sumy = 0, sumz = 0;
  int nbsamples = 400;
  for(int i(0) ; i < nbsamples ; ++i)
  {
     this->ReadRawAccelerations();
     sumx += accData.x;
     sumy += accData.y;
     sumz += accData.z;
     delay(10);
  }
  
  accZero.x = sumx/nbsamples;
  accZero.y = sumy/nbsamples;
  accZero.z = sumz/nbsamples;
  
  //Au repos, l'acc z doit valoir -1000, et les autres 0, la norme doit donc être de 1000
  float norme = sqrt(accZero.x*accZero.x + accZero.y*accZero.y + accZero.z*accZero.z);
  //On fait donc en sorte que la norme vale 1000
  accGain = 1000.f/norme;
  //au repos, l'acc x et y doivent valoir 0, le z 1000
  accZero.x = - accZero.x * accGain;
  accZero.y = - accZero.y * accGain;
  accZero.z = 1000 - accZero.z * accGain;
}

vector3f BMA180::GetAcceleration()
{
  return accData;
}

int BMA180::GetTemperature()
{
  return temperature;
}


void BMA180::ReadRawAccelerations()
{
  Wire.beginTransmission(0x40);
  Wire.write(0x02);
  Wire.endTransmission();
  
  Wire.requestFrom(0x40,6);
  outbuf[0] = Wire.read();//x lsb
  outbuf[1] = Wire.read();//x msb
  outbuf[2] = Wire.read();//y lsb
  outbuf[3] = Wire.read();//y msb
  outbuf[4] = Wire.read();//z lsb
  outbuf[5] = Wire.read();//z msb
  
  //Avec 4G de resolution, on a 0.5 mG /LSB, on divise donc par 2 pour avoir des valeurs en mg
  accData.x = float(((outbuf[1] << 8) | (outbuf[0]))>>2)/2.f;
  accData.y = float(((outbuf[3] << 8) | (outbuf[2]))>>2)/2.f;
  accData.z = float(((outbuf[5] << 8) | (outbuf[4]))>>2)/2.f;
  
  measuresCount++;
  
}

void BMA180::ReadRawTemperature()
{
  Wire.beginTransmission(0x40);
  Wire.write(0x08);
  Wire.endTransmission();
  
  Wire.requestFrom(0x40,1);
  outbuf[0] = Wire.read();//octet de la temperature

  //temperature en complément à 2
  //Lorsque l'octet vaut 0, on a la temperature médiane 23.5°
  temperature = 23.5 + ((outbuf[0]<<8)>>8)/2.f;
}
