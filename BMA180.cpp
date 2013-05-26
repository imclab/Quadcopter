#include "BMA180.h"
#include "Wire.h"

BMA180::BMA180()
{  
  m_acceleration.x = 0;
  m_acceleration.y = 0;
  m_acceleration.z = 0;
  
  m_accelerationZero.x = 0;
  m_accelerationZero.y = 0;
  m_accelerationZero.z = 0;
  
  m_accelerationGain = 1;
}

BMA180::~BMA180()
{
}


void BMA180::Calibrate()
{
  float sumx = 0, sumy = 0, sumz = 0;
  int nbsamples = 400;
  for(int i(0) ; i < nbsamples ; ++i)
  {
     Read();
     sumx += m_acceleration.x;
     sumy += m_acceleration.y;
     sumz += m_acceleration.z;
     delay(10);
  }
  
  m_accelerationZero.x = sumx / nbsamples;
  m_accelerationZero.y = sumy / nbsamples;
  m_accelerationZero.z = sumz / nbsamples;
  
  //Au repos, l'acc z doit valoir -1000, et les autres 0, la norme doit donc être de 1000
  float norme = sqrt(m_accelerationZero.x * m_accelerationZero.x + 
                     m_accelerationZero.y * m_accelerationZero.y + 
                     m_accelerationZero.z * m_accelerationZero.z);
                     
  //On fait donc en sorte que la norme vale 1000
  m_accelerationGain = 1000.f / norme;
  //au repos, l'acc x et y doivent valoir 0, le z 1000
  m_accelerationZero.x =      - m_accelerationZero.x * m_accelerationGain;
  m_accelerationZero.y =      - m_accelerationZero.y * m_accelerationGain;
  m_accelerationZero.z = 1000 - m_accelerationZero.z * m_accelerationGain;
}

void BMA180::Configure()
{  
  delay(10);//Utile ?
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

void BMA180::ProcessData()
{
    //Zéro de l'accéléromètre
    m_acceleration.x *= m_accelerationGain;
    m_acceleration.y *= m_accelerationGain;
    m_acceleration.z *= m_accelerationGain;
    
    m_acceleration.x += m_accelerationZero.x;
    m_acceleration.y += m_accelerationZero.y;
    m_acceleration.z += m_accelerationZero.z;
}


boolean BMA180::IsAlive() const
{
  //On vérifie que le capteur est bien actif en cherchant à lire son id, normalement : 11
  //Pour cela on lit le registre chip_id (2 LSB du registre 0x00)
  
  static byte buffer;
  
  Wire.beginTransmission(0x40);//Adresse i2c du capteur
  Wire.write(uint8_t(0x00));//Adresse i2c du registre
  Wire.endTransmission();
  
  //On lit 1 octet
  Wire.requestFrom(0x40,1);
  buffer = Wire.read();
  
  return (buffer & 0x03) == 0x03;
}

void BMA180::Read()
{
  static byte outbuffer[6];
  
  Wire.beginTransmission(0x40);
  Wire.write(0x02);
  Wire.endTransmission();
  
  Wire.requestFrom(0x40,6);
  outbuffer[0] = Wire.read();//x lsb
  outbuffer[1] = Wire.read();//x msb
  outbuffer[2] = Wire.read();//y lsb
  outbuffer[3] = Wire.read();//y msb
  outbuffer[4] = Wire.read();//z lsb
  outbuffer[5] = Wire.read();//z msb
  
  //Avec 4G de resolution, on a 0.5 mG /LSB, on divise donc par 2 pour avoir des valeurs en mg
  m_acceleration.x = float(((outbuffer[1] << 8) | (outbuffer[0])) >>2) / 2.f;
  m_acceleration.y = float(((outbuffer[3] << 8) | (outbuffer[2])) >>2) / 2.f;
  m_acceleration.z = float(((outbuffer[5] << 8) | (outbuffer[4])) >>2) / 2.f;
  
  m_readingsCount++;
  
}

float BMA180::GetTemperature() const
{
  return m_temperature;
}

void BMA180::ReadTemperature()
{
  static byte buffer;
  Wire.beginTransmission(0x40);
  Wire.write(0x08);
  Wire.endTransmission();
  
  Wire.requestFrom(0x40,1);
  buffer = Wire.read();//octet de la temperature

  //temperature en complément à 2
  //Lorsque l'octet vaut 0, on a la temperature médiane 23.5°
  m_temperature = 23.5 + ((buffer << 8) >>8) / 2.f;
}
