#include "Sensors.h"
#include "Wire.h"

Sensors::Sensors()
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
  
  accData.x = 0;
  accData.y = 0;
  accData.z = 0;
  accZero.x = 0;
  accZero.y = 0;
  accZero.z = 0;
  
  currentTime = millis();
  previousTime = currentTime;
  
  /*
  
  preAdata.x = 0;
  preAdata.y = 0;
  preAdata.z = 0;
  eulerAngles.x = 0;
  eulerAngles.y = 0;
  eulerAngles.z = 0;
  
  accAngles.x = 0;
  accAngles.y = 0;
  preAccAngles.z = 0;
  preAccAngles.x = 0;
  preAccAngles.y = 0;
  accAngles.z = 0;
  
  rotationsCount = 0;
  
  //A réduire pour augmenter la réactivité et réduire le filtrage passe bas
  AccFilterCoeff = 0.82;
  tau=0.075;*/
}

Sensors::~Sensors()
{
}

void Sensors::Init()
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
  this->CalibrateGyro();
  
  /***************** Initialisation des accéléromètres  ********************/
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
  this->CalibrateAcc();
}

void Sensors::UpdateData()
{
  //preAdata = adata;
  //preAccAngles = accAngles;
  currentTime = millis();
  looptime = currentTime-previousTime;
  previousTime = currentTime;
  //Lecture du Gyro
  this->ReadGyros();
  //Zéro du gyro
  gyroData[0].x -= gyroZero.x;
  gyroData[0].y -= gyroZero.y;
  gyroData[0].z -= gyroZero.z;
  //Integration
  gyroAngles.x += RK4Integrate(gyroData[3].x, gyroData[2].x, gyroData[1].x, gyroData[0].x, looptime);
  gyroAngles.y += RK4Integrate(gyroData[3].y, gyroData[2].y, gyroData[1].y, gyroData[0].y, looptime);
  gyroAngles.z += RK4Integrate(gyroData[3].z, gyroData[2].z, gyroData[1].z, gyroData[0].z, looptime);
  //Lecture de l'accélérometre
  this->ReadAccs();
  //Zéro de l'accéléromètre
  
  accData.x *= accGain;
  accData.y *= accGain;
  accData.z *= accGain;
  
  accData.x += accZero.x;
  accData.y += accZero.y;
  accData.z += accZero.z;

}
byte Sensors::IsActive()
{
  
  //adresse du port i2c en lecture
  Wire.beginTransmission(0x69);
  //On set l'adresse du registre WHO_AM_I
  Wire.write(uint8_t(0x00));
  Wire.endTransmission();
  
  //On lit 1 octet depuis l'adresse 0x68
  Wire.requestFrom(0x69,1);
  outbuf[0] = Wire.read();
  Serial.println(outbuf[0],HEX);
  
  return outbuf[0];
}
void Sensors::CalibrateGyro()
{
  
  long int sumx = 0, sumy = 0, sumz = 0;
  int nbsamples = 100;
  for(int i(0) ; i < nbsamples ; ++i)
  {
     this->ReadGyros();
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

void Sensors::CalibrateAcc()
{
  float sumx = 0, sumy = 0, sumz = 0;
  int nbsamples = 400;
  for(int i(0) ; i < nbsamples ; ++i)
  {
     this->ReadAccs();
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
  Serial.print("norme = ");
  Serial.println(norme);
  accGain = 1000.f/norme;
  Serial.print("acc gain = ");
  Serial.println(accGain);
  //au repos, l'acc x et y doivent valoir 0
  accZero.x = - accZero.x * accGain;
  accZero.y = - accZero.y * accGain;
  accZero.z = 1000 - accZero.z * accGain;
  Serial.print("x y z = ");
  Serial.print(accZero.x);
  Serial.print(" | ");
  Serial.print(accZero.y);
  Serial.print(" | ");
  Serial.println(accZero.z);
}

vector3f Sensors::GetGyroData()
{
  return gyroData[0];
}

vector3f Sensors::GetGyroAngles()
{
  return gyroAngles;
}

vector3f Sensors::GetAccData()
{
  return accData;
}

//--------------------------------------------------------------



float Sensors::RK4Integrate(int data4, int data3, int data2, int data1, int deltaTmillis)
{
  float area = (((data4+2*data3+2*data2+data1)/6)*deltaTmillis/1000.f);
  return area;
}
/*
vector3f Sensors::accToEuler(vector3f ang)
{
  vector3f a;
  a.x  = degrees(atan2(-ang.x,-ang.z)+3.14);
  a.y  = degrees(atan2(-ang.y,-ang.z)+3.14);
  
  
  
  float g = sqrt(ang.x*ang.x + ang.y*ang.y + ang.z*ang.z);
  
  //2)Calcul angle
  a.x = -acos(ang.y/g);
  //if(ang.z < 0)
      //ang.x = -ang.x;
  
  a.y = -acos(ang.x/g);
  if(ang.z < 0)
      a.y = -a.y;
      
  a.x = degrees(a.x);
  a.y = degrees(a.y);
  a.z = 0;
  
  return a;  
}*/

void Sensors::ReadGyros()
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
  gyroData[0].y = -((outbuf[2] << 8) | outbuf[3])/14;
  gyroData[0].x =  ((outbuf[0] << 8) | outbuf[1])/14;
  gyroData[0].z =  ((outbuf[4] << 8) | outbuf[5])/14;
  
  measuresCountGyro++;
  
}

void Sensors::ReadAccs()
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
  
  measuresCountAcc++;
  
}

//Instantiate Sensor
Sensors Sensor = Sensors();
