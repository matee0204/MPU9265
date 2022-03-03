#ifndef MPU9265_H
#define MPU9265_H

#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>

#define MPU9250_ADDR 0x68         //Gyorsulásmérő/giroszkóp címe
#define AK8963_ADDR 0x0C         //magnetométer címe

#define CONFIG 0x1A           //Gyorsulásmérő/giroszkóp konfigurációs regisztere
#define SMPLRT_DIV 0x25       //Gyorsulásmérő/giroszkóp mintavételi frekvencia előosztó
#define PWR_MGMT 0x6B         //Gyorsulásmérő/giroszkóp 1. energiamenedzsment regisztere
#define GYRO_CONFIG 0x1B      //giroszkóp konfigurációs regiszterének címe
#define ACCEL_CONFIG 0x1C     //gyorsulásmérő konfigurációs regisztereinek címe
#define ACCEL_CONFIG_2 0x1D   
#define ACCEL_OUT 0x3B        //gyorsulásmérő kimeneti regiszterén kezdőcíme
#define GYRO_OUT 0x43         //giroszkóp kimeneti regiszterén kezdőcíme
#define TEMP_OUT 0x41         //hőmérő kimeneti regiszterének kezdőcíme
#define INT_PIN_CFG 0x37      //bypass/INT pin konfiguráló regiszter

#define MAGNETOMETER_CNTL 0x0A      //A magnetométer controll regiszterének címe
#define MAGNETOMETER_OUT_REG 0x03   //A magnetométer kimeneti regiszterének kezdőcíme
#define ST1 0x02                    //1-es státusz regiszter
#define ASA 0x10					//Együttható regiszter kezdőcíme

#define BYPASS_EN   (uint8_t)0x02
#define MPU9250_RESET (uint8_t)0x80
#define I2C_SLV0_EN (uint8_t)0x80

#define ACCEL_FS_2G (uint8_t)0x00
#define ACCEL_FS_4G (uint8_t)0x01
#define ACCEL_FS_8G (uint8_t)0x02
#define ACCEL_FS_16G (uint8_t)0x03
#define ACCEL_FCHOICE_B_EN (uint8_t)0x00
#define ACCEL_FCHOICE_B_DIS (uint8_t)0x01
#define A_DLPF_CFG_460 (uint8_t)0x00
#define A_DLPF_CFG_184 (uint8_t)0x01
#define A_DLPF_CFG_92 (uint8_t)0x02
#define A_DLPF_CFG_41 (uint8_t)0x03
#define A_DLPF_CFG_20 (uint8_t)0x04
#define A_DLPF_CFG_10 (uint8_t)0x05
#define A_DLPF_CFG_5 (uint8_t)0x06

#define GYRO_FS_250DPS (uint8_t)0x00
#define GYRO_FS_500DPS (uint8_t)0x01
#define GYRO_FS_1000DPS (uint8_t)0x02
#define GYRO_FS_2000DPS (uint8_t)0x03
#define FCHOISE_B_FAST (uint8_t)0x03
#define FCHOISE_B_SLOW (uint8_t)0x02
#define FCHOISE_B_LOWPASS (uint8_t)0x00
#define DLPF_CFG_250_4000 (uint8_t)0x00
#define DLPF_CFG_184_188 (uint8_t)0x01
#define DLPF_CFG_92_98 (uint8_t)0x02
#define DLPF_CFG_41_42 (uint8_t)0x03
#define DLPF_CFG_20_20 (uint8_t)0x04
#define DLPF_CFG_10_10 (uint8_t)0x05
#define DLPF_CFG_5_5 (uint8_t)0x06
#define DLPF_CFG_3600_4000 (uint8_t)0x07

#define BIT_16 (uint8_t)0x10
#define CONT_MEAS_MODE_2 (uint8_t)0x06
#define PWR_DWN_MODE (uint8_t)0x00
#define FUSE_ROM_ACCESS_MODE (uint8_t)0x0F

#define FIRST_RUN_ON_THIS_DEVICE 1
#define NOT_FIRST_RUN_ON_THIS_DEVICE 0

class MPU9265 {
private:
	//Gyorsulásmérőhöz tartozó privát változók
  uint8_t accel_buff[6];						//A szenzorból ide történik a byte-ok elmentése
  float accSens;								//gyorsulásmérő érzékenysége
  float gravityOffset;							//a gravitáció-vektor hossza
  
	//Giroszkóphoz tartozó privát változók
  uint8_t gyro_buff[6];					//A szenzorból ide történik a byte-ok elmentése
  float gyroSens;						//giroszkóp érzékenysége
  int16_t gyroOffset_x, gyroOffset_y, gyroOffset_z;	//giroszkóp offset hiba kompenzálása (a CalibrateGyroscope függvényben van beállítva)
  
	//Magnetométerhez tartozó privát változók
  uint8_t mag_buff[7];							//A szenzorból ide történik a byte-ok elmentése
  float magCoeff_x, magCoeff_y, magCoeff_z;		//A magnetométer ASA regisztereiből kiolvasott értékekből számított együtthatók
  float	magSens;								//magnetométer érzékenysége
  int16_t magOffset_x, magOffset_y;				//keménymágneses kalibráció során kapott offset értékek
  float r, q, sigma, cos_theta, sin_theta;		//lágymágneses kalibráció során kapott értékek (pontos jelentése navigációs doksiban van)
  
	//Hőmérőhöz tartozó privát változók
  uint8_t temp_buff[2];		//A szenzorból ide történik a byte-ok elmentése
	
	//működéshez szükséges egyéb változók
  uint32_t time;		//Időkülönbség eltárolása
  uint16_t timeOffset;	//Itt történik az időszámítás idejének a tárolása. Amikor megtörténik az időkülönbség számítás, utána ki kell ezt vonni belőle
  uint8_t flag; //teljsen szabadon felhasználható flag, én arra használom, hogy detektáljam egy függvény első lefutását
  
public:

	//Gyorsulásmérőhöz tartozó publikus változók
  int16_t acc_x, acc_y, acc_z;					//A nyers szenzoradatok
  float acc_MPSS_x, acc_MPSS_y, acc_MPSS_z;		//m/s^2-té konvertált értékek
  
	//Giroszkóphoz tartozó publikus változók
  int16_t om_x, om_y, om_z;				//A nyers szenzoradatok
  float om_DPS_x, om_DPS_y, om_DPS_z;	//deg/s-má konvertált értékek
  float om_Deg_x, om_Deg_y, om_Deg_z;	//szögelfordulássá konvertált értékek
  
   //Magnetométerhez tartozó publikus változók
  int16_t mag_x, mag_y, mag_z;					//A nyers szenzoradatok
  float mag_uT_x, mag_uT_y, mag_uT_z;			//uT-vá konverált értékek
  
   //Hőmérőhöz tartozó publikus változók
  int16_t temp;				//A nyers szenzoradatok
  float temp_C;				//°C-ká konvertált értékek
  
  MPU9265();

  void Init(int baudrate, int runCount);
  void I2C_write(uint8_t addr, uint8_t reg, uint8_t data);
  void I2C_read(uint8_t addr, uint8_t reg, uint8_t count, uint8_t* buff);
  
  void EnableSamplerateDivider(uint8_t en);
  void ChangeSamplerateDivider(uint8_t divider);
  
  void ChangeAccSensitivity(uint8_t sensitivity);
  void ReadAcceleration();
  void AccelerationToMPSS();
  void EnableAccFilter(uint8_t en);
  void ChangeAccBandwidth(uint8_t bandwidth);
  void FilterGravity();
  
  
  void ChangeGyroSensitivity(uint8_t sensitivity);
  void ReadGyro();
  void GyroToDPS();
  void DPSToDeg();
  void ChangeGyroTempSpeed(uint8_t speed);
  void ChangeGyroTempBandwidth(uint8_t bandwidth);
  void CalibrateGyroscope();


  void ReadMagnetometer();
  void MagnetometerTouT();
  void MagnetometerHardIronCalibration();
  void MagnetometerToHardIronCalibrated();
  void MagnetometerSoftIronCalibration();
  void MagnetometerToSoftIronCalibrated();
  
  void ReadTemperature();
  void TemperatureToC();
  
  
};


#endif