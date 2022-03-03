#include "MPU9265.h"

MPU9265::MPU9265() {
	MPU9265::gyroOffset_x = 0;
	MPU9265::gyroOffset_y = 0;
	MPU9265::gyroOffset_z = 0;
	MPU9265::timeOffset = micros();		//Az időkülönbség kiszámításának idejének meghatározása
	MPU9265::timeOffset = micros() - MPU9265::timeOffset;
	MPU9265::flag = 0;
	MPU9265::om_Deg_x = 0.0;
	MPU9265::om_Deg_y = 0.0;
	MPU9265::om_Deg_z = 0.0;
	MPU9265::magOffset_x = 0;
	MPU9265::magOffset_y = 0;
	MPU9265::q = 0.0;
	MPU9265::r = 1.0;
	MPU9265::sigma = 0.0;
	MPU9265::cos_theta = 0.0;
	MPU9265::sin_theta = 0.0;
}

/*
	Eszköz, a soros port, és az I2C inicializálása. Ha először futtatjuk az adott boardon,
	és még nincs bekalibrálva a szenzor,
	akkor 0-ázni kell az eeprom megfelelő celláit.
	Ha pedig nem az első futtatás, akkor pedig ki kell olvasni az értékeket az eepromból (magnetometer kalibrációs együtthatói).
*/

void MPU9265::Init(int baudrate, int runCount) {
  Wire.begin();
  Serial.begin(baudrate);
  while(!Serial);
  if (runCount == FIRST_RUN_ON_THIS_DEVICE) {
	  EEPROM.write(0, 0);
	  EEPROM.write(1, 0);
	  EEPROM.write(2, 0);
	  EEPROM.write(3, 0);
	  EEPROM.write(4, 0);
	  EEPROM.write(5, 0);
	  EEPROM.write(6, 0);
	  EEPROM.write(7, 0);
	  EEPROM.write(8, 0);
	  EEPROM.write(9, 0);
	  EEPROM.write(10, 0);
	  EEPROM.write(11, 0);
	  EEPROM.write(12, 0);
	  EEPROM.write(13, 0);
	  EEPROM.write(14, 0);
	  EEPROM.write(15, 0);
  }
  else if(runCount == NOT_FIRST_RUN_ON_THIS_DEVICE) {
	   EEPROM.get(0, MPU9265::magOffset_x);
	   EEPROM.get(2, MPU9265::magOffset_y);
	   EEPROM.get(4, MPU9265::sigma);
	   EEPROM.get(8, MPU9265::cos_theta);
	   EEPROM.get(12, MPU9265::sin_theta);
  }
  uint8_t temp[3];
  MPU9265::I2C_write(MPU9250_ADDR, PWR_MGMT, MPU9250_RESET);	//szenzor reset
  delay(10);
  MPU9265::I2C_write(MPU9250_ADDR, ACCEL_CONFIG, (ACCEL_FS_2G << 3));	//alap beállítások a gyorsulásmérőre (2g-s méréshatár)
  MPU9265::I2C_write(MPU9250_ADDR, GYRO_CONFIG, (GYRO_FS_250DPS << 3));	// alap beállítások a giroszkópra (250 deg/s méréshatár)
  MPU9265::I2C_write(MPU9250_ADDR, INT_PIN_CFG, BYPASS_EN);				//I2C bypass engedélyezése (a magnetométer egy külön IC, ezért csak a gyosulásmérőn keresztül lehet vezérelni)
  delay(100);
  MPU9265::I2C_write(AK8963_ADDR, MAGNETOMETER_CNTL, PWR_DWN_MODE);		//magnetométer reset
  delay(10);
  MPU9265::I2C_write(AK8963_ADDR, MAGNETOMETER_CNTL, FUSE_ROM_ACCESS_MODE);	//olyan módba kapcsoljuk a magnetométert, hogy ki lehessen olvasni az együtthaókhoz szükséges konstansokat
  delay(10);
  MPU9265::I2C_read(AK8963_ADDR, ASA, 3, temp);
  MPU9265::magCoeff_x = (((float)temp[0] - 128.0) / 256.0) + 1;			//együtthatók számítása (ez az adatlapban lévő magic konstans)
  MPU9265::magCoeff_y = (((float)temp[1] - 128.0) / 256.0) + 1;
  MPU9265::magCoeff_z = (((float)temp[2] - 128.0) / 256.0) + 1;
  MPU9265::I2C_write(AK8963_ADDR, MAGNETOMETER_CNTL, (BIT_16 | CONT_MEAS_MODE_2));	//16 bites kimenet és folytonos mérés mód beállítása
  delay(100);
  MPU9265::accSens = 2.0 / 32768.0;		//sensitivity = max[G] / 2^15
  MPU9265::magSens = 4912.0 / 32768.0;	//sensitivity = max[uT] / 2^15
  MPU9265::gyroSens = 250.0 / 32768.0;	//sensitivity = max[DPS] / 2^15
  //Mivel itt még valószínűleg nyugalomban van az eszköz, ezért itt lett kiszámolva az aktuális gyorsulásvektor hossza.
  //Ebből később meg lehet valósítani a gravitáció kompenzációt
  MPU9265::ReadAcceleration();
  MPU9265::gravityOffset = sqrt((int32_t)MPU9265::acc_x * (int32_t)MPU9265::acc_x + (int32_t)MPU9265::acc_y * (int32_t)MPU9265::acc_y + (int32_t)MPU9265::acc_z * (int32_t)MPU9265::acc_z);
}

/*
	Ezzel a függvénnyel lehet állítani a szenzor regisztereit.
	Meg kell adni az eszköz címét (MPU9250_ADDR: gyorsulásmérő + giroszkóp, AK8963_ADDR: magnetométer),
	valamint a regiszter címét, és a beírandó adatot
*/

void MPU9265::I2C_write(uint8_t addr, uint8_t reg, uint8_t data) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission(true);
}

/*
	Ezzel a függvénnyel lehet kiolvasni a szenzor regisztereit.
	Meg kell adni az eszköz címét (MPU9250_ADDR: gyorsulásmérő + giroszkóp, AK8963_ADDR: magnetométer),
	valamint a kiolvasandó adatBYTE-ok számát, illetve egy megfelelő méretű buffert az adatoknak.
*/

void MPU9265::I2C_read(uint8_t addr, uint8_t reg, uint8_t count, uint8_t* buff) {
   Wire.beginTransmission(addr);
   Wire.write(reg);
   Wire.endTransmission(false);
   Wire.requestFrom(addr, count, (uint8_t)true);
   for (int i = 0; i < count; i++) {
      buff[i] = Wire.read(); 
   }
}

/*
	Itt lehet engedélyezni a sample rate dividert. Kicsit trükkös, mert ahhoz, hogy működjön,
	a gyro configban be kell állítani az FCHOISE_B biteket 00-ra (ezzel az FCHOISE 11 lesz, mert az egyik a másik invertáltja).
	Csak ekkor működik a sample rate divider. És ekkor ha a DLPF_CFG 0, vagy 7, akkor a sample rate 8kHz,
	ha pedig 1, 2, 3, 4, 5, vagy 6, akkor pedig 1kHz. Alapból a DLPF_CFG-t 7-re állítottam.
	De vigyázz, mert ha be van kapcsolva a sample rate divider, akkor ezzel egy low pass filter is aktiválódik.
	Kicsit hosszabban és hivatalosabban itt van elmagyarázva:
	https://invensense.tdk.com/wp-content/uploads/2015/02/RM-MPU-9250A-00-v1.6.pdf
	Szükséges regiszterek (sorszám decimálisan): 25 - Sample Rate Divider
												 26 - Configuration
												 27 - Gyroscope Configuration
*/

void MPU9265::EnableSamplerateDivider(uint8_t en) {
	uint8_t gyroConfigTemp, configTemp;
	MPU9265::I2C_read(MPU9250_ADDR, GYRO_CONFIG, 1, &gyroConfigTemp);
	gyroConfigTemp &= 0xFC;
	MPU9265::I2C_read(MPU9250_ADDR, CONFIG, 1, &configTemp);
	configTemp &= 0xF8;
	switch(en) {
		case 0 : MPU9265::I2C_write(MPU9250_ADDR, GYRO_CONFIG, FCHOISE_B_FAST | gyroConfigTemp); break;
		case 1 : MPU9265::I2C_write(MPU9250_ADDR, GYRO_CONFIG, FCHOISE_B_LOWPASS | gyroConfigTemp); break;
				 MPU9265::I2C_write(MPU9250_ADDR, CONFIG, DLPF_CFG_3600_4000 | configTemp); break;
	}
}

/*
	Mielőtt használod, engedélyezd a sample rate divider-t
	Illetve ha engedélyezted, akkor ne állítsd a giroszkópot FCHOISE_B_FAST, vagy FCHOISE_B_SLOW módba.
*/

void MPU9265::ChangeSamplerateDivider(uint8_t divider) {
	MPU9265::I2C_write(MPU9250_ADDR, SMPLRT_DIV, divider);
}

/*
	A gyorsulásmérő mérési tartományának állítása
*/

void MPU9265::ChangeAccSensitivity(uint8_t sensitivity) {
	uint8_t temp;
	MPU9265::I2C_read(MPU9250_ADDR, ACCEL_CONFIG, 1, &temp);
	temp &=0xE7;
	switch(sensitivity) {
		case 0 : MPU9265::I2C_write(MPU9250_ADDR, ACCEL_CONFIG, (ACCEL_FS_2G << 3) | temp); 
				 MPU9265::accSens = 2.0 / 32768.0;
				 break;
		case 1 : MPU9265::I2C_write(MPU9250_ADDR, ACCEL_CONFIG, (ACCEL_FS_4G << 3) | temp);
				 MPU9265::accSens = 4.0 / 32768.0;
				 break;
		case 2 : MPU9265::I2C_write(MPU9250_ADDR, ACCEL_CONFIG, (ACCEL_FS_8G << 3) | temp);
				 MPU9265::accSens = 8.0 / 32768.0;
				 break;
		case 3 : MPU9265::I2C_write(MPU9250_ADDR, ACCEL_CONFIG, (ACCEL_FS_16G << 3) | temp);
				 MPU9265::accSens = 16.0 / 32768.0;
				 break;
	}
	delay(10);
	
	//Újra le kell mérni a gravitáció vektor hosszát a gravitáció kompenzációhoz, mert megváltozott mérési tartomány
	MPU9265::ReadAcceleration();
	MPU9265::gravityOffset = sqrt((int32_t)MPU9265::acc_x * (int32_t)MPU9265::acc_x + (int32_t)MPU9265::acc_y * (int32_t)MPU9265::acc_y + (int32_t)MPU9265::acc_z * (int32_t)MPU9265::acc_z);
}

/*
	A gyorsulás adatok kiolvasása a szenzorból. 
*/
void MPU9265::ReadAcceleration() {  
  MPU9265::I2C_read(MPU9250_ADDR, ACCEL_OUT, 6, MPU9265::accel_buff);
  MPU9265::acc_x = (MPU9265::accel_buff[0] << 8 | MPU9265::accel_buff[1]);
  MPU9265::acc_y = (MPU9265::accel_buff[2] << 8 | MPU9265::accel_buff[3]);
  MPU9265::acc_z = (MPU9265::accel_buff[4] << 8 | MPU9265::accel_buff[5]);
}

/*
	Nyers gyorsulásadatok átalakítása m/s^2-té
*/

void MPU9265::AccelerationToMPSS() {
  MPU9265::acc_MPSS_x = (float)MPU9265::acc_x * MPU9265::accSens;
  MPU9265::acc_MPSS_y = (float)MPU9265::acc_y * MPU9265::accSens;
  MPU9265::acc_MPSS_z = (float)MPU9265::acc_z * MPU9265::accSens;
}

/*
	Itt lehet bekapcsolni a gyorsulásmérőn a low pass filtert.
	Ennek a mintavételezési frekvenciájára is hatással van a sample rate divider.
	Bővebben: https://invensense.tdk.com/wp-content/uploads/2015/02/RM-MPU-9250A-00-v1.6.pdf
	Szükséges regiszterek (sorszám decimálisan): 28 - Accelerometer Configuration
												 29 - Accelerometer Configuration 2
*/

void MPU9265::EnableAccFilter(uint8_t en) {
	uint8_t temp;
	MPU9265::I2C_read(MPU9250_ADDR, ACCEL_CONFIG_2, 1, &temp);
	temp &= 0xF7;
	switch(en) {
		case 0 : MPU9265::I2C_write(MPU9250_ADDR, ACCEL_CONFIG_2, (ACCEL_FCHOICE_B_DIS << 3) | temp); break;
		case 1 : MPU9265::I2C_write(MPU9250_ADDR, ACCEL_CONFIG_2, (ACCEL_FCHOICE_B_EN << 3) | temp); break;
	}
	
}

/*
	Itt lehet beállítani a szűrő sávszélességét (pl.: A_DLPF_CFG_460 - 460Hz).
*/

void MPU9265::ChangeAccBandwidth(uint8_t bandwidth) {
	uint8_t temp;
	MPU9265::I2C_read(MPU9250_ADDR, ACCEL_CONFIG_2, 1, &temp);
	temp &= 0xF8;
	switch(bandwidth) {
		case 0 : MPU9265::I2C_write(MPU9250_ADDR, ACCEL_CONFIG_2, A_DLPF_CFG_460 | temp); break;
		case 1 : MPU9265::I2C_write(MPU9250_ADDR, ACCEL_CONFIG_2, A_DLPF_CFG_184 | temp); break;
		case 2 : MPU9265::I2C_write(MPU9250_ADDR, ACCEL_CONFIG_2, A_DLPF_CFG_92 | temp); break;
		case 3 : MPU9265::I2C_write(MPU9250_ADDR, ACCEL_CONFIG_2, A_DLPF_CFG_41 | temp); break;
		case 4 : MPU9265::I2C_write(MPU9250_ADDR, ACCEL_CONFIG_2, A_DLPF_CFG_20 | temp); break;
		case 5 : MPU9265::I2C_write(MPU9250_ADDR, ACCEL_CONFIG_2, A_DLPF_CFG_10 | temp); break;
		case 6 : MPU9265::I2C_write(MPU9250_ADDR, ACCEL_CONFIG_2, A_DLPF_CFG_5 | temp); break;
	}
}

/*
	Gravitáció szűrése.
	először ki kell számolni az aktuális gyorsulásvektor hosszát,
	ezután ki kell számolni, hogy mekkora szögben van a gyorsulásvektor az egyes komponensekhez képest (egészen pontosan a szög cos-át kell kiszámolni).
	Majd ki kell vonni az aktuális gyorsulásvektorból a gravitáció vektort, ezután pedig a különbség és a szögek alapján megkapjuk az új x, y, és z komponenseket.
*/

void MPU9265::FilterGravity() {
	float vectorLength = sqrt((int32_t)MPU9265::acc_x * (int32_t)MPU9265::acc_x + (int32_t)MPU9265::acc_y * (int32_t)MPU9265::acc_y + (int32_t)MPU9265::acc_z * (int32_t)MPU9265::acc_z);
	float cos_x = (float)acc_x / vectorLength;
	float cos_y = (float)acc_y / vectorLength;
	float cos_z = (float)acc_z / vectorLength;
	vectorLength -= MPU9265::gravityOffset;
	MPU9265::acc_x = (int16_t)(vectorLength * cos_x);
	MPU9265::acc_y = (int16_t)(vectorLength * cos_y);
	MPU9265::acc_z = (int16_t)(vectorLength * cos_z);
}

/*
	A giroszkóp mérési tartományának állítása
*/

void MPU9265::ChangeGyroSensitivity(uint8_t sensitivity) {
	uint8_t temp;
	MPU9265::I2C_read(MPU9250_ADDR, GYRO_CONFIG, 1, &temp);
	temp &= 0xE7;
	switch(sensitivity) {
		case 0 : MPU9265::I2C_write(MPU9250_ADDR, GYRO_CONFIG, (GYRO_FS_250DPS << 3) | temp);
				 MPU9265::gyroSens = 250.0 / 32768.0;
				 break;
		case 1 : MPU9265::I2C_write(MPU9250_ADDR, GYRO_CONFIG, (GYRO_FS_500DPS << 3) | temp);
				 MPU9265::gyroSens = 500.0 / 32768.0;
				 break;
		case 2 : MPU9265::I2C_write(MPU9250_ADDR, GYRO_CONFIG, (GYRO_FS_1000DPS << 3) | temp);
				 MPU9265::gyroSens = 1000.0 / 32768.0;
				 break;
		case 3 : MPU9265::I2C_write(MPU9250_ADDR, GYRO_CONFIG, (GYRO_FS_2000DPS << 3) | temp);
				 MPU9265::gyroSens = 2000.0 / 32768.0;
				 break;
	}
}

/*
	A szögsebesség adatok kiolvasása a szenzorból. 
*/
  
void MPU9265::ReadGyro() {
  MPU9265::I2C_read(MPU9250_ADDR, GYRO_OUT, 6, MPU9265::gyro_buff);
  MPU9265::om_x = (MPU9265::gyro_buff[0] << 8 | MPU9265::gyro_buff[1]) - MPU9265::gyroOffset_x;	//a gyroOffset-ek a kalibrálás során kapott értékek
  MPU9265::om_y = (MPU9265::gyro_buff[2] << 8 | MPU9265::gyro_buff[3]) - MPU9265::gyroOffset_y;
  MPU9265::om_z = (MPU9265::gyro_buff[4] << 8 | MPU9265::gyro_buff[5]) - MPU9265::gyroOffset_z;
}

/*
	Nyers gyorsulásadatok átalakítása deg/s-má
*/

void MPU9265::GyroToDPS() {
  MPU9265::om_DPS_x = (float)MPU9265::om_x * MPU9265::gyroSens;
  MPU9265::om_DPS_y = (float)MPU9265::om_y * MPU9265::gyroSens;
  MPU9265::om_DPS_z = (float)MPU9265::om_z * MPU9265::gyroSens;
}

/*
	Szögsebesség átalakítása szögelfordulássá.
	Ehhez integrálni kell a szögsebességet.
	Ez úgy lett megoldva, hogy a mintavételezett érték be lett szorozva a két mintavétel között eltelt idővel,
	majd ezek a szorzatok össze lettek adogatva (integrál közelítő összeg).
	A függvény első futása alkalmával az időkülönbség a program futásától lenne számítva, ami nagy hibát okozna,
	ezért az első alkalommal csak le kell menteni az időt.
*/

void MPU9265::DPSToDeg() {
	if (MPU9265::flag == 0) {
		MPU9265::flag = 1;
	}
	else {
		MPU9265::time = micros() - MPU9265::time - MPU9265::timeOffset;
		MPU9265::om_Deg_x += (om_DPS_x * (float)time) / 1000000.0;
		MPU9265::om_Deg_y += (om_DPS_y * (float)time) / 1000000.0;
		MPU9265::om_Deg_z += (om_DPS_z * (float)time) / 1000000.0;
	}
	MPU9265::time = micros();
}

/*
	Itt lehet bekapcsolni a giroszkóp és a hőmérő lowpass filterét.
	Vigyázz, ha átállítod, akkor elronthatod a sample rate dividert.
	(FAST mód - giroszkóp: 8.8kHz bandwidth, 0.064ms delay, 32kHz sample rate, hőmérő: 4kHz bandwidth, 0.04ms delay
	 SLOW mód - giroszkóp: 3.6kHz bandwidth, 0.11ms delay, 32kHz sample rate, hőmérő: 4kHz bandwidth, 0.04ms delay
	 LOW PASS mód - low pass filter bekapcsolása)
*/

void MPU9265::ChangeGyroTempSpeed(uint8_t speed) {
	uint8_t temp;
	MPU9265::I2C_read(MPU9250_ADDR, GYRO_CONFIG, 1, &temp);
	temp &= 0xFC;
	switch(speed) {
		case 3 : MPU9265::I2C_write(MPU9250_ADDR, GYRO_CONFIG, FCHOISE_B_FAST | temp); break;
		case 2 : MPU9265::I2C_write(MPU9250_ADDR, GYRO_CONFIG, FCHOISE_B_SLOW | temp); break;
		case 0 : MPU9265::I2C_write(MPU9250_ADDR, GYRO_CONFIG, FCHOISE_B_LOWPASS | temp); break;
	}
}

/*
	Itt lehet beállítani a szűrő sávszélességét (pl.: DLPF_CFG_250_4000 - giroszkóp: 250Hz bandwidth, hőmérő: 4000Hz bandwidth)
*/

void MPU9265::ChangeGyroTempBandwidth(uint8_t bandwidth) {
	uint8_t temp;
	MPU9265::I2C_read(MPU9250_ADDR, CONFIG, 1, &temp);
	temp &= 0xF8;
	switch(bandwidth) {
		case 0 : MPU9265::I2C_write(MPU9250_ADDR, CONFIG, DLPF_CFG_250_4000 | temp); break;
		case 1 : MPU9265::I2C_write(MPU9250_ADDR, CONFIG, DLPF_CFG_184_188 | temp); break;
		case 2 : MPU9265::I2C_write(MPU9250_ADDR, CONFIG, DLPF_CFG_92_98 | temp); break;
		case 3 : MPU9265::I2C_write(MPU9250_ADDR, CONFIG, DLPF_CFG_41_42 | temp); break;
		case 4 : MPU9265::I2C_write(MPU9250_ADDR, CONFIG, DLPF_CFG_20_20 | temp); break;
		case 5 : MPU9265::I2C_write(MPU9250_ADDR, CONFIG, DLPF_CFG_10_10 | temp); break;
		case 6 : MPU9265::I2C_write(MPU9250_ADDR, CONFIG, DLPF_CFG_5_5 | temp); break;
		case 7 : MPU9265::I2C_write(MPU9250_ADDR, CONFIG, DLPF_CFG_3600_4000 | temp); break;
	}
}

/*
	Giroszkóp kalibráció.
	Veszünk 50-50 mintát a giroszkópból úgy, hogy a szenzor közben mozdulatlan,
	és ezeknek vesszük a számtani átlagát. 
*/

void MPU9265::CalibrateGyroscope() {
  float avg[] = {0.0, 0.0, 0.0};
  for (int i = 0; i < 50; i++) {
	MPU9265::I2C_read(MPU9250_ADDR, GYRO_OUT, 6, MPU9265::gyro_buff);
	avg[0] += MPU9265::gyro_buff[0] << 8 | MPU9265::gyro_buff[1];
	avg[1] += MPU9265::gyro_buff[2] << 8 | MPU9265::gyro_buff[3];
	avg[2] += MPU9265::gyro_buff[4] << 8 | MPU9265::gyro_buff[5];
  }
  avg[0] /= 50;
  avg[1] /= 50;
  avg[2] /= 50;
  
  MPU9265::gyroOffset_x = (uint16_t)avg[0];
  MPU9265::gyroOffset_y = (uint16_t)avg[1];
  MPU9265::gyroOffset_z = (uint16_t)avg[2];
}

/*
	A magnetometer adatok kiolvasása a szenzorból. 
*/

void MPU9265::ReadMagnetometer() {
  MPU9265::I2C_read(AK8963_ADDR, MAGNETOMETER_OUT_REG, 7, MPU9265::mag_buff);
  MPU9265::mag_x = (MPU9265::mag_buff[1] << 8 | MPU9265::mag_buff[0]);
  MPU9265::mag_y = (MPU9265::mag_buff[3] << 8 | MPU9265::mag_buff[2]);
  MPU9265::mag_z = (MPU9265::mag_buff[5] << 8 | MPU9265::mag_buff[4]);
}

/*
	Nyers magnetométer-adatok átalakítása uT-vá
*/

void MPU9265::MagnetometerTouT() {
  MPU9265::mag_uT_x = (float)MPU9265::mag_x * MPU9265::magSens * MPU9265::magCoeff_x;
  MPU9265::mag_uT_y = (float)MPU9265::mag_y * MPU9265::magSens * MPU9265::magCoeff_y;
  MPU9265::mag_uT_z = (float)MPU9265::mag_z * MPU9265::magSens * MPU9265::magCoeff_z;
}

/*
	Keménymágneses kalibráció
	A működése benne van a navigációs doksiban
	Ez és a keménymágneses kalibráció átalakításra vár, mivel meg szeretném csinálni 3d-s esetre is
*/

void MPU9265::MagnetometerHardIronCalibration() {
	MPU9265::ReadMagnetometer();
	float radius;
	int16_t mag_x_max = mag_x;
	int16_t	mag_y_max = mag_y;
	int16_t mag_x_min = mag_x;
	int16_t	mag_y_min = mag_y;
	int counter = 0;
	Serial.println("Keménymágneses kalibráció");
	Serial.println("Forgasd körbe a szenzort a z tengely körül");
	while(counter < 250) {
		MPU9265::ReadMagnetometer();
		radius = sqrt((int32_t)mag_x * (int32_t)mag_x + (int32_t)mag_y * (int32_t)mag_y);
		if (radius > r) {
			r = radius;
		}
		if (radius < q) {
			q = radius;
		}
		
		if (mag_x > mag_x_max) {
			mag_x_max = mag_x;
		}
		if (mag_y > mag_y_max) {
			mag_y_max = mag_y;
		}
		if (mag_x < mag_x_min) {
			mag_x_min = mag_x;
		}
		if (mag_y < mag_y_min) {
			mag_y_min = mag_y;
		}
		counter++;
		if (counter % 10 == 0) {
			Serial.println(counter);
		}
		delay(100);
	}
	MPU9265::magOffset_x = (uint16_t)((float)(mag_x_max + mag_x_min) / 2.0);
	MPU9265::magOffset_y = (uint16_t)((float)(mag_y_max + mag_y_min) / 2.0);
	
	Serial.println("A kalibráció véget ért");
	Serial.print("x tengely offset:\t");
	Serial.println(MPU9265::magOffset_x);
	Serial.print("y tengely offset:\t");
	Serial.println(MPU9265::magOffset_y);
	
	EEPROM.put(0, MPU9265::magOffset_x);
	EEPROM.put(2, MPU9265::magOffset_y);
	
	delay(2000);
}

/*
	Keménymágneses kalibráció alkalmazása
*/

void MPU9265::MagnetometerToHardIronCalibrated() {
	MPU9265::mag_x -= magOffset_x;
	MPU9265::mag_y -= magOffset_y;
}

/*
	Légymágneses kalibráció.
	Először mindenképpen alkalmazd a keménymágneses kalibrációt!!!
	A működése benne van a navigációs doksiban.
	Ez és a keménymágneses kalibráció átalakításra vár, mivel meg szeretném csinálni 3d-s esetre is.
*/

void MPU9265::MagnetometerSoftIronCalibration() {
	MPU9265::ReadMagnetometer();
	MPU9265::MagnetometerToHardIronCalibrated();
	float radius = sqrt((int32_t)mag_x * (int32_t)mag_x + (int32_t)mag_y * (int32_t)mag_y);
	MPU9265::r = radius;
	MPU9265::q = radius;
	int counter = 0;
	Serial.println("Lágymágneses kalibráció");
	Serial.println("Forgasd körbe a szenzort a z tengely körül");
	while(counter < 250) {
		MPU9265::ReadMagnetometer();
		MPU9265::MagnetometerToHardIronCalibrated();
		float radius = sqrt((int32_t)mag_x * (int32_t)mag_x + (int32_t)mag_y * (int32_t)mag_y);
		if (radius > r) {
			r = radius;
		}
		if (radius < q) {
			q = radius;
		}
		counter++;
		if (counter % 10 == 0) {
			Serial.println(counter);
		}
		delay(100);
	}
	MPU9265::sigma = q / r;
	MPU9265::cos_theta = mag_x / r;
	MPU9265::sin_theta = mag_y / r;
	
	Serial.println("A kalibráció véget ért");
	Serial.print("sigma:\t");
	Serial.println(MPU9265::sigma);
	Serial.print("cos_theta:\t");
	Serial.println(MPU9265::cos_theta);
	Serial.print("sin_theta:\t");
	Serial.println(MPU9265::sin_theta);
	
	EEPROM.put(4, MPU9265::sigma);
	EEPROM.put(8, MPU9265::cos_theta);
	EEPROM.put(12, MPU9265::sin_theta);
	
	delay(2000);
}

/*
	Lágymágneses kalibráció alkalmazása
*/

void MPU9265::MagnetometerToSoftIronCalibrated() {
	float mag_x_temp = (float)mag_x;
	float mag_y_temp = (float)mag_y;
	MPU9265::mag_x = (int16_t)(MPU9265::sigma * (mag_x_temp*MPU9265::cos_theta + mag_y_temp*MPU9265::sin_theta));
	MPU9265::mag_y = (int16_t)(mag_y_temp*MPU9265::cos_theta - mag_x_temp*MPU9265::sin_theta);
}

/*
	A mhőmérő adatok kiolvasása a szenzorból. 
*/

void MPU9265::ReadTemperature() {
  MPU9265::I2C_read(MPU9250_ADDR, TEMP_OUT, 2, MPU9265::temp_buff);
  MPU9265::temp = MPU9265::temp_buff[0] << 8 | MPU9265::temp_buff[1];
}

/*
	Nyers hőmérőadatok átalakítása °C-ká
*/

void MPU9265::TemperatureToC() {
	MPU9265::temp_C = ((float)MPU9265::temp - 21.0) / 333.87 + 21.0;
}