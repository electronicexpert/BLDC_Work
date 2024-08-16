/*
 * MPU6050.cpp
 *
 *  Created on: Jul 8, 2024
 *      Author: joachim
 */
#include "MPU6050.h"
#include "main.h"
#include "stdio.h"
#include "math.h"

#include <iostream>


uint8_t i2cData_setup[1];



extern I2C_HandleTypeDef hi2c1;

Gyro::Gyro() {
	// TODO Auto-generated constructor stub

}

Gyro::~Gyro() {
	// TODO Auto-generated destructor stub
}


void Gyro::MPU6050_Read(uint16_t DevAddress, uint16_t MemAddress,uint8_t *data, uint8_t nr)
{
	HAL_StatusTypeDef ret;

	ret = HAL_I2C_Mem_Read(&hi2c1,(DevAddress << 1) + 1,MemAddress,1,data,nr,100);
//	if(ret == HAL_OK)
//	  std::cout << "I2C-Read  ok\r\n";
//	else
//		std::cout << "I2C-Read failed\r\n";
}



void  Gyro::MPU6050_Write(uint16_t DevAddress, uint16_t MemAddress,uint8_t data)
{

	 HAL_StatusTypeDef ret = HAL_I2C_Mem_Write(&hi2c1,(DevAddress << 1) + 0,MemAddress,1,&data,1,100);
//	 if(ret == HAL_OK)
//	 	 		  std::cout << "I2C-Write  ok\r\n";
//	 	 	  else
//	 	 		  std::cout << "I2C-Write failed\r\n";
}


void Gyro::MPU6050_Initialize(void)
{

	 HAL_StatusTypeDef  ret = HAL_I2C_IsDeviceReady(&hi2c1,(DEVICE_ADDRESS << 1) + 0,1,100);

		  if(ret == HAL_OK)
			  std::cout << "The Device is ready\r\n";
		  else
			  std::cout << "The Device is not ready\r\n";



	Gyro::MPU6050_Write(DEVICE_ADDRESS, MPU6050_RA_PWR_MGMT_1, 1<<7);      //reset the whole module first


	HAL_Delay(50);    //wait for 50ms for the gyro to stable

	// Device RESET
    Gyro::MPU6050_Write(DEVICE_ADDRESS, MPU6050_RA_PWR_MGMT_1, MPU6050_CLOCK_PLL_ZGYRO);//PLL with Z axis gyroscope reference

    // set Sample Rate
    Gyro::MPU6050_Write(DEVICE_ADDRESS, MPU6050_RA_SMPLRT_DIV, 0x07);    // 1 kHz sample rate

    // Diaable FSYNC, set BW Accel and Gyro
    Gyro::MPU6050_Write(DEVICE_ADDRESS, MPU6050_RA_CONFIG, 0x00);        //DLPF_CFG = 0: Acc bw = 260 Hz, Gyro bw = 256 Hz

    // Config Gyro
    Gyro::MPU6050_Write(DEVICE_ADDRESS, MPU6050_RA_GYRO_CONFIG, MPU6050_GYRO_FS_250);    //Gyro full scale setting

    // Config Accel
      Gyro::MPU6050_Write(DEVICE_ADDRESS,MPU6050_RA_ACCEL_CONFIG, MPU6050_ACCEL_FS_2);    //Gyro full scale setting

      Gyro::MPU6050_Read(DEVICE_ADDRESS, MPU6050_RA_WHO_AM_I,i2cData_setup,1);
      if (i2cData_setup[0] == MPU6050_WHO_AM_I_VALUE)
       // Read "WHO_AM_I" register
        std::cout << "Device recognized\r\n";
      else
    	  std::cout << "Device not recognized\r\n";


}

//int16_t read_acc_x_signed(uint8_t *Received_Data)
//{
//	uint8_t h,l;
//	uint16_t x_acc_raw;
//	int16_t x_acc_signed;
//
//	h = Received_Data[0];
//	l = Received_Data[1];
//
//	x_acc_raw =(h << 8) + l;
//
//	 if(x_acc_raw >= 0x8000)
//	       x_acc_signed = -((65535 - x_acc_raw) + 1);
//	 else
//	       x_acc_signed = x_acc_raw;
//
//	 return x_acc_signed;
//}
//
//
//int16_t read_acc_y_signed(uint8_t *Received_Data)
//{
//	uint8_t h,l;
//	uint16_t y_acc_raw;
//	int16_t y_acc_signed;
//
//	h = Received_Data[2];
//	l = Received_Data[3];
//
//	y_acc_raw =(h << 8) + l;
//
//	 if(y_acc_raw >= 0x8000)
//	       y_acc_signed = -((65535 - y_acc_raw) + 1);
//	 else
//	       y_acc_signed = y_acc_raw;
//
//	 return y_acc_signed;
//}
//
//
//int16_t read_acc_z_signed(uint8_t *Received_Data)
//{
//	uint8_t h,l;
//	uint16_t z_acc_raw;
//	int16_t z_acc_signed;
//
//	h = Received_Data[4];
//	l = Received_Data[5];
//
//	z_acc_raw =(h << 8) + l;
//
//	 if(z_acc_raw >= 0x8000)
//	       z_acc_signed = -((65535 - z_acc_raw) + 1);
//	 else
//	       z_acc_signed = z_acc_raw;
//
//	 return z_acc_signed;
//}
//
//double degrees(double a)
//{
//	double grad;
//	grad = 180.0 * a / M_PI;
//	return grad;
//
//}
//
//double dist(double a, double b)
//{
//        return sqrt((a*a)+(b*b));
//}
//
//
//double get_x_rotation(double x,double y,double z)
//{
//
//	// atan2 Wertebereich der Ausgabe: - pi ... + pi
//	double radians;
//    radians = atan2(y, dist(x,z));
////    printf("Radiant: %f\r\n",radians);
//    return  degrees(radians);
//}
//
//
//double  get_y_rotation(double x,double y,double z)
//{
//	double radians;
//
//    radians = atan2(x, dist(y,z));
//    return -degrees(radians);
//}
//
//
