/*
 * Kalman_Initialize.cpp
 *
 *  Created on: Aug 11, 2024
 *      Author: joachim
 */
#include "main.h"
#include "MPU6050.h"
#include "Kalman.h"
#include "Kalman_Initialize.h"
#include "math.h"

#define RAD_TO_DEG  180.0 / M_PI

extern TIM_HandleTypeDef htim2;

extern Gyro gyro;

Kalman kalmanX;    // manX; // Create the Kalman instances
Kalman kalmanY;

extern uint8_t i2cData[14];

/* IMU Data */

extern double accX, accY, accZ;

extern double gyroXangle, gyroYangle; // Angle calculate using the gyro only
extern double compAngleX, compAngleY; // Calculated angle using a complementary filter
extern double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

extern uint32_t timer;


Kal_Init::Kal_Init() {
	// TODO Auto-generated constructor stub

}

Kal_Init::~Kal_Init() {
	// TODO Auto-generated destructor stub
}


  /* Set kalman and gyro starting angle */
void Kal_Init::Kalman_Initialize(void)
{
  gyro.MPU6050_Read(DEVICE_ADDRESS, MPU6050_RA_ACCEL_XOUT_H,i2cData, 6);

    accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
    accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
    accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);

//    std::cout << "ACCX: " << static_cast<int16_t>(accX)  << "\r\n";
//    std::cout << "ACCY: " << static_cast<int16_t>(accY)  << "\r\n";
//    std::cout << "ACCZ: " << static_cast<int16_t>(accZ)  << "\r\n";

#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

//    std::cout << std::fixed;
//  	std::cout  << "Roll: "<< roll  << "\r\n";
//    std::cout  << "Pitch: "<< pitch  << "\r\n";


    kalmanX.setAngle(roll); // Set starting angle
    kalmanY.setAngle(pitch);
    gyroXangle = roll;
    gyroYangle = pitch;
    compAngleX = roll;
    compAngleY = pitch;

    // ich benutze Timer 2
    // setze Timer, sodass dt berechnet werden kann

    timer = __HAL_TIM_GET_COUNTER(&htim2);

    //   das ist nur eine Pruefroutine, die welche Genauigkeit messen soll
    //     			  micros_old = __HAL_TIM_GET_COUNTER(&htim2);
    //                  HAL_Delay(10);
    //    			  micros_new = __HAL_TIM_GET_COUNTER(&htim2);
    //    			  micros_elapsed = micros_new - micros_old;
    //    			  printf("Time elapsed: %d us\r\n",micros_elapsed);

}
