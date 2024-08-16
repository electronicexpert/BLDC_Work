/*
 * Kalman_Loop.cpp
 *
 *  Created on: Aug 11, 2024
 *      Author: joachim
 */

#include "main.h"
#include "MPU6050.h"
#include "Kalman.h"
#include "Kalman_Initialize.h"
#include "Kalman_Loop.h"
#include "math.h"

#include <iostream>
#include <iomanip>

#define RAD_TO_DEG  180.0 / M_PI

extern TIM_HandleTypeDef htim2;

extern Gyro gyro;

extern Kalman kalmanX;    // manX; // Create the Kalman instances
extern Kalman kalmanY;

extern uint8_t i2cData[14];


extern double gyroXangle, gyroYangle; // Angle calculate using the gyro only
extern double compAngleX, compAngleY; // Calculated angle using a complementary filter
extern double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

/* IMU Data */

extern double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;
extern uint32_t timer;


Kal_Loop::Kal_Loop() {
	// TODO Auto-generated constructor stub

}

Kal_Loop::~Kal_Loop() {
	// TODO Auto-generated destructor stub
}

void Kal_Loop::Kalman_Loop(void)
{
#if 1
	  /* Update all the values */
	    gyro.MPU6050_Read(DEVICE_ADDRESS, MPU6050_RA_ACCEL_XOUT_H,i2cData, 14);
	    accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
	    accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
	    accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
	    tempRaw = (int16_t)((i2cData[6] << 8) | i2cData[7]);
	    gyroX = (int16_t)((i2cData[8] << 8) | i2cData[9]);
	    gyroY = (int16_t)((i2cData[10] << 8) | i2cData[11]);
	    gyroZ = (int16_t)((i2cData[12] << 8) | i2cData[13]);;

	  double dt =  __HAL_TIM_GET_COUNTER(&htim2) - timer; // Calculate delta time
	   timer =__HAL_TIM_GET_COUNTER(&htim2);
//	   std::cout << "dT: " << static_cast<int32_t>(dt)  << "\r\n";

	   // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
	   // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
	   // It is then converted from radians to degrees
	   #ifdef RESTRICT_PITCH // Eq. 25 and 26
	     double roll  = atan2(accY, accZ) * RAD_TO_DEG;
	     double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
	   #else // Eq. 28 and 29
	     double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
	     double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
	  #endif

	 double gyroXrate = gyroX / 131.0; // Convert to deg/s
	 double gyroYrate = gyroY / 131.0; // Convert to deg/s

	 #ifdef RESTRICT_PITCH
	   // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
	   if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90))
	   {
	     kalmanX.setAngle(roll);
	     compAngleX = roll;
	     kalAngleX = roll;
	     gyroXangle = roll;
	   }
	   else
	     kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

	   if (abs(kalAngleX) > 90)
	     gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
	   kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
	 #else
	   // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
	   if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90))
	   {
	     kalmanY.setAngle(pitch);
	     compAngleY = pitch;
	     kalAngleY = pitch;
	     gyroYangle = pitch;
	   }
	   else
	     kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

	   if (abs(kalAngleY) > 90)
	     gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
	   kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
	 #endif

	   gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
	   gyroYangle += gyroYrate * dt;
	   //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
	   //gyroYangle += kalmanY.getRate() * dt;

	   compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
	   compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

	   // Reset the gyro angle when it has drifted too much
	   if (gyroXangle < -180 || gyroXangle > 180)
	     gyroXangle = kalAngleX;
	   if (gyroYangle < -180 || gyroYangle > 180)
	     gyroYangle = kalAngleY;

#endif
	   /* Print Data */

#if 0
	   std::cout << static_cast<int16_t>(accX)  << "\t";
	   std::cout << static_cast<int16_t>(accY)  << "\t";
	   std::cout << static_cast<int16_t>(accZ)  << "\t";

	   std::cout << static_cast<int16_t>(gyroX)  << "\t";
	   std::cout << static_cast<int16_t>(gyroY)  << "\t";
	   std::cout << static_cast<int16_t>(gyroZ)  << "\t";

	   std::cout << "\t";

#endif
#if 1 // Set to 1 to activate

	   std::cout << std::fixed;
//	   std::cout  << roll  << "\t";
//	   std::cout  << gyroXangle  << "\t";
//	   std::cout  << compAngleX  << "\t";
	   std::cout  << kalAngleX  << "\t";

//	   std::cout << "\t";

//	   std::cout  << pitch  << "\t";
//	   std::cout  << gyroYangle  << "\t";
//	   std::cout  << compAngleY  << "\t";
//	   std::cout  << kalAngleY  << "\t";


	 #if 0 // Set to 1 to print the temperature
	   std::cout << "\t";

	   double temperature = (double)tempRaw / 340.0 + 36.53;
	   std::cout  << temperature  << "\t";

	 #endif
#endif
	   std::cout << "\r\n";
	   HAL_Delay(10);
}

