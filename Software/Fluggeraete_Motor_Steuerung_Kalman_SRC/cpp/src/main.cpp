/*
 * main.cpp
 *
 *  Created on: August 13, 2024
 *      Author: joachim
 */

#include "MPU6050.h"
#include "Kalman.h"
#include "Kalman_Initialize.h"
#include "Kalman_Loop.h"
#include "pid.h"

#include "main.h"
#include "stdio.h"
#include <iostream>
#include <iomanip>
#include <string>

#define N_LEN 20
#define BASE 10

#define Relais GPIO_PIN_12
#define Direction GPIO_PIN_13
#define ARBEITSPUNKT 1400





extern "C"
{
//	extern TIM_HandleTypeDef htim1;
	extern I2C_HandleTypeDef hi2c1;
	extern TIM_HandleTypeDef htim2;
	extern UART_HandleTypeDef huart5;

	extern char test;
	extern uint8_t Rx_data[1];
	extern uint8_t ready;



    #ifdef __GNUC__
    #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
    #else
    #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
    #endif

    PUTCHAR_PROTOTYPE
    {
       HAL_UART_Transmit(&huart5, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
       return ch;
    }

    #ifdef __GNUC__
    #define GETCHAR_PROTOTYPE int __io_getchar(void)
    #else
    #define GETCHAR_PROTOTYPE int fgetc(FILE *f)
    #endif

    GETCHAR_PROTOTYPE
    {
      uint8_t ch = 0;

  /* Clear the Overrun flag just before receiving the first character */
     __HAL_UART_CLEAR_OREFLAG(&huart5);

  /* Wait for reception of a character on the USART RX line and echo this
   * character on console */
     HAL_UART_Receive(&huart5, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
     HAL_UART_Transmit(&huart5, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
     return ch;
   }

    extern uint32_t Impuls_Delta;

    extern uint32_t Impuls_Breite_1;
    extern uint32_t Impuls_Breite_2;
    extern uint32_t Impuls_Breite_3;
    extern uint32_t Impuls_Breite_4;
}

uint8_t endflag = 0;
uint8_t state = 0;
uint8_t abbruch_flag = 0;
uint8_t nl_flag = 0;


long drehzahl = 0;
uint8_t motor_dreht = 0;
uint8_t drehzahl_valid = 0;
uint8_t crc_4bit;


Gyro gyro;
Kal_Init kal_init;
Kal_Loop kal_loop;
PID pid;

uint8_t i2cData[14];

double accX, accY, accZ;

double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

uint32_t timer;



void maincpp()
{
	int i,k;
    char buffer[N_LEN];
    char msg_drehzahl[22] = "Drehzahl einstellen: ";



     double Winkel, PIDOut, WinkelSetpoint;

	uint8_t stellwert_l;
	uint8_t stellwert_m;
	uint8_t stellwert_h;
	Impuls_Breite_4 = 15 * Impuls_Delta;


	gyro.MPU6050_Initialize();

	 HAL_Delay(100); // Wait for sensor to stabilize

	 kal_init.Kalman_Initialize();


	 HAL_GPIO_WritePin(GPIOG, Direction,GPIO_PIN_RESET);
	 motor_dreht = 0;

	//  NVIC_EnableIRQ(TIM1_CC_IRQn); // enable CC interrupt in NVIC
	//  NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn); // enable update interrupt in NVIC

	HAL_UART_Receive_IT(&huart5, (uint8_t *)Rx_data, 1);
	  while (endflag == 0 )
	  {
		  test = 0;
		      switch(state)
		      {
		      case 0:
		      	std::cout << "\r\n";
		      	std::cout << "****************************************************************\r\n";
		      	std::cout << "*                                                              *\r\n";
		      	std::cout << "*  Fluggeraete-Motorsteuerung fuer 1 Motor (Kalman-Version)    *\r\n";
		      	std::cout << "*                                                              *\r\n";
		      	std::cout << "****************************************************************\r\n";
		      	std::cout << "*                                                              *\r\n";
		      	std::cout << "*         1    Motor testen (Hoch- und Runterlauf)             *\r\n";
		      	std::cout << "*                                                              *\r\n";
		      	std::cout << "*         2    Motordrehzahl eingeben (Hochfahren)             *\r\n";
		      	std::cout << "*                                                              *\r\n";
		      	std::cout << "*         3    Motor herunterfahren (von akt. Drehzahl)        *\r\n";
		      	std::cout << "*                                                              *\r\n";
		      	std::cout << "*         4    Motor Richtungsumkehr                           *\r\n";
		      	std::cout << "*                                                              *\r\n";
		      	std::cout << "*         5    Winkelsensor messen                             *\r\n";
		      	std::cout << "*                                                              *\r\n";
		      	std::cout << "*         6    geregelter Betrieb                              *\r\n";
		      	std::cout << "*                                                              *\r\n";
		      	std::cout << "*         7    Motor-Stop (auch SPACE-Taste)                   *\r\n";
		      	std::cout << "*                                                              *\r\n";
		      	std::cout << "*         8    Programm beenden                                *\r\n";
		      	std::cout << "*                                                              *\r\n";
		      	std::cout << "****************************************************************\r\n";

		        abbruch_flag = 0;

		      	state = 1;


		      break;

		      case 1:

		    	 while(ready == 0);
		      	 ready = 0;
		       	 switch(test)
		      	 {
		      	 case '1':
		      		if(motor_dreht == 0)
		      	    {
		      		   std::cout << "Motor testen\r\n";
		      		   HAL_Delay(1000);
		      		   HAL_GPIO_WritePin(GPIOG, Relais,GPIO_PIN_SET);  // Sicherungsrelais = EIN
		      		   NVIC_EnableIRQ(TIM1_CC_IRQn); // enable CC interrupt in NVIC
		      		   motor_dreht = 1;
		      		   for(k = 0; k < 2; k++)
		      		   {
		      			 if(HAL_GPIO_ReadPin(GPIOG, Direction))
		      		      	 std::cout << "Rechtslauf\r\n";
		      		     else
		      	     	     std::cout << "Linkslauf\r\n";
		      			 for(i = 0; i < 1000; i++)
		      			 {
		      				 if(test == 0x20 || test == '7')
		      				 {
		      					abbruch_flag = 1;
		      					motor_dreht = 0;
		      				    HAL_GPIO_WritePin(GPIOG, Relais,GPIO_PIN_RESET);  // Sicherungsrelais = A
		      				    NVIC_DisableIRQ(TIM1_CC_IRQn); // disable CC interrupt in NVIC
		      				    for(k = 0; k < 1;k++)                             // Mehrfachausgabe vermeiden
		      				    {
		      				   	   std::cout << "Abbruch durch Benutzer!\r\n";
		      				       HAL_Delay(1000);
		      				    }

                              break;
		      				 }
		      				stellwert_l = (uint32_t)i & 0x0f;
		      				stellwert_m = ((uint32_t)i >> 4) & 0x0f;
		      				stellwert_h = ((uint32_t)i >> 8) & 0x0f;

		      				Impuls_Breite_1 = stellwert_h * Impuls_Delta;
		      				Impuls_Breite_2 = stellwert_m * Impuls_Delta;
		      				Impuls_Breite_3 = stellwert_l * Impuls_Delta;
		      				HAL_Delay(10);
		      			 }

		      			 if(abbruch_flag == 0)
		      			 {

		      			     for(i = 1000; i > 0; i--)
		      			     {
		      			    	if(test == 0x20 || test == '7')
		      				   {
		      			    		abbruch_flag = 1;
		      			    		motor_dreht = 0;
		      			    	    HAL_GPIO_WritePin(GPIOG, Relais,GPIO_PIN_RESET);  // Siherungsrelais = AUS
		      			    	    NVIC_DisableIRQ(TIM1_CC_IRQn); // disable CC interrupt in NVIC
		      			    	    for(k = 0; k < 1;k++)                             // Mehrfachausgabe vermeiden
		      			    	    {
		      			    	   	   std::cout << "Abbruch durch Benutzer!\r\n";
		      			    	       HAL_Delay(1000);
		      			    	    }

		      			    	  break;
		      				   }
		      			       stellwert_l = (uint32_t)i & 0x0f;
		      			   	   stellwert_m = ((uint32_t)i >> 4) & 0x0f;
		      			   	   stellwert_h = ((uint32_t)i >> 8) & 0x0f;

		      			       Impuls_Breite_1 = stellwert_h * Impuls_Delta;
		      			       Impuls_Breite_2 = stellwert_m * Impuls_Delta;
		      			       Impuls_Breite_3 = stellwert_l * Impuls_Delta;
		      			       HAL_Delay(10);
		      			     }
		      			 }
		      		 		   // toggle DIRECTION
		    		   HAL_GPIO_TogglePin(GPIOG, Direction);
		      	       }
		      	       HAL_GPIO_WritePin(GPIOG, Direction,GPIO_PIN_RESET);
		      	       motor_dreht = 0;

		      	       HAL_GPIO_WritePin(GPIOG, Relais,GPIO_PIN_RESET);
		      	       NVIC_DisableIRQ(TIM1_CC_IRQn); // disable CC interrupt in NVIC
		      	    }
		      		else
		      		{
		      			std::cout << "Vor Motortest Motor anhalten\r\n";
		      		    HAL_Delay(1000);
		      		}
  	      		    state = 0;

		      	 break;

		      	 case '2':
		      		 if(motor_dreht == 0)
		      		 {
		      			// das geht hier leider nur "hardware-nah"

		      		   HAL_UART_Transmit(&huart5, (uint8_t *)&msg_drehzahl, 22, HAL_MAX_DELAY);

		      		   HAL_Delay(1000);

		      		   HAL_GPIO_WritePin(GPIOG, Relais,GPIO_PIN_SET);  // Sicherungsrelais = EIN
		      		   NVIC_EnableIRQ(TIM1_CC_IRQn); // enable CC interrupt in NVIC

		      		   for(i = 0; i < N_LEN;i++)
	      		    	 buffer[i] = '\0';

		      	       i = 0;
		      	       drehzahl = 0;
		      	  	   nl_flag = 0;

		      	       stellwert_l = (uint32_t)i & 0x0f;
   			   	       stellwert_m = ((uint32_t)i >> 4) & 0x0f;
		      		   stellwert_h = ((uint32_t)i >> 8) & 0x0f;

		      		   Impuls_Breite_1 = stellwert_h * Impuls_Delta;
		      		   Impuls_Breite_2 = stellwert_m * Impuls_Delta;
		      		   Impuls_Breite_3 = stellwert_l * Impuls_Delta;


	      	           while(nl_flag == 0)
	      	           {

	      	   	  	     ready = 0;
	      	   	   	     while(ready == 0){};
	      	   		     ready = 0;
	      	    	     buffer[i] = test;

	                     // das geht hier leider nur "hardware-nah"

	      	    	     HAL_UART_Transmit(&huart5, (uint8_t *)&test, 1, HAL_MAX_DELAY);

	      	    	     if(buffer[i] == '\r')
	      	    	   	   nl_flag = 1;
	      	    	     i++;
	      	    	     if(i > N_LEN-1)
	      	    		   i = 0;
	      	           }
	      	           std::string::size_type sz;   // alias of size_t
	      	           drehzahl = std::stol (buffer,&sz);
	      	           drehzahl_valid = 1;

	      	           if(drehzahl < 0)
	      	             drehzahl = 0;
	      	           if(drehzahl > 2000)
	      	            drehzahl = 2000;

	      	           std::cout << "\r\nDrehzahl ist: " << drehzahl << "\r\n";

	      	          if(HAL_GPIO_ReadPin(GPIOG, Direction))
	      	        	 std::cout << "Rechtslauf\r\n";
	      	          else
	      	      	     std::cout << "Linkslauf\r\n";
	      	          for(i = 0; i < (int) drehzahl; i++)
	      	          {
	      	       	     if(test == 0x20 || test == '7')
	      	       	     {
	      	       			abbruch_flag = 1;
	      	       		    motor_dreht = 0;
	      	       		    HAL_GPIO_WritePin(GPIOG, Relais,GPIO_PIN_RESET);  // Sicherungsrelais = A
	      	       		    NVIC_DisableIRQ(TIM1_CC_IRQn); // disable CC interrupt in NVIC
	      	       		    for(k = 0; k < 1;k++)                             // Mehrfachausgabe vermeiden
	      	       		    {

	      	       		      std::cout << "Abbruch durch Benutzer!\r\n";
	      	       		       HAL_Delay(1000);
	      	       		    }
	      	       		    break;
	      	       	   }
	      	       	   motor_dreht = 1;
	      	       	   stellwert_l = (uint32_t)i & 0x0f;
	      	       	   stellwert_m = ((uint32_t)i >> 4) & 0x0f;
	      	       	   stellwert_h = ((uint32_t)i >> 8) & 0x0f;

	      	       	   Impuls_Breite_1 = stellwert_h * Impuls_Delta;
	      	       	   Impuls_Breite_2 = stellwert_m * Impuls_Delta;
	      	       	   Impuls_Breite_3 = stellwert_l * Impuls_Delta;
	      	       	   HAL_Delay(10);
	      	       	}
		      	  }
		      	  else
		      	  {
		      		 std::cout << "Vor Drehzahleingabe Motor anhalten\r\n";
		      		 HAL_Delay(1000);
		      	  }

//	      	      motor_dreht = 0;
		          state = 0;

	             break;

		      	 case '3':
		      		std::cout << "Drehzahl herunterfahren\r\n";
		      		 HAL_Delay(1000);

		    		 if(drehzahl_valid)
		    		 {
		    		    drehzahl_valid = 0;

		    		    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_12,GPIO_PIN_SET);  // Sicherungsrelais = EIN
		    		    std::cout << "Motor herunterfahren (von aktueller Drehzahl)\r\n";
		    	        HAL_Delay(1000);

		     	        std::cout << "\r\nDrehzahl ist: " << drehzahl << "\r\n";
		    	        if(HAL_GPIO_ReadPin(GPIOG, Direction))
		    	        	std::cout << "Rechtslauf\r\n";
		    	        else
		    	        	std::cout << "Linkslauf\r\n";

		    	        HAL_Delay(1000);
		    	        NVIC_EnableIRQ(TIM1_CC_IRQn); // enable CC interrupt in NVIC

		    	        for(int i = (int) drehzahl; i > 0; i--)
		    	        {
		    		      if(test == 0x20 || test == '7')
		    		      {
		    			      HAL_GPIO_WritePin(GPIOG, GPIO_PIN_12,GPIO_PIN_RESET);  // Sicherungsrelais = AUS
		    			      for(k = 0; k < 1;k++)                     // mehrfachausgabe Meldung verhindern
		    			      {
		    			    	  std::cout << "Abbruch durch Benutzer!\r\n";
		    			    	  HAL_Delay(1000);
		    			      }

		    		     	  break;
		    		      }
		    		      motor_dreht = 1;
		    		      stellwert_l = (uint32_t)i & 0x0f;
		    		      stellwert_m = ((uint32_t)i >> 4) & 0x0f;
		    		      stellwert_h = ((uint32_t)i >> 8) & 0x0f;

		    		      Impuls_Breite_1 = stellwert_h * Impuls_Delta;
		    		      Impuls_Breite_2 = stellwert_m * Impuls_Delta;
		    		      Impuls_Breite_3 = stellwert_l * Impuls_Delta;
		    		      HAL_Delay(10);
		                }
		    		 }
		    	     else
		    	     {
		    	    	 std::cout << "Keine aktuelle Drehzahl eingegeben, somit kann nichts heruntergefahren werden!\r\n";
		    	      	 HAL_Delay(1000);
		    	     }


		    		  motor_dreht = 0;

		    		  state = 0;

		       	 break;

		      	 case '4':
		      		if(motor_dreht == 0)
		      		{
		      		   std::cout << "Motor Drehrichtung umkehren\r\n";
		      		   HAL_Delay(1000);

		      		 // steht Motor still ?

		      		   HAL_GPIO_TogglePin(GPIOG, Direction);
		      		   if(HAL_GPIO_ReadPin(GPIOG, Direction))
		      				std::cout << "Rechtslauf\r\n";
		      		   else
		      		       	std::cout << "Linkslauf\r\n";

		      		   HAL_Delay(1000);
		      		}
		      		else
		      		{
		      			 std::cout << "Vor Richtungsumkehr Motor anhalten\r\n";
		      			 HAL_Delay(1000);
		      		}

		      		 state = 0;

      		     break;

		      	 case '5':
		      		std::cout << "Winkelsensor testen\r\n";

// so, wie der Sensor jetzt montiert ist, ergibt eine Steigung über 0 hinaus negative Winkelwerte

		      		 HAL_Delay(1000);
		      		 abbruch_flag = 0;
		      		std::cout << std::fixed;
		      		 while(abbruch_flag == 0)
		      		 {
		      			 std::cout << kal_loop.Kalman_Loop() << "\r\n";
		      			 if(test == 0x20 || test == '7')
		      				abbruch_flag = 1;
		      		 }


		      		state = 0;
		      	 break;

		      	 case '6':

		      	    if(motor_dreht == 0)
		      	    {
		      		   std::cout << "geregelter Betrieb\r\n";
		      	       HAL_Delay(1000);


		      	       std::cout << "Motor in den Arbeitspunkt fahren\r\n";
		      	       HAL_GPIO_WritePin(GPIOG, Relais,GPIO_PIN_SET);  // Sicherungsrelais = EIN
     	      		   NVIC_EnableIRQ(TIM1_CC_IRQn); // enable CC interrupt in NVIC
		      	       for(int i = 0; i < ARBEITSPUNKT; i++)
		      	       {
		      	    	  if(test == 0x20 || test == '7')
		      	          {
		      	      	  	 abbruch_flag = 1;
		      	      	 	 motor_dreht = 0;
		      	      	 	 HAL_GPIO_WritePin(GPIOG, Relais,GPIO_PIN_RESET);  // Sicherungsrelais = AUS
		      	      	 	 NVIC_DisableIRQ(TIM1_CC_IRQn); // disable CC interrupt in NVIC

		      	      	  	 for(k = 0; k < 1;k++)                     // mehrfachausgabe Meldung verhindern
		      	      	   	 {
		      	      	   		 std::cout << "Abbruch durch Benutzer!\r\n";
		      	      		     HAL_Delay(1000);
		      	      		 }

		      	      		 break;
		      	          }


		      	          stellwert_l = (uint32_t)i & 0x0f;
		      	          stellwert_m = ((uint32_t)i >> 4) & 0x0f;
		      	          stellwert_h = ((uint32_t)i >> 8) & 0x0f;

      	      		      Impuls_Breite_1 = stellwert_h * Impuls_Delta;
      	      		      Impuls_Breite_2 = stellwert_m * Impuls_Delta;
      	      		      Impuls_Breite_3 = stellwert_l * Impuls_Delta;
      	      		      HAL_Delay(10);
      	      		   }



		      	       std::cout << "Regler starten\r\n";
		      	       Winkel = 0;
		      	       WinkelSetpoint = 0;
		      	       pid.Init(&Winkel, &PIDOut, &WinkelSetpoint, 0.3, 5, 0, _PID_P_ON_E, _PID_CD_DIRECT);

		      	      pid.SetMode(_PID_MODE_AUTOMATIC);
		      	      pid.SetSampleTime(50);
		      	      pid.SetOutputLimits(-1000, 1000);

// start controller every sample-time (later, measure it ...)

				      abbruch_flag = 0;
				  	  while(abbruch_flag == 0)
				   	  {

	// hole Istwert (Winkelstellung)
				  		if(test == 0x20 || test == '7')
				  		{
				   			abbruch_flag = 1;
				  		    motor_dreht = 0;
				   	 	    HAL_GPIO_WritePin(GPIOG, Relais,GPIO_PIN_RESET);  // Sicherungsrelais = AUS
				  	 	    NVIC_DisableIRQ(TIM1_CC_IRQn); // disable CC interrupt in NVIC
				  	 	    for(k = 0; k < 1;k++)                     // mehrfachausgabe Meldung verhindern
				  	 	 	{
				  	 	 	   std::cout << "Abbruch durch Benutzer!\r\n";
				  	 	 	   HAL_Delay(1000);
				  	 	 	}
				  		}
				  		Winkel = kal_loop.Kalman_Loop();     // Istwert holen

				  		pid.Compute();

				  		std::cout << "Reglerausgang: " <<  PIDOut << "\r\n";

				  		drehzahl = (ARBEITSPUNKT - (int16_t) PIDOut);

				  		std::cout << "Drehzahl als Regelgroesse: " << drehzahl  << "\r\n";


				  		 stellwert_l = (uint32_t)drehzahl & 0x0f;
				  		 stellwert_m = ((uint32_t)drehzahl >> 4) & 0x0f;
				  		 stellwert_h = ((uint32_t)drehzahl >> 8) & 0x0f;
	      	       	     Impuls_Breite_1 = stellwert_h * Impuls_Delta;
				  		 Impuls_Breite_2 = stellwert_m * Impuls_Delta;
				  		 Impuls_Breite_3 = stellwert_l * Impuls_Delta;

	      	         }

		      	  }
				  else
				 {
				  	std::cout << "Vor Reglerstart Motor anhalten\r\n";
				  	HAL_Delay(1000);
				  }

		      	  state = 0;

		      	 break;

		    	 case '7':
		    		 if(motor_dreht == 1)
		    		 {
		    		   motor_dreht = 0;
		    		   std::cout << "Motor stop\r\n";
		    		   HAL_Delay(1000);
		    		   HAL_GPIO_WritePin(GPIOG, GPIO_PIN_12,GPIO_PIN_RESET);  // Sicherungsrelais = AUS
//		    		 NVIC_DisableIRQ(TIM1_CC_IRQn); // disable CC interrupt in NVIC
		    		   for(k = 0; k < 1;k++)                     // mehrfachausgabe Meldung verhindern
		    		   {
		    			 std::cout << "Abbruch durch Benutzer!\r\n";
		    			 HAL_GPIO_WritePin(GPIOG, Relais,GPIO_PIN_RESET);
		    			 NVIC_DisableIRQ(TIM1_CC_IRQn); // disable CC interrupt in NVIC
		    		   	 HAL_Delay(1000);
		    		   }
		    		 }
		    		 else
		    		 {
		    		   std::cout << "Motor ist im Stillstand\r\n";
		    		   HAL_Delay(1000);
		    		 }

		    		 state = 0;

		    	 break;

		    	 case 0x20:
		    		 if(motor_dreht == 1)
		    		 {
		    		   motor_dreht = 0;
		    		   std::cout << "Motorstop\r\n";
		    		   HAL_GPIO_WritePin(GPIOG, GPIO_PIN_12,GPIO_PIN_RESET);  // Sicherungsrelais = AUS
//		    		 NVIC_DisableIRQ(TIM1_CC_IRQn); // disable CC interrupt in NVIC
		    		   for(k = 0; k < 1;k++)                     // mehrfachausgabe Meldung verhindern
		    		   {
		    			 std::cout << "Abbruch durch Benutzer!\r\n";
		    		     HAL_Delay(1000);
		    		   }
		    		 }
		    		 else
		    		 {
		    			 std::cout << "Motor ist im Stillstand\r\n";
		    			 HAL_Delay(1000);
		    		 }
		    		 state = 0;

		    	 break;


		    	 case '8':
		    		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_12,GPIO_PIN_RESET);  // Sicherungsrelais = AUS
		    		NVIC_DisableIRQ(TIM1_CC_IRQn); // disable CC interrupt in NVIC
		    		std::cout << "Programm wurde beendet\r\n";
		    		std::cout << "bitte Reset ausfuehren\r\n";
		    		 HAL_Delay(1000);
		    	    endflag = 1;
		    	 break;
		         }
		     }
      }
}

