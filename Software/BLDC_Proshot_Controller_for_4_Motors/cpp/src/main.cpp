/*
 * main.cpp
 *
 *  Created on: August 13, 2024
 *      Author: joachim
 */


#include "main.h"
#include "stdio.h"
#include <iostream>
#include <iomanip>
#include <string>

#define N_LEN 20
#define BASE 10

extern "C"
{
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

    extern const struct Impuls imp;
    extern  struct Motor m1;
    extern  struct Motor m2;
    extern  struct Motor m3;
    extern  struct Motor m4;

}

uint8_t endflag = 0;
uint8_t state = 0;
uint8_t abort1_flag = 0;
uint8_t abort2_flag = 0;
uint8_t abort3_flag = 0;
uint8_t abort4_flag = 0;
uint8_t nl_flag = 0;


long speed1 = 0;
long speed2 = 0;
long speed3 = 0;
long speed4 = 0;

uint8_t speed1_valid = 0;
uint8_t speed2_valid = 0;
uint8_t speed3_valid = 0;
uint8_t speed4_valid = 0;

uint8_t motor1_turns = 0;
uint8_t motor2_turns = 0;
uint8_t motor3_turns = 0;
uint8_t motor4_turns = 0;

void setPulseTable(volatile uint32_t *table, const struct Motor * motor)
{
	table[0] = motor->Impuls_Start;
	table[1] = motor->Impuls_Start + imp.Impuls_Base + motor->Impuls_Width_1;
	table[2] = motor->Impuls_Start + imp.Impuls_Distance;
	table[3] = motor->Impuls_Start + imp.Impuls_Distance + imp.Impuls_Base + motor->Impuls_Width_2;
	table[4] = motor->Impuls_Start + 2 * imp.Impuls_Distance;
	table[5] = motor->Impuls_Start + 2 * imp.Impuls_Distance + imp.Impuls_Base + motor->Impuls_Width_3;
	table[6] = motor->Impuls_Start + 3 * imp.Impuls_Distance;
	table[7] = motor->Impuls_Start + 3 * imp.Impuls_Distance + imp.Impuls_Base + motor->Impuls_Width_4;

}

void setzero(uint8_t Motor)
{
	  uint8_t setpoint_l = 0;
	  uint8_t setpoint_m = 0;
	  uint8_t setpoint_h = 0;

	  if(Motor == 1)
	  {
		  m1.Impuls_Width_1 =  setpoint_h * imp.Impuls_Delta;
		  m1.Impuls_Width_2 =  setpoint_m * imp.Impuls_Delta;
		  m1.Impuls_Width_3 =  setpoint_l * imp.Impuls_Delta;

		  setPulseTable(Pulsetable_M1, &m1);

	  }
	  if(Motor == 2)
	  {
	  	  m2.Impuls_Width_1 =  setpoint_h * imp.Impuls_Delta;
	  	  m2.Impuls_Width_2 =  setpoint_m * imp.Impuls_Delta;
	  	  m2.Impuls_Width_3 =  setpoint_l * imp.Impuls_Delta;

	  	  setPulseTable(Pulsetable_M2, &m2);

	  }
	  if(Motor == 3)
	  {
	  	  m3.Impuls_Width_1 =  setpoint_h * imp.Impuls_Delta;
	  	  m3.Impuls_Width_2 =  setpoint_m * imp.Impuls_Delta;
	  	  m3.Impuls_Width_3 =  setpoint_l * imp.Impuls_Delta;

	  	  setPulseTable(Pulsetable_M3, &m3);

	  }
	  if(Motor == 4)
	  {
	  	  m4.Impuls_Width_1 =  setpoint_h * imp.Impuls_Delta;
	  	  m4.Impuls_Width_2 =  setpoint_m * imp.Impuls_Delta;
	  	  m4.Impuls_Width_3 =  setpoint_l * imp.Impuls_Delta;

	  	  setPulseTable(Pulsetable_M4, &m4);

	  }

}

void setvalue(uint8_t Motor, uint32_t value)
{
	 uint8_t setpoint_l = 0;
     uint8_t setpoint_m = 0;
	 uint8_t setpoint_h= 0;

	 setpoint_l= value & 0x0f;
	 setpoint_m = (value >> 4) & 0x0f;
	 setpoint_h= (value >> 8) & 0x0f;

     if(Motor == 1)
     {
	     m1.Impuls_Width_1 =  setpoint_h * imp.Impuls_Delta;
		 m1.Impuls_Width_2 =  setpoint_m * imp.Impuls_Delta;
	     m1.Impuls_Width_3 =  setpoint_l * imp.Impuls_Delta;
	     setPulseTable(Pulsetable_M1, &m1);
     }
     if(Motor == 2)
     {
         m2.Impuls_Width_1 =  setpoint_h * imp.Impuls_Delta;
     	 m2.Impuls_Width_2 =  setpoint_m * imp.Impuls_Delta;
     	 m2.Impuls_Width_3 =  setpoint_l * imp.Impuls_Delta;
     	 setPulseTable(Pulsetable_M2, &m2);
     }
     if(Motor == 3)
     {
     	 m3.Impuls_Width_1 =  setpoint_h * imp.Impuls_Delta;
     	 m3.Impuls_Width_2 =  setpoint_m * imp.Impuls_Delta;
     	 m3.Impuls_Width_3 =  setpoint_l * imp.Impuls_Delta;
     	 setPulseTable(Pulsetable_M3, &m3);
     }
     if(Motor == 4)
     {
     	 m4.Impuls_Width_1 =  setpoint_h * imp.Impuls_Delta;
     	 m4.Impuls_Width_2 =  setpoint_m * imp.Impuls_Delta;
     	 m4.Impuls_Width_3 =  setpoint_l * imp.Impuls_Delta;
     	 setPulseTable(Pulsetable_M4, &m4);
     }
}

long get_speed(void)
{
	int i;
	char buffer[N_LEN];


	for(i = 0; i < N_LEN;i++)
	   buffer[i] = '\0';
	i = 0;

	nl_flag = 0;

	while(nl_flag == 0)
	{

	   ready = 0;
	   while(ready == 0){};
	   ready = 0;
	   buffer[i] = test;

	   HAL_UART_Transmit(&huart5, (uint8_t *)&test, 1, HAL_MAX_DELAY);

	   if(buffer[i] == '\r')
	        nl_flag = 1;
	   i++;
	   if(i > N_LEN-1)
	        i = 0;
	}
	std::string::size_type sz;   // alias of size_t
	return(std::stol (buffer,&sz));

}

void maincpp()
{


	int i,k;
	char msg1_speed[25] = "M1 select speed: ";
    char msg2_speed[25] = "M2 select speed: ";
    char msg3_speed[25] = "M3 select speed: ";
    char msg4_speed[25] = "M4 select speed: ";


	m1.Impuls_Width_4 = 0 * imp.Impuls_Delta;
	m2.Impuls_Width_4 = 0 * imp.Impuls_Delta;
	m3.Impuls_Width_4 = 0 * imp.Impuls_Delta;
	m4.Impuls_Width_4 = 0 * imp.Impuls_Delta;

	m1.Impuls_Start =  300;
	m2.Impuls_Start = 3110;
	m3.Impuls_Start = 5920;
	m4.Impuls_Start = 8730;

 // important to set basic impulse of all channels here!

	setzero(1);
	setzero(2);
	setzero(3);
	setzero(4);

// direction of all motors left
	HAL_GPIO_WritePin(Dir1_GPIO_Port, Dir1_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(Dir2_GPIO_Port, Dir2_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(Dir3_GPIO_Port, Dir3_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(Dir4_GPIO_Port, Dir4_Pin,GPIO_PIN_SET);

	TIM1->CR1 = TIM_CR1_CEN; // now from here start timer 1

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
			std::cout << "*         Motorcontrol for 4 Motors                            *\r\n";
			std::cout << "*                                                              *\r\n";
			std::cout << "****************************************************************\r\n";
			std::cout << "*                                                              *\r\n";
			std::cout << "*         1    Startup    M1-M4 (choose speed)                 *\r\n";
			std::cout << "*                                                              *\r\n";
			std::cout << "*         2    Shutdown   M1-M4 (from akt. speed)              *\r\n";
			std::cout << "*                                                              *\r\n";
			std::cout << "*         3    Change direction M1-M4                          *\r\n";
			std::cout << "*                                                              *\r\n";
			std::cout << "*              Stop all Motors with SPACE-Key                  *\r\n";
			std::cout << "*                                                              *\r\n";
			std::cout << "*         5    Terminate Program                               *\r\n";
			std::cout << "*                                                              *\r\n";
			std::cout << "****************************************************************\r\n";

		    abort1_flag = 0;
		    abort2_flag = 0;
		    abort3_flag = 0;
		    abort4_flag = 0;

			state = 1;

		 break;

		 case 1:

			 while(ready == 0)
			 {
			 };
		   	 ready = 0;
		   	 switch(test)
		  	 {
		    	 case '1':

     		        std::cout << "Choose  Motor 1 to  4 (1,2,3,4)\r\n";


     		       while(ready == 0)
     		       {
     		       };
     		  	 	ready = 0;
      		        if(test == '1')
      		     	{


      		     	   	  HAL_UART_Transmit(&huart5, (uint8_t *)&msg1_speed, 25, HAL_MAX_DELAY);
	    	    		  HAL_Delay(1000);
    	    	      	  HAL_GPIO_WritePin(Relais_GPIO_Port, Relais_Pin,GPIO_PIN_SET);  // Safetyrelais = ON

    	    	      	  speed1 = get_speed();
    	    	    	  speed1_valid = 1;

    	    	    	  if(speed1 < 0)
    	    	    	     speed1 = 0;
    	    	    	  if(speed1 > 2000)
    	    	    	     speed1 = 2000;

    	    	    	  std::cout << "\r\nSpeed M1 is: " << speed1 << "\r\n";

    	    	    	  if(HAL_GPIO_ReadPin(Dir1_GPIO_Port, Dir1_Pin))
    	    	    	  {
    	    	    	  	    std::cout << "M1 CCW\r\n";
    	    	    	  }
    	    	    	  else
    	    	    	  {
    	    	    	        std::cout << "M1 CW\r\n";
    	    	    	  }
    	       	    	  motor1_turns = 1;

    	    	    	  for(i = 0; i < (int) speed1; i++)
    	    	    	  {
    	    	    	      if(test == 0x20)
    	    	    	      {
    	    	    	 	      abort1_flag = 1;
    	    	    			  motor1_turns = 0;
    	    	    			  HAL_GPIO_WritePin(Relais_GPIO_Port, Relais_Pin,GPIO_PIN_RESET);  // Safetyrelais = OFF
    	    	    			  setzero(1);

    	    	    			  for(k = 0; k < 1;k++)        // avoid multiple messages
    	    	    			  {

    	    	    			      std::cout << "User Abort!\r\n";
    	    	    			      HAL_Delay(1000);
    	    	    			  }
    	    	    			  break;
    	    	    	      }


    	    	    	      setvalue(1,i);

    	    	    		  HAL_Delay(10);
    	    	    	  }

                          state = 0;
      		     	}

                    else if(test == '2')
      		        {

                    	  HAL_UART_Transmit(&huart5, (uint8_t *)&msg2_speed, 25, HAL_MAX_DELAY);
                    	  HAL_Delay(1000);
                    	  HAL_GPIO_WritePin(Relais_GPIO_Port, Relais_Pin,GPIO_PIN_SET);  // Safetyrelais = ON

                    	  speed2 = get_speed();
                   	   	  speed2_valid = 1;

                    	   if(speed2 < 0)
                    	        speed2 = 0;
                    	   if(speed2 > 2000)
                    	        speed2 = 2000;

                    	   std::cout << "\r\nSpeed M2 is: " << speed2 << "\r\n";

                    	   if(HAL_GPIO_ReadPin(Dir2_GPIO_Port, Dir2_Pin))
                    	   {
                    	  	    std::cout << "M2 CCW\r\n";
                    	   }
                    	   else
                    	   {
                    	        std::cout << "M2 CW\r\n";
                    	   }
                    	   motor2_turns = 1;

                    	   for(i = 0; i < (int) speed2; i++)
                    	   {
                    		   if(test == 0x20)
                    		   {
                    			   abort2_flag = 1;
                    	    	   motor2_turns = 0;
                    	    	   HAL_GPIO_WritePin(Relais_GPIO_Port, Relais_Pin,GPIO_PIN_RESET);  // Safetyrelais = OFF

                    	    	   setzero(2);

                    	    	   for(k = 0; k < 1;k++)
                    	    	   {

                    	    	       std::cout << "User Abort!\r\n";
                    	    	       HAL_Delay(1000);
                    	    	   }
                    	    	   break;
                    	       }


                    		    setvalue(2,i);

                    	    	HAL_Delay(10);
                    	   }

                    	   state = 0;
      		     	}
                    else if(test == '3')
      		     	{

                    	  HAL_UART_Transmit(&huart5, (uint8_t *)&msg3_speed, 25, HAL_MAX_DELAY);
                    	  HAL_Delay(1000);
                    	  HAL_GPIO_WritePin(Relais_GPIO_Port, Relais_Pin,GPIO_PIN_SET);  // Safetyrelais = ON


                    	  speed3 = get_speed();
                    	  speed3_valid = 1;

                    	  if(speed3 < 0)
                    	      speed3 = 0;
                    	  if(speed3 > 2000)
                    	      speed3 = 2000;

                    	  std::cout << "\r\nSpeed M3 is: " << speed3 << "\r\n";

                    	  if(HAL_GPIO_ReadPin(Dir3_GPIO_Port, Dir3_Pin))
                    	  {
                    	      std::cout << "M3 CCW\r\n";
                    	  }
                    	  else
                    	  {
                    	   	  std::cout << "M3 CW\r\n";
                    	  }
                    	  motor3_turns = 1;

                    	  for(i = 0; i < (int) speed3; i++)
                    	  {
                    	       if(test == 0x20)
                    	       {
                    	           abort3_flag = 1;
                    	           motor3_turns = 0;
                    	           HAL_GPIO_WritePin(Relais_GPIO_Port, Relais_Pin,GPIO_PIN_RESET);  // Safetyrelais = OFF

                    	           setzero(3);

                    	           for(k = 0; k < 1;k++)
                    	           {
                    	              std::cout << "User Abort!\r\n";
                    	              HAL_Delay(1000);
                    	           }
                    	           break;
                    	       }


                    	       setvalue(3,i);

                    	       HAL_Delay(10);
                    	   }
                   	       state = 0;
      		        }
                    else if(test == '4')
      		        {

                    	  HAL_UART_Transmit(&huart5, (uint8_t *)&msg4_speed, 25, HAL_MAX_DELAY);
                    	  HAL_Delay(1000);
                    	  HAL_GPIO_WritePin(Relais_GPIO_Port, Relais_Pin,GPIO_PIN_SET);  // Safetyrelais = ON

                    	  speed4 = get_speed();
                    	  speed4_valid = 1;

                    	  if(speed4 < 0)
                    	     speed4 = 0;
                    	  if(speed4 > 2000)
                    	     speed4 = 2000;

                    	  std::cout << "\r\nSpeed M4 is: " << speed4 << "\r\n";

                    	  if(HAL_GPIO_ReadPin(Dir4_GPIO_Port, Dir4_Pin))
                    	  {
                    	        std::cout << "M4 CCW\r\n";
                    	  }
                    	  else
                    	  {
                    	   	    std::cout << "M4 CW\r\n";
                    	  }
                    	  motor4_turns = 1;

                    	  for(i = 0; i < (int) speed4; i++)
                    	  {
                    	       if(test == 0x20)
                    	       {
                    	          abort4_flag = 1;
                    	          motor4_turns = 0;
                    	          HAL_GPIO_WritePin(Relais_GPIO_Port, Relais_Pin,GPIO_PIN_RESET);  // Safetyrelais = OFF

                    	          setzero(4);

                    	          for(k = 0; k < 1;k++)
                    	          {
                    	               std::cout << "User Abort!\r\n";
                    	               HAL_Delay(1000);
                    	          }
                    	          break;
                    	       }

                    	       setvalue(4,i);

                    	       HAL_Delay(10);
                    	   }
                    	   state = 0;
      		     	}

                    else
                    {
                        std::cout << "please only choose 1,2,3 or 4\r\n";
                    }
      		        state = 0;

         	     break;

	             case '2':

	                std::cout << "Choose Motor 1 to 4 (1,2,3,4)\r\n";

   	     		    while(ready == 0)
	                {
	            	};
	            	ready = 0;
	            	if(test == '1')
	            	{

	            		std::cout << "Shutdown M1\r\n";
	            		HAL_Delay(1000);
	            		if(speed1_valid)
	            		{
	            		     speed1_valid = 0;
	            		     motor1_turns = 1;
	            		     std::cout << "\r\nMotor 1 decreases from speed " << speed1  << "\r\n";
	            		     HAL_Delay(1000);
	            		     if(HAL_GPIO_ReadPin(Dir1_GPIO_Port, Dir1_Pin))
	            		     {
	            		  	    std::cout << "M1 CCW\r\n";
	            		     }
	            		     else
	            		     {
	            		        std::cout << "M1 CW\r\n";
	            		     }
	            		     HAL_Delay(1000);
	          		         for(i = (int) speed1; i > 0; i--)
	            		     {
	            		          if(test == 0x20)
	            		          {
	            		        	   abort1_flag = 1;
	            		        	   motor1_turns = 0;
	            		               HAL_GPIO_WritePin(Relais_GPIO_Port, Relais_Pin,GPIO_PIN_RESET);  // Safetyrelais = OFF

	            		               setzero(1);

	            		               for(k = 0; k < 1;k++)
	            		               {
	            		    	   	      std::cout << "User Abort!\r\n";
	            			  	          HAL_Delay(1000);
	            				       }
	            		   	           break;
	            			      }

	            		          setvalue(1,i);

	            		          HAL_Delay(10);
	            		     }
	          		         motor1_turns = 0;

	            		     state = 0;
	            		}
	            		else
	            		{
	            		  	std::cout << "no actual speed given for M1, so there is no shutdown possible!\r\n";
	            		    HAL_Delay(1000);
	            		}

	            	}
	            	else if(test == '2')
	            	{
	            		std::cout << "Shutdown M2\r\n";
	            		HAL_Delay(1000);
	            		if(speed2_valid)
	            		{
	            		    speed2_valid = 0;

	            		    std::cout << "\r\nMotor 2 decreases from speed " << speed2  <<"\r\n";
	            			HAL_Delay(1000);
	            			if(HAL_GPIO_ReadPin(Dir2_GPIO_Port, Dir2_Pin))
	            			{
	            		  	    std::cout << "M2 CCW\r\n";
	            			}
	            	        else
	            	        {
	            				std::cout << "M2 CW\r\n";
	            	        }
	            		    HAL_Delay(1000);
	            			for(i = (int) speed2; i > 0; i--)
	            			{
	            			    if(test == 0x20)
	            			    {
	            			    	 abort2_flag = 1;
	            			    	 motor2_turns = 0;

	            			        HAL_GPIO_WritePin(Relais_GPIO_Port, Relais_Pin,GPIO_PIN_RESET);  // Safetyrelais = OFF

	            			        setzero(2);

	            			        for(k = 0; k < 1;k++)
	            			        {
	            			   	        std::cout << "User Abort!\r\n";
	            				        HAL_Delay(1000);
	            				    }
	            				    break;
	            				}

	            				setvalue(2,i);

	            				HAL_Delay(10);
	            		    }
	            			motor2_turns = 0;

	            			state = 0;

	            		}
	            		else
	            		{
	            		    std::cout << "no actual speed given for M1, so there is no shutdown possible!\r\n";
	            		    HAL_Delay(1000);
	            		}

	            	}
	            	else if(test == '3')
	            	{
	            		std::cout << "Shutdown M3\r\n";
	               		HAL_Delay(1000);
	               		if(speed3_valid)
	               		{
	               	        speed3_valid = 0;

	               		    std::cout << "\r\nMotor 3 decreases from speed " << speed3 <<"\r\n";
	               			HAL_Delay(1000);

	               		    if(HAL_GPIO_ReadPin(Dir3_GPIO_Port, Dir3_Pin))
	               		    {
	               			   std::cout << "M3 CCW\r\n";
	               		    }
	               		    else
	               		    {
	               		       std::cout << "M3 CW\r\n";
	               		    }
	               			HAL_Delay(1000);
	               			for(i = (int) speed3; i > 0; i--)
	               			{
	               			   if(test == 0x20)
	               			   {
	               				    abort3_flag = 1;
	               					motor3_turns = 0;
	               			        HAL_GPIO_WritePin(Relais_GPIO_Port, Relais_Pin,GPIO_PIN_RESET);  // Safetyrelais = OFF

	               			        setzero(3);

	               			        for(k = 0; k < 1;k++)
	               			        {
	               			   	        std::cout << "User Abort!\r\n";
	               			            HAL_Delay(1000);
	               			        }
	               			        break;
	               			   }

	               			   setvalue(3,i);

	               			   HAL_Delay(10);
	               			}
	               		    motor3_turns = 0;

	               			state = 0;
	               		}
	               		else
	               	    {
	               		    std::cout << "no actual speed given for M1, so there is no shutdown possible!\r\n";
	               		    HAL_Delay(1000);
	               		}
	            	}
	            	else if(test == '4')
	            	{
	            		std::cout << "Shutdown M4\r\n";
	            		HAL_Delay(1000);
	            		if(speed4_valid)
	            		{
	            	   		speed4_valid = 0;

	            		    std::cout << "\r\nMotor 3 decreases from speed " << speed4 << "\r\n";
	            			HAL_Delay(1000);

	            			if(HAL_GPIO_ReadPin(Dir4_GPIO_Port, Dir4_Pin))
	            			{
	            		 	    std::cout << "M4 CCW\r\n";
	            			}
	            		    else
	            		    {
	            		        std::cout << "M4 CW\r\n";
	            		    }
	            			HAL_Delay(1000);

	            			for(i = (int) speed4; i > 0; i--)
	            	        {
	            			    if(test == 0x20)
	            			    {
	            			    	abort4_flag = 1;
	            			    	motor4_turns = 0;
	            			    	HAL_GPIO_WritePin(Relais_GPIO_Port, Relais_Pin,GPIO_PIN_RESET);  // Safetyrelais = OFF

	            			    	setzero(4);


	            			         for(k = 0; k < 1;k++)
	            			         {
	            				   	      std::cout << "User Abort!\r\n";
	            			 	          HAL_Delay(1000);
	            			         }
	            				     break;
	            			     }

	            			    setvalue(4,i);

	            				HAL_Delay(10);
	            		    }
	            			motor4_turns = 0;

	            		    state = 0;
	            		}
	            		else
	            		{
	            		    std::cout << "no actual speed given for M1, so there is no shutdown possible!\r\n";
	            		    HAL_Delay(1000);
	            		}
	            	}
	            	else
	            	{
	            	   std::cout << "please only choose 1,2,3 or 4\r\n";
	            	}

	            	state = 0;


		         break;

		         case '3':

		        	std::cout << "Choose Motor 1 to 4 (1,2,3,4)\r\n";

		        	while(ready == 0)
		        	{
		        	};
		        	ready = 0;
		        	if(test == '1')
		        	{

		        	  if(motor1_turns == 0)
		        	  {
		        	     std::cout << "Reverse Motor 1\r\n";
		        	     HAL_Delay(1000);

		        	 	 HAL_GPIO_TogglePin(Dir1_GPIO_Port, Dir1_Pin);
		        	    if(HAL_GPIO_ReadPin(Dir1_GPIO_Port, Dir1_Pin))
		        	    {
		        	 	    std::cout << "M1 CCW\r\n";
		        	    }
		        	 	else
		        	 	{
		        	 	   	std::cout << "M1 CW\r\n";
		        	 	}

		        	    HAL_Delay(1000);
		        	  }
		        	  else
		        	  {
		        	 	 std::cout << "Before reversing stop motor\r\n";
		        	     HAL_Delay(1000);
		        	  }
		        	}
		        	else if(test == '2')
		        	{
		        		 if(motor2_turns == 0)
		        		 {
		        		     std::cout << "Reverse Motor 2\r\n";
		        	   	     HAL_Delay(1000);

		        	   	     HAL_GPIO_TogglePin(Dir2_GPIO_Port, Dir2_Pin);
		        		    if(HAL_GPIO_ReadPin(Dir2_GPIO_Port, Dir2_Pin))
		        		    {
		        		 	    std::cout << "M2 CCW\r\n";
		        		    }
		        			else
		        		 	{
		        		 	   	std::cout << "M2 CW\r\n";
		        		 	}

		        		    HAL_Delay(1000);
		        	 	 }
		        		 else
		        		 {
		        		 	 std::cout << "Before reversing stop motor\r\n";
		        		     HAL_Delay(1000);
		        				        	  }
		        	}
		        	else if(test == '3')
		        	{
		        		 if(motor3_turns == 0)
		           	     {
		        		     std::cout << "Reverse Motor 3\r\n";
		        		     HAL_Delay(1000);

		        		 	 HAL_GPIO_TogglePin(Dir3_GPIO_Port, Dir3_Pin);
		        	   	     if(HAL_GPIO_ReadPin(Dir3_GPIO_Port, Dir3_Pin))
		        		     {
		        			     std::cout << "M3 CCW\r\n";
		        		     }
		        			 else
		        			 {
		        				 std::cout << "M3 CW\r\n";
		        		 	 }

		        			 HAL_Delay(1000);
		        		  }
		        	   	  else
		        	   	  {
		        	   	 	 std::cout << "Before reversing stop motor\r\n";
		        	   	     HAL_Delay(1000);
		        				        	  }
		        	}
		        	else if(test == '4')
		        	{
		        		 if(motor4_turns == 0)
		        	     {
		        		     std::cout << "Reverse Motor 4\r\n";
		        		     HAL_Delay(1000);

		        			 HAL_GPIO_TogglePin(Dir4_GPIO_Port, Dir4_Pin);
		        		     if(HAL_GPIO_ReadPin(Dir4_GPIO_Port, Dir4_Pin))
		        			 {
		        			    std::cout << "M4 CCW\r\n";
		        			 }
		        			 else
		        			 {
		        				std::cout << "M4 CW\r\n";
		        		 	 }

		        			 HAL_Delay(1000);
		        		  }
		        	   	  else
		        	   	  {
		        	   	 	 std::cout << "Before reversing stop motor\r\n";
		        	   	     HAL_Delay(1000);
		        	      }
		        	}
		        	else
		          	{
		           	   std::cout << "please only choose 1,2,3 or 4\r\n";
		          	}

		            state = 0;

		         break;

		         case 0x20:
		    		 if(motor1_turns == 1)
		    		 {
		    		   motor1_turns = 0;
		    		   std::cout << "M1 stop\r\n";
		    		   HAL_GPIO_WritePin(Relais_GPIO_Port, Relais_Pin,GPIO_PIN_RESET);  // Safetyrelais = OFF

		    		   setzero(1);

		    		   for(k = 0; k < 1;k++)
		    		   {
		    			 std::cout << "User Abort!\r\n";
		    		     HAL_Delay(1000);
		    		   }
		    		 }
		    		 else
		    		 {
		    			 std::cout << "Motor 1 is already stopped\r\n";
		    			 HAL_Delay(1000);
		    		 }
		    		 if(motor2_turns == 1)
		    		 {
		     		     motor2_turns = 0;
		    			 std::cout << "M2 stop\r\n";
		    			 HAL_GPIO_WritePin(Relais_GPIO_Port, Relais_Pin,GPIO_PIN_RESET);  // Safetyrelais = OFF

		    			 setzero(2);

		    			 for(k = 0; k < 1;k++)
		    			 {
		    			    std::cout << "User Abort!\r\n";
		    				HAL_Delay(1000);
		    		     }
		    		 }
		    	     else
		    		 {
		    			 std::cout << "Motor 2 is already stopped\r\n";
		    			 HAL_Delay(1000);
		    		 }
		    		 if(motor3_turns == 1)
		    		 {
		    		     motor3_turns = 0;
		    			 std::cout << "M3 stop\r\n";
		    			 HAL_GPIO_WritePin(Relais_GPIO_Port, Relais_Pin,GPIO_PIN_RESET);  // Safetyrelais = OFF

		    			 setzero(3);

		    			 for(k = 0; k < 1;k++)
		    			 {
		    			    std::cout << "User Abort\r\n";
		    				HAL_Delay(1000);
		    			 }
		    		 }
		    		 else
		    		 {
		    			 std::cout << "Motor 3 is already stopped\r\n";
		    			 HAL_Delay(1000);
		    		 }

		    		 if(motor4_turns == 1)
		       		 {
		       		     motor4_turns = 0;
		      			 std::cout << "M4 stop\r\n";
		       			 HAL_GPIO_WritePin(Relais_GPIO_Port, Relais_Pin,GPIO_PIN_RESET);  // Safetyrelais = OFF

		       			 setzero(4);

		       			 for(k = 0; k < 1;k++)
		       			 {
		       			    std::cout << "User Abort!\r\n";
		       				HAL_Delay(1000);
		       			 }
		       		 }
		       		 else
		       		 {
		       			 std::cout << "Motor 4 is already stopped\r\n";
		       			 HAL_Delay(1000);
		       		 }

		    		 state = 0;

		    	 break;

		         case '5':
			       HAL_GPIO_WritePin(GPIOG, Relais_Pin,GPIO_PIN_RESET);  // Safetyrelais = OFF
			       setzero(1);
			       setzero(2);
			       setzero(3);
			       setzero(4);
			       std::cout << "Programm has been terminated\r\n";
			       std::cout << "please apply Reset\r\n";
			       HAL_Delay(1000);
			       endflag = 1;

		    	   state = 0;

		         break;
		         default:
		        	 std::cout << "Input from  1 to 5 and Space-Key\r\n";
	  	         break;
		  	 }
        }
    }
}



