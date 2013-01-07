/*
  Fivacopter v3.0 - January 2013  UNIVERSITY UPTC
  Copyright (c) 2013 BRIAN LARA, ELKYN FAGUA.  All rights reserved.
  An Open Source Arduino Mpide based UAV100.
  QUADROTOR FOUR SERVOS
  
  soporteboyaca@gmail.com 
   
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program. If not, see <http://www.gnu.org/licenses/>.
  
  BOARD chipKIT MAX32 4 servos (MODULES OCX) INTERRUPTIONS
  READING WITH 9 DOF IMU RAZOR A 57600 bps (putty hyperterminal faster sampling rate)
  GENERATION PWMS 4 servos (quadrotor)
  ELECTRONIC ENGINEERING
  (SOGAMOSO,PAIPA)COLOMBIA..
*/

//--------------------------------------INGRESANDO LIBRERIAS----------------------------------------------------------
#include "core.h"
////---------------------------------------DECLARANDO VARIABLES GLOBALES DE PROGRAMA------------ ----------------------

//----------------------variables para motores..........................
byte Comando;
float varpwm1;
float varpwm2;
float varpwm3;
float varpwm4;
//---------------------variables de imu 9dof razor......................
byte Trama[17];
boolean Error9DOF = false;
boolean Signoroll = true;
boolean Signopitch = true;
boolean Signoyaw = true;
boolean SignoAccelX = true;
boolean SignoAccelY = true;
boolean SignoAccelZ = true;
int roll = 0;
int pitch = 0;
int yaw = 0;
int rollsal = 0;
int pitchsal = 0;
int yawsal = 0;
int AccelX = 0;
int AccelY = 0;
int AccelZ = 0;
int AccelXsal = 0;
int AccelYsal = 0;
int AccelZsal = 0;
//--------------------------------------INGRESANDO CONFIGURACION DE DISPOSITIVOS-------------
void setup()
{
 SYSTEMConfigPerformance(80000000L);///----configuracion max32.. max speed 80mips
 INTEnableSystemMultiVectoredInt(); //-----habilitando multitarea en cpu 
 TurnOffSecondaryOscillator(); 
 
 //----------------------------------------configuracion motores................
 varpwm1=6480;//--
 varpwm2=6480;//-- para grantizar arranque de motores.
 varpwm3=6480;//--
 varpwm4=6480;//
 Serial.begin(57600);
 
 //----------------------------------------configurando razor imu................
 // Set Razor output parameters 
 Serial1.begin(57600);
 Serial1.write("#ob");  // Turn on binary output
 
 ///.......................................configurando interrupcion max32.......
 configurando_interrupciones();
}
//--------------------------------------PROGRAMA PRINCIPAL--------------------------------------------------------------------
void loop()
{
 leer_imu();
 control_motores(); 
 muestra();
}

//--------------------------------------DECLARANDO FUNCIONES SEGUNDARIAS DEL PROGRAMA-----------------------------------------

//---------------------------------Funcion de muestra angulos IMU  y duty PWM.........

void muestra()
{
 Serial.print("  ROLL= ");                             
 Serial.print(rollsal);
 Serial.print("  PITCH= ");
 Serial.print(pitchsal);
 Serial.print("  YAW= ");
 Serial.print(yawsal); 
 Serial.print("  (ciclo util)  =   ");
 Serial.print(varpwm1);
 Serial.print("   ");
 Serial.print(varpwm2);
 Serial.print("   ");
 Serial.print(varpwm3);
 Serial.print("   ");
 Serial.println(varpwm4);
}
//---------------------------------Funcion de control de motores serial-----------------------
void control_motores()
{
  if (Serial.available() > 0) 
              {
                Comando = Serial.read();
                if (Comando == 'q')  
                        {
                         varpwm1=varpwm1+500;
                        }
                if (Comando == 'a')  
                        {
                          varpwm1=varpwm1-500;
                        
                        }
              if (Comando == 'w')  
                        {
                         varpwm2=varpwm2+500;
                        }
               if (Comando == 's')  
                        {
                         varpwm2=varpwm2-500;
                        }
               if (Comando == 'e')  
                        {
                         varpwm3=varpwm3+500;
                        }
               if (Comando == 'd')  
                        {
                         varpwm3=varpwm3-500;
                        } 
               if (Comando == 'r')  
                        {
                         varpwm4=varpwm4+500;
                        } 
                        
               if (Comando == 'f')  
                        {
                         varpwm4=varpwm4-500;
                        } 
              varpwm1=constrain(varpwm1,6480, 22000);
              varpwm2=constrain(varpwm2,6480, 22000);
              varpwm3=constrain(varpwm3,6480, 22000);//----------LIMITADORES DE PWM PARA MOTORES BRUSLESS
              varpwm4=constrain(varpwm4,6480, 22000);
             } 

}

//---------------------------------Funcion lectura de imu 9Dof Razor--------------------
void leer_imu()
{
  if (Serial1.available() > 16) 
            {
             Trama[0] = Serial1.read();
             // Iniciamos a sacar Roll, Pitch, Yaw de la trama de datos....
             if (Trama[0] = '!' && Serial1.available() > 15) {
                                for (int i=1; i < 17; i++)   {
                                             Trama[i] = Serial1.read();  }
                                if (Trama[15] == '\r' && Trama[16] == '\n')   {
                                            Error9DOF = false;  }
                                else  {
                                            Error9DOF = true;   }
                                                     }}
             if (Error9DOF == false)  {
                               if (Trama[1] == 111 || Trama[1] == 110 || Trama[1] == 101 || Trama[1] == 100 ) {
                                            Signoroll = true;   }
                               else   {
                                            Signoroll = false;  }
                               if (Trama[1] == 111 || Trama[1] == 110 || Trama[1] == 11 || Trama[1] == 10 )   {
                                            Signopitch = true;  }
                               else   {
                                            Signopitch = false; }
                                      if (Trama[1] == 111 || Trama[1] == 101 || Trama[1] == 11 || Trama[1] == 1 ) {
                                            Signoyaw = true;    }
                               else   {
                                            Signoyaw =false;    }
                               if (Signoroll == true)    {
                                            roll = word(Trama[2],Trama[3]);   }
                               else   {
                                            roll = - word(Trama[2],Trama[3]); }
                               if (Signopitch == true)  {
                                            pitch = word(Trama[4],Trama[5]);  }
                               else   {
                                            pitch = - word(Trama[4],Trama[5]);}
                               if (Signoyaw == true)  {
                                            yaw = word(Trama[6],Trama[7]);    }
                               else   {
                                            yaw = - word(Trama[6],Trama[7]);  }
                       // No tenemos errores de lectura asi que calculamos AccelX, AccelY y AccelZ
                               if (Trama[8] == 111 || Trama[8] == 110 || Trama[8] == 101 || Trama[8] == 100 ) {
                                            SignoAccelX = true;               }
                               else   {
                                            SignoAccelX = false;              }
                               if (Trama[8] == 111 || Trama[8] == 110 || Trama[8] == 11 || Trama[8] == 10 )   {
                                            SignoAccelY = true;               }
                               else   {
                                            SignoAccelY = false;              }
                               if (Trama[8] == 111 || Trama[8] == 101 || Trama[8] == 11 || Trama[8] == 1 )    {
                                            SignoAccelZ = true;               }
                               else   {
                                            SignoAccelZ =false;               }
                               if (SignoAccelX == true)   {
                                            AccelX = word(Trama[9],Trama[10]);}
                               else   {
                                            AccelX = - word(Trama[9],Trama[10]);}
                               if (SignoAccelY == true)   {
                                            AccelY = word(Trama[11],Trama[12]); }
                               else   {
                                            AccelY = - word(Trama[11],Trama[12]);}
                               if (SignoAccelZ == true)   {
                                            AccelZ = word(Trama[13],Trama[14]); }
                               else   {
                                            AccelZ = - word(Trama[13],Trama[14]);}
                              rollsal=roll/100;
                              pitchsal=pitch/100;
                              yawsal=yaw/100;
                              AccelXsal=AccelX/100;
                              AccelYsal=AccelY/100;
                              AccelZsal=AccelZ/100;
                             }
}

//---------------------------------FUNCION DESACTIVA 2 OSCILADOR------------------------

void TurnOffSecondaryOscillator() 
{
  unsigned int dma_status;
  unsigned int int_status;
  mSYSTEMUnlock(int_status, dma_status);
  OSCCONCLR = _OSCCON_SOSCEN_MASK;
  mSYSTEMLock(int_status, dma_status);
}

//-----------------------------------FUNCION DE CONFIGURACION DE INTERRUPCIONES MAX32----
void configurando_interrupciones()
{
 ConfigIntTimer3(T3_INT_ON | T3_INT_PRIOR_3);
 OpenTimer3(T3_ON | T3_PS_1_8 | T3_SOURCE_INT, F_CPU / 4 / 61 / 9);  ///modificar para cambiear la frecuencia del pwm para 275hz.
 ConfigIntOC1(OC_INT_ON | OC_INT_PRIOR_3);
 ConfigIntOC2(OC_INT_ON | OC_INT_PRIOR_3);
 ConfigIntOC3(OC_INT_ON | OC_INT_PRIOR_3); 
 ConfigIntOC4(OC_INT_ON | OC_INT_PRIOR_3);  
 OpenOC1(OC_ON | OC_TIMER3_SRC | OC_PWM_FAULT_PIN_DISABLE, 0, 0);
 OpenOC2(OC_ON | OC_TIMER3_SRC | OC_PWM_FAULT_PIN_DISABLE, 0, 0);
 OpenOC3(OC_ON | OC_TIMER3_SRC | OC_PWM_FAULT_PIN_DISABLE, 0, 0);
 OpenOC4(OC_ON | OC_TIMER3_SRC | OC_PWM_FAULT_PIN_DISABLE, 0, 0);   
 IPC6SET = 0x001f0000;//set priority to 7 sub 4 0x00060000 for priority 1 sub 0
 IFS1CLR = 0x0001; //clear the interupt flag bit
 IEC1SET= 0x0001; // Enable Change Notice interrupts
}
//-------------------------------FUNCION EXTERNA DE C INTERRUPCIONES--------------------------------
extern "C"
{
    void __ISR(_TIMER_3_VECTOR,ipl3) pwmOn(void)
             {
              IEC1CLR= 0x0001; // Disable Change Notice interrupts
              mT3ClearIntFlag();  // Clear interrupt flag
              
              SetDCOC1PWM(varpwm1);
              SetDCOC2PWM(varpwm2);
              SetDCOC3PWM(varpwm3);
              SetDCOC4PWM(varpwm4);
                                  
              IEC1SET= 0x0001; // Enable Change Notice interrupts
             }
} // end extern "C"
//-------------------------------------FIN DEL PROGRAMA----------------------------------
