#include <SoftwareSerial.h>

  // #include <TinyGPS++.h>

//

          //#include <DS1307RTC.h>
          //#include <Key.h>
          //#include <Keypad.h>  // this does not work
    //#include "DEV_Config.h"     //----- Dubai7_AutoD -23/jan/2023-,6 pm---------  
  
#include <SPI.h>
#include <SD.h>
#include <avr/pgmspace.h>   // <avr/pgmspace.h> is needed to define constants in 'Program memory'
#include <EEPROM.h>
#include <LiquidCrystal.h>
          //#include "Debug.h"
#include <stdlib.h>
#include <math.h>         
//#include <usersetup.h>
    //    create an object
   //File myF;
//---------------------------------------------------
  // drn=1 means rectangle has been drawn once
  volatile  byte Led = 01;  volatile long i11, i13,i14,i14old,i15=0 ; volatile unsigned long i16=0;;  //volatile char ch2='B';
  volatile   byte PrA1,PrC1 ;   volatile int x11=100,x21=100,y11=70;
   volatile byte  by1=0x08,by2,by3, tb1,tb2=157,tb3 ,SD_ok ;     String s1 ;    // moved to LCD_Touch.c++
         
                // ----- - --------23/August/2022 ~3 pm-----------------------
                     //extern char st1[15],st2[15],st4[15] ; extern unsigned int x5,y5; extern float vr1,vr2,vr3,vr4,vr5;   // extern check_Auto_D() ;
               //............................................................................
   //char Rx;
  unsigned int ya13 =250;  char Rx;
   const int Err7 = 28,Ierr= 29 ; 
   const int rs = 28, en = 30, d4 = 32, d5 = 34, d6 = 36, d7 = 38; // In Hht1 (Hand-held terminal for crm-500,RS,En,d[4~7] are 28,30, 32,34,36,38
  LiquidCrystal lcd1(rs, en, d4, d5, d6, d7); 
   volatile unsigned int Colr[] = {0xFEA0, 0xFD20, 0x867C, 0xBDAD, 0xBFB3, 0xD5B6, 0xC618, 0xDEFB, 0xE7FF} ; //  9 colours
      //volatile unsigned int  Tsat = 0, NtRn1 = 5, NtRn2 = 5  , ht1 = 13, EAd = 0 , EAd1, EAd2, EAd3, EAd4, EAd5, EAd6, EAd7, EAd8, EAd9, EAd10, tEA ; // EEprom area
       // extern volatile unsigned int Colr[]; extern volatile byte A1PowN;
      

    
                         
                         
      
    #define otpin0 29  // scan output code on 4 pins
    #define otpin1 31
    #define otpin2 33
    #define otpin3 35
    #define Kbin0  37  // 4 return lines
    #define Kbin1  39
    #define Kbin2  41    
    #define Kbin3  43
   // -------------------------------------for keys 1~F 12 keys-----
    #define  k_1 0x11   // '1'
#define  k_4 0x21   // '4'
#define  k_7 0x41   // '7'
#define  k_dot 0x81 // '.'
#define  k_2 0x12  // '2'
#define  k_5 0x22   // '5'
#define  k_8 0x42   // '8'
#define  k_zr 0x82  // '0'
#define  k_3 0x14 // '3' 
#define  k_6 0x24   // '6'
#define  k_9 0x44   // '9'
#define  k_F 0x84   // 'F'
     
     const int SDC_CS_PIN = 53;    //  chip select pin of SD card
     const int Tch_Screen_SD_Cs =5;
   //...............................................
  //  ---------------------GPS variables -------------------------------------------------------------  
      
     //__________________________________________________________________________________________________________ 
      
         
       
      
            
      //  */
  //Serial.println("Drawing.... 6 ..");

  
  


 //   --------------- ********************************** -----------
   //  **************************************
   
   
   // 
  //********************************************************************************************************
    //END FILE
  //*********************************************************************************************************/
