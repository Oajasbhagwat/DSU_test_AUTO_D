/*****************************************************************************
* | File        : LCD_Touch.h
* | Author      :   Waveshare team
* | Function    : LCD Touch Pad Driver and Draw
* | Info        :
*   Image scanning
*      Please use progressive scanning to generate images or fonts
*----------------
* | This version:   V1.0
* | Date        :   2017-08-16
* | Info        :   Basic version
*
******************************************************************************/
#ifndef __LCD_TOUCH_H_
#define __LCD_TOUCH_H_

#include "DEV_Config.h"
#include "LCD_Driver.h"
#include "LCD_GUI.h"

#define TP_PRESS_DOWN           0x80
#define TP_PRESSED              0x40
  
//Touch screen structure
typedef struct {
  POINT Xpoint0;
  POINT Ypoint0;
  POINT Xpoint;
  POINT Ypoint;
  unsigned char chStatus;
  unsigned char chType;
  int iXoff;
  int iYoff;
  float fXfac;
  float fYfac;
  //Select the coordinates of the XPT2046 touch
  //  screen relative to what scan direction
  LCD_SCAN_DIR TP_Scan_Dir;
 } TP_DEV;

//Brush structure
typedef struct{
  POINT Xpoint;
  POINT Ypoint;
  COLOR Color;
  DOT_PIXEL DotPixel; 
}TP_DRAW;
   // declarations of different functions 
void TP_GetAdFac(void);
void TP_Adjust(void);
void TP_Dialog(void);
void TP_DrawBoard(void);
void TP_Init( LCD_SCAN_DIR Lcd_ScanDir );
void Show_Timr5 (void);
void Updt_Displ(void);
void wrt_Pr(void);
void wrt_8255(  byte x,  byte Ad);
void del1 (void);     
byte RevBits (byte Num);
void A4_D1_DAC(byte x);
void Show_wt(char * st);
void Wr_A2A4(void);
void A4_Init(void);
void Wr2_pulse(void); // -Wr2 pulse to u1, 8255 of A4D1 card
void Updt_RecD(void); // update data received from  crm-500    
     //every bit or (later)every byte
void Erase1(void); // erase bit no.s, bytes 
 void Erase2(void); // erase bi,byte nos. in_Byte,Action taken1
float ScalcK(float L, float l); // calculate  spacing factor for L,l
float WcalcK(float av);    // for Wenner method , K= 2* Pye * av
void Kb_Action(); // keyBoard operatins. Ation taken on each key, in different F_modes
void A1_Power();    // show A1 card powered on status
 void check_Auto_D(); // check data from Auto-D if any
void Get_GPS(void);    // Get & print on screen GPS,Date & time
   void Get_GPS2(void);   // for checking baud rate
  void Updt_DigInpLvls(); // respond to D23 (Request from A1 card
void InitTimr(void);  // Timers 3,5 initialized
void get_Hex(byte x);   // e.g. C3,5F etc.
void show_ByRcvd();  // show 16 bytes received
                      // void del1();(defined aboe no. 10
 void ISR_clk_pin(void);  // interrupt on pin D21 (probably gets defined in 'AttachInterrupt)
  
 void calc_Res(void) ;   // recv. data on Serial2 // calculate & show Resistance
void Recv_Serial2(void);  // recv. data on Serial2
 void calc_Batt(void) ;  // calculate Batt. Voltage
 void curr_Status();   // show current magnitude, if current is flowing
 void Show_LlK(void);  //get L,l from E2prom2 & calculate K is defined
 void Alpha_1();        // 1st screen, 'Survey Status' (when 'Alpha' mode (='G') mode is defined)
 void Alpha_2();        // 2nd screen, 'Srvey closed' (when 'Alpha' mode (='G') mode is defined) 
  void Alpha_3();        // 3rd screen , 'New urvey opened'(when 'Alpha' mode (='G') mode is defined)
 void Normal_1();    // show normal measurement screen void 
 void Screen_1();   //measurement of Resistance
 void Screen_2();   //meas. of Resistance, c1-c2 open
 void Screen_3();   //start meas. of Resistance,
 void Screen_4();   // fill-up Batt,current Resist. etc.
 void Show_Eprom3( unsigned int L, unsigned int l);  //modifie version of Show_Eprom2(L,l)
 void Show_ResistData();   // show 15 bytes received from Seril2 (Auto_D)
  //--------------------------------------------------------------------------
 void entry_fnc_J();    // this shows  Survey Status screen while entering Fpr='J' mode
 void entry_fnc_J1();    // split screen 1
 void entry_fnc_J2();    // split screen 2
 void fnc_J2();       // no particular action
 void fnc_J3();     //increase Srv_No,make 'no. of readings' 'Spacing no. both = 0
 void fnc_J4();     //set schlumberger method (Surv_meth=1)
 void fnc_J5();     //set Wenner method (Surv_meth=2)
 void fnc_J6();     //set dipole-dipole method (Surv_meth=3)
 void stat_mod_J7();     //
 void stat_mod_J8();     //Status Modify action 7
 void stat_mod_J5();     //Status Modify action 5
 void stat_mod_J6();     //
 //---------------------------- 'I'-----------------
 void entry_fnc_I();   // for edit L,l
 void fnc_I2();       // if key 'next' is pressed
 void fnc_I3();       //if key 'prev' is pressed 
 void fnc_I4();       // if key-1 is pressed
 void fnc_I5();       // if key-4 is pressed
 void fnc_I6();       // if key-2 is pressed
 void fnc_I7();       // if key-5 is pressed
 void fnc_I8();       // if key-'save'is pressed
 void fnc_I9();       //if key-'.' is pressed,l-- 'delta'= 0.1 m.' 
 void fnc_I10();       //if key-'0' is pressed,l-- 'delta'= 1 m.' 
 //...................................end of 'I' case.....
 void entry_fnc_G();   // entry function for 'GPS' data, show 'GPS' continuously
  void fnc_G2();       // show  'GPS' for say, 10 Seconds
  void fnc_G3();        // show a symbol to indicate tha latest 'GPS' data was captured
  void fnc_G4();        // show Altitude  
  
  void Show_LlK2(unsigned int n);  // print L,l,K for sr. no. n                            
  void Show_LlK3(unsigned int n1,unsigned int L, unsigned int l);  
                    // Show L,l,K  after Spacing No++/or Spacing No-- (Note: L,l are 10* actual values)
  void entry_fnc_K();       // 'F'+5, show 'F5',Theta', 'show Rdng_No,L,tRho
   void fnc_K2();       //   show 'Next'
  void fnc_K3();        // show 'Previous'
  void fnc_K4();        // not needed 
  // ...............................................................
  void L_Init1(void);  // Initial screen for Survey mod select --schl/wenner/Dipole-Dipole
  void L_Scr_Schlum();  // Schlumberger,if key '1' pressed
  void L_Scr_Wenn();     // Wenner ,if key '2' pressed
  void L_Scr_Dip();    // Dipole-Dipole ,if key '3' pressed
  void L_No_change();  // no desire to change 'L' mode ,if key '4' pressed
  //....................................................................
  void select_print_case(float v);  //selct (7,5),(6,4) etc.
  void entry_fnc_H();   // 'F'+2,'Sigma'('S', 'Normal Survey' mode')
  void fnc_H2();    // Normal Survey mode: stop (Spacing no.)++ 
  void fnc_H3();    // Normal Survey mode: again allow (Spacing no.)++
  void fnc_H4();    // get redy to take next measurement
  void fnc_H5();    // go back to screen of entry_fnc_H
  void fnc_H6();    // call entry_fnc_H (like defining 'F2'  
  void fnc_H9();    // D27<-- '0' going pulde  
  void fnc_H_Prv();    // for key-Previous
  void fnc_H_Nxt();    // for key-next
  //.............................................................................................
  void entry_fnc_Q();   // 'F'+5,'T-special', Test mode Show Resistance only. No L,l,L, or 'Rho'
  void fnc_Q1();   //   key '5'  is pressed. return to 'entry_fnc_Q'
   void fnc_Q2();   // key 4 is pressed, presently no action
   //......................................................................
   void E2prom_Lltbl(unsigned int n1);   // write L,l values from RAM into EEprom
   void Reject_k();    // keys 3,5,6,7,8,9 not allowed immediately after 'F'   Reject_k ?
   float  DipcalcK(unsigned int a, unsigned int n) ; // K, Spacing factor for Dipole-Dipole  
   void Show_Recv_bytes(); // shoo received bytes on Serial monitor
   void E2prom_put() ;  // write bytes into EEPROM 
   void E2prom_get() ;  // read bytes from EEPROM
 // int entry_fnc_M();  // Edit L,l with 0~9 & 'dot' keys
 void wrt_Ll_E2prm(); // show L,l values from PRGMEM on 20x4 memory
 void Read_Ll_E2prm(); // read L,l values from EEPROM & show on 20x4 LCD
 void show_some();  //  A~T etc.
 //.................................................................
 void entry_fnc_M();   // prove that  values of L,l were stored correctly 
 void enter_L_l();   //  enter a new value of 'L' or 'l' to be stored in EEPROM
 unsigned int key_prsd_val1(byte key_pressed); // find integer value of key
 unsigned int k_value(byte keyBf0); // result: if key '2' is typedd , ithe function should return  -2-
 #endif
