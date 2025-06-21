


/*****************************************************************************
  | File          :   LCD_Touch.c
  | Author      :   Waveshare team
  | Function    :   LCD Touch Pad Driver and Draw
  | Info        :
    Image scanning
       Please use progressive scanning to generate images or fonts
                                                                            
  | This version:   V1.0u
  | Date        :   2017-08-16B
  | Info        :   Basic version

******************************************************************************/
#include "LCD_Touch.h"  //---------AutoD_DSU8------------------
#include "Debug.h"
#include <EEPROM.h>
#include <LiquidCrystal.h>
//#include <TinyGPS.h>
//#include <TinyGPS++.h>     //#include <var_const.h>
#include <avr/pgmspace.h>  // <avr/pgmspace.h> is needed to define constants in 'Program memory' 8/march/2023
#include <arduino.h>
#include <stdlib.h>
#include <SD.h>
//------------------------------------------------------------
// wrLlSD=2 (do not write  file on SD) when wrLlSD=2
//.............................................
//void fnc_J5();     //set Wenner method (Surv_meth=2)
File myF;  //myF is a FILE object
           // TinyGPS gps;  // gps is a  TinyGPS object Baud rate for GPS chip is 9600
//HardwareSerial  GPSSer(19,18);  // D19 is Rx1, D18 is Tx1 GPSSer is object of SoftwareSerial
//

extern LCD_DIS sLCD_DIS;
static TP_DEV sTP_DEV;
static TP_DRAW sTP_Draw;

#define wsl 15                                                          // no.  of charrs received from wiighing machine
const int rs = 28, en = 30, d4 = 32, d5 = 34, d6 = 36, d7 = 38;         //
                                                                        //extern const int rs, en, d4, d5, d6, d7;
extern LiquidCrystal lcd1;                                              //(rs, en, d4, d5, d6, d7); //lcd1 is not being treated as Global object !
const unsigned int PLp[] PROGMEM = { 1, 2, 3, 5, 10, 13, 15, 20, 23 };  // not used, (numbers in PROG memory)

const unsigned int PLt[] PROGMEM = { 15, 20, 25, 30, 40, 50, 60, 80, 100, 100, 120, 150, 200, 250, 250, 300, 400, 500, 600, 800, 1000, 1000, 1200, 1500, 1800,
                                     2000, 2000, 2500, 3000, 3000, 3500, 4000, 4500, 5000, 5000, 5500, 6000, 6500 };  //(total 37 integers),10* actual values:-=1.5,2,2.5,3,4 m. etc.
const unsigned int Plt[] PROGMEM = { 5, 5, 5, 5, 5, 5, 5, 5, 5, 20, 20, 20, 20, 20, 50, 50, 50, 50, 50, 50, 50, 100, 100, 100, 100, 100, 200, 200, 200, 400, 400, 400, 400, 400, 500,
                                     500, 500, 500 };                                                                                                     // 0~37 (total 38 integers),10* actual values
const unsigned int Wennat[] PROGMEM = { 20, 40, 60, 80, 100, 120, 140, 160, 180, 200, 220, 240, 260, 280, 300, 320, 340, 360, 380, 400, 440, 480, 520 };  // 0~22,23 values: 2~40 m. chang= 2m., 44~52. cgange by 4m. at each step
const unsigned int Dipat[] PROGMEM = { 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5 };                                       // 0~24, 25 values. like L
const unsigned int Dipnat[] PROGMEM = { 2, 3, 4, 5, 6, 2, 3, 4, 5, 6, 2, 3, 4, 5, 6, 2, 3, 4, 5, 6, 2, 3, 4, 5, 6 };                                      // 0~24,25 values, like l
const char* const RangeSw[] PROGMEM = { "Bat", "mOhm", "ohm", "kOhm", "mV", "V" };                                                                        // Range switch
volatile unsigned int tEA;
volatile byte nby8 = 0, nby7, old7, old8, IeNo;  // IeNo -- Sr. no. of current (0~5
volatile byte bk1 = 0x08, bk2 = 0x19, APrC, bk3, bk4, bk5, bk6, change = 0, change2 = 0, PrADt, PrCDt, Cap = 0;
volatile byte bk7[] = { 0X05, 0X0D, 0x19, 0x3F, 0x7D, 0xFA };  // bk7[0~5] are (bk[i]*0.08 --0.4,1.04,2,5.04,10,20 mAmp
volatile byte n2, n3, n4, n5, n6, n7, n8, n9 = 1, n10 = 2, n11 = 3, n12, n13 = 1, n14 = 0, n15, n16, n17, n18, n19 = 0, n20 = 0, n21, n22, n23, n24, n25, updt_flg = 0;
volatile unsigned int m1 = 0, m2 = 0, m3, m4, m5, m6 = 0, m7, m8, m9 = 1, m10 = 1, m11, m12 = 1, m13 = 0, m14, m15, k1, k2, k3, k4 = 0, k5, k6 = 0, im1, im2 = 20, im3, im4, im5;                              // some integer values
volatile unsigned int Surv_meth = 1, SpScr = 2, WaL;                                                                                                                                                           // =1 means -Schlumberger, ==2 means -Wenner, ==3 means -Dipole-Dipole ,SpScr- split screen
                                                                                                                                                                                                               //volatile POINT x1, y1, x2 = 20, x3 = 20, x4 = 20, x5 = 70, x6 = 80, x7, x8, x9, x10 = 100, y2 = 30, y3, y4, y5 , y6 = 60, y7, y8, y10 = 120, xr1, yr1, quit ;
volatile POINT x1, y1, x2 = 20, x3 = 20, x4 = 20, x5 = 70, x6 = 80, x7, x8, x9, x10 = 100, y2 = 30, y3, y4, y5, y6 = 60, y7, y8, y10 = 120, xr1, yr1, quit, dx1, dx2, dx3, dx4, dx5, dy1, dy2, dy3, dy4, dy5;  // dy-some distance
volatile POINT xr2 = 5, xr3 = 45, xr4 = 20, xr5, yr2 = 50, yr3 = 200, yr4 = 170, yr5;
volatile POINT x12, x13, x14, x15, y12, y13, y14, y15, xi1[6] = { 300, 300, 300, 300, 300, 280 }, yi1[6] = { 50, 70, 90, 110, 130, 160 }, xw1 = 10, yw1 = 190, xv1 = 60, yv1 = 80, xv2 = 60, yv2 = 100;  // x11 defined in 1 st program group
volatile unsigned int vts[5] = { 0, 0, 0, 0, 0 }, vtstot = 0, j2a, j3a;
volatile unsigned char ch1 = 0x41, ch2 = 0x61, ch3, ch4 = 'j', ch5, ch6, ch7 = 'd', ch8, ch9, ch10, ch11, j, j1 = 0, j2 = 0, j3 = 0, j4, j5, j6, j7, j8, j9, j10, chr, TchScrch;  // ch1='A' & ch2='a', ch4=107 (97+10 ('k') )
volatile unsigned char j2old = 0, j3old = 0, j4old = 0, j5old = 0, j6old = 0, j2new = 0, j3new = 0, j4new = 0, j5new = 0, j6new = 0;
// ch4='j' means write to A2,A4 once only, 'm' means show 'weight',
volatile unsigned char dimR;                       // dimR='milli'/blank/'kilo'
volatile unsigned char chs1 = 0, chs2, Kbkaz = 0;  //chs1-- old state of touch pad, chs2-- present state, Kbkaz='a'~'z'means some key is pressed. Kbkaz=0 means action has been taken
volatile unsigned char sgsy[8] = { '+', '+', '-', '-', '-', '-', '+', '+' };
volatile byte PrA, PrC;                    // data to be written into Arduino ports A & C
volatile byte SPrA = 04, SPrB = 00, SPrC;  // data to be wrtten into 'Slave' Ports (of 8255 on A4-D1 card) A,B,C , V5 =V5Low
volatile unsigned int wlk1[] = { 1, 2, 4, 0x08, 0x10, 0x20, 0x40, 0x80, 0x100, 0x200, 0x400, 0x800, 0x1000, 0x2000, 0x4000, 0x8000 };
volatile float vr1 = 49.37, vr2, vr3, vr4, vr5, Vinp, KNp = 5000.0, KNm = 5000.0, Voffs = +0.0, SPVolt;  //27; ( for Gain=5.0 ),
// {KNp=50.5,KNm=5021.5, Voffs=0.0;  // 0.031 for Gain =0.5}
volatile float IeMag[] = { 0.4, 1.0, 2.00, 5.0, 10.0, 20.0 }, Ieval = 2.0, wgt1, wgt2, icnA1;  // Ieval= 2.0 mAmp
volatile char st1[wsl] = { "+000085.12  g" }, st2[wsl], st3[wsl], st4[wsl] = { "" }, st5[wsl], st6[wsl], st7[wsl], st8[wsl], st9[wsl], st10[wsl];
volatile char Blnk[10] = { "          " }, Fpr, N_dot = "No_dot", dot_symbol = N_dot, Y_dot = "dot";  // 'Q' means it is in 'Test' mode 10 blanks,Fpr=0 or 'F',or 'G'....'P' (11 nos.)
const char FName[] = { "SUV9.txt" };
volatile char FName2[10] = { "Srv3.csv" }, StrName1[20], StrName2[20], StrName3[20];
volatile unsigned int Noffp = 100, Noffn = 0, Npp = 500, Npm = 0;  //(say, Npp= 50, Noffset+,- Np =,- (for Gain.5 -- Noffp =155
const int SerD = 22, SClk = 23, Lat = 24, SyncP = 25, Wr2 = 27;    // Serial  data out -D22,Serial clock pin=D23,LatchPin=D24. SyncP for OscScope
//Wr2 is connected to -Wr pin of 8255(U1 on card A4D1)
volatile byte A2_Cntrl, A4_Dt, A4_Cntrl, A4_KAv, A4_Rl, A4_DAC;

const String inStr = "499.98", Str3, Str5 = "Vp1p2", Str6 = "Resist", Str7, Str11 = "Sur4.csv";
volatile String Str10, Str12, Str13, Str14, Str15 = ("Sur4.csv"), Str16("#"), Str17, Str18, Str19, Str20;                                         // volatile Strings, "Suv" -- Survey
                                                                                                                                                  /*
 //volatile unsigned int Colr[] = {0xFEA0, 0xFD20, 0x867C, 0xBDAD, 0xBFB3, 0xD5B6, 0xC618, 0xDEFB, 0xE7FF} ; //
// 0-Gold,1-Orange,2-Sky Blue,3-Dark Khaki,4-Pale Green,5-Tan,6-Silver,7-Gainsboro, 8-Light Cyan
   */
volatile unsigned int Tsat = 0, NtRn1 = 5, NtRn2 = 5, ht1 = 13, EAd = 0, EAd1 = 0, EAd2, EAd3, EAd4, EAd5, EAd6, EAd7, EAd8, EAd9, EAd10, EAd11;  //Colr[];

volatile float Resist[20], yfct = 0.4, Vp1p2, Res, Resx, Resm, fNsig2, BattV, tRes1, fltLv, fltlv, tRohm;                   // yfct=100/Npls[0] (should be Npls[1]
                                                                                                                            //----------------------------------------------------temporary:reduced from 70 to 5 --6/Dec/2022---------------
volatile float tRes2[5];                                                                                                    //   tRes2[70];
                                                                                                                            //.............................................................................................................
volatile signed int Npls[5], Ndots[5], Ycoord[5], NdtxA = 160, Nsig = 20, Nsig2, Nbck = 150, Nbckdt = 100, Slp = 1, IeNo2;  // Slp - slope =1 means Vp1p2 changes by Slp every Sec.
extern volatile byte SD_ok, sh_sg = 0;                                                                                      //(rs, en, d4, d5, d6, d7);
// extern LiquidCrystal lcd1  ;
volatile byte in_Byte, last_lvl = 0, pres_lvl = 0, start_sending = 0, Recv = 0, First_Pls = 0, Ignr_pulses = 1, A1ReqO = 1, A1ReqN = 0, A1Powold = 0, A1PowN = 0, A1PSw, IeStat;
volatile byte Ack1, Actkn1 = 0, Actkn2 = 0;  // Acknoledge signal = D24 output. WHen this is '0', Auto-D starts sending data to DSU
volatile byte Recv_Buff1[100], RcBf_R1[100], GPS_rdng = 0;
volatile unsigned long F_cnt, Resint, ln8;                                                                                  //no. of pulses in 4 Sec, Resint- 'resistance' in int. form
volatile unsigned int icn[] = { 1000, 500, 250, 100, 50, 20, 10 }, CurrMag[] = { 5, 10, 20, 50, 100, 250, 500 }, RdNo = 0;  // 'multipliers' for 5,10,20, 50, 100,250,500 mA
volatile unsigned int Cycls[] = { 1, 4, 16, 64 }, PrCycl_No;                                                                // PrCycl_No-- present cycle no. viz. 1~4/16/64
volatile unsigned int IeMag2[] = { 0, 1, 2, 5, 10, 15, 20, 30, 40, 50, 70, 100, 200, 250, 300, 400, 500 };                  // for AutoC,nos-- 0,1~16 no. 0 is --null

volatile unsigned long Ntv[20], Ntvtot, Ltm2, Ltm1, tint, timr1, timr2 = 0, timr3, timr4, timr5, timr6, timr7, timr8, timr3old = 0, timr5old, timr6old, tmcn, tmcn5 = 0, timc1 = 0;  // Ltm1/2 & tint used for measuring time interval in milliSec.

volatile float Lv[] = { 1.5, 3, 5, 8, 10, 12, 15, 20, 25, 30, 40, 50, 60, 65, 70, 75, 80, 85, 90, 95, 100, 110, 120, 130, 140, 150, 160, 170, 180, 190, 200, 210, 230, 250, 270,
                        290, 300, 320, 340, 360, 380, 400, 420, 440, 460, 480, 500 };  //0~47 (total-48 float values)

volatile unsigned int Lt[] = { 15, 20, 25, 30, 40, 50, 60, 80, 100, 100, 120, 150, 200, 250, 250, 300, 400, 500, 600, 800, 1000, 1000, 1200, 1500, 1800,
                               2000, 2000, 2500, 3000, 3000, 3500, 4000, 4500, 5000, 5000, 5500, 6000, 6500 };  // 0~37 (total 38 integers),10* actual values
volatile unsigned int lt[] = { 5, 5, 5, 5, 5, 5, 5, 5, 5, 20, 20, 20, 20, 20, 50, 50, 50, 50, 50, 50, 50, 100, 100, 100, 100, 100, 200, 200, 200, 400, 400, 400, 400, 400, 500,
                               500, 500, 500 };       // 0~37 (total 38 integers),10* actual values
volatile unsigned int wrLlSD = 2;                     // if weLlSD==2,do not write L,l (float)  or a (float on SD
volatile float lv[] = { 0.5, 2, 5, 10, 15, 20, 30 };  // 0~ 6 (total-7)

volatile float Rest[5], Rho[5], Kv[5], Gain[4] = { 5, 0.5, 0.05, 0.005 }, tRes, tRho, MRdNmb[5], MLv[5], Mlv[5], MKv[5], MRes[5], MRho[5];        // ;  // Gain -- 4 values
volatile float Kvt;                                                                                                                               // a K-value
volatile byte Sccd[4] = { 0x01, 0x02, 0x04, 0x08 }, keyBf0 = 0, freezeSP = 0;                                                                     // Scan code to be changed every 10 mSec.
volatile unsigned int TotRd = 0, StRdNo = 1, Rds = 0, RdLNo = 0, StNos[] = { 1, 7, 13, 19, 25, 31, 37, 43, 49 }, Srv_No = 2, Survey_No_Stat = 0;  //('StRdNo' not used, tot readings,starting read. no.reading sets,starting no.s
volatile unsigned int trow, Llmax = 20, LlTbEd = 1, RdgEd = 0, SrvEd = 0;
volatile unsigned int LNo[80], lNo[4] = { 0, 1, 2, 3 }, ChPts[3] = { 3, 8, 13 }, SNo, LCNo[100], lCNo[100], RdNo1, MRdNo, MRdN1 = 0, Show_Alt = 1, ShSpcRd = 0, ShRdg = 1, CNomax;  //ShSpcN=1 means show spacings
volatile unsigned int tlim2, tlim3, tlim4, tlim5, tlim6, tlim7, tlim8, Lnlth = 0, LRdSr = 0, RdSr, IntSz, FltSz;                                                                    // Lnlth-- no. of char.s of type '0'~'9'/','/'.'. LRdSn- Last Reading Srl. No.(0~N)
volatile unsigned int SpcN, LSpcN, tm2, tm3, tm4, tm4old, TstE2p = 0, tm5, tm6, tm7, tm8, Ltm3, Ltm4, Ltm5;                                                                         // Spacing no, Last Spac. tm5,6 used in Timer5 Interrupt used
volatile unsigned int LSpcN2, LRdSr2, alph_st = 0, Norm_st = 0, StRecrds = 5, MSpcN, MLlNo, MLvt, Mlvt, tMLvt, tMlvt, chbfr = 0;                                                    //StRecrds-no. of records stored in  EAd6~EAd7 region. ; 3                         // LSpcN2,LRdSr2-last Reading & spacing nos., MSpcN-spacing no.as stored 'Readings list'.chbfe==0 means change by 1 m.,else 0.1 m.
volatile unsigned int xw2 = 70, yw2 = 50, nw2 = 1, noByRc;                                                                                                                          // no. of Bytes Recvd.
volatile unsigned int LCNomax = 100, lCNomax = 8, Lint1, Lint2, Lint3, lint1, lint2, lint3, Ldig1, ldig1, LlpSz, LNomax = 38, amax, Dip_a, Dip_n;                                   //Dipoe_Dipole metods' 'a' & 'n'LCNomax will get redefined
volatile unsigned int xn1, timr7_flag;                                                                                                                                              // used for printing 4 values of Resistance in 4-cycle mode
volatile unsigned int tn1 = 0, tn2, tn3, t_transf = 0, no_rdgs = 0, kpr = 0, kpr_key_2 = 1, F_kpr;                                                                                  //tn1=0~14 (15 nos.),15~20(6 nos.),21~35 (15 nos.)& for 2nd,3rd & 4th cycles --(36~50)(51~65),(66~80),no. of readings
// kpr--no. of 'F' group keys. 0-F not pressed after 'Powe On', 1- 'F' pressed, 2-1 of (1~9,.) keys pressd after 'F',F_kpr means 'F' pressed
volatile float fact1, fact2, fact3, fact4, fact5 = 0, tenf[] = { 1, 10, 100 };  // different factors of Resistance 'Resm'
volatile unsigned int StRd[] = { 1, 2, 3, 4, 5 }, StL[] = { 15, 30, 50, 100, 150 }, StRd1, StL1, StL2, NRec = 5;
// NRec='no. of records'  StRd-stored readings No. StL1- Stored L (10* actual value) But in Dipole-Dipole method 2 values'a'(StL1) & 'n' StL2 are read (insead of 1)
volatile float StRho[] = { 30.53, 25.41, 17.62, 20.28, 21.73 }, StRho1;  // Rho values
#define Kbin0 37                                                         // 4 return lines
#define Kbin1 39
#define Kbin2 41
#define Kbin3 43
#define otpin0 29  // scan output code on 4 pins
#define otpin1 31
#define otpin2 33
#define otpin3 35
//---------------------------------------
//  define key-codes -------------------

#define k_1 0x11    // '1'
#define k_4 0x21    // '4'
#define k_7 0x41    // '7'
#define k_dot 0x81  // '.'
#define k_2 0x12    // '2'
#define k_5 0x22    // '5'
#define k_8 0x42    // '8'
#define k_zr 0x82   // '0'
#define k_3 0x14    // '3'
#define k_6 0x24    // '6'
#define k_9 0x44    // '9'
#define k_F 0x84    // 'F'

#define k_pr 0x18                                                                                  // "Prev"
#define k_nx 0x28                                                                                  // "Next"
#define k_cl 0x48                                                                                  //"Clear"
#define k_sv 0x88                                                                                  //"Save"
volatile int ldg2, lmin2, ldg0 = 0, ldg1 = 0, lmin0 = 0, lmin1 = 0, ldg0L, ldg1L, lmin0L, lmin1L;  //  ( ldg2,lmin2  not used presently)


// ...............................................
//  for KeyBoard :. . . LKSt
volatile byte Kbout[4], sccdV[4] = { 0x0E, 0x0D, 0x0B, 0x07 }, RetL = 0, RetL2 = 0x41, print_save = 0, no_char = 0, decim_pt = 0;
// no_char is 'no. of chars. typed 0~9 and 'decimal point'  decim_pt means that user has typed 'decimal point'
volatile struct {
  byte KSt;
  byte j;
  byte DbD[4];
  byte LKSt;
  byte DefKSt[2];
} Cl[4];  //
//Structure for keyboard parsing. Cl[4] is object, LKSt -- is last value of Key State
volatile byte RtLD, Curr_Sw, Range_Sw, Cycl_Sw = 2, Meas_Sw;  // Cycl_Sw=2 means 4-cycle mode  rtLD is read from kkbin0~3
volatile byte tick1[8] = { 0, 1, 2, 0x14, 8, 0, 0 }, tick2[8] = { 0, 1, 3, 0x16, 0x1c, 8, 0 };
//-------------------------------Variables defined in sketch-GPS2, tab-GPS function -----------------------------------------------------------------------
volatile float Altit, lat, lon, ltg2, lsec2, lx2, frdg2, frmin2;  // Of these only lat,lon have been used presently
volatile float err0, err1, Rlsec0, Rlsec1;                        // err=(present)lsec0- (Reference ) Rlsec0. fAltit-Altitude(float)
volatile float frdg0, frdg1, lx0, lx1, frmin0, frmin1, lsec0, lsec1, ltg0, ltg1;
volatile unsigned int LnNo = 0, tLn = 0, Yr, YrL, EYr[4];
volatile byte Mn, Dte, Hr, mint, Scnd, EMn[4], EDte[4], EHr[4], Emint[4], EScnd[4], DeciSec, Cr, DteL, MnL, HrL, mintL, ScndL;  // E- extra
volatile unsigned long LAltit, f_age = 0, sz1, sz2, tsz1, tsz2;                                                                 // sz1,sz2 for file size, tsz1,tsz2-- total sizes
//volatile byte Date, Mn  ; // use new names (byte) Dte,Mn

//volatile char st1[20];
//volatile unsigned int Colr[]={0xFEA0, 0xFD20,0x867C, 0xBDAD,0xBFB3,0xD5B6, 0xC618,0xDEFB, 0xE7FF} ; //
// 0-Gold,1-Orange,2-Sky Blue,3-Dark Khaki,4-Pale Green,5-Tan,6-Silver,7-Gainsboro, 8-Light Cyan
//....................................................................................
volatile unsigned int i1, i2, i3, i4, i5, EdSpc_No = 2;  // 'Edit_Spacing_Srl. No.
//....................................................................................
volatile float latitude, longitude;  // from
//
//extern volatile float vr2,vr3,vr4;

/*************************************************
        ******************************
  

/*******************************************************************************
  function:
        Paint the Delete key and paint color choose area
*******************************************************************************/
void TP_Dialog(void) {
  LCD_Clear(Colr[7]);  // 7- Gainsboro,,4-Pale green
  DEBUG("Drawing...\r\n");
  //Horizontal screen display
  if (sLCD_DIS.LCD_Dis_Column > sLCD_DIS.LCD_Dis_Page) {
    //Clear screen
    GUI_DisString_EN(sLCD_DIS.LCD_Dis_Column - 60, 0,
                     "CLEAR", &Font16, RED, BLUE);
    //adjustment
    GUI_DisString_EN(sLCD_DIS.LCD_Dis_Column - 120, 0,
                     "AD", &Font24, RED, BLUE);
    //choose the color ( Actualyy, draw 5 coloured rectangles ) // Blue(dark) rect. 30x50
    GUI_DrawRectangle(sLCD_DIS.LCD_Dis_Column - 30, 20,
                      sLCD_DIS.LCD_Dis_Column, 70,
                      BLUE, DRAW_EMPTY, DOT_PIXEL_1X1);
    //GUI_DisString_EN(sLCD_DIS.LCD_Dis_Column - 30, 30,"Close", &Font24, Colr[7], BLUE);   // close Survey
    GUI_DrawRectangle(sLCD_DIS.LCD_Dis_Column - 30, 80,  // Green rect. 30x50
                      sLCD_DIS.LCD_Dis_Column, 130,
                      GREEN, DRAW_EMPTY, DOT_PIXEL_1X1);
    // GUI_DisString_EN(sLCD_DIS.LCD_Dis_Column - 30, 145,"Open", &Font24, GREEN, BLUE);  // open new Survey
    GUI_DrawRectangle(sLCD_DIS.LCD_Dis_Column - 30, 140,  // Red rect. 30x50
                      sLCD_DIS.LCD_Dis_Column, 190,
                      RED, DRAW_EMPTY, DOT_PIXEL_1X1);
    // GUI_DisString_EN(sLCD_DIS.LCD_Dis_Column - 30, 210,"Continue", &Font24, GREEN, BLUE);
    GUI_DrawRectangle(sLCD_DIS.LCD_Dis_Column - 30, 200,  // Yellow rect. 30x50
                      sLCD_DIS.LCD_Dis_Column, 250,
                      YELLOW, DRAW_EMPTY, DOT_PIXEL_1X1);
    GUI_DrawRectangle(sLCD_DIS.LCD_Dis_Column - 30, 260,  // Black rect. 30x50
                      sLCD_DIS.LCD_Dis_Column, 310,
                      BLACK, DRAW_EMPTY, DOT_PIXEL_1X1);
    //--------------------------------------------------now 8/April/2020 -----  old(16/oct/'18) ------
    yr1 = 210;
    if (Cap == 0) chr = ch2;
    else chr = ch1;  // (ch1='A'(41h),ch2='a'(61h) )use 270 for only 1 line height of Font16.,use 210 for 4 lines
    //    draw keyboard   //only 1 line -- A ~ K  {A~Z in (4 lines )}      ( 16 dots)
    for (xr1 = 10; xr1 < 430; xr1 = xr1 + 40) {
      GUI_DrawRectangle(xr1, yr1, xr1 + 30, yr1 + 25, GREEN, DRAW_EMPTY, DOT_PIXEL_1X1);                  // 1st line
      GUI_DrawRectangle(xr1 - 1, yr1 - 1, xr1 + 30 + 1, yr1 + 25 + 1, BLACK, DRAW_EMPTY, DOT_PIXEL_1X1);  // Black border
      GUI_DisChar(xr1 + 7, yr1 + 4, chr, &Font16, WHITE, BLUE);
      chr++;
    }
    yr1 += 30;  // (A~K)(11)chars
    //GUI_DrawLine( 10, 160, 400 ,160 ,   BLUE, LINE_SOLID, DOT_PIXEL_1X1); // x-axis
    //GUI_DrawLine( 10, 10,  10,260 ,   BLUE, LINE_SOLID, DOT_PIXEL_1X1);  // y-axis
    //   -----------------------8/april/2020  -  -----------------------------

    for (xr1 = 10; xr1 < 430; xr1 = xr1 + 40) {
      GUI_DrawRectangle(xr1, yr1, xr1 + 30, yr1 + 25, GREEN, DRAW_EMPTY, DOT_PIXEL_1X1);                  // 2nd line
      GUI_DrawRectangle(xr1 - 1, yr1 - 1, xr1 + 30 + 1, yr1 + 25 + 1, BLACK, DRAW_EMPTY, DOT_PIXEL_1X1);  // Black border
      GUI_DisChar(xr1 + 7, yr1 + 4, chr, &Font16, WHITE, BLUE);
      chr++;
    }
    yr1 += 30;  // (L~V) 11 chars

    for (xr1 = 10; xr1 <= 290; xr1 = xr1 + 40) {
      GUI_DrawRectangle(xr1, yr1, xr1 + 30, yr1 + 25, GREEN, DRAW_EMPTY, DOT_PIXEL_1X1);                  // 3rd line
      GUI_DrawRectangle(xr1 - 1, yr1 - 1, xr1 + 30 + 1, yr1 + 25 + 1, BLACK, DRAW_EMPTY, DOT_PIXEL_1X1);  // Black border
      GUI_DisChar(xr1 + 7, yr1 + 4, chr, &Font16, WHITE, BLUE);
      chr++;
    }  //   (W~Z),(7) chars

    GUI_DrawRectangle(xr1, yr1, xr1 + 30, yr1 + 25, GREEN, DRAW_EMPTY, DOT_PIXEL_1X1);
    xr1 += 40;                                                                                          // only a blank rectangle. xr1=40 for next key
    GUI_DrawRectangle(xr1 - 1, yr1 - 1, xr1 + 30 + 1, yr1 + 25 + 1, BLACK, DRAW_EMPTY, DOT_PIXEL_1X1);  // Black border
    x5 = 20;
    y5 = 40;
    // ----------------------------------------------------8/april/2020----
    // GUI_DisNum (x5,y5+30,5278,&Font16, WHITE,BLUE );  // for testing 'display  an integer'
    dtostrf(vr1, 7, 3, st1);
    vr2 = vr1 / 2.5;  // vr2 not used
    //vr2 = -( ( (27.0/KNm)+0.030) ); dtostrf (vr2,9,4,st2);    // for testing 'display  a float number'
    //GUI_DisString_EN (x5+10+80,y5-15, &st1[0], &Font16, WHITE,BLUE);    // show vr1   // */
    /*  //  show vr2  */

    //    ---------------- 8/April/2020  -----------------------
    /*  // show "Frq+, Frq-
      n2=n1; GUI_DisNum (70,100,n9, &Font16, WHITE,BLUE);
           GUI_DisNum (70,145,n10, &Font16, WHITE,GRED);
      GUI_DisString_EN (5,y6, "Frq-" ,&Font16, WHITE,BLUE);
      GUI_DisString_EN (5,y6+20, "Frq+",&Font16,WHITE,BLACK);
    */
    //   -----------------------------------------------------
    //    -------following initialization is done in Setup() . So it is not neede here
    // PORTA = 0x80; PrC=0x07; PORTC = PrC; wrt_Pr();     // define all 3 ports of 8255 (on A4-D1 card) as output ports
    // key-'k', 8255-A1,A0 <-- 11b , 'command' 80h   written into 8255

    //============================================================================
  } else {  //Vertical screen display
    GUI_DisString_EN(sLCD_DIS.LCD_Dis_Column - 60, 0,
                     "CLEAR", &Font16, RED, BLUE);
    GUI_DisString_EN(sLCD_DIS.LCD_Dis_Column - 120, 0,
                     "AD", &Font24, RED, BLUE);
    GUI_DrawRectangle(20, 20, 70, 70, BLUE, DRAW_FULL, DOT_PIXEL_1X1);
    GUI_DrawRectangle(80, 20, 130, 70, GREEN, DRAW_FULL, DOT_PIXEL_1X1);
    GUI_DrawRectangle(140, 20, 190, 70, RED, DRAW_FULL, DOT_PIXEL_1X1);
    GUI_DrawRectangle(200, 20, 250, 70, YELLOW, DRAW_FULL, DOT_PIXEL_1X1);
    GUI_DrawRectangle(260, 20, 310, 70, BLACK, DRAW_FULL, DOT_PIXEL_1X1);
  }
}
// ---------------------------------------------------
// --    draw keyboard  a~z or A~Z
// ......................................................
void Dr_Kb()  // draw keyBoard
//......................................................................................................................
{
  yr1 = 210;
  if (Cap == 0) chr = ch2;
  else chr = ch1;  // use 270 for only 1 line height of Font16 is 16 dots ch1='A', ch2='a'
  for (xr1 = 10; xr1 < 430; xr1 = xr1 + 40) {
    GUI_DrawRectangle(xr1, yr1, xr1 + 30, yr1 + 25, GREEN, DRAW_EMPTY, DOT_PIXEL_1X1);                  // 1st line
    GUI_DrawRectangle(xr1 - 1, yr1 - 1, xr1 + 30 + 1, yr1 + 25 + 1, BLACK, DRAW_EMPTY, DOT_PIXEL_1X1);  // Black border
    GUI_DisChar(xr1 + 7, yr1 + 4, chr, &Font16, WHITE, BLUE);
    chr++;
  }
  yr1 += 30;  // (A~K)(11)chars
  //GUI_DrawLine( 10, 160, 400 ,160 ,   BLUE, LINE_SOLID, DOT_PIXEL_1X1); // x-axis
  //GUI_DrawLine( 10, 10,  10,260 ,   BLUE, LINE_SOLID, DOT_PIXEL_1X1);  // y-axis
  //   -----------------------8/april/2020  -  -----------------------------

  for (xr1 = 10; xr1 < 430; xr1 = xr1 + 40) {
    GUI_DrawRectangle(xr1, yr1, xr1 + 30, yr1 + 25, GREEN, DRAW_EMPTY, DOT_PIXEL_1X1);                  // 2nd line
    GUI_DrawRectangle(xr1 - 1, yr1 - 1, xr1 + 30 + 1, yr1 + 25 + 1, BLACK, DRAW_EMPTY, DOT_PIXEL_1X1);  // Black border
    GUI_DisChar(xr1 + 7, yr1 + 4, chr, &Font16, WHITE, BLUE);
    chr++;
  }
  yr1 += 30;  // (L~V) 11 chars

  for (xr1 = 10; xr1 <= 290; xr1 = xr1 + 40) {
    GUI_DrawRectangle(xr1, yr1, xr1 + 30, yr1 + 25, GREEN, DRAW_EMPTY, DOT_PIXEL_1X1);                  // 3rd line
    GUI_DrawRectangle(xr1 - 1, yr1 - 1, xr1 + 30 + 1, yr1 + 25 + 1, BLACK, DRAW_EMPTY, DOT_PIXEL_1X1);  // Black border
    GUI_DisChar(xr1 + 7, yr1 + 4, chr, &Font16, WHITE, BLUE);
    chr++;
  }  //   (W~Z),(7) chars

  GUI_DrawRectangle(xr1, yr1, xr1 + 30, yr1 + 25, GREEN, DRAW_EMPTY, DOT_PIXEL_1X1);
  xr1 += 40;                                                                                          // only a blank rectangle. xr1=40 for next key
  GUI_DrawRectangle(xr1 - 1, yr1 - 1, xr1 + 30 + 1, yr1 + 25 + 1, BLACK, DRAW_EMPTY, DOT_PIXEL_1X1);  // Black border
}
// ................................end of drw KeyBoard ,,,,,,,,,,,,,,,,,,,

/*...................................................................
  initialisation for 8255(U1 of A4D1 Card)
  ...........................................................................*/
void A4_Init() {
  //A2_ntrl= 0x00 ;   //0x02; means A2- all relays off and Gain=0.005. 03h means Gain=5.0, 02h- 0.005, 01h- 0.05 * 00h - Gain=0.5
  //A4_Cntrl= 0x03; // A4- Command port address= 1, 1b
  //A4_Dt= 0x80;   // Command for 8255(U1 of A4D1 card)
  //Wr_A2A4(); Wr2_pulse() ; // Write into 3 Shift registers & then a -Wr2 pulse to U1,8255
  //----------------------------------------------------------------
  // void fnc_J4();  // trying to remove the error :--undefined reference to 'fnc_J4('

  InitTimr();    // Timers 3,5,0 initialized
  interrupts();  //enable all global interrupts
  j2 = 0;
  j3 = 0;
  in_Byte = 0;  //j2- no. of bits,j3 - no.of bytes, D22 logic level is shifted into 'in_byte'
  A1ReqN = digitalRead(23);
  A1ReqO = A1ReqN;  // initialization -----no longer used after Sept 2022--------------------
  //-- done later at line 831 A1PowN= digitalRead(40); A1Powold= A1PowN ; if ( A1Powold== 0 && A1PowN== 1) n20=0; // initialization of n20,--- =1 means ,A1 card is presently powered on -----
  // A1PowN=1 means A1 is presently powere on. So keyboard scanning does not interfere with 'read Seriai2'
  //----------------------------following 6 lines are used to indicate that Auto-C is powered On----------------------
  if (A1PowN == 1) {
    GUI_DrawRectangle(420 - 1, 30 - 1, 420 + 1 * 16, 30 + 32, Colr[7], 1, DOT_PIXEL_DFT);  // erase  rect.A1 card is on,disable keyb. scanning
    GUI_DisNum(420, 46, 2, &Font16, Colr[7], RED);
  }  //n
  else {
    GUI_DrawRectangle(420 - 1, 30 - 1, 420 + 1 * 16, 30 + 32, Colr[7], 1, DOT_PIXEL_DFT);  // erase  rect. A1 card is off. Let keyb. scanning be enabled
    GUI_DisNum(420, 30, 1, &Font16, Colr[7], BLUE);
  }  //
  //......................................................................
  //---------------------------------------------------------------L
  tlim2 = 800 / 4;
  tlim3 = (tlim2 + 1) / 4;
  tlim4 = 3 * tlim3;  // tlim2,3,4 = 200,50,150  used in Timer3 10 mSec interrupt
  //Str10  = "Suv"; SLRdSrtr10+= ".csv";
  // ----------- 1-time ---- initialization done ---- All this gets overridden ~ 50 lines later
  IntSz = 2;
  FltSz = 4;  //IntSz = sizeof(int); FltSz = sizeof (float);
              /*
  EAd1 = 0; EAd2 = EAd1 + (20 * 2); EAd3 = EAd2 + (20 * 4);  EAd4 = EAd3 + (20 * 2); // EAd2=40, EAd3=(40+80)=120, EAd4=(120+40)=160
  EAd5 = EAd4 + (200 * 2);   ; EAd6 = EAd5 + (60 * 4); //EAd5=(160+400)=560(100*4 integers),  EAd6= =(560+200)=760 (50 L,l combinations * 4 bytes each)
  EAd7 = EAd6 + (250 * IntSz) + 125 * FltSz; // EAd7=EAd6+(500+500)= (800+1000)=1800 (125 intgers+125 integers(spare)+125 Float ? Addresses in E2PROM.
  EAd8 = EAd7 + 220 * IntSz; // EAd8=EAd7+440=(1800+440)=2240,
  EAd9 = EAd8 + 20 * IntSz; // EAd9=EAd8+40= (2240+40)=2280
  EAd10 = EAd9 + 100 * IntSz;  // EAd10=EAd9 + 200 bytes=2280 + 200 = 2480.  2480~ 4000 (1520 bytes ; free)  
        //  */
  EAd1 = 0;
  EAd2 = 40;
  EAd3 = 120;
  EAd4 = 160;
  EAd5 = 460;
  EAd6 = 760;
  EAd7 = 1060;
  EAd8 = 2060;
  EAd9 = 2280;
  EAd10 = 2480;  // EAd1 ,2,3,4,5,6,7,8 redefined 6/March/2023, Monday
                 // 2480 ~ 4000 (2520 Bytes free)
                 /*
    tEA = EAd1; Srv_No = 1; EEPROM.put ( tEA, Srv_No);  // Survey no. =1 stored at addr=EAD1 - 1 time initialization
    tEA=EAd1+(3*2) ;LRdSr2=0;  EEPROM.put ( tEA, LRdSr2);  //   write 0 at  integer 3 tEA=EAd1+(1*2)  
    tEA=EAd1+(4*2) ; LSpcN2=0; EEPROM.put ( tEA, LSpcN2);  //  write 0 at  integer 4 tEA=EAd1+(2*2) ;
      
    tEA=EAd1+(5*2) ; StRecrds=0; EEPROM.put ( tEA, StRecrds);//no. of records stored in  EAd6~EAd7 region     
    tEA=EAd1+(6*2) ; Surv_meth=1; EEPROM.put ( tEA, Surv_meth); // 1-- means Schlumberger, 2-Wenner
        
    tEA=EAd1+(7*2) ; Survey_No_Stat=0; EEPROM.put ( tEA, Survey_No_Stat); // 0-- means No Survey is open presently
    //  should use EEPRM.get EEPRPm.put or (EEPROM.update)E2 to modify the value
      //    */
  Srv_No = 8;
  LSpcN = 7;
  LRdSr = 9;  //these values should get redefined when the statements LSpcN2<-- E2prom[tEA] etc are executed (2~4 lines from here)
  // EAd1~2: 20 integers, EAD2~3: 20 Float nos., EAD3~4: 20 integers// --------------- Get Survey no. from EEPROM  ------------------
  //delay(1000);
  // -------------read from E2PROM -------------------
  //  /*
  //---------------------------------
  tEA = EAd1;
  EEPROM.get(tEA, Srv_No);  // Survey no. gets updated in mode F4
  //---------------------------Both (LRdSr2 & LSpcN2) become 0, when new survey is opened------------------
  //-----------EAd1+1*IntSz & EAd1+2*IntSz  , unused
  tEA = EAd1 + (3 * IntSz);
  EEPROM.get(tEA, LRdSr2);  //  Read  integer 3 as LRdSr2 earlier: tEA=EAd1+(1*IntSz) ; Last Reading Sr. No & l. spac. no.
  tEA = EAd1 + (4 * IntSz);
  EEPROM.get(tEA, LSpcN2);  //  Read 4th integer as LSPcN2  Earlier: tEA=EAd1+(2*IntSz) ; Last Spac. no. (both get updated after each reading
  tEA = EAd1 + (5 * IntSz);
  EEPROM.get(tEA, StRecrds);  //no. of records stored i  EAd6~EAd7 region
  tEA = EAd1 + (6 * IntSz);
  EEPROM.get(tEA, Surv_meth);  //  Survey method - 1/2/3 -Schlum/Wenn/DipoleDipole
  tEA = EAd1 + (7 * 2);
  EEPROM.get(tEA, Survey_No_Stat);          // 0-- means No Survey is open presently, 1- means 'Srv_No' is the latest survey
  Str11 = "Srv" + String(Srv_No) + ".csv";  // e.g. Srv5.csvStr11="Srv"+String(Srv_No) + ".csv";// e.g. Srv5.csv
    //  */
  // --------........................-----------------

  //..............................................
  A1_Power();  // this defines: A1PowN. A1PowN==1mens A1 card is powered On
  /*
    if (A1PowN == 1) TIMSK3 = 0; else {
    TIMSK3 = 0;  // if A1PowN ==1,disable Keyboard scanning. if A1PowN=0, then enable keyboard scanning
    TIMSK3 |= (1 << OCIE3A);
    }
  */

  /*     m5=5; m6=25; m7=155; m8=195;
    j10=0; GUI_DisNum(20+(60*j10),35, m5, &Font12, WHITE,BLUE); j10++; GUI_DisNum(20+(60*j10),35, m6, &Font12, WHITE,BLUE); j10++;
    GUI_DisNum(20+(60*j10),35, m7, &Font12, WHITE,BLUE); j10++; GUI_DisNum(20+(60*j10),35, m8, &Font12, WHITE,BLUE);
    //print 4 nos. at x=20,80 ,140 & 200 and y=100. m5,6,7,8 are printed correctly as 5,25,155,195

    m1=5; m2=210; m3=1453; m4=3798;
    tEA= EAd1; EEPROM.put ( tEA, m1);  tEA= EAd1+(1*IntSz); EEPROM.put(tEA, m2);  tEA= EAd1+(2*IntSz); EEPROM.put ( tEA, m3);
    tEA= EAd1+(3*IntSz); EEPROM.put ( tEA, m4);  // write 4 nos. E2p viz. 5,210,1453& 3798
    tEA= EAd1; EEPROM.get ( tEA, m5);  tEA= EAd1+(1*IntSz); EEPROM.get(tEA, m6);  tEA= EAd1+(2*IntSz); EEPROM.get ( tEA, m7);
    tEA= EAd1+(3*IntSz); EEPROM.get ( tEA, m8);  //read these nos. into m5~8 & print the nos.

    j10=0; GUI_DisNum(20+(60*j10),200, m5, &Font12, WHITE,BLUE); j10++; GUI_DisNum(20+(60*j10),200, m6, &Font12, WHITE,BLUE); j10++; // at y=120
    GUI_DisNum(20+(60*j10),200, m7, &Font12, WHITE,BLUE); j10++; GUI_DisNum(20+(60*j10),200, m8, &Font12, WHITE,BLUE); j2++; //print 4 nos. at x=20,80 ,140 & 200
    // now m5,6,7,8 show EEPROM values viz. 5,210,1453,37,98  ------ EEProm works correctly----
  */
  //.............................................................
  // --------------------calculation of about float 100 Lv[100] & lv[100] values---19/july/2022 --------------------------------------------------
  for (j2 = 0; j2 <= LNomax - 1; j2++) {  //LNomax = 38 ,presently
    LNo[j2] = j2;
    // --- printing suppressed----
    // if (j2 <10)  GUI_DisChar(20+(10*j2),140, LNo[j2]+'0' , &Font12, WHITE,BLUE);  // only 10-dot spacing (single digit)
    // else GUI_DisNum(120+ (20*(j2-10)),140, LNo[j2], &Font12, WHITE,BLUE);  // 20-dot spacing (2 digits)
  }  // end of j2 0~,++loop
  j2 = 0;
  j4 = 0;
  j5 = 0;
  dy1 = 0;
  dx1 = 0;
  for (j3 = 0; j3 <= LNomax - 1; j3++) {  //note:LNomax=38 (because, only 38 values have been defined (as of 13/Nov/2022, Sunday
    j5 = j3 + j2;
    LCNo[j5] = LNo[j3];
    lCNo[j5] = j2;
    if (j3 > 19) {
      dy1 = 35;
      dx1 = 23;
    }
    //   --- printing suppressed----
    //if (j5<10)  { GUI_DisChar(20+(10*j5),120+dy1, LCNo[j5]+'0', &Font12,Colr[2], BLACK); GUI_DisChar(20+(10*j5),135+dy1, lCNo[j5]+'0', &Font12,Colr[2], BLACK);}  // only 10-dot spacing
    // if (j5>=10 && j5<=22) {GUI_DisNum(120+ (20*(j5-10)),120+dy1, LCNo[j5], &Font12,Colr[2], BLACK); GUI_DisNum(120+(20*(j5-10)),135+dy1, lCNo[j5], &Font12,Colr[2], BLACK);}  // 20-dot spacing
    // if (j5>22)  {GUI_DisNum(10+ (18*(j5-22)),120+dy1, LCNo[j5], &Font12,Colr[2], BLACK); GUI_DisNum(10+(18*(j5-22)),135+dy1, lCNo[j5], &Font12,Colr[2], BLACK);} //

    //if (LNo[j3] == ChPts[j2]) {j2++; j4++; j5=j3+j2; LCNo[j5]=LNo[j3]; lCNo[j5]=j2;  // j5 has changedPrint these Numbers in red
    vr3 = lv[j2] * 18.0;
    if ((Lv[LNo[j3]] < vr3) && (vr3 <= Lv[LNo[j3 + 1]])) {  // one 'chpts' (change point detected)
      j2++;
      j4++;
      j5 = j3 + j2;
      LCNo[j5] = LNo[j3];
      lCNo[j5] = j2;  // ( j5 has changed, j2 (which was 0 originally,has changed, so j5=j3+j2 changes too

      // --- printing suppressed----
      // if (j5<10)  { GUI_DisChar(20+10*(j5),120+dy1, LCNo[j5]+'0', &Font12,Colr[7], RED); GUI_DisChar(20+(10*j5),135+dy1, lCNo[j5]+'0', &Font12,BLACK, YELLOW); }  // only 10-dot spacing
      // if (j5>=10 && j5<=22) {GUI_DisNum(120+ (20*(j5-10)),120+dy1, LCNo[j5], &Font12,Colr[7], RED); GUI_DisNum(120+20*(j5-10),135+dy1, lCNo[j5], &Font12,BLACK, YELLOW);}  // 20-dot spacing
      //  if (j5>=22) {GUI_DisNum(10+ (18*(j5-22)),120+dy1, LCNo[j5], &Font12,Colr[7], RED); GUI_DisNum(10+(18*(j5-22)),135+dy1, lCNo[j5], &Font12,BLACK, YELLOW);}  //
    }  // end of 'j5 has changed'
  }    // end of j3 loop
  LCNomax = j5 + 1;
  lCNomax = j2 + 1;  // now,LCNomax overrides the value declared in variable list. if LNomax=47, then LCNomax= ~47+8=55
  // GUI_DisNum(10+70, 135+dy1+15, LCNomax, &Font12,Colr[2], BLACK); GUI_DisNum(10+ 100, 135+dy1+15, lCNomax, &Font12,Colr[2], BLACK);
  //  ............................end of study of L_l_Table ......................

  //  -------------------calculate all K values ---Scalck(float,float)defined on L 874----------
  for (j5 = 0; j5 <= LCNomax - 1; j5++) Kv[j5] = ScalcK(Lv[LCNo[j5]], lv[lCNo[j5]]);  // example:  LCNo= 0,1,2,3, 3,4,5 lCNo= 0,0,0,0, 1,1,1
  //  .................................end of calculate all K values.........................
  // ..................................End of 19/july/2022 .part...............................

  //  -------------------calculate all K values ---Scalck(float,float)defined on L 874----------
  for (j5 = 0; j5 <= 22; j5++) Kv[j5] = ScalcK(Lv[LCNo[j5]], lv[lCNo[j5]]);  // example:  LCNo= 0,1,2,3, 3,4,5 lCNo= 0,0,0,0, 1,1,1
                                                                             //  .................................end of calculate all K values.........................

  // -------- ----------------------------Define Survey file e.g. Srv15.csv ---------------------------------------------
  // ....................................................................end of old (prior to June 2022) Lv[LCNo[1~20]] values
  //-----------------------------------------------------------------------------
  del1();
  //.............................................................................
  IntSz = 2;
  FltSz = 4;
  // tEA = EAd1;  EEPROM.get ( tEA, Srv_No);  // Survey no. =1 stored at addr=EAD1 - 1 time initialization
  // tEA=EAd1+(1*IntSz) ;  EEPROM.get ( tEA, LRdSr);  // Last Reading Sr. No & l. spac. no.
  // tEA=EAd1+(2*IntSz) ;  EEPROM.get ( tEA, LSpcN);
  // -----------------------------------show Survey no. Spacing,Reading, 'file name' ?
  //EAd2 = 40; EAd3 = 120; EAd4 = 160; EAd5 = 560; EAd6 = 800;  IntSz = 2; FltSz = 4; // EAd7,EAd8,EAd9 as per ~line 870-880 ,LSpcN2=0; EAd5=560;
  //GUI_DisString_EN (50+12*m6,120,Str11 , &Font16, BLACK,BLUE);keyBf0=0;m6++;
  //lcd1.setCursor(0, 1); lcd1.print("Srv no");  lcd1.print(Srv_No); lcd1.print("   "); //Srvey no., next Spacing no. & Reading no.
  //lcd1.setCursor(11, 1);lcd1.print("Sp"); lcd1.print(LSpcN2 + 1); lcd1.setCursor(16, 1);lcd1.print("Rd"); lcd1.print(LRdSr2 + 1); // last reading Sr. No.
  for (m7 = 0; m7 <= 8; m7++) FName2[m7] = Str11[m7];
  //lcd1.setCursor(0, 2);lcd1.print("FNme");lcd1.print(Str11); // copy const string 'Str11' into const 'char' array
  GUI_DrawRectangle(50, 160, 50 + 9 * 12, 180 + 16, Colr[7], DRAW_FULL, DOT_PIXEL_DFT);  // erase rect
  //-------show new names ----------------------------------------------
  // /*
  GUI_DisString_EN(165, 187, FName2, &Font12, YELLOW, BROWN);  // SHOW File name
  GUI_DisString_EN(165, 175, "LSpcN2", &Font12, YELLOW, BROWN);
  GUI_DisNum(240, 175, LSpcN2 + 1, &Font16, YELLOW, BROWN);  // show LSpcN2(new name)Last Spacing no.
  GUI_DisString_EN(270, 175, "LRdSr2", &Font12, YELLOW, BROWN);
  GUI_DisNum(360, 175, LRdSr2 + 1, &Font16, YELLOW, BROWN);  // show LRdSr2(new name)Last Rdg.no.
  GUI_DisString_EN(385, 175, "EAd5", &Font12, YELLOW, BROWN);
  GUI_DisNum(435, 175, EAd5, &Font16, YELLOW, BROWN);  // EAd5: L & l are stored fom hereonward
                                                       //   */
                                                       //---------------------------------------------------------------
  del1();
  //--------------------------------------------------------------------


  //..................................................end of 'define Survey file...............................
  //---------------------------------write  into 'SD' . ---- Then read back & show it----------------------------
  xv1 = 60;
  yv1 = 80;  // xv2=60,yv2=100 -initially
             // ---------------------------------------First, write data, '1.5,0.5'
             //myF = SD.open("Srv6.csv", FILE_WRITE);
             //if (myF) {myF.print(1.5); myF.print(","); myF.println(0.5); myF.close(); }
             // ---------------------------- now read it back & show on Touch screen --------------------------------------------
             /*
  myF = SD.open (FName2, FILE_READ);
     
    if (myF) while (myF.available()>0) {ch10= myF.read();

     if (xv1>=400) {xv1=60;yv1+=12; }
     n15 = ch10 & 0x0F; if (n15 <= 9) ch6 = 0x30 + n15; else ch6 = 0x41 + (n15 - 10); // show right hand nibble, (ch6)
    GUI_DisChar(xv1 + 8, yv1, ch6, &Font12, Colr[7], BLUE); // show 1st nibble (ch6)

    n15 = (ch10 & 0xF0) >> 4 ; if (n15 <= 9) ch8 = 0x30 + n15; else ch8 = 0x41 + (n15 - 10); // show left hand  nibble, (ch8)
    GUI_DisChar(xv1, yv1, ch8, &Font12, Colr[7], BLUE); //  show 2nd nibble(ch8), to the left of 1st byte
          //
    // ? GUI_DisString_EN (xv1, yv1 , "Srv6.csv-size", &Font12, YELLOW, BROWN);
    GUI_DisChar(xv2, yv2+12, ch10, &Font12, Colr[7], BLUE); //  show as char 1/0/./,  etc.
    xv1+=20;  xv2+=8;  }
  
  //------ myF is 'FName2' = Srv3.csv as of 3/Oct/2022 -----
  //----------------  (xv2=60, yv2=100)-------------------------- 'FName2' defined correctly------- 
  if (myF){m8 = myF.size();  GUI_DrawRectangle(xv2 + 20, yv2, xv2 + 20 + (11 * 8), yv2 + 12, Colr[7], DRAW_FULL, DOT_PIXEL_DFT); // erase rect for 6 cha. & 5 digits
  Str11 = "Srv" + String(Srv_No) + ".csv";
  for (m7 = 0; m7 <= 9; m7++)  FName2[m7] = Str11[m7]; // e.g. srv100.csv has 910 chars.(String)Str11 copied into char FName2[0~7]
  
  //GUI_DisString_EN (50+12*m6,170,Str11 , &Font16, WHITE,BLUE);keyBf0=0;m6++;
  lcd1.setCursor(4, 3);lcd1.print("FNam"); lcd1.print(Str11);
  GUI_DisString_EN(xv2 + 10, yv2, FName2, &Font12, Colr[7], BLUE);
  GUI_DisString_EN(xv2 + 80, yv2, "F-size", &Font12, Colr[7], BLUE); GUI_DisNum (xv2 + 80 + (7 * 8), yv2 ,  m8, &Font12, Colr[7], BLUE); // print size of present file(Srv6.csv)
  lcd1.setCursor(0, 3); lcd1.print("F-size "); lcd1.print(m8);
  myF.close(); }
            */
             //----------------------------------------------------------------------
  del1();
  //......................................................................
  //Alpha_1();
  // */
  //---- print ch10 as Hex code -------
  /*
     myF = SD.open ("test1.txt");  if (myF) {Serial.println("test1.txt:"); while (myF.available())  Serial.write (myF.read()); myF.close();}//SD data on Serial Mon.
          else Serial.println("error reopening test1.txt to see if data on SD card ok");
         */
  //......................................end of writing into 'SD'............................................

  //----- for testing E2p write------------------------------------------------
  //    testing done ok
  //tEA = EAd1; Srv_No++; EEPROM.put ( tEA, Srv_No);  // Survey no. =1 stored at addr=EAD1 - 1 time initialization
  //tEA=EAd1+(1*IntSz) ;LRdSr++;  EEPROM.put ( tEA, LRdSr);     // Last Reading Sr. No & l. spac. no.
  //tEA=EAd1+(2*IntSz) ; LSpcN++; EEPROM.put ( tEA, LSpcN);
  //.............................end of E2p write.testing......................................


  // -------------- Below ??---
  // Define some parameters
  Npls[1] = Nbck + Nsig;
  Npls[2] = Nbck - Nsig;
  Npls[3] = Nbck - Nsig;
  Npls[4] = Nbck + Nsig;
  yfct = (float)(70.0 / (float)Npls[1]);
  Ycoord[0] = NdtxA - (Nbck * yfct);
  Ycoord[1] = NdtxA - (Npls[1] * yfct);
  Ycoord[2] = NdtxA - (Npls[2] * yfct);
  Ycoord[3] = NdtxA - (Npls[3] * yfct);
  Ycoord[4] = NdtxA - ((Npls[1] * yfct) + (12 * 3));
  j2 = 0;  // bit position in received byte
  //---------------------------------------------------------------------------------------------------------------------------------
  // digitalWrite(24, HIGH);     // make ack 'high'
  // digitalWrite(27,HIGH);  //  This pin is to be made high for ~0.5 Sec for turning A1 card on momentarily
  if (digitalRead(22) == HIGH) {
    pres_lvl = 1;  // -----Should be done only when +D is on--------------------
    last_lvl = pres_lvl;
  } else {
    pres_lvl = 0;
    last_lvl = pres_lvl;
  }
  //  In this one-time initialization last_lvl is made =pres_lvl
  // this is to ensure correct detecti on of risi. pres_lvl
  Recv = 0;  // ignore received data
  //..............................................................................................


  //--------------------------------------initialize for 'No handshake data' from A1 card  ---- ?
  Erase2();
  j3 = 0;  // j3= no. of bytes receied on 'Serial1' initialized to 0
           // ----------------------Next:write data into EAd6~EAd7 area to simulate calc_Res filling Resistivity data---------------------------------------
           /*
   for (n9=1; n9<=NRec; n9++)     //E2prom[tEA] <--StRd[i],StL[i], StRho[i] (int,int,float)
    {tEA= EAd6+ 8*(n9-1); EEPROM.put ( tEA, StRd[n9-1]); tEA+=2; EEPROM.put (tEA, StL[n9-1]); tEA+=2; EEPROM.put ( tEA, StRho[n9-1]); }
            */
           //..............................................end of data into EAd6~Ead7 area.....................................................................
           //
           //-----------------------get file names fro 'SD'------------------------------------------------------------------
           /*
     File root= SD.open("/") ; printDir(root,0); timr3=0;timc1=0;  while(timc1<4) {timc1=timr3/100; lcd1.setCursor(3,2); lcd1.print(timc1); }    // wait till timc1 becomes =4
     Serial.println(); Serial.println("Rewinding & repeating below"); Serial.println(); printDir(root,0); root.close();
          */

  //..............................................end o.f file names from SD................................
  //  ---------------
  kpr = 2;
  Fpr = 'H';      // 8/Feb/2025
  entry_fnc_H();  // now defined at power-On: 'Survey' mode
                  //------------------------------------------------------------------------commented, , writing originak Lt[].lt[] into E2prom-is suppressed-24/jan/2023--------------
                  //---------------following should be done only oncer-------------------------------------
                  //  timr3=0;  while(timc1<4) {timc1=timr3/100; lcd1.setCursor(3,2); lcd1.print(timc1); }    // wait till timc1 becomes =4

  n16 = 0;
  E2prom_Lltbl(n16);  // actually,both Sclumberger & Wennersets are copied into EEPROM n16=0 means , it starts from EAd5 + 0
                      //......................................................................................................................
  lcd1.createChar(1, tick1);
  lcd1.createChar(2, tick2);
  //
  //  ---------------------------------------------------------------
  // .............Read files using 'dir.openNextFile .......------------------------------------
  /*
    Serial.print("  SD --- 1  ");
            ch11= 'A' ;          //File root;
      myF = SD.open( "Srv3.csv",FILE_READ) ;      // earlier tried "srv3.csv"
              //if (myF)   {myF.rewindDirectory(); File entry,dir; entry= dir.openNextFile(); Serial.print(entry.name() );}
       if (myF) {    Serial.print("Srv3.csv") ; sz1=0; tsz1=myF.size(); Serial.print("file size = "); Serial.print(tsz1); Serial.print("   ");     // while (ch11 != 0xFF)    // 1{ FFh is 'EOF', End Of File     
                    while (sz1 < tsz1-4)
                 {   while (ch11 != 0x0A  )      // // 2{ 0Dh is CR, 'Carriage return'      
           { ch11=myF.read(); sz1++; if(ch11>='0' && ch11 <='9') {ch10=ch11-'0'; Serial.print(ch10,HEX); }        //  {3Serial.print(" ");
                 if (ch11==',')Serial.print(","); if (ch11=='.')Serial.print("."); if (ch11==0x0D)Serial.print("[");     //
                 if (ch11==0x0A) {Serial.print("] ");Serial.print("size = "); Serial.print(sz1); Serial.println("  "); }
                 }       //ch11=myF.read();
                ch11 = 'A'; // purposely alter value of ch11 from oAh to some other value
              }    //  end of keep repeating if (sz1 < tsz1-3 
               
              }  //.end of 'if myF=.true
          else Serial.println("unable to open file 'SRV3.csv' ") ;  
    Serial.print("SD End- 2---");
             myF.close();
             */
  //---------------------------------------------------print 'program memory constants on Serial monitor-----------------
  /*
            for (n21=0; n21<=3; n21++)      // print 4 pairs L,l (10* values)
    {n22= pgm_read_word_near (PLt+n21) ;     n23= pgm_read_word_near (Plt+n21) ;  Serial.print("n22 = "); Serial.print(n22); Serial.print("n23 = "); Serial.print(n23);  }
                */
  //..................................................................end of Program memory constants......

  //-----------------------------------write 'Spacing table 'Spc5.csv' file on SD-----------------------------------------------
  /*
                 if (wrLlSD==1)
                 //  /*
           {myF= SD.open("Spc5.csv", FILE_WRITE );    
           if (myF) 
           {   myF.flush(); myF.seek(0); Serial.println("Spc5.csv opened ----1 ");          // for (n1=0; n1<=37; n1++)
             {       for (n21=11; n21<=37; n21++)               // write 5 sets of L,l from program memory
               { n22= pgm_read_word_near (PLt+n21) ; fltLv= (float)n22/10; n23= pgm_read_word_near (Plt+n21) ; fltlv = (float) n23/10;  // n15 ignored
                      
                           //Serial.print("n22 = "); Serial.print(n22); Serial.print("n23 = "); Serial.print(n23);
                       myF.print (fltLv) ;myF.print (",") ; myF.println (fltlv) ; }  // 4 records: L,l thrn (Enter) CR,LF
                 
                     tsz1=myF.size(); Serial.print(" file size = "); Serial.print(tsz1); Serial.print("  ");
                myF.close(); }
           }
            else Serial.println (" file Spc5.csv could not be opened ---2 ");  myF.close();
           }  
                        */
  //.........................................end of write file Spc5.csv ..on SD.......

  //------------------------cancelled: 'writing Wenner data on SD'-----------write 'Spacing table 'WSpc.csv' file on SD--(for Wenner method)---------------------------------------------
  /*
                 if (wrLlSD==1)
                 //  /*
           {myF= SD.open("WSpc2.csv", FILE_WRITE );    
           if (myF) 
           {   myF.flush(); myF.seek(0); Serial.println("WSpc2.csv opened (writing)----1 "); amax= sizeof(Wennat)/sizeof(Wennat[0] );         // for (n1=0; n1<=37; n1++)
             {       for (n21=0; n21<=amax-1; n21++)               // write 5 sets of L,l from program memory
               { n22= pgm_read_word_near (Wennat+n21) ; vr4= (float)n22/10; // vr4= float value for 'a' of Srl. no. n21
                         // n23= pgm_read_word_near (Plt+n21) ; fltlv = (float) n23/10;  // Not for Wenner method
                      
                           //Serial.print("n22 = "); Serial.print(n22); Serial.print("n23 = "); Serial.print(n23);
                       myF.print (n21+1) ;myF.print (",") ; myF.println (vr4) ; }  // 4 records: L,l thrn (Enter) CR,LF
                 
                     tsz1=myF.size(); Serial.print(" file size = "); Serial.print(tsz1); Serial.print("  ");
                myF.close(); }
           }
            else Serial.println (" file WSpc2.csv could not be opened(writing) ---2 ");  myF.close();
           }  
                       // */
  //.........................................end of write file Spc5.csv ..on SD.......
  //--------Cancelled : 'Reading SD & ouputting on Serial monitor'  Below: read SD data unconditioally ----------
  /*
                // read SD data unconditionally
              myF= SD.open("WSpc2.csv", FILE_READ );
               if (myF) { myF.seek(0); sz1=0; Serial.println("WSpc2.csv opened for verifying----3");
                  tsz1=myF.size(); Serial.print(" file size = "); Serial.print(tsz1); Serial.print("  ");
                while (sz1 < tsz1-3)
                 {  ch11=myF.read(); sz1++; if(ch11>='0' && ch11 <='9') {ch10=ch11-'0'; Serial.print(ch10,HEX); }        //  {3Serial.print(" ");
                 if (ch11==',')Serial.print(","); if (ch11=='.')Serial.print("."); if (ch11==0x0D)Serial.print("[");     //
                 if (ch11==0x0A) {Serial.print("] "); Serial.print("size ");Serial.println(sz1);} if (ch11==0xFF) {Serial.println("EOF");Serial.print(" size = "); Serial.print(sz1); Serial.println("  "); }  
                  } // end of 'while (sz1<tsz1-3)             
               }  // end of 'if (myF)
         else Serial.println (" file WSpc2.csv could not be opened(for read & vrify) --- 4");
        myF.close();
                       //  */
  //.............................................end of 'write Spacing table on SD.................................

  /*
            // open "Spc2.csv" for reading & print all chars till FFh-----------------------
            ch11= 'A';   // some non-'LF', non FFh char
         myF=SD.open ("Spc2.csv", FILE_READ);  
          if (myF)  {    Serial.print("Spc2.csv opened") ; sz1=0; tsz1=myF.size(); Serial.print("file size = "); Serial.print(tsz1); Serial.print("   "); //file name & size
                    while (ch11 != 0xFF)
                 {  ch11=myF.read(); sz1++; if(ch11>='0' && ch11 <='9') {ch10=ch11-'0'; Serial.print(ch10,HEX); }        //  {3Serial.print(" ");
                 if (ch11==',')Serial.print(","); if (ch11=='.')Serial.print("."); if (ch11==0x0D)Serial.print("[");     //
                 if (ch11==0x0A) Serial.print("] "); if (ch11==0xFF) {Serial.print("EOF");Serial.print(" size = "); Serial.print(sz1); Serial.println("  "); }  
                  } // end of 'if (ch11 != FFh'
                 }   // end of 'if myF  
                 else Serial.println (" file Spc2.csv could not be opened");
                 myF.close(); 
                       */
  // open "Spc2.csv" for closing the file-----------------------
  //-------------------read Schlumberger & Wenner spacings from EEPROM & show on Serial monitor
  /*         
     Serial.println();     // ---Schlumbergerr spacings below -------
     for (i1=0; i1<=37; i1++) {tEA = EAd4+ i1*4; EEPROM.get ( tEA, n21); Serial.print("-n21-"); Serial.print(n21);  // show on Serial monitor screen
     tEA+=2; EEPROM.get( tEA, n22 ); Serial.print(","); Serial.print(n22);Serial.print("* "); if (i1==10 ||i1==20 ||i1==30) Serial.println(); }  //EAd4=160,38 sets       
         //--------Wenner spacings below------------
    for (i1=0; i1<=22; i1++) {tEA = EAd5+ i1*2;  Serial.print("-i1-"); Serial.print(i1+1); Serial.print(",");
    EEPROM.get ( tEA, n23);      Serial.print(n23);  // show on Serial monitor screen
       Serial.print("*"); if (i1==10 ||i1==20 ||i1==30) Serial.println(); }  //EAd4=160,38 sets 
          */
}

// ............. ..................................end of A4_Init  ..............................
//--------------------------------------
unsigned int k_value(byte keyBf0) {
  unsigned int val_key;
  if (keyBf0 == k_1) val_key = 1;
  if (keyBf0 == k_2) val_key = 2;
  if (keyBf0 == k_3) val_key = 3;
  if (keyBf0 == k_4) val_key = 4;
  if (keyBf0 == k_5) val_key = 5;
  if (keyBf0 == k_6) val_key = 6;
  if (keyBf0 == k_7) val_key = 7;
  if (keyBf0 == k_8) val_key = 8;
  if (keyBf0 == k_9) val_key = 9;
  if (keyBf0 == k_zr) val_key = 0;  //if (keyBf0 == k_dot) val_key=0;
  return val_key;
}
// Fpr is defined aschar
// -------------------------------------------------------------------------
//------------ find value of key pressed-----------------------
unsigned int key_prsd_val1(byte keyBf0) {  //
}
// ................................end of key_prsd_val1.....................
//-----------------------------enter_L_l -------------------------------------------------------
//      User uses this function to enter a new value for 'L' or 'l' like 0.6 601.2 meter & storemin
//    ------
void enter_L_l() {
  lcd1.setCursor(0, 0);
  lcd1.print("vary the value ");
  lcd1.setCursor(5, 1);
  lcd1.print("of L & l");
}
//...................................................'end of function  "enter_L_l "  .................
//----------------------------- 18/January/2024--------------
// show ABCD......etc.
//______________________________________________________________
void show_some() {
  lcd1.setCursor(0, 0);
  lcd1.print("ABCDEFGHIJKLMNOPQRST");
  lcd1.setCursor(0, 1);
  lcd1.print("abcdefghijklmnopqrst");
  lcd1.setCursor(0, 2);
  lcd1.print("UVWXYZ");
  lcd1.setCursor(0, 3);
  lcd1.print("uvwxyz");
}
//....................end of' show_some'.................................
void wrt_Ll_E2prm()  //---------------------------------------
{
  volatile unsigned int i12;
  tEA = EAd10;
  for (i12 = 0; i12 <= 35; i12++) EEPROM.put(tEA, pgm_read_word_near(PLt + i12));
  // this line will wrte 36 bytesin E2prom -------
}

//----------------------------begin E2prom_put----------------
void E2prom_put() {
  tEA = EAd9;
  for (i5 = 0; i5 >= 15; i5++) {
    EEPROM.put(tEA, RcBf_R1[i5]);
    tEA++;
  }
  ln8 = (RcBf_R1[6] * 0x100) + RcBf_R1[5];
  BattV = (float)ln8 / 100.0;
  EEPROM.put(tEA, BattV);  //16 th numeri a float (4 bytes)

  // store 15 bytes in E2prom
}
//------------------------------Read L,l values from E2prom---------------
//   Read L,l values from EEPROM
//........................................................................
/*
   void Read_Ll_E2prm()   
   {
                volatile unsigned int i12 = 0;
    const unsigned int   L_fr_E2prm = 0;
  tEA = EAd10; for(  i12=0; i12<=35;i12++ ) EEPROM.get(tEA,L_fr_E2prm + i12 ) ; // L value from EEPROM

   
   }
         */
//...........................end of 'Read_Ll from EEPROM & display on 20x4 LCD'
//---------------------------put 15 bytes in E2prom------------
//-----------------------------7/Jan/2024--------------
// read bytes from E2prom\
//..............................end of 7/jan/2024.........
void E2prom_get() {
  tEA = EAd9;
  for (i5 = 0; i5 >= 15; i5++) {
    EEPROM.get(tEA, RcBf_R1[i5]);
    tEA++;
  }  // read off 16 bytes of received data
  // read15 bytes from E2prom
  EEPROM.get(tEA, BattV);
}
//---------------------------put 15 bytes in E2prom------------

// -----SHOW RECEIVED BYTES----------------------------
//.................................................
void Show_Recv_bytes() {
  for (i5 = 0; i5 >= 14; i5++) Serial.write(RcBf_R1[i5]);
}
//-----------------------------------------------------------------------------------
//  function : printDir(  ,)
//-----------------------------------------------------------------------------------------------------
void printDir(File dir, unsigned int ntb) { /*
  while (true) {
   File entry=dir.openNextFile(); if (entry==0 ) {  if (ntb==0) Serial.println("** Done**" ); return;  } for (int i1=0; i1<ntb ; i1++) Serial.print('\t') ;  Serial.print(entry.name) ;
     
i2f (entry.isDirectry() ) {Serial.println("/"); printDir(entry, ntb+1);  }  }   
    */
}

//..............................................end of printDir..........................................
//----------------------------------------begin E2prom_Lltbl(n1) ------------------------------------
//  PROGMEM to EEPROM EAD5~EAd6  PROGMEM constants PLt[38]= {15,20,25 etc.Plt[]={5,5,5,5 etc.}  ~40 records ,EAd6~EAd7: PROGMEM constants Wennat[23]
//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -- - - - - - - - - - - - - - -- - - - - -
void E2prom_Lltbl(unsigned int n1) {
  unsigned int i1;
  for (i1 = 0; i1 <= 37; i1++) {
    tEA = EAd4 + i1 * 4;
    EEPROM.put(tEA, pgm_read_word_near(PLt + i1));  // RAM to EEPROM, EAd4=160
    tEA += 2;
    EEPROM.put(tEA, pgm_read_word_near(Plt + i1));
  }  //EAd4=160,38 sets

  for (i1 = 0; i1 <= 22; i1++) {
    tEA = EAd5 + i1 * 2;
    EEPROM.put(tEA, pgm_read_word_near(Wennat + i1));
  }  // EAd5=460,PROGMEM to E2PROM  EAd5=460,23 sets

  for (i1 = 0; i1 <= 24; i1++) {
    tEA = EAd9 + i1 * 4;
    EEPROM.put(tEA, pgm_read_word_near(Dipat + i1));  // EAd9=2280, 2 integers(4 bytes) per record
    tEA += 2;
    EEPROM.put(tEA, pgm_read_word_near(Dipnat + i1));
  }  //EAd9=2280,25 sets
}
//....................................end of E2prom_Lltbl:-- writing ~40 records into EEPROM........................
//
// ----------------------------------------print PLt, Plt say, 5 pairs on Serial monitor----------------------
// Serial.println(" "); // next line


// ---------------------------------------------------------------------------------------
// respond to Request from A1 card . D24 output shold become =0 or a low going pulse on D24
// ----------------------------------------------------------------------------------------
void Updt_DigInpLvls() {  // vr2 = strtof(st1);  // just for testing 'strtof'
}
// ............... ..........................................................................
//-----------------------------calculate Batt. Voltage--(7/Sep/2022---------------------------------------------------------------
void calc_Batt() {
  //ln8 = (RcBf_R1[6] * 0x100) + RcBf_R1[5];
  BattV = (float)ln8 / 100.0;
  dtostrf(BattV, 5, 2, st1);                                                         // Batt Volt e.g. 12.83, RcBf_R1[7] expected to be 0
  GUI_DrawRectangle(10, 180, 10 + 60, 180 + 13, Colr[7], DRAW_FULL, DOT_PIXEL_DFT);  // erase rect 60x13
  GUI_DisString_EN(10, 180, &st1[0], &Font12, Colr[7], BLUE);                        // Batt. Voltage
                                                                                     //-------------- do not clear entire lcd ----------------------------
  lcd1.setCursor(0, 0);
  if (Fpr == 'Q') lcd1.print("F0");
  if (Fpr == 'H') lcd1.print("F2");
  if (Fpr == 'J') lcd1.print("F4");
  lcd1.setCursor(13, 0);
  lcd1.print("B=");
  lcd1.print(BattV, 2);  // now,'Sigma',1lcd1.clear();show at (13,0) B=12.68
  lcd1.setCursor(0, 3);
  lcd1.print("                   ");  // erase line-3 (press measure)
                                      //-----------------------show switch positions, only if RcBf_R1==3,i.e.Rannge switch pos. is on 'Bat'---------------------
  Curr_Sw = RcBf_R1[2];
  Range_Sw = RcBf_R1[3];
  Cycl_Sw = RcBf_R1[4];
  if (RcBf_R1[3] == 1)  // Range Switch on 'Batt' posn.
  {
    lcd1.setCursor(0, 3);
    lcd1.print("Cur Sw:");
    if (Curr_Sw <= 7) { lcd1.print("Auto pres F0"); }  // curr_Sw: line-3
    else
      lcd1.print("Error");
    lcd1.setCursor(0, 1);
    lcd1.print("Range Sw:");
    if (Range_Sw == 1) lcd1.print("Bat      ");  //Line-1: Range switch
    if (Range_Sw >= 2 && Range_Sw <= 4) lcd1.print("Resist");
    if (Range_Sw == 5 || Range_Sw == 6) lcd1.print("S.P.");
    if (Range_Sw >= 7) lcd1.print("Error");
    lcd1.setCursor(0, 2);
    lcd1.print("Cycl Sw:");
    if (Cycl_Sw <= 4) {
      lcd1.print(Cycls[Cycl_Sw - 1]);
      lcd1.print(" cycl");
    } else lcd1.print("Error");  //
                                 // Line-2   Cycls Switch  posn.==1,2,3,4 means 1,4,16,64 cycles respectively
  }
}
//.........................................end of calcul. Batt. Volt..............................................................
//----------------------------------copied from Ketch SimCrm1------ ~90 lines----------------------------------------
//--------------------------------------------------------------
//     show Status of Current
//---------------------------------------------------------------------------
void curr_Status() {
  Serial.println("curr_Status_Start");
  if (RcBf_R1[17] == 1) IeStat = 0;
  else IeStat = 1;  // IeStat =0, means 'current not flowing
  //y7=60;
  if (IeStat == 1) {  // Current flowing ok
    IeNo2 = RcBf_R1[18];
    GUI_DrawRectangle(110, y7 + 30, 110 + 136, y7 + 60 + 13, Colr[7], DRAW_FULL, DOT_PIXEL_DFT);  // erase rect *8= length of 17*8cha  for ' no current
    //GUI_DisString_EN (70, y7 + 30 , "--- current"  , &Font12, WHITE, BLUE); //

    GUI_DisNum(55, 180, IeMag2[IeNo2], &Font12, WHITE, BLUE);  // Current switch(byte 2)CurrMag[i] (i is 0~6 (not 1~7)
                                                               //GUI_DisString_EN (170 + 50, y7 + 30 ,  "mA flowing OK --", &Font12, WHITE, BLUE);
                                                               //  Batt Volt.shown at (2,2) in calc_Batt
                                                               //--------------------------------------------------------------------------------------------------
    if (Fpr != 'Q' && Range_Sw != 1) Show_LlK();               // 'show L,l K on line 1,' is blocked , if in 'Test' mode or if in Batt posn.

    //     but later line 0 will be overwritten by the followin 2 lines
    //.................................................................................................
    //lcd1.setCursor(0, 0); lcd1.print("B="); lcd1.print(BattV, 2);    // Batt Volt will be overwritten by Current
    lcd1.setCursor(13, 0);
    lcd1.print("I=");
    lcd1.setCursor(15, 0);
    lcd1.print("     ");
    lcd1.setCursor(15, 0);
    lcd1.print(IeMag2[IeNo2]);
    lcd1.print("mA");  // first erase 15~19,then current magnitude
    lcd1.setCursor(0, 0);
    if (Fpr == 'Q') lcd1.print("F0");
    if (Fpr == 'H') lcd1.print("F2");
    if (Fpr == 'J') lcd1.print("F4");
    //    if (Fpr!='Q')
    //{ lcd1.setCursor(1, 0);lcd1.print("2");}  // now, 'Sigma',2 lcd1.setCursor(8, 0);  lcd1.print("           ");
    // else { lcd1.setCursor(0, 0);lcd1.write(0xCE);lcd1.print("2");}  // CEh is 'T-modified'
  }  //

  else {                                                                                        // IeStat==0,means 'No current'
    GUI_DrawRectangle(70, y7 + 30, 70 + 240, y7 + 30 + 16, Colr[7], DRAW_FULL, DOT_PIXEL_DFT);  // erase 'current flowing ok'
    GUI_DisString_EN(110, y7 + 30, "---- No Current--", &Font12, WHITE, RED);                   // current is not flowing
    lcd1.setCursor(8, 0);
    lcd1.print("-No Curr--- ");
    lcd1.setCursor(3, 1);
    lcd1.print("mode");
    lcd1.setCursor(0, 3);
    lcd1.print("-check connections-");
    tn1 = 0;
    tn2 = 0;
    t_transf = 0;  // So DSU knows that Auto_D will now turn off
  }
  Serial.println("curr_Status_End");
}
//................................................................end of 'curr_status'.........................
//---------------------------------------------------------------------------------------------------
// Get L,l & K
//------------------------------------------------------------------------------------------------------------------
void Show_LlK(void) {
  if (Surv_meth == 1) {
    if (LSpcN2 == 0) freezeSP = 0;
    LlpSz = 2 * IntSz;
    if (freezeSP == 0) tEA = EAd4 + LlpSz * LSpcN2;
    else tEA = EAd4 + LlpSz * (LSpcN2 - 1);  //(EAd5 changed to EAd4)if (freezeSP==0)  Show_LlK2(LSpcN2); else Show_LlK2(LSpcN2-1) ;    //
                                             // above is for Schlumberger spacing
    EEPROM.get(tEA, Lint1);
    tEA += IntSz;
    EEPROM.get(tEA, lint1);  //get( Lint1, lint1)from E2prom (these are 10*actual values)
    fltLv = (float)Lint1 / 10.0;
    fltlv = (float)lint1 / 10.0;
    Kvt = ScalcK(fltLv, fltlv);  // Lvalue, lvalue &Kv
                                 // dtostrf (fltLv, 5, 1, st1);       GUI_DisString_EN (100, 130 , &st1[0], &Font12, Colr[7], BLUE);  //L
                                 // dtostrf (fltlv, 5, 1, st1);       GUI_DisString_EN (150, 130 , &st1[0], &Font12, Colr[7], BLUE);  //l
                                 // Lint3 used in Show_Eprom2 is actual value of L

    ldig1 = lint1 % 10;

    if (ldig1 == 0) lint3 = lint1 / 10;                      // if Ldig1==0 we use integer division, otherwise we use 'fltLv' a float value
                                                             //--------------------------------------------------------------------                                                      //GUI_DisNum (40,130, Lint1, &Font12, Colr[7],BLUE); GUI_DisNum (60,130, lint1, &Font12, Colr[7],BLUE);  //
                                                             /*   //  next 3 lines commented
  if (LSpcN2 == 0) {       // next 2 lines which show "Sr no , L= ,l= & K=" are not needed
    lcd1.clear();  lcd1.setCursor(0, 0); lcd1.print("Sr No ");  lcd1.setCursor(0, 1); lcd1.print("L= "); lcd1.setCursor(9, 1); lcd1.print("l= "); // Sr. no. etc.(lettering)
    lcd1.setCursor(0, 2); lcd1.print("K= ");
  }      // "Sr No" at (0,0), "L=" & "l=" at (0,1) &(11,1) & "K=" at (0,2)
        */
                                                             //.............................................................................
    GUI_DisNum(10, 130, LSpcN2 + 1, &Font16, WHITE, BROWN);  //lcd1.setCursor(8, 0);lcd1.print("Sp"); lcd1.print(LSpcN2 + 1); // Srl. No,.1st line
    lcd1.clear();
    lcd1.setCursor(0, 0);
    lcd1.print("F2");
    lcd1.print(" Rd");
    lcd1.print(LRdSr2 + 1);
    lcd1.print("  Sp");
    //lcd1.setCursor(10,0); lcd1.print("    "); lcd1.setCursor(10,0);
    if (freezeSP == 0) lcd1.print(LSpcN2 + 1);
    else lcd1.print(LSpcN2);  // //F6h is 'Sigma',4,Rd,Sp
                              //'Sigma'4,Reading no. & Spacing no (1~N numbering)

    GUI_DrawRectangle(10, 130, 70 + 240, 130 + 16, Colr[7], DRAW_FULL, DOT_PIXEL_DFT);  // erase earlier Graphics screen
                                                                                        //--------------------------Next: L= at (0,1), l= at (6,1), K= at (11,1)------------------------------------
    Ldig1 = Lint1 % 10;                                                                 //ldig1 = lint1 % 10;
    if (Ldig1 == 0) Lint3 = Lint1 / 10;

    if (Ldig1 == 0) {
      GUI_DisNum(50, 130, Lint3, &Font16, WHITE, BROWN);
      lcd1.setCursor(0, 1);
      lcd1.print("L=");
      lcd1.print(Lint3);
    }  // L integer, 2nd line

    else {
      dtostrf(fltLv, 7, 1, st1);
      lcd1.setCursor(0, 1);
      lcd1.print("L=");
      lcd1.print(fltLv, 1);
    }  // L-Float, 2nd line

    if (ldig1 == 0) {
      GUI_DisNum(100, 130, lint3, &Font16, WHITE, BROWN);  // 'l', 2nd line,integert
      lcd1.setCursor(8, 1);
      lcd1.print("            ");
      lcd1.setCursor(8, 1);
      lcd1.print("l=");
      lcd1.print(lint3);
    }  // 'erase chars.8~19,line 1.l'- integer,, 2nd line
    else {
      dtostrf(fltlv, 7, 1, st1);
      GUI_DisString_EN(100, 130, &st1[0], &Font16, WHITE, BROWN);  // 'l', 2nd line,float
      lcd1.setCursor(8, 1);
      lcd1.print("            ");
      lcd1.setCursor(8, 1);
      lcd1.print("l=");
      lcd1.print(fltlv, 1);
    }  // erase chars.8~19,line 1
    dtostrf(Kv[LSpcN2], 7, 2, st1);
    GUI_DisString_EN(150, 130, &st1[0], &Font16, WHITE, BROWN);  //Kv(Graphic LCD
    lcd1.setCursor(0, 2);
    lcd1.print("                    ");  // erase entire line-2

    lcd1.setCursor(0, 2);
    lcd1.print("K=");
    lcd1.print(Kvt, 2);  // Kv,  line-2
  }
  //-------- Wenner, below--------------------------------------------------------
  if (Surv_meth == 2)  //---- Wenner------
  {
    if (LSpcN2 == 0) freezeSP = 0;
    LlpSz = 1 * IntSz;
    if (freezeSP == 0) tEA = EAd5 + LlpSz * LSpcN2;
    else tEA = EAd5 + LlpSz * (LSpcN2 - 1);  //(EAd4 changed to EAd5 (Wenner))if (freezeSP==0)  Show_LlK2(LSpcN2); else Show_LlK2(LSpcN2-1) ;    //
                                             // above is for Wenner spacing
    EEPROM.get(tEA, Lint1);                  //tEA += IntSz; EEPROM.get(tEA, lint1); // Lint1='a',get( Lint1, lint1)from E2prom (these are 10*actual values)
    fltLv = (float)Lint1 / 10.0;
    Kvt = WcalcK(fltLv);                                     // fltlv = (float)lint1 / 10.0; Lvalue, lvalue &Kv
                                                             // dtostrf (fltLv, 5, 1, st1);       GUI_DisString_EN (100, 130 , &st1[0], &Font12, Colr[7], BLUE);  //L
                                                             // dtostrf (fltlv, 5, 1, st1);       GUI_DisString_EN (150, 130 , &st1[0], &Font12, Colr[7], BLUE);  //l
                                                             // Lint3 used in Show_Eprom2 is actual value of L
    Ldig1 = Lint1 % 10;                                      //ldig1 = lint1 % 10;
    if (Ldig1 == 0) Lint3 = Lint1 / 10;                      //if (ldig1 == 0) lint3 = lint1 / 10; // if Ldig1==0 we use integer division, otherwise we use 'fltLv' a float value
                                                             //GUI_DisNum (40,130, Lint1, &Font12, Colr[7],BLUE); GUI_DisNum (60,130, lint1, &Font12, Colr[7],BLUE);  //
                                                             /*
  if (LSpcN2 == 0) {       // next 2 lines which show "Sr no , L= ,l= & K=" are not needed
    lcd1.clear();  lcd1.setCursor(0, 0); lcd1.print("Sr No ");  lcd1.setCursor(0, 1); lcd1.print("L= "); lcd1.setCursor(9, 1); lcd1.print("l= "); // Sr. no. etc.(lettering)
    lcd1.setCursor(0, 2); lcd1.print("K= ");
  }      // "Sr No" at (0,0), "L=" & "l=" at (0,1) &(11,1) & "K=" at (0,2)
       */
    GUI_DisNum(10, 130, LSpcN2 + 1, &Font16, WHITE, BROWN);  //lcd1.setCursor(8, 0);lcd1.print("Sp"); lcd1.print(LSpcN2 + 1); // Srl. No,.1st line
    lcd1.setCursor(0, 0);
    lcd1.print("F2");
    lcd1.print(" Rd");
    lcd1.print(LRdSr2 + 1);
    lcd1.print("  Sp");
    if (freezeSP == 0) lcd1.print(LSpcN2 + 1);
    else lcd1.print(LSpcN2);  // //F6h is 'Sigma',4,Rd,Sp
                              //'Sigma'4,Reading no. & Spacing no (1~N numbering)

    GUI_DrawRectangle(10, 130, 70 + 240, 130 + 16, Colr[7], DRAW_FULL, DOT_PIXEL_DFT);  // erase earlier Graphics screen
    //--------------------------Next: L= at (0,1), l= at (6,1), K= at (11,1)------------------------------------
    if (Ldig1 == 0) {
      GUI_DisNum(50, 130, Lint3, &Font16, WHITE, BROWN);
      lcd1.setCursor(0, 1);
      lcd1.print("a=");
      lcd1.print(Lint3);
    }  // L integer, 2nd line

    else {
      dtostrf(fltLv, 7, 1, st1);
      lcd1.setCursor(0, 1);
      lcd1.print("a=");
      lcd1.print(fltLv, 1);
    }  // L-Float, 2nd line
       /*
  if (ldig1 == 0)   {
    GUI_DisNum (100, 130 , lint3, &Font16, WHITE, BROWN);  // 'l', 2nd line,integert
     lcd1.setCursor(8, 1); lcd1.print("            ");lcd1.setCursor(8, 1); lcd1.print("l="); lcd1.print(lint3);} // 'erase chars.8~19,line 1.l'- integer,, 2nd line
  else {
    dtostrf (fltlv, 7, 1, st1);GUI_DisString_EN (100, 130 , &st1[0], &Font16, WHITE, BROWN);  // 'l', 2nd line,float 
     lcd1.setCursor(8, 1); lcd1.print("            "); lcd1.setCursor(8, 1);lcd1.print("l="); lcd1.print(fltlv, 1);}  // erase chars.8~19,line 1
  dtostrf (Kv[LSpcN2], 7, 2, st1);  GUI_DisString_EN (150, 130 , &st1[0], &Font16, WHITE, BROWN); //Kv(Graphic LCD
                  */
    lcd1.setCursor(0, 2);
    lcd1.print("                    ");
    lcd1.setCursor(5, 2);
    lcd1.print("Wenner");  // erase entire line-2
    lcd1.setCursor(11, 1);
    lcd1.print("K=");
    lcd1.print(Kvt, 2);  // Kv,  line-1
  }
  //.......................................Wenner--end............................
}
//.....................................................................end of 'Show_LlK' .............................
//---------------------------------------------------------------------------------------------calculate Resistance----------
//   /*
void calc_Res() {
  Serial.println("calc_Res_Start");
  //------------------------------------temporary predefinition of 7 nos. bytes[10,11,12]= D0h,07h,00 bytes[2,3,4,5]= 0,10,0,01 & Recv_Buff1[2]=0-----
  //Recv_Buff1[10]=0xD0; Recv_Buff1[11]=07; Recv_Buff1[12]=0;  Recv_Buff1[2]=0; Recv_Buff1[3]=25;Recv_Buff1[4]=0; Recv_Buff1[5]=01; Recv_Buff1[7]=1;
  // this should make: F_count=2000  icnA1=10  & (5/Gain[7])= 1  i.e. Resm= (2000*10*1/1000) = 10 milli-ohm
  //...................................................................................................................................................
  vr2 = (float)RcBf_R1[31] + (float)RcBf_R1[32] * 256.0 + (float)RcBf_R1[33] * 65536.0;  //
                                                                                         //F_cnt = (long)Recv_Buff1[10] + (long)(Recv_Buff1[11] * 0x100) +  (long)(Recv_Buff1[12] * 65536); // Recv_Buff1[k] is 1~7 (we need 0~6 ;
                                                                                         //F_cnt = (long)(Recv_Buff1[10] + Recv_Buff1[11] * 256 +  Recv_Buff1[12]*65536);
                                                                                         //Resint= F_cnt * icn[IeNo2];  //GUI_DisNum (220,145,Resint, &Font16, WHITE,WHITE);//
  ln8 = ((long)RcBf_R1[25] * (long)0x10000) + (long)RcBf_R1[24] * (long)0x100 + (long)RcBf_R1[23];
  icnA1 = ((float)ln8 / 256.0) * tenf[RcBf_R1[26] - 1];  //Gain[0~3]-- 5/0.5/0.05/0.005
  Resx = (vr2 * icnA1 * (5.0 / Gain[RcBf_R1[28]]));      // Resx= (pulse count over 4 Sec.)*multiplier*(5/actual gain)
                                                         // now Resx (resistance) is calculated
  //-----------------------------temporary --------------------------------------------------------
  //F_cnt=24000; icnA1=40.0;  Resx = (float)F_cnt * icnA1;// predefined for testing (later removed)
  //.................................................................................................
  //-------------for 'Current multipl.=4.9883x10^1, actual Gain=1
  Resm = Resx / 1000.0;  // Resm --Resistance in milli-ohm on dividing 'Resx'by 1000, we get value in milliOhm
  // calclate different factors of Resm ,to trace error of 'infinite result'
  fact1 = (float)ln8 / 256.0;
  fact2 = tenf[Recv_Buff1[5] - 1];
  fact3 = 5.0 / Gain[Recv_Buff1[7]];
  fact4 = 4.9883 / 100;
  GUI_DrawRectangle(10, 150, 420, 150 + 16, Colr[7], DRAW_FULL, DOT_PIXEL_DFT);  // erase rect {resistance =  */fact1,2,3
  /*
    GUI_DisNum (10,150 ,ln8, &Font16, WHITE,BROWN); dtostrf (fact1, 8, 4, st1);GUI_DisString_EN (10+40, 150 , &st1[0], &Font16, Colr[7], BROWN); // fact1 = bytes[4,3].[2]
    dtostrf (fact2, 3, 1, st1);GUI_DisString_EN (50+110, 150 , &st1[0], &Font16, Colr[7], BROWN);  // fact2 = 1/10/100
    dtostrf (fact3, 5, 1, st1);GUI_DisString_EN (140+100, 150 , &st1[0], &Font16, Colr[7], BROWN);  //  fact3 = 5.0 /actual gain
  */
  dtostrf(fact4, 8, 5, st1);
  GUI_DisString_EN(240, 150, &st1[0], &Font16, Colr[7], BROWN);  //  fact4 = 0.
  GUI_DisNum(240 + 100, 150, F_cnt, &Font16, WHITE, BROWN);
  if (Surv_meth == 1)  // Schlumberger
  {
    LlpSz = 2 * IntSz;
    tEA = EAd4 + LlpSz * LSpcN2;  // EAd4 for Schlumberger 10/march/2023
    EEPROM.get(tEA, Lint1);
    tEA += IntSz;
    EEPROM.get(tEA, lint1);  //get( Lint1, lint1)from E2prom (these are 10*actual values)
    fltLv = (float)Lint1 / 10.0;
    fltlv = (float)lint1 / 10.0;
    Kvt = ScalcK(fltLv, fltlv);  // Lvalue, lvalue &Kv
  }
  if (Surv_meth == 2)  // Wenner
  {
    LlpSz = 1 * IntSz;
    tEA = EAd5 + LlpSz * LSpcN2;  // EAd4 changed to EAd5 18/march/2023
    EEPROM.get(tEA, Lint1);       // tEA += IntSz; EEPROM.get(tEA, lint1); get( Lint1, lint1)from E2prom (these are 10*actual values)
    fltLv = (float)Lint1 / 10.0;
    Kvt = WcalcK(fltLv);  // avalue,  & Kvt, fltlv = (float)lint1 / 10.0;
  }
  //----------------------------Below: Dipole case-----------------------
  if (Surv_meth == 3)  // Dipole-Dipole
  {
    LlpSz = 2 * IntSz;
    tEA = EAd9 + LlpSz * LSpcN2;  // EAd9-- for Dipole
    EEPROM.get(tEA, Lint1);
    tEA += IntSz;
    EEPROM.get(tEA, lint1);        //  Lint1='a'(Dipole) & lint1='n'(Dipole) {both 'a' & 'n' are1~5 & 1~5}
    Kvt = DipcalcK(Lint1, lint1);  // both 'a' & 'n' are actual values (NOT 10*values) integers (1~5) & (1~5)
  }

  //.....................................end of Dipole case...............

  //---------------------show on Serial monitor-------------------------------------------------------------
  Serial.print("|F_cnt|");
  Serial.print(vr2);
  Serial.print("| ");
  Serial.print(RcBf_R1[31]);
  Serial.print(" ");
  Serial.print(RcBf_R1[32]);
  Serial.print(" ");
  Serial.print(RcBf_R1[33]);
  Serial.print(" ");
  Serial.print(" |icnA1|- ");
  Serial.print(icnA1);
  Serial.print(" exp-");
  Serial.print(RcBf_R1[26] - 1);
  Serial.print(" ");
  Serial.print("Gain No ");
  Serial.print(RcBf_R1[28]);
  Serial.print(" ");
  //Serial.print("Puls cnt-");  Serial.print(F_cnt);  Serial.print(" Gain No.-");Serial.print(Recv_Buff1[7]);
  //Serial.print(" icnA1-");Serial.print(icnA1); Serial.print(" ");Serial.print(" Res-"); Serial.print(Resm);Serial.println("mohm");

  //...............................................................................end 'Serial moonitor.......................

  // now show resist. in milli_ohm/ Ohm
  //GUI_DrawRectangle(110, y7 + 30, 110 + (20 * 8), y7 + 43 , Colr[7], DRAW_FULL, DOT_PIXEL_DFT); // erase rect " No current"
  //GUI_DrawRectangle(180, y7 + 30, 180 + 150 , y7 + 13 , Colr[7], DRAW_FULL, DOT_PIXEL_DFT); // erase rect5
  //y7 = 100;
  if (Resm <= 1200.0) {
    dimR = 'm';
    Rest[RdNo] = Resm;
    tRes = Resm;                 // Resistance  in milliOhm
    tRho = Kvt * (Resm / 1000);  // (Resm/1000) is in 'Ohm'
  }                              //GUI_DrawRectangle(270, y7 + 40, 270 + 70 , y7 + 53 , Colr[7], DRAW_FULL, DOT_PIXEL_DFT); // erase rect for value
  else {
    tRes = Resm / 1000;  //now tRes is in 'ohm'
    tRho = Kvt * tRes;   // tRho is in ohm-meter
    if (tRes <= 1200.0)  // if tres<=1200 ohms
    {
      dimR = ' ';    //  now tRes is in 'ohm'
      tRohm = tRes;  //tRohm will be anything from 0.0001 to 10,000 ohm
    } else {
      tRes /= 1000.0;
      dimR = 'k';  //now Res is in kohm
    }
  }
  //---------------now 'tRho' is defined-----------------------------------------------------------------
  //----------------------------------now store: Reading_No(LRdSr2 0~n numbering) ,L(10* the actual value). & tRho(float value) --(2+2+4)= 8 bytes-----------------
  if ((Fpr != 'Q') && ((Cycl_Sw == 1) || ((Cycl_Sw == 2) && PrCycl_No == 4))) {
    if ((Surv_meth == 1) || (Surv_meth == 2))  // Schlumberger or Wenner-- store 8 bytes
    {
      tEA = EAd7 + 8 * StRecrds;
      EEPROM.put(tEA, LRdSr2);
      tEA += IntSz;
      EEPROM.put(tEA, Lint1);
      tEA += IntSz;
      EEPROM.put(tEA, tRho);  // 1 record written,(Spacing no.,Resistance)
                              // in Schlumberger mode Lint1 would be L (actually 10* actual value). In Wenner moode Lint1 would be 'a'(actually 10 * actal value)
    }
    if (Surv_meth == 3)  // Dipole-Dipole method -- store 10 bytes
    {
      tEA = EAd7 + 10 * StRecrds;
      EEPROM.put(tEA, LRdSr2);
      tEA += IntSz;
      EEPROM.put(tEA, Lint1);
      tEA += IntSz;
      EEPROM.put(tEA, lint1);  // note: Lint1=(Dipole)'a' and lint1=(Dipole)'n'
      tEA += IntSz;
      EEPROM.put(tEA, tRho);  //
    }
    StRecrds++;
    tEA = EAd1 + (5 * 2);
    EEPROM.put(tEA, StRecrds);  //the 'no. of records' stored at   EAd7~EAd8 & EAd1+5*IntSz.region
  }
  // first,tEA -->to correct Record in EAd6~EAd7.then save int'Reading no.' then L (10*actual value), & finally  float no.'tRho' (Resistivity)
  //...............................................then,......................................................................................
  dtostrf(tRes, 9, 2, st1);  //
  GUI_DisString_EN(210, 130, &st1[0], &Font12, Colr[7], BLUE);
  GUI_DisString_EN(130 + xn1 + 70, 180, "m-ohm", &Font12, Colr[7], BLUE);  // "milli-ohm"

  dtostrf(tRho, 9, 2, st1);  //
  GUI_DisString_EN(280, 130, &st1[0], &Font12, Colr[7], BLUE);
  GUI_DisString_EN(290 + 108, 130, "ohm-m", &Font12, Colr[7], BLUE);  // "milli-ohm"
  // For testing:--LSpcN++not done; update Last Spacing No
  GUI_DisNum(255 + xn1, 180, LSpcN2, &Font12, Colr[7], BLUE);
  GUI_DisNum(255 + xn1 + (5 * 8), 180, LRdSr2, &Font12, Colr[7], BLUE);  //  no.of readings
                                                                         //-----------------Next:-- Res. & Rho written on 20x4 Lcd-------------------------------------------------------                                           //    */
                                                                         //-------------------------------now show Resistace on line-1---------------------------------
  if (Cycl_Sw == 1)                                                      // 1-cycle
  {
    lcd1.setCursor(0, 1);
    lcd1.print("                   ");
    lcd1.setCursor(19, 1);
    lcd1.print(" ");  // erase entire line-1 & ch. (19,1)
    lcd1.setCursor(8, 1);
    lcd1.print("R=");
    lcd1.print(tRes, 2);
    if (dimR == 'm') lcd1.print("m");
    if (dimR == ' ') lcd1.print(" ");
    if (dimR == 'k') lcd1.print("k");
    lcd1.write(0xF4);  // lcd1.setCursor(4, 0);F4h is 'ohm'
    //----------------------------no 'Rho' in 'Test' mode---Line-2 --------------------
    if (Fpr != 'Q') {  //
      lcd1.setCursor(0, 2);
      lcd1.write(0xE6);
      lcd1.print("=");
      lcd1.print(tRho, 2);
      lcd1.print("");
      lcd1.write(0xF4);
      lcd1.print("m");  //4th line, E6 means Rho, ohm-meter
      lcd1.print("-stored-");
    } else {
      lcd1.setCursor(0, 2);
      lcd1.print("-Res. not stored- ");
    }
    lcd1.setCursor(0, 3);
    lcd1.print("press 6 or F4");
    //..............................................................................
    lcd1.setCursor(0, 0);
    if (Fpr == 'Q') lcd1.print("F0");
    if (Fpr == 'H') lcd1.print("F2");
    if (Fpr == 'J') lcd1.print("F4");
    //lcd1.setCursor(1,0); lcd1.print("4");  // now, 'Sigma),4 ('T',4 in 'Test' mode)

    if (Fpr != 'Q') {
      lcd1.setCursor(0, 3);
      lcd1.print("press 6 for next");  // message for next operation
      lcd1.setCursor(12, 0);
      lcd1.print("       ");
      lcd1.setCursor(19, 0);
      lcd1.print(" ");
    }  // erase chars. 12~19 & ch. (19,0)
    else {
      lcd1.setCursor(0, 3);
      lcd1.print("press 6 or F4");
    }                  // 'Test mode
                       //-----------------------------no printing of 'L' in 'test' mode----(1-cycle mode---------------------------------------
    if (Fpr != 'Q') {  // i.e. F2, Survey mode

      if (Surv_meth == 1)  // Schlumberger
      {
        if (Ldig1 == 0) {
          GUI_DisNum(50, 130, Lint3, &Font16, WHITE, BROWN);
          lcd1.setCursor(0, 0);
          lcd1.print("                    ");
          lcd1.setCursor(0, 0);  // erase line 0
          lcd1.print("F2");
          lcd1.print(" Rd");
          lcd1.print(LRdSr2 + 1);
          lcd1.print("  Sp");
          //lcd1.setCursor(10,0); lcd1.print("    "); lcd1.setCursor(10,0);
          if (freezeSP == 0) lcd1.print(LSpcN2 + 1);
          else lcd1.print(LSpcN2);  //
          lcd1.setCursor(13, 0);
          lcd1.print("L=");
          lcd1.print(Lint3);
        }  // L integer, at (12,0)
        else {
          dtostrf(fltLv, 7, 1, st1);
          lcd1.setCursor(0, 0);
          lcd1.print("                    ");
          lcd1.setCursor(0, 0);  // erase line 0& brng the Cursor back to(0,0)
          lcd1.setCursor(0, 0);
          lcd1.print("F2");
          lcd1.print(" Rd");
          lcd1.print(LRdSr2 + 1);
          lcd1.print("  Sp");
          //lcd1.setCursor(10,0); lcd1.print("    "); lcd1.setCursor(10,0);
          if (freezeSP == 0) lcd1.print(LSpcN2 + 1);
          else lcd1.print(LSpcN2);  //
          lcd1.setCursor(13, 0);
          lcd1.print("L=");
          lcd1.print(fltLv, 1);
        }  // L-Float at (12,0)
      }

      if (Surv_meth == 2)  // Wenner method
      {
        if (Ldig1 == 0) {
          GUI_DisNum(50, 130, Lint3, &Font16, WHITE, BROWN);
          lcd1.setCursor(0, 0);
          lcd1.print("                    ");
          lcd1.setCursor(0, 0);  // erase line 0
          lcd1.setCursor(0, 0);
          lcd1.print("F2");
          lcd1.print(" Rd");
          lcd1.print(LRdSr2 + 1);
          lcd1.print("  Sp");
          //lcd1.setCursor(10,0); lcd1.print("    "); lcd1.setCursor(10,0);
          if (freezeSP == 0) lcd1.print(LSpcN2 + 1);
          else lcd1.print(LSpcN2);  //
          lcd1.setCursor(13, 0);
          lcd1.print("a=");
          lcd1.print(Lint3);
        }  // L integer, at (12,0)
        else {
          dtostrf(fltLv, 7, 1, st1);
          lcd1.setCursor(0, 0);
          lcd1.print("                    ");
          lcd1.setCursor(0, 0);  // erase line 0
          lcd1.setCursor(0, 0);
          lcd1.print("F2");
          lcd1.print(" Rd");
          lcd1.print(LRdSr2 + 1);
          lcd1.print("  Sp");
          //lcd1.setCursor(10,0); lcd1.print("    "); lcd1.setCursor(10,0);
          if (freezeSP == 0) lcd1.print(LSpcN2 + 1);
          else lcd1.print(LSpcN2);  //
          lcd1.setCursor(13, 0);
          lcd1.print("a=");
          lcd1.print(fltLv, 1);
        }  // L-Float at (12,0)
      }
      //---------------------- Wenner , below  ?
    }

    //............................................................................................................
  }
  //----------------- 20x4 LCD screen when it is 4-Cycle mode.----  if ( ==1|| ==2 ||  ==3 ||  ==4)
  //---------enter here 4 blocks of writing on 20x4 LCD-----------------------
  if (Cycl_Sw == 2 || Cycl_Sw == 3 || Cycl_Sw == 4)  // 4/16/64 cycls mode
  {
    lcd1.setCursor(12, 0);
    lcd1.print("       ");
    lcd1.setCursor(19, 0);
    lcd1.print(" ");  // erase chars. 12~19 & ch. (19,0)
    if (Fpr != 'Q') {
      //------------------------------------------no printing of 'L' in 'test' mode----(4-cycle mode---------------------------------------
      if (Ldig1 == 0) {  //L=--- at top right corner
        GUI_DisNum(50, 130, Lint3, &Font16, WHITE, BROWN);
        lcd1.setCursor(13, 0);
        lcd1.print("L=");
        lcd1.print(Lint3);
      }  // L integer, at (12,0)
      else {
        dtostrf(fltLv, 7, 1, st1);
        lcd1.setCursor(13, 0);
        lcd1.print("L=");
        lcd1.print(fltLv, 1);
      }  // L-Float at (12,0)
         //...............................................................................................................................
    }
    //- - - - - - - - Cycl-1- - - - - - - - - - -- - - - - - - - - --  - - - - - - - - - -
    if (PrCycl_No == 1) {
      lcd1.setCursor(0, 1);
      lcd1.print("                   ");
      lcd1.setCursor(19, 1);
      lcd1.print(" ");  // erase entire line-1 & ch. (19,1)
      lcd1.setCursor(3, 1);
      lcd1.print("R(");
      lcd1.print(PrCycl_No);
      lcd1.print("/4)=");
      lcd1.print(tRes, 2);  //erase 0~2
      if (dimR == 'm') lcd1.print("m");
      if (dimR == ' ') lcd1.print(" ");
      if (dimR == 'k') lcd1.print("k");
      lcd1.write(0xF4);  //  F4h is 'ohm'
    }                    //
                         //. . . . . . . . . . . . . . . . . .end of cycl-1 . . . . . . . . . . . . . . . . . . . . . . . . .
                         //- - - - - - - - Cycl-2- - - - - - - - - - -- - - - - - - - - --  - - - - - - - - - -
    if (PrCycl_No == 2) {
      lcd1.setCursor(0, 1);
      lcd1.print("                   ");
      lcd1.setCursor(19, 1);
      lcd1.print(" ");  // erase entire line-1 & ch. (19,1)
      lcd1.setCursor(3, 1);
      lcd1.print("R(");
      lcd1.print(PrCycl_No);
      lcd1.print("/4)=");
      lcd1.print(tRes, 2);  //erase 0~2
      if (dimR == 'm') lcd1.print("m");
      if (dimR == ' ') lcd1.print(" ");
      if (dimR == 'k') lcd1.print("k");
      lcd1.write(0xF4);  //  F4h is 'ohm'
    }                    //
                         //. . . . . . . . . . . . . . . . . .end of cycl-2 . . . . . . . . . . . . . . . . . . . . . . . . .
                         //- - - - - - - - Cycl-3- - - - - - - - - - -- - - - - - - - - --  - - - - - - - - - -
    if (PrCycl_No == 3) {
      lcd1.setCursor(0, 1);
      lcd1.print("                   ");
      lcd1.setCursor(19, 1);
      lcd1.print(" ");  // erase entire line-1 & ch. (19,1)
      lcd1.setCursor(3, 1);
      lcd1.print("R(");
      lcd1.print(PrCycl_No);
      lcd1.print("/4)=");
      lcd1.print(tRes, 2);  //erase 0~2
      if (dimR == 'm') lcd1.print("m");
      if (dimR == ' ') lcd1.print(" ");
      if (dimR == 'k') lcd1.print("k");
      lcd1.write(0xF4);  //  F4h is 'ohm'
    }                    //
                         //. . . . . . . . . . . . . . . . . .end of cycl-3 . . . . . . . . . . . . . . . . . . . . . . . . .
                         //- - - - - - - - Cycl-4- - - - - - - - - - -- - - - - - - - - --  - - - - - - - - - -
    if (PrCycl_No == 4)  // ----------last of 4 cycles------------------------
    {
      lcd1.setCursor(0, 1);
      lcd1.print("                   ");
      lcd1.setCursor(19, 1);
      lcd1.print(" ");  // erase entire line-1  & ch. (19,1)
      lcd1.setCursor(3, 1);
      lcd1.print("R(");
      lcd1.print(PrCycl_No);
      lcd1.print("/4)=");
      lcd1.print(tRes, 2);  //erase 0~2
      if (dimR == 'm') lcd1.print("m");
      if (dimR == ' ') lcd1.print(" ");
      if (dimR == 'k') lcd1.print("k");
      lcd1.write(0xF4);  //  9Ah is 'ohm'
                         //----------------------No printing of 'Rho' in 'Test' mode-------------------------
      if (Fpr != 'Q') {
        lcd1.setCursor(0, 2);
        lcd1.write(0xE6);
        lcd1.print("=");
        lcd1.print(tRho, 2);
        lcd1.print("");
        lcd1.write(0xF4);
        lcd1.print("m");  //4th line, E6 means Rho
        lcd1.print("-stored-");
      } else {
        lcd1.setCursor(0, 2);
        lcd1.print("-Res. not stored- ");
        lcd1.setCursor(0, 3);
        lcd1.print("press 6 or F4");
      }  // 'F0', 'Test' mode
         //...................................................................................................................
      lcd1.setCursor(0, 3);
      lcd1.print("press 6 for next");  // message for next operation
    }                                  //
    //. . . . . . . . . . . . . . . . . .end of cycl-4 . . . . . . . . . . . . . . . . . . . . . . . . .

  }  // end of 'if '4-Cycle mode'
     //..................................................................................................................
     //else
     //tRes,tRho defined ,Reading no. & Spacing n0. Now sore tRes & spacing n0. in range EAd6~EAd7
     //  -------------------------------------------------------store on SD--only if Cycles-1 or (cycles-4/16/64 &&-present_cycle==4)---------------------------------
  if (Cycl_Sw == 1 || (PrCycl_No == 4 && (Cycl_Sw == 2 || Cycl_Sw == 3 || Cycl_Sw == 4))) {
    //---------- now writing  into 'SD'-----(upto Ln ~1252-----------------------------------------------------------------
    //- - - - - - - - - - - - - - - --- - --  ---- -  - - - - - - -- -- - - - - - - -- -- - -  - --  - -- -
    if (Fpr != 'Q')  // {writing into 'SD' not to be done when Fpr==0, Test mode)
    {
      myF = SD.open(FName2, FILE_WRITE);
      //   /*
      if (LRdSr2 == 0) {  // 1st line:Survey No.  2nd line: Survey method (in 4th column)
        myF.print(",");
        myF.print(" Survey No.  ");
        myF.print(",");
        myF.print(Srv_No);
        myF.print(",");
        myF.println();
        myF.print(",");
        myF.print(",");
        myF.print(",");
        if (Surv_meth == 1) myF.print("Schlumberger");
        if (Surv_meth == 2) myF.print("Wenner");
        if (Surv_meth == 3) myF.print("Dipole-Dipole");
        myF.print(",");
        myF.print("Configuration");
        myF.println();  //"Dip-Dip",next column:Conf.

        //   headinmg: Rdng,Spac., (L,l)/('a')/ (a(Dipole),'n'),K,Res,Rho
        myF.print("Readig.");
        myF.print(",");
        myF.print("Spac. ");
        myF.print(",");
        if (Surv_meth == 1) {
          myF.print("           L");
          myF.print(",");
          myF.print("           l");
        }
        if (Surv_meth == 2) myF.print("         a");
        if (Surv_meth == 3) {
          myF.print("    a");
          myF.print(",");
          myF.print("          n");
        }
        myF.print(",");
        myF.print("         K");
        myF.print(",");
        myF.print("         Res");
        myF.print(",");
        myF.print("         Rho");  //  print rho and theen next line
        myF.print(",");
        myF.println("         Batt ");  // Batt. & then next line
                                        // next: 2nd line of heading
        myF.print("no.");
        myF.print(",");
        myF.print(" no.");
        myF.print(",");
        if (Surv_meth == 1) {
          myF.print("           m.");
          myF.print(",");
          myF.print("           m.");
        }
        if (Surv_meth == 2) myF.print("         m.");
        if (Surv_meth == 3) {
          myF.print("    m.");
          myF.print(",");
          myF.print("          m.");
        }
        myF.print(",");
        myF.print(",");
        myF.print("         ohm");
        myF.print(",");
        myF.print("         ohm-m.");  //  print blank,ohm.uhm-m.
        myF.print(",");
        myF.println("          Volt.");  // (Batt-)volt.,next line
      }                                  //Print the header only once after a new file is opened. i.e. 1st reading
      //      */
      //  from 1st reading onward : print values of Rdng, Spacing no.,(L,l). K,Res, Rho
      myF.print(LRdSr2 + 1);
      myF.print(",");
      myF.print(LSpcN2 + 1);
      myF.print(",");
      if (Surv_meth == 1) {
        myF.print(fltLv);
        myF.print(",");
        myF.print(fltlv);
        myF.print(",");
      }  //RdNo,Spacing no.,L,l
      if (Surv_meth == 2) {
        myF.print(fltLv);
        myF.print(",");
      }  //myF.print(fltlv ); myF.print(",");} //
      if (Surv_meth == 3) {
        myF.print(Dip_a);
        myF.print(",");
        myF.print(Dip_n);
        myF.print(",");
      }  //  integers (Dipole) a & (Dipole)n
         //------------------------------------------following 3 lines now split
         //dtostrf (tRohm, 11, 5, st1)   ;   Str11 = {""}; // initialize Str11
         //for (m7 = 0; m7 <= 10; m7++) Str11 += st1[m7]; // addd chars. st1[0~10] to Str11 one by one
         //myF.print(Str11); myF.print(",");

      myF.print(Kvt);
      myF.print(",");  // K
                       //--------------------------------------by now Rd no.,Spacing no.,L,l & K have been printed -------
                       //------------------------- next: tRohm (resistance) will be printed via 1 of 6 branches ---------
                       //  myF.print(Str11); myF.print(",");
      //vr3=0.00001; vr4=vr3; select_print_case( 0.00001) ; select_print_case( 0.99999); select_print_case( 0.0001) ; select_print_case( 9.9999); //write 4 nos. on 'SD'
      tRohm = Resm / 1000;  //tRohm in 'ohm' redefined
                            //-------------------------Different case 0.00001 to >10000.0 , 6 braches------------------------------------------------
                            //--------------------------case 1-------------------------------------------------------------------
      if (tRohm < 1.0)      // <1
      {
        dtostrf(tRohm, 7, 5, st1);
        Str11 = { "" };                                // initialize Str11
        for (m7 = 0; m7 <= 6; m7++) Str11 += st1[m7];  // copy 7 chars. st1[0~6] to Str11 one by one
        myF.print(Str11);
        myF.print(",");  // print tRohm,before tRho
      }
      //................................................end of case 1 ......................................
      //--------------------------case 2-------------------------------------------------------------------
      if (tRohm >= 1.0 && tRohm < 10.0)  // 1~10
      {
        dtostrf(tRohm, 6, 4, st1);
        Str11 = { "" };                                // initialize Str11
        for (m7 = 0; m7 <= 5; m7++) Str11 += st1[m7];  // copy 6 chars. st1[0~5] to Str11 one by one
        myF.print(Str11);
        myF.print(",");  // print tRohm,before tRho
      }
      //................................................end of case 2 ......................................
      //--------------------------case 3-------------------------------------------------------------------
      if (tRohm >= 10.0 && tRohm < 100.0)  // 10~100
      {
        dtostrf(tRohm, 6, 3, st1);
        Str11 = { "" };                                // initialize Str11
        for (m7 = 0; m7 <= 5; m7++) Str11 += st1[m7];  // copy 6 chars. st1[0~5] to Str11 one by one
        myF.print(Str11);
        myF.print(",");  // print tRohm,before tRho
      }
      //................................................end of case 3 ......................................
      //--------------------------case 4-------------------------------------------------------------------
      if (tRohm >= 100.0 && tRohm < 1000.0)  // 100~1000
      {
        dtostrf(tRohm, 6, 2, st1);
        Str11 = { "" };                                // initialize Str11
        for (m7 = 0; m7 <= 5; m7++) Str11 += st1[m7];  // copy 6 chars. st1[0~5] to Str11 one by one
        myF.print(Str11);
        myF.print(",");  // print tRohm,before tRho
      }
      //................................................end of case 4 ......................................
      //--------------------------case 5-------------------------------------------------------------------
      if (tRohm >= 1000.0 && tRohm < 10000.0)  // 1000~10000
      {
        dtostrf(tRohm, 6, 1, st1);
        Str11 = { "" };                                // initialize Str11
        for (m7 = 0; m7 <= 5; m7++) Str11 += st1[m7];  // copy 6 chars. st1[0~5] to Str11 one by one
        myF.print(Str11);
        myF.print(",");  // print tRohm,before tRho
      }
      //................................................end of case 5 ......................................
      //--------------------------case 6-------------------------------------------------------------------
      if (tRohm >= 10000.0)  // >10000
      {
        dtostrf(tRohm, 7, 0, st1);
        Str11 = { "" };                                // initialize Str11
        for (m7 = 0; m7 <= 6; m7++) Str11 += st1[m7];  // copy 7 chars. st1[0~5] to Str11 one by one
        myF.print(Str11);
        myF.print(",");  // print tRohm,before tRho
      }
      //................................................end of case 6 ......................................
      //-------now : StRecrds++ must be done---------------------------------
      //'tRohm' has been converted to apprpriate String 'Str11' & printed on 'SD'. next: 'Rho'
      myF.print(tRho);
      myF.print(",");
      myF.print(BattV);
      myF.println();  // Rho, Batt_Voltage, then. next line

      //----------------------------------------write test values----------------------------------------------------------

      //myF.print(Hr); myF.print(":"); myF.print(mint); myF.print(":"); myF.print(Scnd); myF.print(","); // after Rho, Hours:minutes:Seconds is printed
      //myF.print (Dte); myF.print("/"); myF.print (Mn); myF.print("/"); myF.println (Yr);

      Serial.println("Stored Successfully");  // Message on Serial Monitor {st1 is string of Resist., st2 is rhoK,Resist. ,Rho, (Rho with println) }
                                              // print string 'st1' instead float 'tRes'
                                              //--------write 2 lines of 8 resist. values ------------------------
                                              //for (m12=0; m12<=1; m12++) { myF.print(m11+ (m12*8)); for (m11=0; m11<=7; m11++) {myF.print(",");myF.print(tRes2[m11 +(m12*8)]);}  myF.println(" ");
      //---------------------------------------------------------------------------------------------------------------------
      //------------------block LSpcN2++ unless a key say '3' is pressed in 'Normal Survey' mode------------------------------------------------------------------------------------
      if ((LSpcN2 <= 43 && freezeSP == 0) && (Cycl_Sw == 1 || (PrCycl_No == 4 && (Cycl_Sw == 2 || Cycl_Sw == 3 || Cycl_Sw == 4)))) LSpcN2++;  // this way,the upper limit is = 44 (As LSpcN2+1) displayed on 20*4 LCD) max.valued displayed is 45
                                                                                                                                              //...............................................................................................................................
                                                                                                                                              //...........................................................................................................................
      LRdSr2++;                                                                                                                               // if (Cycl_Sw ==2/3/4) && Present cycle No==4) then Reading no++

      // reintroduced today,24/nov/2022-- writing LrdSr2++ &LSpcN2++ into E2prom
      //  ---------------E2prom values updated -----------------------------
      tEA = EAd1 + (3 * 2);
      EEPROM.put(tEA, LRdSr2);  // Last Reading Sr. No & l. spac. no. both updated
      tEA = EAd1 + (4 * 2);
      EEPROM.put(tEA, LSpcN2);
      // .....................................................................
      myF.close();  // update Last Spacing No  }
                    // end of 'if(Cycl_Sw==1 etc.
    }               // end of writing into 'SD''
    // --------------copied ~ 10 lines from Ln1780------------------------------------------------------------
    // ..........................................end of writing on SD...........................................
    // ------ for testing :-- LSpcN not changed
    // }
    Serial.println("calc_Res_End");
  }  // end of writing into 'SD'
     //   */
}  //..........................................end of 'calc_Res'............................................
   //--------------------------------------select print format-----?----------------

// ------------------------------show data received from (now,(7/Sept/2022) from Sketch DSU2)(Earlier,Auto_D, on Serial2 channel)-----------------------------
void Recv_Serial2() {  //j3a=0 to be done at initialization time          xv1=60 & yv1==80 initially
  // from ~ 1/August/2022 onward, variable names are changed from j2,j3 to j2a,j3a (j2a--no. of bits received so far.Similarly, j3a--no. of bytes received so far


  while (Serial2.available() > 0) {  // data expected from Serial-2 channel (Serial-1 is for GPS module) as of 8/August/2022)

    ch10 = Serial2.read();  //Recv_Buff1[j3a] = ch10; j3a++;

    // to serial monitor ?
    //......... now print the character on LCD Screen (j=0~29 means a~z at x2,y2 ...Before printing, erase rect. if necessary..................
    //if (xv1 >= 400 && yv1>=218) {
    //GUI_DrawRectangle(60, yv1, 400, 218, Colr[7], DRAW_FULL, DOT_PIXEL_DFT);  // show  lines withou erasing    Erase 60~400,height=17 if xv1>=400
    //xv1 = 60;
    //}
    if (xv1 >= 400) {
      yv1 += 16;
      xv1 = 0;
    }
    // ---------the following 2 lines are suppressed ------------------------------------------
    //         if  ( (ch10 >= 0x61 &&  ch10<= 0x7A) || (ch10 >= 0x41 &&  ch10<= 0x5A) )   // if ch10= 'a'~'z' or 'A'~'Z'
    //{GUI_DisChar (xv1,yv1, ch10 , &Font16,WHITE, BROWN) ;xv1+=13;}   // print a letter from a~z or A~Z, at xv1(=60~400max),yv1=50. then xv1+=13(advance cursor)
    //..........................................................................................
    Serial.print(ch10);
    RcBf_R1[tn1] = ch10;
    if (t_transf == 1) {
      Recv_Buff1[tn2] = ch10;  // copy ch10 into Recv_Buff1 (15 bytes)
      tn2++;
    }
    // --- 2 nibbles of each byte-----at x=140,160 ------
    // else {
    n15 = ch10 & 0x0F;
    if (n15 <= 9) ch6 = 0x30 + n15;
    else ch6 = 0x41 + (n15 - 10);                            // calculate 1st nibble (lower nibble only) (later try st3=String (n14,HEX);)
    GUI_DisChar(xv1 + 8, yv1, ch6, &Font12, Colr[7], BLUE);  // show 1st nibble (ch6)

    n15 = (ch10 & 0xF0) >> 4;
    if (n15 <= 9) ch8 = 0x30 + n15;
    else ch8 = 0x41 + (n15 - 10);                        //calculate 2nd nibble (ch8)
    GUI_DisChar(xv1, yv1, ch8, &Font12, Colr[7], BLUE);  //  show 2nd nibble(ch8), to the left of 1st byte
    //  }
    //show char. received on serial monitor ------------n 22/nov/2022: print j3a too----------------------
    /*
      Serial.print(" j3a="); Serial.print(j3a); Serial.print(" tn1="); Serial.print(tn1); Serial.print(" tn2="); Serial.print(tn2);    //  Serial.print("j3a=" );Serial.print(j3a);
                    Serial.print(" RcBf_R1[tn1] "); Serial.println(RcBf_R1[tn1]);  //  Serial.print("ch- "); Serial.print(ch10);
                  */
    //----------------write ch10-on lcd1 at (m13,1) 7 advance m13-------------------------------------------------------------------------------------------
    //lcd1.setCursor(m13,1); lcd1.print(ch10); m13++; j3ac

    //..........................................................................................................
    if (RcBf_R1[tn1] == 0xFF && RcBf_R1[tn1 - 1] == 0xFF) {
      xv1 += 20;
      GUI_DisNum(xv1, yv1, tn1, &Font12, Colr[7], BLACK);
      xv1 += 20;
      GUI_DisNum(xv1, yv1, tn2 + 1, &Font12, Colr[7], BROWN);  //tn1& tn2+1
      xv1 += 20;
      GUI_DisNum(xv1, yv1, t_transf + 1, &Font12, Colr[7], BLUE);
      xv1 = 60;
      yv1 += 12;
      tn1++;  // // show resistance
      // tRes= 65.0; dtostrf (tRes, 9, 2, st1);GUI_DisString_EN (80, 90+90 , &st1[0], &Font12, Colr[7], BLUE); //for testing
      //--------------------------------------------------------------------------
      //---- for testing, suppress  show_LlK) ---------------------
      if (tn1 == 15) {
        E2prom_put();  // store RcBf_R1 [0~15 byttes in EEPROM]
        calc_Batt();
        if (Range_Sw != 1 && Fpr != 'Q') Show_LlK();
      }  // do not call Show_LlK ,if in Batt position,, or in 'Test'mode. show 15 bytes, expected to be 01,0Fh,.....upto FFh,FFh
         //call 'Show_Llk' if not in Batt posn. & not in 'Test' mode)
         //  { (show Batt. Voltage & L,l,K values GUI_DisNum (30,180,tn1, &Font12, Colr[7],BLACK);
      //.................................................................................................................
      if (tn1 == 21) curr_Status();
      if (tn1 == 21) {
        tn2 = 0;
        tn3 = 0;
        t_transf = 1;
      }  // tn2 will go from 0 to 14 (15 bytes)
      // now, t_transf==1 means that tn2 will start increasing
      if (Cycl_Sw == 1 && (tn1 == 35 || tn1 == 36)) {
        xn1 = 0;
        PrCycl_No = 0;
        calc_Res();
        tn1 = 0;
      }  //  && means 1-cycle only. after 'calc_Res' make tn1=0;

      if (Cycl_Sw == 2 || Cycl_Sw == 3 || Cycl_Sw == 4) {  //4/16/64 cycles
        if (tn2 == 15) {
          xn1 = 0;
          PrCycl_No = 1;
          calc_Res();
        }  //Res.-1  (tn1 == 35)

        if (tn2 == 30) {
          xn1 = 70;
          PrCycl_No = 2;
          calc_Res();
        }  //Res.-2     tn1 == 50

        if (tn2 == 45) {
          xn1 = 140;
          PrCycl_No = 3;
          calc_Res();
        }  // Res.-3   tn1 == 65

        if (tn2 == 60) {
          xn1 = 210;
          PrCycl_No = 4;
          calc_Res();
          tn1 = 0;
        }  //  Res.-4 tn1 == 81 Last  Resistance value received. So make tn1=0;

      }  // end of 'if Cycl_Sw==1/2/3/4
    }    //end of 'if 2 bytes=FFh,FFh
    // GUI_DisString_EN (100, 90+90 , &st1[0], &Font12, Colr[7], BLUE);}
    // Removed---calc_Res();

    else {
      xv1 += 20;
      tn1++;
    }  // if (t_transf==1) tn2++;  at Ln ~ 1475
    //if (tn1>=2 && ch10==0xFF && Ch10old==0xFF) {xv1=60;yv1+=16; ch10old=ch10;}
    //  ---- 2 nibbles  over -------

    /*
      if (j3a >= 2) // if no. of bytes received is >= 2
      {
      if (j3a >= Recv_Buff1[1] ) Updt_RecD();  // if the expected no. of bytes have received , then Updt_RecD will show the appropriate Batt Volt. etc.
      }
      //  */
  }  // end of 'if Serial2 available
}
// ............................................................
//..............................................end of 'Recev_Serial2' & (calc_Res)..... ~90 lines.............
//--------------------------------------------------------------------------------------------show Resist.- 15 bytes----------

void Show_ResistData() {
}

//....................................................................................end of 'show Resist. 15 bytes'.........

//----------------------the following function 'check_Auto_D' will not be called now (7/Sept/2022--------Instead, 'Recv_Serial2' will be called----
// ------------------------------check data received from SimCrm1/Auto-D-, on Serial2 channel-----------------------------

// ..............................................end of 'Data from Auto-D'/sketch-- SimCrm1..............

// --------------------------------------------------------------------------
//   calculate K , spacing factor for 'a' , Wenner
// ..........................................................................
float WcalcK(float av) {
  float vr1, vr2;
  vr2 = 2 * 3.1416 * av;  // 2 * Pye * av

  return vr2;
}
// .............................................end of WcalcK..............
// --------------------------------------------------------------------------
//   calculate K , spacing factor for all Lv.lv--Schlumberger spacings
// ..........................................................................

float ScalcK(float Lv, float lv) {
  float vr1, vr2;

  vr1 = Lv / lv;
  vr2 = ((3.1416 * lv) / 2) * (vr1 * vr1 - 1);
  return vr2;
}
// .............................................end of ScalcK..............
// --------------------------------------------------------------------------
//   calculate K , spacing factor for all Lv.lv--Schlumberger spacings
// ..........................................................................

float DipcalcK(unsigned int a, unsigned int n) {
  float vr1;

  vr1 = (3.1416 * n * (n + 1) * (n + 2)) * a;  // pye*n*n+1*n+2 * a
  return vr1;
}
// .............................................end of DipcalcK..............
// --------------- check for data on D22, when there is a falling level on D21. Defined in setup()
// ---------------------------------------------------------------------------
// void ISR_clk_pin()

// ....... end of ISR ......................................

// -----------------------------------------------------------------------------
void Erase1(void) {
}
// ........................................... end of Erase1..................
// -----------------------------------------------------------------------------
void Erase2(void) {
}
// ........................................... end of Erase2..................
//--------------------------------------------------------------------------------------------------------------------
//  show Hex values of bytes  receivd (if (Actkn2==0) ) & then make Actkn2=1
//--------------------------------------------------------------------------------------------------
void get_Hex(byte x) {
  n15 = x & 0x0F;
  if (n15 <= 9) ch6 = 0x30 + n15;
  else ch6 = 0x41 + (n15 - 10);  // calculate 1st nibble (lower nibble only) (later try st3=String (n14,HEX);)
  //GUI_DisChar(160+8+(j5*20),y7, ch6, &Font12, WHITE,BLUE);// show 1st nibble (ch6)

  n15 = (x & 0xF0) >> 4;
  if (n15 <= 9) ch8 = 0x30 + n15;
  else ch8 = 0x41 + (n15 - 10);  //calculate 2nd nibble (ch8)
  //GUI_DisChar(160+(j5*20),y7, ch8, &Font12, WHITE,BLUE);   //  show 2nd nibble(ch8), to the left of 1st byte
}
//--------------------------------------------------------------------------------------------------------------------
//  show bytes receivd (if (Actkn2==0) ) & then make Actkn2=1
//--------------------------------------------------------------------------------------------------
void show_ByRcvd() {
}
//.........................................................................................
// -----------------------------------------------------------
// void Updt_RecD(void)-- show received data (j2new,j3new defined in ISR)
//  ----------------------------------------------------
void Updt_RecD(void) {  //1.1{

}  // 1.1}
//  ......................end of Updt_RecD  ...................

//-----------------------------------------------------------------------------
//  A1_Power()  :--- A1 card communication
//  ------------------------------------------------------------------------
void A1_Power() {
}

// .................... end of A1_Connect...............................
//---------------------------------------------------------show keycode-----
//  check_Keyboard() (check if a key hs been pressed . show hex code at rect. (100,180,  124,192)updt
// -------------------------------------------------------------------------
void check_Keyboard() {
}
// .....................................................end of check_Keyboard()...........
//---------------------------------------------------------------------------
// ---------------------------------------copied from sketch Anv2--9/Sept/2022------------------------------------------
// show L,l stored in E2prom (Lint1, lint1)
//----------------------------------------------------------------------------------------------
void Show_Eprom2(unsigned int L, unsigned int l) {
  volatile float fltL, fltl, vr5, Kv;
  //----- note that L=10 * actual value, similarly l= l0* actual value
  Ldig1 = L % 10;
  Lint3 = L / 10;
  ldig1 = l % 10;
  lint3 = l / 10;  // when Ldig1=0, it means that L is a multiple of 1. similarly for 'l'
  fltL = (float)L / 10.0;
  fltl = (float)l / 10.0;
  Kv = ScalcK(fltL, fltl);                                                     //
  GUI_DrawRectangle(4, 96 + 16, 430, 140, Colr[7], DRAW_FULL, DOT_PIXEL_DFT);  // erase rect 426x28
  n14 = LSpcN2;                                                                //Spacing n0.
  if (n14 == 0) {
    lcd1.clear();
    lcd1.setCursor(0, 0);
    lcd1.print("Sr No ");
    lcd1.setCursor(0, 1);
    lcd1.print("L= ");
    lcd1.setCursor(9, 1);
    lcd1.print("l= ");  // Sr. no. etc.
    lcd1.setCursor(0, 2);
    lcd1.print("K= ");
  }  // "Sr No" at (0,0), "L=" & "l=" at (0,1) &(11,1) & "K=" at (0,2)

  GUI_DisNum(5, 124, n14 + 1, &Font16, WHITE, BROWN);
  lcd1.setCursor(6, 0);
  lcd1.print(n14 + 1);  // Srl. No,.1st line
  if (Ldig1 == 0) {
    GUI_DisNum(40, 124, Lint3, &Font16, WHITE, BROWN);  // L integer, 2nd line
    lcd1.setCursor(3, 1);
    lcd1.print("L= ");
    lcd1.print(Lint3);
    lcd1.print("   ");
  } else {
    dtostrf(fltL, 7, 1, st1);  // L-Float, 2nd line
    GUI_DisString_EN(40, 124, &st1[0], &Font16, WHITE, BROWN);
    lcd1.print("L= ");
    lcd1.setCursor(3, 1);
    lcd1.print(fltL, 1);
  }
  if (ldig1 == 0) {
    GUI_DisNum(110, 124, lint3, &Font16, WHITE, BROWN);  // 'l'- integer,, 2nd line
    lcd1.setCursor(11, 1);
    lcd1.print("L= ");
    lcd1.print(lint3);
    lcd1.print("   ");
  } else {
    dtostrf(fltl, 7, 1, st1);  // 'l'-, 2nd linefloat
    GUI_DisString_EN(110, 124, &st1[0], &Font16, WHITE, BROWN);
    lcd1.setCursor(11, 1);
    lcd1.print("L= ");
    lcd1.print(fltl, 1);
  }
  dtostrf(Kv, 7, 2, st1);
  GUI_DisString_EN(200, 124, &st1[0], &Font16, WHITE, BROWN);
  lcd1.setCursor(2, 2);
  lcd1.print("K= ");
  lcd1.print(Kv, 2);  // calculated 40 lines
}
// .........................................................end of Show_Eprom2 ...copied on .9/Sept/2022.......................
// ---------------------------------------Show_Epro3 (modified from Show_Eprom2)------------------------------------------
// show L,l stored in E2prom (Lint1, lint1)
//-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-..-.-.-.-...-..-.-..-..-..--.---.-.--.-.-.--.--.---------------------
void Show_Eprom3(unsigned int L, unsigned int l) {
  volatile float fltL, fltl, vr5, Kv;
  //----- note that L=10 * actual value, similarly l= l0* actual value
  Ldig1 = L % 10;
  Lint3 = L / 10;
  ldig1 = l % 10;
  lint3 = l / 10;  // when Ldig1=0, it means that L is a multiple of 10. similarly for 'l'
  fltL = (float)L / 10.0;
  fltl = (float)l / 10.0;
  Kv = ScalcK(fltL, fltl);                                                     //
  GUI_DrawRectangle(4, 96 + 16, 430, 140, Colr[7], DRAW_FULL, DOT_PIXEL_DFT);  // erase rect 426x28
  //if (n14==0) {lcd1.clear();  lcd1.setCursor(0,0);lcd1.print("Sr No ");  lcd1.setCursor(0,1);lcd1.print("L= "); lcd1.setCursor(9,1);lcd1.print("l= "); // Sr. no. etc.
  //lcd1.setCursor(0,2);lcd1.print("K= ");}  // "Sr No" at (0,0), "L=" & "l=" at (0,1) &(11,1) & "K=" at (0,2)

  GUI_DisNum(5, 124, n14 + 1, &Font16, WHITE, BROWN);
  lcd1.setCursor(9, 0);
  lcd1.print("     ");
  lcd1.setCursor(10, 0);
  lcd1.print(n14 + 1);  // Srl. No,.1st line, LSpcN2+1 (n14+1, for 1~n numbering
  if (Ldig1 == 0) {
    GUI_DisNum(40, 124, Lint3, &Font16, WHITE, BROWN);
    lcd1.setCursor(2, 1);
    lcd1.print("                 ");
    lcd1.setCursor(2, 1);
    lcd1.print("L=");
    lcd1.print(Lint3);
  }  // L integer, 2nd line lcd1.print("   ");posn-2
  else {
    dtostrf(fltL, 7, 1, st1);  // L-Float, 2nd line posn-2
    GUI_DisString_EN(40, 124, &st1[0], &Font16, WHITE, BROWN);
    lcd1.setCursor(2, 1);
    lcd1.print("L=");
    lcd1.print(fltL, 1);
  }
  if (ldig1 == 0) {
    GUI_DisNum(110, 124, lint3, &Font16, WHITE, BROWN);  // 'l'- integer,, 2nd line,posn-8
    lcd1.setCursor(6, 1);
    lcd1.print("l=");
    lcd1.print(lint3);
  } else {
    dtostrf(fltl, 7, 1, st1);  // 'l'-, 2nd line,posn-8 float lcd1.print("   ");
    GUI_DisString_EN(110, 124, &st1[0], &Font16, WHITE, BROWN);
    lcd1.setCursor(6, 1);
    lcd1.print("l=");
    lcd1.print(fltl, 1);
  }

  dtostrf(Kv, 7, 2, st1);
  GUI_DisString_EN(200, 124, &st1[0], &Font16, WHITE, BROWN);
  lcd1.setCursor(11, 1);
  lcd1.print("        ");
  lcd1.setCursor(11, 1);
  lcd1.print("K=");
  lcd1.print(Kv, 2);  // Kv, 2nd line, posn-12
  tRho = 5.4 * Kv;
  lcd1.setCursor(2, 3);
  lcd1.print("Rho=");
  lcd1.print(tRho, 2);  // Rho in line-3
  //lcd1.setCursor(10,0); lcd1.print(LSpcN2);  // updated val  //ue of Last spacing no.
}
//................................................................end of Show_Eprom3
// show status of 4 keyboard pins Note: new name:-- 'Kb_Action'
//----------------------------------------------------------------------------
void Kb_Action() {
  timr2++;
  RetL2 = 0x41;  // xr4=50,yr4=150, im2=20, initially. m6 goes from 0~ im2(==20)
                 //  if (timr2>= 5) {timr2=0;

  volatile int xt1, xt2, yt1, yt2;
  xt1 = xr4 + (12 * m6);
  //---------------following statement was an error-----------
  //lcd1.setCursor(0, 0); lcd1.print('F'); keyBf0 = 0;
  //..........
  if (keyBf0 != 0) {  //   some key has definnitly been pressed
    lcd1.setCursor(0, 0);
    lcd1.print(' ');                                                                                     // some key is pressed                                                                              // some key is presed
    if (m6 >= im2) {                                                                                     // after 20 chars. are shown , m6 is made = 0
      GUI_DrawRectangle(xr4 - 1, yr4 - 1, xr4 + im2 * 12, yr4 + 18, Colr[7], DRAW_FULL, DOT_PIXEL_DFT);  // erase rect. for ~30 chars.
      m6 = 0;
    }
    //}
    //-------------------------------'F' key pressed---------------------------------------------------
    if (keyBf0 == k_F) {  //Serial.println("F2 mode just set");
      lcd1.clear();
      lcd1.setCursor(m6, 2);
      lcd1.print('F');
      lcd1.setCursor(0, 0);
      lcd1.print('F');
      GUI_DisChar(xt1, yr4, 'F', &Font16, WHITE, BLUE);
      kpr = 1;
      Fpr = 'F';  // if (kpr=2) (F-Mode was G~P, it should go back to 'F' mode, when 'F' key is pressed
      F_kpr = 1;
      keyBf0 = 0;
      m6++;  //  m6++ means cursor should advance (on Screen Lcd)just as it advaces on all other 15 keys
    }

    //..................................................'F' key over
    //---------------------------------- for keys 1~9,'.'-- kpr should be == 1/2-------------------------------------
    //----------------when we make keyBf0=0, we mean that the key has been honored ---------------------------------
    else {        // now study keys other than 'F' i.e. 0~9,keys :--'.'previous,next,clear & save
      F_kpr = 0;  //F_kpr{ is used in 10 milli-Second for flashin 'F' is now removed

      if (keyBf0 == k_1) {
        if (Fpr != 'L') {  //normal operation of key '1'
          if (kpr == 1) {
            lcd1.clear();
            lcd1.setCursor(m6, 2);  //
            //lcd1.write(0xE0);
            lcd1.setCursor(0, 0);
            lcd1.print("F1");
            lcd1.setCursor(0, 1);
            lcd1.print("for GPS (if module available");  //'alpha' show 'alpha' at(0,0) too. 'alpha' overwritten by 'entry_fnc_G()'
            GUI_DisChar(xt1, yr4, '1', &Font16, WHITE, BLUE);
            Fpr = 'F' + 1;  //'G' mode
            kpr = 2;        // 'if (kpr==1)' is equivalent t0 'if (Fpr=='F''Function set' mode print '1'
                            // Get_GPS();      // Reject_k();  //    entry_fnc_G();now cancelled--8/may/2023 'alpha' means 'GPS' mode, Fpr='G'
            keyBf0 = 0;     //...............................................................
          }
        }
      }
      //----------'end of key '1' normal operation------------------------
      //----------------begin  key-'1' operation for 'change of 'Schlumberger/enner/Dipole-Dipole mode---
      if (keyBf0 == k_1) {
        if (Fpr == 'L') {  // change to Sclumberger method'
          if (kpr == 2) {  // note : 2 keys have been pressed
            lcd1.clear();
            lcd1.setCursor(m6, 2);  //
                                    // lcd1.write(0xE0);
            lcd1.setCursor(0, 0);
            lcd1.print("Setting ");  //
            lcd1.setCursor(0, 1);
            lcd1.print("Schlumberger method");  //'
            GUI_DisChar(xt1, yr4, '1', &Font16, WHITE, BLUE);
            // do not change

            Fpr = 'F' + 1;  //'G' mode
            kpr = 2;        // 'if (kpr==1)' is equivalent t0 'if (Fpr=='F''Function set' mode print '1'
            Surv_meth = 1;  // set Schlum meth0d

            L_Init1();   // Get_GPS();      // Reject_k();  //    entry_fnc_G();now cancelled--8/may/2023 'alpha' means 'GPS' mode, Fpr='G'
            keyBf0 = 0;  //...............................................................
          }
        }
      }
      // ------------------------------------------------------------
      //------------------------------------ key 2 ----------------------------------------------------------------------------
      if (keyBf0 == k_2) {  // normal operation of key '2'
        if (Fpr != 'L') {
          if (kpr == 1) {

            lcd1.clear();
            lcd1.setCursor(m6, 2);
            // lcd1.print("F2");
            lcd1.setCursor(0, 0);
            lcd1.print("F2  key 2 pressed");  // F2'Sigma',  Earlier 'Beta'.
            GUI_DisChar(xt1, yr4, '2', &Font16, WHITE, BLUE);
            Fpr = 'F' + 2;  // Fpr <-- 'H'
            kpr = 2;
            kpr_key_2 = 2;  //keys 'F' &'2'are now pressed. now deteect this in key '9' operation
            keyBf0 = 0;
            Serial.println("key 2 pressed, after F");
            entry_fnc_H();  //now Fpr='H','Sigma' normal Survey mode
          }
        }  //  ----set H-mode-----('F'+2)
      }
      //----------------begin  key-'2' operation for 'change of 'Schlumberger/enner/Dipole-Dipole mode---
      if (keyBf0 == k_2) {
        if (Fpr == 'L') {  //change Srvey mode
          if (kpr == 2) {

            lcd1.clear();
            lcd1.setCursor(m6, 2);
            // lcd1.print("F2");

            lcd1.setCursor(0, 0);
            lcd1.print("Setting ");  //
            lcd1.setCursor(0, 1);
            lcd1.print("Wenner method");  //
            GUI_DisChar(xt1, yr4, '2', &Font16, WHITE, BLUE);
            //  --- do not chang Fpr now
            //Fpr = 'F' + 2;  // Fpr <-- 'H'
            kpr = 2;
            kpr_key_2 = 2;  //keys 'F' &'2'are now pressed. now deteect this in key '9' operation
            Surv_meth = 2;  // set Wenner meth0d

            L_Init1();  //
            keyBf0 = 0;
            Serial.println("change Survey mode");
            //entry_fnc_H();  //now Fpr='H','Sigma' normal Survey mode
          }
        }  //  ----set H-mode-----('F'+2)
      }
      //................end of  key-'2' operation for 'change of 'Schlumberger/enner/Dipole-Dipole mode.....--
      //-------------------key-'3'---------------------------------------------------------
      if (keyBf0 == k_3) {  //mnormal key '3' operation
        if (Fpr != 'L') {
          if (kpr == 1) {
            lcd1.setCursor(m6, 2);  //
                                    // lcd1.print("F3");
            lcd1.setCursor(0, 0);
            lcd1.print("F3");
            lcd1.print(" key 3 pressed");  //
            GUI_DisChar(xt1, yr4, '3', &Font16, WHITE, BLUE);
            kpr = 2;
            Fpr = 'I';  // now Fpr = 'F'+3 (='I') is ,edit L,l, mode
            print_save = 1;
            entry_fnc_I();
            keyBf0 = 0;
          }
        }  //  Reject_k(); keyBf0=0 means key-3 is honoured------  ----- now--------
      }
      //----------------begin  key-'3' operation for 'change of 'Schlumberger/enner/Dipole-Dipole mode
      if (keyBf0 == k_3) {  //mnormal key '3' operation
        if (Fpr == 'L') {
          if (kpr == 2) {
            lcd1.setCursor(m6, 2);  //
                                    // lcd1.print("F3");
            lcd1.setCursor(0, 0);
            lcd1.print("Setting ");  //
            lcd1.setCursor(0, 1);
            lcd1.print(" Dipole method");  //
            GUI_DisChar(xt1, yr4, '3', &Font16, WHITE, BLUE);
            kpr = 2;  // no use
                      // do not change Fpr now
                      //Fpr = 'I';  // now Fpr = 'F'+3 (='I') is ,edit L,l, mode
            print_save = 1;
            Surv_meth = 3;  // set Wenner meth0d
            L_Init1();      //entry_fnc_I();
            keyBf0 = 0;
          }
        }  //  Reject_k(); keyBf0=0 means key-3 is honoured------  ----- now--------
      }
      //................end of  key-'3' operation for 'change of 'Schlumberger/enner/Dipole-Dipole mode.....--
      //-------------------------------
      //-------------------ket-'4'---------------------------------------------------------
      if (keyBf0 == k_4) {
        if (kpr == 1) {
          lcd1.setCursor(m6, 2);
          // lcd1.print("F4");
          lcd1.setCursor(0, 0);
          lcd1.print("F4");
          lcd1.print(" key 4 pressed");  //'
          GUI_DisChar(xt1, yr4, '4', &Font16, WHITE, BLUE);
          Fpr = 'F' + 4;  // 'J' mode
          kpr = 2;        // --set 'J' mode-- ('F'+4) ('Survey Status' mode)
          entry_fnc_J();  // entry_fnc_J(), means: 'show  1st screen of Survey status'
          keyBf0 = 0;
        }
      }
      //-------------------key-'5'-----------------------------------------------------------
      if (keyBf0 == k_5) {
        if (kpr == 1) {
          lcd1.setCursor(m6, 2);
          // lcd1.print("F5");
          lcd1.setCursor(0, 0);
          lcd1.print("F5");
          lcd1.print(" key 5 pressed");  //'

          // 'Sigma',show 'Sigma' at(0,0)
          GUI_DisChar(xt1, yr4, '6', &Font16, WHITE, BLUE);
          Fpr = 'F' + 5;  //  'K' mode
          kpr = 2;        //  --set 'L' mode-- ('F'+6)
          L_Init1();      //Reject_k();
          keyBf0 = 0;
        }
      }
      //-----------------------key 6-------
      //-------------------key-'6'-----------------------------------------------------------
      if (keyBf0 == k_6) {
        if (kpr == 1) {
          lcd1.setCursor(m6, 2);
          // lcd1.print("F6");
          lcd1.setCursor(0, 0);
          lcd1.print("F6");
          lcd1.setCursor(0, 1);
          lcd1.print("Setting L mode");  //'


          // 'Sigma',show 'Sigma' at(0,0)
          GUI_DisChar(xt1, yr4, '6', &Font16, WHITE, BLUE);
          Fpr = 'F' + 6;  // 'L' mode
          kpr = 2;        //  --set 'L' mode-- ('F'+6)
          keyBf0 = 0;
          L_Init1();  //Init1();  //Reject_k();
        }
      }
      //----------------------key-7'----------------------------------------------------------------
      if (keyBf0 == k_7) {  // key '7'
        if (kpr == 1) {     // key 'F' was pressed prior to key '7'
          lcd1.clear();
          lcd1.setCursor(0, 0);
          lcd1.print("F7 'M' mode");  //'Pye', show 'Pye' at(0,0) too lcd1.print('7');
          GUI_DisChar(xt1, yr4, '7', &Font16, WHITE, BLUE);
          Fpr = 'M';         // 'M' mode 'F'+ 7 ,  when 'F'is pressed followed by,'7'
          kpr = 2;           //2.keys 'F' &  7 have been pressed
          void enter_L_l();  //prove that values of L,l were stored correctly
          //-----------------keyBf0 <-- is cancelled here-----------because of this.program will go thru Kb_Action
          //magain bu this time it will execute t tcde patch

          keyBf0 = 0;
          //E2prom_get() ; // read RcBf_R1[16 bytes from EEPROM] calc_Batt() ;  // show received bytes on Serial Moniter   //Reject_k();
        }
      }
      //---------------------key 7--again with if(kr==2)------------------

      //........................end of key--again.........................
      //----------------------key-'8'--------------------------------------------------------------------
      if (keyBf0 == k_8) {
        if (kpr == 1) {
          lcd1.clear();
          lcd1.setCursor(0, 0);
          lcd1.print("F8 Edit L, l mode");  //'Rect.''DBh',show 'E-mirror' at(0,0) too,  lcd1.print('8');
          GUI_DisChar(xt1, yr4, '8', &Font16, WHITE, BLUE);
          Fpr = 'F' + 8;  // 'N'
          kpr = 2;        //  --set 'N' mode-- ('F'+8)
                          //-------------------------------key 2nd action--------------------------
          lcd1.setCursor(5, 2);
          lcd1.print("L(");
          lcd1.print(EdSpc_No);
          lcd1.print(")= ");
          lcd1.print(pgm_read_word_near(PLt + EdSpc_No));
          //pgm  (11)= 234 ");
          //.................................end of ' 2nd action............
          keyBf0 = 0;
        }
      }
      //----------------------------key-'.'--this mode not to be used---------------------------------------------------
      if (keyBf0 == k_dot) {

        if (kpr == 1) {
          lcd1.clear();
          lcd1.setCursor(m6, 2);
          lcd1.print("Fdot");
          lcd1.setCursor(0, 0);
          lcd1.print("Fdot");  //'Omicron',show 'Omicron' at(0,0) too,  lcd1.print('.');
          GUI_DisChar(xt1, yr4, '.', &Font16, WHITE, BLUE);
          Fpr = 'R';  // = 'P'
          kpr = 2;    // ----'P' mode-- (not used)
          keyBf0 = 0;
          //Reject_k();  // presently, 'F'+ '.' does not deffine any function mode
        }
      }

      //----------------------key-'9'--------------------------------------------------------------------

      if (keyBf0 == k_9) {
        if (kpr == 1) {
          lcd1.clear();
          lcd1.setCursor(m6, 2);
          lcd1.print("F9");
          lcd1.setCursor(0, 0);
          lcd1.print("F9");
          lcd1.print(" key 9 pressed");  //''E-mirror''D6h'' lcd1.print('9');
          GUI_DisChar(xt1, yr4, '9', &Font16, WHITE, BLUE);
          Fpr = 'F' + 9;  //  i.e. 'O' (oh) mode
          kpr = 2;        // Fpr= 'O' ('Oh')
                          //if (kpr_key_2 == 2)  Serial.println("key 9 pressed, after 'F' & '2' ");
          keyBf0 = 0;
        }
      }
      //----------------------key-''F' then key-2''-(now kpr=2 -& key '9'---------------------------------------------------------------
      if (keyBf0 == k_9) {
        if (kpr == 2) {  // note : now kpr=2
          lcd1.clear();
          lcd1.setCursor(5, 0);
          lcd1.print("keys F,2,now , ");
          lcd1.setCursor(0, 1);
          lcd1.print("9 pressed");  //''E-mirror''D6h'' lcd1.print('9');
          GUI_DisChar(xt1, yr4, '9', &Font16, WHITE, BLUE);
          Fpr = 'P';  //'F' + 9;  // use Fpr ='P'
          kpr = 2;    // Fpr= 'O' ('Oh')
          fnc_Q2();   // turn On  AutoD
          keyBf0 = 0;
          //Reject_k();
        }
      }
      //------------when key '0' follows 'F' key, 'Test' mode is defined ----------------------------------------
      if (keyBf0 == k_zr) {
        if (kpr == 1) {
          lcd1.clear();
          lcd1.setCursor(m6, 2);
          lcd1.print("F0");
          lcd1.setCursor(0, 0);
          lcd1.print("F0");  // 'F0''tau-modified',show at (0,0) & (m6,2), lcd1.write(0xCE);
          GUI_DisChar(xt1, yr4, '0', &Font16, WHITE, BLUE);
          Fpr = 'F' + 11;  //= 'Q'

          kpr = 2;     // Fpr= 'Q', Test mode. ('F'+11)
          keyBf0 = 0;  // show 'Resistance' only. No L,l,K and 'Rho'
        }
      }
      //..................................end of 'if (kpr==1)'.cases...........keys 1~8 & '0'..................
      //.................................................................................................
      //  ------ end of 'if (kpr not =0)' ---(but keys '9' & '0' not included in defining F-mode--------------------------------------
      //  should use EEPRM.get EEPRPm.put (EEPROM.update)E2 to modify the value
      //-------------  --------------
      //--------------------------------- ----------------------------------------------------------------
      // --- kpr==0 means that after 'Power On' the 'F" key has not yet been  pressed ----

      //-----------------------show kpr(int), Fpr(char)-----------------------------------------------------------------------
      // ---- the following 6-line block is executed when any key is pressed --------------------------      //
      //.....................................................................................................................

      if (keyBf0 == k_pr) {
        if (kpr == 1) {
          lcd1.clear();
          lcd1.setCursor(0, 0);
          lcd1.print("F8 Edit L, l mode");  //  -- do not print char'P' (for 'Previous' on screen & 20x4lcd1.print('P');
          lcd1.setCursor(0, 1);
          lcd1.print("Prev. key pressed");  //
          lcd1.setCursor(3, 2);             //GUI_DisString_EN (xt1, yr4, "P", &Font16, WHITE, BLUE);-
          if (Fpr != 'H' && Fpr != 'I' && Fpr != 'K' && Fpr != 'L') keyBf0 = 0;
          Fpr = 'S';
          if (Fpr == 'N') {
            if (EdSpc_No >= 2) EdSpc_No--;
            lcd1.print("L(");
            lcd1.print(EdSpc_No);
            lcd1.print(")= ");
            lcd1.print(pgm_read_word_near(PLt + EdSpc_No));

            kpr = 2;
            keyBf0 = 0;
          }
        }
      }  //   //minimumm EdSpc_No is 1
         //...............................................................................................
      if (keyBf0 == k_nx) {
        if (kpr == 1) {
          lcd1.clear();
          lcd1.setCursor(0, 0);
          lcd1.print("F8 Edit L, l mode");  //
          lcd1.setCursor(0, 1);
          lcd1.print("Next key pressed");

          lcd1.setCursor(3, 2);  //  do not print char.N- for 'Next' lcd1.print('N');
                                 //GUI_DisString_EN (xt1, yr4, "N", &Font16, WHITE, BLUE);  //
                                 //  ++;  maximum value of,EdSpc_No is 36

          if (Fpr != 'H' && Fpr != 'I' && Fpr != 'K') keyBf0 = 0;  //keyBf0 will be made 0 only if  'not' Survey/Edit L,l/Recal Reading mode
          Fpr = 'T';
          if (Fpr == 'N') {
            if (EdSpc_No <= 35) EdSpc_No++;
            lcd1.print("L(");
            lcd1.print(EdSpc_No);
            lcd1.print(")= ");
            lcd1.print(pgm_read_word_near(PLt + EdSpc_No));
          }
          kpr = 2;
          keyBf0 = 0;
        }  //
        kpr = 2;
        keyBf0 = 0;
      }
      //....................................................
      if (keyBf0 == k_cl) {
        lcd1.clear();
        lcd1.setCursor(m6, 2);
        lcd1.print("Clear");
        if (kpr == 1) {
          Fpr = 'U';
          // j3a<--0 is probably needed
          GUI_DisString_EN(xt1, yr4, "C", &Font16, WHITE, BLUE);  // print char C for- 'Clear'
          keyBf0 = 0;                                             //keyBf0 will be made 0 ,toward the end of Kb_Action
        }
      }
      if (keyBf0 == k_sv) {  // for normal operation of key 'Save'
        if (Fpr != 'I') {
          lcd1.clear();
          lcd1.setCursor(m6, 2);  //
          lcd1.print("key: Save");
          //GUI_DisString_EN (xt1, yr4, "S", &Font16, WHITE, BLUE);  // print char S for- 'Save'
          if (Fpr != 'H' && Fpr != 'I') keyBf0 = 0;  // ( keyBf0 will not be made 0 in F2('Normal Survey'mode) and in F3(Edit L,l mode)
        }
      }
      if (keyBf0 == k_sv) {  //operation of key in 'Edit mode'
        if (Fpr == 'I') {    // key 'F'followed
          lcd1.setCursor(0, 3);
          lcd1.print("                   ");  // Erase entire line. char. posn.=0~19
          lcd1.setCursor(0, 3);
          lcd1.print("--Reading Saved--");                        // saved in EEPROM
          GUI_DisString_EN(xt1, yr4, "S", &Font16, WHITE, BLUE);  // print char S for- 'Save'
          if (Fpr != 'H' && Fpr != 'I') keyBf0 = 0;               //
                                                                  // Commented out:-- entire line"if (Fpr != 'H' && Fpr != 'I') keyBf0 = 0;" date22/Jan/2025, Tuesday
        }
      }
      // after taking the action for a key, keyBf0 is cleared to 0 , which means the action has been taken

      m6++;  // advance the 'cursor' to next position
    }
    //  end of case: 'keys other than 'F' , i.e. ,0,~9 & 'dot' plus 'prev,next,clear & Save keys
    //------i. e. end of last key ,namely key 'Save'

    //---------------------------- '---------------
    if (Fpr == 'M') {
      if (kpr == 2) {
        if (keyBf0 == k_2) {
          enter_L_l();  // show lines 0 & 1
          kpr = 3;
          keyBf0 = 0;
        }
      }
    }  //
    // ----------- accept  0~9 keys-------------------------
    if (Fpr == 'G') {
      if (keyBf0 == k_2) {
        fnc_G2();
        keyBf0 = 0;
      }  // if key-2 is pressed, get GPS readings
      if (keyBf0 == k_3) {
        fnc_G3();
        keyBf0 = 0;
      }  // if key-3 is pressed
      if (keyBf0 == k_4) {
        fnc_G4();
        keyBf0 = 0;
      }  // if key-4 is pressed
    }    //  end of 'if Fpr=='G''
         //.................................end of keys 2,3,4 actions for'G', GPS...................................
    //...........................................end of keys 2,3 for 'GPS' (alpha)....................................
    //===========--------------begn-'H',keys 2,3. for 'Normal Survey' mode-----------------------------------------------------
    //-----------------------------------'H'-- keys 2,3,4,5 within 'Normal Survey' mode-------------------------------------------------
    if (Fpr == 'H') {  // 'H', F+'2' 'Sigma',Normal Survey mode
      if (keyBf0 == k_2) {
        fnc_H2();
        keyBf0 = 0;
      }  // if key-2 is pressed, block
      if (keyBf0 == k_3) {
        fnc_H3();
        keyBf0 = 0;
      }  // if key-3 is pressed      //
      if (keyBf0 == k_4) {
        fnc_H4();
        keyBf0 = 0;
      }  // if key-4 is pressed

      if (keyBf0 == k_pr) {
        fnc_H_Prv();
        keyBf0 = 0;
      }                      // if key-'Previous' is pressed
      if (keyBf0 == k_nx) {  // if key-'next' is pressed

        fnc_H_Nxt();
        keyBf0 = 0;
      }
      if (keyBf0 == k_6) {
        fnc_H6();
        keyBf0 = 0;
      }  // if key-6 is pressed,like pressing F2, call entry_fnc_H
      if (keyBf0 == k_9) {
        Serial.println("F2 mode key-9 to be pressed");
        fnc_H9();
        keyBf0 = 0;
      }  // if key-9 is pressed, turn on Auto-D
    }    //  end of  Fpr=='H'
         // . . . . . . . . . . . . . . . . . . . . . . . . . .       //

    //.................................end of 'Normal Survey mode'..()................................
    //...........................................end of keys 2,3 for 'H', Normal Survey mode..('Sigma.)....................................
    //-----------------------------------'Q'--'Test mode, Resistance only'-------------------------------------------------
    if (Fpr == 'Q') {
      if (keyBf0 == k_6) {
        fnc_Q1();
        keyBf0 = 0;
      }  // if key-6 is pressed, call entry_fnc_Q, like pressing 'F0'
      if (keyBf0 == k_9) {
        fnc_Q2();
        keyBf0 = 0;
      }  // if key-9 is pressed,   D27<-- '0' givng a pulse to turn on Auto_D
    }    //  end of 'if Fpr=='Q'' */
         //.................................end of keys 2,3 actions
         //  /*
    if (Fpr == 'K') {
      if (keyBf0 == k_pr) {
        fnc_K2();
        keyBf0 = 0;
      }  // if key-'previous'' is pressed, show previous reading
      if (keyBf0 == k_nx) {
        fnc_K3();
        keyBf0 = 0;
      }  // if key-'next' is pressed, show next reading
    }    //  end of 'if Fpr=='Q'' */
    //.................................end of keys 2,3 actions    void entry_fnc_K() {  }    // to be defined 'show Rdng_N0,L,Rho
    //--------------------------------------- 'L'  select Survey mode-------------------
    if (Fpr == 'L') {
      if (kpr == 2) {
        if (keyBf0 == k_1) {
          L_Scr_Schlum();
          keyBf0 = 0;
        }  // if key '1' is pressed, set Schlumberger
        if (keyBf0 == k_2) {
          L_Scr_Wenn();
          keyBf0 = 0;
        }  // if key '2' is pressed, set Wenner method
        if (keyBf0 == k_3) {
          L_Scr_Dip();
          keyBf0 = 0;
        }  // if key '3' is pressed, set Dipole method
        if (keyBf0 == k_4) {
          L_No_change();
          keyBf0 = 0;  //if key '4' is pressed,no change
        }              //
      }
    }
    //---------------------------- '---------------

    //-----------------------------------------------------------------------------------
    if (Fpr == 'J') {
      if (keyBf0 == k_3) {
        fnc_J3();
        keyBf0 = 0;
      }  // if key '3' is pressed, call fnc_J3(),close present Srvey
         // if (keyBf0 == k_4) {
      // fnc_J4();
      //keyBf0 = 0;
      //        }  //
    }
    //  end of 'if Fpr=='J''
    //-----------------------------------'I'--'Edit L,l'-------------------------------------------------
    if (Fpr == 'I') {
      if (keyBf0 == k_nx) {
        fnc_I2();
        keyBf0 = 0;
      }  // if key-next is pressed, get next readings
      if (keyBf0 == k_pr) {
        fnc_I3();
        keyBf0 = 0;
      }  //if key-'previous' is pressed
      if (keyBf0 == k_1) {
        fnc_I4();
        keyBf0 = 0;
      }  // if key-1 is pressed, L++
      if (keyBf0 == k_4) {
        fnc_I5();
        keyBf0 = 0;
      }  //if key-4 is pressed,L--
      if (keyBf0 == k_2) {
        fnc_I6();
        keyBf0 = 0;
      }  // if key-2 is pressed, l++
      if (keyBf0 == k_5) {
        fnc_I7();
        keyBf0 = 0;
      }  // if key-5 is pressed,l--
      if (keyBf0 == k_dot) {
        fnc_I9();
        keyBf0 = 0;
      }  // if key-'.' is pressed,l-- 'delta'= 0.1 m.'
      if (keyBf0 == k_zr) {
        fnc_I10();
        keyBf0 = 0;
      }
      if (keyBf0 == k_sv) {
        fnc_I8();
        keyBf0 = 0;
      }

    }  //end 'if (Fpr=='I''

    //...........................end of if (Fpr=='J').......................................................

    //*************************************////////  // only kesy-3 & 4 honoured
  }
}  // ...............end of Kb_Action............

//----------------------------------Reject+k() function definition-----------------------------------
void Reject_k() {
  lcd1.clear();
  lcd1.setCursor(0, 0);
  lcd1.print("F0,F2,F3,F4,F5,F6");
  lcd1.setCursor(0, 1);
  lcd1.print("---allowed only");
  // F0-Test mode, F2-Survey mode, F3-Edit L,l  'a' mode, F4-Survey Status mode, F5-Recall Readings mode, F6-Select Schlumberger/Wenner/Dipole-Dipole method
  lcd1.setCursor(0, 2);
  lcd1.print("F1,F7,F8,F9,F.");
  lcd1.setCursor(0, 3);
  lcd1.print(" ---not allowed");
}
//------------------------------------------------------------------------------------------------
//
void entry_fnc_J()  // just entering into 'J' Survey status mode
{
  lcd1.clear();
  lcd1.setCursor(0, 0);
  lcd1.print("F4");
  lcd1.setCursor(4, 0);
  lcd1.print(" Srv Stat ");  //'Mu',1,Srv_No,8(say)
  if (Surv_meth == 1) lcd1.print("--Schlum--");
  if (Surv_meth == 2) lcd1.print("--Wenn-");
  if (Surv_meth == 3) lcd1.print("--Dipole--");  // show 'Schlumberger' or Wenner
  lcd1.setCursor(0, 1);
  lcd1.print("                    ");
  lcd1.setCursor(0, 1);
  lcd1.print("Srv. ");
  lcd1.print(Srv_No);  // erase entire line-1 (20 spaces)
  lcd1.setCursor(8, 1);
  lcd1.print(" Rd");
  lcd1.print(LRdSr2);
  lcd1.setCursor(13, 1);  //this ensures that there will be blanks after LSpcN2
  if (Surv_meth == 1) lcd1.print("Sp");
  if (Surv_meth == 2) lcd1.print("Sp");
  lcd1.print(LSpcN2);  // Last Reading No. & Last Spacing No.,StRecord no.//
  lcd1.setCursor(0, 2);
  lcd1.print("St Rd--");
  lcd1.print(StRecrds);
  if (LRdSr2 != 0) lcd1.print(" F2 to Continu");  //  for displaying L,Rho

  lcd1.setCursor(0, 3);
  if (LRdSr2 == 0) lcd1.print("press F2 to continue");  // line-3 :if (Readings==0) " press F2 to continue normal survey "
  lcd1.setCursor(0, 3);
  if (LRdSr2 != 0) lcd1.print("press 3 for new Srv");  // line-3: if (Readings not=0)"press 3 for new srvey"
}
//.......................................end of entry into ' 'J'(survey status) mode'

//------------entry_fnc_J part 1.
void entry_fnc_J1()  // presently, (13/March/2023 ) this function is not selected by any key
{
  lcd1.clear();
  lcd1.setCursor(0, 0);
  lcd1.print("F4-1");
  lcd1.setCursor(4, 0);
  lcd1.print(" Srv Status ");  //'Mu',1,Srv_No,8(say)
  Surv_meth = 2;               // set Wenner mode
                               /*
  lcd1.setCursor(0, 1);  lcd1.print("                    "); lcd1.setCursor(0, 1); lcd1.print("Srv." ); lcd1.print(Srv_No); // erase entire line-1 (20 spaces)
  lcd1.setCursor(7, 1);lcd1.print(" Rd"); lcd1.print(LRdSr2); lcd1.setCursor(13, 1);//this ensures that there will be blanks after LSpcN2
   if (Surv_meth==1)lcd1.print("S-Sp");  if (Surv_meth==2) lcd1.print("W-Sp");    lcd1.print(LSpcN2);    // Last Reading No. & Last Spacing No.,StRecord no.//
      lcd1.setCursor(0, 2); lcd1.print("St Rd"); lcd1.print(StRecrds); //  for displaying L,Rho
          */
}
// ----------entry_fnc_J part 2-------------------------------
void entry_fnc_J2()  // presently, (13/March/2023 ) this function is not selected by any key
{
  lcd1.clear();
  lcd1.setCursor(0, 0);
  lcd1.print("F4-2 ");
  if (Surv_meth == 1) lcd1.print("Schlumberger");
  if (Surv_meth == 2) lcd1.print("Wenner");  // show 'Schlumberger' or Wenner
                                             /*
    lcd1.setCursor(0, 2);   if (LRdSr2==0)lcd1.print("press F2 to continue");  // line-3 :if (Readings==0) " press F2 to continue normal survey "
  lcd1.setCursor(0, 2); if (LRdSr2!=0)  lcd1.print("press 3 for new Srv");  // line-3: if (Readings not=0)"press 3 for new srvey"
             */
}
//.......................................end of entry into ' 'J'(survey status) mode'
//-----------------------------------------------------------------------------
void fnc_J2()  // presently, (13/March/2023 ) this function is not selected by any key
{
  lcd1.setCursor(0, 0);
  lcd1.print("F4");  // linn-0: 'mu',2
  lcd1.setCursor(0, 2);
  lcd1.print("                ");  // line-2 :erase2~19
  lcd1.setCursor(0, 3);
  lcd1.print("        ");  // line-3
}
//...............................'end of fnc_J2..................................................

//-----------------------------------------------------------------------------
void fnc_J3()  // key 3 pressed 3rd action within 'J' (Survey Status)(mu-3} New Survey no.,Readng=0, Spacing=0
{
  //linn-0: 'mu',2
  Srv_No++;  // Survey no. incremented
  LRdSr2 = 0;
  LSpcN2 = 0;
  StRecrds = 0;
  Str11 = "Srv" + String(Srv_No) + ".csv";
  for (m7 = 0; m7 <= 9; m7++) FName2[m7] = Str11[m7];  // copy 'file name'(10 CHARS.) into FName2[]
  lcd1.clear();
  lcd1.setCursor(0, 0);
  lcd1.print("F4 ");
  lcd1.setCursor(4, 0);
  lcd1.print("New Surv");
  lcd1.setCursor(12, 0);
  lcd1.print(" no. ");
  lcd1.print(Srv_No);  //
  tEA = EAd1;
  EEPROM.put(tEA, Srv_No);  // Survey no. =1 stored at addr=EAD1 - 1 time initialization
  tEA = EAd1 + (3 * 2);
  ;
  EEPROM.put(tEA, LRdSr2);  // Earlier:  write 0 at 3rd integer tEA=EAd1+(1*2) Last Reading Sr. No & l. spac. no.
  tEA = EAd1 + (4 * 2);
  EEPROM.put(tEA, LSpcN2);  // Earlier: write 0 at 4th integer tEA=EAd1+(2*2) ;
  tEA = EAd1 + (5 * 2);
  EEPROM.put(tEA, StRecrds);  // total no. of records in EAd6~EAd7
                              //  should use preferably (EEPROM.update)E2 to modify the value
  //-------show New Surv. no. & 0rdng, 9spacing
  lcd1.setCursor(0, 1);
  lcd1.print("                   ");
  lcd1.setCursor(0, 1);
  lcd1.print("Rd ");
  lcd1.print(LRdSr2);
  lcd1.print(" Sp ");
  lcd1.print(LSpcN2);
  lcd1.print(" StRd ");
  lcd1.print(StRecrds);  // erase line-1
                         // Last Reading No. & Last Spacing No.
                         //'Mu',1 lcd1.setCursor(4,0);  lcd1.print("Srv." ); lcd1.print(Srv_No); //'Mu',1
  lcd1.setCursor(0, 2);
  lcd1.print("                   ");
  lcd1.setCursor(0, 2);
  lcd1.print("press F0 or F2");  // line-2 :erase0~19
                                 // line-3: blank
}
//...............................'end of fnc_J3..................................................
//--------------------------------------------------fnc_j4----------------------------------
//-----------------------------------------------------------------------------

//------------------function stat_mod_J5---------------------------------------------------------------------------------------
void stat_mod_J5() {  // --- key 5 ---pressed----
  lcd1.clear();
  lcd1.setCursor(0, 0);
  lcd1.print("F4");
  lcd1.setCursor(4, 0);
  lcd1.print("New Surv");
  lcd1.setCursor(12, 0);
  lcd1.print(" no.");
  lcd1.print(Srv_No);  //
  Surv_meth = 1;
  tEA = EAd1 + (6 * IntSz);
  EEPROM.put(tEA, Surv_meth);  // ----store in E2prom 'Schlumberger'--------
  lcd1.setCursor(0, 1);
  lcd1.print("Schlumberger");
}
// .................................................end of fnc_J5...............................
//------------------function stat_mod_J6---------------------------------------------------------------------------------------
void stat_mod_j6() {  // ------ this function not used-----------
}
// .................................................end of fnc_J6...............................
//------------------function stat_mod_J7---------------------------------------------------------------------------------------
void stat_mod_J7() {  // --- key 7 ---pressed----
  lcd1.clear();
  lcd1.setCursor(0, 0);
  lcd1.print("F4");
  lcd1.setCursor(4, 0);
  lcd1.print("New Surv");
  lcd1.setCursor(12, 0);
  lcd1.print(" no.");
  lcd1.print(Srv_No);  //
  Surv_meth = 2;
  tEA = EAd1 + (6 * IntSz);
  EEPROM.put(tEA, Surv_meth);  // ---- Wenner--------
  lcd1.setCursor(0, 1);
  lcd1.print(" Wenner");
}
// .................................................end of stat_mod_J7...............................
//------------------function stat_mod_J8----------------------------------------------------------------------------------------------------
void stat_mod_J8() {  // --- key 8 ---pressed----
  lcd1.clear();
  lcd1.setCursor(0, 0);
  lcd1.print("F4");
  lcd1.setCursor(4, 0);
  lcd1.print(" New Surv");
  lcd1.setCursor(12, 0);
  lcd1.print(" no.");
  lcd1.print(Srv_No);  //
  Surv_meth = 3;
  tEA = EAd1 + (6 * IntSz);
  EEPROM.put(tEA, Surv_meth);  // ---- Dipole-Dipole--------
  lcd1.setCursor(0, 1);
  lcd1.print(" Dipole-Dipole");
}
// .................................................end of stat_mod_J8...............................
//------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------
void entry_fnc_I()  // 'edit L,l' mode
{
  if (Surv_meth == 1) {
    n15 = LSpcN2;
    MLlNo = n15;
    tEA = EAd4 + n15 * 4;
    EEPROM.get(tEA, MLvt);
    tEA += IntSz;
    EEPROM.get(tEA, Mlvt);  // MSpcN-spacing srl. no.for Memory: EAd5~EAd6 ,record size, L,l 4 bytes
    // EAd5 changed to EAd4 (9/march/2023
    tMLvt = MLvt;
    tMlvt = Mlvt;  // MLvt, Mlvt copied into 'temporary' values
    lcd1.clear();
    lcd1.setCursor(0, 0);
    lcd1.print("F3");
    ;
    lcd1.print("  Edit L,l ");
    lcd1.setCursor(13, 0);
    lcd1.print("MSp=");
    lcd1.print(MLlNo + 1);
    lcd1.setCursor(0, 1);
    lcd1.print("ML= ");
    fltLv = (float)MLvt / 10.0;
    lcd1.print(fltLv, 1);
    lcd1.setCursor(11, 1);
    lcd1.print(" Ml= ");
    fltlv = (float)Mlvt / 10.0;
    lcd1.print(fltlv, 1);  // L,l

    lcd1.setCursor(0, 2);
    Kvt = ScalcK(fltLv, fltlv);
    lcd1.print("K= ");
    lcd1.print(Kvt, 2);
  }
  if (Surv_meth == 2)  // Wenner
  {
    n15 = LSpcN2;
    MLlNo = n15;
    tEA = EAd5 + n15 * 2;
    EEPROM.get(tEA, MLvt);  //tEA += IntSz; EEPROM.get(tEA, Mlvt);  MSpcN-spacing srl. no.for Memory: EAd5~EAd6 ,record size, L,l 4 bytes
    // EAd4 changed to EAd5 for Wenner. Record size =2 (Bytes) MLvt = 'a'
    tMLvt = MLvt;  //tMlvt = Mlvt;  MLvt, Mlvt copied into 'temporary' values
    lcd1.clear();
    lcd1.setCursor(0, 0);
    lcd1.print("F3");

    lcd1.print("  Edit L,l ");
    lcd1.setCursor(13, 0);
    lcd1.print("MSp=");
    lcd1.print(MLlNo + 1);
    lcd1.setCursor(0, 1);
    lcd1.print("Ma= ");
    fltLv = (float)MLvt / 10.0;
    lcd1.print(fltLv, 1);
    //fltlv = (float)Mlvt / 10.0;   lcd1.print(fltlv, 1); // L,  lcd1.print(" Ml= " );

    lcd1.setCursor(11, 1);
    Kvt = WcalcK(fltLv);
    lcd1.print("K= ");
    lcd1.print(Kvt, 2);  // fltLv is 'a'
  }                      // bn

  lcd1.setCursor(0, 3);
  lcd1.print("change by");
  lcd1.print("         ");
  lcd1.setCursor(9, 3);  // erase ((9~15),1)show 1/0.5
  if (chbfr == 0) {
    lcd1.print(" 1 m.");
  } else lcd1.print(" 0.5 m.");
  //--------------------------------following 4 lines are for 'recall readings' mode-------------------
}
// .................................................end of entry_fnc_I...............................
//------------------------------------------------------------------------------------------------
void fnc_I2()  //key-next is pressed, operation: 'next L,l'
{
  if (Surv_meth == 1)  //Schlumberger
  {
    if (MLlNo < 37) MLlNo++;
    n15 = MLlNo;
    tEA = EAd4 + n15 * 4;
    EEPROM.get(tEA, MLvt);
    tEA += IntSz;
    EEPROM.get(tEA, Mlvt);  // MSpcN-spacing srl. no.for Memory
                            // EAd54 changed to EAd5  for Wenner. MLvt= 'a'
    tMLvt = MLvt;
    tMlvt = Mlvt;  // MLvt, Mlvt copied into 'temporary' values
    lcd1.clear();
    lcd1.setCursor(0, 0);
    lcd1.print("F3");
    ;
    lcd1.print("  Edit L,l ");
    lcd1.setCursor(13, 0);
    lcd1.print("MSp=");
    lcd1.print(MLlNo + 1);
    lcd1.setCursor(0, 1);
    lcd1.print("ML= ");
    fltLv = (float)tMLvt / 10.0;
    lcd1.print(fltLv, 1);  // next L,l
    lcd1.setCursor(13, 1);
    lcd1.print("Ml= ");
    fltlv = (float)tMlvt / 10.0;
    lcd1.print(fltlv, 1);  // L,l
    lcd1.setCursor(0, 2);
    Kvt = ScalcK(fltLv, fltlv);
    lcd1.print("K= ");
    lcd1.print(Kvt, 2);
  }
  if (Surv_meth == 2)  //  Wenner
  {
    if (MLlNo < 22) MLlNo++;
    n15 = MLlNo;
    tEA = EAd5 + n15 * 2;
    EEPROM.get(tEA, MLvt);  //tEA += IntSz; EEPROM.get(tEA, Mlvt); // MSpcN-spacing srl. no.for Memory
                            // EAd5 changedto EAd4 (9/march/2023  , MLvt= 'a'
    tMLvt = MLvt;           //tMlvt = Mlvt;  MLvt, Mlvt copied into 'temporary' values
    lcd1.clear();
    lcd1.setCursor(0, 0);
    lcd1.print("F3");
    ;
    lcd1.print("  Edit L,l ");
    lcd1.setCursor(13, 0);
    lcd1.print("MSp=");
    lcd1.print(MLlNo + 1);
    lcd1.setCursor(0, 1);
    lcd1.print("Ma= ");
    fltLv = (float)tMLvt / 10.0;
    lcd1.print(fltLv, 1);  // next L,l
                           //lcd1.print(" Ml= " ); fltlv = (float)tMlvt / 10.0;   lcd1.print(fltlv, 1); // L,l
    lcd1.setCursor(11, 1);
    Kvt = WcalcK(fltLv);
    lcd1.print("K= ");
    lcd1.print(Kvt, 2);  // K for Wenner
  }
  lcd1.setCursor(0, 3);
  lcd1.print("change by");
  lcd1.print("         ");
  lcd1.setCursor(9, 3);  // show 1/0.5
  if (chbfr == 0) {
    lcd1.print(" 1 m.");
  } else lcd1.print(" 0.5 m.");  //
  //----------------------------------------------------------------------------------------
}
// .................................................end of fnc_I2...............................
//------------------------------------------------------------------------------------------------
void fnc_I3()  // key-previous is pressed, operation: 'previous L,l'
{
  if (Surv_meth == 1)  //Schlumberger
  {
    if (MLlNo >= 1) MLlNo--;
    n15 = MLlNo;
    tEA = EAd4 + n15 * 4;
    EEPROM.get(tEA, MLvt);
    tEA += IntSz;
    EEPROM.get(tEA, Mlvt);  // MSpcN-spacing srl. no.for Memory
                            // EAd5 changedto EAd4 (9/march/2023
    tMLvt = MLvt;
    tMlvt = Mlvt;  // MLvt, Mlvt copied into 'temporary' values
    lcd1.clear();
    lcd1.setCursor(0, 0);
    lcd1.print("F3");
    ;
    lcd1.print("  Edit L,l ");
    lcd1.setCursor(13, 0);
    lcd1.print("MSp=");
    lcd1.print(MLlNo + 1);
    lcd1.setCursor(0, 1);
    lcd1.print("ML= ");
    fltLv = (float)tMLvt / 10.0;
    lcd1.print(fltLv, 1);  // previous L,l
    lcd1.setCursor(11, 1);
    lcd1.print(" Ml= ");
    fltlv = (float)tMlvt / 10.0;
    lcd1.print(fltlv, 1);
    lcd1.setCursor(0, 2);
    Kvt = ScalcK(fltLv, fltlv);
    lcd1.print("K= ");
    lcd1.print(Kvt, 2);  // K-Spacing factor at(0,2)
  }
  if (Surv_meth == 2)  // Wenner
  {
    if (MLlNo >= 1) MLlNo--;
    n15 = MLlNo;
    tEA = EAd5 + n15 * 2;
    EEPROM.get(tEA, MLvt);  //tEA += IntSz; EEPROM.get(tEA, Mlvt); // MSpcN-spacing srl. no.for Memory
                            // EAd4 changedto EAd5 ,for Wenner
    tMLvt = MLvt;           // tMlvt = Mlvt; // MLvt, Mlvt copied into 'temporary' values
    lcd1.clear();
    lcd1.setCursor(0, 0);
    lcd1.print("F3");
    ;
    lcd1.print("  Edit L,l ");
    lcd1.setCursor(13, 0);
    lcd1.print("MSp=");
    lcd1.print(MLlNo + 1);
    lcd1.setCursor(0, 1);
    lcd1.print("Ma= ");
    fltLv = (float)tMLvt / 10.0;
    lcd1.print(fltLv, 1);  // 'a' for previous Spacing no.
                           //lcd1.print(" Ml= " ); fltlv = (float)tMlvt / 10.0;   lcd1.print(fltlv, 1);
    lcd1.setCursor(11, 1);
    Kvt = WcalcK(fltLv);
    lcd1.print("K= ");
    lcd1.print(Kvt, 2);  // K-Spacing factor at(11.1)
  }

  lcd1.setCursor(0, 3);
  lcd1.print("change by");
  lcd1.print("         ");
  lcd1.setCursor(9, 3);  // // show 1/0.1
  if (chbfr == 0) {
    lcd1.print(" 1 m.");
  } else lcd1.print(" 0.5 m.");  //
  //---------------------------------------------------------------------------------
}
// .................................................end of fnc_I3...............................
//  ------fnc_I4,I5,I6,I7
//------------------------------------------------------------------------------------------------2
void fnc_I4()  //F3 mode key-1 is pressed, operation: 'L+=1m.'/0.5 m. note:  mLvt & Mlvt are 10* actual values
{
  //n15= MLlNo;tEA= EAd5+n15*4; EEPROM.get(tEA,MLvt); tEA+= IntSz; EEPROM.get(tEA,Mlvt);   // MSpcN-spacing srl. no.for Memory
  if (Surv_meth == 1)  // Schlumberger
  {
    if (MLlNo < 37) MLlNo++;
    n15 = MLlNo;
    tEA = EAd4 + n15 * 4;
    EEPROM.get(tEA, MLvt);
    tEA += IntSz;
    EEPROM.get(tEA, Mlvt);  // MSpcN-spacing srl. no.for Memory
                            // EAd5 changedto EAd4 (9/march/2023
    tMLvt = MLvt;
    tMlvt = Mlvt;  //
    lcd1.clear();
    lcd1.setCursor(0, 0);
    lcd1.print("F3");
    ;
    lcd1.print("  Edit L,l ");
    lcd1.setCursor(13, 0);
    lcd1.print("MSp=");
    lcd1.print(MLlNo + 1);
    lcd1.setCursor(0, 1);
    lcd1.print("ML= ");
    if (chbfr == 0) tMLvt += 10;  // L: +1/+0.5/
    else tMLvt += 5;
    fltLv = (float)tMLvt / 10.0;
    lcd1.print(fltLv, 1);  //
    lcd1.setCursor(11, 1);
    lcd1.print(" Ml= ");
    fltlv = (float)tMlvt / 10.0;
    lcd1.print(fltlv, 1);  // L,l
    lcd1.setCursor(0, 2);
    Kvt = ScalcK(fltLv, fltlv);
    lcd1.print("K= ");
    lcd1.print(Kvt, 2);
  }
  if (Surv_meth == 2)  // Wenner
  {
    if (MLlNo < 37) MLlNo++;
    n15 = MLlNo;
    tEA = EAd5 + n15 * 2;
    EEPROM.get(tEA, MLvt);  //tEA += IntSz; EEPROM.get(tEA, Mlvt); // MSpcN-spacing srl. no.for Memory
                            // EAd4 changedto EAd5 ,for Wenner
    tMLvt = MLvt;           //
    lcd1.clear();
    lcd1.setCursor(0, 0);
    lcd1.print("F3");
    ;
    lcd1.print("  Edit L,l ");
    lcd1.setCursor(13, 0);
    lcd1.print("MSp=");
    lcd1.print(MLlNo + 1);
    lcd1.setCursor(0, 1);
    lcd1.print("ML= ");
    if (chbfr == 0) tMLvt += 10;
    else tMLvt += 5;
    fltLv = (float)tMLvt / 10.0;
    lcd1.print(fltLv, 1);  // // a: +1/+0.5
                           //lcd1.print(" Ml= " ); fltlv = (float)tMlvt / 10.0;   lcd1.print(fltlv, 1); // L,l
    lcd1.setCursor(11, 1);
    Kvt = WcalcK(fltLv);
    lcd1.print("K= ");
    lcd1.print(Kvt, 2);
  }
  lcd1.setCursor(0, 3);
  lcd1.print("change by");
  lcd1.print("         ");
  lcd1.setCursor(11, 3);  //show 1/0.5
  if (chbfr == 0) {
    lcd1.print(" 1 m.");
  } else lcd1.print(" 0.5 m.");  //
  //----------------------------------------------------------------------------------------
}
// .................................................end of fnc_I4...............................
//------------------------------------------------------------------------------------------------
void fnc_I5()  // key-4 is pressed, operation: 'L-=1m.'/0.5
{
  //n15= MLlNo;tEA= EAd5+n15*4; EEPROM.get(tEA,MLvt); tEA+= IntSz; EEPROM.get(tEA,Mlvt);   // MSpcN-spacing srl. no.for Memory
  if (Surv_meth == 1)  // Schlumberger
  {
    if (MLlNo >= 1) MLlNo--;
    n15 = MLlNo;
    tEA = EAd4 + n15 * 4;
    EEPROM.get(tEA, MLvt);
    tEA += IntSz;
    EEPROM.get(tEA, Mlvt);  // MSpcN-spacing srl. no.for Memory
                            // EAd5 changedto EAd4 (9/march/2023
    tMLvt = MLvt;
    tMlvt = Mlvt;
    lcd1.clear();
    lcd1.setCursor(0, 0);
    lcd1.print("F3");
    ;
    lcd1.print("  Edit L,l ");
    lcd1.setCursor(13, 0);
    lcd1.print("MSp=");
    lcd1.print(MLlNo + 1);
    lcd1.setCursor(0, 1);
    lcd1.print("ML= ");
    if (chbfr == 0) tMLvt -= 10;
    else tMLvt -= 5;
    fltLv = (float)tMLvt / 10.0;
    lcd1.print(fltLv, 1);  // L: -1/-0.5
    lcd1.setCursor(11, 1);
    lcd1.print(" Ml= ");
    fltlv = (float)tMlvt / 10.0;
    lcd1.print(fltlv, 1);
    lcd1.setCursor(0, 2);
    Kvt = ScalcK(fltLv, fltlv);
    lcd1.print("K= ");
    lcd1.print(Kvt, 2);  // K-Spacing factor at(0,2)
  }
  if (Surv_meth == 2)  // Wenner
  {
    if (MLlNo >= 1) MLlNo--;
    n15 = MLlNo;
    tEA = EAd4 + n15 * 4;
    EEPROM.get(tEA, MLvt);
    tEA += IntSz;
    EEPROM.get(tEA, Mlvt);  // MSpcN-spacing srl. no.for Memory
                            // EAd5 changedto EAd4 (9/march/2023
    tMLvt = MLvt;
    tMlvt = Mlvt;
    lcd1.clear();
    lcd1.setCursor(0, 0);
    lcd1.print("F3");
    ;
    lcd1.print("  Edit L,l ");
    lcd1.setCursor(13, 0);
    lcd1.print("MSp=");
    lcd1.print(MLlNo + 1);
    lcd1.setCursor(0, 1);
    lcd1.print("Ma= ");
    if (chbfr == 0) tMLvt -= 10;
    else tMLvt -= 5;
    fltLv = (float)tMLvt / 10.0;
    lcd1.print(fltLv, 1);  // a: -1/-0.5
                           //lcd1.print(" Ml= " ); fltlv = (float)tMlvt / 10.0;   lcd1.print(fltlv, 1);
    lcd1.setCursor(11, 1);
    Kvt = WcalcK(fltLv);
    lcd1.print("K= ");
    lcd1.print(Kvt, 2);  // K-Spacing factor at(11,1)
  }
  lcd1.setCursor(0, 3);
  lcd1.print("change by");
  lcd1.print("         ");
  lcd1.setCursor(9, 3);  // show 1/0.5
  if (chbfr == 0) {
    lcd1.print(" 1 m.");
  } else lcd1.print(" 0.5 m.");
  //---------------------------------------------------------------------------------
}
// .................................................end of fnc_I5...............................
//------------------------------------------------------------------------------------------------
void fnc_I6()  //key-2 is pressed, operation: 'l+=1m.'/0.5

{
  if (Surv_meth == 1)  // Schlumberger only, key-2 will be ignored if 'Wenner' mode
                       //n15= MLlNo;tEA= EAd5+n15*4; EEPROM.get(tEA,MLvt); tEA+= IntSz; EEPROM.get(tEA,Mlvt);   // MSpcN-spacing srl. no.for Memory
  {
    if (MLlNo < 37) MLlNo++;
    tMLvt = MLvt;
    tMlvt = Mlvt;
    lcd1.clear();
    lcd1.setCursor(0, 0);
    lcd1.print("F3");
    ;
    lcd1.print("  Edit L,l ");
    lcd1.setCursor(13, 0);
    lcd1.print("MSp=");
    lcd1.print(MLlNo + 1);
    lcd1.setCursor(0, 1);
    lcd1.print("ML= ");
    fltLv = (float)tMLvt / 10.0;
    lcd1.print(fltLv, 1);  // next L,l
    lcd1.setCursor(11, 1);
    lcd1.print(" Ml= ");
    if (chbfr == 0) tMlvt += 10;
    else tMlvt += 5;
    fltlv = (float)tMlvt / 10.0;
    lcd1.print(fltlv, 1);  // l: +1/+0.5
    lcd1.setCursor(0, 2);
    Kvt = ScalcK(fltLv, fltlv);
    lcd1.print("K= ");
    lcd1.print(Kvt, 2);
    lcd1.setCursor(0, 3);
    lcd1.print("change by");
    lcd1.print("         ");
    lcd1.setCursor(12, 3);  // show 1/0.5
    if (chbfr == 0) {
      lcd1.print(" 1 m.");
    } else lcd1.print(" 0.5 m.");
  }
  //----------------------------------------------------------------------------------------
}
// .................................................end of fnc_I6...............................
//------------------------------------------------------------------------------------------------
void fnc_I7()  // key-5 is pressed, operation: 'l-=1m.'/0.5
{
  if (Surv_meth == 1)  // Schlumberger only, key-5 will be ignored if 'Wenner' mode
  {                    //n15= MLlNo;tEA= EAd5+n15*4; EEPROM.get(tEA,MLvt); tEA+= IntSz; EEPROM.get(tEA,Mlvt);   // MSpcN-spacing srl. no.for Memory
    if (MLlNo >= 1) MLlNo--;
    tMLvt = MLvt;
    tMlvt = Mlvt;
    lcd1.clear();
    lcd1.setCursor(0, 0);
    lcd1.print("F3");
    ;
    lcd1.print("  Edit L,l ");
    lcd1.setCursor(13, 0);
    lcd1.print("MSp=");
    lcd1.print(MLlNo + 1);
    lcd1.setCursor(0, 1);
    lcd1.print("ML= ");
    fltLv = (float)tMLvt / 10.0;
    lcd1.print(fltLv, 1);  // previous L,l
    lcd1.setCursor(11, 1);
    lcd1.print(" Ml= ");
    if (chbfr == 0) tMlvt -= 10;
    else tMlvt -= 5;
    fltlv = (float)tMlvt / 10.0;
    lcd1.print(fltlv, 1);  //l: -1/-0.5

    lcd1.setCursor(0, 2);
    Kvt = ScalcK(fltLv, fltlv);
    lcd1.print("K= ");
    lcd1.print(Kvt, 2);  // K-Spacing factor at(0,2)
    lcd1.setCursor(0, 3);
    lcd1.print("change by");
    lcd1.print("         ");
    lcd1.setCursor(9, 3);  // show 1/0.5
    if (chbfr == 0) {
      lcd1.print(" 1 m.");
    } else lcd1.print(" 0.5 m.");
  }

  //---------------------------------------------------------------------------------
}
// .................................................end of fnc_I7...............................
//------------------------------------------------------------------------------------------------
void fnc_I8()  // key-'Save' is pressed, operation: 'E2prom<--L,l'
{
  if (Surv_meth == 1)  // Schlumberger
  {
    tMLvt = MLvt;
    tMlvt = Mlvt;
    n15 = MLlNo;
    tEA = EAd4 + n15 * 4;
    EEPROM.put(tEA, tMLvt);
    tEA += IntSz;
    EEPROM.put(tEA, tMlvt);  // E2prom <-- L,l MSpcN-spacing srl. no.for Memory

    //lcd1.clear();
    lcd1.setCursor(0, 0);
    lcd1.print("                   ");  // Erase entire line-0 char.posn. 0~19
    lcd1.setCursor(0, 0);
    lcd1.print("F3");

    lcd1.print("  Edit L,l ");
    lcd1.setCursor(13, 0);
    lcd1.print("MSp=");
    lcd1.print(MLlNo + 1);
    lcd1.setCursor(0, 1);
    lcd1.print("                   ");  //Erase entiere line-1 char. posn.0~19
    lcd1.setCursor(0, 1);
    lcd1.print("ML= ");
    fltLv = (float)tMLvt / 10.0;
    lcd1.print(fltLv, 1);  // previous L,l
    lcd1.setCursor(11, 1);
    lcd1.print(" Ml= ");
    fltlv = (float)tMlvt / 10.0;

    lcd1.setCursor(0, 1);
    lcd1.print(fltlv, 1);
    lcd1.setCursor(0, 2);
    lcd1.print("                   ");  //Erase entire line-2 char. posn.0~19
    Kvt = ScalcK(fltLv, fltlv);
    lcd1.print("K= ");
    lcd1.print(Kvt, 2);  // K-Spacing factor at(0,2)
  }
  if (Surv_meth == 2)  // Wenner
  {
    tMLvt = MLvt;
    tMlvt = Mlvt;
    n15 = MLlNo;
    tEA = EAd5 + n15 * 2;
    EEPROM.put(tEA, tMLvt);  //tEA += IntSz; EEPROM.put(tEA, tMlvt); // E2prom<-- 'a',MSpcN-spacing srl. no.for Memory


    lcd1.setCursor(0, 0);
    lcd1.print("                   ");  //Erase entiere line-0 char. posn.0~19
    lcd1.print("F3");

    lcd1.print("  Edit L,l ");
    lcd1.setCursor(13, 0);
    lcd1.print("MSp=");
    lcd1.print(MLlNo + 1);
    lcd1.setCursor(0, 1);
    lcd1.print("                   ");  //Erase entiere line-1 char. posn.0~19
    lcd1.print("Ma= ");
    fltLv = (float)tMLvt / 10.0;
    lcd1.print(fltLv, 1);  // previous L,l
                           //lcd1.setCursor(11, 1); lcd1.print(" Ml= " ); fltlv = (float)tMlvt / 10.0;   lcd1.print(fltlv, 1);
    lcd1.setCursor(11, 1);
    Kvt = WcalcK(fltLv);
    lcd1.print("K= ");
    lcd1.print(Kvt, 2);  // K-Spacing factor at(0,2)
  }

  lcd1.setCursor(0, 3);
  lcd1.print("--saved-- ");
  //---------------------------------------------------------------------------------
  //  */
}
// .................................................end of fnc_I8...............................
//-----------------------entry_fnc_M--------------------------------------------------------
//---------------------------------------------------------------------------------------------
void entry_fnc_M()  // prove that  values of L,l were stored correctly

{
  if (Surv_meth == 1)  // Schlumberger
  {
    tMLvt = MLvt;
    tMlvt = Mlvt;
    n15 = MLlNo;
    tEA = EAd4 + n15 * 4;
    EEPROM.get(tEA, tMLvt);
    tEA += IntSz;
    EEPROM.get(tEA, tMlvt);  // E2prom <-- L,l MSpcN-spacing srl. no.for Memory

    //lcd1.clear();                   // prefix,----'M'means valuesstored in EEPROMmemory
    lcd1.setCursor(0, 0);
    lcd1.print("                   ");  // Erase entire line-0 char.posn. 0~19
    lcd1.setCursor(0, 0);
    lcd1.print("F3");

    lcd1.print("  Edit L,l ");
    lcd1.setCursor(13, 0);
    lcd1.print("MSp=");
    lcd1.print(MLlNo + 1);
    lcd1.setCursor(0, 1);
    lcd1.print("                   ");  //Erase entiere line-1 char. posn.0~19
    lcd1.setCursor(0, 1);
    lcd1.print("ML= ");
    fltLv = (float)tMLvt / 10.0;
    lcd1.print(fltLv, 1);  // previous L,l
    lcd1.setCursor(11, 1);
    lcd1.print(" Ml= ");
    fltlv = (float)tMlvt / 10.0;

    lcd1.setCursor(0, 1);
    lcd1.print(fltlv, 1);
    lcd1.setCursor(0, 2);
    lcd1.print("                   ");  //Erase entire line-2 char. posn.0~19
    Kvt = ScalcK(fltLv, fltlv);
    lcd1.print("MK= ");
    lcd1.print(Kvt, 2);  // K-Spacing factor at(0,2)
  }
  if (Surv_meth == 2)  // Wenner
  {
    tMLvt = MLvt;
    tMlvt = Mlvt;
    n15 = MLlNo;
    tEA = EAd5 + n15 * 2;
    EEPROM.put(tEA, tMLvt);  //tEA += IntSz; EEPROM.put(tEA, tMlvt); // E2prom<-- 'a',MSpcN-spacing srl. no.for Memory


    lcd1.setCursor(0, 0);
    lcd1.print("                   ");  //Erase entiere line-0 char. posn.0~19
    lcd1.print("F3");

    lcd1.print("  Edit L,l ");
    lcd1.setCursor(13, 0);
    lcd1.print("MSp=");
    lcd1.print(MLlNo + 1);
    lcd1.setCursor(0, 1);
    lcd1.print("                   ");  //Erase entiere line-1 char. posn.0~19
    lcd1.print("Ma= ");
    fltLv = (float)tMLvt / 10.0;
    lcd1.print(fltLv, 1);  // previous L,l
                           //lcd1.setCursor(11, 1); lcd1.print(" Ml= " ); fltlv = (float)tMlvt / 10.0;   lcd1.print(fltlv, 1);
    lcd1.setCursor(11, 1);
    Kvt = WcalcK(fltLv);
    lcd1.print("K= ");
    lcd1.print(Kvt, 2);  // K-Spacing factor at(0,2)
  }

  lcd1.setCursor(0, 3);
  lcd1.print("--was saved-- ");
  //---------------------------------------------------------------------------------
  //  */
}
// .................................................end of entry_fnc_Mfnc_I8...............................

//.............................end of entry_fnc_M...............................

//----------------------------------------------------------------------------------------------

void fnc_I9()  // key-'.' is pressed, operation: 'change by 0.5 m.'
{
  chbfr = 1;  //change L,l by 0.1 m in fnc_I4/I5/I6/I7

  lcd1.setCursor(0, 3);
  lcd1.print("                   ");
  lcd1.setCursor(0, 3);
  lcd1.print("change by");
  lcd1.setCursor(11, 3);  // erase ch. 9~17, line 3
  if (chbfr == 0) lcd1.print(" 1 m.");
  else lcd1.print(" 0.5 m.");  // show 1/0.5
  //---------------------------------------------------------------------------------
}
// .................................................end of fnc_I9...............................
//------------------------------------------------------------------------------------------------
void fnc_I10()  // key-'0' is pressed, operation: 'change by 1 m.'
{
  chbfr = 0;  //change L,l by 1 m in fnc_I4/I5/I6/I7

  lcd1.setCursor(0, 3);
  lcd1.print("                   ");
  lcd1.setCursor(0, 3);
  lcd1.print("change by");
  lcd1.setCursor(11, 3);  // erase ch. 9~17, line 3
  if (chbfr == 0) lcd1.print(" 1 m.");
  else lcd1.print(" 0.5 m.");  // show 1/0.5  // show 1/0.5
  //---------------------------------------------------------------------------------
}

//...........................end of 'from 'Dubai1_AutoD .........................................
//--------------------------------------------fnc_I4,I5,I6,I7-----Above----------------------------
//-------------------------------------------entrry_fnc_G,fnc_G2,fnc_G3------------------------------
void entry_fnc_G()

{
  // entry function for 'GPS' data, show 'GPS' continuously
  lcd1.clear();
  lcd1.setCursor(0, 0);
  lcd1.print("F1");
  lcd1.setCursor(3, 0);
  lcd1.print("Alt=");  //'alpha',1
  vr4 = (float)LAltit / 100;
  lcd1.print(vr4);
  lcd1.print(" m.");  //Laltit is in cm.
                      //--------------Latt.,Long.-----------------
  lcd1.setCursor(0, 1);
  lcd1.print("Latt:");
  lcd1.print(ldg0);
  lcd1.print(" ");
  lcd1.print(lmin0);
  lcd1.print(" ");
  lcd1.print(lsec0, 2);  //Latt:deg,min,sec
  lcd1.setCursor(0, 2);
  lcd1.print("Long:");
  lcd1.print(ldg1);
  lcd1.print(" ");
  lcd1.print(lmin1);
  lcd1.print(" ");
  lcd1.print(lsec1, 2);  //Long:deg,min,sec
                         //lcd1.setCursor(0,1); lcd1.print("Latt:"); lcd1.print(ldg0); lcd1.print(" "); lcd1.print(lmin0); lcd1.print(" ");lcd1.print(lsec0,2);  // print float latitude
  //lcd1.setCursor(0,2); lcd1.print("Long:"); lcd1.print(longitude,2);  //  print float longitude
  //---------date,time------------------------------------
  lcd1.setCursor(0, 3);
  lcd1.print(Dte);
  lcd1.print("/");
  lcd1.print(Mn);
  lcd1.print("/");
  lcd1.print(Yr);
  lcd1.print(" ");  //date,month,year
  lcd1.print(Hr);
  lcd1.print(":");
  lcd1.print(mint);
  lcd1.print(":");
  lcd1.print(Scnd);
  lcd1.print(" ");  //Hours,minute,Seconds
}
//.......................................end of entry_fnc_G...........................................
void fnc_G2() {
  // show  'GPS' for say, 10 Seconds
  lcd1.setCursor(0, 0);
  lcd1.print("F1");
  lcd1.setCursor(4, 0);
  lcd1.print("GPS,10 Sec");  //'alpha',2
}
void fnc_G3() {
  // show a symbol to indicate that latest 'GPS' data was captured
}
void fnc_G4() {
  // show Altitude in 'GPS' data
}
//.....................................end of 3 'G' functions.......................................
//---------------------------------------------- 'H'('Sigma',1 Normal Survey mode ----------------------------
void entry_fnc_H() {  // Survey mode
                      // entry function for 'Survey' mode
  tn1 = 0;
  tn2 = 0;
  t_transf = 0;  // this means no. of chars. received at Serial2=0. This may help
  lcd1.clear();
  lcd1.setCursor(0, 0);
  lcd1.print("F2");  // F2'Sigma',1
  lcd1.setCursor(3, 0);
  lcd1.print("Rd ");
  lcd1.print(LRdSr2 + 1);
  if (Surv_meth == 1) lcd1.print(" ");
  if (Surv_meth == 2) lcd1.print(" ");
  lcd1.print(" Sp=");
  if (freezeSP == 0) lcd1.print(LSpcN2 + 1);
  else lcd1.print(LSpcN2);
  if ((LSpcN2 + 1) <= 9) lcd1.print(" ");  // line-0, reading no.,Spacing no. .
  lcd1.setCursor(12, 0);
  lcd1.print(" Srv ");
  lcd1.print(Srv_No);  //
  lcd1.setCursor(10, 2);
  if (Surv_meth == 1) lcd1.print(" Schlumb ");
  if (Surv_meth == 2) lcd1.print(" Wenner ");
  if (Surv_meth == 3) lcd1.print("Dip-Dipo ");  // show Schlum/wenner/Dipo-Dipo at line-2, column-10
  lcd1.print("Sp2=");                           // why  SP & SP2 ?
  if (freezeSP == 0) Show_LlK2(LSpcN2);
  else Show_LlK2(LSpcN2 - 1);
  lcd1.setCursor(0, 2);
  lcd1.print("K= ?");  //
                       // Sp++  blocked by key 2 only & enabled by key 3 only
  lcd1.setCursor(1, 3);
  lcd1.print("-press Measur(9)");  // this message "-press Measure-" wil get erased when 'Batt' voltage is received

  //EEPROM.get(tEA, Lint1);  tEA += 2; EEPROM.get(tEA, lint1);
  //Ldig1=L%10; Lint3=L/10; ldig1=l%10; lint3=l/10;  fltL=(float)L/10; fltl=(float)l/10; Kv=ScalcK(fltL,fltl);
}
//...............................................
//-------------------------------fnc_H2------------------------------------------------------
void fnc_H2() {  // within 'H', key-2 pressed 'Sigma',2 , Normal Survey mode
  if (LSpcN2 >= 1) freezeSP = 1;
  lcd1.setCursor(0, 0);
  lcd1.print("F2");  //  key-2 pressed
  lcd1.setCursor(0, 3);
  lcd1.print("                   ");
  lcd1.setCursor(0, 3);
  if (freezeSP == 1) lcd1.print("SP++, press 6");  //
                                                   //entry_fnc_H();    // done , so that frezeein/releasing action should be done for present reading

}  //.....................................end of fnc_H2......................
//-------------------------------fnc_H3------------------------------------------------------
void fnc_H3() {  // within 'H' 'Sigma',key-3 pressed , Normal Survey mode
  freezeSP = 0;
  lcd1.setCursor(0, 0);
  lcd1.print("F2");  //  key-3 pressed// 'Sigma' 3
  lcd1.setCursor(0, 3);
  lcd1.print("                   ");
  lcd1.setCursor(0, 3);
  lcd1.print("Sp++, press 6  ");
  //entry_fnc_H();    // done , so that frezeein/releasing action should be done for present reading
}

//entry_fnc_H();    // done , so that frezeein/releasing action should be done for present reading

//.....................................end of fnc_H3......................
//-------------------------------fnc_H4------------------------------------------------------
void fnc_H4() {  // within 'H' ,key-4 pressed,'Sigma',4 , Normal Survey mode,show SpacingNo fixed/not fixed status
  lcd1.setCursor(0, 0);
  lcd1.print("F2");  //  key-4 pressed//
  lcd1.setCursor(0, 3);
  lcd1.print("                   ");
  lcd1.setCursor(0, 3);
  if (freezeSP == 0) lcd1.print("Sp++, press 6  ");
  else lcd1.print("Sp fixed,press 6");
  //entry_fnc_H();  moved to timr7
}
//.....................................end of fnc_H4......................

//-------------------------------fnc_H6-----------------------------------------------------
void fnc_H6() {  // within 'H' ,key-6 pressed,'Sigma',4 , Normal Survey mode,

  entry_fnc_H();  //like pressing 'F2'
}
//.....................................end of fnc_H6......................
//

//-------------------------------fnc_H9------------------------------------------------------
void fnc_H9() {  // within 'H' ,key-9  pressed,'Sigma',5 , Normal Survey mode,SpacingNo

  digitalWrite(27, LOW);
  Serial.println("key 9 pressed");

  timr7 = 0;
  timr7_flag = 1;  //  D27<--0,turns on Auto_D
}
//.....................................end of fnc_H5......................
//-------------------------------fnc_H_Prv---for 'previous' key---------------------------------------------------
void fnc_H_Prv() {     // within 'H' 'Sigma',6 , Normal Survey mode, SpacingNo--,show new LlK
  if (Surv_meth == 1)  // Schlumberger
  {
    lcd1.setCursor(0, 0);
    lcd1.print("F2");  //  key-Previous pressed//
    n14 = LSpcN2;
    if (n14 >= 1) {
      n14--;
      LSpcN2--;
    }
    LlpSz = 2 * 2;
    tEA = EAd4 + LlpSz * n14;  //  (changed to EAd4 9/march/2023)

    EEPROM.get(tEA, Lint1);
    tEA += 2;
    EEPROM.get(tEA, lint1);
    Show_LlK3(n14, Lint1, lint1);  //Show_Eprom3( Lint1, lint1);
  }
  if (Surv_meth == 2)  // Wenner
  {
    lcd1.setCursor(0, 0);
    lcd1.print("F2");  //  key-Previous pressed//
    n14 = LSpcN2;
    if (n14 >= 1) {
      n14--;
      LSpcN2--;
    }
    LlpSz = 1 * IntSz;
    tEA = EAd5 + LlpSz * n14;  //  (changed to EAd5 14/march/2023)

    EEPROM.get(tEA, Lint1);
    Show_LlK3(n14, Lint1, lint1);  //Show_Eprom3(n14 ,Lint1, lint1)      tEA += 2; EEPROM.get(tEA, lint1);
                                   // in Show_LlK3, lint3 is ignored
  }
  if (Surv_meth == 3)  // Dipole-Dipole
  {
    lcd1.setCursor(0, 0);
    lcd1.print("F2");  //  key-Next pressed//
    n14 = LSpcN2;
    if (n14 >= 1) {
      n14--;
      LSpcN2--;
    }
    LlpSz = 2 * 2;
    tEA = EAd9 + LlpSz * n14;  // (changed to EAd4 9/march/2023) there are total of 38(0~37) records of L,l
    //
    EEPROM.get(tEA, Lint1);
    tEA += 2;
    EEPROM.get(tEA, lint1);
    Show_LlK3(n14, Lint1, lint1);
    lcd1.setCursor(0, 2);
    lcd1.print("K=");  //  'a'=Lint1, 'n' = lint1
  }
}
//.....................................end of fnc_H_Prv......................
//-------------------------------fnc_H_Nxt----for 'Next' key--------------------------------------------------
void fnc_H_Nxt() {  // within 'H' 'Sigma',7 , Normal Survey mode, SpacingNo++,show new LlK
  if (Surv_meth == 1) {
    tMLvt = MLvt;
    tMlvt = Mlvt;
    lcd1.setCursor(0, 0);
    lcd1.print("F2");  //  key-Next pressed//
    n14 = LSpcN2;
    if (n14 < 37) {
      n14++;
      LSpcN2++;
    }
    LlpSz = 2 * 2;
    tEA = EAd4 + LlpSz * n14;  // (changed to EAd4 9/march/2023) there are total of 38(0~37) records of L,l
    //
    EEPROM.get(tEA, Lint1);
    tEA += 2;
    EEPROM.get(tEA, lint1);
    Show_LlK3(n14, Lint1, lint1);  //
    lcd1.setCursor(0, 2);
    fltLv = (float)tMLvt / 10.0;
    fltlv = (float)tMlvt / 10.0;
    Kvt = ScalcK(fltLv, fltlv);
    lcd1.print("K= ");
    lcd1.print(Kvt, 2);  // K-Spacing factor at(0,2)
  }
  if (Surv_meth == 2)  // Wenner
  {
    lcd1.setCursor(0, 0);
    lcd1.print("F2");  //  key-'Next' pressed//
    n14 = LSpcN2;
    if (n14 < 22) {
      n14++;
      LSpcN2++;
    }
    LlpSz = 1 * IntSz;
    tEA = EAd5 + LlpSz * n14;  //  (changed to EAd5, for Wenner, 14/march/2023)

    EEPROM.get(tEA, Lint1);
    Show_LlK3(n14, Lint1, lint1);  //Show_Eprom3(n14 ,Lint1, lint1)      tEA += 2; EEPROM.get(tEA, lint1);
    lcd1.setCursor(0, 2);
    Kvt = WcalcK(fltLv);  // calculate K for Wenner
    lcd1.print("K= ");
    lcd1.print(Kvt, 2);  // K-Spacing factor at(0,2)                              // in Show_LlK3, lint3 is ignored
  }
  if (Surv_meth == 3)  // Dipole-Dipole
  {
    lcd1.setCursor(0, 0);
    lcd1.print("F2");  //  key-Next pressed//
    n14 = LSpcN2;
    if (n14 < 24) {
      n14++;
      LSpcN2++;
    }
    LlpSz = 2 * 2;
    tEA = EAd9 + LlpSz * n14;  // (changed to EAd4 9/march/2023) there are total of 38(0~37) records of L,l
    //
    EEPROM.get(tEA, Lint1);
    tEA += 2;
    EEPROM.get(tEA, lint1);
    Show_LlK3(n14, Lint1, lint1);
    lcd1.setCursor(0, 2);
    lcd1.print("K=");  //  'a'=Lint1, 'n' = lint1
    Kvt = DipcalcK(Lint1, lint1);
    lcd1.print(Kvt, 2);  // K --for dipole-DipoLe
  }

}  //
   //
   //

//.....................................end of fnc_H_Nxt......................

//-----------------------------mode 'K', 'theta', show & modify l & l
//------------------------------------------------------------------------------------------------
//
void entry_fnc_K()  // just entering into 'K' Undefined. When using simulated data, use 'NRec' in place of 'StRecrds' NRec
{
  lcd1.clear();
  lcd1.setCursor(0, 0);
  lcd1.print("F5 ");
  lcd1.print("Rdng Recall mode");  //
  if (StRecrds == 0) {
    lcd1.setCursor(0, 1);
    lcd1.print("-No stored readings-");
  } else {
    if ((Surv_meth == 1) || (Surv_meth == 2)) {
      n8 = StRecrds - 1;
      tEA = EAd7 + (8 * n8);
      EEPROM.get(tEA, StRd1);
      tEA += 2;
      EEPROM.get(tEA, StL1);
      tEA += 2;
      EEPROM.get(tEA, StRho1);  // get 3 variables from E2prom
      // in the above line EAd6 was changed to EAd7 on 4/march/2023
      lcd1.setCursor(0, 1);
      lcd1.print("SrlN= ");
      lcd1.print(StRd1 + 1);
      lcd1.setCursor(10, 1);
      if (Surv_meth == 1) lcd1.print("L=");
      if (Surv_meth == 2) lcd1.print("a=");

      {
        vr2 = (float)StL1 / 10.0;
        lcd1.print(vr2, 1);
      }  //Reading no. ,L.
         //  Strd1 is retrieved from EEPRM. When it is 0, we should it as 1, meaning 1st reading
         // StRd1,StL1, StRho1 <-- from E2prom (tEa =EAd6 + (n8*8)
    }
    if (Surv_meth == 3) {
      n8 = StRecrds - 1;
      tEA = EAd7 + (10 * n8);
      EEPROM.get(tEA, StRd1);
      tEA += 2;
      EEPROM.get(tEA, StL1);
      tEA += 2;
      EEPROM.get(tEA, StL2);
      tEA += 2;
      EEPROM.get(tEA, StRho1);  // get 4 (10 bytes) variables from E2prom
      lcd1.setCursor(0, 1);
      lcd1.print("SrlN= ");
      lcd1.print(StRd1 + 1);
      lcd1.setCursor(10, 1);
      lcd1.print("a=");
      lcd1.print(StL1);
      lcd1.print(",n=");
      lcd1.print(StL2);
    }

    lcd1.setCursor(0, 2);
    lcd1.write(0xE6);
    lcd1.print("=");
    lcd1.print(StRho1, 1);  // E6h - rho

    //  lcd1.setCursor(0,3);  lcd1.print("entered-'K'mode"); // commented 23/may/2023. line-2 :" just entered 'Survey Status' "
  }
}
//...................end of entry_fnc_K............................................
//.--------------
//-------------------------below:-----'Previos key pressed'_________________________.
void fnc_K2() {

  // 'Previous' key is pressed
  lcd1.clear();
  lcd1.setCursor(0, 0);
  lcd1.print("F5 ");
  lcd1.print("Rdng Recall mode");  //
  if (StRecrds == 0) {
    lcd1.setCursor(0, 1);
    lcd1.print("-No stored readings-");
  } else {
    if ((Surv_meth == 1) || (Surv_meth == 2)) {
      if (n8 >= 1) n8--;
      tEA = EAd7 + (8 * n8);
      EEPROM.get(tEA, StRd1);
      tEA += 2;
      EEPROM.get(tEA, StL1);
      tEA += 2;
      EEPROM.get(tEA, StRho1);  // get 3 variables from E2prom
                                // in the above line EAd6 was changed to EAd7 on 4/march/2023
      lcd1.setCursor(0, 1);
      lcd1.print("SrlN= ");
      lcd1.print(StRd1 + 1);
      lcd1.setCursor(10, 1);
      if (Surv_meth == 1) lcd1.print("L=");
      if (Surv_meth == 2) lcd1.print("a=");

      {
        vr2 = (float)StL1 / 10.0;
        lcd1.print(vr2, 1);
      }  //Reading no. ,L.
         //  Strd1 is retrieved from EEPRM. When it is 0, we should it as 1, meaning 1st reading
         // StRd1,StL1, StRho1 <-- from E2prom (tEa =EAd6 + (n8*8)
    }
    if (Surv_meth == 3) {
      if (n8 >= 1) n8--;
      tEA = EAd7 + (10 * n8);
      EEPROM.get(tEA, StRd1);
      tEA += 2;
      EEPROM.get(tEA, StL1);
      tEA += 2;
      EEPROM.get(tEA, StL2);
      tEA += 2;
      EEPROM.get(tEA, StRho1);  // get 4 (10 bytes) variables from E2prom
      lcd1.setCursor(0, 1);
      lcd1.print("SrlN= ");
      lcd1.print(StRd1 + 1);
      lcd1.setCursor(10, 1);
      lcd1.print("a=");
      lcd1.print(StL1);
      lcd1.print(",n=");
      lcd1.print(StL2);
    }

    lcd1.setCursor(0, 2);
    lcd1.write(0xE6);
    lcd1.print("=");
    lcd1.print(StRho1, 1);  // E6h - rho

    // lcd1.setCursor(0,3);  lcd1.print("entered-'K'mode"); // linon 23//may/2023e-2 :" just entered 'Survey Status' "
  }
  //--------------------------------------------------------------------------------------------------------------block1-----
}
//..........................end of .fnc_K2..................................................
//.---------------------below:--.''Next'' key pressed'_________________________.
void fnc_K3() {
  //---------------------------------------------new fnc_K3-----
  // next key is pressed
  lcd1.clear();
  lcd1.setCursor(0, 0);
  lcd1.print("F5 ");
  lcd1.print("Rdng Recall mode");  //
  if (StRecrds == 0) {
    lcd1.setCursor(0, 1);
    lcd1.print("-No stored readings-");
  } else {
    if ((Surv_meth == 1) || (Surv_meth == 2)) {
      if (n8 < StRecrds - 1) n8++;
      tEA = EAd7 + (8 * n8);
      EEPROM.get(tEA, StRd1);
      tEA += 2;
      EEPROM.get(tEA, StL1);
      tEA += 2;
      EEPROM.get(tEA, StRho1);  // get 3 variables from E2prom
                                // in the above line EAd6 was changed to EAd7 on 4/march/2023
      lcd1.setCursor(0, 1);
      lcd1.print("SrlN= ");
      lcd1.print(StRd1 + 1);
      lcd1.setCursor(10, 1);
      if (Surv_meth == 1) lcd1.print("L=");
      if (Surv_meth == 2) lcd1.print("a=");

      {
        vr2 = (float)StL1 / 10.0;
        lcd1.print(vr2, 1);
      }  //Reading no. ,L.
         //  Strd1 is retrieved from EEPRM. When it is 0, we should it as 1, meaning 1st reading
         // StRd1,StL1, StRho1 <-- from E2prom (tEa =EAd6 + (n8*8)
    }
    if (Surv_meth == 3) {
      if (n8 < StRecrds - 1) n8++;
      tEA = EAd7 + (10 * n8);
      EEPROM.get(tEA, StRd1);
      tEA += 2;
      EEPROM.get(tEA, StL1);
      tEA += 2;
      EEPROM.get(tEA, StL2);
      tEA += 2;
      EEPROM.get(tEA, StRho1);  // get 4 (10 bytes) variables from E2prom
      lcd1.setCursor(0, 1);
      lcd1.print("SrlN= ");
      lcd1.print(StRd1 + 1);
      lcd1.setCursor(10, 1);
      lcd1.print("a=");
      lcd1.print(StL1);
      lcd1.print(",n=");
      lcd1.print(StL2);
    }

    lcd1.setCursor(0, 2);
    lcd1.write(0xE6);
    lcd1.print("=");
    lcd1.print(StRho1, 1);  // E6h - rho

    // lcd1.setCursor(0,3);  lcd1.print("entered-'K'mode"); // on 23/may/2023line-2 :" just entered 'Survey Status' "
  }
}  //---
   //............................................end of new fnc_K3.........

//......................................
//...............................end of .fnc_K3......'K' mode ,'...................................
//  Initialization screen for 'Q', Test mode
//- - - - - - - - - - - - - - - - - - - - - -- - - - - - -
void entry_fnc_Q() {
  unsigned int i1;  //show Resistance  only. No L,l,K,nor 'Rho'
  tn1 = 0;
  tn2 = 0;
  t_transf = 0;  // this means no. of chars. received at Serial2=0. This may help
  lcd1.clear();
  lcd1.setCursor(0, 0);
  lcd1.print("F0");
  lcd1.print(" Test mode");
  lcd1.setCursor(0, 1);
  lcd1.print("");
  lcd1.setCursor(5, 2);
  lcd1.print("          ");  // set cursor at (char. 0,line-2)

  //for (i1=0; i1<=7;i1++) {lcd1.print(pgm_read_word_near(Lp+i1)); lcd1.print(" ");   }    // write 8 nos. from program memory. --now cancelled 31/Jan/2023



  lcd1.setCursor(0, 3);
  lcd1.print("--press Measur(9) ");
}
//...............................................................end of entry_fmc_Q..................................


//------------------------------------begin fnc_Q2-----------------------------
void fnc_Q2()  //---- when key_9 is pressed
{
  //  ----now timr7 will keep incrementing in L5149 ...)
  digitalWrite(27, LOW);
  timr7 = 0;
  timr7_flag = 1;  //  D27<--0  timr7 starts with '0'
}
//..............................................................
//-----------------------------------------------------------------------------------
void fnc_Q1()  // key-6 is pressed. No action
{
  entry_fnc_Q();  //like typing 'F0'
}  // 'timer++'
//.............................................................................


//'timer++ i.e. 1st line after 'Kb_Action
//............................................................................
//--------------------------------------------------------------------------------------
//  Initial Screen for 'L' mode select Survey mode
//.... . . . . ..-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.--
void L_Init1(void) {
  lcd1.clear();
  lcd1.setCursor(0, 0);
  lcd1.print("F6 Set Survey mode");  // line: 0 , lcd1.write(0xDB) ;
  lcd1.setCursor(0, 1);
  lcd1.print("1-Schlumberger");
  if (Surv_meth == 1) lcd1.write(byte(2));
  lcd1.setCursor(0, 2);
  lcd1.print("2-Wenner");
  if (Surv_meth == 2) lcd1.write(byte(2));  //star, line:1 , line:2
  lcd1.setCursor(10, 2);
  lcd1.print("3-Dipole");
  if (Surv_meth == 3) lcd1.write(byte(2));
  lcd1.setCursor(0, 3);
  if (Surv_meth == 1) lcd1.print("--Select 2/3-");
  if (Surv_meth == 2) lcd1.print("--Select 1/3-");
  if (Surv_meth == 3) lcd1.print("--Select 1/2-");  // line:3
  lcd1.print("else 4");                             // uncnditional printing
}
// ..................................end of L_Init1...................................
//--------------------------------------------------------------------------------------
// Schlumberger Screen for 'L' mode
//.... . . . . ..-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.--
void L_Scr_Schlum(void)  //key-1
{
  lcd1.clear();
  lcd1.setCursor(0, 0);
  lcd1.print("F6 Surv mode Set");  // line: 0 , lcd1.write(0xDB) ;
  lcd1.setCursor(0, 1);
  lcd1.print("1-Schlumberger");
  lcd1.write(byte(2));  //tick2, line:1 , line:2
  lcd1.setCursor(0, 3);
  lcd1.print("--press F2--");  // line:3
  Surv_meth = 1;
  tEA = EAd1 + (6 * IntSz);
  EEPROM.put(tEA, Surv_meth);  // ----store in E2prom
}
// ..................................end of L_Scr_Schlum...................................
// Wenner Screen for 'L' mode
//.... . . . . ..-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.--
void L_Scr_Wenn(void)  //key-2
{
  lcd1.clear();
  lcd1.setCursor(0, 0);
  lcd1.print("F6 Surv mode Set");  // line: 0 , lcd1.write(0xDB) ;
  lcd1.setCursor(0, 1);
  lcd1.print("2-Wenner");
  lcd1.write(byte(2));  //tick2, line:1 , line:2
  lcd1.setCursor(0, 3);
  lcd1.print("--press F2--");  // line:3
  Surv_meth = 2;
  tEA = EAd1 + (6 * IntSz);
  EEPROM.put(tEA, Surv_meth);  // ----store in E2prom
}
// ..................................end of L_Scr_Wenn...................................
// ..................................end of ...................................
//Dipole Screen for 'L' mode
//.... . . . . ..-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.--
// ----------------------------------------------------------------
void L_Scr_Dip(void)  //key-3
{
  lcd1.clear();
  lcd1.setCursor(0, 0);
  lcd1.print("F6 Surv mode Set");  // line: 0 , lcd1.write(0xDB) ;
  lcd1.setCursor(0, 1);
  lcd1.print("3-Dipole");
  lcd1.write(byte(2));  //tick2, line:1 , line:2
  lcd1.setCursor(0, 3);
  lcd1.print("--press F2--");  // line:3
  Surv_meth = 3;
  tEA = EAd1 + (6 * IntSz);
  EEPROM.put(tEA, Surv_meth);  // ----store in E2prom
}
// ..................................end of L_Scr_Dip...................................
// -------------------------L_No_change------------------------------------------------
void L_No_change(void)  //key-4
{
  lcd1.clear();
  lcd1.setCursor(0, 0);
  lcd1.print("F6 Surv mode Same");  // line: 0 , lcd1.write(0xDB) ;
  lcd1.setCursor(0, 1);
  lcd1.print("namely");
  lcd1.setCursor(0, 2);
  if (Surv_meth == 1) (lcd1.print("1-Schlumberger"));
  if (Surv_meth == 2) (lcd1.print("2-Wenner"));
  if (Surv_meth == 3) (lcd1.print("3-Dipole"));
  //lcd1.print("-");
  lcd1.write(byte(2));  //tick2, line:1 , line:2
  lcd1.setCursor(0, 3);
  lcd1.print("--press F2--");  // line:3
  //Surv_meth = 3;
  tEA = EAd1 + (6 * IntSz);
  EEPROM.put(tEA, Surv_meth);  // ----store in E2prom
}
//..........................end of L_No_change........................................
//-----------------------------------------------------------------------------------------------------
// void Show_LlK3(unsigned int n1, L, & l) (L,l --10* actual values}
//- - - - - - - - - - - - - -  - - - - - - - -  - - -- - -   -  -  - - - -
void Show_LlK3(unsigned int n1, unsigned int nL, unsigned int nl) {
  float Kvf;
  if (Surv_meth == 1)  // Schlumberger
  {
    fltLv = (float)nL / 10.0;
    fltlv = (float)nl / 10.0;
    Kvf = ScalcK(fltLv, fltlv);  //
    Ldig1 = nL % 10;
    ldig1 = nl % 10;
    if (Ldig1 == 0) Lint3 = nL / 10;
    if (ldig1 == 0) lint3 = nl / 10;  // if Ldig1==0 we use integer division, otherwise we use 'fltLv' a float value

    if (Ldig1 == 0) {
      lcd1.setCursor(0, 1);
      lcd1.print("                   ");
      lcd1.setCursor(0, 1);
      lcd1.print("L=");
      lcd1.print(Lint3);
    }  // L integer,erase line-2 (0~19)
    else {
      dtostrf(fltLv, 7, 1, st1);
      lcd1.setCursor(0, 1);
      lcd1.print("L=");
      lcd1.print(fltLv, 1);
    }  // L-Float, 2nd line

    if (ldig1 == 0) {
      lcd1.setCursor(11, 1);
      lcd1.print("l=");
      lcd1.print(lint3);
    }  // 'l'- integer,, 2nd line
    else {
      lcd1.setCursor(11, 1);
      lcd1.print("l=");
      lcd1.print(fltlv, 1);
    }  // 'l'-, 2nd line float

    lcd1.setCursor(0, 2);
    lcd1.print("                    ");  // erase entire line-2
    lcd1.setCursor(0, 2);
    lcd1.print("K=");
    lcd1.print(Kvf, 2);  // Kv, 3rd line
                         //---------------------------now update 'LSpcN2'-----------------------
    lcd1.setCursor(3, 0);
    lcd1.print("                 ");  // erased (3~19, 0)
    lcd1.setCursor(3, 0);
    lcd1.print("Rd");
    lcd1.print(LRdSr2 + 1);
    lcd1.print(" Sp");
    lcd1.print(LSpcN2 + 1);  // line-0, reading no.,Spacing no. 1~(n+1)numbering,Srv no.
    lcd1.setCursor(12, 0);
    lcd1.print(" Srv");
    lcd1.print(Srv_No);
  }
  if (Surv_meth == 2)  // Wenner
  {
    fltLv = (float)nL / 10.0;
    fltlv = (float)nl / 10.0;
    Kvf = WcalcK(fltLv);  // fltLv== float 'a');
    Ldig1 = nL % 10;
    ldig1 = nl % 10;
    if (Ldig1 == 0) Lint3 = nL / 10;
    if (ldig1 == 0) lint3 = nl / 10;  // if Ldig1==0 we use integer division, otherwise we use 'fltLv' a float value

    if (Ldig1 == 0) {
      lcd1.setCursor(0, 1);
      lcd1.print("                   ");
      lcd1.setCursor(0, 1);
      lcd1.print("a=");
      lcd1.print(Lint3);
    }  // L integer,erase line-2 (0~19)
    else {
      dtostrf(fltLv, 7, 1, st1);
      lcd1.setCursor(0, 1);
      lcd1.print("a=");
      lcd1.print(fltLv, 1);
    }  // L-Float'a' , 2nd line

    /*
  if (ldig1 == 0)   {
     lcd1.setCursor(11, 1); lcd1.print("l=");lcd1.print(lint3);} // 'l'- integer,, 2nd line 
  else {
    lcd1.setCursor(11, 1);lcd1.print("l="); lcd1.print(fltlv, 1);}  // 'l'-, 2nd line float   
                    */
    lcd1.setCursor(0, 2);
    lcd1.print("                    ");  // erase entire line-2
    lcd1.setCursor(11, 1);
    lcd1.print("K=");
    lcd1.print(Kvf, 2);  // Kv, 3rd line
                         //---------------------------now update 'LSpcN2'-----------------------
    lcd1.setCursor(3, 0);
    lcd1.print("                 ");  // erased (3~19, 0)
    lcd1.setCursor(3, 0);
    lcd1.print("Rd");
    lcd1.print(LRdSr2 + 1);
    lcd1.print(" Sp");
    lcd1.print(LSpcN2 + 1);  // line-0, reading no.,Spacing no. 1~(n+1)numbering,Srv no.
    lcd1.setCursor(12, 0);
    lcd1.print(" Srv");
    lcd1.print(Srv_No);
  }
  //-------------------------------------Below: Dipole case
  if (Surv_meth == 3)  // Dipole
  {
    fltLv = (float)nL / 10.0;
    fltlv = (float)nl / 10.0;
    Kvf = DipcalcK(nL, nl);  //
    Ldig1 = nL % 10;
    ldig1 = nl % 10;
    //if (Ldig1 == 0)   if (ldig1 == 0) /10; // if Ldig1==0 we use integer division, otherwise we use 'fltLv' a float value
    Lint3 = nL;
    lint3 = nl;  // a=Lint3  'n' = lint3
    Ldig1 = 0;
    ldig1 = 0;  // Ldig1,ldig1-- expected to '0' in dipole method
    if (Ldig1 == 0) {
      lcd1.setCursor(0, 1);
      lcd1.print("                   ");
      lcd1.setCursor(0, 1);
      lcd1.print("a=");
      lcd1.print(Lint3);
    }  // a= Lint3 (0~19)
    else {
      dtostrf(fltLv, 7, 1, st1);
      lcd1.setCursor(0, 1);
      lcd1.print("L=");
      lcd1.print(fltLv, 1);
    }  // L-Float, 2nd line

    if (ldig1 == 0) {
      lcd1.setCursor(11, 1);
      lcd1.print("n=");
      lcd1.print(lint3);
    }  // 'n'=lint3', 2nd line
    else {
      lcd1.setCursor(11, 1);
      lcd1.print("l=");
      lcd1.print(fltlv, 1);
    }  // 'l'-, 2nd line float

    // can't erase this line, because 'Dipo-Dip" letters exist in in this line from 10~to19. lcd1.setCursor(0, 2);lcd1.print("                    "); // erase entire line-2
    lcd1.setCursor(0, 2);
    lcd1.print("K=");
    lcd1.print(Kvf, 2);  // Kv, 3rd line
                         //---------------------------now update 'LSpcN2'-----------------------
    lcd1.setCursor(3, 0);
    lcd1.print("                   ");  // erased (3~19, 0)
    lcd1.setCursor(3, 0);
    lcd1.print("Rd");
    lcd1.print(LRdSr2 + 1);
    lcd1.print(" Sp");
    lcd1.print(LSpcN2 + 1);  // line-0, reading no.,Spacing no. 1~(n+1)numbering,Srv no.
    lcd1.setCursor(12, 0);
    lcd1.print(" Srv");
    lcd1.print(Srv_No);
  }
}
//.........................................end of Show_LlK3..........................................

//-----------------------------------------------show_LlK2 modified from {show_LlK------------------------------
//---------------------------------------------------------------------------------------------------
// Get L,l & K
//------------------------------------------------------------------------------------------------------------------
void Show_LlK2(unsigned int n1) {
  if (Surv_meth == 1)  // Schlumberger
  {
    LlpSz = 2 * IntSz;
    tEA = EAd4 + LlpSz * n1;  // EAd4,for Schlumberger spacing14/march/2023


    EEPROM.get(tEA, Lint1);
    tEA += IntSz;
    EEPROM.get(tEA, lint1);  //get( Lint1, lint1)from E2prom (these are 10*actual values)
    fltLv = (float)Lint1 / 10.0;
    fltlv = (float)lint1 / 10.0;
    Kv[LSpcN2] = ScalcK(fltLv, fltlv);  // Lvalue, lvalue &Kv
                                        // dtostrf (fltLv, 5, 1, st1);       GUI_DisString_EN (100, 130 , &st1[0], &Font12, Colr[7], BLUE);  //L
                                        // dtostrf (fltlv, 5, 1, st1);       GUI_DisString_EN (150, 130 , &st1[0], &Font12, Colr[7], BLUE);  //l
                                        // Lint3 used in Show_Eprom2 is actual value of L
    Ldig1 = Lint1 % 10;
    ldig1 = lint1 % 10;
    if (Ldig1 == 0) Lint3 = Lint1 / 10;
    if (ldig1 == 0) lint3 = lint1 / 10;  // if Ldig1==0 we use integer division, otherwise we use 'fltLv' a float value
                                         //GUI_DisNum (40,130, Lint1, &Font12, Colr[7],BLUE); GUI_DisNum (60,130, lint1, &Font12, Colr[7],BLUE);  //
                                         /*
  if (LSpcN2 == 0) {
    lcd1.clear();  lcd1.setCursor(0, 0); lcd1.print("Sr No ");  lcd1.setCursor(0, 1); lcd1.print("L= ");  // Sr. no. etc.(lettering)
    lcd1.setCursor(0, 2); lcd1.print("K= ");
  }      // "Sr No" at (0,0), "L=" & "l=" at (0,1) &(11,1) & "K=" at (0,2)
            */
    GUI_DisNum(10, 130, LSpcN2 + 1, &Font16, WHITE, BROWN);
    //----------------------Spac. no. already printed at at (6,0) in entry_fnc_H----------------
    //lcd1.setCursor(9, 0); lcd1.print("Sp");lcd1.print(LSpcN2 + 1); // Spacing no. printed already,.1st line

    GUI_DrawRectangle(10, 130, 70 + 240, 130 + 16, Colr[7], DRAW_FULL, DOT_PIXEL_DFT);  // erase earlier
    if (Ldig1 == 0) {
      GUI_DisNum(50, 130, Lint3, &Font16, WHITE, BROWN);
      lcd1.setCursor(0, 1);
      lcd1.print("L=");
      lcd1.print(Lint3);
    }  // L integer, 2nd line

    else {
      dtostrf(fltLv, 7, 1, st1);
      lcd1.setCursor(0, 1);
      lcd1.print("L=");
      lcd1.print(fltLv, 1);
    }  // L-Float, 2nd line

    if (ldig1 == 0) {
      GUI_DisNum(100, 130, lint3, &Font16, WHITE, BROWN);
      lcd1.setCursor(11, 1);
      lcd1.print("l=");
      lcd1.print(lint3);
    }  // 'l'- integer,, 2nd line
    else {
      dtostrf(fltlv, 7, 1, st1);
      GUI_DisString_EN(100, 130, &st1[0], &Font16, WHITE, BROWN);
      lcd1.setCursor(11, 1);
      lcd1.print("l=");
      lcd1.print(fltlv, 1);
    }  // 'l'-, 2nd line float

    dtostrf(Kv[LSpcN2], 7, 2, st1);
    GUI_DisString_EN(150, 130, &st1[0], &Font16, WHITE, BROWN);
    lcd1.setCursor(0, 2);
    lcd1.print("K=");
    lcd1.print(Kv[LSpcN2], 2);  // Kv, 3rd line
  }
  if (Surv_meth == 2)  //  Wenner
  {
    LlpSz = 1 * IntSz;
    tEA = EAd5 + LlpSz * n1;  //  EAd5, for Wennr  14/March/2023
    EEPROM.get(tEA, Lint1);   //   tEA += IntSz; EEPROM.get(tEA, lint1); //get( Lint1) only Lint1 = say,int1 from E2prom (these are 10*actual values)
    fltLv = (float)Lint1 / 10.0;
    Kvt = WcalcK(fltLv);                 //  fltlv = (float)lint1 / 10.0; Lvalue, only
                                         // dtostrf (fltLv, 5, 1, st1);       GUI_DisString_EN (100, 130 , &st1[0], &Font12, Colr[7], BLUE);  //L
                                         // dtostrf (fltlv, 5, 1, st1);       GUI_DisString_EN (150, 130 , &st1[0], &Font12, Colr[7], BLUE);  //l
                                         // Lint3 used in Show_Eprom2 is actual value of L
    Ldig1 = Lint1 % 10;                  //ldig1 = lint1 % 10;
    if (Ldig1 == 0) Lint3 = Lint1 / 10;  //if (ldig1 == 0) lint3 = lint1 / 10; if Ldig1==0 we use integer division, otherwise we use 'fltLv' a float value
                                         //GUI_DisNum (40,130, Lint1, &Font12, Colr[7],BLUE); GUI_DisNum (60,130, lint1, &Font12, Colr[7],BLUE);  //
                                         /*
  if (LSpcN2 == 0) {
    lcd1.clear();  lcd1.setCursor(0, 0); lcd1.print("Sr No ");  lcd1.setCursor(0, 1); lcd1.print("L= ");  // Sr. no. etc.(lettering)
    lcd1.setCursor(0, 2); lcd1.print("K= ");
  }      // "Sr No" at (0,0), "L=" & "l=" at (0,1) &(11,1) & "K=" at (0,2)
            */
    GUI_DisNum(10, 130, LSpcN2 + 1, &Font16, WHITE, BROWN);
    //----------------------Spac. no. already printed at at (6,0) in entry_fnc_H----------------
    //lcd1.setCursor(9, 0); lcd1.print("Sp");lcd1.print(LSpcN2 + 1); // Spacing no. printed already,.1st line

    GUI_DrawRectangle(10, 130, 70 + 240, 130 + 16, Colr[7], DRAW_FULL, DOT_PIXEL_DFT);  // erase earlier
    if (Ldig1 == 0) {
      GUI_DisNum(50, 130, Lint3, &Font16, WHITE, BROWN);
      lcd1.setCursor(0, 1);
      lcd1.print("a=");
      lcd1.print(Lint3);
    }  // L integer, 2nd line

    else {
      dtostrf(fltLv, 7, 1, st1);
      lcd1.setCursor(0, 1);
      lcd1.print("a=");
      lcd1.print(fltLv, 1);
    }  // L-Float, 2nd line
       /*
     if (ldig1 == 0)   {
    GUI_DisNum (100, 130 , lint3, &Font16, WHITE, BROWN); lcd1.setCursor(11, 1); lcd1.print("l=");lcd1.print(lint3);} // 'l'- integer,, 2nd line 
  else {
    dtostrf (fltlv, 7, 1, st1);GUI_DisString_EN (100, 130 , &st1[0], &Font16, WHITE, BROWN);lcd1.setCursor(11, 1);lcd1.print("l="); lcd1.print(fltlv, 1);}  // 'l'-, 2nd line float   
  
  dtostrf (Kvt, 7, 2, st1);  GUI_DisString_EN (150, 130 , &st1[0], &Font16, WHITE, BROWN); 
          */
    lcd1.setCursor(11, 1);
    lcd1.print("K=");
    lcd1.print(Kvt, 2);  // Kv, Kine-1
  }
  //--------------------------Dipole-Dipole , below-------------------------------------------------
  if (Surv_meth == 3)  // Dipole-Dipole
  {
    LlpSz = 2 * IntSz;
    tEA = EAd9 + LlpSz * n1;  // (LlpSz=4 bytes) EAd9,for Dipole-Dipole


    EEPROM.get(tEA, Lint1);
    tEA += IntSz;
    EEPROM.get(tEA, lint1);  //get( Lint1, lint1)from E2prom (these are 10*actual values)
    fltLv = (float)Lint1 / 10.0;
    fltlv = (float)lint1 / 10.0;
    Kv[LSpcN2] = DipcalcK(Lint1, lint1);  // Lvalue, lvalue &Kv
                                          // dtostrf (fltLv, 5, 1, st1);       GUI_DisString_EN (100, 130 , &st1[0], &Font12, Colr[7], BLUE);  //L
                                          // dtostrf (fltlv, 5, 1, st1);       GUI_DisString_EN (150, 130 , &st1[0], &Font12, Colr[7], BLUE);  //l
                                          // Lint3 used in Show_Eprom2 is actual value of L
    Ldig1 = Lint1 % 10;
    ldig1 = lint1 % 10;
    if (Ldig1 == 0) Lint3 = Lint1 / 10;
    if (ldig1 == 0) lint3 = lint1 / 10;  // if Ldig1==0 we use integer division, otherwise we use 'fltLv' a float value
                                         //GUI_DisNum (40,130, Lint1, &Font12, Colr[7],BLUE); GUI_DisNum (60,130, lint1, &Font12, Colr[7],BLUE);  //
                                         /*
  if (LSpcN2 == 0) {
    lcd1.clear();  lcd1.setCursor(0, 0); lcd1.print("Sr No ");  lcd1.setCursor(0, 1); lcd1.print("L= ");  // Sr. no. etc.(lettering)
    lcd1.setCursor(0, 2); lcd1.print("K= ");
  }      // "Sr No" at (0,0), "L=" & "l=" at (0,1) &(11,1) & "K=" at (0,2)
            */
    GUI_DisNum(10, 130, LSpcN2 + 1, &Font16, WHITE, BROWN);
    //----------------------Spac. no. already printed at at (6,0) in entry_fnc_H----------------
    //lcd1.setCursor(9, 0); lcd1.print("Sp");lcd1.print(LSpcN2 + 1); // Spacing no. printed already,.1st line

    GUI_DrawRectangle(10, 130, 70 + 240, 130 + 16, Colr[7], DRAW_FULL, DOT_PIXEL_DFT);  // erase earlier
    Ldig1 = 0;
    ldig1 = 0;  // in Dipole-Dipole method both are expected to be 0
    if (Ldig1 == 0) {
      GUI_DisNum(50, 130, Lint3, &Font16, WHITE, BROWN);
      lcd1.setCursor(0, 1);
      lcd1.print("a=");
      lcd1.print(Lint1);
    }  //note 'a' =Lint1 from E2prom L integer, 2nd line

    else {
      dtostrf(fltLv, 7, 1, st1);
      lcd1.setCursor(0, 1);
      lcd1.print("L=");
      lcd1.print(fltLv, 1);
    }  // L-Float, 2nd line

    if (ldig1 == 0) {
      GUI_DisNum(100, 130, lint3, &Font16, WHITE, BROWN);
      lcd1.setCursor(11, 1);
      lcd1.print("n=");
      lcd1.print(lint1);
    }  // note 'c' =lint1from E2prom'l'- integer,, 2nd line
    else {
      dtostrf(fltlv, 7, 1, st1);
      GUI_DisString_EN(100, 130, &st1[0], &Font16, WHITE, BROWN);
      lcd1.setCursor(11, 1);
      lcd1.print(fltlv, 1);
    }  // 'l'-, 2nd line float

    dtostrf(Kv[LSpcN2], 7, 2, st1);
    GUI_DisString_EN(150, 130, &st1[0], &Font16, WHITE, BROWN);
    lcd1.setCursor(0, 2);
    lcd1.print("K=");
    lcd1.print(Kv[LSpcN2], 2);  // Kv, 3rd line
    Show_LlK3(n1, Lint1, lint1);
    Dip_a = Lint1;
    Dip_n = lint1;  // (n1=LSpcn2, a,n )
  }
}

//}
//.....................................................................end of 'Show_LlK2' .............................
//..................................................................
// -------------------------------------------------------------------------------------------------
//  show 'N1' mode,Last Reading Sr. no.-2,Last Spacing Sr. no. -2,Function set no.G~P ('alpha'~ 'omicron')
//-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.--.-.-.-.-.-.-.--.-.--..-.-.-.-.-.-.--.-
void Screen_1()  //outdated  --measurement of Resistance, code-'m', on Screen LCD
{
}
//.........................................end of Screen_1..(N1)......................................
// -------------------------------------------------------------------------------------------------
//  show show 'N2'mode,Last Reading Sr. no.-2,Last Spacing Sr. no. -2,measure again, code-'n'
//-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.--.-.-.-.-.-.-.--.-.--..-.-.-.-.-.-.--.-
void Screen_2()  //meas. of Resistance, C1-C2 open, code-'n',on Screen LCD
{
}
//..............................end of Screen_2......................................................................
//------------------------------Screen_3-below---(N3)------------------------------------------------------
// -------------------------------------------------------------------------------------------------
//  show show 'N3'mode,Last Reading Sr. no.-2,Last Spacing Sr. no. -2,start measurement of Resistance, 'Press Measure
//-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.--.-.-.-.-.-.-.--.-.--..-.-.-.-.-.-.--.-
void Screen_3()  //start meas. of Resistance, code--'o', on Screen LCD
{
}
//.........................................end of Screen_3().....................................
//...................................................................................................
//------------------------------Screen_4-below---(N4)------------------------------------------------------
// -------------------------------------------------------------------------------------------------
//  show show 'N3'mode,Last Reading Sr. no.-2,Last Spacing Sr. no. -2, measurement of Resistance started, '
//-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.--.-.-.-.-.-.-.--.-.--..-.-.-.-.-.-.--.-
void Screen_4()  //insert Batt,current & Resistance, code--'s', on Screen LCD
{
}
//.........................................end of Screen_4().....................................
//--------------------------------------------------------------Alpha_1--(below)-------------------------
//   about 'Survey Status'
//-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.--..-.-.-.-.-.-.-.-.-.-.-.-.-.-.--.-.-.--.-.-.--.-.-.--..-.
void Alpha_1()  // code - 'p', on Screen LCD
{
}
//.............................................. end of 'Alpha_1'..................................................
//----------------------------------------begining of Alpha_2---------------------------------------------------------------------
//--------------------------------------------------------------Alpha_2---------------------------
//   about 'Survey closed'
//-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.--..-.-.-.-.-.-.-.-.-.-.-.-.-.-.--.-.-.--.-.-.--.-.-.--..-.
void Alpha_2()  // code - 'q',on Screen LCD
{
}
//.............................................. ..................................................
//....................................................end of Alpha_2.............................................................

//--------------------------------------------------------------Alpha_3---------------------------
//   about 'New Survey opened'
//-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.--..-.-.-.-.-.-.-.-.-.-.-.-.-.-.--.-.-.--.-.-.--.-.-.--..-.
void Alpha_3()  // code - 'r',on Screen LCD
{
}
//.............................................. ..................................................
//....................................................end of Alpha_3.............................................................
//-------------------------------------------------Normal_1  is normal measurement screen ---------------------------
//  ----- show reading,spacing Lv,lv,Kv
//-------------------------------------------------------------------------------------------------
void Normal_1()  // writes data on Screen LCD only
{
}
//......................................................end Normal_1().........................
void Wr_A2A4() {
}
/*.........................................................................
            -Wr2 pulse to U1,8255 IC on A4_D1 card
  ............................................................................*/
void Wr2_pulse() {
}

/* ............................................................................
   Walking '1' on ports A & C/G
  .............................................................................  */

void Test_Port(void) {

}  //  ------ end of function Test_Port  ------------------------
/*******************************************************************************
  function:
        show  timer5 value
*******************************************************************************/
void Show_Timr5(void) {
  n3++;          //Serial.println("Drawing.... 3 ..");
  n5 = TCNT5;    // read the value 'ext-ck' timer5
  nby7 = TCNT0;  // read the value 'ext-ck' timer0
  //n7= (nby8*256)+ nby7;    // nby8 is upper byte
  n7 = (nby8 * 250) + nby7;  // nby7 goes from 0 to 19. nby8 is upper byte

  // GUI_DisNum (100,100,n2, &Font16, WHITE,WHITE);     // erase earlier no.
  // GUI_DisNum (100,100,n3, &Font16, WHITE,BLUE);

  if (n5 != n4)  // print timr5 whenever its value changes
  {
    GUI_DisNum(130, 130, n4, &Font20, WHITE, WHITE);  // erase earlier no. n4
    GUI_DisNum(130, 130, n5, &Font20, WHITE, RED);    // print the new value of timr5
    n2 = n3;                                          // the present no. n3 will get treated as old no. (n2) in the next pass )
    n4 = n5;                                          // the present no. n5 will get treated as old no. (n4) in the next pass )
    Serial.println("printing....  Timr-5..");
  }
  //............................................................................
  if (n7 != n6)  // print timr0 whenever its value changes
  {
    GUI_DisNum(160, 145, n6, &Font20, WHITE, WHITE);  // erase earlier no. n6
    GUI_DisNum(160, 145, n7, &Font20, WHITE, BLACK);  // print the new value of timr0

    GUI_DisNum(220, 145, old7, &Font20, WHITE, WHITE);  //erase the old7
    GUI_DisNum(220, 145, nby7, &Font20, WHITE, BLACK);  // print the new value of timr0

    GUI_DisNum(280, 145, old8, &Font20, WHITE, WHITE);  // erase the old8 of timr0
    GUI_DisNum(280, 145, nby8, &Font20, WHITE, BLACK);  // print the new value of timr0
    n6 = n7;
    old7 = nby7;
    old8 = nby8;  // the present no. n7 will get treated as old no. (n6) in the next pass

    Serial.println("printing....  Timr-0..");
  }
}
//---------------end of Show Timer 5--------------------------------------------------
//     (int) Get_key  :-- returns a byte= keycode
//-----------------------------------------------------------------
byte Get_key() {
  //------  entry_fnc_I();----- not used--------
}
//  .........................................end of Get_key.........
//-------------------------ISR(Timer4 interrupt)--------------------
//..........................end of'ISR(Timer4 interrupt)'.............
/**************************************************************************
      cf    Interrupt every 10 mSec, because OCR3A=625 in Timer3 initialization
 *************** ****************************************************/

ISR(TIMER3_COMPA_vect) {
  volatile byte k3;  //  1{  k3=column no.
  RtLD = 0;
  if (digitalRead(Kbin0) == LOW) RtLD |= 0x10;
  if (digitalRead(Kbin1) == LOW) RtLD |= 0x20;
  if (digitalRead(Kbin2) == LOW) RtLD |= 0x40;
  if (digitalRead(Kbin3) == LOW) RtLD |= 0x80;  // RtLD[bits 7,6,5,4]=Kbin0/1/2/3
  //if ( k1==0) keyBf0= ch7+1;    // ch7='d'
  {
    Cl[k6].DbD[Cl[k6].j] = RtLD;
    Cl[k6].j++;
    if (Cl[k6].j >= 3) Cl[k6].j = 0;                                                           //k6=0/1/2/3 only. similarly , j= 0/1/2 only
    if (Cl[k6].DbD[0] == 0 && Cl[k6].DbD[1] == 0 && Cl[k6].DbD[2] == 0) Cl[k6].DefKSt[0] = 0;  //Key was 'up' for 3 consec. samples
    Cl[k6].KSt = (Cl[k6].DbD[0]) & (Cl[k6].DbD[1]) & (Cl[k6].DbD[2]);                          //  AND 3 samples (Earlier: 3 samples)
    if (Cl[k6].KSt != 0 && Cl[k6].DefKSt[0] == 0) {
      keyBf0 = Cl[k6].KSt | Sccd[k6];  // 'Lastkey_Status' defined for next pass if (RtLD !=0)    keyBf0= RtLD;
      Cl[k6].DefKSt[0] = Cl[k6].KSt;
    }
  }  //present  .KSt 'not='0 And last Definite state was 'Key_Up' then keyBf0 is now defined.
  //But Cl[k6].DefKSt[0] updated
  // now change to next scan code

  k6++;
  if (k6 >= 4) k6 = 0;
  digitalWrite(otpin0, HIGH);
  digitalWrite(otpin1, HIGH);
  digitalWrite(otpin2, HIGH);
  digitalWrite(otpin3, HIGH);  //digitalWrite(24,HIGH);
  if (k6 == 0) digitalWrite(otpin0, LOW);
  if (k6 == 1) digitalWrite(otpin1, LOW);
  //if (digitalRead (23)==LOW) digitalWrite(24,LOW); }
  if (k6 == 2) digitalWrite(otpin2, LOW);
  if (k6 == 3) digitalWrite(otpin3, LOW);
  //-------------------------------------------------------------------
  timr3++;
  tm4 = timr3 % 200;
  timr4++;
  timr5++;
  timr6++;           // tm4 goes from 0 to 199 only
  if (F_kpr == 1) {  // if F_kpr==1, it means that 'F' is to be turned  on & off ~ every 2 Seconds
    if (timr5old <= 50 && timr5 > 50) {
      lcd1.setCursor(0, 0);  // print 'F' (in top left corner) when timr5 crosses value of 200
      lcd1.print('F');
    }
    if (timr5old <= 150 && timr5 > 150) {
      lcd1.setCursor(0, 3);  // print ' '(blank) when timr5 crosses value of 200L
      lcd1.print(' ');
    }
  }
  // timr7 is enabled in 'F0' or 'F2' when we to turn on CRM_Auto_D
  if (timr7_flag == 1) timr7++;  // count only while timr7_flag == 1  .timer7=100 means 1 Second has elapsed
  if (timr7 >= 100) {
    digitalWrite(27, HIGH);
    timr7 = 0;
    timr7_flag = 0;
  }  // turn off CRM-Auto_D main instrument

  //if (timr6old <=200 && timr6 > 200 )  entry_fnc_J1() ;  // 1st screen

  //if (timr6old <=500 && timr6 > 500 ) entry_fnc_J2() ;    // 2nd screen
  //  /*
  // RdNo1= RdNo
  //if ( tm4old <=tlim2 && tm4>tlim2 )       // show '1' Read E2p & print

  if (sh_sg == 1) {
    if (timr3old <= (tmcn + 1) * 100 && timr3 > (tmcn + 1) * 100) {
      tmcn++;
      if (tmcn <= 7) {
        lcd1.setCursor(2 + tmcn, 2);
        lcd1.write(sgsy[tmcn]);
      }
      timr3old = timr3;  // update timr3old
    }
  }
  if (timr5 > 200) timr5 = 0;     // thus, timr5 goes from 0 to200, 2 asec.
  if (timr6 > 600) timr6 = 0;     // thus, timr6 goes from 0 to 600, 6 Sec.
  if (timr4 >= 12000) timr4 = 0;  // timrr4 goes from 0 ~ 120 Sec(2 minutes)
  if (timr3 >= 801) {
    timr3 = 0;  // timr3 = 800, means 8 Seconds have elapsed
    tmcn = 0;
    sh_sg = 0;
  }
  if (Show_Alt == 1) {


    // RdNo1= RdNo
    if (tm4old <= tlim3 && tm4 > tlim3) {  //tm4 just crosses tlim3(50)
      lcd1.setCursor(19, 3);
      lcd1.write(0x23);
      lcd1.setCursor(12, 1);
      lcd1.cursor();  // write '#'

      if (ShSpcRd == 1) {
        if (n14 == 0) lcd1.clear();
        lcd1.setCursor(10, 3);
        lcd1.print(n14);
        n14++;
        if (n14 > 22) n14 = 0;
        lcd1.setCursor(0, 1);
        lcd1.print(n14 + 1);
        if (n14 < 8) lcd1.print(" ");
        lcd1.setCursor(3, 1);
        lcd1.print("L=");
        lcd1.print(Lv[LCNo[n14]], 1);
        lcd1.setCursor(10, 1);
        lcd1.print("l=");
        lcd1.print(lv[lCNo[n14]], 1);
        lcd1.setCursor(3, 2);
        lcd1.print("K=");
        lcd1.print(Kv[n14], 2);  // 1st line-No,L,l,K 2nd line- values of No,L,l, 3rd line--K
      }
      //.............................................end of 'Show Spacings.................
      //-------------------------- Show Readings stored in SD --------------------------
      if (ShSpcRd == 2) {
        lcd1.clear();
        lcd1.setCursor(19, 3);
        lcd1.print("1");
        lcd1.setCursor(0, 0);
        lcd1.print("M ");
        lcd1.print(MRdN1 + 1);  // Reading no, MRdN1
        lcd1.setCursor(4, 0);
        lcd1.print("L= ");
        lcd1.print(MLv[MRdN1], 1);  //  MLv
        lcd1.setCursor(12, 0);
        lcd1.print("l= ");
        lcd1.print(Mlv[MRdN1], 1);  // Mlv
        lcd1.setCursor(0, 1);
        lcd1.print("K= ");
        lcd1.print(MKv[MRdN1], 2);  //MKv
        lcd1.setCursor(9, 1);
        lcd1.print("R=");
        lcd1.print(MRes[MRdN1], 2);
        lcd1.print("m");
        lcd1.write(0xF4);  // MRes
        lcd1.setCursor(1, 2);
        lcd1.write(0xE6);
        lcd1.print("=");
        lcd1.print(MRho[MRdN1], 2);
        lcd1.write(0xF4);
        lcd1.print("-m");  // MRho
        MRdN1++;
        if (MRdN1 >= 6) MRdN1 = 0;  // if there are only 4 readings
      }
      // .................................end of rdng. from SD................
    }  // 4th line,20th col.if (timr3old=80 & timr3=81)(2 Sec) show 1


    //lcd1.clear();lcd1.setCursor(0, 0);  m7=RdNo1+1; lcd1.print(m7); lcd1.setCursor(2, 0); lcd1.print("L="); lcd1.print(Lv[RdNo1],1); // 1st line
    //lcd1.setCursor(10, 0); lcd1.print("l=");lcd1.print(lv[RdNo1],1); // 1st line
    //lcd1.setCursor(0, 1);lcd1.print("R=");  lcd1.print(Rest[RdNo1],2); if (Resm< 1000) lcd1.print(" m-Ohm");// 2nd line
    //}     //

    if (tm4old <= tlim4 && tm4 > tlim4) {  //tm4 crosses tlim4=150
      lcd1.setCursor(19, 3);
      lcd1.write(0xA0);  // write 'blank'
      lcd1.setCursor(12, 1);
      lcd1.noCursor();
    }  // 4th line,20th col{if (timr3old=240 & timr3=241)(6 Sec) show 2}

    //lcd1.clear();lcd1.setCursor(0, 0);lcd1.print("K=");  lcd1.print(Kv[RdNo1],2);  // 1st line (of Scren2 )( Earlier: on 3rd line)
    //lcd1.setCursor(9,0);lcd1.print("Rho=");  lcd1.print(Rho[RdNo1],2);  //1st line (of Scren2 )( Earlier: on 3rd line)
    // }
    tm4old = tm4;
    timr6old = timr6;  // update tm4old & timr6old

    // timr3 =40 ( or 100 ?)means 1 Sec.has elapsed. timr3 is cleared after 8 sec. 1 or 2 is displayed at line 1(2nd line) posn. 13 (14 th place)
  }
  timr5old = timr5;  // update timr5olduuuuu
}
//---------------------------------end of ISR(Timer3_COMPA_vector) --------------------------------------------------------


//  ------------------------------------------------------
void Show_Spc(int Sp) {
}
// ----------------------------------------------------------------------------
//-------------------------------------------------------------------------------------
//  Timer5 -- Interrupt (~once every Second)
//-------------------------------------------------------------------------------------

ISR(TIMER5_COMPA_vect)  // Interrupt when Timer5 count reaches 60,000 (62500/65536)=0.953 Second
{
  timr5++;
  if (timr5 >= 3) {
    timr5 = 0;
    digitalWrite(24, HIGH);
  }
}
/*******************************************************************************
     Interrupt routine called when Timr0 overflows
 ******************************************************************************/
ISR(TIMER0_COMPA_vect)  // Interrupt when Timer0reaches 250 ( was 20 earlier)              //  ISR(TIMER0_OVF_VECT)
{
  nby8 += 1;  // add 1 to the upper byte nby8
}

//  *****************************************************************
//---------------------------------Timr4--------------------------------
/*******************************************************************************
     Interrupt routine called when Timr4 overflows,once every Second
 ******************************************************************************/

ISR(TIMER4_COMPA_vect)  // Interrupt when Timer0reaches 250 ( was 20 earlier)              //  ISR(TIMER0_OVF_VECT)
{
  n5 = TCNT5;  // Frq+
  nby7 = TCNT0;
  n7 = (nby8 * 250 + nby7);  //Frq-
                             //show n5 & n7 on LCD
}

//  *****************************************************************
//.....................................end of Timr4.......................
/*******************************************************************************
  function:
       Update Dispay
*******************************************************************************/
void Updt_Displ(void) {

}  //  end of function 'Updt_disp'



/* ----------------------------------------------------------------   */
/*  (float) wt <-- string received from weighing machine       */
/* ----------------------------------------------------------------   */
void Show_wt(char* st2) {
}

/*******************************************************************************
  function:
      Initialize 8255 ( Actually this is done in 'Setup'
*******************************************************************************/
//-----------------------------------------------------------------
//  DAC <-- byt1 (In the A4D1 circuit, the bits 7~0 are swapped. Also there is an inversion
//
//--------------------------------------------------------------------
void A4_D1_DAC(byte byt1)

{
  //
}
/*******************************************************
     result [7~0]<-- Num[0~7] ( Bits reversed
 *******************************************************/
byte RevBits(byte Num) {
}

/*******************************************************
     8255_port(Ad) <-- x  (This function now outdated because 3 shift registers are used )
 *******************************************************/

/*******************************************************
    Delay 'del1'
*******************************************************/
void del1() {
  unsigned long i10, k1;
  for (i10 = 1; i10 < 200000; i10++) k1 = k1 + 1;  // delay with dummy k1++
}
/*******************************************************
    initialization of Timer
*******************************************************/

/*******************************************************************************
  function:  write into 8255 port
 *******************************************************************************/
void wrt_Pr(void) {}
/*******************************************************************************
  function:
     //  temporary Get_GPS2 ---- not used now  ---for testng

       -
       *******************************************************************************/

void Get_GPS2(void) {  // 1{
                       /*
             while (Serial1.available()   )     
              { ch11=Serial1.read();  if (ch11>0x21 && ch11<= 0x5F) Serial.print(ch11);  else Serial.print(ch11,HEX);  // print '$' or hex vallue
              Serial.print(" ") ;  // 1 space
              */
  //----------------------------------------------------------------------------------------------------
  if (Serial1.available() > 0) {  // 2{
    ch11 = Serial1.read();
    if (ch11 == '$') {  // 3{}
      //----------------------------------------------------------------------------------------------------------------
      //   /*
      volatile unsigned char message[100];
      volatile unsigned int ind = 0;
      message[ind++] = ch11;
      Serial.print('$');
      while (Serial1.available() > 0 && ch11 != '\n') {  // 4{
        ch11 = Serial1.read();
        message[ind++] = ch11;
        if (ch11 == 0x2C) Serial.print(",");
        else {
          if (ch11 >= '0' && ch11 <= '9') {
            ch10 = ch11 - '0';
            Serial.print(ch10);
          } else Serial.print(ch11, HEX);
        }  // 4}
      }
      message[ind] = '\0';           // 'zero' terrminated String ?
      if (strstr(message, "GPRMC"))  // $GPGGA
      {
        Serial.print("$GPRMC");  // 5{  $GPGGA
        char* p = strtok(message, ",");
        volatile unsigned int i = 0;
        volatile unsigned char* array[15];
        while (p != NULL) {
          array[i++] = p;
          p = strtok(NULL, ",");
        }
        latitude = atof(array[2]) / 100;
        longitude = atof(array[4]) / 100;  // get Latitude & Longitude   volatile  float {now declared globally at Ln 200}
        Serial.print("Latitude=");
        Serial.print(latitude, 2);
        Serial.print("Longitude=");
        Serial.print(longitude, 2);  // show on Serial monitor
                                     // print on LCD
                                     // 5}
      }                              //   */
    }                                // 3}
                                     //........................................................................................................
                                     /*
           String message="$";
           while (Serial1.available() > 0 && ch11 != '\n' )  
          {      // 4{  
           ch11 = Serial1.read() ;  message+= ch11;
          }     // 4}
          if (message.substring(3,6) == "GGA" )
         {  //5{
           int comma1=message.indexOf (',' , 7); int comma2=message.indexOf (',' , comma1+1); int comma3=message.indexOf (',' , comma2+1); int comma4=message.indexOf (',' , comma3+1); 
         int comma5=message.indexOf (',' , comma4+1); int comma6=message.indexOf (',' , comma5+1); int comma7=message.indexOf (',' , comma6+1); int comma8=message.indexOf (',' , comma7+1);  
         int comma9=message.indexOf (',' , comma8+1);  int comma10=message.indexOf (',' , comma9+1); int comma11=message.indexOf (',' , comma10+1); int comma12=message.indexOf (',' , comma11+1); 
         
          String latitude = message.substring (comma1+1,comma2) ;  String longitude = message.substring (comma3+1,comma4) ;
        Serial.println("Lattitude:"+latitude+ "Longitude:"+longitude); 
            }   //5}
              */
  }                                  // 2}
                                     // 2}

}  // 1}
   //*/
//---------------------------------------------------------------------------------


//--------------------------------------------------------------------------------------
//   Get_GPS() --  get & display GPS & Date,Time
//-----------------------------------------------------------------
/*
void Get_GPS(void) {
  unsigned int x1, y1;  // {1
  // if ( Kbkaz>= 'a' && Kbkaz<='z')
  //{          // 1.5 get & Show GPS only if any key 'a' ~ 'z' has been taken
  while (Serial1.available())          //while
  {                                    //2
    if (gps.encode(Serial1.read())) {  //{3
      //--------------------------'try new metods'----- (Ref. Shankaran's printout, 5/oct/2019)--------
      //lat= gps.location.lat();   lon= gps.location.lng();  ltg0 = lat; ltg1 = lon;   //ltgo--latitude,ltg1--longitude
      // compiler does not recognize methods .location.lat() nor  .location.lng()
      // .....................................................................................................
      gps.f_get_position(&lat, &lon);
      ltg0 = lat;
      ltg1 = lon;  // 2 new variables for 'Lattitude' & 'Longitude'
      ldg0 = ltg0 / 1;
      frdg0 = ltg0 - (float)(ldg0);
      lx0 = frdg0 * 60.0;
      lmin0 = lx0 / 1;
      frmin0 = lx0 - (float)(lmin0);
      lsec0 = frmin0 * 60.0;  // for Lattitude
      ldg1 = ltg1 / 1;
      frdg1 = ltg1 - (float)(ldg1);
      lx1 = frdg1 * 60.0;
      lmin1 = lx1 / 1;
      frmin1 = lx1 - (float)(lmin1);
      lsec1 = frmin1 * 60.0;  // for Longitude
      if (LnNo == 1) {
        Rlsec0 = lsec0;
        Rlsec1 = lsec1;
      }
      err0 = lsec0 - Rlsec0;
      err1 = lsec1 - Rlsec1;
      //GUI_DrawRectangle(20-1, 45,  460,45+13 , YELLOW, DRAW_FULL, DOT_PIXEL_DFT);// Earlier erase Rect.440x13 (now,small rects. 24x13)
      GUI_DisString_EN(10, 45, "Lat: ", &Font12, WHITE, BLUE);  // lcd1.clear();   lcd1.setCursor(1, 0); lcd1.print("Lat: ");
      if (ldg0 != ldg0L && LnNo >= 1) {
        GUI_DrawRectangle(60 - 1, 45, 60 + 24, 45 + 13, YELLOW, DRAW_FULL, DOT_PIXEL_DFT);
        GUI_DisNum(60, 45, ldg0, &Font16, YELLOW, BLUE);  // first erase 2 Degrees
      }                                                   // lcd1.print("/");

      if (lmin0 != lmin0L && LnNo >= 1) {
        GUI_DrawRectangle(90 - 1, 45, 90 + 24, 45 + 13, YELLOW, DRAW_FULL, DOT_PIXEL_DFT);  //Minutes (Lattit.)
        GUI_DisNum(90, 45, lmin0, &Font16, YELLOW, BLUE);
      }
      dtostrf(lsec0, 5, 2, st1);  // Seconds
      GUI_DrawRectangle(120, 45, 120 + 60, 45 + 13, YELLOW, DRAW_FULL, DOT_PIXEL_DFT);
      GUI_DisString_EN(120, 45, &st1[0], &Font16, YELLOW, BLUE);  //erase & print,lsec0,err0
      dtostrf(err0, 5, 2, st1);                                   // err0,Seconds
      GUI_DrawRectangle(185, 45, 185 + 60, 45 + 13, YELLOW, DRAW_FULL, DOT_PIXEL_DFT);
      GUI_DisString_EN(185, 45, &st1[0], &Font16, YELLOW, BLUE);  //
                                                                  //lcd1.setCursor(0,1); lcd1.print("Latt:"); lcd1.print(ldg0);lcd1.print(" ");lcd1.print(lmin0); lcd1.print(" ");lcd1.print(lsec0,1); //Latt:deg,min,sec
                                                                  //lcd1.setCursor(0,2); lcd1.print("Long:"); lcd1.print(ldg1);lcd1.print(" ");lcd1.print(lmin1); lcd1.print(" ");lcd1.print(lsec1,1); //Long:deg,min,sec
      if (LnNo == 0) {
        GUI_DisNum(60, 45, ldg0, &Font16, YELLOW, BLUE);  // Degrees & Minutes (initialization)(Latt.)
        GUI_DisNum(95, 45, lmin0, &Font16, YELLOW, BLUE);
      }

      //  next: Lattitude & Longitude, (suppressed: 8/Feb/2023)
      //   /*
      Serial.print("Pos: ");
      Serial.print("Latt: ");
      Serial.print(ldg0);
      Serial.print(" ");
      Serial.print(lmin0);
      Serial.print(" ");
      Serial.print(lsec0, 2);
      Serial.print(" ");
      Serial.print(err0, 2);
      // next: longitude
      Serial.print("  Long: ");
      Serial.print(ldg1);
      Serial.print(" ");
      Serial.print(lmin1);
      Serial.print(" ");
      Serial.print(lsec1, 2);
      Serial.print(" ");
      Serial.print(err1, 2);
      Serial.print(" ");
      //  
      GUI_DisString_EN(240, 45, "Long: ", &Font12, WHITE, BLUE);
      if (ldg1 != ldg1L && LnNo >= 1) {
        GUI_DrawRectangle(280, 45, 280 + 24, 45 + 13, YELLOW, DRAW_FULL, DOT_PIXEL_DFT);  // Degrees(Long.)
        GUI_DisNum(280, 45, ldg1, &Font16, YELLOW, BLUE);
      }
      if (lmin1 != lmin1L && LnNo >= 1) {
        GUI_DrawRectangle(305, 45, 303 + 24, 45 + 13, YELLOW, DRAW_FULL, DOT_PIXEL_DFT);  //Minutes
        GUI_DisNum(303, 45, lmin1, &Font16, YELLOW, BLUE);
      }
      dtostrf(lsec1, 5, 2, st1);  // Seconds
      GUI_DrawRectangle(330, 45, 330 + 60, 45 + 13, YELLOW, DRAW_FULL, DOT_PIXEL_DFT);
      GUI_DisString_EN(330, 45, &st1[0], &Font16, YELLOW, BLUE);  // erase & print,lsec1,err1
      dtostrf(err1, 5, 2, st1);                                   // err1,Seconds
      GUI_DrawRectangle(400, 45, 400 + 60, 45 + 13, YELLOW, DRAW_FULL, DOT_PIXEL_DFT);
      GUI_DisString_EN(400, 45, &st1[0], &Font16, YELLOW, BLUE);  // erase & print
      if (LnNo == 0) {
        GUI_DisNum(280, 45, ldg1, &Font16, YELLOW, BLUE);  // Degrees & Minutes (initialization) (Long.)
        GUI_DisNum(305, 45, lmin1, &Font16, YELLOW, BLUE);
      }
      // next: date & time
      gps.crack_datetime(&Yr, &Mn, &Dte, &Hr, &mint, &Scnd, &DeciSec, &f_age);  // get date & time
      mint += 30;
      if (mint >= 60) {
        mint -= 60;
        Cr = 1;
      } else Cr = 0;
      Hr += 5 + Cr;
      ldg0L = ldg0;
      ldg1L = ldg1;
      lmin0L = lmin0;
      lmin1L = lmin1;
      LAltit = gps.altitude();
      vr4 = (float)LAltit / 100;  // update 'Last' values

      //--------------------------following 2 lines for writing data on Serial monitor- date & time---suppresed: 8/Feb/2023----------------------------------------
      //  /*
      Serial.print(Dte);
      Serial.print("/");
      if (Mn == 2) Serial.print("Feb");
      if (Mn == 3) Serial.print("March");
      Serial.print("/");
      Serial.print(Yr);
      Serial.print("--");  // date
      Serial.print(Hr);
      Serial.print(":");
      Serial.print(mint);
      Serial.print(":");
      Serial.print(Scnd);
      Serial.print(" ");    // time & NL
      Serial.println(vr4);  //  print Alitude (float) & New line
                            //   
      //-----------------------------lcd1 statements--------------------------
      lcd1.setCursor(0, 0); lcd1.print("Date:"); lcd1.print(Dte); lcd1.print("/"); lcd1.print(Mn); lcd1.print("/"); lcd1.print(Yr); // dd/mm/yyyy
        lcd1.setCursor(0, 1); lcd1.print("Time:"); lcd1.print(Hr); lcd1.print(":"); lcd1.print(mint); lcd1.print(":"); lcd1.print(Scnd); // hh:mm:ss
               lcd1.setCursor(14, 1);
               lcd1.print("A");  //Altitude
               vr4 = (float)LAltit / 100;
                 lcd1.print(vr4,0);   lcd1.print("m.");  //Laltit is in cm.       
        
        //-----------now Latitude & Longitude---lines 2 & 3----------------------------------------
        lcd1.setCursor(0,2); lcd1.print("Lat:"); lcd1.print(ldg0);lcd1.print(" ");lcd1.print(lmin0); lcd1.print(" "); lcd1.print(lsec0,1); //
                      lcd1.print('N');//Latitude:deg,min,sec & North
        lcd1.setCursor(0,3); lcd1.print("Long:"); lcd1.print(ldg1);lcd1.print(" ");lcd1.print(lmin1); lcd1.print(" ");lcd1.print(lsec1,1); //
                   lcd1.print('E');
      //....................................end of lcd1 statement..........
      //..........................................................................................................................
      //Serial.println(gps.altitude() );   // Altitude in meters
      //GUI_DrawRectangle(20-1, 65,  460,65+13 , YELLOW, DRAW_FULL, DOT_PIXEL_DFT);  // earlier, erase 440x13 rect.
      // print unconditionally, when LnNo==0. print date,only if it has changed since the last sample time
      if (Dte != DteL && LnNo >= 1) {
        GUI_DrawRectangle(20 - 1, 65, 20 + 24, 65 + 13, YELLOW, DRAW_FULL, DOT_PIXEL_DFT);  // erase 2 digits rect. 24x13
        GUI_DisNum(20, 65, Dte, &Font16, YELLOW, BLUE);
        GUI_DisString_EN(45, 65, "/", &Font16, YELLOW, BLUE);
        //-----------------------------printing
        //lcd1.setCursor(3, 0); lcd1.print(Dte); lcd1.print("/");  // date
      }  //  date moved to (4,0)dd/
      //
      if (Mn != MnL && LnNo >= 1) {
        GUI_DrawRectangle(57 - 1, 65, 57 + 24, 65 + 13, YELLOW, DRAW_FULL, DOT_PIXEL_DFT);  // erase 2 digits
        GUI_DisNum(57, 65, Mn, &Font16, YELLOW, BLUE);
        GUI_DisString_EN(82, 65, "/", &Font16, YELLOW, BLUE);
        //------------------------------month 'Mn'--moved to (7,0)
        //lcd1.setCursor(6, 0); lcd1.print(Mn); lcd1.print("/"); // month
      }  // mm/

      if (Yr != YrL && LnNo >= 1) {
        GUI_DrawRectangle(94 - 1, 65, 94 + 48, 65 + 13, YELLOW, DRAW_FULL, DOT_PIXEL_DFT);  // erase 4 digits of year
        //------------------------------printing of year halted temporarily 21/oct/2022 (on 20x4 LCD)
        GUI_DisNum(94, 65, Yr, &Font16, YELLOW, BLUE);
      }  //printing of year temporarily stopped [ lcd1.setCursor(6, 0); lcd1.print(Yr);] // yyyy
      if (Hr != HrL && LnNo >= 1) {
        GUI_DrawRectangle(160 - 1, 65, 160 + 24, 65 + 13, YELLOW, DRAW_FULL, DOT_PIXEL_DFT);  // erase 2 digits
        GUI_DisNum(160, 65, Hr, &Font16, YELLOW, BLUE);
        GUI_DisString_EN(185, 65, ":", &Font16, YELLOW, BLUE);
        //lcd1.setCursor(11, 0); lcd1.print(Hr); lcd1.print(":");  // Hour
      }  //hh:

      if (mint != mintL && LnNo >= 1) {
        GUI_DrawRectangle(197 - 1, 65, 197 + 24, 65 + 13, YELLOW, DRAW_FULL, DOT_PIXEL_DFT);  // erase 2 digits
        GUI_DisNum(197, 65, mint, &Font16, YELLOW, BLUE);
        GUI_DisString_EN(222, 65, ":", &Font16, YELLOW, BLUE);
        // lcd1.setCursor(14, 0); lcd1.print(mint); lcd1.print(":");
      }  // mm:

      if (Scnd != ScndL && LnNo >= 1) {
        GUI_DrawRectangle(234 - 1, 65, 234 + 24, 65 + 13, YELLOW, DRAW_FULL, DOT_PIXEL_DFT);  // erase 2 digits
        GUI_DisNum(234, 65, Scnd, &Font16, YELLOW, BLUE);                                     // lcd1.setCursor(17, 0); lcd1.print(Scnd); lcd1.print(" ");
      }                                                                                       // ss ('blank' at (19,0)

      if (LnNo == 0) {
        GUI_DisNum(20, 65, Dte, &Font16, YELLOW, BLUE);
        GUI_DisString_EN(45, 65, "/", &Font16, YELLOW, BLUE);  // date
        GUI_DisNum(57, 65, Mn, &Font16, YELLOW, BLUE);
        GUI_DisString_EN(82, 65, "/", &Font16, YELLOW, BLUE);  // month
        GUI_DisNum(94, 65, Yr, &Font16, YELLOW, BLUE);
        GUI_DisNum(160, 65, Hr, &Font16, YELLOW, BLUE);
        GUI_DisString_EN(185, 65, ":", &Font16, YELLOW, BLUE);  // year & hours
        GUI_DisNum(197, 65, mint, &Font16, YELLOW, BLUE);
        GUI_DisString_EN(222, 65, ":", &Font16, YELLOW, BLUE);  // minutes (Seconds is automatic)
                                                                /*
        lcd1.setCursor(2, 0); lcd1.print(" "); lcd1.print(Dte); lcd1.print("/"); lcd1.print(Mn); lcd1.print("/"); lcd1.print("  "); // dd/mm/yyyy[], lcd1.print(Yr); skip year)
        lcd1.setCursor(10, 0); lcd1.print(" "); lcd1.print(Hr); lcd1.print(":"); //  lcd1.setCursor(14, 0);
        lcd1.print(mint); lcd1.print(":");  lcd1.print(Scnd); lcd1.print(" ");      //Hours  'lcd1.setCursor(17, 0);'--because hr,minute & sec. can be 1digit
        //-----------now Lattitude & Longitude-------------------------------------------
        lcd1.setCursor(0,1); lcd1.print("Latt:"); lcd1.print(ldg0);lcd1.print(" ");lcd1.print(lmin0); lcd1.print(" ");lcd1.print(lsec0,1); //Latt:deg,min,sec
        lcd1.setCursor(0,2); lcd1.print("Long:"); lcd1.print(ldg1);lcd1.print(" ");lcd1.print(lmin1); lcd1.print(" ");lcd1.print(lsec1,1); //Long:deg,min,sec
           
        //........................................................................................
      }  // all date,month,year, hours,minutes,Seconds done
      LnNo++;
      tLn++;
      if (tLn >= 5) {
        tLn = 0;  // print sr no. of line
        Serial.println(" ");
        Serial.print(LnNo);
        Serial.print(" ");
      } else {
        Serial.print(tLn);
        Serial.print(" ");
      }

      DteL = Dte;
      MnL = Mn;
      YrL = Yr;
      HrL = Hr;
      mintL = mint;
      ScndL = Scnd;  // update 'Last' values
      //  ----- Try Altitude----

      // Next: Altitude
      GUI_DisString_EN(265, 65, "Alt.", &Font16, YELLOW, BLUE);                         //
      GUI_DrawRectangle(330, 65, 330 + 75, 65 + 16, YELLOW, DRAW_FULL, DOT_PIXEL_DFT);  //
      GUI_DisNum(330, 65, LAltit, &Font16, YELLOW, BLUE);                               //  some error in Altitude
      //print_float(gps.f_altitude(),TinyGPS::GPS_INVALID_F_ALTITUDE,8,2);
    }  //}3
       // entry_fnc_G();  // show Latt,Long,Date,time,Altitude
  }    // 2}
  //}   // 1.5
  //Kbkaz=0;  //  action on a key 'a~z' has been taken
}  //  }1  end of Get_GPS
  */
// --- - - - - - - - - - - end of Get_GPS- - - - - - - - - - - --  - - - -- - - - - - - - -- -

//*******************************************************************************
//  function:
//        Draw Board (Actually,if the touch-Pad has been pressed select 1 of 5 colors or
//   find out as to which key has been pressed
//*******************************************************************************/
void TP_DrawBoard(void) {
}  // }1 end of function TP_DrawBoard

/*******************************************************************************
  function:
        Touch pad initialization
*******************************************************************************/