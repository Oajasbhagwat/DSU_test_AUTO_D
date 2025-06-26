


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



// ---------------------------------------------------
// --    draw keyboard  a~z or A~Z
// ......................................................
// 
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
  //......................................................................
  //---------------------------------------------------------------L
  tlim2 = 800 / 4;
  tlim3 = (tlim2 + 1) / 4;
  tlim4 = 3 * tlim3;  // tlim2,3,4 = 200,50,150  used in Timer3 10 mSec interrupt
  //Str10  = "Suv"; SLRdSrtr10+= ".csv";
  // ----------- 1-time ---- initialization done ---- All this gets overridden ~ 50 lines later
  IntSz = 2;
  FltSz = 4;  //IntSz = sizeof(int); FltSz = sizeof (float);
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
  
  Srv_No = 8;
  LSpcN = 7;
  LRdSr = 9;  //these values should get redefined when the statements LSpcN2<-- E2prom[tEA] etc are executed (2~4 lines from here)
  // EAd1~2: 20 integers, EAD2~3: 20 Float nos., EAD3~4: 20 integers// --------------- Get Survey no. from EEPROM  ------------------
  //delay(1000);
  // -------------read from E2PROM -------------------
  
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
  // --------------------calculation of about float 100 Lv[100] & lv[100] values---19/july/2022 --------------------------------------------------
  for (j2 = 0; j2 <= LNomax - 1; j2++) {  //LNomax = 38 ,presently
    LNo[j2] = j2;
    // --- printing suppressed----
    
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
    vr3 = lv[j2] * 18.0;
    if ((Lv[LNo[j3]] < vr3) && (vr3 <= Lv[LNo[j3 + 1]])) {  // one 'chpts' (change point detected)
      j2++;
      j4++;
      j5 = j3 + j2;
      LCNo[j5] = LNo[j3];
      lCNo[j5] = j2;  // ( j5 has changed, j2 (which was 0 originally,has changed, so j5=j3+j2 changes too

      // --- printing suppressed----
     
    }  // end of 'j5 has changed'
  }    // end of j3 loop
  LCNomax = j5 + 1;
  lCNomax = j2 + 1;  // now,LCNomax overrides the value declared in variable list. if LNomax=47, then LCNomax= ~47+8=55
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
  for (m7 = 0; m7 <= 8; m7++) FName2[m7] = Str11[m7];
  //lcd1.setCursor(0, 2);lcd1.print("FNme");lcd1.print(Str11); // copy const string 'Str11' into const 'char' array
  //-------show new names ----------------------------------------------
                                                       //   */
                                                       //---------------------------------------------------------------
  del1();
  //--------------------------------------------------------------------


  //..................................................end of 'define Survey file...............................
  //---------------------------------write  into 'SD' . ---- Then read back & show it----------------------------
  xv1 = 60;
  yv1 = 80;  // xv2=60,yv2=100 -initially
             // ---------------------------------------First, write data, '1.5,0.5'
             //----------------------------------------------------------------------
  del1();
  //......................................................................
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
   lcd1.setCursor(10, 0);
  lcd1.print("BY=");
  lcd1.print(BattV, 2);  // now,'Sigma',1lcd1.clear();show at (13,0) B=12.68
  lcd1.setCursor(0, 3);
  lcd1.print("                   ");  // erase line-3 (press measure)
                                      //-----------------------show switch positions, only if RcBf_R1==3,i.e.Rannge switch pos. is on 'Bat'---------------------
  Curr_Sw = RcBf_R1[2];
  Range_Sw = RcBf_R1[3];
  Cycl_Sw = RcBf_R1[4];
  if (RcBf_R1[3] == 1)  // Range Switch on 'Batt' posn.
   {
    if (Range_Sw == 1) lcd1.print("Bat      ");  //Line-1: Range switch
    if (Range_Sw >= 2 && Range_Sw <= 4) lcd1.print("Resist");
    //if (Range_Sw == 5 || Range_Sw == 6) lcd1.print("S.P.");
    if (Range_Sw >= 7) lcd1.print("Error");
    lcd1.setCursor(0, 2);
                                
  }
}
//.........................................end of calcul. Batt. Volt..............................................................
//----------------------------------copied from Ketch SimCrm1------ ~90 lines----------------------------------------
//--------------------------------------------------------------
//     show Status of Current
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
  
    lcd1.clear();
    
    dtostrf(Kv[LSpcN2], 7, 2, st1);
    
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
                                                             
    Ldig1 = Lint1 % 10;                                      //ldig1 = lint1 % 10;
    if (Ldig1 == 0) Lint3 = Lint1 / 10;                      //if (ldig1 == 0) lint3 = lint1 / 10; // if Ldig1==0 we use integer division, otherwise we use 'fltLv' a float value
                                                             //GUI_DisNum (40,130, Lint1, &Font12, Colr[7],BLUE); GUI_DisNum (60,130, lint1, &Font12, Colr[7],BLUE);  //
    lcd1.setCursor(0, 0);
    lcd1.print("F2");
    lcd1.print(" Rd");
    lcd1.print(LRdSr2 + 1);
    lcd1.print("  Sp");
    if (freezeSP == 0) lcd1.print(LSpcN2 + 1);
    else lcd1.print(LSpcN2);  // //F6h is 'Sigma',4,Rd,Sp
                              //'Sigma'4,Reading no. & Spacing no (1~N numbering)

    
  }
  //.......................................Wenner--end............................
}
//.....................................................................end of 'Show_LlK' .............................
//---------------------------------------------------------------------------------------------calculate Resistance----------

void calc_Res() {
  ln8 = ((long)RcBf_R1[25] * (long)0x10000) + (long)RcBf_R1[24] * (long)0x100 + (long)RcBf_R1[23];
  dtostrf(fact4, 8, 5, st1);
  }
  //   //. . . . . . . . . . . . . . . . . .end of cycl-4 . . . . . . . . . . . . . . . . . . . . . . . . .

  // }  // end of 'if '4-Cycle mode'
  //}  //..........................................end of 'calc_Res'............................................
   //--------------------------------------select print format-----?----------------

// ------------------------------show data received from (now,(7/Sept/2022) from Sketch DSU2)(Earlier,Auto_D, on Serial2 channel)-----------------------------
void Recv_Serial2() {  //j3a=0 to be done at initialization time          xv1=60 & yv1==80 initially
  // from ~ 1/August/2022 onward, variable names are changed from j2,j3 to j2a,j3a (j2a--no. of bits received so far.Similarly, j3a--no. of bytes received so far


  while (Serial2.available() > 0) {  // data expected from Serial-2 channel (Serial-1 is for GPS module) as of 8/August/2022)

    ch10 = Serial2.read();  //Recv_Buff1[j3a] = ch10; j3a++;

    if (xv1 >= 400) {
      yv1 += 16;
      xv1 = 0;
    }
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
   // GUI_DisChar(xv1 + 8, yv1, ch6, &Font12, Colr[7], BLUE);  // show 1st nibble (ch6)

    n15 = (ch10 & 0xF0) >> 4;
    if (n15 <= 9) ch8 = 0x30 + n15;
    else ch8 = 0x41 + (n15 - 10);                        //calculate 2nd nibble (ch8)
    //GUI_DisChar(xv1, yv1, ch8, &Font12, Colr[7], BLUE);  //  show 2nd nibble(ch8), to the left of 1st byte
    //  }
    //show char. received on serial monitor ------------n 22/nov/2022: print j3a too----------------------
    //----------------write ch10-on lcd1 at (m13,1) 7 advance m13-------------------------------------------------------------------------------------------
    //lcd1.setCursor(m13,1); lcd1.print(ch10); m13++; j3ac

    //..........................................................................................................
    if (RcBf_R1[tn1] == 0xFF && RcBf_R1[tn1 - 1] == 0xFF) {
      xv1 += 20;
      //GUI_DisNum(xv1, yv1, tn1, &Font12, Colr[7], BLACK);
      xv1 += 20;
      //GUI_DisNum(xv1, yv1, tn2 + 1, &Font12, Colr[7], BROWN);  //tn1& tn2+1
      xv1 += 20;
      //GUI_DisNum(xv1, yv1, t_transf + 1, &Font12, Colr[7], BLUE);
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
 
    }    //end of 'if 2 bytes=FFh,FFh
    // GUI_DisString_EN (100, 90+90 , &st1[0], &Font12, Colr[7], BLUE);}
    // Removed---calc_Res();

    else {
      xv1 += 20;
      tn1++;
    }  // if (t_transf==1) tn2++;  at Ln ~ 1475
    //if (tn1>=2 && ch10==0xFF && Ch10old==0xFF) {xv1=60;yv1+=16; ch10old=ch10;}
    //  ---- 2 nibbles  over -------

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
}
// .........................................................end of Show_Eprom2 ...copied on .9/Sept/2022.......................
// ---------------------------------------Show_Epro3 (modified from Show_Eprom2)------------------------------------------
// show L,l stored in E2prom (Lint1, lint1)
//-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-..-.-.-.-...-..-.-..-..-..--.---.-.--.-.-.--.--.---------------------
void Show_Eprom3(unsigned int L, unsigned int l) {
  }
//................................................................end of Show_Eprom3
// show status of 4 keyboard pins Note: new name:-- 'Kb_Action'
//----------------------------------------------------------------------------
void Kb_Action() {
  
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
  //lcd1.setCursor(0, 0);
  //lcd1.print("F2");  // F2'Sigma',1
  //lcd1.setCursor(3, 0);
  //lcd1.print("Rd ");
  // lcd1.print(LRdSr2 + 1);
  // if (Surv_meth == 1) lcd1.print(" ");
  // if (Surv_meth == 2) lcd1.print(" ");
  // lcd1.print(" Sp=");
  // if (freezeSP == 0) lcd1.print(LSpcN2 + 1);
  // else lcd1.print(LSpcN2);
  // if ((LSpcN2 + 1) <= 9) lcd1.print(" ");  // line-0, reading no.,Spacing no. .
  lcd1.setCursor(12, 0);
  lcd1.print(" Srv ");
  lcd1.print(Srv_No);  //
  lcd1.setCursor(10, 2);
  //if (Surv_meth == 1) lcd1.print(" Schlumb ");
  //if (Surv_meth == 2) lcd1.print(" Wenner ");
  //if (Surv_meth == 3) lcd1.print("Dip-Dipo ");  // show Schlum/wenner/Dipo-Dipo at line-2, column-10
  //lcd1.print("Sp2=");                           // why  SP & SP2 ?
  //if (freezeSP == 0) Show_LlK2(LSpcN2);
  //else Show_LlK2(LSpcN2 - 1);
  // lcd1.setCursor(0, 2);
  // lcd1.print("K= ?");  //
  //                      // Sp++  blocked by key 2 only & enabled by key 3 only
  // lcd1.setCursor(1, 3);
  // lcd1.print("-press Measur(9)");  // this message "-press Measure-" wil get erased when 'Batt' voltage is received

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
