#include "LCD_Touch.h"  //---------AutoD_DSU8------------------
#include "Debug.h"
#include <EEPROM.h>
#include <LiquidCrystal.h>
#include <avr/pgmspace.h>  // <avr/pgmspace.h> is needed to define constants in 'Program memory' 8/march/2023
#include <arduino.h>
#include <stdlib.h>
#include <SD.h>
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
volatile float IeMag[] = { 0.4, 1.0, 2.00, 5.0, 10.0, 20.0 }, Ieval = 2.0, wgt1, wgt2, icnA1;  // Ieval= 2.0 mAmp
volatile char st1[wsl] = { "+000085.12  g" }, st2[wsl], st3[wsl], st4[wsl] = { "" }, st5[wsl], st6[wsl], st7[wsl], st8[wsl], st9[wsl], st10[wsl];
volatile char Blnk[10] = { "          " }, Fpr, N_dot = "No_dot", dot_symbol = N_dot, Y_dot = "dot";  // 'Q' means it is in 'Test' mode 10 blanks,Fpr=0 or 'F',or 'G'....'P' (11 nos.)
const char FName[] = { "SUV9.txt" };
volatile char FName2[10] = { "Srv3.csv" }, StrName1[20], StrName2[20], StrName3[20];
volatile unsigned int Noffp = 100, Noffn = 0, Npp = 500, Npm = 0;  //(say, Npp= 50, Noffset+,- Np =,- (for Gain.5 -- Noffp =155
const int SerD = 22, SClk = 23, Lat = 24, SyncP = 25, Wr2 = 27;    // Serial  data out -D22,Serial clock pin=D23,LatchPin=D24. SyncP for OscScope
volatile byte A2_Cntrl, A4_Dt, A4_Cntrl, A4_KAv, A4_Rl, A4_DAC;
const String inStr = "499.98", Str3, Str5 = "Vp1p2", Str6 = "Resist", Str7, Str11 = "Sur4.csv";
volatile String Str10, Str12, Str13, Str14, Str15 = ("Sur4.csv"), Str16("#"), Str17, Str18, Str19, Str20;                                         // volatile Strings, "Suv" -- Surve
volatile unsigned int Tsat = 0, NtRn1 = 5, NtRn2 = 5, ht1 = 13, EAd = 0, EAd1 = 0, EAd2, EAd3, EAd4, EAd5, EAd6, EAd7, EAd8, EAd9, EAd10, EAd11;  //Colr[];
volatile float Resist[20], yfct = 0.4, Vp1p2, Res, Resx, Resm, fNsig2, BattV, tRes1, fltLv, fltlv, tRohm;                   // yfct=100/Npls[0] (should be Npls[1]
volatile float tRes2[5];                                                                                                    //   tRes2[70];
volatile signed int Npls[5], Ndots[5], Ycoord[5], NdtxA = 160, Nsig = 20, Nsig2, Nbck = 150, Nbckdt = 100, Slp = 1, IeNo2;  // Slp - slope =1 means Vp1p2 changes by Slp every Sec.
extern volatile byte SD_ok, sh_sg = 0;                                                                                      //(rs, en, d4, d5, d6, d7);
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
volatile float fact1, fact2, fact3, fact4, fact5 = 0, tenf[] = { 1, 10, 100 };  // different factors of Resistance 'Resm'
volatile unsigned int StRd[] = { 1, 2, 3, 4, 5 }, StL[] = { 15, 30, 50, 100, 150 }, StRd1, StL1, StL2, NRec = 5;
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
volatile byte RtLD, Curr_Sw, Range_Sw, Cycl_Sw = 2, Meas_Sw;  // Cycl_Sw=2 means 4-cycle mode  rtLD is read from kkbin0~3
volatile byte tick1[8] = { 0, 1, 2, 0x14, 8, 0, 0 }, tick2[8] = { 0, 1, 3, 0x16, 0x1c, 8, 0 };
//-------------------------------Variables defined in sketch-GPS2, tab-GPS function -----------------------------------------------------------------------
volatile float Altit, lat, lon, ltg2, lsec2, lx2, frdg2, frmin2;  // Of these only lat,lon have been used presently
volatile float err0, err1, Rlsec0, Rlsec1;                        // err=(present)lsec0- (Reference ) Rlsec0. fAltit-Altitude(float)
volatile float frdg0, frdg1, lx0, lx1, frmin0, frmin1, lsec0, lsec1, ltg0, ltg1;
volatile unsigned int LnNo = 0, tLn = 0, Yr, YrL, EYr[4];
volatile byte Mn, Dte, Hr, mint, Scnd, EMn[4], EDte[4], EHr[4], Emint[4], EScnd[4], DeciSec, Cr, DteL, MnL, HrL, mintL, ScndL;  // E- extra
volatile unsigned long LAltit, f_age = 0, sz1, sz2, tsz1, tsz2;                                                                 // sz1,sz2 for file size, tsz1,tsz2-- total sizes
//....................................................................................
volatile unsigned int i1, i2, i3, i4, i5, EdSpc_No = 2;  // 'Edit_Spacing_Srl. No.
//....................................................................................
volatile float latitude, longitude;  // from
void A4_Init() {
  InitTimr();    // Timers 3,5,0 initialized
  interrupts();  //enable all global interrupts
  j2 = 0;
  j3 = 0;
  in_Byte = 0;  //j2- no. of bits,j3 - no.of bytes, D22 logic level is shifted into 'in_byte'
  A1ReqN = digitalRead(23);
  A1ReqO = A1ReqN;  // initialization -----no longer used after Sept 2022-------------------
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
  Srv_No = 8;
  LSpcN = 7;
  LRdSr = 9;  //these values should get redefined when the statements LSpcN2<-- E2prom[tEA] etc are executed (2~4 lines from here)
  tEA = EAd1;
  EEPROM.get(tEA, Srv_No);  // Survey no. gets updated in mode F4
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
  A1_Power();  // this defines: A1PowN. A1PowN==1mens A1 card is powered On
  // --------------------calculation of about float 100 Lv[100] & lv[100] values---19/july/2022 --------------------------------------------------
  for (j2 = 0; j2 <= LNomax - 1; j2++) {  //LNomax = 38 ,presently
    LNo[j2] = j2;
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
  for (j5 = 0; j5 <= LCNomax - 1; j5++) Kv[j5] = ScalcK(Lv[LCNo[j5]], lv[lCNo[j5]]);  // example:  LCNo= 0,1,2,3, 3,4,5 lCNo= 0,0,0,0, 1,1,1
  for (j5 = 0; j5 <= 22; j5++) Kv[j5] = ScalcK(Lv[LCNo[j5]], lv[lCNo[j5]]);  // example:  LCNo= 0,1,2,3, 3,4,5 lCNo= 0,0,0,0, 1,1,1
  del1();
  xv1 = 60;
  yv1 = 80;  // xv2=60,yv2=100 -initially
  del1();
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
  if (digitalRead(22) == HIGH) {
    pres_lvl = 1;  // -----Should be done only when +D is on--------------------
    last_lvl = pres_lvl;
  } else {
    pres_lvl = 0;
    last_lvl = pres_lvl;
  }
  Recv = 0;  // ignore received data
  Erase2();
  j3 = 0;  // j3= no. of bytes receied on 'Serial1' initialized to 0
  //  ---------------
  kpr = 2;
  Fpr = 'H';      // 8/Feb/2025
  entry_fnc_H();  // now defined at power-On: 'Survey' mode
  n16 = 0;
  E2prom_Lltbl(n16);  // actually,both Sclumberger & Wennersets are copied into EEPROM n16=0 means , it starts from EAd5 + 0
  lcd1.createChar(1, tick1);
  lcd1.createChar(2, tick2); 
}
// ............. ..................................end of A4_Init  ..............................


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
void Show_Recv_bytes() {
  for (i5 = 0; i5 >= 14; i5++) Serial.write(RcBf_R1[i5]);
}
void printDir(File dir, unsigned int ntb) { }
void E2prom_Lltbl(unsigned int n1) {}
void Updt_DigInpLvls() {  // vr2 = strtof(st1);  // just for testing 'strtof'
}
//-----------------------------calculate Batt. Voltage--(7/Sep/2022---------------------------------------------------------------
void calc_Batt() {
  //ln8 = (RcBf_R1[6] * 0x100) + RcBf_R1[5];
  BattV = (float)ln8 / 100.0;
  dtostrf(BattV, 5, 2, st1);                                                         // Batt Volt e.g. 12.83, RcBf_R1[7] expected to be 0
   lcd1.setCursor(8, 0);
  lcd1.print("BR=");
  lcd1.print(BattV, 2);  // now,'Sigma',1lcd1.clear();show at (13,0) B=12.68
  lcd1.setCursor(0, 3);
  lcd1.print("                   ");  // erase line-3 (press measure)
                                      //-----------------------show switch positions, only if RcBf_R1==3,i.e.Rannge switch pos. is on 'Bat'---------------------
  Curr_Sw = RcBf_R1[2];
  Range_Sw = RcBf_R1[3];
  Cycl_Sw = RcBf_R1[4];
  if (RcBf_R1[3] == 1)  // Range Switch on 'Batt' posn.
   {
    if (Range_Sw == 1) lcd1.print(" Batt    ");  //Line-1: Range switch
  }
}
void Show_LlK(void) {} // cannot remove 
void Recv_Serial2() {  //j3a=0 to be done at initialization time          xv1=60 & yv1==80 initially
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
    n15 = ch10 & 0x0F;
    if (n15 <= 9) ch6 = 0x30 + n15;
    else ch6 = 0x41 + (n15 - 10);                            // calculate 1st nibble (lower nibble only) (later try st3=String (n14,HEX);)
   
    n15 = (ch10 & 0xF0) >> 4;
    if (n15 <= 9) ch8 = 0x30 + n15;
    else ch8 = 0x41 + (n15 - 10);                        //calculate 2nd nibble (ch8)
    if (RcBf_R1[tn1] == 0xFF && RcBf_R1[tn1 - 1] == 0xFF) {
      xv1 += 20;  
      xv1 += 20;    
      xv1 += 20;
      xv1 = 60;
      yv1 += 12;
      tn1++;  // // show resistance
      if (tn1 == 15) {
        E2prom_put();  // store RcBf_R1 [0~15 byttes in EEPROM]
        calc_Batt();
        if (Range_Sw != 1 && Fpr != 'Q') Show_LlK();
      }  // do not call Show_LlK ,if in Batt position,, or in 'Test'mode. show 15 bytes, expected to be 01,0Fh,.....upto FFh,FFh
 
    }    //end of 'if 2 bytes=FFh,FFh
    // Removed---calc_Res();

    else {
      xv1 += 20;
      tn1++;
    }  // if (t_transf==1) tn2++;  at Ln ~ 1475
    
    //  ---- 2 nibbles  over -------

     }  // end of 'if Serial2 available
}
void Show_ResistData() {
}
float WcalcK(float av) {
  float vr1, vr2;
  vr2 = 2 * 3.1416 * av;  // 2 * Pye * av
  return vr2;
}
float ScalcK(float Lv, float lv) {
  float vr1, vr2;

  vr1 = Lv / lv;
  vr2 = ((3.1416 * lv) / 2) * (vr1 * vr1 - 1);
  return vr2;
}
float DipcalcK(unsigned int a, unsigned int n) {
  float vr1;

  vr1 = (3.1416 * n * (n + 1) * (n + 2)) * a;  // pye*n*n+1*n+2 * a
  return vr1;
}
void Erase1(void) {
}
void Erase2(void) {
}
void get_Hex(byte x) {
  n15 = x & 0x0F;
  if (n15 <= 9) ch6 = 0x30 + n15;
  else ch6 = 0x41 + (n15 - 10);  // calculate 1st nibble (lower nibble only) (later try st3=String (n14,HEX);)
  n15 = (x & 0xF0) >> 4;
  if (n15 <= 9) ch8 = 0x30 + n15;
  else ch8 = 0x41 + (n15 - 10);  //calculate 2nd nibble (ch8)
}
void show_ByRcvd() {
}
void Updt_RecD(void) {  //1.1{
}

void A1_Power() {
}
// .................................................end of fnc_I8...............................
//-----------------------entry_fnc_M--------------------------------------------------------
//---------------------------------------------------------------------------------------------
void entry_fnc_H() {  // Survey mode
}
ISR(TIMER3_COMPA_vect) {
  volatile byte k3;  //  1{  k3=column no.
  RtLD = 0;
  if (digitalRead(Kbin0) == LOW) RtLD |= 0x10;
  if (digitalRead(Kbin1) == LOW) RtLD |= 0x20;
  if (digitalRead(Kbin2) == LOW) RtLD |= 0x40;
  if (digitalRead(Kbin3) == LOW) RtLD |= 0x80;  // RtLD[bits 7,6,5,4]=Kbin0/1/2/3
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

  k6++;
  if (k6 >= 4) k6 = 0;
  digitalWrite(otpin0, HIGH);
  digitalWrite(otpin1, HIGH);
  digitalWrite(otpin2, HIGH);
  digitalWrite(otpin3, HIGH);  //digitalWrite(24,HIGH);
  if (k6 == 0) digitalWrite(otpin0, LOW);
  if (k6 == 1) digitalWrite(otpin1, LOW);
  if (k6 == 2) digitalWrite(otpin2, LOW);
  if (k6 == 3) digitalWrite(otpin3, LOW);
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
  if (timr7_flag == 1) timr7++;  // count only while timr7_flag == 1  .timer7=100 means 1 Second has elapsed
  if (timr7 >= 100) {
    digitalWrite(27, HIGH);
    timr7 = 0;
    timr7_flag = 0;
  }  // turn off CRM-Auto_D main instrument
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
      // .................................end of rdng. from SD................
    }  // 4th line,20th col.if (timr3old=80 & timr3=81)(2 Sec) show 1

    if (tm4old <= tlim4 && tm4 > tlim4) {  //tm4 crosses tlim4=150
      lcd1.setCursor(19, 3);
      lcd1.write(0xA0);  // write 'blank'
      lcd1.setCursor(12, 1);
      lcd1.noCursor();
    }  // 4th line,20th col{if (timr3old=240 & timr3=241)(6 Sec) show 2}

    tm4old = tm4;
    timr6old = timr6;  // update tm4old & timr6old

  }
  timr5old = timr5;  // update timr5olduuuuu
}
ISR(TIMER5_COMPA_vect)  // Interrupt when Timer5 count reaches 60,000 (62500/65536)=0.953 Second
{
  timr5++;
  if (timr5 >= 3) {
    timr5 = 0;
    digitalWrite(24, HIGH);
  }
}
ISR(TIMER0_COMPA_vect)  // Interrupt when Timer0reaches 250 ( was 20 earlier)              //  ISR(TIMER0_OVF_VECT)
{
  nby8 += 1;  // add 1 to the upper byte nby8
}
ISR(TIMER4_COMPA_vect)  // Interrupt when Timer0reaches 250 ( was 20 earlier)              //  ISR(TIMER0_OVF_VECT)
{
  n5 = TCNT5;  // Frq+
  nby7 = TCNT0;
  n7 = (nby8 * 250 + nby7);  //Frq-
}
void A4_D1_DAC(byte byt1)
{
 }
byte RevBits(byte Num) {
}
void del1() {
  unsigned long i10, k1;
  for (i10 = 1; i10 < 200000; i10++) k1 = k1 + 1;  // delay with dummy k1++
}
