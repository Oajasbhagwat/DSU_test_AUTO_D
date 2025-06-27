extern unsigned long timr3, timr4;  // declared in LCD_Touch.c++
#include "DEV_Config.h"
#include "LCD_Driver.h"
#include "LCD_GUI.h"
#include "LCD_Touch.h"
void setup() {  
  const int Tch_Screen_SD_Cs = 53;  //  we have used SDC_CS_PIN=53 (line no.71) should it be 5, (i.e. D5)(for Touch Screen LCD)
  digitalWrite(42, HIGH);           // this will ensure that flip-flop u1-A(4013) remains on & +7.5 V remains on. if we wish to turn off DSU, we should make D42=0
  Serial1.begin(9600, SERIAL_8N1);  // for GPS module ,9/August/2022
  Serial2.begin(4800, SERIAL_8N2);  // Now,(5/Sept/2022)receives Resistance,Batt etc.on DSU2_Rx2 But transmits any data to SimCrm1_Tx2(Earlier,for receiving data from SimCrm1 9/August/2022
  System_Init();
  pinMode(SDC_CS_PIN, OUTPUT);  // this is pin 53
  pinMode(22, INPUT_PULLUP);
  digitalWrite(22, HIGH);
  pinMode(23, INPUT_PULLUP);
  digitalWrite(23, HIGH);  //D22,D23 & D26 are 'input' pins is permanent.
  pinMode(26, INPUT_PULLUP);
  pinMode(40, INPUT_PULLUP); pinMode(28, INPUT_PULLUP);pinMode(29, INPUT_PULLUP);  //
  pinMode(24, OUTPUT);
  digitalWrite(24, HIGH);
  pinMode(25, OUTPUT);
  digitalWrite(25, HIGH);  // D24, D25 are ouputs,initially '0' become '1'at U7,8255 pins
  pinMode(42, OUTPUT);
  digitalWrite(42, HIGH);  // to keep power to Arduino on
  pinMode(27, OUTPUT);
  digitalWrite(27, HIGH);  // D27 is made '0' for ~200 mSec. to turn A1 card on
  Serial.begin(9600, SERIAL_8N1);  //for Serial monitor (LapTop)
  Serial.println("examining DSU data");
  LCD_SCAN_DIR Lcd_ScanDir = SCAN_DIR_DFT;  //SCAN_DIR_DFT = D2U_L2R
  LCD_Init(Lcd_ScanDir, 200);
  del1();
  InitTimr();
  interrupts();       // This is moved to A4_Init enable all global interrup
  lcd1.begin(20, 4);  //  20, 4): 20 chars.(0~19)x 4 lines(0~3)
  lcd1.setCursor(0, 0);
  lcd1.print("Anvic systems ");
  Serial.println("SR11----");
 Serial1.begin(9600);
Serial.println("SR12----");
  const int SDC_CS_PIN = 53;
  Serial.println("SR13---");
       SD_ok==1; Serial.println("---SD 1.2 forced to OK -----");
       lcd1.setCursor(0, 0);
  lcd1.print("Anvic systems ");
  pinMode(26, INPUT_PULLUP);  // this pin goes Low when Measure switch is pressed

  A4_Init();  // ( one-time initialization
  pinMode(otpin0, OUTPUT);
  pinMode(otpin1, OUTPUT);  // out (3,2,1,0) 1110
  pinMode(otpin2, OUTPUT);
  pinMode(otpin3, OUTPUT);

  pinMode(Kbin0, INPUT_PULLUP);
  pinMode(Kbin1, INPUT_PULLUP);
  pinMode(Kbin2, INPUT_PULLUP);
  pinMode(Kbin3, INPUT_PULLUP);
     // actually,The next 4 lin are not needed 
  
}  // end of 'setup()

void loop() {
   i11++;
  if (i11 >= 100000) {
    i11 = 0;
    i13++;
  }

  Recv_Serial2();     //    //  void receive data from A1-card
  Led += 1;  // ?
  }  // end of 'loop'
void InitTimr(void) {
  TCNT5 = 0;  // Timer count (16-bit) <-- 0
  TCCR5A = 0;
  TCCR5B = 0;                                         // Initializz to 0
  TCCR5B |= (1 << CS52) | (1 << CS51) | (1 << CS50);  // bits[2,1,0]=111 Select external clock (rising edge  )
  TCCR5B |= (1 << WGM52);                             // bits [3,2,1,0] = 0100 means 'select CTC mode ( 16-bit Timer/counter ). thus timer value will be 0 ~ FFFFh
  OCR5A = 62500;                                      //(         presently 'Compare match A interrupt is not being used)
   TIMSK5 = 02;
  TCNT0 = 0;  // Timer count (8-bit) <-- 0
  TCCR0A = 0;
  TCCR0B = 0;                                         // Initializz to 0
  TCCR0A |= (1 << WGM01);                             // turn on CTC (Clear timer0 on conpare natch) mode
  TCCR0B |= (1 << CS02) | (1 << CS01) | (1 << CS00);  // bits[2,1,0]=111 Select external clock (rising edge  )
  OCR0A = 250;                                        // 'Compare Regiser ='250' (0~249) ( it was 20 earlier )
  TIFR0 = 0;                                          // initialze
  TIFR0 |= (1 << OCF0A);                              // clear any pending interrupts
  TIMSK0 = 02;                                        // bit[1] = 1 means, 'timer- compare match A' interrupt is enabled
  TCCR3A = 0;
  TCCR3B = 0;
  TCNT3 = 0;
  TCCR3B |= (1 << WGM32) | (1 << CS32);  //|(1<<CS30);   // WGM3{3~0]= 0100b (=4) means CTC(Clear timer on Compare) mode,
  TIMSK3 |= (1 << OCIE3A);               // Compare Match A interrupt is enabled (it is understood that TIMSK3 is originally 0)
  OCR3A = 625;  // This will generate an interrupt every 10 mSec ( for 62,500 pulses per Sec.
  TCNT3 = 0;
  TIFR3 |= (1 << OCF3A);  // clear any pending interrupts
  TIMSK3 = 02;  // Compare Match A interrupt is enabled 
  TCCR4A = 0;
  TCCR4B = 0;
  TCNT4 = 0;
  TCCR4B |= (1 << WGM42) | (1 << CS42);  //|(1<<CS40);   // WGM3{3~0]= 0100b (=4) means CTC(Clear timer on Compare) mode,
  TIMSK4 |= (1 << OCIE4A);               // Compare Match A interrupt is enabled (it is understood that TIMSK3 is originally 0)
  // .........................................................
  OCR4A = 62500;  // This will generate an interrupt every 1  Sec ( for a frequency of 62,500 pulses per Sec.
  TCNT4 = 0;
  TIFR4 |= (1 << OCF4A);  // clear any pending interrupts
  TIMSK4 = 02;  // Compare Match A interrupt is enabled 
}
