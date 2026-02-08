// 6 Channel Transmitter | 6 Kanal Verici
// KendinYap Channel

#include "main.h"
#include <SPI.h>
#include <EEPROM.h>
#include <U8g2lib.h>
//#include <Wire.h>
#include "display.h"
//#include "expo.h"
#include "expo8.h"
#include <nRF24L01.h>
#include <RF24.h>
#include <Bounce2.h> // github.com/thomasfredericks/Bounce2
#include <avr/io.h>

#include <elapsedMillis.h>
#include "defines.h"

#include "MS5611.h"

const uint64_t pipeOut = 0xABCDABCD71LL;         // NOTE: The address in the Transmitter and Receiver code must be the same "0xABCDABCD71LL" | Verici ve Alıcı kodundaki adres aynı olmalıdır
extern "C" 


// github.com/olikraus/u8g2/discussions/1865

// 0.96"
// >> code in display.h

uint16_t loopcounter0 = 0;
uint16_t loopcounter1 = 0;

#define RAMPETEST    0
#define TEST         0
#define CE_PIN       9
#define CSN_PIN      10

// instantiate an object for the nRF24L01 transceiver
RF24 radio(CE_PIN, CSN_PIN);

#define LOOPLED 4

#define MS_PIN       5

#define EEPROMTASTE  5

#define EEPROM_WRITE 0
#define EEPROM_READ  1

#define EEPROMINDEX_U 0x10
#define EEPROMINDEX_O 0x20
#define EEPROMINDEX_M 0x30

#define EEPROMLEVELSETTINGS  0x40
#define EEPROMEXPOSETTINGS  0x48


#define BLINKRATE 0xFF

// defines for PINS

uint8_t DIPA_Array[4] = {DIPA_0_PIN,DIPA_1_PIN,DIPA_2_PIN,DIPA_3_PIN};




#define TASTATUR_PIN A7
#define TASTE_OFF  0
#define TASTE_ON  1


#define BATT_PIN         A6

uint8_t debouncecheck = 0;

uint16_t loopcounter = 0;
uint8_t blinkcounter = 0;
uint8_t impulscounter = 0;
uint16_t throttlecounter = 0;
uint16_t throttlesekunden = 0;

// RC_22
//#define POT0LO 620  // Min wert vom ADC Pot 0
//#define POT0HI 3400 // Max wert vom ADC Pot 0

#define POTLO   0
#define POTHI  710

//Impulslaenge, ms
#define PPMLO  850  // Minwert ms fuer Impulslaenge
#define PPMHI  2150 // Maxwert ms fur Impulslaenge

#define MINDIFF 4

const uint64_t pipeIn = 0xABCDABCD71LL;

uint16_t schritt = 32;


uint8_t PCB_BOARD = 6;


uint16_t                   impulstimearray[NUM_LOKS] = {};


const int                  adcpinarray[NUM_LOKS] = {A0,A1};    // pins der Pots


//uint8_t                    kanalsettingarray[ANZAHLMODELLE][NUM_LOKS][KANALSETTINGBREITE] = {};

uint16_t                   servomittearray[NUM_LOKS] = {}; // Werte fuer Mitte

uint8_t levelwert= 0;
uint8_t levelwerta = 0;
uint8_t levelwertb = 0;

uint8_t levelwertarray[NUM_LOKS] = {}; // leelwert pro servo


uint8_t levelwertayaw = 0;
uint8_t levelwertbyaw = 0;

uint16_t potwertyaw = 0;


uint8_t expowert = 0;
uint8_t expowerta = 0;
uint8_t expowertb = 0;

uint8_t expowertarray[NUM_LOKS] = {}; // expowert pro Servo



uint16_t          potwertarray[NUM_LOKS] = {}; // Werte fuer Mitte
//uint16_t          externpotwertarray[NUM_LOKS] = {}; // Werte von extern  pro servo

//uint16_t currentexpoarray[5][513] = {};

uint8_t                        curr_pfeil = 0;

//uint16_t      blink_cursorpos=0xFFFF;


uint16_t stopsekunde=0;
uint16_t stopminute=0;
uint16_t motorsekunde=0;
uint16_t motorminute=0;
uint8_t motorstunde=0;

uint16_t sendesekunde=0;
uint16_t sendeminute=0;
uint8_t sendestunde=0;

// Status
uint8_t blinkstatus = 0;

uint8_t curr_steuerstatus = 0;

uint8_t calibstatus = 0;

uint8_t savestatus = 0;

uint8_t anzeigestatus = 0;



float potlo = POTLO; // min pot
float pothi = POTHI; // max pot
float ppmlo = PPMLO; // min ppm
float ppmhi = PPMHI; // max ppm

uint16_t diffa = 0;

uint16_t potwertpitch = 0;
uint16_t diffapitch = 0;
uint16_t diffbpitch = 0;

uint16_t diffb = 0;
float expofloat = 0;
uint16_t expoint = 0;
uint16_t levelint = 0;

uint16_t levelintraw = 0;

uint16_t levelintcheck = 0;


uint16_t levelintpitcha = 0;

uint16_t levelintpitchb = 0;
float batteriespannung = 0;
float batteriespannungraw = 0;

uint16_t batteriearray[8] = {};
uint16_t batteriemittel = 0;
uint8_t batteriemittelwertcounter = 0;
uint16_t batterieanzeige = 0;
float UBatt = 0;

float flyerbatteriespannung = 0;
float flyerbatteriespannungraw = 0;
uint16_t flyerbatterieanzeige = 0;
float UFlyerBatt = 0;



uint8_t temperaturint = 0;
float temperaturfloat = 0;


uint8_t eepromstatus = 0;
uint16_t eepromprelltimer = 0;
uint16_t intdiff = 0;
uint16_t intdiffpitch = 0;

//Bounce2::Button eepromtaste = Bounce2::Button();


#define ANZ_REP 8
uint16_t tastaturwert = 0;
uint16_t tastaturwertarray[ANZ_REP] = {};
uint8_t mittelposition = 0;

uint16_t winkelcounter = 0;
uint8_t rampe = 0;
int8_t ramprichtung = 1;

uint8_t tastencounter = 0;
uint8_t tastaturstatus = 0;
uint8_t Taste = 0;
uint8_t taste5counter = 0;



uint8_t taskarray[4] = {'Y', 'P', 'R', 'T'};


volatile uint8_t currentChannel = 0;
volatile uint8_t pausecounter = 0;
volatile bool pulseState = true;
volatile uint32_t elapsedFrame = 0;
volatile uint8_t ISRcounter = 0;


volatile uint16_t channels[NUM_LOKS] = 
{
   1500,1500
};

// V2
volatile uint16_t ppm[NUM_LOKS] = {1500,2000};

#define PPM_PIN 6
#define FRAME_LENGTH 32000  // µs
#define PULSE_LENGTH 300    // µs
#define CHANNEL_MIN 1000
#define CHANNEL_MAX 2000

//volatile uint8_t currentChannel = 0;
volatile uint16_t restTime = FRAME_LENGTH;








// OLED > in display.cpp
uint16_t pot0 = 0;

uint16_t potwert = 0;

uint16_t resetcounter = 0;
uint16_t ackcounter = 0;
uint16_t radiocounter = 0;
uint8_t radiostatus = 0;

// ********************
// ACK data ***********
uint8_t ackData[8] = {21,22,23,24,25,26,27,28};
// ********************
// ********************

// uint16_t                posregister[8][8]={}; // Aktueller screen: werte fuer page und daraufliegende col fuer Menueintraege (hex). geladen aus progmem

//uint16_t                cursorpos[8][8]={}; // Aktueller screen: werte fuer page und darauf liegende col fuer den cursor

unsigned char           char_x = 0;
unsigned char           char_y = 0;

// Menu
uint8_t                 curr_model=0; // aktuelles modell
uint8_t                 speichermodel=0;
uint8_t                 curr_funktion=0; // aktuelle funktion
uint8_t                 curr_aktion=0; // aktuelle aktion

uint8_t                  curr_wert = 0;

uint8_t                 curr_impuls=0; // aktueller impuls

uint8_t                 curr_modus=0; // Modell oder Sim oder Calib


uint8_t                 curr_setting=0; // aktuelles Setting fuer Modell
uint8_t                          speichersetting=0;

uint8_t                 curr_trimmkanal=0; // aktueller  Kanal fuerTrimmung
uint8_t                 curr_trimmung=0; // aktuelle  Trimmung fuer Trimmkanal


uint8_t                 curr_screen = 0; // aktueller screen
uint8_t                 last_screen=0; // letzter MASTERscreen

uint8_t                 curr_page=7; // aktuelle page
uint8_t                 curr_col=0; // aktuelle colonne

uint8_t                 curr_cursorzeile=0; // aktuelle zeile des cursors
uint8_t                 curr_cursorspalte=0; // aktuelle colonne des cursors
uint8_t                 last_cursorzeile=0; // letzte zeile des cursors
uint8_t                 last_cursorspalte=0; // letzte colonne des cursors

// Tastatur
uint8_t                 Tastenindex=0;
uint16_t                Tastenwert=0;
uint8_t                 adcswitch=0;
uint16_t                lastTastenwert=0;
int16_t                 Tastenwertdiff=0;
uint16_t                tastaturcounter=0;
uint16_t                tastaturdelaycounter=0;

elapsedMillis           zeitintervall;
uint8_t                 sekundencounter = 0;
elapsedMillis           sinceLastBlink = 0;

elapsedMillis           paketcounter;
elapsedMillis           sincelasttastatur;


uint16_t                impulsdelayarray[4] = {};
//uint16_t                restzeitarray[4] = {};
//uint16_t                impulsCCMParray[4] = {};
//uint16_t                restCCMParray[4] = {};
//uint16_t                restCCM = 0;
elapsedMillis        buzzintervall = 0;

int Border_Mapvar255(uint8_t servo, int val, int lower, int middle, int upper, bool reverse);


Signal data;

void ResetData() 
{
   data.task = 0;                  
   data.A = 0;
   data.B = 0;
   data.C = 0;
   resetcounter++;   
   
}


uint8_t debounceTaste()
{
   static  uint8_t debounced_state = 0;
   static uint16_t state = 0;
   state = (state<<1) | (tastaturwert>10) ;
   if(state >= 0xF00)
   {
      //debouncecheck = 1;
      return 1;
      
   }
   
   //debouncecheck = 0;
   return 2;
}


void setupDebounce()
{
   // GPT "nano_every timer interrupt 4ms "
   // Timer stoppen und auf Normal Mode konfigurieren
   TCA0.SINGLE.CTRLA = 0; // Timer aus
   
   // Periodenwert für 4 ms bei Prescaler 64
   TCA0.SINGLE.PER = 999; // Zählt von 0 bis 999 = 1000 Schritte
   
   // Compare Match Interrupt aktivieren
   TCA0.SINGLE.INTCTRL = TCA_SINGLE_OVF_bm; // Overflow Interrupt aktivieren
   
   // Prescaler = 64, Timer starten
   TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV64_gc | TCA_SINGLE_ENABLE_bm;
}

ISR(TCA0_OVF_vect) 
{
   // Hier passiert alle 4 ms etwas
   
   debouncecheck = debounceTaste();
   
  
   // Interrupt-Flag löschen
   TCA0.SINGLE.INTFLAGS = TCA_SINGLE_OVF_bm;
}




// PPM decode



void printgrenzen()
{
   //Serial.print("\nprintgrenzen\n");  
   for (uint8_t i = 0;i<NUM_LOKS;i++)
   {
      //Serial.print("grenzen i:\t");
      //Serial.print(i);
      //Serial.print("\t");
      //Serial.write(taskarray[i]);
      //Serial.print("\t");
      //Serial.print("potgrenze HI:\t");
      
      //Serial.print("\t");
      //Serial.print("servomitte:\t");
      //Serial.print(servomittearray[i]);
      //Serial.print("\n");
   }
   //Serial.print("end printgrenzen\n");  
}


void clearsettings(void)
{
   //Serial.print("clearsettings\n");  
   
}






uint8_t Joystick_Tastenwahl_33_2(uint16_t Tastaturwert)
{
   //return 0;
   if (Tastaturwert < JOYSTICKTASTE1) 
      return 2;
   if (Tastaturwert < JOYSTICKTASTE2)
      return 1;
   if (Tastaturwert < JOYSTICKTASTE3)
      return 4;
   if (Tastaturwert < JOYSTICKTASTE4)
      return 7;
   if (Tastaturwert < JOYSTICKTASTE5)
      return 8;
   if (Tastaturwert < JOYSTICKTASTE6)
      return 3;
   if (Tastaturwert < JOYSTICKTASTE7)
      return 6;
   if (Tastaturwert < JOYSTICKTASTE8)
      return 9;
   if (Tastaturwert < JOYSTICKTASTE9)
      return 5;
   /*
    if (Tastaturwert < JOYSTICKTASTEL)
    return 10;
    if (Tastaturwert < JOYSTICKTASTE0)
    return 0;
    if (Tastaturwert < JOYSTICKTASTER)
    return 12;
    */
   return 0;
}
uint8_t Joystick_Tastenwahl_33_tastatur(uint16_t Tastaturwert)
{
   //return 0;
   if (Tastaturwert < JOYSTICKTASTE1) 
      return 1;
   if (Tastaturwert < JOYSTICKTASTE2)
      return 2;
   if (Tastaturwert < JOYSTICKTASTE3)
      return 3;
   if (Tastaturwert < JOYSTICKTASTE4)
      return 4;
   if (Tastaturwert < JOYSTICKTASTE5)
      return 5;
   if (Tastaturwert < JOYSTICKTASTE6)
      return 6;
   if (Tastaturwert < JOYSTICKTASTE7)
      return 7;
   if (Tastaturwert < JOYSTICKTASTE8)
      return 8;
   if (Tastaturwert < JOYSTICKTASTE9)
      return 9;
   /*
    if (Tastaturwert < JOYSTICKTASTEL)
    return 10;
    if (Tastaturwert < JOYSTICKTASTE0)
    return 0;
    if (Tastaturwert < JOYSTICKTASTER)
    return 12;
    */
   return 0;
}

uint16_t readADC_A6() {
   // A6 entspricht intern AIN5
   ADC0.MUXPOS = ADC_MUXPOS_AIN5_gc;    // ← angepasst: AIN5 statt AIN4
   
   // kleine Wartezeit (Settling)
   for (volatile uint16_t i = 0; i < 200; ++i) {
      __asm__ volatile("nop");
   }
   
   // Start der Konversion
   ADC0.COMMAND = ADC_STCONV_bm;
   
   // Warten bis Resultat bereit
   while (!(ADC0.INTFLAGS & ADC_RESRDY_bm)) { }
   
   uint16_t result = ADC0.RES;
   ADC0.INTFLAGS = ADC_RESRDY_bm;
   
   return result;
}



uint8_t Joystick_Tastenwahl_33_6(uint16_t Tastaturwert)
{
   //return 0;
   if (Tastaturwert < JOYSTICKTASTE1) 
      return 2;
   if (Tastaturwert < JOYSTICKTASTE2)
      return 1;
   if (Tastaturwert < JOYSTICKTASTE3)
      return 4;
   if (Tastaturwert < JOYSTICKTASTE4)
      return 7;
   if (Tastaturwert < JOYSTICKTASTE5)
      return 8;
   if (Tastaturwert < JOYSTICKTASTE6)
      return 3;
   if (Tastaturwert < JOYSTICKTASTE7)
      return 6;
   if (Tastaturwert < JOYSTICKTASTE8)
      return 9;
   if (Tastaturwert < JOYSTICKTASTE9)
      return 5;
   /*
    if (Tastaturwert < JOYSTICKTASTEL)
    return 10;
    if (Tastaturwert < JOYSTICKTASTE0)
    return 0;
    if (Tastaturwert < JOYSTICKTASTER)
    return 12;
    */
   return 0;
}
// tastenwahl

uint16_t readTastatur(uint8_t kanal)
{
   uint16_t tastenwertsumme = 0;
   tastaturwertarray[mittelposition++] = analogRead(kanal);
   for (uint8_t i = 0;i<ANZ_REP;i++)
   {
      tastenwertsumme+= tastaturwertarray[i];
   }
   if(mittelposition >= ANZ_REP)
   {
      mittelposition = 0;
   }
   return  tastenwertsumme / ANZ_REP;
}

void tastenfunktion_debounce(uint16_t wert)
{
   
}

void tastenfunktion(uint16_t Tastenwert)
{  
   tastaturcounter++;   
   if (Tastenwert>10) // ca Minimalwert der Matrix
   {      
      
      
      if (tastaturcounter>=400)   //   Prellen
      {        
         if(ANZEIGE_TAST)
         {
            //Serial.print(Tastenwert);
            //Serial.print("\t");
            //Serial.print(tastaturcounter);
            tastaturcounter = 0;
            //Serial.print("\n");
            return;
         }
         tastaturcounter=0x00;
         ////Serial.println("Taste down");
         if (!(tastaturstatus & (1<<TASTE_OK))) // Taste noch nicht gedrueckt
         {
            
            ////Serial.println(Tastenwert);
            //Taste = 0;
            
            //tastaturstatus |= (1<<TASTE_ON); // nur einmal   
            tastaturstatus |= (1<<TASTE_OK); // nur einmal   
            
            switch (PCB_BOARD)
            {
               case BOARD_2:
               {
                  Taste= Joystick_Tastenwahl_33_2(Tastenwert);
                  
               }break;
               case BOARD_6:
               {
                  Taste= Joystick_Tastenwahl_33_6(Tastenwert);
               }
            }
            
            /*
             Serial.print("Tastenwert: ");
             Serial.print(Tastenwert);
             Serial.print("\t Taste: ");
             Serial.print(Taste);
             Serial.print("\n");
             */
            tastaturstatus |= (1<<AKTION_OK);
            if(OLED && Taste) // Taste und Tastenwert anzeigen
            {
               /*
                oled_delete(0,62,20);
                u8g2.setCursor(0,62);
                u8g2.print(tastaturwert);
                u8g2.print(" T ");
                u8g2.setCursor(40,62);
                u8g2.print(Taste);
                
                u8g2.sendBuffer(); 
                */
            }
            
            
            //;
         }
         else // Taste neu gedrückt
         {
            /*
             Taste = 0;
             //tastaturstatus |= (1<<TASTE_ON); // nur einmal 
             
             Taste= Joystick_Tastenwahl(Tastenwert);
             tastaturstatus |= (1<<AKTION_OK);
             if(OLED && Taste) // Taste und Tastenwert anzeigen
             {
             oled_delete(0,62,40);
             u8g2.setCursor(0,62);
             //u8g2.print(tastaturwert);
             u8g2.print("T ");
             u8g2.print(Taste);
             
             u8g2.sendBuffer(); 
             
             }
             */
            
         }
      }
      
      
   }// if tastenwert
   else 
   {
      //if (tastaturstatus & (1<<TASTE_ON))
      {
         
         //tastaturstatus &= ~(1<<TASTE_OK);
      }
   }
   
   
}//tastenfunktion

void setModus(void)
{
   switch (curr_modus)
   {
      case MODELL:
      {
      }break;
         
      case SIM:
      {
         // Joystick-Settings auf neutral stellen
         for (uint8_t i=0;i<NUM_LOKS;i++)
         {
            ////Serial.print(adcpinarray[i]);
            ////Serial.print("\t");
            ////Serial.print(servomittearray[i]);
            ////Serial.print("\t");
            
         
         }
      }break;
         
      case CALIB:
      {
         
      }break;
   }// switch curr_steuerstatus
}

void setCalib(void)
{
   
}

uint8_t initradio(void)
{
   ResetData();                   // Configure the NRF24 module  | NRF24 Modül konfigürasyonu
   radio.begin();
   radio.openReadingPipe(1,pipeIn);
   //radio.setChannel(100);
   radio.setChannel(124);
   
   // ********************
   // ACK Payload ********
   //radio.setAutoAck(false);
   // ********************
   // ********************
   
   //radio.setDataRate(RF24_250KBPS);    // The lowest data rate value for more stable communication  | Daha kararlı iletişim için en düşük veri hızı.
   radio.setDataRate(RF24_2MBPS); // Set the speed of the transmission to the quickest available
   radio.setPALevel(RF24_PA_MAX);                           // Output power is set for maximum |  Çıkış gücü maksimum için ayarlanıyor.
   radio.setPALevel(RF24_PA_MIN); 
   radio.setPALevel(RF24_PA_MAX); 
   
   
   // ********************
   // ACK Payload ********
   radio.enableAckPayload();
   // ********************
   
   radio.startListening(); 
   if (radio.failureDetected) 
   {
      radio.failureDetected = false;
      delay(250);
     
      return 0;
   }
   else
   {
      //ResetData();
     
      return 1;
      
   }
   // Start the radio comunication for receiver | Alıcı için sinyal iletişimini başlatır.
   
}

unsigned long lastRecvTime = 0;

void recvData()
{
   if ( radio.available() ) 
   {
      radiocounter++;
      //OSZIBLO;
      //ackData[0] = data.A;
      radio.read(&data, sizeof(Signal));
      lastRecvTime = millis();   // Receive the data | Data alınıyor
      
      // ********************
      // ACK Payload ********
      // ********************
      

      if(radio.writeAckPayload(1, &ackData, sizeof(ackData)))
      {
         ackcounter++;
      }

      if(radiocounter % 4 == 0)
      {
        
        
      }



      //OSZIBHI;
      
   }
}



void setup()
{
   anzeigestatus = ANZEIGE_DATA;
   
  
   delay(50);
   
   pinMode(OSZIA_PIN,OUTPUT);

   pinMode(LED_PIN,OUTPUT);
   
   analogReference(EXTERNAL);
   
   //Serial.begin(9600);
   
   // Kanal
   pinMode(DIPA_0_PIN,INPUT);
   pinMode(DIPA_1_PIN,INPUT);
   pinMode(DIPA_2_PIN,INPUT);
   pinMode(DIPA_3_PIN,INPUT);
   pinMode(AUXA_PIN,INPUT);
   pinMode(DIR_PIN,INPUT);



   pinMode(DIPA_COM_PIN,OUTPUT);
   digitalWrite(DIPA_COM_PIN,LOW);

   pinMode(DIPB_COM_PIN,OUTPUT);
   digitalWrite(DIPB_COM_PIN,LOW);

 

   //pinMode(MS_PIN, OUTPUT);
   //digitalWrite(MS_PIN,HIGH);
   
   //pinMode(BUZZPIN,OUTPUT);
   //digitalWrite(BUZZPIN,LOW);
   
   curr_steuerstatus = MODELL;
   

   //pinMode(BUZZPIN,OUTPUT);
   
   pinMode(LOOPLED,OUTPUT);
   
   pinMode(BATT_PIN,INPUT);

   pinMode(A0,INPUT);
   pinMode(A1,INPUT);
   

      // OLED
   
   
   // 0.96"
   
   initDisplay();
   _delay_ms(100);
     
   
   //setHomeScreen();
   _delay_ms(100);
     
   //                Configure the NRF24 module  | NRF24 modül konfigürasyonu

    if(initradio())
   {
      radiostatus |= (1<<RADIOSTARTED);
      //lcd_gotoxy(19,0);
      //lcd_puts("+");
   }

   //Serial.println("printDetails:");
   //radio.printDetails();
   ResetData();
   
 
   
   //Serial.print("servomitte\n");
   for (uint8_t i=0;i<NUM_LOKS;i++)
   {
      uint16_t wert = 500 + i * 50;
      wert = 750;
      //impulstimearray[i] = wert; // mittelwert
      
  
      
      
      
   } // for NUM_LOKS
   
  
   
   setupDebounce();
   
} // setup



float fmap(float x, float in_min, float in_max, float out_min, float out_max) 
{
   return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

uint16_t testwert=0;

void loop()
{                


   //            
   loopcounter++;

   
   if (zeitintervall > 500) 
   { 
      zeitintervall = 0;
      
      sekundencounter++;
      if (sekundencounter%2)
      {
         //digitalWrite(MS_PIN,LOW);
         //throttlesekunden = throttlecounter >> 8;
         blinkstatus = 1;
         if(throttlesekunden > 250)
         {
            //tone(BUZZPIN,1000);
            
         }
         
         stopsekunde++;
         if(stopsekunde%2 == 0)
         {
            //tone(BUZZPIN,1000);
         }
         if(stopsekunde == 60)
         {
            stopsekunde = 0;
            stopminute++;
         }
         refreshScreen();
         
      }
      else
      {
         //digitalWrite(MS_PIN,HIGH);
         blinkstatus = 0;
        // noTone(BUZZPIN);
      }
      
      if(curr_screen == 0)
      {
         //updateHomeScreen();
      }
      
   }   // zeitintervall > 500
   
   // Tastatur
   if (tastaturstatus & (1<<TASTE_OK) && Taste) // Menu ansteuern
   {
      tastaturcounter = 0;
      switch (Taste)
      {
         case 0: // null-pos, nichts tun
         {
            
         }break;
         case 1:
         {
            //Serial.print("T 1");   
            switch (curr_screen)
            {
                case 0:
               {
                  
                  updateHomeScreen();
                  
               }break;

            }     // switch curr_screen  
         }break;
            
        
      }//switch (Taste)
      if(Taste)
      {
         ////Serial.print("\n");
         Taste = 0;
         tastaturstatus &= ~(1<<TASTE_OK);
      }
      
   }// if TASTE_OK
   // end Tastatur
   
   if(loopcounter >= BLINKRATE)
   {



      if( radiostatus & (1<<RADIOSTARTED))
      {
            
         recvData();
         
         unsigned long now = millis();
         if ( now - lastRecvTime > 1000 ) 
         {
            
            ResetData();  // Signal lost.. Reset data
         }

      } 



       loopcounter1++;
      if(loopcounter1 > 25)
      {
         loopcounter1 = 0;
      }
       
      u8g2.setCursor(12,30);
      u8g2.print(potwertarray[0]);
      u8g2.setCursor(64,30);
      u8g2.print(potwertarray[1]);
      charh = u8g2.getMaxCharHeight();
      oled_delete(0,45,40);
      u8g2.setCursor(0,45);
      u8g2.print("A ");
      u8g2.print(ackData[1]);
      u8g2.setCursor(48,45);
      u8g2.print("B ");
      u8g2.print(ackData[3]);

      u8g2.setCursor(12,60);
      u8g2.print("E");
      u8g2.print(resetcounter);
      u8g2.setCursor(64,60);
      u8g2.print("R");
      u8g2.print(radiocounter);


      u8g2.sendBuffer();
       loopcounter = 0;
      if(TEST)
      {
         Serial.print("ACK erhalten: ");
         Serial.print("\t");
         Serial.print(ackData[0]);
         Serial.print("\t");
         Serial.print(ackData[1]);
         
         Serial.print("\t2\t");
         Serial.print(ackData[2]);
         Serial.print("\t3\t");
         Serial.print(ackData[3]);
         
         
         Serial.print(" \n");
      }

      
     
      blinkcounter++;
      impulscounter+=16;
      digitalWrite(LOOPLED, ! digitalRead(LOOPLED));
      float faktor = 0.2;
      //analogWrite(BUZZPIN,127);
      
      /*   
      batteriespannungraw = (float)analogRead(A6);
      
      if(batteriespannung == 0)
      {
         batteriespannung = batteriespannungraw;
      }
      else
      {
         batteriespannung = batteriespannung + faktor * (batteriespannungraw - batteriespannung);
      }

      //Serial.print("\t");
      //Serial.print(batteriespannung);
      //Serial.print("\n");
      UBatt = (batteriespannung) / 154;
      */   
    
      
      
      
      char buf0[4];
            //oled_horizontalbalken_setwert(HBX,HBY,balkenhb,balkenhh,werth);
      
     // batterieanzeige = (0x4C*batteriespannung)/0x9A/8; // integer-operation, resp. /154 als float
      
      
      
      /*
       Serial.print(batteriespannung);
       Serial.print("\t");
       Serial.print(batterieanzeige);
       Serial.print("\t");
       Serial.println(UBatt);
       */
      
      if(curr_screen == 0)
      {
         //updateHomeScreen();
         //u8g2.sendBuffer();
      }
     
      
      if (TEST == 1)
      {
 
         //Serial.print(" *\n");
      } // if TEST 1
      
      

      
     
      ////Serial.print(" *\n");
   }// BLINKRATE
   // EEPROM
   //eepromtaste.update();
   
   
   if(UBatt < 4.07)
   {
      //digitalWrite(BUZZPIN,!(digitalRead(BUZZPIN)));
      
   }
   if(paketcounter > 20) // 20ms
   {
      paketcounter = 0;
      digitalWrite(OSZIA_PIN, !digitalRead(OSZIA_PIN));

      // LOK A
      // pot lesen
     // POT A0
     potwert=analogRead(adcpinarray[0]);
      potwertarray[0] = potwert >> 2;
      ackData[0] = potwertarray[0];
      ackData[1] = 0;
      uint8_t del = 2;
      digitalWrite(DIPA_COM_PIN,HIGH); // DIP A enable
      //_delay_us(del);
      for(uint8_t i = 0;i<4;i++)
      {
         if(digitalRead(DIPA_Array[i]))
         {
            ackData[1] |= (1<<(4+i));
         }
         else
         {
            ackData[1] &= ~(1<<(4+i));
         }
      //_delay_us(del);
      }

      
      if(digitalRead(AUXA_PIN)) // AUXA
      {
         ackData[1] |= (1<<(0));
      }
      else
      {
         ackData[1] &= ~(1<<(0));
      }
      //_delay_us(del);
      
      if(digitalRead(DIR_PIN)) // DIR
      {
         ackData[1] |= (1<<(1));
      }
      else
      {
         ackData[1] &= ~(1<<(1));
      }
      
      ackData[1] &= ~(1<<(1));
      //_delay_us(del);
      digitalWrite(DIPA_COM_PIN,LOW); // DIPA enable END
      
      // LOK B
      // pot lesen
     // POT A1
      potwert=analogRead(adcpinarray[1]);
      potwertarray[1] = potwert >> 2;
      ackData[2] = potwertarray[1]; // speed
      ackData[3] = 0; // Code
      digitalWrite(DIPB_COM_PIN,HIGH); // DIP B enable START
      //delay(1);
      for(uint8_t i = 0;i<4;i++)
      {
         if(digitalRead(DIPA_Array[i]))
         {
            ackData[3] |= (1<<(4+i));
         }
         else
         {
            ackData[3] &= ~(1<<(4+i));
         }
      //delay(1);
      }

      if(digitalRead(AUXA_PIN)) // AUXA
      {
         ackData[3] |= (1<<(0));
      }
      else
      {
         ackData[3] &= ~(1<<(0));
      }

      if(digitalRead(DIR_PIN)) // DIR
      {
         ackData[3] |= (1<<(1));
      }
      else
      {
         ackData[3] &= ~(1<<(1));
      }
      ackData[3] &= ~(1<<(1));
      digitalWrite(DIPB_COM_PIN,LOW); // DIP enable END


      
      
      
      if(RAMPETEST)
      {
         data.B = rampe;       
      }
      
       
      
      
      /*
         digitalWrite(MS_PIN,HIGH);
         if (radio.write(&data, sizeof(data)))
         {
            radiocounter++; 
            
            // ********************
            // ACK Payload ********
            if (radio.isAckPayloadAvailable()) 
            {
               radio.read(&ackData, sizeof(ackData));
               

               
                //Serial.print("ACK erhalten: ");
                //Serial.print("\t");
                Serial.print(ackData[0]);
                Serial.print("\t");
                Serial.print(ackData[1]);
                Serial.print("\t");
                Serial.print(ackData[2]);
                Serial.print("\t");
                Serial.print(ackData[3]);
                Serial.print(" \n");
                
            } 
            else 
            {
               //Serial.println(F("Keine ACK-Daten erhalten"));
            }
            // ********************
            // ********************
         }
         else
         {
            //Serial.println("radio error\n");
            digitalWrite(BUZZPIN,!(digitalRead(BUZZPIN)));
            errcounter++;
         }
      */
        
   } // if paketcounter
}
