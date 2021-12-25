 /* Programm:   tiny_KS
 * Author:      Peter S.
 * Startdatum   13.03.2018
 * chip:        attiny85
 * Zweck:       KS Generator für Unterwegs, fuer USB Handy Netzteil, 
 *              Strombegrenzung 10mA, Spannungsbegrenzung 58V
 * 
 * Anschlüsse:  ADC3 = PB3 - Pin2 (Strom)
 *              ADC2 = PB4 - Pin3 (Spannung)
 *              OC1A = PB1 - Pin6 (PWM out)
 *              Led  = PB2 - Pin7
 *              Pol. = PB0 - Pin5
 * Link:        https://learn.adafruit.com/diy-boost-calc/the-calculator
 *              100kHz, Vin= 4.5-5.5V,Vout= 8-60V,Iout= 10mA, Vripple= 0.05V
 *              duty cycle = 90.5 - 92.5%,Lmin= 236uH,I Lmax= 212mA, Cmin= 2uF, Shottky D. 60V 0,21A
 * Infos tiny core readme:             
 *              bei analoRead() die A# Konstante benutzen und NICHT die digitale PinNummer
 *              *** die PBn Notation kann NICHT genutzt werden*** scheinbar sollen die ARDUINO Pins genommen werden
 *              unbedingt ein 0.1uF zwischen Vcc und Gnd, so dicht wie möglich
 *				      https://github.com/SpenceKonde/ATTinyCore/
 *              
 *              2021 anderen core benutzt: http://fabacademy.org/2020/labs/akgec/students/afsha/week9.html
 *              https://raw.githubusercontent.com/damellis/attiny/ide-1.6.x-boards manager/package_damellis_attiny_index.json
 *              
 * Kontakt      https://www.facebook.com/peter.schmidt.52831
 * Lizenz:      This work is licensed under the Creative Commons Attribution-NonCommercial-ShareAlike 3.0 Unported License.
 *              To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-sa/3.0/ or send a letter to
 *              Creative Commons, 171 Second Street, Suite 300, San Francisco, California, 94105, USA.
 *              
 */
 
      +--------\/--------+
      |                  |
 PB5 [1] Rst/ADC0   Vcc [8]
      |                  |
      |                  |
 PB3 [2] ADC3      ADC1 [7] PB2 SCK 
      |                  |
      |                  |    
 PB4 [3]ADC2/OC1B  MISO [6] PB1 OC1A  
      |                  |  
      |                  |    
 GND [4]           MOSI [5] PB0 OC0A  
      |     ATtiny85     |  
      +------------------+


#include <avr/io.h>
#include <stdio.h>
 
// Konstanten
#define F_CPU 8000000UL             // CPU freq 8 MHz
#define TIMER_TO_USE_FOR_MILLIS 0

#define PWMMAX 159                  // 100 kHz
#define PWMSTART 148                // 93% errechnet

#define ACTION1 7                   // 7 (ms) Auswertung U & I und PWN regeln 
#define ACTION3 10000               // 10s

#define POLWECHSEL 20               // alle 20 Sekunden Polwechsel

#define UMAX 580                    // ADC Wert bei Sp.Teil. 390k/3.9k , 1.1V Uref , 58V , ADCwert= 535,  angepasst auf 384k /3,78k =525
#define IMAX 475                    // ADC Wert bei Shunt 51 Ohm, Imax=10mA (50 Ohm ==> 465), angepasst auf 384k /3,78k = 475

#define POLW_PIN 0                  // Arduino Pin 0 - Pin5 MOSI
#define LED_PIN 2                   // Arduino Pin 2 - Pin2 
#define PWM_PIN 1                   // Arduino Pin 1 - Pin6
#define STROM_PIN A3                // Pin 2   
#define SPANNUNG_PIN A2             // Pin 3
// neu 24.09.2021
//#define ZYKLUS 30                 // Ein Zyklus 5 Sek - 3 x blinken pro Sekunde 333ms/Periode und 333ms/2=166ms für Schaltflanken
#define ZYKLUS 40                   // 10 x blinken = 10mA = 3,3ms und 1,7s Pause
//#define LED_SCHALTZEIT 166        // 166ms
#define LED_SCHALTZEIT 250
 
bool pol_status = false;
bool toggle_flag = false;
bool ausgabe_an_led = false;

int  pwmMaxVal  = PWMMAX;
int  u_mess, i_mess;                 // 0-1023
volatile unsigned int  merke_zeit;

uint8_t blinken = 0;                 // Anzahl blinken == mA
uint8_t zyklus = ZYKLUS;

// Funktionsdeklarationen  
void toggle_pin7(void);
void pol_wechsel(void);
void regel_pwm(void);
uint8_t adcwert2mA(int messwert_i);

unsigned long tstamp1 = ACTION1;              // Startwert 7 (ms), ca. 142 Hz
const unsigned long action_time1 = ACTION1;

unsigned long tstamp2 = LED_SCHALTZEIT;       // Startwert 166ms
const unsigned long action_time2 = LED_SCHALTZEIT;  
    
unsigned long tstamp3 = ACTION3;              // Startwert 500 (ms), ca. 2 Hz
const unsigned long action_time3 = ACTION3;
    
unsigned long tstamp4 = 0;                    // Polwechsel alle 20 sek.
const unsigned long action_time4 = POLWECHSEL * 1000;

void setup()
{
    DDRB |= (1 << POLW_PIN);         // PB0 Output - Polwechsel
    PORTB &= ~(1 << POLW_PIN);
    
    DDRB |= (1 << LED_PIN);          // PB2 Output - LED
    //PORTB |= (1 << LED_PIN);       // TESTWEISE auf 1
    PORTB &= ~(1 << LED_PIN);        // Startwert 0
    digitalWrite(LED_PIN, LOW);
    
    DDRB |= (1 << PWM_PIN);          // PWM Output
    analogWrite(PWM_PIN, LOW);

    analogReference(INTERNAL1V1);
    pinMode(STROM_PIN,INPUT);
    pinMode(SPANNUNG_PIN,INPUT);
    u_mess = analogRead(SPANNUNG_PIN);
    i_mess = analogRead(STROM_PIN);  // Dummy Readout
    
    PLLCSR = 6;                      // Datenblatt 12.3.9 - 6 = PLLE & PCKE set 
                                     // PLLE PLL Enable
                                     // PCKE aendert die Timer/Couner1 clock source 
                                     // async clock mode ein und 64MHz PCK 
    TCCR1 = 0xe3;                    // 11100011 PCK/4
                                     // CTC1, PWM1A, COM1A1, COM1A0, CS13, CS12, CS11, CS10
                                     // Datenblatt 12.3.1
                                     // CTC1 - Clear  Timer/Counter on compare match OCR1C
                                     // PWM1A - Pulse Width Modulator A Enable
                                     // COM1A[1:0] - Clear OC1A output line
                                     // CS13, CS12, CS11, CS10 = PCK/4 
    GTCCR = 0;                       // General Timer counter Control Register
    
    OCR1C = pwmMaxVal;               // (Compare match Register) 100 kHz bei 159
    OCR1A = PWMSTART;                // (Komperator) 94 % PWM PulsPauseVerhaeltnis, muss unter OCR1C sein
	
    blinken = adcwert2mA(i_mess);    // blink anzahl ermitteln, startwert
    merke_zeit = millis();           // aktuelle Zeit merken
}

void loop()
{       
    if(millis() > tstamp1)           // alle 7ms Auswertung U + I und PWM regeln
        {
        tstamp1 = millis() + action_time1;       
        u_mess = analogRead(SPANNUNG_PIN);
        i_mess = analogRead(STROM_PIN);
        regel_pwm();
        }

    if(millis() > tstamp4)           // alle 20 Sekunden Polwechsel
        {
        tstamp4 = millis() + action_time4;
        pol_wechsel();
        }

    if(millis() > tstamp2)          // alle 166ms
          {
            tstamp2 = millis() + action_time2; // Zeitstempel für den nächsten 166ms Zeitpunkt !
            if(zyklus > 0)          // der Zyklus beinhaltet auch die Pause zwischen 2 Blinkzyklen 
              {
               zyklus--;
               if(blinken > 0)      // solabge blinken ungleich 0 
                {
                  toggle_pin7();
                  blinken--;        // decrement Variable
                }
              }
            else
              {
              zyklus=ZYKLUS; // neuer Blink-Zyklus
              blinken = adcwert2mA(i_mess) * 2; // wegen toggle LED mal 2
              PORTB &= ~(1 << LED_PIN);  // LED Startwert Aus
              }
          }               
}           
		
									 // Ende loop()

// *************** Funktionen ******************

void toggle_pin7(void){
    PORTB ^= (1 << LED_PIN);
  }

void pol_wechsel(void)
  { if (pol_status == true)          // wenn HIGH dann
      {
      //digitalWrite(POLW_PIN, LOW);
      PORTB &= ~(1 << POLW_PIN);
      //PORTB &= ~(1 << LED_PIN);
      pol_status = false;
      }
   else
      {
      //digitalWrite(POLW_PIN, HIGH);
      PORTB |= (1 << POLW_PIN);
      //PORTB |= (1 << LED_PIN);
      pol_status = true; 
      }
    
  }

void regel_pwm(void)
  { if(u_mess > UMAX )
    //if(u_mess > UMAX || i_mess > IMAX)
      OCR1A =  10;
    else
      OCR1A =  148; 
  }

uint8_t adcwert2mA(int messwert_i)
{ 
   if(messwert_i < 47)
      return 1;                         // Strom unter 1mA 
    else if (messwert_i >=  47  && messwert_i <=  94  )
      return 2;                         // Strom zwischen 1 und 1,9mA
    else if (messwert_i >=  95  && messwert_i <= 141  )
      return 3 ;                         // Strom zwischen 2 und 2,9mA
    else if (messwert_i >= 142  && messwert_i <= 188  )
      return 4 ;                         // Strom zwischen 3 und 3,9mA
    else if (messwert_i >= 189  && messwert_i <= 235  )
      return 5 ;                         // Strom zwischen 4 und 4,9mA
    else if (messwert_i >= 236  && messwert_i <= 283  )
      return 6 ;                         // Strom zwischen 5 und 5,9mA
    else if (messwert_i >= 284  && messwert_i <= 329   )
      return 7 ;                         // Strom zwischen 6 und 6,9mA 
    else if (messwert_i >= 330  && messwert_i <= 377   )
      return 8 ;                         // Strom zwischen 7 und 7,9mA
    else if (messwert_i >= 378  && messwert_i <= 425   )
      return 9 ;                         // Strom zwischen 8 und 8,9mA
    else if (messwert_i >= 426  && messwert_i <= 472   )
      return 10 ;                         // Strom zwischen 9 und 9,9mA
    else if (messwert_i > 473)           // && messwert_i <= 483
      return 11 ;                         // Strom zwischen 10 und 10,9mA  
  }
