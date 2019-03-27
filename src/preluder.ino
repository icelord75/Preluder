/*
   //      _______ __  _________ _________
   //      \_     |  |/  .  \   |   \ ___/
   //        / :  |   \  /   \  |   / _>_
   //       /      \ | \ |   /  |  /     \
   //      /   |___/___/____/ \___/_     /
   //      \___/--------TECH--------\___/
   //        ==== ABOVE SINCE 1994 ====
   //
   //   Ab0VE TECH - HONDA Prelude Gen4 Extra Gauges and Odometer controller
 */

#include <Arduino.h>
#include <avr/pgmspace.h>
#include <Arduino.h>
#include <font5x7.h>            // Local font lib
#include <SPI.h>
#include <avr/pgmspace.h>
#include <LedDisplay.h>
#include <I2C_eeprom.h>         // platformio lib install "I2C_EEPROM"
#include <U8glib.h>             // platformio lib install "U8glib"
#include <Adafruit_ADS1015.h>   // platformio lib install "Adafruit ADS1X15"
#include <Adafruit_MLX90614.h>  // platformio lib install "Adafruit MLX90614 Library"
#include <OneWire.h>            // platformio lib install "OneWire"

#include "img.h"                // Mode logos
#ifdef OPTIBOOT
#include <avr/wdt.h>
#endif

I2C_eeprom eeprom(0x50,16384/8);  // FM24C16A FRAM
U8GLIB_SSD1306_128X64_2X u8g(U8G_I2C_OPT_NONE);
Adafruit_ADS1115 ads;
Adafruit_ADS1115 ads1115(0x48); // construct an ads1115 at address 0x48
Adafruit_MLX90614 mlx = Adafruit_MLX90614();

//
// SENSORS
//
/* OIL TRESSURE / OIL TEMPERATURE */
#define OIL_TEMP_SENSOR     A0
#define R3 9920.0 // exact resistance of R3 (10K) in voltage devider
float OIL_TEMP = 0;

#define THERMISTOR_NOMINAL 1400
// temp. for nominal resistance (almost always 25 C)
#define TEMPERATURE_NOMINAL 25
// The beta coefficient of the thermistor (usually 3000-4000)
#define B_COEFFICIENT 3950
#define SENSORS_DELAY 2
#define NUM_SAMPLES  5
uint16_t samples[NUM_SAMPLES];

#define OIL_PRESSURE_SENSOR A1
#define R4 994.0 // exact resistance of R4 (1K) in voltage devider
float OIL_PRESSURE = 0;

/* VOLTMETER    /        - */
#define VOLTMETER_SENSOR    A2
#define R1 47000.0 // exact resistance of R1 (47K ) in voltage divider
#define R2 9950.0  // exact resistance of R2 (10K) in voltage divider
float VOLTAGE = 0;

/*    -         / BRAKES TEMPERATURE
   MLX90614
      SDL-----------A4
      SDA-----------A5
 */
float BRAKES_TEMP = 0;

/* O2 LAMBDA   / EXHAUST TEMPERATURE

   ADS1X15/max6675    arduino
     SDL-------------A4
     SDA-------------A5
   MAX6675
 */
int thermoDO  = 2;
int thermoCS  = 3;
int thermoCLK = 4;

#define AFR_INPUT 0
float AFR = 0;
float EGT = 0;

/*
     VFD Contoller
     μPD6232C    arduino
     SI-------------8 BLue
     SCK------------7 red
     LH-------------6 green
 */
int SI_PIN  = 8; // Blue
int SCK_PIN = 7; // Green
int LH_PIN  = 6; // Gray

int DIMMER_PIN = 13; // Dimer input - High +12V!
int DIMMER_STATE = 0;
int DIMMER_PREVSTATE = 0;

int BUTTON_PIN = 12; // BUTTON to Ground
int _STATE = 0;
int BUTTON_STATE = 0;
int BUTTON_PREVSTATE = 0;

/*
     OLED DISPLAY  arduino
     SDL-------------A4
     SDA-------------A5
 */
#define NONE   0
#define BAR    1
#define NEEDLE 2

boolean DRAW_R = true;
boolean DRAW_RL = true;  // C - H
boolean DRAW_L = true;
byte TYPE_R = BAR;
byte TYPE_L = BAR;

byte POSITION_L = 0;
byte TARGETPOS_L = 0;
byte POSITION_R = 0;
byte TARGETPOS_R = 0;

#define STATE_OIL     1
#define STATE_EXHAUST 2
#define STATE_BRAKES  3
#define STATE_VOLT    4

boolean SHOW_LOGO = true;
int LOGO_STATUS = STATE_OIL;
#define MAX_LOGO      4

unsigned long time=0;

#define DATALOG_ENABLE        // SERIAL DATALOGING
unsigned long timeP=0;
#define LOG_DELAY 1000

// Delay for data update on OLED after new logo shown
unsigned long timeL=0;
#define LOGO_DELAY 3000

// OLED data update delay
unsigned long timeOLED=0;
#define OLED_DELAY 500

// ALARMS
#define ALARM_PIN 5

#define ALARM_EGT 1000           // Exhaust Temtrature too high
#define ALARM_OIL 0.5            // Oil pressure is too low
#define ALARM_TEMP 140           // Oil temperature is too high
#define ALARM_BRAKES 351         // Brakes temperature is too high
#define ALARM_BATTERY_LOW 12.5   // Alternator output is too low
#define ALARM_BATTERY_HIGH 15.0  // Alternator output it too high
#define ALARM_BETTERY_DELAY 2000 // 2.0sec delay for engine start


#define PPR 15 // VSS pulses per axle revolution

#define TRIP_A              0
#define TRIP_B              1
#define MOTOR_HOUR          2
#define OUTSIDE_TEMP        3

#define MAX_SHOW            3

// default tire size 205/55R15
#define TIRE_WIDTH_DEFAULT 4
#define TIRE_SIDE_DEFAULT  6
#define TIRE_RIM_DEFAULT   2

uint16_t TIRE_WIDTH_ARRAY[] = { 165, 175, 185, 195, 205, 215, 225, 235, 245, 255, 265, 275, 285, 295, 305 };
uint8_t TIRE_SIDE_ARRAY[] =   { 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85 };
uint8_t TIRE_RIM_ARRAY[] =    { 13, 14, 15, 16, 17, 18, 19, 20, 21, 22 };

#define CONFIG_POS_MAX 11

// CONFIG
uint8_t TIRE_RIM;
uint8_t TIRE_WIDTH;
uint8_t TIRE_SIDE;
#define NEEDLE_STEP 16
#define NEEDLE_DIMMED_DEFAULT     127
#define NEEDLE_UNDIMMED_DEFAULT   255
uint8_t NEEDLE_DIMMED;
uint8_t NEEDLE_UNDIMMED;

#define MIN_DISPLAY 1 // Minimal visiable display
#define MAX_DISPLAY 15
#define DISPLAY_DIMMED_DEFAULT    10
#define DISPLAY_UNDIMMED_DEFAULT  15
uint8_t DISPLAY_DIMMED;
uint8_t DISPLAY_UNDIMMED;

#define INDIGLO_STEP 16
#define INDIGLO_DIMMED_DEFAULT    127
#define INDIGLO_UNDIMMED_DEFAULT  255
uint8_t INDIGLO_DIMMED;
uint8_t INDIGLO_UNDIMMED;

double TOTAL_TRIP;
double DAILY_TRIP_A;
double DAILY_TRIP_B;
int CURRENT_SHOW=TRIP_A;
boolean LEADING_ZERO;

// VSS input pin
#define VSS_PIN 2
unsigned int PULSES = 0;
float LENPERPULSE = 0;

// RPM input pin
#define RPM_PIN 3
volatile unsigned int RPM_COUNT=0;
unsigned long timeold=0;
unsigned int RPMs=0;
float MOTOR_TIME;
unsigned int NOMINAL_RPM; // RPM with 1 hour == 1 motor hour
#define MAX_NOMINAL 6000
#define NOMINAL_STEP 200
float MOTOR_HOURS;
uint16_t MOTOR_HOURS_LIMIT;
#define DEFAULT_MOTOR_HOURS_LIMIT 200
#define MOTOR_HOURS_STEP 10
#define MAX_MOTOR_HOURS 1000
boolean LIMIT_BLINK=true;
uint8_t DIMMING;

// PINS CONFIG
#define BUTTON_PIN 12
#define NEEDLE_PIN 9
#define DIM_PIN 14 // A0 as DIGITAL_PIN
#define SETUP_PIN 11
#define INDIGLO_PIN 10

#define LED_dataPin 4
#define LED_registerSelect 5
#define LED_clockPin 6
#define LED_enable 7
#define LED_reset 8

#define LED_displayLength 16  // Two HCMS-297x led matrix displays
LedDisplay myDisplay = LedDisplay(LED_dataPin, LED_registerSelect, LED_clockPin, LED_enable,
                                  LED_reset, LED_displayLength);

#define FIRST_LOGO_DELAY    500 // first logo output delay
#define LONGPRESS_TIME      1000

float LEN=0;
float TIRE_CIRCUMFERENCE;
float TEMPERATURE=0;
unsigned long TIME,TIMES;
char buffer[20];
bool PRESSED=false;
bool LONGPRESS=false;
bool DIM=false;
bool DIMMED=false;
bool SETUP_PRESSED=false;
bool SETUP_DO=false;
uint8_t val;
uint8_t SETUP_POS=0;
uint8_t data[12];
uint8_t addr[8];
uint8_t type_s;

#define DISPLAY_TRIP 0
#define DISPLAY_SETUP 1
uint8_t DISPLAY_MODE=DISPLAY_TRIP;
uint8_t DEFAULT_BRIGHTNESS;
uint8_t DEFAULT_NEEDLE;
uint8_t DEFAULT_INDIGLO;

boolean ALARM_STATUS = false;
boolean ALARM_BLINK;
uint32_t ALARM_TIME=0;


void CalcTire()
{
        TIRE_CIRCUMFERENCE = (TIRE_WIDTH_ARRAY[TIRE_WIDTH]*TIRE_SIDE_ARRAY[TIRE_SIDE]/100+TIRE_RIM_ARRAY[TIRE_RIM]*25.4)*3.1416/1000;
        LENPERPULSE=TIRE_CIRCUMFERENCE/PPR; // in Meters
#ifdef DEBUG
        Serial.print("TIRE CIRCUMFERENCE: "); Serial.print(TIRE_CIRCUMFERENCE);
        Serial.println("m");
        Serial.print("LEN PER PULSE: "); Serial.print(LENPERPULSE);
        Serial.println("m");
#endif
}

void VSS() // VSS signal interrupt
{
        PULSES++;
}

void RPM() // VSS signal interrupt
{
        RPM_COUNT++;
}

void setBrightness(uint8_t bright)
{
        // set the brightness: for each 4 chars
        for ( int a=1; a<LED_displayLength/4; a++ )
                myDisplay.loadControlRegister(B01110000 + (bright & B00001111));
}


void setup() {
#ifdef DEBUG
        Serial.begin(115200);
#endif
        eeprom.begin();
        myDisplay.begin();
        setBrightness(0);
#ifdef DEBUG
        Serial.print("\n\nAb0VE-TECH\nHonda Prelude Gauges add Oddometer\n\n");
#endif
}

/*
    MAIN GAUGES LOOP
 */
void loop() {

}
