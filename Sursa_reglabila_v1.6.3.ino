/*
 * Voltage divider
 * R7 from +V to ADCin = 50.1 kOhm
 * R6 from ADCin to GND = 10.18 kOhm
 * External voltage reference measured 4.947V 
 * D8 > protection relay
 * D9 > informational LED
 * 
 * 
 * V_1.6.3 - Some parameters added. Run/Stop with protection indication on LCD
 * V_1.6.2 - Start/Stop button function added
 * V_1.6.1 - Move fixed text to EEPROM for space
 * V_1.6 - Digital potentiometer in circuit
 * V_1.5 - Free the pins for SPI potentimeter. Relay to D4, LED to D2.
 * V_1.4 - Reading potentiometer. Set value from potentiometer
 * V_1.3 - Format the readings for output, send it on Serial at preset time as in SETTINGS
 * V_1.2 - added millis for LCD Display to avoid flicker and for LED to blink, dunno why.
 * v_1.1 - Button on pin 12 and LED on pin 9. Both to ground
 * V_0.9 - Power output relay on D8 pin
 *       - HW > DC/DC Step down
 * V_0.8 - Introducing timed LCD showing
 * V_0.7 - DEBUG to serial added as separate function.
 *       - Some housekeeping
 * V_0.6 - LCD added
 * 
 */

//////////// SETTINGS AND CALIBRATION ////////////////
#define NUM_SAMPLES 5                 // number of analog samples to take per reading. Keep it under 50!
#define SAMPLING_TIME 3               // time interval between ADC samples in milliseconds. Needed to settle the ADC. Keep it between 2-8 msec.
#define I2C_LCD_ADDRESS 0x27          // I2C address of the LCD
#define SERIAL_SPEED 115200           // Serial port speed 
#define BLINK_1_INTERVAL 600          // LED blink interval, msec, SLOW
#define BLINK_2_INTERVAL 200          // LED blink interval, msec, FAST
#define SHOW_LCD_INTERVAL 150         // Interval at which the LCD updates
#define SERIAL_SEND_INTERVAL 0.005      // -in minutes- We send data at this interval via Serial port
#define Imax_THRESH 1900              // Shortcircuit protection current in mA
#define Imin_THRESH 0                 // Min current thresh, good for rechargeable batteries
#define Vmin_THRESH 0                 // No specific reason
#define Vmax_THRESH 4.2                // Max V, good for rechargeable batteries

// I2C LCD
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
// Set the LCD address to 0x27 for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(I2C_LCD_ADDRESS, 16,2); 

// SPI for Digital potentiometer MCP 41xxx
#include <SPI.h>
const byte CS = 10;       // Chip select MCP on D10

byte A_pot = A0;          // Wiper for analog potentiometer
byte A_pin = A1;          // Analog current measured by voltage
byte V_pin = A2;          // Analog voltage measuring by voltage divider
byte REL_pin = 4;         // relay pin - protection
byte LED_pin = 2;         // software defined LED
byte SWITCH1_pin = 7;     // software defined function button 1


unsigned char sample_count = 0;     // current sample number
int sumV = 0;                       // sum of Voltage samples taken

float VsADC = 0.0;                  // ADC calculated voltage
float Vcc = 0.0;                    // This is the Source voltage without compensation on current resistor
float Vef = 0.0;                    // This is the calculated value of the voltage at output connector

int sumA = 0;                       // sum of Current samples taken
float iA = 0.0;                     // calculated current on Load in Amperes
int imA = 0;                        // calculated current on Load in milliAmperes
float VaADC = 0.0;                  // voltage across the Measuring resistor (0.1 Ohm in my case)
float Resistor = 2.176;             // "sensor" resistor on the ground path.  former 2.264  @0.1 ohm - 0.0987

float Pow = 0.0;                    // We can determine POWER by math
int potValue = 0;                   // value read from the analog potentiometer

int pot_to_MCP = 0;                 // BYTE for sending to the MCP potentiometer, mapped from ADC

// Timer things

unsigned long current_show_LCD_millis;
unsigned long previous_show_LCD_millis = 0;
//int show_LCD = LOW;

unsigned long current_Blink1_millis;
unsigned long previous_Blink1_millis = 0;
int led_State = LOW;

unsigned long DATA_INTERVAL = 0;
unsigned long current_Send_millis;
unsigned long previous_Send_millis = 0;
unsigned long Serial_Data_Pace;
int send_Data = LOW;
                                      // Time manipulation via global variables to be used in several functions
unsigned long Now;
int timeStamp = 0;

  // Button things
const byte debounceTime = 5;  // 5 mSec, enough for most switches 
// remember byte SWITCH1_pin = 7;  
const bool switch1On  = false;     // using INPUT_PULLUP
const bool switch1Off = true;

bool lastState1   = switch1Off;
bool newState1    = switch1Off;
bool toggleState1 = false;

void checkButton_1(){
          newState1 = digitalRead( SWITCH1_pin );
              if( lastState1 != newState1 ) // state changed
              {
                delay( debounceTime );
                 lastState1 = newState1;
               
                // push on, push off
                if( newState1 == switch1On && toggleState1 == false )
                {
                  toggleState1 = true;
                 // doSomething with ON
                 // Serial.println( F ( "Switched ON" ) );
                }
                else if( newState1 == switch1On && toggleState1 == true )
                {
                  toggleState1 = false;
                 // doSomething with OFF
                 // Serial.println( F ( "Switched OFF" ) );
                }
            }
}

void loop(){ 
          checkButton_1();              // Boolean check
            measureAV();                // Measure both voltage at output and voltage drop on measuring resistor
            readPot();                  //
            MCP41010Write(pot_to_MCP);  // Send mapped value to MCP pot. Must be before readPot         
            disp_lcd();
            check_protection();         // Here we test protection condition and set boolean for relay
            main_relay();               // In this function we check boolean and activate/deactivate relay
            send_Serial_Data();         // as csv 

}


void check_protection(){
                        if ((imA > Imax_THRESH) || (Vef > Vmax_THRESH)) // ARBITRARY TEST CONDITION
                        {
                          toggleState1 = false;
                          lcd.setCursor(0, 1);
                          lcd.print(F("PROT!"));
                        }
                        else if ( toggleState1 == true)
                        {
                          lcd.setCursor(0, 1);
                          lcd.print(F("     "));
                        }
}


void main_relay(){                      
                                        // I used boolean of switch 1 to trip protection by de-energising the main relay
              if (toggleState1 == true){ digitalWrite(REL_pin, HIGH);
                                         blink_2();                  // blink type 1}
                                        }
              else if (toggleState1 == false) {digitalWrite(REL_pin, LOW);
                                         blink_1();
                                         }
}


void readPot(){
      potValue = analogRead(A_pot);                  // read pot analog value
      pot_to_MCP = map(potValue, 0, 1023, 0, 255);   // map the value to one BYTE
      }


void MCP41010Write(byte pot_to_MCP){
                                      // This function is called with VALUE from readpot() function
           digitalWrite(CS,LOW); // select the chip
           SPI.transfer(B00010001); // command byte
           SPI.transfer(pot_to_MCP); // data byte 
           digitalWrite(CS,HIGH); // de-select the chip
}


void disp_lcd(){                       // Display values on LCD on time intervals to avoid flicker
     current_show_LCD_millis = millis();
         if (current_show_LCD_millis - previous_show_LCD_millis >= SHOW_LCD_INTERVAL){         
                printLCD();
              //  debugLCDmillis();    // Uncomment this for debugging millis
             previous_show_LCD_millis = current_show_LCD_millis;      
         }
         else { }
}




void printLCD(){               // This execute after millis check
      //  lcd.setCursor(0,0);       // Show calculated Vcc at the output of DC/DC
      //  if (Vcc < 10){            // without the voltage drop on Current Measuring resistor
      //        lcd.print(" ");}
      //        lcd.print(Vcc);
      //        lcd.print("Vcc");
        
        lcd.setCursor(10, 0);        // Show measured Voltage at output
        if (Vef < 10){
              lcd.print(F(" "));}
              lcd.print(Vef, 1);
              lcd.print(F(" V"));

        lcd.setCursor(9, 1);        // Show delivered current
        if (imA >= 100 && imA < 1000)
            {lcd.print(F(" "));}
        if (imA >= 10 && imA < 100)
            {lcd.print(F("  "));}
        if (imA >= 0 && imA <10)
            {lcd.print(F("   "));}        
        lcd.print(imA);
        lcd.print(F(" mA"));

                    // Show START/STOP on LCD

             if (toggleState1 == true){
                  lcd.setCursor(0, 0);
                  lcd.print(F("  RUN     "));
             }
             else if (toggleState1 == false){
                  lcd.setCursor(0, 0);
                  lcd.print(F(" STOP"));
             }
        
        /*      
        lcd.setCursor(0,1);            // Show BYTE sent to digital potentiometer
     
        if (pot_to_MCP >= 10 && pot_to_MCP < 100)
            {lcd.print(F("0"));}
        if (pot_to_MCP >= 0 && pot_to_MCP <10)
            {lcd.print(F("00"));}        
        lcd.print(pot_to_MCP);

        */
        //lcd.print(" Trip");
        //lcd.setCursor(11, 1);
       // lcd.print(Pow);
}


void debugLCDmillis(){
        Serial.print(F("LCD mills:  "));
        Serial.print(current_show_LCD_millis);
          Serial.print(F("  "));
        Serial.print(previous_show_LCD_millis);
          Serial.print(F("  "));
        Serial.println(current_show_LCD_millis-previous_show_LCD_millis);
}


void measureAV(){
      while (sample_count < NUM_SAMPLES) {                               // take a number of analog samples and add them up
                                          sumV += analogRead(V_pin);
                                                 delay(SAMPLING_TIME);   // ADC are multiplexed. Time between swtiching ADC inputs
                                          sumA += analogRead(A_pin);
                                          sample_count++;
                                          delay(SAMPLING_TIME);          // Time between ADC readings. Keep it small.
                                          }

        VsADC = ((float)sumV / (float)NUM_SAMPLES * 4.947) / 1024.0;
        // voltage multiplied by x when using voltage divider that
        // divides by ratio of x. 5.931 is the calibrated voltage divide
        // value
        VaADC = ((float)sumA / (float)NUM_SAMPLES * 4.947) / 1024.0;
        iA = 1.00 * (VaADC/Resistor);      // calculate current through Current  measuring Resistor
        imA = 1000 * (VaADC/Resistor);
        Vcc = VsADC * 5.925;
        Vef = Vcc - VaADC;
        Pow = Vcc * iA;
        timeStamp = millis()/1000;
     // serialAV_Debug();                                     // Uncomment this line for printing the values on Serial
   
    sample_count = 0;    // reset some variables for the next cycle
    sumV = 0;
    sumA = 0;
}

void send_Serial_Data(){                            // Send csv to Serial UART
     DATA_INTERVAL = 60000 * SERIAL_SEND_INTERVAL;
     current_Send_millis = millis();
     //Serial.println(current_send_Data_millis);
      if (current_Send_millis - previous_Send_millis >= DATA_INTERVAL){         
         DataSend();            
         previous_Send_millis = current_Send_millis;     
     }
     else { }
}

void DataSend() {                 // String class should be used but it's simpler with Serial.print
  // Serial.print(timeStamp);       // Seconds from start Comment for Arduino Serial Plotter
  // Serial.print(F(","));        // Comment for Arduino Serial Plotter
   Serial.print(Vef*10);
   Serial.print(F(" "));          // Replace it with comma when csv is needed
   Serial.println(imA);
}


void serialAV_Debug(){
          
          Serial.print(F("V/R: "));
          Serial.print(VaADC,4);   // Print voltage over Resistor
          Serial.print(F(" V"));
          Serial.print(F("  "));
          Serial.print(F("I(A): "));
          Serial.print(iA,2);      // Print Current          
          Serial.print(F(" A"));
          Serial.print(F("  "));
          Serial.print(F("I(mA): "));
          Serial.print(imA);         // Print Current          
          Serial.print(F(" mA"));
          Serial.print(F("  "));
         // Serial.print(Vcc);       // Print Power supply voltage
         // Serial.print (F(" V"));
          Serial.print(F("  "));
          Serial.print(F("Vef: "));
          Serial.print(Vef,2);       // Print Voltage at output
          Serial.print(F(" V"));
          Serial.print(F("  "));
          Serial.print(F("PWR: "));
          Serial.print(Pow,2);     // Print Power
          Serial.println(F(" W"));    
}

void splashScreen(){
        delay(100);
    lcd.setCursor(0, 0);
    lcd.print(F("Lab Power supply"));
    lcd.setCursor(0, 1);
    lcd.print(F("YO3HJV"));
        delay(1000);
    lcd.clear();
}

void referenceCheck(){                     // To be enhanced. We can check the reference agains internal reference
  lcd.setCursor(0, 0);                     // and compare the value we get with a fixed value. If what we get is 
  lcd.print(F("Ext. Vref POST:"));            // near that, we have a good reference and we can go on.
  delay(1000);
  lcd.setCursor(0, 1);
  lcd.print(F("4.944 V - PASSED"));
  delay(1000);
  lcd.clear();
}

void active_output(){  
    digitalWrite(REL_pin, HIGH);          // energise the output
}

void setup(){
    Serial.begin(SERIAL_SPEED);
        pinMode (CS, OUTPUT);
        pinMode (LED_pin, OUTPUT);
        pinMode (SWITCH1_pin, INPUT_PULLUP);
        pinMode (REL_pin, OUTPUT);         
                digitalWrite(REL_pin, LOW);       // We start with power disconnected from output
    analogReference(EXTERNAL);                    // We use more precise external reference // https://www.arduino.cc/reference/en/language/functions/analog-io/analogreference/
      
      SPI.begin(); // initialize SPI
      lcd.init();                                 // initialize the LCD
      lcd.backlight();                            // Turn on the LCD blacklight
      splashScreen();                    
      referenceCheck();
      delay(1000);    
    }


void blink_1(){
     current_Blink1_millis = millis();
     if (current_Blink1_millis - previous_Blink1_millis >= BLINK_1_INTERVAL){         
          // debugBlink1millis();                                  // uncomment to debug Blink1 millis
           previous_Blink1_millis = current_Blink1_millis;
               // if the LED is off turn it on and vice-versa:
                if (led_State == LOW) {
                  led_State = HIGH;
                } else {
                  led_State = LOW;
                }
        digitalWrite (LED_pin, led_State);
}
}

void blink_2(){
     current_Blink1_millis = millis();
     if (current_Blink1_millis - previous_Blink1_millis >= BLINK_2_INTERVAL){         
          // debugBlink1millis();                                  // uncomment to debug Blink1 millis
           previous_Blink1_millis = current_Blink1_millis;
               // if the LED is off turn it on and vice-versa:
                if (led_State == LOW) {
                  led_State = HIGH;
                } else {
                  led_State = LOW;
                }
        digitalWrite (LED_pin, led_State);
}
}



void debugBlink1millis(){
    Serial.print(F("Blink 1 mills:  "));
  Serial.print(current_Blink1_millis);
    Serial.print(F("  "));
  Serial.print(previous_Blink1_millis);
    Serial.print("  ");
  Serial.println(current_Blink1_millis-previous_Blink1_millis);
}
