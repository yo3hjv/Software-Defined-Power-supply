/*
 * Voltage divider
 * R1 from +V to ADCin = 50.1 kOhm
 * R2 from ADCin to GND = 10.18 kOhm
 * External voltage reference measured 4.947V 
 * D8 > protection relay
 * D9 > informational LED
 * 
 * 
 * V_1.8 - ICONs added for RUN/STOP
 * V_1.7 - RUN/STOP function added. Blink SLOW and blink FAST added. Fixed START on REBOOT bug
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
#define NUM_SAMPLES 15                // number of analog samples to take per reading. Keep it under 50!
#define SAMPLING_TIME 5               // time interval between ADC samples. Needed to settle the ADC. Keep it between 2-8 msec.
#define I2C_LCD_ADDRESS 0x27          // I2C address of the LCD
#define SERIAL_SPEED 115200           // Serial port speed 
#define BLINK_1_INTERVAL 700          // LED blink interval 1, msec
#define BLINK_2_INTERVAL 70           // LED blink interval 2, msec
#define SHOW_LCD_INTERVAL 150         // Interval at which the LCD updates
#define SERIAL_SEND_INTERVAL 0.1      // -in minutes- We send data at this interval via Serial port (0.1 means update at every 6 seconds)

#define SW_VERSION   1.8 

// I2C LCD
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
// Set the LCD address to 0x27 for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(I2C_LCD_ADDRESS, 16,2); 

// SPI for Digital potentiometer MCP 41xxx
#include <SPI.h>
const byte CS = 10;

byte A_pot = A0;          // command potentiometer wiper
byte A_pin = A1;          // Analog current measured by voltage
byte V_pin = A2;          // Analog voltage measuring by voltage divider
byte REL_pin = 4;         // relay pin - protection
byte LED_pin = 2;         // software defined LED
byte BUT1_pin = 7;        // software defined function button 1


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

byte but1_value = 0;                // value read from the button #1
bool bool_active = true;

/////////////////////////// CUSTOM CHARACTERS ////////////////////////////////////////
byte pause_char[] = {
  B11011,
  B11011,
  B11011,
  B11011,
  B11011,
  B11011,
  B11011,
  B11011
};

byte play_char[] = {
  B10000,
  B11000,
  B11110,
  B11111,
  B11111,
  B11110,
  B11000,
  B10000
};



void checkButton_1(){                                           // We put here boolean check of the button_1
          but1_value = digitalRead(BUT1_pin);
          if (but1_value == 0) bool_active = !bool_active;
                                                               // Decomment for button debugging
         // Serial.print(F("but1_value:  "));
         // Serial.print(but1_value);
         // Serial.print(F("   bool_active:  "));
         // Serial.println(bool_active);
}


void relayonoff() {
                                                          // We set here crowbar relay
         if ( !bool_active) {                             // 
           digitalWrite(REL_pin, HIGH);                   // Relay On
           lcd.setCursor(0, 0);
            lcd.print(F(" RUN"));
            lcd.setCursor(5, 0);
             lcd.write(1);            // special character (icon) for PLAY
            blink_2();
         }
         else {
         digitalWrite(REL_pin, LOW);                      // Relay Off
          lcd.setCursor(0, 0);
            lcd.print(F("STOP"));
            lcd.setCursor(5, 0);
             lcd.write(0);            // special character (icon) for PLAY
          blink_1();      
         }
}

void loop(){ 
          checkButton_1();            // Check button 1 set boolean
           relayonoff();
            measureAV();              // Measure both voltage at output and voltage drop on measuring resistor
            readPot();  
            MCP41010Write(pot_to_MCP);  // Send mapped value to MCP pot          
            lcd_refresh();
            send_Serial_Data();         // as csv 
            
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



void lcd_refresh(){                       // Display values on LCD on time intervals to avoid flicker
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
        
        lcd.setCursor(8, 0);        // Show Effective Voltage at output
        if (Vef < 10){
              lcd.print(" ");}
              lcd.print(Vef, 1);
              lcd.print(" Vef");

       //   lcd.setCursor(0, 1);
       //   lcd.print(F("V."));
       //   lcd.print(SW_VERSION,1);
    
        lcd.setCursor(9, 1);        // Show delivered current
        if (imA >= 100 && imA < 1000)
            {lcd.print(" ");}
        if (imA >= 10 && imA < 100)
            {lcd.print("  ");}
        if (imA >= 0 && imA <10)
            {lcd.print("   ");}        
        lcd.print(imA);
        lcd.print(" mA");
        
         /*     
        lcd.setCursor(0,1);            // Show BYTE sent to digital potentiometer
      // if (pot_to_MCP >= 100 && pot_to_MCP < 1000)
      //      {lcd.print(" ");}        // This was to show 0-1023 values from ADC. No need 
        if (pot_to_MCP >= 10 && pot_to_MCP < 100)
            {lcd.print("0");}
        if (pot_to_MCP >= 0 && pot_to_MCP <10)
            {lcd.print("00");}        
        lcd.print(pot_to_MCP);
        //lcd.print(" Trip");
        //lcd.setCursor(11, 1);
       // lcd.print(Pow);
       */
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
   Serial.print(timeStamp);       // Seconds from start
   Serial.print(F(","));
   Serial.print(Vef);
   Serial.print(F(","));
   Serial.println(imA);
}


void serialAV_Debug(){
          
          Serial.print(F("V/R: "));
          Serial.print(VaADC,4);   // Print voltage over Resistor
          Serial.print(F(" V"));
          Serial.print("  ");
          Serial.print(F("I(A): "));
          Serial.print(iA,2);      // Print Current          
          Serial.print(F(" A"));
          Serial.print("  ");
        Serial.print(F("I(mA): "));
          Serial.print(imA);         // Print Current          
          Serial.print(F(" mA"));
          Serial.print(F("  "));
         // Serial.print(Vcc);       // Print Main Power supply voltage
         // Serial.print (F(" V"));
          Serial.print(F("  "));
          Serial.print(F("Vef: "));
          Serial.print(Vef,2);       // Print Voltage at output
          Serial.print(F(" V"));
          Serial.print(F("  "));
          Serial.print(F("PWR: "));
          Serial.print(Pow,2);       // Print Power
          Serial.println(F(" W"));    
}

void splashScreen(){
        delay(100);
    lcd.setCursor(0, 0);
    lcd.print(F("Lab Power supply"));
    lcd.setCursor(0, 1);
    lcd.print(F("YO3HJV"));
              lcd.setCursor(8, 1);
          lcd.print(F("V."));
          lcd.print(SW_VERSION,1);
        delay(1500);
    lcd.clear();
}

void referenceCheck(){                     // To be enhanced. We can check the reference agains internal reference
  lcd.setCursor(0, 0);                     // and compare the value we get with a fixed value. If what we get is 
  lcd.print(F("Ext. Vref POST:"));            // near that, we have a good reference and we can go on.
  delay(500);
  lcd.setCursor(0, 1);
  lcd.print(F("4.944 V - PASSED"));
  delay(1000);
  lcd.clear();
}


void setup(){
    Serial.begin(SERIAL_SPEED);
        pinMode (CS, OUTPUT);
        pinMode (LED_pin, OUTPUT);
        pinMode (BUT1_pin, INPUT_PULLUP);
        pinMode (REL_pin, OUTPUT);         
                digitalWrite(REL_pin, LOW);       // We start with power disconnected from output
    analogReference(EXTERNAL);                    // We use more precise external reference // https://www.arduino.cc/reference/en/language/functions/analog-io/analogreference/
      
      SPI.begin(); // initialize SPI
      lcd.init();                                 // initialize the LCD
       lcd.createChar(0, pause_char);             // create special character for pause
       lcd.createChar(1, play_char);              // create special character for play/run
      lcd.backlight();                            // Turn on the LCD blacklight
      splashScreen();                    
      referenceCheck();
      delay(1000);  
    }


void blink_1(){             // Blink SLOW RATE
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


void blink_2(){        //  Blink FAST RATE
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
    Serial.print(F("  "));
  Serial.println(current_Blink1_millis-previous_Blink1_millis);
}
