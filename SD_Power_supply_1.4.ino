 /*
 * Divizorul rezistiv pentru volti
 * R1 de la +V la ADCin = 50.1 kOhm
 * R2 de la ADCin la GND = 10.18 kOhm
 * Referinta externa masurata la 4.947V dupa activare
 * D8 > comanda releului de protectie
 * D9 > LED informatii
 * 
 * 
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
 Copyright: Adrian Florescu YO3HJV @2021
 */

//////////// SETTINGS AND CALIBRATION ////////////////
#define NUM_SAMPLES 15                // number of analog samples to take per reading. Keep it under 50!
#define SAMPLING_TIME 5               // time interval between ADC samples. Needed to settle the ADC. Keep it between 2-8 msec.
#define I2C_LCD_ADDRESS 0x27          // I2C address of the LCD
#define SERIAL_SPEED 115200           // Serial port speed 
#define BLINK_1_INTERVAL 300          // LED blink interval, msec
#define SHOW_LCD_INTERVAL 150         // Interval at which the LCD updates
#define SERIAL_SEND_INTERVAL 1        // -in minutes- We send data at this interval via Serial port


// Afisaj I2C
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
// Set the LCD address to 0x27 for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(I2C_LCD_ADDRESS, 16,2); 



byte A_pot = A0;          // pin pentru cursorul potentiometrului
byte A_pin = A1;          // pin pentru masurarea curentului
byte V_pin = A2;          // pin pentru masurarea tensiunii
byte REL_pin = 8;         // relay pin for activating the output
byte LED_pin = 9;         // software defined LED
byte BUT1_pin = 10;       // software defined function button 1


unsigned char sample_count = 0;     // current sample number
int sumV = 0;                       // sum of Voltage samples taken

float VsADC = 0.0;                  // ADC calculated voltage
float Vcc = 0.0;                    // This is the Source voltage without compensation on current resistor
float Vef = 0.0;                    // This is the calculated value of the voltage at the connectors

int sumA = 0;                       // sum of Current samples taken
float iA = 0.0;                     // calculated current on Load in Amperes
int imA = 0;                        // calculated current on Load in milliAmperes
float VaADC = 0.0;                  // voltage across the Measuring resistor ) 0.1 Ohm in my case)
float Resistor = 2.176;             // "sensor" resistor on the ground path.  former 2.264  @0.1 ohm - 0.0987

float Pow = 0.0;                    // Variabila pentru puterea calculata
int potValue = 0;                   // value read from the potentiometer
byte but1_value = 0;                // value read from the button #1

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


void readPot(){
      potValue = analogRead(A_pot);
      }



void loop(){ 
          checkButton_1();            // Boolean check
            measureAV(); 
            readPot();            
            disp_lcd();
            send_Serial_Data();         // csv log
            blink_1();
}


void checkButton_1(){                   // We put here boolean check of the button_1
   
}


void disp_lcd(){                       // Display values on LCD on time intervals to avoid flicker
     current_show_LCD_millis = millis();
     if (current_show_LCD_millis - previous_show_LCD_millis >= SHOW_LCD_INTERVAL){         
            printLCD();
          //  debugLCDmillis();                                         // Uncomment this for debugging millis
         previous_show_LCD_millis = current_show_LCD_millis;      
     }
     else { }
}




void printLCD(){  // millis check
        lcd.setCursor(0,0);
        if (Vcc < 10){
              lcd.print(" ");}
              lcd.print(Vcc);
              lcd.print("Vcc");
        
        lcd.setCursor(9, 0);
        if (Vef < 10){
              lcd.print(" ");}
              lcd.print(Vef);
              lcd.print(" V");

        lcd.setCursor(9, 1);
        if (imA >= 100 && imA < 1000)
            {lcd.print(" ");}
        if (imA >= 10 && imA < 100)
            {lcd.print("  ");}
        if (imA >= 0 && imA <10)
            {lcd.print("   ");}        
        lcd.print(imA);
        lcd.print(" mA");
        
              
        lcd.setCursor(0,1);
        if (potValue >= 100 && potValue < 1000)
            {lcd.print(" ");}
        if (potValue >= 10 && potValue < 100)
            {lcd.print("  ");}
        if (potValue >= 0 && potValue <10)
            {lcd.print("   ");}        
        lcd.print(potValue);
        //lcd.print(" Trip");
        //lcd.setCursor(11, 1);
        // lcd.print(Pow);
}


void debugLCDmillis(){
        Serial.print("LCD mills:  ");
        Serial.print(current_show_LCD_millis);
          Serial.print("  ");
        Serial.print(previous_show_LCD_millis);
          Serial.print("  ");
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
        // send voltage for display on Serial Monitor
        // voltage multiplied by x when using voltage divider that
        // divides by ratio of x. 5.931 is the calibrated voltage divide
        // value
        VaADC = ((float)sumA / (float)NUM_SAMPLES * 4.947) / 1024.0;
        iA = 1.00 * (VaADC/Resistor);      // calculate current through Resistor
        imA = 1000 * (VaADC/Resistor);
        Vcc = VsADC * 5.925;
        Vef = Vcc - VaADC;
        Pow = Vcc * iA;
        timeStamp = millis()/1000;
     // serialAV_Debug();                                     // Uncomment this line for printing the values on Serial

    
    sample_count = 0;
    sumV = 0;
    sumA = 0;
}

void send_Serial_Data(){
     DATA_INTERVAL = 60000 * SERIAL_SEND_INTERVAL;
     current_Send_millis = millis();
     //Serial.println(current_send_Data_millis);
      if (current_Send_millis - previous_Send_millis >= DATA_INTERVAL){         
         DataSend();
            
         previous_Send_millis = current_Send_millis;     
     }
     else { }
}

void DataSend() {  // String class should be used but it's simpler with Serial.print
   Serial.print(timeStamp);       // Seconds from start
   Serial.print(",");
   Serial.print(Vef);
   Serial.print(",");
   Serial.println(imA);
}


void serialAV_Debug(){
          
          Serial.print("V/R: ");
          Serial.print(VaADC,4);   // Print voltage over Resistor
          Serial.print(" V");
          Serial.print("  ");
          Serial.print("I(A): ");
          Serial.print(iA,2);      // Print Current          
          Serial.print(" A");
          Serial.print("  ");
        Serial.print("I(mA): ");
          Serial.print(imA);         // Print Current          
          Serial.print(" mA");
          Serial.print("  ");
         // Serial.print(Vcc);       // Print Power supply voltage
         // Serial.print (" V");
          Serial.print("  ");
          Serial.print("Vef: ");
          Serial.print(Vef,2);       // Print Voltage at output
          Serial.print(" V");
          Serial.print("  ");
          Serial.print("PWR: ");
          Serial.print(Pow,2);     // Print Power
          Serial.println(" W");    
}

void splashScreen(){
        delay(100);
    lcd.setCursor(0, 0);
    lcd.print("Lab Power supply");
    lcd.setCursor(0, 1);
    lcd.print("YO3HJV");
        delay(100);
    lcd.clear();
}

void referenceCheck(){              // To be enhanced
  lcd.setCursor(0, 0);
  lcd.print("Ext. Vref POST:");
  delay(500);
  lcd.setCursor(0, 1);
  lcd.print("4.944 V - PASSED");
  delay(500);
  lcd.clear();
}

void active_output(){  
    digitalWrite(REL_pin, HIGH);                // energise the output

}

void setup(){
    Serial.begin(SERIAL_SPEED);
        pinMode (LED_pin, OUTPUT);
        pinMode (BUT1_pin, INPUT_PULLUP);
        pinMode(REL_pin, OUTPUT);         
                digitalWrite(REL_pin, LOW);       // We start with power disconnected from output
    analogReference(EXTERNAL);                    // We use more precise external reference // https://www.arduino.cc/reference/en/language/functions/analog-io/analogreference/
      lcd.init();                                 // initialize the LCD
      lcd.backlight();                            // Turn on the LCD blacklight
      splashScreen();                    
      referenceCheck();
      delay(1000);  
      active_output();                            // execute function energise the output      
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

void debugBlink1millis(){
    Serial.print("Blink 1 mills:  ");
  Serial.print(current_Blink1_millis);
    Serial.print("  ");
  Serial.print(previous_Blink1_millis);
    Serial.print("  ");
  Serial.println(current_Blink1_millis-previous_Blink1_millis);
}
