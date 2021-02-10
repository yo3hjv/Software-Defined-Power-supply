/*
 * Voltage divider
 * R7 from +V to ADCin = 50.1 kOhm
 * R6 from ADCin to GND = 10.18 kOhm
 * External voltage reference measured 4.947V 
 * D8 > protection relay
 * D9 > informational LED
 * 
 * 
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
