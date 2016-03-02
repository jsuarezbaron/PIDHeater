# PIDHeater
PID based heater control for NXP FRDM-K64F board based on mBed platform

/** A demo application to show how to mangage and control a heater element 
 * through a PID loop using a Thermistor input and PWM output
 * for the @NXP (@freescale) FRDM-K64F demo board.
 *
 * This particular example drives the heater element for a 3d Printer Extruder.
 *
 * For more information on PID control check out Brett Beauregard's Arduino PID library:
 *
 *  https://github.com/br3ttb/Arduino-PID-Library
 *
 * The wikipedia article on PID controllers is a good place to start on
 * understanding how they work:
 *
 *  http://en.wikipedia.org/wiki/PID_controller
 *
 * The Thermistor value to Temerature routine uses the Steinhart-Hart equation.
 This is a Thermistor to Temerature conversion demo 

Much thanks to @Adafruit for this tutorial:
https://learn.adafruit.com/thermistor/using-a-thermistor

The 100K Thermistor is configured with a 4.7k series resistor 
tied to vcc (3.3v)  like this:

    +3.3v
      |
      \
      /  4.7k series resistor
      \
      /
      |
      .-----------O To Anlog pin on FRDM board
      |
      \
      /
  Thermistor  100k Nominal
      \
      /
      |
     ---
     GND
 *
 * Author(s): Michael Ball  unix_guru@hotmail.com
 *
 */

