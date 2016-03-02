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

#include "mbed.h"
#include "millis.h"
#include "PID.h"                                       
                                        
float getTemperature();
                                        
Serial pc(USBTX, USBRX);
Ticker PrintTicker;                     // Send process results to Console once per second       
Ticker ticker;                          // Set up the millis() ticker.
                                       
#define  DEFAULT_Kp 1
#define  DEFAULT_Ki 0.002
#define  DEFAULT_Kd 20

#define AUTOMATIC 1
#define MANUAL    0
#define DIRECT  0
#define REVERSE  1
#define thermistor A3                       // FRDM-K64F Analog input pin A3   - Adjust to your particular board
#define driver PTC3                         // FRDM-K64F PWM output pin PTC3   - Adjust to your particular board

AnalogIn Thermistor(thermistor);            // Read temperature value from thermistor on A3
PwmOut Driver(driver);                      // PWM drive FET heater on PTC3  values are 0-1.0 
                                            // For 0-100% 
             
float Input, Output, Setpoint; 
PID controller(&Input, &Output, &Setpoint, DEFAULT_Kp , DEFAULT_Ki , DEFAULT_Kd , DIRECT);

#define RATE 1.0                            // Print rate  Once per second
  
void PrintValues() {                        // Routine to print out results to console
    pc.printf("Input      Output     Setpoint   Kp        Ki        Kd        time\r\n");
    pc.printf("%f, %f, %f, %f, %f, %f, %d \r\n", 
            Input, Output, Setpoint, controller.GetKp() , controller.GetKi() , controller.GetKd() , millis() );

}    


int main(){
 
  startMillis();                            // Initialize timer.
 
  pc.baud(115200);    
  pc.printf("\r\nThermistor PID Test - Build " __DATE__ " " __TIME__ "\r\n");
  
  PrintTicker.attach(&PrintValues,RATE);    // Start PID process running at 100ms rate.

  Setpoint = 80;                            // Set target temperature in degrees Celcius.
  controller.SetMode(AUTOMATIC);            // Turn PID controller on.
  

  while(1){
   
     Input = getTemperature();              // Actual temperature in Degrees Celcius

     controller.Compute();                  // Process PID loop. 

     Driver = Output/1000;                  // Sent PWM value scaled 0 - 1.0 as mbed requires 

  }
 
}


// This is the workhorse routine that calculates the temperature
// using the Steinhart-Hart equation for thermistors
// https://en.wikipedia.org/wiki/Steinhart%E2%80%93Hart_equation

float getTemperature() {
#define THERMISTORNOMINAL 100000      // 100k 
// temp. for nominal resistance (almost always 25 C)
#define TEMPERATURENOMINAL 25   
// The beta coefficient of the thermistor (usually 3000-4000)
#define BCOEFFICIENT 3950
// the value of the 'other' resistor
#define SERIESRESISTOR 4700    

// This is the workhorse routine that calculates the temperature
// using the Steinhart-Hart equation for thermistors
// https://en.wikipedia.org/wiki/Steinhart%E2%80%93Hart_equation

    float temperature, resistance;
    float steinhart;
    int a;
    
    a = Thermistor.read_u16();       // Read 16bit Analog value
//    pc.printf("Raw Analog Value for Thermistor = %d\r\n",a);
  
    /* Calculate the resistance of the thermistor from analog votage read. */
    resistance = (float) SERIESRESISTOR / ((65536.0 / a) - 1);
//    pc.printf("Resistance for Thermistor = %f\r\n",resistance);
   
    steinhart = resistance / THERMISTORNOMINAL;         // (R/Ro)
    steinhart = log(steinhart);                         // ln(R/Ro)
    steinhart /= BCOEFFICIENT;                          // 1/B * ln(R/Ro)
    steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15);   // + (1/To)
    steinhart = 1.0 / steinhart;                        // Invert
    temperature = steinhart - 273.15;                   // convert to C

//    pc.printf("Extruder Temperature is %f\r\n", temperature);
 
    return temperature;    

}
 
