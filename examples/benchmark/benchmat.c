// benchmark demo for RGBmatrixPanel library.
// Measure

#include <Adafruit_GFX.h>
#include <RGBmatrixPanel.h>

#include "inc/hw_timer.h"
#include "driverlib/timer.h"

// FIXME: These should be based on matrix panel config
#define TIMER_BASE WTIMER4_BASE
#define TIMER_AB  TIMER_A


#if defined(__AVR__)
#define CLK 8  // MUST be on PORTB! (Use pin 11 on Mega)
#define LAT A3
#define OE  9
#define A   A0
#define B   A1
#define C   A2

#elif defined(__LM4F120H5QR__) || defined(__TM4C123GH6PM__)

#define CLK PB_4
#define LAT PE_4
#define OE  PE_5      
#define A   PE_3
#define B   PE_1
#define C   PE_2
#define D   PE_4

// For testing - Address lines -> LEDs
// 1 - RED, 2 - blue, 3 - Green
//#define A   PF_3
//#define B   PF_1
//#define C   PF_2

#elif defined( __TM4C1294NCPDT__ )

#define CLK PM_1
#define LAT PM_2
#define OE   PM_0

#define A   PQ_3
#define B   PP_3
#define C   PH_1
#define D   PH_0

// For testing - Address lines -> LEDs
//#define A   PN_0
//#define B   PN_1
//#define C   PF_0

//#define D   PF_4

#endif

// From: http://forum.arduino.cc/index.php/topic,138802.0.html

//
// Pi_2
//
// Steve Curd
// December 2012
//
// This program approximates pi using Newton's approximation.  It quickly
// converges on the first 5-6 digits of precision, but converges verrrry slowly
// after that.  For example, it takes over a million iterations to get to 7-8
// significant digits.
//
// For demonstration purposes, drives a JY-LKM1638 display module to show the
// approximated value after each 1,000 iterations, and toggles the pin13 LED for a
// visual "sign of life".
//
//

#define ITERATIONS 10000000L    // number of iterations
#define FLASH 10000            // blink LED every 1000 iterations

#define LEDPIN RED_LED

//#define UART_SPEED 57600
#define UART_SPEED 9600

//#include "cyclecount.h"

#define NUM_LOOPS 0xFFFFFFFFUL

//RGBmatrixPanel RGBmatrix(A, B, C, D, CLK, LAT, OE, false, 1);

// Do something and time how long it takes, report the result
unsigned long runbench(void) {
/*  uint64_t i;
  uint32_t j;

  c_start = ReadTiming();
  for(j = 0; j < NUM_OUTER; j++);
    for(i = 0; i < NUM_LOOPS; i++);
  c_stop = ReadTiming();
  return c_stop - c_start; */

  unsigned long start, time;
  unsigned long niter=ITERATIONS;
  unsigned long LEDcounter = 0;
  boolean alternate = false;
  unsigned long i;
  double x = 1.0;
  double pi=1.0;

  start = millis();
  for ( i = 2; i < niter; i++) {
    x *= -1.0;
    pi += x / (2.0*(double)i-1.0);
#if defined(FLASH)    
    if (LEDcounter++ > FLASH) {
      LEDcounter = 0;
      if (alternate) {
        digitalWrite(LEDPIN, HIGH);
        alternate = false;
      } else {
        digitalWrite(LEDPIN, LOW);
        alternate = true;
      }
    }
#endif    
  }

  time = millis() - start;
  return time;
}

#define PANEL_MAX  8

void setup() {
  unsigned i;
//  EnableTiming();
  pinMode(LEDPIN, OUTPUT);
  Serial.begin(UART_SPEED);
  
  unsigned long t_nodisp, t_disp;

  Serial.println("***********************************");
  Serial.println("RGBmatrixPanel benchmark begin");
  Serial.println();

  Serial.print("No display: ");
  t_nodisp = runbench();
  Serial.println(t_nodisp);

#if defined(DEBUG_RGBMAT)
  Serial.println("RGBMatrix Debugging on");
#endif
#if defined(BENCHMARK_RGBMAT)
  Serial.println("RGBMatrix Timing Measurement on");
#endif
  
  for( i = 1; i <= PANEL_MAX; i++){
      Serial.println("_______");
      Serial.print("Panels: ");
      Serial.println(i);
      Serial.println();
      RGBmatrixPanel LEDmatrix(A, B, C, D, CLK, LAT, OE, false, i);
    
      LEDmatrix.begin();
      delay(100);      // Short delay to avoid startup timing on RGBmatrix
      if(LEDmatrix.isRunning())
        Serial.print("Running");
      else
        Serial.println("Refresh halted");
      
      Serial.println();
      Serial.print("Display: ");
      t_disp = runbench();
      Serial.println(t_disp);
      Serial.print("Difference: ");
      Serial.print(t_disp - t_nodisp);
      Serial.print(" % ");
      Serial.println((float)(t_disp - t_nodisp)/t_nodisp);
      delay(100);      // Short delay to avoid startup timing on RGBmatrix
      Serial.print("Slow refresh: ");
      Serial.println(LEDmatrix.setRefresh(70));
      Serial.print("Display: ");
      t_disp = runbench();
      Serial.println(t_disp);
      Serial.print("Difference: ");
      Serial.print(t_disp - t_nodisp);
      Serial.print(" % ");
      Serial.println(((float)t_disp - t_nodisp)/t_nodisp);
      Serial.println();
    
      Serial.print("Faster refresh: ");
      Serial.println(LEDmatrix.setRefresh(140));
      Serial.print("Display: ");
      t_disp = runbench();
      Serial.println(t_disp);
      Serial.print("Difference: ");
      Serial.print(t_disp - t_nodisp);
      Serial.print(" % ");
      Serial.println(((float)t_disp - t_nodisp)/t_nodisp);
      Serial.println();
      
      Serial.print("Fastest refresh: ");
      Serial.println(LEDmatrix.setRefresh(2000));
      delay(1);  // Be sure the refresh has time to take effect
      if(LEDmatrix.isRunning())
        Serial.print("Running");
      else
        Serial.println("Refresh halted");
      
      Serial.print("Display: ");
      t_disp = runbench();
      Serial.println(t_disp);
      Serial.print("Difference: ");
      Serial.print(t_disp - t_nodisp);
      Serial.print(" % ");
      Serial.println(((float)t_disp - t_nodisp)/t_nodisp);
      Serial.println();
      LEDmatrix.end();
  }
  Serial.println();
  Serial.println("Benchmark end");
 // If optimize driver to handle blank lines specially, then would be interesting to benchmark
 // with something on screen vs. blank screen.
 //  drawstuff();
 // runbench();
}

void loop() {

}
