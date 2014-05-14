
/* RGBmatrixPanel Arduino/Energia library for 16x32 and 32x32 RGB LED matrix panels. */


#ifndef RGBmatrixPanel_h
#define RGBmatrixPanel_h

#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
 #include "pins_arduino.h"
#endif

#include "Adafruit_GFX.h"


// Define build control macros: __TIVA__ and __ARM__
// FIXME: What is the Macro that indicates Stellaris/Tiva LP in Energia?
// Just a temporary patch - until find the proper macro 
// There is a macro for MSP430 - __MSP430_CPU__
// Is there a macro for C2000?

#if defined(ENERGIA)

//#if !defined(__MSP430_CPU__)

#if defined(__TM4C129XNCZAD__) || defined(__TM4C1294NCPDT__) || defined(__LM4F120H5QR__) || defined(__TM4C123GH6PM__)

#ifndef __TIVA__
#define __TIVA__
#endif

#ifndef __ARM__
#define __ARM__
#endif

#else

#error "**** Unrecognized processor ****"

#endif

#endif // ENERGIA


// define FADE to include support for fading between displays
#define FADE

// FIXME: Number of buffers
const uint8_t nBuf = 2;

class RGBmatrixPanel : public Adafruit_GFX {

 public:
    
  // Constructor for 16x32 panel:
  RGBmatrixPanel(uint8_t a, uint8_t b, uint8_t c,
    uint8_t sclk, uint8_t latch, uint8_t oe, boolean dbuf, uint8_t pwidth = 1);
    /* Parameters
    a, b, c are the pins used for addressing the rows
    cclk, latch and oe are the pins used for Serial Clock, Latach and Output Enable
    dbuf enables double buffering. This will use 2x RAM for frame buffer, but will give nice smooth animation
    pwidth is the number of panels used together in a multi panel configuration
    */

  // Constructor for 32x32 panel (adds 'd' row address pin)
  RGBmatrixPanel(uint8_t a, uint8_t b, uint8_t c, uint8_t d,
    uint8_t sclk, uint8_t latch, uint8_t oe, boolean dbuf, uint8_t pwidth);

// TODO: See if can get 32x32 constructor with default pwidth
// Compiler couldn't distinguish this one.
//  RGBmatrixPanel(uint8_t a, uint8_t b, uint8_t c, uint8_t d,
//    uint8_t sclk, uint8_t latch, uint8_t oe, boolean dbuf);

  void
    begin(void),              // Start display
    stop(void),               // Stop display
    drawPixel(int16_t x, int16_t y, uint16_t c),
    fillScreen(uint16_t c),   // fill current drawing buffer with color c
    updateDisplay(void),      // TODO: Why is updateDisplay public?
    swapBuffers(boolean copy = false),  // Display next buffer
    dumpMatrix(void);
  int8_t
    loadBuffer(uint8_t *img, uint16_t imgsize);
  uint8_t
    setDraw(uint8_t back),
    setNext(uint8_t next),
    getBack(void),
    getNext(void);
  uint8_t
    *frontBuffer(void),
    *backBuffer(void);
  uint16_t
    getPixel(int16_t x, int16_t y),
    Color333(uint8_t r, uint8_t g, uint8_t b),
    Color444(uint8_t r, uint8_t g, uint8_t b),
    Color888(uint8_t r, uint8_t g, uint8_t b),
    Color888(uint8_t r, uint8_t g, uint8_t b, boolean gflag),
    ColorHSV(long hue, uint8_t sat, uint8_t val, boolean gflag);

  uint16_t
    setRefresh(uint16_t freq);   // Set number of display updates per second
  uint16_t
    getRefresh();                // Return refresh frequency
#if defined(FADE)
  uint8_t
    swapFade(uint16_t tfade, boolean copy = false);
  boolean
    fading();   // true if fade is in progress
#endif

  int8_t
    copyBuffer(uint8_t from, uint8_t to); // Duplicate contents of one buffer to another

 private:

  uint8_t         *matrixbuff[nBuf];
  uint8_t          nRows;
  volatile uint8_t backindex,   // Page drawing on
    frontindex,                 // Page being displayed
    nextindex;                  // Next page to display
  volatile boolean swapflag;
  uint8_t          nPanels;
  
  // Init/alloc code common to both constructors:
  void init(uint8_t rows, uint8_t a, uint8_t b, uint8_t c,
    uint8_t sclk, uint8_t latch, uint8_t oe, boolean dbuf, uint8_t pwidth);

  // PORT register pointers, pin bitmasks, pin numbers:
  volatile uint8_t
    *latport, *oeport, *addraport, *addrbport, *addrcport, *addrdport;

  uint8_t
    sclkpin, latpin, oepin, addrapin, addrbpin, addrcpin, addrdpin,
    _sclk, _latch, _oe, _a, _b, _c, _d;

#if defined(FADE)
  volatile uint16_t FadeCnt, FadeLen; 
  volatile int16_t FadeNAccum; // replaces FadeNNext
  boolean copyflag;
#endif

#if defined(__TIVA__)
  volatile uint8_t *sclkport;
  uint16_t         refreshFreq;
  volatile uint32_t rowtime;      // time to display one row
#endif

  // Counters/pointers for interrupt handler:
  volatile uint8_t row, plane;
  volatile uint8_t *buffptr;
};

#endif
