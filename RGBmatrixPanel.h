/*
RGBmatrixPanel library for 16x32 and 32x32 RGB LED matrix panels.
*/


#ifndef RGBmatrixPanel_h
#define RGBmatrixPanel_h

#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
 #include "pins_arduino.h"
#endif


#include "Adafruit_GFX.h"
#include "RGBmatrixPanelConfig.h"



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
// TODO: Find whatever trick need to make the compiler count the arguments
//   There is only 1 prototype with 7 uin8_t followed by 1 boolean
//
//  RGBmatrixPanel(uint8_t a, uint8_t b, uint8_t c, uint8_t d,
//    uint8_t sclk, uint8_t latch, uint8_t oe, boolean dbuf);

  void
    begin(void),              // Start display
    stop(void),               // Stop display
    drawPixel(int16_t x, int16_t y, uint16_t c),
    fillScreen(uint16_t c),   // fill current drawing buffer with color c
    updateDisplay(void),      // TODO: Why is updateDisplay public?
    swapBuffers(boolean copy = false),  // Display next buffer
    // swapBuffers(uint8_t nextPage, boolean copy),
    dumpMatrix(void),
    dumpMatrix(uint8_t buf);
  int8_t
    loadBuffer(uint8_t *img, uint16_t imgsize);
  uint8_t
    setDraw(uint8_t back),
    setNext(uint8_t next),  // Select buffer that will be displayed next (by swap)
    setNext(void),          // Select drawing buffer as next
    getDraw(void),
    getNext(void),
    getFront(void);
  uint8_t
    *frontBuffer(void),
    *backBuffer(void),
    *buffer(uint8_t buf);
  uint16_t
    getPixel(int16_t x, int16_t y),
    getPixel(uint8_t buf, int16_t x, int16_t y),
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
    swapFade(uint16_t tfade),    // Fade to next, no copy
    swapFade(uint8_t nextPage, uint16_t tfade, boolean copy);
  boolean
    fading();   // true if fade is in progress
#endif
#if defined(__TIVA__)
  void
    setDim(uint32_t time);
#endif

  int8_t
    copyBuffer(uint8_t from, uint8_t to), // Duplicate contents of one buffer to another
    copyBuffer(uint8_t from),
    copyBuffer(void);

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
  volatile uint16_t FadeLen; 
  volatile int32_t FadeNAccum, FadeCnt;
  boolean copyflag;
#endif

#if defined(__TIVA__)
  volatile uint8_t *sclkport;
  uint16_t         refreshFreq;
  volatile uint32_t rowtime;      // time to display one row

  volatile uint32_t dimtime;
  boolean dimwait;
#endif

  // Counters/pointers for interrupt handler:
  volatile uint8_t row, plane;
  volatile uint8_t *buffptr;
};

#endif
