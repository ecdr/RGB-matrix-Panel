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


// FIXME: Including Adafruit_GFX.h here does not work in Energia.
// Workaround: Include Adafruit_GFX in the main sketch.
//
#include <Adafruit_GFX.h>
#include "RGBmatrixPanelConfig.h"

/*
// This might work in concept, but probably not in practice.
struct crgb16 {
    union {
      uint16_t c;
      struct { uint16_t r:5, g:6, b:5 }
    }
  };

typedef struct crgb16 crgb16_t;
*/

typedef uint16_t crgb16_t;


class RGBmatrixPanel : public Adafruit_GFX {

 public:
    
  // Constructor for 16x32 panel:
  RGBmatrixPanel(uint8_t a, uint8_t b, uint8_t c,
    uint8_t sclk, uint8_t latch, uint8_t oe, boolean dbuf, uint8_t pwidth = 1);
    /* Parameters
    a, b, c are the pins used for addressing the rows
    cclk, latch and oe are the pins used for Serial Clock, Latch and Output Enable
    dbuf enables double buffering. This will use 2x RAM for frame buffer, but will give nice smooth animation
    pwidth is the number of panels used together in a multi panel configuration
    */

  // Constructor for 32x32 panel (adds 'd' address pin)
  RGBmatrixPanel(uint8_t a, uint8_t b, uint8_t c, uint8_t d,
    uint8_t sclk, uint8_t latch, uint8_t oe, boolean dbuf, uint8_t pwidth);

// Compiler couldn't distinguish this one.
// TODO: Find whatever trick need to make the compiler count the arguments
//   There is only 1 prototype with 7 uin8_t followed by 1 boolean
//
//  RGBmatrixPanel(uint8_t a, uint8_t b, uint8_t c, uint8_t d,
//    uint8_t sclk, uint8_t latch, uint8_t oe, boolean dbuf);

  ~RGBmatrixPanel(void);

  void
    begin(void),
    stop(void),               // TODO: Maybe should be end (to match begin)?
    drawPixel(int16_t x, int16_t y, crgb16_t c),
    fillScreen(crgb16_t c),   // fill current drawing buffer with color c
    updateDisplay(void),      // Public because called by interrupt handler
    swapBuffers(boolean copy = false),  // Display next buffer
    drawPixelI(int16_t x, int16_t y, uint16_t c), // Testing: same as drawpixel but interleaved color

    dumpMatrix(void) const;
  int8_t
    loadBuffer(uint8_t *img, uint16_t imgsize);

// Low level buffer access
  uint16_t
    bufferSize(void) const;    // Size of a buffer (in bytes)
  uint8_t
    *frontBuffer(void),
    *backBuffer(void);   // Return type should probably be void *

// Color
  crgb16_t
    getPixel(int16_t x, int16_t y) const,
    Color333(uint8_t r, uint8_t g, uint8_t b) const,
    Color444(uint8_t r, uint8_t g, uint8_t b) const,
    Color888(uint8_t r, uint8_t g, uint8_t b) const,
    Color888(uint8_t r, uint8_t g, uint8_t b, boolean gflag) const,
    ColorHSV(long hue, uint8_t sat, uint8_t val, boolean gflag) const;
// TODO: Might be easier to have number of planes available as a constant (for macro color selection)?
  uint8_t
    bitsPerColor(void) const;

// Refresh frequency
  uint16_t
    setRefresh(uint16_t freq),   // Set number of display updates per second
    getRefresh() const;          // Return refresh frequency

#if defined(FADE)
// Fade - transition one buffer to another
  uint8_t
    swapFade(uint16_t tfade, boolean copy = false);
  boolean
    fading() const;   // true if fade is in progress
#endif

#if defined(__TIVA__)
  void
    setDim(uint32_t time);
  uint32_t getDim(void) const;  
#endif

 private:

  uint8_t         *matrixbuff[nBuf];
  uint8_t          nRows;
  uint8_t          nPanels;

  volatile uint8_t backindex;

  volatile boolean swapflag;    // Page change pending

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
  boolean copyflag;             // Copy buffer at end of fade
#endif

#if defined(__TIVA__)
  volatile uint8_t *sclkport;

// Refresh frequency
  uint16_t         refreshFreq;
  volatile uint32_t rowtime, newrowtime;

// Dimmer
  volatile uint32_t dimtime;
  boolean dimwait;
#endif

  // Counters/pointers for updateDisplay interrupt handler:
  volatile uint8_t row, plane;
  volatile uint8_t *buffptr;
};

#endif
