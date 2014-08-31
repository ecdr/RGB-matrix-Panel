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


#if defined(__arm__)
#define SET_REFRESH
#endif

#include "RGBmatrixPanelConfig.h"


// Type for timer arguments
#if defined(__AVR__)
typedef uint16_t deltat_t;
#else
typedef uint32_t deltat_t;
#endif

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

// Internal (or interleaved) version of 5/6/5 color
typedef uint16_t crgb16i_t;


// Conversion from Adafruit color (5/6/5) to internal/interleaved color 

// Public

// Convert constants at compile time, do rest at run time
#define COLOR_INTERN(c) \
  ((__builtin_constant_p((c))) ? _COLORI_CONST((c)) : ColorIntern1((c)))

// Alternative version of interface - allows argument/type checking
//   since always inline, it should disappear at compile time when appropriate
// 
/*crgb16i_t ColorIntern(crgb16_t c) INLINE
{
  if (__builtin_constant_p(c))
    return _COLORI_CONST(c);
  else
    return ColorIntern1(c);
};*/


// Protected

extern crgb16i_t ColorIntern1(crgb16_t c);

#define BIT_TST_SET(fromvar, frombit, tobit) (((fromvar) & (1<<(frombit))) ? (1<<(tobit)))

// Convert c to internal color at compile time
// FIXME: Needs update to handle 5 bit color
#define _COLORI_CONST(c) ((crgb16i_t)\
  BIT_TST_SET(c, 12, 1)|  /* R0 */ \
  BIT_TST_SET(c, 13, 4)|  /* R1 */ \
  BIT_TST_SET(c, 14, 7)|  /* R2 */ \
  BIT_TST_SET(c, 15, 10)| /* R3 */ \
  BIT_TST_SET(c,  7, 2)|  /* G0 */ \
  BIT_TST_SET(c,  8, 5)|  /* G1 */ \
  BIT_TST_SET(c,  9, 8)|  /* G2 */ \
  BIT_TST_SET(c, 10, 11)| /* G3 */ \
  BIT_TST_SET(c,  1, 3)|  /* B0 */ \
  BIT_TST_SET(c,  2, 6)|  /* B1 */ \
  BIT_TST_SET(c,  3, 9)|  /* B2 */ \
  BIT_TST_SET(c,  4, 12))



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


  ~RGBmatrixPanel(void);

  void
    begin(void),
    end(void),
    drawPixel(int16_t x, int16_t y, crgb16_t c),
    fillScreen(crgb16_t c),     // fill current drawing buffer with color c
    swapBuffers(boolean copy = false),  // Display next buffer
    drawPixelI(int16_t x, int16_t y, crgb16i_t c), // Testing: same as drawpixel but interleaved color

    dumpMatrix(void) const;

  crgb16_t
    getPixel(int16_t x, int16_t y) const;


#if defined(COLORI_DRAW)
  // Use internal color representation to speed these up
  void
    drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, crgb16_t color),
    drawRect(int16_t x, int16_t y, int16_t w, int16_t h, crgb16_t color),
    fillRect(int16_t x, int16_t y, int16_t w, int16_t h, crgb16_t color),
// The ColorI, drawPixelI and I versions of line functions are used internally,
// it is not necesarry to call them in order to speed up drawing, 
// but they are left accessible to the user if desired.
    drawLineI(int16_t x0, int16_t y0, int16_t x1, int16_t y1, crgb16i_t color),
    drawFastVLineI(int16_t x, int16_t y, int16_t h, crgb16i_t color),
    drawFastHLineI(int16_t x, int16_t y, int16_t w, crgb16i_t color);
#endif

  int8_t
    loadBuffer(const uint8_t *img, uint16_t imgsize);

// Low level buffer access
  uint16_t
    bufferSize(void) const;    // Size of a buffer (in bytes)
  uint8_t
    *frontBuffer(void),
    *backBuffer(void);         // Return type should probably be void *

// Color
  crgb16_t
    Color333(uint8_t r, uint8_t g, uint8_t b) const,
    Color444(uint8_t r, uint8_t g, uint8_t b) const,
    Color888(uint8_t r, uint8_t g, uint8_t b) const,
    Color888(uint8_t r, uint8_t g, uint8_t b, boolean gflag) const,
    ColorHSV(long hue, uint8_t sat, uint8_t val, boolean gflag) const;

// Use ColorI to convert AdafruitGFX color to interleaved color, 
// then pass that as color argument of I functions.
  crgb16i_t ColorIntern(crgb16_t c) const; // Convert AdafruitGFX color to interleaved color

// TODO: Might be easier to have number of planes available as a constant (for macro color selection)?
  uint8_t
    bitsPerColor(void) const;

// Refresh frequency
#if defined(SET_REFRESH)
  uint16_t
    setRefresh(uint16_t freq),   // Set number of display updates per second
    getRefresh(void) const;      // Return refresh frequency
#endif

#if defined(FADE)
// Fade - transition one buffer to another
  uint8_t
    swapFade(uint16_t tfade, boolean copy = false);
  boolean
    fading(void) const;          // true if fade is in progress
#endif

#if defined(DIMMER)
  void
    setDim(uint32_t time);
  uint32_t getDim(void) const;  
#endif


// Display refresh function - need to be able to call from ISR, but not part of public interface

#if defined(__TIVA__)
 protected:
// Make interrupt handler a friend so can make updateDisplay protected
  friend void TmrHandler(void);

// TODO: test
  void updateDisplay(void);      // Dispaly refresh

#elif defined(__AVR__)

// FIXME: Need to figure out actual function name of ISR for arduino
// until then, just leave updateDisplay public
  void updateDisplay(void);      // Public because called by interrupt handler
#warning Would be nice to make Arduino timer ISR a friend

#else

#warning Unrecognized processor

  void updateDisplay(void);      // Public because called by interrupt handler

#endif


 private:

  uint8_t         *matrixbuff[nBuf];
  uint8_t          nRows;
  uint8_t          nPanels;

  volatile uint8_t backindex;

  volatile boolean swapflag;     // Page change pending

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

#if !defined(__AVR__)
  volatile uint8_t *sclkport;
#endif

#if defined(SET_REFRESH)
// Display refresh frequency
  uint16_t         refreshFreq;
  volatile deltat_t rowtime, newrowtime;
#endif

#if defined(DIMMER)
// Dimmer
  volatile deltat_t dimtime;
  boolean dimwait;
#endif

  // Counters/pointers for updateDisplay interrupt handler:
  volatile uint8_t row, plane;
  volatile uint8_t *buffptr;
};

#endif
