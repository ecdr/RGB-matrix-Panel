// fade, multiple buffer demo for Adafruit RGBmatrixPanel library.
// image from flash memory demo for Adafruit RGBmatrixPanel library.

#if defined(__AVR__)
#include <avr/pgmspace.h>
#else
#define PROGMEM 
#define pgm_read_byte( a ) (*(a))
#endif

#include <Adafruit_GFX.h>   // Core graphics library
#include <RGBmatrixPanel.h> // Hardware-specific library

#if defined(__AVR__)

#define CLK 8
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

#elif defined( __TM4C1294NCPDT__ )

#define CLK PM_1
#define LAT PM_2
#define OE  PM_0
#define A   PQ_3
#define B   PP_3
#define C   PH_1
#define D   PH_0

#endif


// Last parameter = 'true' enables double-buffering, for flicker-free,
// buttery smooth animation.  Note that NOTHING WILL SHOW ON THE DISPLAY
// until the first call to swapBuffers().  This is normal.
RGBmatrixPanel matrix(A, B, C, CLK, LAT, OE, true);

//#include "image.h"


// TODO: Adapt Energia timer functions to fill in these pseudo-functions
typedef uint32_t TIME_T;

// Time/timer operations
// Get current timer time
TIME_T gettime(void);

// Convert from time in ms to time in timer units
TIME_T msToTimet(uint16_t timems);

// Delay until a given time
int8_t delayuntil(TIME_T);


// returned color cresult : c2 - c1 :: r1 : r2
uint16_t interpolateColor(uint16_t c1, uint16_t c2, uint16_t r1, uint16_t r2);
//RRRRRGGGGGgBBBBB 

// Fade from front to next in time tFade (ms)
// bufTmp1, 2 are temporary for interpolated images
uint8_t myFade(uint8_t bufTmp1, uint8_t bufTmp2, uint16_t tFade, uint8_t nSteps){
  uint8_t step;
  uint16_t x, y;

  TIME_T tstart = gettime();               // When started showing previous image
  TIME_T tStep = msToTimet(tFade / nSteps);
  uint8_t bufFrom = matrix.getFront();
  uint8_t bufTo = matrix.getNext();
  uint8_t myNxtBuf = bufTmp1;
  
  // Range checks
  if ((bufFrom == nBuf) || (bufTo == nBuf) || (bufTmp1 >= nBuf) || 
    (bufTmp2 >= nBuf))
    return 1;

  for (step = 0; step < nSteps - 1; step++){
    // Calculate next intermediate image
    matrix.setDraw(myNxtBuf);
    for (y = 0; y < matrix.width(); y++)        // For every pixel
      for (x = 0; x < matrix.height(); x++)
        matrix.drawPixel(x, y, 
          interpolateColor(matrix.getPixel(x, y, bufFrom), 
            matrix.getPixel(x, y, bufTo), step, nSteps) );
    matrix.setNext();

    // Wait until time to show the image    
    delayuntil(tstart + tStep);
    matrix.swapBuffers();           // Or could try FadeTo

    // Switch to other temp buffer (show one while generate the next)
    myNxtBuf = ((myNxtBuf == bufTmp1) ? bufTmp2 : bufTmp1);
  }
  matrix.setNext(bufTo);          // Last time - display final buffer
  delayuntil(tstart + tStep);   // Wait until time to show the final buffer
  matrix.swapBuffers();           // Or could try FadeTo
  return 0;
}

// Convert 5/6/5 to 8/8/8 color
//RRRRRGGGGGgBBBBB  -> RRRRR000, GGGGGg00, BBBBB000

#define GETRGB(c, r, g, b) {r = (c >> 8) & 0xF8; g = (c >> 3) & 0xFC ; b = (c << 3) & 0xF8;}


// This will probably be really slow.
// Might be able to speed up by caching current value and delta value for each color for each pixel
//   (might take fair ammount of space).
// Simple speedup would be to cache ratdif, since that is constant for many calls
uint16_t interpolateColor(uint16_t c1, uint16_t c2, uint16_t rat1, uint16_t rat2){
  //result = c1 + rat1/rat2 * (c2 - c1);
  uint16_t r1, g1, b1, r2, g2, b2;
  uint8_t rr, gr, br;
  GETRGB(c1, r1, g1, b1);
  GETRGB(c2, r2, g2, b2);
  uint16_t ratdif = rat2 - rat1;
//  rr = r1 + ((r2 - r1) * rat1)/rat2;
  rr = (r2 * rat1 + r1 * ratdif)/rat2;
  gr = (g2 * rat1 + g1 * ratdif)/rat2;
  br = (b2 * rat1 + b1 * ratdif)/rat2;
  return matrix.Color888(rr,gr,br);
}


void setup() {
//  matrix.setBufferPtr(4, img, sizeof(img));
#warning need to draw something
  matrix.begin();
  matrix.setNext(4);
  matrix.swapBuffers(false);
  delay(1000);

}


void loop() {

  myFade(2, 3, 10000, 2);

//  matrix.swapBuffers(false);
}
