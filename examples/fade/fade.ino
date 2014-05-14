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


#define CLK 8
#define LAT A3
#define OE  9
#define A   A0
#define B   A1
#define C   A2
// Last parameter = 'true' enables double-buffering, for flicker-free,
// buttery smooth animation.  Note that NOTHING WILL SHOW ON THE DISPLAY
// until the first call to swapBuffers().  This is normal.
RGBmatrixPanel matrix(A, B, C, CLK, LAT, OE, true);

//#include "image.h"


void setup() {
//  matrix.setBufferPtr(4, img, sizeof(img));
#warning need to draw something
  matrix.begin();
  matrix.setNext(4);
  matrix.swapBuffers(false);
  delay(1000);

}

// Try using multiple buffers

typedef uint32_t TIME_T;

TIME_T gettime(void);
int8_t delayuntil(TIME_T);


uint16_t interpolateColor(uint16_t c1, uint16_t c2, uint16_t r1, uint16_t r2);


// Fade from front to next in time tFade (ms)
// bufTmp1, 2 are temporary for interpolated images
uint8_t myFade(uint8_t bufTmp1, uint8_t bufTmp2, uint16_t tFade, uint8_t nSteps){
  uint8_t step;
  uint16_t x, y;

  TIME_T tstart = gettime();               // When started showing previous image
  uint16_t tStep = tFade / nSteps;
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

    // Switch to other temp buffer
    myNxtBuf = ((myNxtBuf == bufTmp1) ? bufTmp2 : bufTmp1);
  }
  matrix.setNext(bufTo);          // Last time - display final buffer
  delayuntil(tstart + tStep);   // Wait until time to show the final buffer
  matrix.swapBuffers();           // Or could try FadeTo
  return 0;
}


void loop() {

  myFade(2, 3, 10000, 2);

//  matrix.swapBuffers(false);
}
