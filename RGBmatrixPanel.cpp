/*
RGBmatrixPanel Arduino library for Adafruit 16x32 and 32x32 RGB LED
matrix panels.  Pick one up at:
  http://www.adafruit.com/products/420
  http://www.adafruit.com/products/607

This version uses a few tricks to achieve better performance and/or
lower CPU utilization:

- To control LED brightness, traditional PWM is eschewed in favor of
  Binary Code Modulation, which operates through a succession of periods
  each twice the length of the preceeding one (rather than a direct
  linear count a la PWM).  It's explained well here:

    http://www.batsocks.co.uk/readme/art_bcm_1.htm

  I was initially skeptical, but it works exceedingly well in practice!
  And this uses considerably fewer CPU cycles than software PWM.

- Although many control pins are software-configurable in the user's
  code, a couple things are tied to specific PORT registers.  It's just
  a lot faster this way -- port lookups take time.  Please see the notes
  later regarding wiring on "alternative" Arduino boards.

- A tiny bit of inline assembly language is used in the most speed-
  critical section.  The C++ compiler wasn't making optimal use of the
  instruction set in what seemed like an obvious chunk of code.  Since
  it's only a few short instructions, this loop is also "unrolled" --
  each iteration is stated explicitly, not through a control loop.

Written by Limor Fried/Ladyada & Phil Burgess/PaintYourDragon for
Adafruit Industries.
BSD license, all text above must be included in any redistribution.

Revisions:
    getPixel, by RobF42 - Rob Fugina
    daisychain displays, by protonmaster - Phillip Burgess
    Tiva Launchpad support, 
*/

#include "RGBmatrixPanel.h"
#include "gamma.h"


#if !defined(__AVR__)
#define pgm_read_byte( a ) (*(a))
#endif


#if defined(__TIVA__)

#include "wiring_private.h"

#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/gpio.h"
#include "driverlib/rom_map.h"

#include "driverlib/sysctl.h"
#include "inc/hw_ints.h"
#include "driverlib/interrupt.h"
#include "inc/hw_timer.h"
#include "driverlib/timer.h"


// On Tiva, define BENCHMARK to measure TimerHandler time
//#define BENCHMARK

#if defined( BENCHMARK )
#include "cyclecount.h"
#endif

// Define UNROLL_LOOP to linearize the inner loop in display
//#define UNROLL_LOOP

// portOutputRegister(port) not defined for Tiva, so make up own version
// include port mask so do not have to do read modify write
// can just write to the address, without changing other pins

// For reference:
//  void digitalWrite(uint8_t pin, uint8_t val) =
//  HWREG(portBASERegister(digitalPinToPort(pin)) + (GPIO_O_DATA + (digitalPinToBitMask(pin) << 2))) = val ? 0xFF : 0;
//  HWREGB should work instead (since all ports are 8 bits max)
//                                     0  
//#define HWREG(x)   (*((volatile uint32_t *)(x)))
//#define HWREGB(x)  (*((volatile uint8_t  *)(x)))

// TODO: Consider bitband version, for 1 bit control lines
// BITBAND 
/*#define HWREGBITB(x, b)                                                       \
        HWREGB(((uint32_t)(x) & 0xF0000000) | 0x02000000 |                    \
               (((uint32_t)(x) & 0x000FFFFF) << 5) | ((b) << 2))
 */


// Caution - be careful of adding masks to pointer types.
// Cast the portBASERegister back to uint32_t before add offset
// Otherwise it will do a clandestine left shift 2 on the mask
// Alternative would be to make use of that left shift, 
// but then need a big note explaining the occult behavior

#define portMaskedOutputRegister(port, mask) \
  ((volatile uint8_t *) (((uint32_t)portBASERegister(port)) + (GPIO_O_DATA + (((uint32_t)mask) << 2))))


// Port/pin definitions for various launchpads
#if defined(__TM4C129XNCZAD__)

#elif defined(__TM4C1294NCPDT__)
// Tiva Connected Launchpad

// Candidates for data port: 
//   PortE (BP1), PortK (BP2), or PortL (BP1) (pins 0-5)

#define DATAPORTMASK  B11111100
#define DATAPORT      (*portMaskedOutputRegister(PK, DATAPORTMASK))
#define DATAPORTBASE  ((uint32_t)portBASERegister(PK))

// SCLK pin must be on port defined here
//#define SCLKPORT      (*portDATARegister(PM))

#elif defined(__LM4F120H5QR__) || defined(__TM4C123GH6PM__)
// Stellaris or Tiva Launchpad

// Data port should be PA or PB, 
// PA: Do not use 0,1 - console Uart
// PB: use caution in other pin assignments, 
//     since PB6 and PB7 are connected to PD0 and PD1

#define DATAPORTMASK  B11111100
#define DATAPORT      (*portMaskedOutputRegister(PA, DATAPORTMASK))
#define DATAPORTBASE  ((uint32_t)portBASERegister(PA))

// Used in testing - show data on onboard LED
//#define DATAPORT      (*portMaskedOutputRegister(PF, DATAPORTMASK))
//#define DATAPORTBASE  ((uint32_t)portBASERegister(PF))


//#define SCLKPORT      (*portDATARegister(PB))

#endif


// Timer selection

// Note: 
// Timer 4A is used by Tone
// Timer 5 is used by Energia time tracking

// Energia doesn't use all the timers, maybe better to just grab one not used
//   WTIMER4 or WTIMER5 on LP, TIMER6 or TIMER7 on Connected LP

// Launchpad: uses regular timer 0-3, wide timer 0-3, 6, 7
//     Thus wide timers 4 and 5 appear to be available.
// Connected LP: Energia uses timers 0-5 (so 6 and 7 should be available?)

//  TODO: Check Energia timer use for timers 4 and 5 on Connected LP
//    Seems strange that, on Connected LP, Timers 4 and 5 are used both by tone/Energia and by PWM??
//   Could select timer that does PWM for a pin that is otherwise in use.


#if defined(__TM4C1294NCPDT__)

#define TIMER TIMER6

#else

#define TIMER WTIMER4

// Can use timer or wide timer
// For regular timer, uses full width timer
// For wide timer, uses timer A

// TODO: could adapt code to allow use of timer B of wide timer

#endif


// FIXME: Add masking to sclkport?
//   Either make sclkpin a constant (so can use constant port masked version of SCLKPORT), 
//   or make sclkport a variable (so can fill in masked version of sclkport)
//   (Then fix up the tick/tock code appropriately)
// TODO: Could use sclkpin instead if made smaller/faster code

#define SCLKPORT        (*sclkport)


#else 

// On AVRs:
// A full PORT register is required for the data lines, though only the
// top 6 output bits are used.  For performance reasons, the port # cannot
// be changed via library calls, only by changing constants in the library.
// For similar reasons, the clock pin is only semi-configurable...it can
// be specified as any pin within a specific PORT register stated below.
#define DATAPORTMASK B11111100


#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
 // Arduino Mega is now tested and confirmed, with the following caveats:
 // Because digital pins 2-7 don't map to a contiguous port register,
 // the Mega requires connecting the matrix data lines to different pins.
 // Digital pins 24-29 are used for the data interface, and 22 & 23 are
 // unavailable for other outputs because the software needs to write to
 // the full PORTA register for speed.  Clock may be any pin on PORTB --
 // on the Mega, this CAN'T be pins 8 or 9 (these are on PORTH), thus the
 // wiring will need to be slightly different than the tutorial's
 // explanation on the Uno, etc.  Pins 10-13 are all fair game for the
 // clock, as are pins 50-53.
 #define DATAPORT PORTA
 #define DATADIR  DDRA
 #define SCLKPORT PORTB
 
#elif defined(__SAM3X8E__)
 // Arduino Due uses the SAM3X8E.
 #define DATAPORT PORTC
 #define DATADIR  tobedefined
 #define SCLKPORT tobedefined  

 #warning Arduino Due not finished

#elif defined(__AVR_ATmega32U4__)
 // Arduino Leonardo: this is vestigial code an unlikely to ever be
 // finished -- DO NOT USE!!!  Unlike the Uno, digital pins 2-7 do NOT
 // map to a contiguous port register, dashing our hopes for compatible
 // wiring.  Making this work would require significant changes both to
 // the bit-shifting code in the library, and how this board is wired to
 // the LED matrix.  Bummer.
 #define DATAPORT PORTD
 #define DATADIR  DDRD
 #define SCLKPORT PORTB
#else
 // Ports for "standard" boards (Arduino Uno, Duemilanove, etc.)
 #define DATAPORT PORTD
 #define DATADIR  DDRD
 #define SCLKPORT PORTB
#endif

#endif


static const uint8_t nPlanes = 4;
static const uint8_t BYTES_PER_ROW = 32;
static const uint8_t nPackedPlanes = 3;  // 3 bytes holds 4 planes "packed"


#if defined(__TIVA__)

static const uint16_t defaultRefreshFreq = 100; // Cycles per second 
  // (200 should work for 1 16 row panel)
//const uint32_t ticksPerSecond = 1000000; // Number of timer ticks in 1 second

#endif


#if defined(__TIVA__)

#ifdef __TM4C1294NCPDT__
// TM4C1294 - can only get clock speed from clock set call
#define TIMER_CLK F_CPU
#else

// SysCtlClockGet only works on TM4C123x
//#define TIMER_CLK SysCtlClockGet()
// However there is a bug in SysCtlClockGet() - in Tivaware 2.1...., 
// SysCtlClockGet incorrect if request 80MHz clock 
// So just bypass it for now

#define TIMER_CLK F_CPU


#endif


/*
Given name of a timer, assemble names of the various associated constants.
  BASE(TIMER0) =>  TIMER0_BASE
  SYSCTL_PERIPH_TIMER0
  INT_TIMER0A
*/

// Auxiliary macros (to get substitution to happen correctly)
#define BASE1(t)   (t##_BASE)
#define SYSCTL1(t) (SYSCTL_PERIPH_##t)
#define INTA1(t)   (INT_##t##A)
#define INTB1(t)   (INT_##t##B)

#define BASE(t)    BASE1(t)
#define SYSCTL(t)  SYSCTL1(t)
#define INTA(t)    INTA1(t)
#define INTB(t)    INTB1(t)


// Actually assemble the timer macro names

#define TIMER_BASE   BASE(TIMER)
#define TIMER_SYSCTL SYSCTL(TIMER)
#define TIMER_INT    INTA(TIMER)


// extern "C" {
//void enableTimerPeriph(uint32_t offset);
//}

void TmrHandler(void);

#endif


// Todo: Allow multiple displays (share data and address, separate OE)

// The fact that the display driver interrupt stuff is tied to the
// singular Timer1 doesn't really take well to object orientation with
// multiple RGBmatrixPanel instances.  The solution at present is to
// allow instances, but only one is active at any given time, via its
// begin() method.  The implementation is still incomplete in parts;
// the prior active panel really should be gracefully disabled, and a
// stop() method should perhaps be added...assuming multiple instances
// are even an actual need.
static RGBmatrixPanel *activePanel = NULL;


// -------------------- Constructors  --------------------

// Code common to both the 16x32 and 32x32 constructors:
void RGBmatrixPanel::init(uint8_t rows, uint8_t a, uint8_t b, uint8_t c,
  uint8_t sclk, uint8_t latch, uint8_t oe, boolean dbuf, uint8_t pwidth) {

  nRows = rows; // Number of multiplexed rows; actual height is 2X this
  nPanels = pwidth;

  // Allocate and initialize matrix buffer:
  int buffsize  = BYTES_PER_ROW * nRows * nPackedPlanes * nPanels, // x3 = 3 bytes holds 4 planes "packed"
      allocsize = (dbuf == true) ? (buffsize * nBuf) : buffsize;
  if(NULL == (matrixbuff[0] = (uint8_t *)malloc(allocsize))) return;
  memset(matrixbuff[0], 0, allocsize);
  // If not double-buffered, both buffers then point to the same address:
  matrixbuff[1] = (dbuf == true) ? &matrixbuff[0][buffsize] : matrixbuff[0];
// FIXME: Adapt for nBuf > 2

  // Save pin numbers for use by begin() method later.
  _a     = a;
  _b     = b;
  _c     = c;
  _sclk  = sclk;
  _latch = latch;
  _oe    = oe;

  // Look up port registers and pin masks ahead of time,
  // avoids many slow digitalWrite() calls later.
#if defined(__TIVA__)

// Tiva Energia does not provide portOutputRegister macro
  sclkpin   = digitalPinToBitMask(sclk);
  sclkport  = portMaskedOutputRegister(digitalPinToPort(sclk), sclkpin);
  
  latpin    = digitalPinToBitMask(latch);
  latport   = portMaskedOutputRegister(digitalPinToPort(latch), latpin);

  oepin     = digitalPinToBitMask(oe);
  oeport    = portMaskedOutputRegister(digitalPinToPort(oe), oepin);

  addrapin  = digitalPinToBitMask(a);
  addraport = portMaskedOutputRegister(digitalPinToPort(a), addrapin);

  addrbpin  = digitalPinToBitMask(b);
  addrbport = portMaskedOutputRegister(digitalPinToPort(b), addrbpin);

  addrcpin  = digitalPinToBitMask(c); 
  addrcport = portMaskedOutputRegister(digitalPinToPort(c), addrcpin);

#else
  sclkpin   = digitalPinToBitMask(sclk);
  latport   = portOutputRegister(digitalPinToPort(latch));
  latpin    = digitalPinToBitMask(latch);
  oeport    = portOutputRegister(digitalPinToPort(oe));
  oepin     = digitalPinToBitMask(oe);
  addraport = portOutputRegister(digitalPinToPort(a));
  addrapin  = digitalPinToBitMask(a);
  addrbport = portOutputRegister(digitalPinToPort(b));
  addrbpin  = digitalPinToBitMask(b);
  addrcport = portOutputRegister(digitalPinToPort(c));
  addrcpin  = digitalPinToBitMask(c); 
#endif  // __TIVA__

  plane     = nPlanes - 1;
  row       = nRows   - 1;
  swapflag  = false;
  backindex = 0;     // Array index of back buffer

#if defined(FADE)
  FadeCnt = 0;
  FadeNAccum = 0;
//  FadeNNext = 0;
  FadeLen = 0;
  copyflag = false;
#endif // FADE
}

// Constructor for 16x32 panel:
RGBmatrixPanel::RGBmatrixPanel(
  uint8_t a, uint8_t b, uint8_t c,
  uint8_t sclk, uint8_t latch, uint8_t oe, boolean dbuf, uint8_t pwidth) :
  Adafruit_GFX(BYTES_PER_ROW*pwidth, 16) {

  init(8, a, b, c, sclk, latch, oe, dbuf, pwidth);
}

// Constructor for 32x32 panel:
RGBmatrixPanel::RGBmatrixPanel(
  uint8_t a, uint8_t b, uint8_t c, uint8_t d,
  uint8_t sclk, uint8_t latch, uint8_t oe, boolean dbuf, uint8_t pwidth) :
  Adafruit_GFX(BYTES_PER_ROW*pwidth, 32) {

  init(16, a, b, c, sclk, latch, oe, dbuf, pwidth);

  // Init a few extra 32x32-specific elements:
  _d        = d;
  addrdpin  = digitalPinToBitMask(d);

#if defined(__TIVA__)
  addrdport = portMaskedOutputRegister(digitalPinToPort(d), addrdpin);
#else
  addrdport = portOutputRegister(digitalPinToPort(d));
#endif
}

/* Compiler complained about this one
// Constructor for 32x32 panel: - only 1 panel
RGBmatrixPanel::RGBmatrixPanel(
  uint8_t a, uint8_t b, uint8_t c, uint8_t d,
  uint8_t sclk, uint8_t latch, uint8_t oe, boolean dbuf) :
  Adafruit_GFX(BYTES_PER_ROW*pwidth, 32) {

  init(16, a, b, c, sclk, latch, oe, dbuf, 1);

  // Init a few extra 32x32-specific elements:
  _d        = d;
  addrdport = portOutputRegister(digitalPinToPort(d));
  addrdpin  = digitalPinToBitMask(d);
}
*/


void RGBmatrixPanel::begin(void) {

#if defined(DEBUG) || defined( BENCHMARK )
  Serial.begin(9600);
#endif

#if defined(DEBUG)

  // prints title with ending line break
  Serial.println("RGBMatrix:begin");

  Serial.print("DATAPORT:");
  Serial.print((uint32_t)&DATAPORT, HEX);

#if defined(__TIVA__)  
  Serial.print(", DATAPORTMASK:");
  Serial.print(DATAPORTMASK, HEX);
  Serial.print(", DATA port BASE:");
  Serial.println((uint32_t)DATAPORTBASE, HEX);

  Serial.print("DPB+MASK");
  Serial.print((uint32_t)(((uint32_t *)DATAPORTBASE)+(DATAPORTMASK)), HEX);
#endif

  Serial.print("SCLKPORT ");
  Serial.print((uint32_t)&SCLKPORT, HEX);
  Serial.print("sclkpin ");
  Serial.println(sclkpin, HEX);

  Serial.print("Addraport ");
  Serial.print((uint32_t)addraport, HEX);
  Serial.print("Addrapin ");
  Serial.println(addrapin, HEX);

  Serial.print("Addrbport ");
  Serial.print((uint32_t)addrbport, HEX);
  Serial.print("Addrbpin ");
  Serial.println(addrbpin, HEX);

  Serial.print("Addrcport ");
  Serial.print((uint32_t)addrcport, HEX);
  Serial.print("Addrcpin ");
  Serial.println(addrcpin, HEX);

if(nRows > 8) {
  Serial.print("Addrdport ");
  Serial.print((uint32_t)addrdport, HEX);
  Serial.print("Addrdpin ");
  Serial.println(addrdpin, HEX);
}
  Serial.print("Latport ");
  Serial.print((uint32_t)latport, HEX);
  Serial.print("latpin ");
  Serial.println(latpin, HEX);

  Serial.print("oeport ");
  Serial.print((uint32_t)oeport, HEX);
  Serial.print("oepin ");
  Serial.println(oepin, HEX);
#endif

  backindex   = 0;                         // Back buffer
  buffptr     = matrixbuff[1 - backindex]; // -> front buffer
// FIXME: Adapt for nBuf > 2  
  activePanel = this;                      // For interrupt hander

  // Enable all comm & address pins as outputs, set default states:
  pinMode(_sclk , OUTPUT); SCLKPORT   &= ~sclkpin;  // Low
  pinMode(_latch, OUTPUT); *latport   &= ~latpin;   // Low
  pinMode(_oe   , OUTPUT); *oeport    |= oepin;     // High (disable output)
  pinMode(_a    , OUTPUT); *addraport &= ~addrapin; // Low
  pinMode(_b    , OUTPUT); *addrbport &= ~addrbpin; // Low
  pinMode(_c    , OUTPUT); *addrcport &= ~addrcpin; // Low
  if(nRows > 8) {
    pinMode(_d  , OUTPUT); *addrdport &= ~addrdpin; // Low
  }

  // The high six bits of the data port are set as outputs;
  // Might make this configurable in the future, but not yet.
#if defined(__TIVA__)
  MAP_GPIOPinTypeGPIOOutput( DATAPORTBASE, DATAPORTMASK );
  MAP_GPIOPinWrite(DATAPORTBASE, DATAPORTMASK, 0);
#else
  DATADIR  = DATAPORTMASK;
  DATAPORT = 0;
#endif  

// Timer setup
#if defined(BENCHMARK)
  EnableTiming();
#endif

#if defined(__TIVA__)
  setRefresh(defaultRefreshFreq);

#if defined(DEBUG)
  
  Serial.print("Rowtime ");
  Serial.println(rowtime);

#endif

  MAP_SysCtlPeripheralEnable( TIMER_SYSCTL );

//  MAP_TimerDisable( TIMER_BASE, TIMER_A );

// For wide timer, use half timer
// This assumes that WTIMER bases all come after regular timer bases
//  (which is true in TM4C123x, but need not be true in general)
#if ( TIMER_BASE >= WTIMER0_BASE )
#define TIMER_CFG   (TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_ONE_SHOT)
#else
#define TIMER_CFG   TIMER_CFG_ONE_SHOT
#endif

  MAP_TimerConfigure( TIMER_BASE, TIMER_CFG );

  IntRegister( TIMER_INT, TmrHandler );
  MAP_IntMasterEnable();

  MAP_IntEnable( TIMER_INT );
  MAP_TimerIntEnable( TIMER_BASE, TIMER_TIMA_TIMEOUT );

  MAP_TimerLoadSet( TIMER_BASE, TIMER_A, rowtime );  // Dummy initial interrupt period
  
  MAP_TimerEnable( TIMER_BASE, TIMER_A );

#if defined(DEBUG)
  Serial.println("Timer setup");
#endif

#else   //#elif defined(__AVR__)

  // Set up Timer1 for interrupt:
  TCCR1A  = _BV(WGM11); // Mode 14 (fast PWM), OC1A off
  TCCR1B  = _BV(WGM13) | _BV(WGM12) | _BV(CS10); // Mode 14, no prescale
  ICR1    = 100;
  TIMSK1 |= _BV(TOIE1); // Enable Timer1 interrupt
  sei();                // Enable global interrupts
#endif
}

/* Notes:
#define timerToTimeout(timer)  (TIMER_TIMA_TIMEOUT << timerToAB(timer))

  uint32_t timerBase = getTimerBase(timerToOffset(TIMER));
  uint32_t timerAB = TIMER_A << timerToAB(TIMER);

// Timer setup

  uint32_t final = ( ( uint64_t )period_us * TIMER_CLK ) / ticksPerSecond;
  MAP_TimerDisable( timerBase, timerAB );
  MAP_TimerIntClear( timerBase, timerToTimeout(TIMER) );
  MAP_TimerLoadSet( timerBase, timerAB, ( uint32_t )final - 1 );
 
*/  


void RGBmatrixPanel::stop(void) {
#if defined(__TIVA__)
// Stop timer
  
  MAP_TimerIntDisable(TIMER_BASE, TIMER_TIMA_TIMEOUT);
  MAP_TimerIntClear(TIMER_BASE, TIMER_TIMA_TIMEOUT);
  MAP_TimerDisable(TIMER_BASE, TIMER_A);
  IntUnregister(TIMER_INT);
#else

#endif  // __TIVA__
// TODO: Should probably send all 0's to the panel

  activePanel = NULL;
}

  
// -------------------- Color  --------------------


// Original RGBmatrixPanel library used 3/3/3 color.  Later version used
// 4/4/4.  Then Adafruit_GFX (core library used across all Adafruit
// display devices now) standardized on 5/6/5.  The matrix still operates
// internally on 4/4/4 color, but all the graphics functions are written
// to expect 5/6/5...the matrix lib will truncate the color components as
// needed when drawing.  These next functions are mostly here for the
// benefit of older code using one of the original color formats.

// Promote 3/3/3 RGB to Adafruit_GFX 5/6/5
uint16_t RGBmatrixPanel::Color333(uint8_t r, uint8_t g, uint8_t b) {
  // RRRrrGGGgggBBBbb
  return ((r & 0x7) << 13) | ((r & 0x6) << 10) |
         ((g & 0x7) <<  8) | ((g & 0x7) <<  5) |
         ((b & 0x7) <<  2) | ((b & 0x6) >>  1);
}

// Promote 4/4/4 RGB to Adafruit_GFX 5/6/5
uint16_t RGBmatrixPanel::Color444(uint8_t r, uint8_t g, uint8_t b) {
  // RRRRrGGGGggBBBBb
  return ((r & 0xF) << 12) | ((r & 0x8) << 8) |
         ((g & 0xF) <<  7) | ((g & 0xC) << 3) |
         ((b & 0xF) <<  1) | ((b & 0x8) >> 3);
}

// Demote 8/8/8 to Adafruit_GFX 5/6/5
// If no gamma flag passed, assume linear color
uint16_t RGBmatrixPanel::Color888(uint8_t r, uint8_t g, uint8_t b) {
  return ((r & 0xF8) << 11) | ((g & 0xFC) << 5) | (b >> 3);
}

// TODO: Seems it would be simpler to just use full 6-bit gamma output 
// (or possibly 8 bit gamma), and convert from 6/6/6 or 8/8/8 to 5/6/5
// (Fewer operations to handle made-up bits)
// Also means less adapting for greater bit depth display

// 8/8/8 -> gamma -> 5/6/5
uint16_t RGBmatrixPanel::Color888(
  uint8_t r, uint8_t g, uint8_t b, boolean gflag) {
  if(gflag) { // Gamma-corrected color?
    r = pgm_read_byte(&gamma_table[r]); // Gamma correction table maps
    g = pgm_read_byte(&gamma_table[g]); // 8-bit input to 4-bit output
    b = pgm_read_byte(&gamma_table[b]);
//#if (nPlanes == 4)
    return (r << 12) | ((r & 0x8) << 8) | // 4/4/4 -> 5/6/5
           (g <<  7) | ((g & 0xC) << 3) |
           (b <<  1) | ( b        >> 3);
/*#elif (nPlanes == 5)  // Requires 5 bit gamma
    return (r << 11) |                    // 5/5/5 -> 5/6/5
           (g <<  6) | ((g & 0x10) << 1) |
           (b) ;
#else
#error Unsupported number of planes
#endif
*/
  } // else linear (uncorrected) color
  return ((r & 0xF8) << 11) | ((g & 0xFC) << 5) | (b >> 3);
}

// TODO: Color mapping extends a color by repeating the upper few bits of the color in the low bits.  
// Why?
// 000R, 000G, 000B -> R000 | r[4]000:00 | 0:0ggg:g000:0000 | g[3:2]00:000 | b:bbb0 | b[3]
// Resulting bits are: R4R3R2R1R4 G4G3G2G1G4G3 B4B3B2B1B4  (R4 means bit 4 of R)

uint16_t RGBmatrixPanel::ColorHSV(
  long hue, uint8_t sat, uint8_t val, boolean gflag) {

  uint8_t  r, g, b, lo;
  uint16_t s1, v1;

  // Hue
  hue %= 1536;             // -1535 to +1535
  if(hue < 0) hue += 1536; //     0 to +1535
  lo = hue & 255;          // Low byte  = primary/secondary color mix
  switch(hue >> 8) {       // High byte = sextant of colorwheel
    case 0 : r = 255     ; g =  lo     ; b =   0     ; break; // R to Y
    case 1 : r = 255 - lo; g = 255     ; b =   0     ; break; // Y to G
    case 2 : r =   0     ; g = 255     ; b =  lo     ; break; // G to C
    case 3 : r =   0     ; g = 255 - lo; b = 255     ; break; // C to B
    case 4 : r =  lo     ; g =   0     ; b = 255     ; break; // B to M
    default: r = 255     ; g =   0     ; b = 255 - lo; break; // M to R
  }

  // Saturation: add 1 so range is 1 to 256, allowing a quick shift operation
  // on the result rather than a costly divide, while the type upgrade to int
  // avoids repeated type conversions in both directions.
  s1 = sat + 1;
  r  = 255 - (((255 - r) * s1) >> 8);
  g  = 255 - (((255 - g) * s1) >> 8);
  b  = 255 - (((255 - b) * s1) >> 8);

  // Value (brightness) & 16-bit color reduction: similar to above, add 1
  // to allow shifts, and upgrade to int makes other conversions implicit.
  v1 = val + 1;
  if(gflag) { // Gamma-corrected color?
    r = pgm_read_byte(&gamma_table[(r * v1) >> 8]); // Gamma correction table maps
    g = pgm_read_byte(&gamma_table[(g * v1) >> 8]); // 8-bit input to 4-bit output
    b = pgm_read_byte(&gamma_table[(b * v1) >> 8]);
  } else { // linear (uncorrected) color
//#if (nPlanes == 4)
    r = (r * v1) >> 12; // 4-bit results
    g = (g * v1) >> 12;
    b = (b * v1) >> 12;
/* #elif (nPlanes == 5)
    r = (r * v1) >> 11; // 5-bit results
    g = (g * v1) >> 11;
    b = (b * v1) >> 11;
#else
#error Unsupported number of planes
#endif
*/
  }
//#if (nPlanes == 4)
  return (r << 12) | ((r & 0x8) << 8) | // 4/4/4 -> 5/6/5
         (g <<  7) | ((g & 0xC) << 3) |
         (b <<  1) | ( b        >> 3);
/* #elif (nPlanes == 5)

#warning Requires 5 bit gamma

  return (r << 11) |                    // 5/5/5 -> 5/6/5
         (g <<  6) | ((g & 0x10) << 1) |
         (b) ;
#else
#error Unsupported number of planes
#endif 
*/
}


// -------------------- Screen read/write  --------------------

void RGBmatrixPanel::drawPixel(int16_t x, int16_t y, uint16_t c) {
  uint8_t r, g, b, bit, limit, *ptr;

/*
#if defined(DEBUG)
  Serial.print("DrawPixel(");
  Serial.print(x);
  Serial.print(", ");
  Serial.print(y);
  Serial.print(", ");
  Serial.print(c);
  Serial.println(")");
#endif
*/
  if((x < 0) || (x >= _width) || (y < 0) || (y >= _height)) return;

  switch(rotation) {
   case 1:
    swap(x, y);
    x = WIDTH  - 1 - x;
    break;
   case 2:
    x = WIDTH  - 1 - x;
    y = HEIGHT - 1 - y;
    break;
   case 3:
    swap(x, y);
    y = HEIGHT - 1 - y;
    break;
  }

  // Adafruit_GFX uses 16-bit color in 5/6/5 format, while matrix needs
  // 4/4/4.  Pluck out relevant bits while separating into R,G,B:
  r =  c >> 12;        // RRRRrggggggbbbbb
  g = (c >>  7) & 0xF; // rrrrrGGGGggbbbbb
  b = (c >>  1) & 0xF; // rrrrrggggggBBBBb

  // Loop counter stuff
  bit   = 2;
  limit = 1 << nPlanes;

  if(y < nRows) {
    // Data for the upper half of the display is stored in the lower
    // bits of each byte.
    ptr = &matrixbuff[backindex][y * WIDTH * (nPlanes - 1) + x]; // Base addr
// FIXME: Adapt for nBuf > 2
    // Plane 0 is a tricky case -- its data is spread about,
    // stored in least two bits not used by the other planes.
    ptr=ptr+BYTES_PER_ROW*2*nPanels;
    *ptr &= ~B00000011;            // Plane 0 R,G mask out in one op
    if(r & 1) *ptr |=  B00000001;  // Plane 0 R: 64 bytes ahead, bit 0
    if(g & 1) *ptr |=  B00000010;  // Plane 0 G: 64 bytes ahead, bit 1
    ptr=ptr-BYTES_PER_ROW*nPanels;
    if(b & 1) *ptr |=  B00000001;  // Plane 0 B: 32 bytes ahead, bit 0
    else      *ptr &= ~B00000001;  // Plane 0 B unset; mask out
    // The remaining three image planes are more normal-ish.
    // Data is stored in the high 6 bits so it can be quickly
    // copied to the DATAPORT register w/6 output lines.
    ptr=ptr-BYTES_PER_ROW*nPanels;
    for(; bit < limit; bit <<= 1) {
      *ptr &= ~B00011100;             // Mask out R,G,B in one op
      if(r & bit) *ptr |= B00000100;  // Plane N R: bit 2
      if(g & bit) *ptr |= B00001000;  // Plane N G: bit 3
      if(b & bit) *ptr |= B00010000;  // Plane N B: bit 4
      ptr  += WIDTH;                  // Advance to next bit plane
    }
  } else {
    // Data for the lower half of the display is stored in the upper
    // bits, except for the plane 0 stuff, using 2 least bits.
    ptr = &matrixbuff[backindex][(y - nRows) * WIDTH * (nPlanes - 1) + x];
// FIXME: Adapt for nBuf > 2
    *ptr &= ~B00000011;               // Plane 0 G,B mask out in one op
    if(r & 1)  ptr[BYTES_PER_ROW*nPanels] |=  B00000010; // Plane 0 R: 32 bytes ahead, bit 1
    else       ptr[BYTES_PER_ROW*nPanels] &= ~B00000010; // Plane 0 R unset; mask out
    if(g & 1) *ptr     |=  B00000001; // Plane 0 G: bit 0
    if(b & 1) *ptr     |=  B00000010; // Plane 0 B: bit 1
    for(; bit < limit; bit <<= 1) {
      *ptr &= ~B11100000;             // Mask out R,G,B in one op
      if(r & bit) *ptr |= B00100000;  // Plane N R: bit 5
      if(g & bit) *ptr |= B01000000;  // Plane N G: bit 6
      if(b & bit) *ptr |= B10000000;  // Plane N B: bit 7
      ptr  += WIDTH;                  // Advance to next bit plane
    }
  }
}

uint16_t RGBmatrixPanel::getPixel(int16_t x, int16_t y) {
  uint8_t r, g, b, bit, limit, *ptr;

  if((x < 0) || (x >= _width) || (y < 0) || (y >= _height)) return 0;

  switch(rotation) {
   case 1:
    swap(x, y);
    x = WIDTH  - 1 - x;
    break;
   case 2:
    x = WIDTH  - 1 - x;
    y = HEIGHT - 1 - y;
    break;
   case 3:
    swap(x, y);
    y = HEIGHT - 1 - y;
    break;
  }

  // Loop counter stuff
  bit   = 2;
  limit = 1 << nPlanes;

  // This code was taken and reversed from drawPixel above.
  // A bit easier, since we can start with zeros for r, g, and b.

  r = g = b = 0;

  if(y < nRows) {
    // Data for the upper half of the display is stored in the lower
    // bits of each byte.
    ptr = &matrixbuff[1-backindex][y * WIDTH * (nPlanes - 1) + x]; // Base addr
// FIXME: Adapt for nBuf > 2
    // Plane 0 is a tricky case -- its data is spread about,
    // stored in least two bits not used by the other planes.
    if (ptr[BYTES_PER_ROW*2] & B00000001) r |= 1;   // Plane 0 R: 64 bytes ahead, bit 0
    if (ptr[BYTES_PER_ROW*2] & B00000010) g |= 1;   // Plane 0 G: 64 bytes ahead, bit 1
    if (ptr[BYTES_PER_ROW] & B00000001) b |= 1;     // Plane 0 B: 32 bytes ahead, bit 0
    // The remaining three image planes are more normal-ish.
    // Data is stored in the high 6 bits so it can be quickly
    // copied to the DATAPORT register w/6 output lines.
    for(; bit < limit; bit <<= 1) {
      if (*ptr & B00000100) r |= bit;  // Plane N R: bit 2
      if (*ptr & B00001000) g |= bit;  // Plane N G: bit 3
      if (*ptr & B00010000) b |= bit;  // Plane N B: bit 4
      ptr  += WIDTH;                   // Advance to next bit plane
    }
  } else {
    // Data for the lower half of the display is stored in the upper
    // bits, except for the plane 0 stuff, using 2 least bits.
    ptr = &matrixbuff[1-backindex][(y - nRows) * WIDTH * (nPlanes - 1) + x];
// FIXME: Adapt for nBuf > 2
    if (ptr[BYTES_PER_ROW] & B00000010) r |= 1;   // Plane 0 R: 32 bytes ahead, bit 1
    if (*ptr    & B00000001) g |= 1;   // Plane 0 G: bit 0
    if (*ptr    & B00000010) b |= 1;   // Plane 0 B: bit 1
    for(; bit < limit; bit <<= 1) {
      if (*ptr & B00100000) r |= bit;  // Plane N R: bit 5
      if (*ptr & B01000000) g |= bit;  // Plane N G: bit 6
      if (*ptr & B10000000) b |= bit;  // Plane N B: bit 7
      ptr  += WIDTH;                   // Advance to next bit plane
    }
  }

  return Color444(r, g, b);;
}


void RGBmatrixPanel::fillScreen(uint16_t c) {
  if((c == 0x0000) || (c == 0xffff)) {
    // For black or white, all bits in frame buffer will be identically
    // set or unset (regardless of weird bit packing), so it's OK to just
    // quickly memset the whole thing:
    memset(matrixbuff[backindex], c, BYTES_PER_ROW * nRows * nPackedPlanes * nPanels);
  } else {
    // Otherwise, need to handle it the long way:
    Adafruit_GFX::fillScreen(c);
  }
}


// -------------------- Buffers --------------------

// Return address of front buffer -- can then read display directly
uint8_t *RGBmatrixPanel::frontBuffer() {
  return matrixbuff[1-backindex];
}

// Return address of back buffer -- can then load/store data directly
uint8_t *RGBmatrixPanel::backBuffer() {
  return matrixbuff[backindex];
}


// For smooth animation -- drawing always takes place in the "back" buffer;
// this method pushes it to the "front" for display.  Passing "true", the
// updated display contents are then copied to the new back buffer and can
// be incrementally modified.  If "false", the back buffer then contains
// the old front buffer contents -- your code can either clear this or
// draw over every pixel.  (No effect if double-buffering is not enabled.)
void RGBmatrixPanel::swapBuffers(boolean copy) {
  if(matrixbuff[0] != matrixbuff[1]) {
#if defined(DEBUG)
  Serial.print("swapBuffers");
#endif
// FIXME: Adapt for nBuf > 2
    // To avoid 'tearing' display, actual swap takes place in the interrupt
    // handler, at the end of a complete screen refresh cycle.
    swapflag = true;                  // Set flag here, then...
// TODO: Try allow swap at end of line rather than waiting for end of frame
// What kind of tearing are they concerned with?  Suspect a holdover from old code?
// 
// Since screen redraw goes one line at a time, as long as you finish this line
// you should have a smooth transition from one screen to the next
// (sure, the transition starts in the middle of a frame, but is that a problem?
//  would one see that temporary top of screen is old display bottom new for that
//  vs whole display new repaint - given that lasts 1/200 second, and that each one
//  is a continuation of temporally adjacent patterns (just part is from the old, and part is from the new))

    while(swapflag == true) delay(1); // wait for interrupt to clear it
    if(copy == true)
      memcpy(matrixbuff[backindex], matrixbuff[1-backindex], BYTES_PER_ROW * nRows * nPackedPlanes * nPanels);
// TODO: Reduce busy wait - if copy is false, we could avoid this delay 
//  However other update functions would need to check to be sure the backbuffer was okay to use
//  (while swapflag is true, can not modify either buffer).
  }
}

#if defined(FADE)
// Fade between front and next buffers
// Take tfade refresh cycles for fade
// If copy is true, then at end copy next to front at end of fade
// Todo: change tfade unit from refresh cycles to time

// Fade is done by PWM between front and next buffers
uint8_t RGBmatrixPanel::swapFade(uint16_t tfade, boolean copy) {

#if defined(DEBUG)
  Serial.print("swapFade");
#endif

  if (0 != FadeLen) {
    if (0 == tfade){  // Setting tfade = 0 cancels fade in progress
// FIXME: Consider whether this may cause glitches (e.g. if need to wait until end of frame)
      FadeLen = 0;
      FadeNAccum = 0;
      FadeCnt = 0;
      return 0;
    }
      return 1; // Return error if fade already in progress
  }
  else if(matrixbuff[0] == matrixbuff[1]) 
// FIXME: Adapt for nBuf > 2
    return 2;
  else {
    FadeCnt = 0;        // How many cycles have occured in this fade
//    FadeNNext = 0;    // How many times has the next frame been shown
    FadeNAccum = 0;
    copyflag = copy;    // reminder to do copy at end
    // Start the fade

// TODO: change to use fade time, rather than number of refresh cycles
    FadeLen = tfade;    // FadeLen != 0 indicates fade in progress
// For fade time in milli-seconds, rather than in refresh cycles
//    FadeLen = (refreshFreq * (uint32_t)tfade)/1000; 
    return 0;
  }
}

// Or could make this return uint - fade time remaining
boolean RGBmatrixPanel::fading() {
  return (FadeLen != 0);
}

#endif


// Dump display contents to the Serial Monitor, adding some formatting to
// simplify copy-and-paste of data as a PROGMEM-embedded image for another
// sketch.  If using multiple dumps this way, you'll need to edit the
// output to change the 'img' name for each.  Data can then be loaded
// back into the display using a pgm_read_byte() loop.
void RGBmatrixPanel::dumpMatrix(void) {

  int i, buffsize = BYTES_PER_ROW * nRows * nPackedPlanes * nPanels;

#if defined(__AVR__)
  Serial.print("\n\n"
    "#include <avr/pgmspace.h>\n\n"
    "static const uint8_t PROGMEM img[] = {\n  ");
#else
  Serial.print("\n\n"
    "static const uint8_t img[] = {\n  ");
#endif

  for(i=0; i<buffsize; i++) {
    Serial.print("0x");
    if(matrixbuff[backindex][i] < 0x10) Serial.print('0');
    Serial.print(matrixbuff[backindex][i],HEX);
    if(i < (buffsize - 1)) {
      if((i & 7) == 7) Serial.print(",\n  ");
      else             Serial.print(',');
    }
  }
  Serial.println("\n};");
}


// -------------------- Interrupt handler stuff --------------------


#if defined(__TIVA__)

#if defined( BENCHMARK )

uint8_t nprint = 20;

#endif

void TmrHandler()
{
#if defined( BENCHMARK )
  c_start = HWREG(DWT_BASE + DWT_O_CYCCNT); // at the beginning of the tested code
#endif

  activePanel->updateDisplay();

  MAP_TimerIntClear( TIMER_BASE, TIMER_TIMA_TIMEOUT );

#if defined( BENCHMARK )
  c_stop = HWREG(DWT_BASE + DWT_O_CYCCNT);  // at the end of the tested code  

  if (nprint > 0){
    --nprint;
    Serial.print("Tmh: ");
    Serial.println(c_stop - c_start);
    };
#endif
}

// Timing for Stellaris launchpad 1x 16x32 panel: 1348, 1316, 1316, 1706
//   2x 32x32 panel: 3274, 2514, 2472, 2472, 3246, ...
//   3x 32x32 panel: 4804, 3662, 3620, 3620, 4778, 3662, ...


/*
2*m + n = 2472
3*m + n = 3620
n = 2472 - 2 * m
3*m + 2472 - 2 * m = 3620

m* (3 - 2) = 3620 - 2472
m = 1148
n = 176

*/

//   Does not include interrupt overhead to call timer handler
//   Includes a couple of assignments, to record start/stop times

// TODO: See how minRowTime scales with number of panels

#if defined(__TM4C1294NCPDT__)
// Connected Launchpad (120 MHz clock)

const uint16_t minRowTimePerPanel = 1610;        // Ticks per panel for a row
const uint16_t minRowTimeConst = 270;            // Overhead ticks

#else
// For Stellaris Launchpad (80 MHz clock)

const uint16_t minRowTimePerPanel = 1150;        // Ticks per panel for a row
const uint16_t minRowTimeConst = 180;            // Overhead ticks

#endif

// minRowTime = 1148 * nPanels + 176 = 1324

// At 80 MHz, with 4 planes, 200 cycles/second gives about 3333 ticks/row
//   CPU takes about 1300 ticks to process a row on one panel, so plenty of time

// TODO: Reameasure and update minRowTime if make major changes to timer service routine


#else
//#elif defined(__AVR__)

ISR(TIMER1_OVF_vect, ISR_BLOCK) { // ISR_BLOCK important -- see notes later
  activePanel->updateDisplay();   // Call refresh func for active display
  TIFR1 |= TOV1;                  // Clear Timer1 interrupt flag
}

#endif


#if defined(__TIVA__)

// Caution: Blank the display before changing refresh
// May introduce a visual glitch if redefine refresh rate while displaying something
// FIXME: Revise to change refresh rate at end of a frame.

// Takes about 1300 ticks for minimum row time on Stellaris for 1 16row panel
//  So maximum refresh something in neighborhood of 500 cycles/second (maybe a bit less)

// With 2 32 row panels
//  Maximum refresh something in neighborhood of 128 cycles/second 
//  Might get to neighborhood of 200 cycles/second with 120MHz clock (TM4C1294)

// Optimization can probably improve those figures


// Returns refresh rate
uint16_t RGBmatrixPanel::setRefresh(uint8_t freq){
  uint32_t rowtimetemp;
  
//  uint16_t refreshTime = 1 * ticksPerSecond / refreshFreq;   // Time for 1 display refresh
//  rowtime = refreshTime / (nRows * ((1<<nPlanes) - 1));  // Time to display LSB of one row
//  rowtime = ticksPerSecond / (refreshFreq * nRows * ((1<<nPlanes) - 1));  // Time to display LSB of one row
  rowtimetemp = (uint32_t) TIMER_CLK / ((uint32_t) freq * nRows * ((1<<nPlanes) - 1));  // Time to display LSB of one row

#if defined(DEBUG)
  Serial.print("Rowtime raw: ");
  Serial.print(rowtimetemp);
#endif

  if (rowtimetemp < minRowTimePerPanel * nPanels + minRowTimeConst){  // Approximate sanity check
    rowtime = minRowTimePerPanel * nPanels + minRowTimeConst;
#if defined(DEBUG)
  Serial.print(", Rowtime: ");
  Serial.print(rowtime);
#endif
    }
  else
    rowtime = rowtimetemp;

  refreshFreq = ((uint32_t) TIMER_CLK / (rowtime * nRows * ((1<<nPlanes) - 1)));
#if defined(DEBUG)
  Serial.print("Freq:");
  Serial.print(freq);
  Serial.print(" refresh:");
  Serial.println(refreshFreq);
#endif
  return refreshFreq;
}

#endif


// Two constants are used in timing each successive BCM interval.
// These were found empirically, by checking the value of TCNT1 at
// certain positions in the interrupt code.
// CALLOVERHEAD is the number of CPU 'ticks' from the timer overflow
// condition (triggering the interrupt) to the first line in the
// updateDisplay() method.  It's then assumed (maybe not entirely 100%
// accurately, but close enough) that a similar amount of time will be
// needed at the opposite end, restoring regular program flow.
// LOOPTIME is the number of 'ticks' spent inside the shortest data-
// issuing loop (not actually a 'loop' because it's unrolled, but eh).
// Both numbers are rounded up slightly to allow a little wiggle room
// should different compilers produce slightly different results.
#define CALLOVERHEAD 60   // Actual value measured = 56
#define LOOPTIME     350  // Actual value measured = 188
// measured value is probably for single panel
// FIXME: should adjust for the number of panels
// LOOPTIME was 200 for single panel,
// was increased to 350 for multi-panel (2x measured value would be 376)

// The "on" time for bitplane 0 (with the shortest BCM interval) can
// then be estimated as LOOPTIME + CALLOVERHEAD * 2.  Each successive
// bitplane then doubles the prior amount of time.  We can then
// estimate refresh rates from this:
// 4 bitplanes = 320 + 640 + 1280 + 2560 = 4800 ticks per row.
// 4800 ticks * 16 rows (for 32x32 matrix) = 76800 ticks/frame.
// 16M CPU ticks/sec / 76800 ticks/frame = 208.33 Hz.
// Actual frame rate will be slightly less due to work being done
// during the brief "LEDs off" interval...it's reasonable to say
// "about 200 Hz."  The 16x32 matrix only has to scan half as many
// rows...so we could either double the refresh rate (keeping the CPU
// load the same), or keep the same refresh rate but halve the CPU
// load.  We opted for the latter.
// Can also estimate CPU use: bitplanes 1-3 all use 320 ticks to
// issue data (the increasing gaps in the timing invervals are then
// available to other code), and bitplane 0 takes 920 ticks out of
// the 2560 tick interval.
// 320 * 3 + 920 = 1880 ticks spent in interrupt code, per row.
// From prior calculations, about 4800 ticks happen per row.
// CPU use = 1880 / 4800 = ~39% (actual use will be very slightly
// higher, again due to code used in the LEDs off interval).
// 16x32 matrix uses about half that CPU load.  CPU time could be
// further adjusted by padding the LOOPTIME value, but refresh rates
// will decrease proportionally, and 200 Hz is a decent target.

// The flow of the interrupt can be awkward to grasp, because data is
// being issued to the LED matrix for the *next* bitplane and/or row
// while the *current* plane/row is being shown.  As a result, the
// counter variables change between past/present/future tense in mid-
// function...hopefully tenses are sufficiently commented.


void RGBmatrixPanel::updateDisplay(void) {
  uint8_t  i, *ptr;
#if defined(__TIVA__)
  uint32_t duration;
#else
  uint16_t t;
  uint16_t duration;
#endif
  uint8_t panelcount;


#if defined(__TIVA__)
  *oeport  = oepin;  // Disable LED output during row/plane switchover
  *latport = latpin; // Latch data loaded during *prior* interrupt
#else
  *oeport  |= oepin;  // Disable LED output during row/plane switchover
  *latport |= latpin; // Latch data loaded during *prior* interrupt
#endif

/*
#if defined(DEBUG)
  Serial.print(plane);
#endif
*/

#if defined(__TIVA__)
  duration = rowtime << plane;
  // FIXME: Check counter calculation - can probably simplify duration vs this
//  uint32_t final = ( ( uint64_t )duration * TIMER_CLK ) / 1000000;  // duration in usec
#else
  // Calculate time to next interrupt BEFORE incrementing plane #.
  // This is because duration is the display time for the data loaded
  // on the PRIOR interrupt.  CALLOVERHEAD is subtracted from the
  // result because that time is implicit between the timer overflow
  // (interrupt triggered) and the initial LEDs-off line at the start
  // of this method.
  t = (nRows > 8) ? LOOPTIME : (LOOPTIME * 2);
  // FIXME: adjust for number of panels (maybe * nPanels )
  // TODO: Why is t longer for 32x32 than for 32x16?
  duration = ((t + CALLOVERHEAD * 2) << plane) - CALLOVERHEAD;
#endif

  // Borrowing a technique here from Ray's Logic:
  // www.rayslogic.com/propeller/Programming/AdafruitRGB/AdafruitRGB.htm
  // This code cycles through all four planes for each scanline before
  // advancing to the next line.  While it might seem beneficial to
  // advance lines every time and interleave the planes to reduce
  // vertical scanning artifacts, in practice with this panel it causes
  // a green 'ghosting' effect on black pixels, a much worse artifact.

  if(++plane >= nPlanes) {      // Advance plane counter.  Maxed out?
    plane = 0;                  // Yes, reset to plane 0, and
    if(++row >= nRows) {        // advance row counter.  Maxed out?
      row     = 0;              // Yes, reset row counter, then...
      if(swapflag == true) {    // Swap front/back buffers if requested
        backindex = 1 - backindex;
        swapflag  = false;
        buffptr = matrixbuff[1-backindex]; // Reset into front buffer
      }
#if defined(FADE)
      else if(FadeLen) {
        if (++FadeCnt == FadeLen){  // Fade done, swap the buffers
          backindex = 1 - backindex;
          FadeLen = 0;
          FadeCnt = 0;
//          FadeNNext = 0;
          FadeNAccum = 0;
// FIXME: Need to get the copy done somehow (or just remove that option)
          buffptr = matrixbuff[1-backindex]; // Reset into front buffer
        }
        else {  // Calculate which buffer to show this time

// Need to test following expression 
//  (derived from calculating error differences and a bunch of algebra, checked in a spreadsheet so think okay)

//
// if (abs(FadeCnt ^ 2 - 2 * FadeLen * FadeNNext) > abs(FadeCnt ^ 2 - 2 * FadeLen * (FadeNNext + 1)))
//
// If it works, see how to optimize calculation - 
//   2*FadeLen*FadeNNext can be calculated cumulatively 
//     (Keep running total, when increment FadeNNext, then FadeNAccum += 2 * FadeLen)
//   FadeCnt^2 used on both sides (calculate once)
//   Could cache 2*FadeLen - but just a bit shift, so may not be worth it

          int16_t FadeCntSq = FadeCnt * FadeCnt;

          if (abs(FadeCntSq - FadeNAccum ) > abs(FadeCntSq - FadeNAccum - (2 * FadeLen) )){
//          if (abs(FadeCntSq - FadeNAccum ) > FadeLen )){    // TODO: Something like this might also work
//    Show NextBuffer;
            buffptr = matrixbuff[backindex]; // Reset into back buffer
            FadeNAccum += 2 * FadeLen; // Update accumulated showing next // FadeNNext++
          }
          else {
//    Show FrontBuffer;
            buffptr = matrixbuff[1-backindex]; // Reset into front buffer
          }
        }
      }
#endif // FADE
      else
        buffptr = matrixbuff[1-backindex]; // Reset into front buffer
    }
  } else if(plane == 1) {
/*
#if defined(DEBUG)
    Serial.print("P1R");
    Serial.print(row);
#endif
*/
    // Plane 0 was loaded on prior interrupt invocation and is about to
    // latch now, so update the row address lines before we do that:
#if defined(__TIVA__)
// TODO: See if code generated is smaller if use 0xFF in place of addrxpin in the conditional expression
//  Since using pin masking, they should have same effect
//  (Or if using code as above, but simple = addrpin or = ~addrpin, or = 0, or = 0xFF)
//  could also compare to using bitbanding (then just write a 1 or a 0)
/*
    *addraport = (row & 0x1) ? addrapin : 0;
    *addrbport = (row & 0x2) ? addrbpin : 0;
    *addrcport = (row & 0x4) ? addrcpin : 0;
    if(nRows > 8)
      *addrdport = (row & 0x8) ? addrdpin : 0; */

    *addraport = ((row & 0x1) ? 0xFF : 0);
    *addrbport = ((row & 0x2) ? 0xFF : 0);
    *addrcport = ((row & 0x4) ? 0xFF : 0);
    if(nRows > 8)
      *addrdport = ((row & 0x8) ? 0xFF : 0);
#else
//#elif defined(__AVR__)
    if(row & 0x1)   *addraport |=  addrapin;
    else            *addraport &= ~addrapin;
    if(row & 0x2)   *addrbport |=  addrbpin;
    else            *addrbport &= ~addrbpin;
    if(row & 0x4)   *addrcport |=  addrcpin;
    else            *addrcport &= ~addrcpin;
    if(nRows > 8) {
      if(row & 0x8) *addrdport |=  addrdpin;
      else          *addrdport &= ~addrdpin;
    }
#endif
  }

  // buffptr, being 'volatile' type, doesn't take well to optimization.
  // A local register copy can speed some things up:
  ptr = (uint8_t *)buffptr;

#if defined(__TIVA__)

/*
#if defined(DEBUG)
    Serial.print(" du");
    Serial.println(duration);
#endif
*/
//  MAP_TimerDisable( timerBase, timerAB );
  MAP_TimerLoadSet( TIMER_BASE, TIMER_A, duration );
  MAP_TimerEnable( TIMER_BASE, TIMER_A );

#else
  ICR1      = duration; // Set interval for next interrupt
  TCNT1     = 0;        // Restart interrupt timer
#endif

#if defined(__TIVA__)
  *oeport  = 0;  // Re-enable output
  *latport = 0;  // Latch down
#else
  *oeport  &= ~oepin;   // Re-enable output
  *latport &= ~latpin;  // Latch down
#endif 

#if !defined(__TIVA__)
  // Record current state of SCLKPORT register, as well as a second
  // copy with the clock bit set.  This makes the innnermost data-
  // pushing loops faster, as they can just set the PORT state and
  // not have to load/modify/store bits every single time.  It's a
  // somewhat rude trick that ONLY works because the interrupt
  // handler is set ISR_BLOCK, halting any other interrupts that
  // might otherwise also be twiddling the port at the same time
  // (else this would clobber them).
  uint8_t tick, tock;
  tock = SCLKPORT;
  tick = tock | sclkpin;
#else
  static const uint8_t tick = 0xFF;
  static const uint8_t tock = 0;
#endif

  if(plane > 0) { // 188 ticks from TCNT1=0 (above) to end of function

    // Planes 1-3 copy bytes directly from RAM to PORT without unpacking.
    // The least 2 bits (used for plane 0 data) are presumed masked out
    // by the port direction bits.

#if defined(__AVR__)
    // A tiny bit of inline assembly is used; compiler doesn't pick
    // up on opportunity for post-increment addressing mode.
    // 5 instruction ticks per 'pew' = 160 ticks total
    #define pew asm volatile(                 \
      "ld  __tmp_reg__, %a[ptr]+"    "\n\t"   \
      "out %[data]    , __tmp_reg__" "\n\t"   \
      "out %[clk]     , %[tick]"     "\n\t"   \
      "out %[clk]     , %[tock]"     "\n"     \
      :: [ptr]  "e" (ptr),                    \
         [data] "I" (_SFR_IO_ADDR(DATAPORT)), \
         [clk]  "I" (_SFR_IO_ADDR(SCLKPORT)), \
         [tick] "r" (tick),                   \
         [tock] "r" (tock));

#define UNROLL_LOOP

#else				// Code for non AVR (i.e. Due and ARM based systems)

#define UNROLL_LOOP

// In this case local variables easier for the compiler to access/optimize
// (Might be able to do it with the dataport variable in "this" instead?)
// Makes the inner "loop" 4 instructions, a load and 3 stores

volatile uint8_t * dataport = &DATAPORT;
volatile uint8_t * sclkp = &SCLKPORT;

#define pew *dataport = *ptr++; * sclkp = tick; * sclkp = tock;

//#define pew DATAPORT = *ptr++; SCLKPORT = tick; SCLKPORT = tock;

#endif

#if defined( UNROLL_LOOP )
    for (panelcount = 0; panelcount < nPanels; panelcount++)
    {
    	// Loop is unrolled for speed:
      pew pew pew pew pew pew pew pew
      pew pew pew pew pew pew pew pew
      pew pew pew pew pew pew pew pew
      pew pew pew pew pew pew pew pew
    } 

#else     // Loopy version

    // Calculating final value ahead saves 2 instructions per loop
    uint8_t iFinal = (BYTES_PER_ROW*nPanels); 

    for(i=0; i<iFinal; i++)
    {
      DATAPORT = ptr[i];
// TODO: Or could try bitbanding
// I think the comments here were wrong, said tick was lo, tock hi
      SCLKPORT = tick;
      SCLKPORT = tock;
    }
/*
// Pointers rather than indexing, saves 2 more instructions per loop
    uint8_t *pFinal = ptr + (BYTES_PER_ROW*nPanels);
    for(; ptr<pFinal; ptr++)
    {
      DATAPORT = *ptr;
      SCLKPORT = tick;
      SCLKPORT = tock;
    }
*/
#endif
    buffptr += (BYTES_PER_ROW*nPanels);


  } else { // 920 ticks from TCNT1=0 (above) to end of function

    // Planes 1-3 (handled above) formatted their data "in place,"
    // their layout matching that out the output PORT register (where
    // 6 bits correspond to output data lines), maximizing throughput
    // as no conversion or unpacking is needed.  Plane 0 then takes up
    // the slack, with all its data packed into the 2 least bits not
    // used by the other planes.  This works because the unpacking and
    // output for plane 0 is handled while plane 3 is being displayed...
    // because binary coded modulation is used (not PWM), that plane
    // has the longest display interval, so the extra work fits.
    for(i=0; i<(BYTES_PER_ROW*nPanels); i++) {
      DATAPORT =
        ( ptr[i]    << 6)                   |
        ((ptr[i+(BYTES_PER_ROW*nPanels)] << 4) & 0x30) |
        ((ptr[i+(BYTES_PER_ROW*2*nPanels)] << 2) & 0x0C);
      SCLKPORT = tick;
      SCLKPORT = tock;
    } 
  }
}
