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

Version v1.x.x-x.x.x, 17 Aug 2014

Revisions:
    getPixel, by RobF42 - Rob Fugina
    daisychain displays, by protonmaster - Phillip Burgess
    Energia/TI Stellaris/Tiva Launchpad support, by ecdr - Michael Hanson

- Tiva version:
  Works with Energia.
  More port selection from user code (faster to use port variables rather than constants). 
  Does not need inline assembly language on ARM (carefully selected c++ code sufficient).

  User selectable refresh rate.
  Add fade between immages.

*/

#include "RGBmatrixPanel.h"
#include "RGBmatrixPanelConfig.h"

#include "gamma.h"



#if !defined(__AVR__)

#if !defined(pgm_read_byte)
#define pgm_read_byte( a ) (*(a))
#endif

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


#if defined( BENCHMARK )
#include "benchmark/cyclecount.h"
#endif

#endif

static const uint8_t nPlanes = 4;
// Will not work for nPlanes < 4 (would need fixing to not do packing)
static const uint8_t nPackedPlanes = (nPlanes - 1);  // 3 bytes holds 4 planes "packed"
static const uint8_t BYTES_PER_ROW = 32;



// -------------------- Utility macros --------------------

#if defined(__TIVA__)


// TODO: See how much extra time needed, 
//   compare loop unrolled with extra operations to looping version without SLOW_CLOCK
// TODO: Clean up (or remove) code for UNROLL_LOOP not defined


// Energia does not define portOutputRegister(port) for Tiva, so make up replacement. 
// portMaskedOutputRegister(port, mask)
// include port mask so do not need read, modify, write
// can just write to the address, and it will not change other pins

// For reference:
//  void digitalWrite(uint8_t pin, uint8_t val) =
//  HWREG(portBASERegister(digitalPinToPort(pin)) + (GPIO_O_DATA + (digitalPinToBitMask(pin) << 2))) = val ? 0xFF : 0;
//  HWREGB should work instead (since all ports are 8 bits max)
//                                     0  
//#define HWREG(x)   (*((volatile uint32_t *)(x)))
//#define HWREGB(x)  (*((volatile uint8_t  *)(x)))

// TODO: Consider bitband version, for 1 bit control lines
// BITBAND 
/*
//#define HWREGBITB(x, b)                                                       \
//        HWREGB(((uint32_t)(x) & 0xF0000000) | 0x02000000 |                    \
//               (((uint32_t)(x) & 0x000FFFFF) << 5) | ((b) << 2))
*/

// Caution - be careful of adding masks to pointer types.
// Cast the portBASERegister back to uint32_t before add offset
// Otherwise it will do a clandestine left shift 2 on the mask
// Alternative would be to make use of that left shift, 
// but then need a big note explaining the occult behavior

#define portMaskedOutputRegister(port, mask) \
  ((volatile uint8_t *) (((uint32_t)portBASERegister(port)) + (GPIO_O_DATA + (((uint32_t)mask) << 2))))

// True if a number indicates a valid pin
/*
// FIXME: Add range checks.  
#define PIN_OK(pin) (( pin < ((sizeof (digital_pin_to_port))/(sizeof (digital_pin_to_port[0])))) && \
  (NOT_A_PIN != digital_pin_to_port[pin]))
*/
/*
FIXME: The compiler complains that

libraries\RGBmatrixpanel\RGBmatrixPanel.cpp:395:3: error: invalid application of 'sizeof' to incomplete type 'const uint8_t [] {aka const unsigned char []}'

sizeof an array doesn't work in C++ (unlike C)
Tried various incantations which are supposed to do this in C++, 
but none of them work in Energia
*/
#define PIN_OK(pin) ( (NOT_A_PIN != digital_pin_to_port[pin]))


// Use variable for sclkport
#if !defined( SCLKPORT )
#define SCLKPORT        (*sclkport)
#else
#warning Defining SCLKPORT as a constant slows down refresh on Stellaris/Tiva
#endif


#ifdef __TM4C1294NCPDT__
// TM4C1294 - can only get clock speed from clock set call
#define TIMER_CLK F_CPU
#else

// SysCtlClockGet only works on TM4C123x
#define TIMER_CLK SysCtlClockGet()
// However there is a bug in SysCtlClockGet() - in Tivaware 2.1...., 
// SysCtlClockGet incorrect if request 80MHz clock
// But Energia does not use Tivaware 2.1 at this point

//#define TIMER_CLK F_CPU

#endif


/*
Given name of a timer, assemble names of the various associated constants.
  BASE(TIMER0) =>  TIMER0_BASE
  SYSCTL_PERIPH_TIMER0
  INT_TIMER0A
  TIMER_A
  TIMER_TIMA_TIMEOUT
*/

#if (TIMER_CHANEL == A)
#define TIMER_AB  TIMER_A
#define TIMER_TIMEOUT TIMER_TIMA_TIMEOUT

#define INT1(t)   (INT_##t##A)

#elif (TIMER_CHANEL == B)

#define TIMER_AB  TIMER_B
#define TIMER_TIMEOUT TIMER_TIMB_TIMEOUT

#define INT1(t)   (INT_##t##B)

#else

#error Tiva: Unrecognized timer channel (TIMER_CHANEL should be A or B)

#endif

#define INTAB(t)    INT1(t)

#define TIMER_INT   INTAB(TIMER)


// TODO: Finish filling in using these macros for timer control (to use chanel A or B)


// Auxiliary macros (to get substitution to happen correctly)
#define BASE1(t)   (t##_BASE)
#define SYSCTL1(t) (SYSCTL_PERIPH_##t)
//#define INTA1(t)   (INT_##t##A)

#define BASE(t)    BASE1(t)
#define SYSCTL(t)  SYSCTL1(t)
//#define INTA(t)    INTA1(t)


// Actually assemble the timer macro names

#define TIMER_BASE   BASE(TIMER)
#define TIMER_SYSCTL SYSCTL(TIMER)
//#define TIMER_INT    INTA(TIMER)


// TODO: If miss an interrupt, the display driver dies
//   Try make more robust.
//   Maybe using repeat timer, rather than ONE_SHOT might help?

// For wide timer, use half timer
// This assumes that WTIMER bases all come after regular timer bases
//  (which is true in TM4C123x, but need not be true in general)

#if ( TIMER_BASE >= WTIMER0_BASE )

#if ( TIMER_CHANEL == A)
#define TIMER_CFG   (TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_ONE_SHOT)
#else
#define TIMER_CFG   (TIMER_CFG_SPLIT_PAIR | TIMER_CFG_B_ONE_SHOT)
#endif

#else
#define TIMER_CFG   TIMER_CFG_ONE_SHOT
#endif





void TmrHandler(void);

#endif


#if !defined( DATAPORTSHIFT )
#define DATAPORTSHIFT 0
#endif

// Shift value left by shift bits  If shift is negative then uses a right shift.
// (The compiler put out some more complicated code when given a constant negative shift.)
#define LEFT_SHIFT(value, shift) ((shift < 0) ? (value) >> - (shift) : ((value) << (shift)))


// FIXME: Need to import real assert
#if defined(DEBUG)
#define ASSERT(expr) if (!(expr)) {Serial.print("Error: assertion failure in "); \
    Serial.print(__FILE__); Serial.print(", line"); Serial.println(__LINE__);}
#else
#define ASSERT(expr)
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

// -------------------- Benchmark  --------------------

#if defined( BENCHMARK )

// Number of times to print time taken by updateDisplay
//uint16_t nprint = 20;

// Timing points
volatile uint32_t c_tmr_handler_start = 0;
volatile uint32_t c_tmr_handler_tset = 0;  // Where set delay to next ISR
volatile uint32_t c_tmr_handler_loop = 0;  // Benchmark time for preamble (just before display loop)
volatile uint32_t c_tmr_handler_end = 0;

volatile uint32_t c_tmr_handler_start_old = 0;

#if defined(BENCHMARK_OE)
// Timing for OE - oeon / (oeoff + oeonn) gives duty cycle

volatile uint32_t oeon_time = 0;
volatile uint32_t oeoff_time = 0;

volatile boolean oeflag = false;
volatile uint64_t oeon = 0;
volatile uint64_t oeoff = 0;
volatile uint32_t c_tmr_oeoff = 0, c_tmr_oeon = 0;
#endif

#endif


// -------------------- Constructors  --------------------

// Code common to both the 16x32 and 32x32 constructors:
void RGBmatrixPanel::init(uint8_t rows, uint8_t a, uint8_t b, uint8_t c,
  uint8_t sclk, uint8_t latch, uint8_t oe, boolean dbuf, uint8_t pwidth) {

// FIXME: Error to not have any panels.  Should throw exception or something.
// (Actually the compiler should catch this, but it doesn't seem to care if omit pwidth)
  if (pwidth == 0){
    pwidth = 1;
    // FIXME: Signal error somehow
  }

  // TODO: Make nRows, nPanels constants (set in constructor) - would be an error if changed
  nRows = rows; // Number of multiplexed rows; actual height is 2X this
  nPanels = pwidth;

  // Allocate and initialize matrix buffer:
  unsigned int buffsize  = bufferSize();
  unsigned int allocsize = (dbuf == true) ? (buffsize * nBuf) : buffsize;
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
  newrowtime = 0;

  dimtime = 0;
  dimwait = false;

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
  ASSERT(PIN_OK(d));

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
  addrdpin  = digitalPinToBitMask(d);
  
#if defined(__TIVA__)
  addrdport = portMaskedOutputRegister(digitalPinToPort(d), addrdpin);
#else
  addrdport = portOutputRegister(digitalPinToPort(d));
#endif  
}
*/


void RGBmatrixPanel::begin(void) {

#if defined(DEBUG) || defined( BENCHMARK )
// FIXME: Should check if Serial already begun, and only begin if not
//   Don't see a call in the reference to test this, have to check the code
#if defined(DBUG_CON_SPEED)
  Serial.begin(DBUG_CON_SPEED);
#endif  
#endif

// Didn't get any output from ASSERTs when put in init
  ASSERT(WIDTH == BYTES_PER_ROW * nPanels);
// FIXME: This assertion fails (1x32 row panel) - ?????
//F:\Programming\energia-0101E0012-windows\energia-0101E0012\hardware\lm4f\libraries\RGBmatrixpanel\RGBmatrixPanel.cpp, line547
// If leave the number of panels implicit, then this assertion fails
// Should default to 1, but see what really get
  ASSERT(PIN_OK(_a));
  ASSERT(PIN_OK(_b));
  ASSERT(PIN_OK(_c));
  ASSERT(PIN_OK(_sclk));
  ASSERT(PIN_OK(_latch));
  ASSERT(PIN_OK(_oe));

/*
#if defined(DEBUG)

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
*/

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
    ASSERT(PIN_OK(_d));
    pinMode(_d  , OUTPUT); *addrdport &= ~addrdpin; // Low
  }

  // Configure dataport bits as output, and initialize to 0

#if defined(__TIVA__)
  MAP_GPIOPinTypeGPIOOutput( DATAPORTBASE, DATAPORTMASK );
  MAP_GPIOPinWrite(DATAPORTBASE, DATAPORTMASK, 0);
#else
  DATADIR  = DATAPORTMASK;
  DATAPORT = 0;
#endif  


#if defined(BENCHMARK)
  EnableTiming();
#endif


// Timer setup


#if defined(__TIVA__)

  setRefresh(defaultRefreshFreq);
  if (newrowtime)
    rowtime = newrowtime;         // Be sure rowtime is valid

#if defined(DEBUG)
  
  Serial.print("Rowtime ");
  Serial.println(rowtime);

#endif

  MAP_SysCtlPeripheralEnable( TIMER_SYSCTL );

//  MAP_TimerDisable( TIMER_BASE, TIMER_A );

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


// Destructor - free allocated memory
RGBmatrixPanel::~RGBmatrixPanel(void) {
  free(matrixbuff[0]);
}


void RGBmatrixPanel::stop(void) {

#if defined(__TIVA__)
// Stop timer
  
  MAP_TimerIntDisable(TIMER_BASE, TIMER_TIMA_TIMEOUT);
  MAP_TimerIntClear(TIMER_BASE, TIMER_TIMA_TIMEOUT);
  MAP_TimerDisable(TIMER_BASE, TIMER_A);
  IntUnregister(TIMER_INT);
#else

// AVR
	TCCR1B = _BV(WGM13);    // From TimerOne, stop clock(?) TODO: Test
	TIMSK1 = 0;             // From TimerOne, detachInterrupt TODO: test

#endif  // __TIVA__

// TODO: Should probably send all 0's to the panel

  *oeport    |= oepin;     // High (disable output)
#if defined(BENCHMARK_OE)
  c_tmr_oeoff = HWREG(DWT_BASE + DWT_O_CYCCNT);
  if (oeflag){
    oeflag = false;
    oeon += c_tmr_oeoff - c_tmr_oeon;
    }
#endif

  activePanel = NULL;
}


// -------------------- Color  --------------------

// Note that some colors may use more bits than others (e.g. Green - 5/6/5)
uint8_t RGBmatrixPanel::bitsPerColor(void) const {
  return nPlanes;
}


// color = bgr4 bgr3 b|gr2 bgr1 bgr0 g-1

// Convert b3b2b1b0 -> 000 00b30 0b200 b100b0

// Space out bits every SPACING bits
#define SPACING 3
// So far only works for first 4 bits
#define SPACEOUT( n ) ((((n) & 8) << (3*SPACING - 3)) | (((n) & 4) << (2*SPACING - 2)) | (((n) & 2) << (SPACING-1)) | ((n) & 1))
// Note that the m'th bit is already in the m'th position, so bit position is subtracted from ammount to shift

// This handles one nibble
// Could extend space_bits to handle slightly larger values (e.g. 5 bits/color)

// For full 8 bits the table would have 256 entries
//   (768 bytes, possibly 1kb if can't get it to generate 3 byte entries)
//   Probably an acceptable ammount of flash,
//   would need some better way to generate (e.g. nested macros)
//   Could give a switch for user to control time vs. space tradeoff.
//
// Moving this array to RAM does not increase speed on 80MHz Tiva
static const uint16_t space_bit[] = {
  SPACEOUT( 0), SPACEOUT( 1), SPACEOUT( 2), SPACEOUT( 3),
  SPACEOUT( 4), SPACEOUT( 5), SPACEOUT( 6), SPACEOUT( 7),
  SPACEOUT( 8), SPACEOUT( 9), SPACEOUT(10), SPACEOUT(11),
  SPACEOUT(12), SPACEOUT(13), SPACEOUT(14), SPACEOUT(15) };

// Control interleaving order
// Currently bgr, with spare green bit at bit 0
#define RED_SHIFT   1
#define GREEN_SHIFT 2
#define BLUE_SHIFT  3

//5 planes  R4R3R2R1:R0G4G3G2|G1G0gxB4:B3B2B1B0
//4 planes  R3R2R1R0:rxG3G2G1|G0gxgxB3:B2B1B0bx
//bit#      15141312:11100908|07060504:03020100 
//IColor    b4g4r4b3:g3r3b2g2|r2b1g1r1:b0g0r0g-1


// Fastest - 40 clocks (including call, loop, and benchmark code), 11 instructions
crgb16i_t ColorIntern1(crgb16_t c) {
  crgb16i_t cnew = 0;

//#if (4 == nPlanes)
  // Adafruit_GFX uses 16-bit color in 5/6/5 format, 
  // while matrix uses low 4 planes.
  // Pluck out relevant bits while interleaving into b,g,r

  cnew  = space_bit[(c >>  1) & 0xF] << BLUE_SHIFT; // Blue bits
  cnew |= space_bit[(c >>  7) & 0xF] << GREEN_SHIFT; // Green bits
  cnew |= space_bit[(c >> 12) & 0xF] << RED_SHIFT; // Red bits

// Or handle nibble at a time, e.g.
// cnew1 = (space_bit[(r >> 4) & 0xF] << 12 | space_bit[r & 0xF]) << RED_SHIFT;

/*
  Other ways of doing conversion - can do all at compile time, but take longer

  But since don't use this that often, may not matter much (unless convert basic point plot to use this).

// Slightly slower - 44 clocks (including call, loop, and benchmark code) - 27 instructions
//   just a tad (4 clocks) slower than table lookup version on 80 MHz Tiva 
//  (Could compare on Connected LP - faster clock, more cache)
// 
//  If extend to more bits the table version would stay same speed (possibly take longer if go to 32 bit table?)
//  While this takes longer for more bits

// Set the tobit'th bit of tovar if the frombit'th bit of fromvar is set
// Caution: Using bitfield is not portable (order may vary, have not verified what order Energia uses)
  union {
    uint16_t word;
    struct {
        uint32_t b0:1, b1:1, b2:1, b3:1, b4:1, b5:1, b6:1, b7:1,
          b8:1, b9:1, b10:1, b11:1, b12:1, b13:1, b14:1, b15:1;
    };
  } cold, cnew1;
  cold.word = c;

#undef BIT_TST_SET
#define BIT_TST_SET(fromvar, frombit, tovar, tobit) tovar.b##tobit = fromvar.b##frombit

  BIT_TST_SET(cold, 12, cnew1, 1);  // R0
  BIT_TST_SET(cold, 13, cnew1, 4);  // R1
  BIT_TST_SET(cold, 14, cnew1, 7);  // R2
  BIT_TST_SET(cold, 15, cnew1, 10); // R3
  BIT_TST_SET(cold,  7, cnew1, 2);  // G0
  BIT_TST_SET(cold,  8, cnew1, 5);  // G1
  BIT_TST_SET(cold,  9, cnew1, 8);  // G2
  BIT_TST_SET(cold, 10, cnew1, 11); // G3
  BIT_TST_SET(cold,  1, cnew1, 3);  // B0
  BIT_TST_SET(cold,  2, cnew1, 6);  // B1
  BIT_TST_SET(cold,  3, cnew1, 9);  // B2
  BIT_TST_SET(cold,  4, cnew1, 12);  // B3

  return cnew1.word;


// Slowest - 80 clocks (including call, and benchmark overhead (loop))

// Set the tobit'th bit of tovar if the frombit'th bit of fromvar is set
#undef BIT_TST_SET

#define BIT_TST_SET(fromvar, frombit, tovar, tobit) if (fromvar & (1<<frombit)) tovar |= (1<<tobit)

  BIT_TST_SET(c, 12, cnew1, 1);  // R0
  BIT_TST_SET(c, 13, cnew1, 4);  // R1
  BIT_TST_SET(c, 14, cnew1, 7);  // R2
  BIT_TST_SET(c, 15, cnew1, 10); // R3
  BIT_TST_SET(c,  7, cnew1, 2);  // G0
  BIT_TST_SET(c,  8, cnew1, 5);  // G1
  BIT_TST_SET(c,  9, cnew1, 8);  // G2
  BIT_TST_SET(c, 10, cnew1, 11); // G3
  BIT_TST_SET(c,  1, cnew1, 3);  // B0
  BIT_TST_SET(c,  2, cnew1, 6);  // B1
  BIT_TST_SET(c,  3, cnew1, 9);  // B2
  BIT_TST_SET(c,  4, cnew1, 12);  // B3

  ASSERT(cnew1 == cnew);

  return cnew1;

*/

/*#elif (5 == nPlanes)
// TODO: test 5 planes version 

  cnew  = (space_bit[(c      ) & 0xF] | space_bit[(c >>   4) & 0xF] << 12) << BLUE_SHIFT; // Blue bits
  cnew |= (space_bit[(c >>  6) & 0xF] | space_bit[(c >>  10) & 0xF] << 12) << GREEN_SHIFT; // Green bits
  cnew |= (space_bit[(c >> 11) & 0xF] | space_bit[(c >>  15)      ] << 12) << RED_SHIFT; // Red bits

#else
#error drawPixel Unsupported number of planes
#endif
*/

  return cnew;
}

// FIXME: No clear reason for this to be a member function - maybe just get rid of this version
crgb16i_t RGBmatrixPanel::ColorIntern(crgb16_t c) const {
  return ColorIntern1(c);
}


// Original RGBmatrixPanel library used 3/3/3 color.  Later version used
// 4/4/4.  Then Adafruit_GFX (core library used across all Adafruit
// display devices now) standardized on 5/6/5.  The matrix still operates
// internally on 4/4/4 color, but all the graphics functions are written
// to expect 5/6/5...the matrix lib will truncate the color components as
// needed when drawing.  These next functions are mostly here for the
// benefit of older code using one of the original color formats.

// Promote 3/3/3 RGB to Adafruit_GFX 5/6/5
uint16_t RGBmatrixPanel::Color333(uint8_t r, uint8_t g, uint8_t b) const {
  // RRRrrGGGgggBBBbb
  return ((r & 0x7) << 13) | ((r & 0x6) << 10) |
         ((g & 0x7) <<  8) | ((g & 0x7) <<  5) |
         ((b & 0x7) <<  2) | ((b & 0x6) >>  1);
}

// Promote 4/4/4 RGB to Adafruit_GFX 5/6/5
uint16_t RGBmatrixPanel::Color444(uint8_t r, uint8_t g, uint8_t b) const {
  // RRRRrGGGGggBBBBb
  return ((r & 0xF) << 12) | ((r & 0x8) << 8) |
         ((g & 0xF) <<  7) | ((g & 0xC) << 3) |
         ((b & 0xF) <<  1) | ((b & 0x8) >> 3);
}

// Demote 8/8/8 to Adafruit_GFX 5/6/5
// If no gamma flag passed, assume linear color
uint16_t RGBmatrixPanel::Color888(uint8_t r, uint8_t g, uint8_t b) const {
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

// TODO: Seems it would be simpler to just use full 6-bit gamma output 
// (or possibly 8 bit gamma), and convert from 6/6/6 or 8/8/8 to 5/6/5
// (Fewer operations to handle made-up bits)
// Also means less adapting for greater bit depth display

// 8/8/8 -> gamma -> 5/6/5
uint16_t RGBmatrixPanel::Color888(
  uint8_t r, uint8_t g, uint8_t b, boolean gflag) const {
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
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

// TODO: Color mapping extends a color by repeating the upper few bits of the color in the low bits.  
// Why?
// 000R, 000G, 000B -> R000 | r[4]000:00 | 0:0ggg:g000:0000 | g[3:2]00:000 | b:bbb0 | b[3]
// Resulting bits are: R4R3R2R1R4 G4G3G2G1G4G3 B4B3B2B1B4  (R4 means bit 4 of R)


uint16_t RGBmatrixPanel::ColorHSV(
  long hue, uint8_t sat, uint8_t val, boolean gflag) const {

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

//#if (4 == nPlanes)
  // Adafruit_GFX uses 16-bit color in 5/6/5 format, while matrix needs
  // 4/4/4.  Pluck out relevant bits while separating into R,G,B:
  r =  c >> 12;        // RRRRrggggggbbbbb
  g = (c >>  7) & 0xF; // rrrrrGGGGggbbbbb
  b = (c >>  1) & 0xF; // rrrrrggggggBBBBb
/*
#elif (5 == nPlanes)
  r =  c >> 11;         // RRRRRggggggbbbbb
  g = (c >>  6) & 0x1F; // rrrrrGGGGGgbbbbb
  b = (c )      & 0x1F; // rrrrrggggggBBBBB
#elif (8 == nPlanes)
  r = c.red;
  g = c.green;
  b = c.blue;
#else
#error drawPixel Unsupported number of planes
#endif
*/  

  // Loop counter stuff
  bit   = 2;
  limit = 1 << nPlanes;

  if(y < nRows) {
    // Data for the upper half of the display is stored in the lower
    // bits of each byte.
    ptr = &matrixbuff[backindex][y * WIDTH * nPackedPlanes + x]; // Base addr
// FIXME: Adapt for nBuf > 2
    // Plane 0 is a tricky case -- its data is spread about,
    // stored in least two bits not used by the other planes.
    ptr=ptr+WIDTH*2;
    *ptr &= ~B00000011;            // Plane 0 R,G mask out in one op
    if(r & 1) *ptr |=  B00000001;  // Plane 0 R: 64 bytes ahead, bit 0
    if(g & 1) *ptr |=  B00000010;  // Plane 0 G: 64 bytes ahead, bit 1
    ptr=ptr-WIDTH;
    if(b & 1) *ptr |=  B00000001;  // Plane 0 B: 32 bytes ahead, bit 0
    else      *ptr &= ~B00000001;  // Plane 0 B unset; mask out
    // The remaining three image planes are more normal-ish.
    // Data is stored in the high 6 bits so it can be quickly
    // copied to the DATAPORT register w/6 output lines.
    ptr=ptr-WIDTH;
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
    ptr = &matrixbuff[backindex][(y - nRows) * WIDTH * nPackedPlanes + x];
// FIXME: Adapt for nBuf > 2
    *ptr &= ~B00000011;               // Plane 0 G,B mask out in one op
    if(r & 1)  ptr[WIDTH] |=  B00000010; // Plane 0 R: 32 bytes ahead, bit 1
    else       ptr[WIDTH] &= ~B00000010; // Plane 0 R unset; mask out
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

// DrawPixel with interleaved color
// color = bgr4 bgr3 bg|r2 bgr1 bgr0 g-1

// Benchmark (on Stellaris LP, with refresh going): 
//  Regular drawPixel code takes about 400 (397) cycles
//  Interleaved color takes about 300 (291) cycles
//  Interleaved code saves about 100 (107) cycles per call 
//   So, if code is correct, could cut about 25% off pixel time

void RGBmatrixPanel::drawPixelI(int16_t x, int16_t y, uint16_t c) {
  uint8_t bit, limit, *ptr;

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

  // Loop counter stuff
  bit   = 2;
  limit = 1 << nPlanes;

  if(y < nRows) {
    // Data for the upper half of the display is stored in the lower
    // bits of each byte.
    ptr = &matrixbuff[backindex][y * WIDTH * nPackedPlanes + x]; // Base addr
// FIXME: Adapt for nBuf > 2
    // Plane 0 is a tricky case -- its data is spread about,
    // stored in least two bits not used by the other planes.
    ptr=ptr+WIDTH*2;
    *ptr &= ~B00000011;            // Plane 0 R,G mask out in one op
    *ptr |= c >> 1 & B00000011;        // Plane 0 G,R: 64 bytes ahead, bit 1,0
    
//    if(r & 1) *ptr |=  B00000001;  // Plane 0 R: 64 bytes ahead, bit 0
//    if(g & 1) *ptr |=  B00000010;  // Plane 0 G: 64 bytes ahead, bit 1
    ptr=ptr-WIDTH;
    if(c & 8) *ptr |=  B00000001;  // Plane 0 B: 32 bytes ahead, bit 2
    else      *ptr &= ~B00000001;  // Plane 0 B unset; mask out
    // The remaining three image planes are more normal-ish.
    // Data is stored in the high 6 bits so it can be quickly
    // copied to the DATAPORT register w/6 output lines.
    c >>= 2;  // Line up the next bgr triple as bits 2-4
    ptr=ptr-WIDTH;
    for(; bit < limit; bit <<= 1) {
      *ptr &= ~B00011100;             // Mask out R,G,B in one op
      *ptr |= (uint8_t) c & B00011100;
//      if(r & bit) *ptr |= B00000100;  // Plane N R: bit 2
//      if(g & bit) *ptr |= B00001000;  // Plane N G: bit 3
//      if(b & bit) *ptr |= B00010000;  // Plane N B: bit 4
      c >>= 3;                        // On to the next triple
      ptr  += WIDTH;                  // Advance to next bit plane
    }
  } else {
    // Data for the lower half of the display is stored in the upper
    // bits, except for the plane 0 stuff, using 2 least bits.
    ptr = &matrixbuff[backindex][(y - nRows) * WIDTH * nPackedPlanes + x];
// FIXME: Adapt for nBuf > 2
    *ptr &= ~B00000011;               // Plane 0 G,B mask out in one op
    ptr[WIDTH] |= ((uint8_t) c) & B00000010 ;// Plane 0 R: 32 bytes ahead, bit 1
//    if(r & 1)  ptr[WIDTH] |=  B00000010; // Plane 0 R: 32 bytes ahead, bit 1
//    else       ptr[WIDTH] &= ~B00000010; // Plane 0 R unset; mask out
    *ptr     |= ((uint8_t) c >> 2) & B00000011 ; // Plane 0, B, G bit 1, 0
//    if(g & 1) *ptr     |=  B00000001; // Plane 0 G: bit 0
//    if(b & 1) *ptr     |=  B00000010; // Plane 0 B: bit 1
    uint32_t ctemp = c << 1;  // Line up the next bgr triple as bits 5-7
      // Need a longer word since doing left shift
    for(; bit < limit; bit <<= 1) {
      *ptr &= ~B11100000;             // Mask out B,G,R in one op
      *ptr |= (uint8_t) ctemp & B11100000;
//      if(r & bit) *ptr |= B00100000;  // Plane N R: bit 5
//      if(g & bit) *ptr |= B01000000;  // Plane N G: bit 6
//      if(b & bit) *ptr |= B10000000;  // Plane N B: bit 7
      ctemp >>= 3;                      // On to the next triple      
      ptr  += WIDTH;                  // Advance to next bit plane
    }
  }
}

uint16_t RGBmatrixPanel::getPixel(int16_t x, int16_t y) const {
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
    ptr = &matrixbuff[1-backindex][y * WIDTH * nPackedPlanes + x]; // Base addr
// FIXME: Adapt for nBuf > 2
    // Plane 0 is a tricky case -- its data is spread about,
    // stored in least two bits not used by the other planes.
    if (ptr[WIDTH*2] & B00000001) r |= 1;   // Plane 0 R: 2*WIDTH bytes ahead, bit 0
    if (ptr[WIDTH*2] & B00000010) g |= 1;   // Plane 0 G: 2*WIDTH bytes ahead, bit 1
    if (ptr[WIDTH] & B00000001) b |= 1;     // Plane 0 B: WIDTH bytes ahead, bit 0
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
    ptr = &matrixbuff[1-backindex][(y - nRows) * WIDTH * nPackedPlanes + x];
// FIXME: Adapt for nBuf > 2
    if (ptr[WIDTH] & B00000010) r |= 1;   // Plane 0 R: WIDTH bytes ahead, bit 1
    if (*ptr    & B00000001) g |= 1;   // Plane 0 G: bit 0
    if (*ptr    & B00000010) b |= 1;   // Plane 0 B: bit 1
    for(; bit < limit; bit <<= 1) {
      if (*ptr & B00100000) r |= bit;  // Plane N R: bit 5
      if (*ptr & B01000000) g |= bit;  // Plane N G: bit 6
      if (*ptr & B10000000) b |= bit;  // Plane N B: bit 7
      ptr  += WIDTH;                   // Advance to next bit plane
    }
  }

//#if (4 == nPlanes)
  return Color444(r, g, b);
/*
#elif (5 == nPlanes)
//  return Color555(r, g, b);
  // RRRRRGGGGGgBBBBB
  return ((r & 0x1F) << 11) | 
         ((g & 0x1F) <<  6) | ((g & 0x10) << 2) |
         ((b & 0x1F) <<   ) | ;
#elif (8 == nPlanes)
  return rgb24(r, g, b);
#else
#error getPixel Unsupported number of planes
#endif
*/
}



void RGBmatrixPanel::fillScreen(uint16_t c) {
  if((c == 0x0000) || (c == 0xffff)) {
    // For black or white, all bits in frame buffer will be identically
    // set or unset (regardless of weird bit packing), so it's OK to just
    // quickly memset the whole thing:
    memset(matrixbuff[backindex], c, bufferSize() );
  } else {
    // Otherwise, need to handle it the long way:
// TODO: Could fill one row, then copy that
    Adafruit_GFX::fillScreen(c);
  }
}

// -------------------- Faster draw --------------------

// Convert color argument to internal color, then pass that on down to speed up drawing

// The ColorI, drawPixelI and I versions of various line functions are used internally,
// it is not necesarry to call them in order to get speedup, but they are left accessible
// to the user if desired.
// Use ColorI to convert AdafruitGFX color to interleaved color, 
// then pass that as color argument of I functions.


// benchmark: On Stellaris LP, with display going
// Using interleaved color speeds up a horizontal line by about 20%
//  Normal color: 12,700 cycles, interleaved color 10,000 cycles, saves 2700 cycles 2700/12700 ~= 20%

#if defined(COLORI_DRAW)
void RGBmatrixPanel::drawRect(int16_t x, int16_t y,
			    int16_t w, int16_t h,
			    crgb16_t color) {
  crgb16i_t icolor = ColorIntern(color);

  drawFastHLineI(x, y, w, icolor);
  drawFastHLineI(x, y+h-1, w, icolor);
  drawFastVLineI(x, y, h, icolor);
  drawFastVLineI(x+w-1, y, h, icolor);
}

void RGBmatrixPanel::fillRect(int16_t x, int16_t y, int16_t w, int16_t h,
			    crgb16_t color) {
  crgb16i_t icolor = ColorIntern(color);

  for (int16_t i=x; i<x+w; i++) {
    drawFastVLineI(i, y, h, icolor);
  }
}

// Bresenham's algorithm - thx wikpedia
void RGBmatrixPanel::drawLine(int16_t x0, int16_t y0,
			    int16_t x1, int16_t y1,
			    uint16_t color) {
  crgb16i_t icolor = ColorIntern(color);

  int16_t steep = abs(y1 - y0) > abs(x1 - x0);
  if (steep) {
    swap(x0, y0);
    swap(x1, y1);
  }

  if (x0 > x1) {
    swap(x0, x1);
    swap(y0, y1);
  }

  int16_t dx, dy;
  dx = x1 - x0;
  dy = abs(y1 - y0);

  int16_t err = dx / 2;
  int16_t ystep;

  if (y0 < y1) {
    ystep = 1;
  } else {
    ystep = -1;
  }

  for (; x0<=x1; x0++) {
    if (steep) {
      drawPixelI(y0, x0, icolor);
//      drawPixel(y0, x0, color);
    } else {
      drawPixelI(x0, y0, icolor);
//      drawPixel(y0, x0, color);
    }
    err -= dy;
    if (err < 0) {
      y0 += ystep;
      err += dx;
    }
  }
}

// Bresenham's algorithm - thx wikpedia
void RGBmatrixPanel::drawLineI(int16_t x0, int16_t y0,
			    int16_t x1, int16_t y1,
			    crgb16i_t color) {
  int16_t steep = abs(y1 - y0) > abs(x1 - x0);
  if (steep) {
    swap(x0, y0);
    swap(x1, y1);
  }

  if (x0 > x1) {
    swap(x0, x1);
    swap(y0, y1);
  }

  int16_t dx, dy;
  dx = x1 - x0;
  dy = abs(y1 - y0);

  int16_t err = dx / 2;
  int16_t ystep;

  if (y0 < y1) {
    ystep = 1;
  } else {
    ystep = -1;
  }

  for (; x0<=x1; x0++) {
    if (steep) {
      drawPixelI(y0, x0, color);
    } else {
      drawPixelI(x0, y0, color);
    }
    err -= dy;
    if (err < 0) {
      y0 += ystep;
      err += dx;
    }
  }
}

void RGBmatrixPanel::drawFastVLineI(int16_t x, int16_t y,
				 int16_t h, crgb16i_t color) {
  drawLineI(x, y, x, y+h-1, color);
}

void RGBmatrixPanel::drawFastHLineI(int16_t x, int16_t y,
				 int16_t w, crgb16i_t color) {
  drawLineI(x, y, x+w-1, y, color);
}

#endif

// -------------------- Buffers --------------------

// Size of a buffer (in bytes) -- so can use a buffer without assuming as much about encoding
inline uint16_t RGBmatrixPanel::bufferSize() const {
  return WIDTH * nRows * nPackedPlanes;
}

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
//  is a continuation of temporally adjacent patterns 
//  (just part is from the old, and part is from the new))

    while(swapflag == true) delay(1); // wait for interrupt to clear it
    if(copy == true)
      memcpy(matrixbuff[backindex], matrixbuff[1-backindex], bufferSize() );
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
    FadeNAccum = 0;     // Sum of 2*FadeNNext over course of fade
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
boolean RGBmatrixPanel::fading() const {
  return (FadeLen != 0);
}

#endif


// Dump display contents to the Serial Monitor, adding some formatting to
// simplify copy-and-paste of data as a PROGMEM-embedded image for another
// sketch.  If using multiple dumps this way, you'll need to edit the
// output to change the 'img' name for each.  Data can then be loaded
// back into the display using a pgm_read_byte() loop.
void RGBmatrixPanel::dumpMatrix(void) const {

  int i, buffsize = bufferSize();
  uint row = 0, plane = 0;

  Serial.print("// RGBmatrixPanel image, ");
  Serial.print(nPanels);
  Serial.print( (nRows > 8) ? ", 32" : ", 16" );
  Serial.print(" row panels\n");
  Serial.print("static const uint16_t imgsize =");
  Serial.print(buffsize);
  Serial.print(";\n");
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
//      if((i & ((nRows * WIDTH)-1)) == ((nRows * WIDTH)-1)) Serial.print(",\n\\ Plane\n  ");
//      else 
      if((i & (WIDTH-1)) == (WIDTH-1)) {
        if (++plane >= nPackedPlanes){
          row++;
          plane = 0;
        };
        Serial.print(",\n\\\\ row ");
        Serial.print(row);
        Serial.print(" , ");
        Serial.print(row + nRows);
        Serial.print(" plane ");
        Serial.print(plane);
        Serial.print("\n  ");
        }
      else if((i & 7) == 7) Serial.print(",\n  ");
      else             Serial.print(',');
    }
  }
  Serial.println("\n};");
}

// TODO: Add information to matrix dump structure 
//   (number of rows, number panels, number panes, whether it uses fancy panel compression)
//   so can do conversion/error checking when load

// TODO: Add way to display images in flash memory
//   Direct pointer to buffer in flash, flag as read only image

// TODO: Load image routine

// TODO: Maybe pointer argument should be a void * (uint8_t's are not what dealing with)
int8_t RGBmatrixPanel::loadBuffer(const uint8_t *img, uint16_t imgsize) {

  // Could do more sophisticated error handling - adjust for change in nPanels, nRows
  if (imgsize != bufferSize())
    return -1;
#if defined(__AVR__)
#warning Need to write this - use memcpy that uses pgm_read
  unsigned int i;
  for (i = 0; i < imgsize; i++)
    matrixbuff[backindex][i] = pgm_read_byte(img[i]);
#else
// FIXME: Check which way memcpy works (source vs. dest)  
  memcpy(matrixbuff[backindex], img, imgsize);
#endif
  return 0;
}


// -------------------- Refresh timing --------------------

#if defined(__TIVA__)

// Minimum refresh timing
// minRowTime =  minRowTimePerPanel * nPanels + minRowTimeConst
//   minRowTime needs to be greater than the time for the shortest row
//   It also needs to be long enough that planes with extra processing
//     will still fit when done with the appropriate multiplier
//     e.g. if use fancy coding for plane 0, 
//     then handling that must take less than minRowTime << 3 (assuming nPanels is 4)
//
// Can measure by setting BENCHMARK, and checking serial output
//   BENCHMARK result does not include interrupt overhead to call timer handler
//   Includes a couple of assignments, to record start/stop times
//
// TimePerPanel = time for 2 panel - time for 1 panel
// TimeConst    = time for 1 panel - TimePerPanel
//
// Benchmark now includes constant time (so can calculate both const and loop time from one measure)
//   TODO: But calculation of loop time doesn't seem to work well from that
//


// TODO: Reameasure and update minRowTime if make major changes to timer service routine
// FIXME: Need to measure the loop timing values for Connected LP, and for Stellaris LP

// TODO: Timing does not include shift - adjust if DATAPORTSHIFT != 0

// TODO: take off the safety check and see how high can push refresh before it fails
//  (compare real min time to what calculated here)

#if defined(__TM4C1294NCPDT__)
// Connected Launchpad (120 MHz clock)

// Time between beginning of ISR and timer set instruction (approx)
// TODO: Need to estimate values for offset
//  Estimate limit about 230 (with bench code), works at 170 (1 x32 panel) not at 200 (1x32)
#define TIMER_SET_OFFSET 170


#if defined(UNROLL_LOOP)

#if defined( SLOW_NOP1 )
// For version with 1 NOP, x16 panels
//const uint16_t minRowTimePerPanel = 156;         // Ticks per panel for a row
//const uint16_t minRowTimeConst = 350;            // Overhead ticks

// For version with 1 NOP, x32 panels
const uint16_t minRowTimePerPanel = 180;         // Ticks per panel for a row
const uint16_t minRowTimeConst = 290;            // Overhead ticks

//#elif defined( REROLL ) || defined ( REROLL_B )
#else

//const uint16_t minRowTimePerPanel = 160;         // Ticks per panel for a row
//const uint16_t minRowTimeConst = 270;            // Overhead ticks
// Was working with numbers below for 2 x32 panels
const uint16_t minRowTimePerPanel = 170;         // Ticks per panel for a row
const uint16_t minRowTimeConst = 265;            // Overhead ticks

#endif

#else

const uint16_t minRowTimePerPanel = 1610;        // Ticks per panel for a row
const uint16_t minRowTimeConst = 270;            // Overhead ticks

#endif // UNROLL_LOOP

#elif defined(__LM4F120H5QR__) || defined(__TM4C123GH6PM__)
// For Stellaris Launchpad (80 MHz clock)

// Time between beginning of ISR and timer set instruction (approx)
// Fails if 120 (measure says 130, so must not be allowing enough for the loop after)
#define TIMER_SET_OFFSET 100

#if defined(UNROLL_LOOP)

// Based on version without NOP
const uint16_t minRowTimePerPanel = 210;         // Ticks per panel for a row
const uint16_t minRowTimeConst = 150;            // Overhead ticks
// const - 150 to 195 (with bench code)
//  limit const 144, per panel 205, with bench code
//  limit const 130, per panel 205, without bench

#else
const uint16_t minRowTimePerPanel = 1150;        // Ticks per panel for a row
const uint16_t minRowTimeConst = 180;            // Overhead ticks
// minRowTime = 1148 * nPanels + 176 = 1324

#endif // UNROLL_LOOP


#elif defined(__TM4C129XNCZAD__)

#warning "Row timing not determined for TM4C129 DK"

#else

#error Unknown TIVA processor

#endif



// Caution: Blank the display before changing refresh
// May introduce a visual glitch if redefine refresh rate while displaying something
// FIXME: Revise to actually change refresh rate at end of a frame. - Fix added.
// TODO: *** Test the fix.  (Adjust refresh rate during display, see if changes)

// Optimized version can do about 800 refreshes/second 
//   (for 4 bit color on 2 32 row panels with 120MHz clock)

// Minimum refresh frequency (cycles/second)
// Calculate based on maximum timer interrupt period (timer size and clock)
// Calculation is a bit pessimistic (assumes a 16x32 panel) to make it a constant
#define MIN_ROWS 8

const uint32_t minRefreshFreq = (uint32_t) (TIMER_CLK / 
  ((uint64_t) (TIMER_MAX >> nPlanes) * MIN_ROWS * ((1<<nPlanes) - 1)));

// Returns refresh rate
uint16_t RGBmatrixPanel::setRefresh(uint16_t freq){
  uint32_t rowtimetemp;
  
//  uint16_t refreshTime = 1 * ticksPerSecond / refreshFreq;   // Time for 1 display refresh
//  rowtime = refreshTime / (nRows * ((1<<nPlanes) - 1));  // Time to display LSB of one row
//  rowtime = ticksPerSecond / (refreshFreq * nRows * ((1<<nPlanes) - 1));  // Time to display LSB of one row

// Limit frequency based on maximum timer period
  if (freq < minRefreshFreq)
    freq = (minRefreshFreq) ? minRefreshFreq : 1 ;  // Must be at least 1

  rowtimetemp = (uint32_t) TIMER_CLK / ((uint32_t) freq * nRows * ((1<<nPlanes) - 1));  // Time to display LSB of one row

#if defined(DEBUG)
  Serial.print("Min refresh: ");
  Serial.print(minRefreshFreq);

  Serial.print(" Rowtime raw: ");
  Serial.print(rowtimetemp);
#endif

  // Approximate sanity check (try to keep from making refresh rate too high)
  // Typecast gets the compiler to shut up.
  //  All the constituents are unsigned, and there is no way this could come out with
  //  a negative result, but the compiler complains about comparison between signed and unsigned
  if (rowtimetemp < (uint32_t) minRowTimePerPanel * nPanels + minRowTimeConst){
    rowtimetemp = (uint32_t) minRowTimePerPanel * nPanels + minRowTimeConst;

#if defined(DEBUG)
  Serial.print(", Rowtime: ");
  Serial.print(rowtimetemp);
#endif

    }

  ASSERT(rowtimetemp > 0);

  newrowtime = rowtimetemp;   // Set new refresh time (to be picked up on next screen refresh)

  refreshFreq = ((uint32_t) TIMER_CLK / (rowtimetemp * nRows * ((1<<nPlanes) - 1)));
#if defined(DEBUG)
  Serial.print("Freq:");
  Serial.print(freq);
  Serial.print(" refresh:");
  Serial.println(refreshFreq);
#endif
  return refreshFreq;
}


uint16_t RGBmatrixPanel::getRefresh() const {
  return refreshFreq;
}


// TODO: Do dimmer properly - e.g., make it a fraction of maximum on time
// TODO: Possibly - add dimmer sweeps (e.g. fade to black)

// minimum delay, allow time for clearing interrupt and return from interrupt
#define DIM_TIME_MIN  20
// TODO: Figure out what this should be - this was set arbitrarily

// Turn off LEDs for about 4x dtime (clock ticks) per refresh cycle
//  e.g. if dtime = 2xrowtime should cut the time the LEDs are on in half 
//   (and reduce refresh rate to about half)

// Dimmer - set a delay between refreshes (with LEDs turned off)
void RGBmatrixPanel::setDim(uint32_t dtime){
  if (dtime < DIM_TIME_MIN)
    return;
  dimwait = false;  // Next interrupt will be a refresh, one after that will be a delay
  dimtime = dtime;  // Setting dimtime != 0 enables the delay
}


uint32_t RGBmatrixPanel::getDim(void) const {
  return dimtime;
}

#endif // __TIVA__


// -------------------- Interrupt handler stuff --------------------


#if defined(__TIVA__)


void TmrHandler(void)
{

#if defined( BENCHMARK )
  c_tmr_handler_start_old = c_tmr_handler_start;        // Time since last int
  c_tmr_handler_start = HWREG(DWT_BASE + DWT_O_CYCCNT); // beginning of the tested code
#endif

  activePanel->updateDisplay();

// MAP_ version of library call takes longer
//  TimerIntClear( TIMER_BASE, TIMER_TIMA_TIMEOUT );
  HWREG(TIMER_BASE + TIMER_O_ICR) = TIMER_TIMEOUT;    // Inlining code - just in ISR, where speed helpful

#if defined( BENCHMARK )
  c_tmr_handler_end = HWREG(DWT_BASE + DWT_O_CYCCNT);  // end of the tested code  
#endif
}


#else
//#elif defined(__AVR__)

ISR(TIMER1_OVF_vect, ISR_BLOCK) { // ISR_BLOCK important -- see notes later
  activePanel->updateDisplay();   // Call refresh func for active display
  TIFR1 |= TOV1;                  // Clear Timer1 interrupt flag
}

#endif // __TIVA__


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

#define __ASM asm

__attribute__( ( always_inline ) ) static inline void __NOP(void)
{
  __ASM volatile ("nop");
}


void RGBmatrixPanel::updateDisplay(void) {
  uint8_t  i, *ptr;
#if defined(__TIVA__)
  uint32_t duration;
  volatile uint8_t * dataport = &DATAPORT;
  volatile uint8_t * sclkp = sclkport;

#else
  uint16_t t;
  uint16_t duration;
#endif


#if defined(__TIVA__)
#if defined(DIMMER)

  // Dimmer - Insert a delay with LEDs off between refreshes
  if (dimtime){
    if (dimwait){
      // last time was a refresh, so this time we wait
      dimwait = false;   // Next interrupt will not be a wait
      *oeport  = 0xFF;  // Disable LED output during delay

#if defined(BENCHMARK_OE)
  c_tmr_oeoff = HWREG(DWT_BASE + DWT_O_CYCCNT);
  if (oeflag){
    oeflag = false;
    oeon_time = c_tmr_oeoff - c_tmr_oeon;
//    oeon += oeon_time;
    }
#endif

// TODO: should the latch be done now, or should that wait until really going to change addr

      MAP_TimerLoadSet( TIMER_BASE, TIMER_A, dimtime );
      MAP_TimerEnable( TIMER_BASE, TIMER_A );
      return;
    } else
      dimwait = true;
  }
#endif

#endif

  // Calculate time to next interrupt BEFORE incrementing plane #.
  // This is because duration is the display time for the data loaded
  // on the PRIOR interrupt.  
#if defined(__TIVA__)
  duration = (rowtime << plane) - TIMER_SET_OFFSET;

// Draft dimmer - use timer to turn off OE early - would retain refresh rate
//  oetime = ((duration + TIMER_SET_OFFSET - OE_OFF_OFFSET) * dimFactor) / dimmScale;
#else
  // CALLOVERHEAD is subtracted from the result because that time is 
  // implicit between the timer overflow (interrupt triggered) and 
  // the initial LEDs-off line at the start of this method.
  t = (nRows > 8) ? LOOPTIME : (LOOPTIME * 2);
  // FIXME: adjust for number of panels (maybe * nPanels )
  // TODO: Why is t longer for 32x16 than for 32x32?
  duration = ((t + CALLOVERHEAD * 2) << plane) - CALLOVERHEAD;
#endif

// TODO: Comment below about latching when latch rises may be wrong? - 
// Octoscroller said latch happens on falling edge of latch (i.e. this is get ready to latch)
// Below in address prep it says "about to latch now" so think it is when latch goes low
// Moved the comment to where latch is lowered (think that is correct place for it).

// TODO: Could move this closer to where latch is turned on again.
//  oe Has to be off when change address and when latch
//  but probably doesn't have to be off for calculating delay or updating timer
//  (unless there is some minimum time involved).
#if defined(__TIVA__)
  *oeport  = 0xFF;  // Disable LED output during row/plane switchover

// End of display timing  
#if defined(BENCHMARK_OE)
  c_tmr_oeoff = HWREG(DWT_BASE + DWT_O_CYCCNT);
  if (oeflag){
    oeflag = false;
    oeon_time = c_tmr_oeoff - c_tmr_oeon;
//    oeon += oeon_time;
    }
#endif
  *latport = 0xFF;
#else
  *oeport  |= oepin;  // Disable LED output during row/plane switchover
  *latport |= latpin;
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
#if defined(SWAP_AT_END_ROW)
// Do swap when finish drawing all the planes in a row, rather than waiting until end of screen.
// Means not guaranteed to fully display a screen if do two swaps in succession
// Means display handler a little slower (test swapflag on every row, rather than at end screen)
// But also means swapbuffer much faster
    if(swapflag) {    // Swap front/back buffers if requested
      backindex = 1 - backindex;
      swapflag  = false;
    // Change to reading from front buffer, but using current row
      buffptr = matrixbuff[1-backindex] + (buffptr-matrixbuff[backindex]);
// Should be the same as buffptr = matrixbuff[1-backindex] + row * WIDTH;
    }
#endif
    if(++row >= nRows) {        // advance row counter.  Maxed out?
      row     = 0;              // Yes, reset row counter, then...
      if(newrowtime) {          // Update refresh frequency if new one specified
        // TODO: May be simpler to not bother with the test, 
        // but just always copy newrowtime into rowtime
        rowtime = newrowtime;   
        newrowtime = 0;
        };
#if !defined(SWAP_AT_END_ROW)
      if(swapflag) {    // Swap front/back buffers if requested
        backindex = 1 - backindex;
        swapflag  = false;
        buffptr = matrixbuff[1-backindex]; // Reset into front buffer
      }
      else 
#endif
#if defined(FADE)
      if(FadeLen) {
        if (++FadeCnt >= FadeLen){  // Fade done, swap the buffers
          backindex = 1 - backindex;
          FadeLen = 0;
          FadeCnt = 0;
//          FadeNNext = 0;
          FadeNAccum = 0;
// FIXME: Need to get the copy done somehow (or just remove that option)
          buffptr = matrixbuff[1-backindex]; // Reset into front buffer
        }
        else {  // Calculate which buffer to show this time

//  Fade by image flipping (?Delta-Sigma DAC):
//  Does a linear interpolation between the two images.
//  What it should show is an image that is a certain percent FrontBuffer and a certain % next.
//  Use the ammount of time spent showing FrontBuffer vs NextBuffer to approximate that percent
//  For the next interval, display whichever one will bring displayed percent closer to
//    the current target percent.
//  Need a fairly high refresh rate for this to work

//
// if (abs(FadeCnt ^ 2 - 2 * FadeLen * FadeNNext) > abs(FadeCnt ^ 2 - 2 * FadeLen * (FadeNNext + 1)))
//
// To improve efficiency:
//   2*FadeLen*FadeNNext calculated cumulatively - FadeNAccum 
//     (Keep running total, when increment FadeNNext, then FadeNAccum += 2 * FadeLen)
//   FadeCnt^2 used on both sides (calculate once)
//   Could cache 2*FadeLen - but just a bit shift, so may not be worth it

// After efficiency improvements - expression has been tested and works
//          int32_t FadeCntSq = FadeCnt * FadeCnt;
//          if (abs(FadeCntSq - FadeNAccum ) > abs(FadeCntSq - FadeNAccum - (2 * FadeLen) )){

// Following expression should yield the same result as above, with less calculation

// TODO: Would it be more efficient to keep FadeLen as a uint32_t also?
          if (abs(FadeCnt * FadeCnt - FadeNAccum ) > FadeLen ){
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

    // Plane 0 was loaded on prior interrupt invocation and is about to
    // latch now, so update the row address lines before we do that:
#if defined(__TIVA__)
// TODO: could compare to using bitbanding (then just write a 1 or a 0)

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

// FIXME: Check other drivers - should latch be lowered while output is still disabled?
//  In raspi version, latch is lowered while output enable is high
//   (latch set and immediately cleared, suggesting no minimum high time)
//  I changed it so latch set to 0 before re-enable output 
// TODO: Test, see if it helps any with ghosts (e.g. at end of line).

#if defined(__TIVA__)
  *latport = 0;  // Latch data loaded during *prior* interrupt
  
// Beginning of display timing
#if defined(BENCHMARK_OE)
  if (!oeflag){
    c_tmr_oeon = HWREG(DWT_BASE + DWT_O_CYCCNT);
    oeflag = true;
    oeoff_time = c_tmr_oeon - c_tmr_oeoff;
//    oeoff += oeoff_time;
    }
#endif
  *oeport  = 0;  // Re-enable output
#else
  *latport &= ~latpin;   // Latch data loaded during *prior* interrupt
  *oeport  &= ~oepin;   // Re-enable output
#endif


  // buffptr, being 'volatile' type, doesn't take well to optimization.
  // A local register copy can speed some things up:
  ptr = (uint8_t *)buffptr;


// Set timer for next interrupt
#if defined(__TIVA__)

/*
#if defined(DEBUG)
    Serial.print(" du");
    Serial.println(duration);
#endif
*/

#if defined(BENCHMARK)
  c_tmr_handler_tset = HWREG(DWT_BASE + DWT_O_CYCCNT);
#endif

//  TimerDisable( timerBase, timerAB ); // Not needed

// BENCHMARK - together the two timer calls take about 60 cycles (Stellaris) (MAP_ version)
//   Non-MAP_ version takes 8 cycles less than MAP version on Stellaris LP
//   Inline with just the HWREG cuts off another 30+ cycles

#if (A == TIMER_CHANEL)
//  TimerLoadSet( TIMER_BASE, TIMER_A, duration );
    HWREG(TIMER_BASE + TIMER_O_TAILR) = duration;
//  TimerEnable( TIMER_BASE, TIMER_A );
    HWREG(TIMER_BASE + TIMER_O_CTL) |= TIMER_A & (TIMER_CTL_TAEN |
                                                  TIMER_CTL_TBEN);

#elif (B == TIMER_CHANEL)
//  TimerLoadSet( TIMER_BASE, TIMER_B, duration );
    HWREG(TIMER_BASE + TIMER_O_TBILR) = duration;
//  TimerEnable( TIMER_BASE, TIMER_B );
    HWREG(TIMER_BASE + TIMER_O_CTL) |= TIMER_B & (TIMER_CTL_TAEN |
                                                  TIMER_CTL_TBEN);
#else

#error Unrecognized timer chanel

#endif

#else
  ICR1      = duration; // Set interval for next interrupt
  TCNT1     = 0;        // Restart interrupt timer
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
  // Tiva uses masking, so do not worry about changing other pins.
  static const uint8_t tick = 0xFF;
  static const uint8_t tock = 0;
  uint8_t temp;
#endif

#if defined(BENCHMARK)
  c_tmr_handler_loop = HWREG(DWT_BASE + DWT_O_CYCCNT);
#endif

  if(plane > 0) { // 188 ticks from TCNT1=0 (above) to end of function

    // Planes 1-3 copy bytes directly from RAM to PORT without unpacking.
    // The least 2 bits (used for plane 0 data) are presumed masked out
    // by the port direction bits (on Tiva they are masked by port mask).

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

// Always unroll loop on AVR
#if !defined( UNROLL_LOOP )
#define UNROLL_LOOP
#endif

#else				// Code for non AVR (i.e. Due and ARM based systems)


// TODO: Gather what information available on timing constraints.
// 
// Raspberry Pi driver - Says takes 3.4 uSec to clock out the data
//   (i.e. 106ns per data item)
//   However code says clock high for >256ns, then 
//     low for >256ns before output data, >256 ns after output data
//     So one data item would take 768 ns to put out, and 32 would be 24.5 uSec
//
// FPGA driver has a clock divider that goes from 50mHz clock to 10mHz clock
//   (50mHz would be 20ns per clock, 10 mHz would be 100ns per clock)
//
// Octoscroller says 10 MHz seems to be highest reliable clock
//   However, note that they are driving 4 panels in series.
//
// This forum thread
//   http://forums.adafruit.com/viewtopic.php?f=47&t=26130&start=0
//   Says 25MHz may be recomended maximum for some of the parts, with 50MHz absolute max
//     One user with FPGA reports success at 40MHz (25ns), with problems above that.
//
// At a guess, clock speed about 40ns (25mHz) might be reasonable starting point for experiment?
//   That would be 5 cycles at 120mHz, or 3.2 cycles at 80 mHz
//
// Seems to work at about 30 mHz with 2 x32 panels (with some ghosting).
//
//   The initial code gives messed up display
//      has the clock high for about 1 instruction cycle, and low for about 3 cycles.
//   (So on Connected Launchpad the clock is high for about 8.3 ns, and low for about 24 ns)
//
//   Adding one NOP while clock high fixes the display. (5 cycles at 120 mHz)
//     (NOP while clock low does not fix the problem)
//   Rearranging the code so there is an instruction between clock up and clock down 
//     (4 cycles at 120 mHz) also fixes the display.
//     There might be requirements for particular parts of the clock 
//       (e.g. clock needs to be high for so many ns).
//
//    A 25MHz clock evenly divided would be high for 20ns,
//     and a 50MHz clock would be high for 10 ns
//
// Octoscroller says that the data is shifted on falling edge of clock.
//   (Looks like it leaves clock high, whereas Adafruit code leaves clock low).
//


#ifdef SLOW_CLOCK

#define pew DATAPORT = LEFT_SHIFT((*ptr++), DATAPORTSHIFT); SCLKPORT = tick; SCLKPORT = tock;
// This version takes 7 instructions per item
// Works for Connected LP and 2 panels
/* 
    ldrb	r7, [r5, #1]
    strb.w	r7, [r6, #1008]
    ldr	r6, [r4, #88]       ; tick
    strb	r2, [r6, #0]      ;  "
    ldr	r7, [r4, #88]       ; tock
    strb	r3, [r7, #0]      ;  "
    ldr	r6, [r1, #4]
*/

#else

// For sclkport, dataport - local variables easier for the compiler to access/optimize
// TODO: Might be able to do it with the dataport variable in "this" instead?

// Need instruction between clock toggle instructions on the TM4C1294,
// otherwise it is a little too fast for the panel.

// Use a NOP
#ifdef SLOW_NOP1

#define pew *dataport = LEFT_SHIFT((*ptr++), DATAPORTSHIFT); * sclkp = tick; __NOP(); * sclkp = tock;
/* 4 instructions plus one pad (pad keep clock high for extra 8 ns)

ldrb	r0, [r6, #1]  ; Load value
strb	r0, [r7, #0]  ; Output value
strb	r2, [r5, #0]  ; tick
nop
strb	r3, [r5, #0]  ; tock
*/

// Putting NOP in other positions does not help

#elif defined(REROLL)

/* Rearrange instructions to eliminate NOP - fetch next value while clocking out last value.
  This works.

ldrb	r0, [r6, #1]  ; Setup


strb	r0, [r7, #0]  ; Output value
strb	r2, [r5, #0]  ; tick
ldrb	r0, [r6, #2]  ; Load value for next time around
strb	r3, [r5, #0]  ; tock
...

// *** CAUTION: This does one more *ptr++ than the base version
// so it leaves ptr pointing beyond the end of the data area
// At the moment this is not a problem, since ptr is not used after
// However it reads 1 beyond the end of data - which could be a problem if memory protection was used.
//  Workaround - could add an extra dummy entry at end of last buffer
*/


#define PREP temp = LEFT_SHIFT((*ptr++), DATAPORTSHIFT);

#define pew *dataport = temp; * sclkp = tick; \
  temp = LEFT_SHIFT((*ptr++), DATAPORTSHIFT); * sclkp = tock;

#elif defined(REROLL_B)

//  Different rearrangement - works
// Read first, then tock, then output data, then tick
//  So the first tock will be ignored (since clock should already be tocked)
//  Means need to add extra tock at end to finish

/*
ldrb	r0, [r6, #2]  ; Load value
strb	r3, [r5, #0]  ; tock
strb	r0, [r7, #0]  ; Output value
strb	r2, [r5, #0]  ; tick
...
*/


#define pew \
  temp = LEFT_SHIFT((*ptr++), DATAPORTSHIFT); * sclkp = tock; \
  *dataport = temp; * sclkp = tick;

//  Then, rather than needing prep, need an extra tock at the end

// TODO: Probably do not need the NOP (because of loop overhead)
#define FINISHUP   __NOP(); * sclkp = tock;

#else

// Basic fast version - 
// Makes the inner "loop" 4 instructions, a load and 3 stores
//   (+1 instruction if a shift is needed)

// Clock pulse not long enough at 120 MHz (TM4C129x), may be okay on TM4C123x

/* 
ldrb	r0, [r6, #1]
strb	r0, [r7, #0]
strb	r2, [r5, #0]    ; tick
strb	r3, [r5, #0]    ; tock
...
*/

#define pew *dataport = LEFT_SHIFT((*ptr++), DATAPORTSHIFT); * sclkp = tick; * sclkp = tock;

#endif
#endif


// Possible speedup might be to read a word,
// then shift it to output 4 successive bytes.
// (Reduce memory fetch overhead).
// TODO: Try this, see if it improves performance
// However, may not be much point since already pushing speed limit (at least on Connected Launchpad)
//
// uint32_t * ptr;
// t = *ptr++; *dataport = t; *dataport = t>>8; * dataport = t>>16; ...
//
// ldrb.w r7, [r5, #0]
// strb	r7, [r2, #0]
// ... toggle sclkp
// lsrs	r7, r7, #8
// strb	r7, [r2, #0]
// ...
// ldrb.w r7, [r5, #4]
//


#endif

#if defined( UNROLL_LOOP )
    uint8_t panelcount;

#if defined( PREP )    
    PREP
#endif


    for (panelcount = 0; panelcount < nPanels; panelcount++)
    {
    	// Loop is unrolled for speed:
      pew pew pew pew pew pew pew pew
      pew pew pew pew pew pew pew pew
      pew pew pew pew pew pew pew pew
      pew pew pew pew pew pew pew pew
    } 

#if defined( FINISHUP )
    FINISHUP
#endif

#else     // Loopy version
/*
    // This version was tested and does work
    for(i=0; i<WIDTH; i++)
    {
      * dataport = LEFT_SHIFT((ptr[i]), DATAPORTSHIFT);
      * sclkp = tick;
      * sclkp = tock;
    }
*/
// Using for loop pointer, rather than indexing, saves 2 more instructions per loop
    uint8_t *pFinal = ptr + WIDTH;  // Avoid recalculation in loop
    for(; ptr<pFinal; ptr++)
    {
#ifdef SLOW_CLOCK
      DATAPORT = LEFT_SHIFT((*ptr), DATAPORTSHIFT);;
      SCLKPORT = tick;
      SCLKPORT = tock;
#else
      * dataport = LEFT_SHIFT((*ptr), DATAPORTSHIFT);
      * sclkp = tick;
// FIXME: This might need an extra instruction on Connected LP
//   (either NOP, or do code rearrangement as in REROLL)
      * sclkp = tock;
#endif
    }
#endif
    buffptr += WIDTH;


  } else { // AVR: 920 ticks from TCNT1=0 (above) to end of function

    // Planes 1-3 (handled above) formatted their data "in place,"
    // their layout matching that of the output PORT register (where
    // 6 bits correspond to output data lines), maximizing throughput
    // as no conversion or unpacking is needed.  Plane 0 then takes up
    // the slack, with all its data packed into the 2 least bits not
    // used by the other planes.  This works because the unpacking and
    // output for plane 0 is handled while plane 3 is being displayed...
    // because binary coded modulation is used (not PWM), that plane
    // has the longest display interval, so the extra work fits.
    for(i=0; i<WIDTH; i++) {
#if defined(__AVR__)
      DATAPORT =
        ( ptr[i]    << 6)                   |
        ((ptr[i+WIDTH] << 4) & 0x30) |
        ((ptr[i+(WIDTH*2)] << 2) & 0x0C);
      SCLKPORT = tick;
      SCLKPORT = tock;
#else
#ifdef SLOW_CLOCK
      DATAPORT =
        LEFT_SHIFT((( ptr[i]    << 6)                   |
        ((ptr[i+WIDTH] << 4) & 0x30) |
        ((ptr[i+(WIDTH*2)] << 2) & 0x0C)), DATAPORTSHIFT);
      SCLKPORT = tick;
      SCLKPORT = tock;
#else      
#ifdef SLOW_NOP1
      * dataport =
        LEFT_SHIFT((( ptr[i]    << 6)                   |
        ((ptr[i+WIDTH] << 4) & 0x30) |
        ((ptr[i+(WIDTH*2)] << 2) & 0x0C)), DATAPORTSHIFT);
      * sclkp = tick;
      __NOP();
      * sclkp = tock;
#elif defined(REROLL) || defined(REROLL_B)
//  TODO: Test
//   Re-rolled the loop to put useful operation to replace the NOP
      temp =
        LEFT_SHIFT((( ptr[i]    << 6)                   |
        ((ptr[i+WIDTH] << 4) & 0x30) |
        ((ptr[i+(WIDTH*2)] << 2) & 0x0C)), DATAPORTSHIFT);
      * sclkp = tock;
      * dataport = temp;
      * sclkp = tick;
#endif // SLOW_NOP1
#endif // SLOW_CLOCK
#endif
    }
#if defined(REROLL) || defined(REROLL_B)
// TODO: Probably do not need the NOP (because of loop overhead)
    __NOP();
    * sclkp = tock;
#endif
  }
}

