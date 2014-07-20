/*
RGBmatrixPanel library for 16x32 and 32x32 RGB LED matrix panels.

Configuration - settings that need to be done at compile time.
*/


#ifndef RGBmatrixPanelConfig_h
#define RGBmatrixPanelConfig_h



// FIXME: What is the Macro that indicates Stellaris/Tiva LP in Energia?
// __arm__ - GCC any ARM processor, but that probably applies to any ARM (e.g. Tiva, CC3200, SAM)
// __AVR__ - Atmel AVR
//
// __MSP430_CPU__ - TI MSP430
// Is there a macro for the TI CC3200?
//   (Since it is ARM might be able to run the panel, but uses different libraries so might not be easy port)
// Is there a macro for TI C2000 launchpad?
//
// Just a temporary patch - until find the proper macro (if there is one)

#if defined(ENERGIA)

#if defined(__arm__)

#ifndef __TIVA__

#if defined(__TM4C129XNCZAD__) || defined(__TM4C1294NCPDT__) || defined(__LM4F120H5QR__) || defined(__TM4C123GH6PM__)

#define __TIVA__

#else

#error "**** Unrecognized ARM processor ****"

#endif

#endif //__TIVA__

#else

#error "**** Unsupported processor ****"

#endif

#endif // ENERGIA


// -------------------- Configuration --------------------


// define DEBUG to enable debugging messages on serial console
#define DEBUG

// Serial port speed for debug output (leave undefined if calling program sets up serial console)
#define DBUG_CON_SPEED  9600

// On Tiva, define BENCHMARK to measure TimerHandler time (may work on other ARM)
#if !defined(__AVR__)
#define BENCHMARK
#define BENCHMARK_OE
#endif


// Display refresh loop optimization controls

// Define UNROLL_LOOP to speed up display refresh by linearizing the inner loop
#define UNROLL_LOOP

// Processor can put out data too fast for the display - various ways of slowing it down
// (Use only one of following)

// Slow down the clock pulse (use less efficient code)
//#define SLOW_CLOCK

// Add extra NOP during clock pulse (slow down data output a little)
// extra NOP needed on TM4C1294, suspect may not need on TM4C123x
// TODO: Test on TM4C123x - see if need the extra NOP
// #define SLOW_NOP1

// TODO: Test Alternative version - rearrange instructions may eliminate need for NOP
//#define REROLL
#define REROLL_B

#if defined(__TIVA__)

// Port/pin definitions for various launchpads
// TODO: Could make data port configurable at run time 
//   Compiling the port into the library does not help performance on ARM

#if defined(__TM4C129XNCZAD__)

#warning "Ports not defined for TM4C129 DK"

#elif defined(__TM4C1294NCPDT__)
// Tiva Connected Launchpad

// Candidates for data port:
//   (have enough pins on booster pack connector without conflicting functions) 
//   PortE (BP1), PortK (BP2), or PortL (BP1) (pins 0-5)

#define DATAPORTMASK  B11111100
#define DATAPORT      (*portMaskedOutputRegister(PK, DATAPORTMASK))
#define DATAPORTBASE  ((uint32_t)portBASERegister(PK))

// Dataport values start as bits 1-7 of the byte
// To use lower pin numbers, define DATAPORTSHIFT as the number of bits to shift the value left
// e.g. if connect panel to pins 0-5, then define DATAPORTSHIFT to be -2 
// and DATAPORTMASK B00111111
//#define DATAPORTSHIFT -2
//#define DATAPORTMASK  B00111111
// If leave DATAPORTSHIFT undefined there will be no shifting

// Some ARM processors (e.g. SAM) have more than 8 pins per port,
// therefore more likely to want to shift the pins on those processors.
// So DATAPORTSHIFT is defined as a left shift, 
// even though right (negative) shifts are the only useful ones on Tiva


// On Tiva (unlike AVR), defining SCLKPORT as a constant slows down refresh code,
// instead sclkport is derived from sclk pin.
//#define SCLKPORT      (*portDATARegister(PM))


#elif defined(__LM4F120H5QR__) || defined(__TM4C123GH6PM__)
// Stellaris or Tiva Launchpad

// Data port should be PA or PB, 
// PA: Do not use pins 0,1 - console Uart
// PB: use caution in other pin assignments, 
//     since PB6 and PB7 are connected to PD0 and PD1

#define DATAPORTMASK  B11111100
#define DATAPORT      (*portMaskedOutputRegister(PA, DATAPORTMASK))
#define DATAPORTBASE  ((uint32_t)portBASERegister(PA))

// Used in testing - data makes onboard LED blink
//#define DATAPORT      (*portMaskedOutputRegister(PF, DATAPORTMASK))
//#define DATAPORTBASE  ((uint32_t)portBASERegister(PF))
//#define DATAPORTMASK B01111110
//#define DATAPORTSHIFT -1

// On Tiva (unlike AVR), defining SCLKPORT as a constant slows down refresh code,
// instead sclkport is derived from sclk pin.
//#define SCLKPORT      (*portDATARegister(PB))

#else

#error Unknown TIVA processor

#endif


// Timer selection

// Note: 
// Timer 5 is used by Energia time tracking
// Timer 2A is used by Servo library
// Timer 4A is used by Tone library

// Stellaris Launchpad: Energia analog write uses regular timer 0-3, and wide timer 0-3, 6, 7
//     Thus wide timers 4 and 5 appear to be available.
// Connected LP: Energia analog write uses timers 0-5 (so 6 and 7 should be available)

// Energia doesn't use all the timers, maybe better to just grab one not used
//   WTIMER4 or WTIMER5 on Stellaris LP, TIMER6 or TIMER7 on Connected LP
//   Or could select timer that does PWM for a pin that is otherwise in use.
//     (For instance, pins used for RGBMatrix)

//  TODO: Check Energia timer use for timers 4 and 5 on Connected LP
//    Seems strange that, on Connected LP, Timers 4 and 5 are used both by tone and by PWM??

// TODO: Make timer selectable by user code


#if defined(__TM4C1294NCPDT__)

#define TIMER TIMER6

#elif defined(__LM4F120H5QR__) || defined(__TM4C123GH6PM__)

#define TIMER WTIMER4

// Can use timer or wide timer
// For regular timer, uses full width timer
// For wide timer, uses timer A

// TODO: could adapt code to allow use of timer B of wide timer
#else

#warning "Unrecognized Tiva processor, timer not defined"

#endif



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


#if defined(__TIVA__)

static const uint16_t defaultRefreshFreq = 100; // Cycles per second 

//const uint32_t ticksPerSecond = 1000000; // Number of timer ticks in 1 second

#endif



// Control whether to include fade support (not much point on AVR - too slow for PWM fade)
#if !defined(__AVR__)
#define FADE
#endif

const uint8_t nBuf = 2;


// Number of bits per color per pixel
// Will not work for nPlanes < 4 (would need fixing to not do packing)

// Use macro since, unfortunately, can not do conditional compliation based on constant
#define nPlanes 5
//const uint8_t nPlanes = 4;


#endif
