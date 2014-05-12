// Count execution time for Cortex M processors
// Stellaris/Tiva
// http://forum.stellarisiti.com/topic/1908-execution-time-the-easy-way/

#include "inc/hw_nvic.h"          /* for definition of NVIC_DBG_INT */
#include "inc/hw_memmap.h"        /* for definition of DWT_BASE */
#include "inc/hw_types.h"         /* for definition of HWREG */

#define DWT_O_CYCCNT 0x00000004
//static  
uint32_t     c_start, c_stop;

// declaration of an initialization function
void  EnableTiming(void);
void  ResetTiming(void);
uint32_t ReadTiming(void);

// definition of that function
/******************************************************************************/
void EnableTiming(void){
   HWREG(NVIC_DBG_INT) |= 0x01000000;      /*enable TRCENA bit in NVIC_DBG_INT*/
   HWREG(DWT_BASE + DWT_O_CYCCNT) = 0;     /* reset the counter */
   HWREG(DWT_BASE) |= 0x01;                /* enable the counter */
//   enabled = 1;
   c_start = 0;
   c_stop = 0;
}
/******************************************************************************/

/*
 // and then, in the code to be examined write these:
 EnableTiming();
 c_start = HWREG(DWT_BASE + DWT_O_CYCCNT); // at the beginning of the tested code
 // your code follows here
 c_stop = HWREG(DWT_BASE + DWT_O_CYCCNT);  // at the end of the tested code

 // then c_stop - c_start is the execution number of cycles for the code;
 // just multiply with clock period to find out the execution time of the code.
 // At 80MHz, the 32 bit counter gives you a total time 2^32 x 12.5ns = ~53 seconds!
 // At 120MHz, 2^32 x 8.3ns = ~35 seconds
*/

inline uint32_t ReadTiming(void) {return HWREG(DWT_BASE + DWT_O_CYCCNT);}

#define READTIMING (HWREG(DWT_BASE + DWT_O_CYCCNT))

// Reset counter - not tested
inline void ResetTiming(void) {HWREG(DWT_BASE + DWT_O_CYCCNT) = 0;}


 // Delay based on cyclecount
 // FIXME: handle timer overflow
 /**********************************************************/
 void TimingDelay(unsigned int tick)
 {
   unsigned int start, current;

//   HWREG(DWT_BASE + DWT_O_CYCCNT) = 0;      // Reduce chances of overflow - untested
   start = HWREG(DWT_BASE + DWT_O_CYCCNT);
   do {
	current = HWREG(DWT_BASE + DWT_O_CYCCNT);
   } while((current-start)<tick);
 }
/*********************************************************/
