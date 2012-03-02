//
// This code is distributed under the GNU General Public License
// which can be found at http://www.gnu.org/licenses/gpl.txt
// It is meant for information purposes only and is in no way guaranteed
// to be accurate, bug free or even working.
// Use it at your own risk.
// 

#ifndef _QUADCTRL_GLOBAL_H
#define _QUADCTRL_GLOBAL_H

#include <avr/io.h>
#include <avr/interrupt.h>

// global AVRLIB defines
#include "avrlibdefs.h"
// global AVRLIB types definitions
#include "avrlibtypes.h"



/*
Quadcopter hardware wiring:

GYRO(ITG3200) - atm32
 SDA(24) -> SDA(20)
 SCL(23) -> SCL(19) 
 INT(12) -> PB0(40)

Accel(LIS3Lv02) - atm32
 CS(6)  -> SS/PB4(44)
 SPC(5) -> SCK(3)
 SDI(3) -> MOSI(1)
 SDO(2) -> MISO(2)
 RDY(1) -> INT2/PB2(42)

PowerMon:
  -> ADC7(30)

Motors:

 frontLeft  -> OC2(16)
 frontRight -> OC0(43)
 backLeft   -> OC1B(13)
 backRight  -> OC1A(14)

ZigBee:
 DIN(3)  -> TXD(10)
 DOUT(2) -> RXD(9)
 CTS(12) -> PB1(41)
	

*/

// supress timer.c interrupt handlers
#define TIMER_DO_NOT_IMPLEMENT_INTERRUPT_HANDLERS

////// Clock

// CPU clock speed
#define F_CPU          7372800             			// 7.37MHz processor
#define CYCLES_PER_US ((F_CPU+500000)/1000000) 		// cpu cycles per microsecond

//for rprintf. use better printf
#define RPRINTF_COMPLEX
#define RPRINTF_FLOAT




//// Hardware description:


#define LIS3L02_CS_DDR  DDRB
#define LIS3L02_CS_PORT PORTB
#define LIS3L02_CS_BIT  4

#define LIS3L02_DRY_PORT PORTB
#define LIS3L02_DRY_PIN	 PINB
#define LIS3L02_DRY_DDR	 DDRB
#define LIS3L02_DRY_BIT	 2

#define ITG3200_INT_PIN PINB
#define ITG3200_INT_BIT 0  



#ifndef BT
#define BT(A) (1<<(A))
#endif


inline BOOL xbee_readyToSend()
{
	return !(PINB & BT(1));
}


#endif
