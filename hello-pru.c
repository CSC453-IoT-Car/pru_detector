/*
 * Source Modified by Zubeen Tolani < ZeekHuge - zeekhuge@gmail.com >
 * Based on the examples distributed by TI
 *
 * Copyright (C) 2015 Texas Instruments Incorporated - http://www.ti.com/
 *
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *	* Redistributions of source code must retain the above copyright
 *	  notice, this list of conditions and the following disclaimer.
 *
 *	* Redistributions in binary form must reproduce the above copyright
 *	  notice, this list of conditions and the following disclaimer in the
 *	  documentation and/or other materials provided with the
 *	  distribution.
 *
 *	* Neither the name of Texas Instruments Incorporated nor the names of
 *	  its contributors may be used to endorse or promote products derived
 *	  from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
				  
#include <stdint.h>
#include <pru_cfg.h>
#include <pru_ctrl.h>
#include "resource_table_empty.h"

#define	INS_PER_US 200           // 5ns per instruction
#define INS_PER_DELAY_LOOP 2	 // two instructions per delay loop
#define DELAY_CYCLES_US (INS_PER_US / INS_PER_DELAY_LOOP)

#define CARRIER_PERIOD = (5264)
#define CARRIER_PULSE_WIDTH (2632)  // 38kHz = 26.32us * 200 cycles/us = 5264 cycles / signal period. signal is on for half of period -> /2
#define CARRIER_PULSE_US (26.32)

#define GPIO1 0x4804C000
#define GPIO_CLEARDATAOUT 0x190
#define GPIO_SETDATAOUT 0x194
#define USR0 (1<<21)
#define USR1 (1<<22)
#define USR2 (1<<23)
#define USR3 (1<<24)
unsigned int volatile * const GPIO1_CLEAR = (unsigned int *) (GPIO1 + GPIO_CLEARDATAOUT);
unsigned int volatile * const GPIO1_SET = (unsigned int *) (GPIO1 + GPIO_SETDATAOUT);

volatile register unsigned int __R30;
volatile register unsigned int __R31;
#define PRU0_GPIO (1<<2)

#ifndef DECAY_RATE
#define DECAY_RATE 100
#endif
#ifndef DELAY_CYCLES
#define DELAY_CYCLES DELAY_CYCLES_US
#endif

//#define LED_DISABLE 1

const int decay = DECAY_RATE;

#define LED_DISABLE 1


void mark(unsigned pulses){
	//for now this just makes one pulse
	while(pulses > 0){
		__R30 |= PRU0_GPIO;
		__delay_cycles(CARRIER_PULSE_WIDTH);
		__R30 ^= PRU0_GPIO;
		__delay_cycles(CARRIER_PULSE_WIDTH);
		pulses--;
	}
	
}

void space(unsigned pulses){
	__R30 &= ~PRU0_GPIO;
	while(pulses > 0){
		__delay_cycles(CARRIER_PULSE_WIDTH);
		__delay_cycles(CARRIER_PULSE_WIDTH);
		pulses--;
	}
}


void putBits(unsigned data, unsigned char nbits){
	unsigned char i;
	for (i = 0; i < nbits; i++) {
		//mark always 600us
        if (data & 0x8000) {//always 16 bits
        	//1 is space for 1600
          mark(23);  space(61);
        } else {
        	//0 is space for 460
          mark(23 ); space(17);
        };
        data <<= 1;
     }
}

void main(void) {
	
	// int i, j;

	/* Clear SYSCFG[STANDBY_INIT] to enable OCP master port */
	
	/*
	
	 unsigned int loops, delay;
    for (loops = 0; loops < 10; loops++) {
     //   __R30 = __R30 | (1 << 15); // Turn on the LED
        __R30 = __R30 | PRU0_GPIO; 
        for (delay = 0; delay < 20000000; delay++) { // loop delay
        }
        __R30 = __R30 ^= PRU0_GPIO; // Turn off the LED
       // __R30 - __R30 & ~1;
        __delay_cycles(60000000); // Intrinsic method delay
    }
    */
    
    
    /*
    __R30 = __R30 |= PRU0_GPIO;
    __delay_cycles(200000000);
    __R30 = __R30 ^= PRU0_GPIO;
    __delay_cycles(200000000);
    __R30 = __R30 |= PRU0_GPIO;
    __delay_cycles(200000000);
    __R30 = __R30 ^= PRU0_GPIO;
    */
    
    //011*****10*****0
    
    __R30 &= ~PRU0_GPIO;
    
    while(1){
    	mark(171);
    	space(171);
    	putBits(0x00006b96, 16);
    	__delay_cycles(200000000);
    }
    
    
	
	
	
	//CT_CFG.SYSCFG_bit.STANDBY_INIT = 0;

/*

	for(i = 1000000; i > 0; i = (i * decay) / 100) {
		
		#ifndef LED_DISABLE
				*GPIO1_SET = USR0;
		#endif
		#ifndef GPIO_DISABLE
				__R30 |= PRU0_GPIO;
		#endif
		for(j=0;j<i;j++){
			__delay_cycles( DELAY_CYCLES); 
		}
		#ifndef LED_DISABLE
				*GPIO1_CLEAR = USR0;
		#endif
		#ifndef GPIO_DISABLE
				__R30 ^= PRU0_GPIO;
		#endif
		for(j=0;j<i;j++){
			__delay_cycles(DELAY_CYCLES);
		}
	}
	*/
	
/*

	unsigned int loops, delay;
    for (loops = 0; loops < 10; loops++) {
        __R30 |= PRU0_GPIO; 
        for (delay = 0; delay < 20000000; delay++) { // loop delay
        }
        __R30 ^= PRU0_GPIO; // Turn off the LED
        __delay_cycles(60000000); // Intrinsic method delay
    }
*/
	__halt();
}
