/*
 * Copyright (C) 2016 Texas Instruments Incorporated - http://www.ti.com/
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
#include <stdio.h>
#include <pru_cfg.h>
#include <pru_intc.h>
#include <rsc_types.h>
#include <pru_rpmsg.h>
#include <pru_iep.h>
#include "resource_table_1.h"

volatile register uint32_t __R31;

/* Host-1 Interrupt sets bit 31 in register R31 */
#define HOST_INT			((uint32_t) 1 << 31)

/* The PRU-ICSS system events used for RPMsg are defined in the Linux device tree
 * PRU0 uses system event 16 (To ARM) and 17 (From ARM)
 * PRU1 uses system event 18 (To ARM) and 19 (From ARM)
 */
#define TO_ARM_HOST			18
#define FROM_ARM_HOST			19

/*
 * Using the name 'rpmsg-pru' will probe the rpmsg_pru driver found
 * at linux-x.y.z/drivers/rpmsg/rpmsg_pru.c
 */
#define CHAN_NAME			"rpmsg-pru"
#define CHAN_DESC			"Channel 31"
#define CHAN_PORT			31

/*
 * Used to make sure the Linux drivers are ready for RPMsg communication
 * Found at linux-x.y.z/include/uapi/linux/virtio_config.h
 */
#define VIRTIO_CONFIG_S_DRIVER_OK	4

uint8_t payload[RPMSG_BUF_SIZE];

//units are us
//HIGH is 1620
#define MIN_HIGH  1200
#define MAX_HIGH 2000
//LOW is 460
#define MIN_LOW  100
#define MAX_LOW 800
//START is 4500
#define MAX_START 5000
#define MIN_START 3500
// the maximum pulse we'll listen for 
#define MAXPULSE 10000
//Pins 8.39,41,43,45
const unsigned pinMasks[] = {1 << 6, 1 << 4, 1 << 2, 1 << 0};
#define SENSORS (4)
#define RESOLUTION 10
#define SAMPLE_PERIOD 2000     //10us / 5ns = 2000cycles

unsigned codes[SENSORS];
unsigned highTime[SENSORS];
unsigned lowTime[SENSORS];
unsigned pulseCount[SENSORS];
unsigned high;
unsigned active;
unsigned printFlag;


void pollReceivers(){
	unsigned i;
	unsigned mask;
    for(i = 0; i < SENSORS ; i++){
        mask = pinMasks[i];
        if(!(__R31 & mask)){ //inverted by transistor
            if(! (high & mask)){ //first high
                high |= mask;               
                highTime[i] = 0;
                //any logic on lowtime
            } else{  //all other high polls
                highTime[i] += RESOLUTION; 
                if(highTime[i] > MAXPULSE){
                    if(active & mask){
                       active &= ~mask;
                        //verify
                        //print - warning the timing of this will effect concurrent reads! 
                    	
                    	printFlag |= mask;
                    }
                }
            }
             
        } else {
            if(high & mask){//first low
                high &= ~mask;
                lowTime[i] = 0;
                  if(! (active & mask)){ //in range, not currently active
                    //check if in range for start signal -> active = true
                    if(highTime[i] >= MIN_START && highTime[i] <= MAX_START){
                        active |= mask;               
                    }
                } else{ //in range and actively listening
                    //if in range for high, shift in 1 to code
                    if(highTime[i] >= MIN_HIGH && highTime[i] <= MAX_HIGH){
                        codes[i] = (codes[i] << 1) + 1;
                    }
                    //if in range for low, shift in 0 to code
                    if(highTime[i] >= MIN_LOW && highTime[i] <= MAX_LOW){
                        codes[i] <<= 1;
                    }
                    //else, discard code as bad data
                  
                }           
            }//end first low
            else{//not first low
                lowTime[i]+=RESOLUTION;
             //   Serial.println(lowTime[pin]);
                 //if low for too long, end of code. verify code is valid and then print
                if((lowTime[i] > MAXPULSE || pulseCount[i] >= 16) && (active & mask)){
                    active &= ~mask;
                    //verify
                    //print - warning the timing of this will effect concurrent reads! 
                    printFlag |= mask;
                }  
            }
           
        }      
    }
}

void resetIEP(){
	//Reset and enable IEP timer
	// Set counter to 0
    CT_IEP.TMR_CNT = 0x0;
    // Enable counter
    CT_IEP.TMR_GLB_CFG = 0x11;
}

/*
 * main.c
 */
void main(void)
{
	struct pru_rpmsg_transport transport;
	uint16_t src, dst, len;
	volatile uint8_t *status;

	/* Allow OCP master port access by the PRU so the PRU can read external memories */
	CT_CFG.SYSCFG_bit.STANDBY_INIT = 0;

	/* Clear the status of the PRU-ICSS system event that the ARM will use to 'kick' us */
	CT_INTC.SICR_bit.STS_CLR_IDX = FROM_ARM_HOST;

	/* Make sure the Linux drivers are ready for RPMsg communication */
	status = &resourceTable.rpmsg_vdev.status;
	while (!(*status & VIRTIO_CONFIG_S_DRIVER_OK));

	/* Initialize the RPMsg transport structure */
	pru_rpmsg_init(&transport, &resourceTable.rpmsg_vring0, &resourceTable.rpmsg_vring1, TO_ARM_HOST, FROM_ARM_HOST);

	/* Create the RPMsg channel between the PRU and ARM user space using the transport structure. */
	while (pru_rpmsg_channel(RPMSG_NS_CREATE, &transport, CHAN_NAME, CHAN_DESC, CHAN_PORT) != PRU_RPMSG_SUCCESS);
	
	//wait for start signal from ARM
	waitForMessage:
	while(!(__R31 & HOST_INT));	
	/* Clear the event status */
	CT_INTC.SICR_bit.STS_CLR_IDX = FROM_ARM_HOST;
	while (pru_rpmsg_receive(&transport, &src, &dst, payload, &len) != PRU_RPMSG_SUCCESS) {
		//wait for start signal reception to work
	}
	
	if(len < 2 || payload[0] != 'g' || payload[1] != 'o'){
		//did not get the start message
		goto waitForMessage;
	} 
	
    
	unsigned i;
	unsigned x;
	unsigned extracted;
	
	//Initialize variables
	for(i =0; i < SENSORS; i++){
		codes[i] = 0;
	}
	high = 0;
	active = 0;
	printFlag = 0;
	unsigned noPrint = 0;
	
	
	while (1) {
		resetIEP();
		pollReceivers();
		
		//verify and Print if we need to 
		if(printFlag){
			noPrint  = 0;
			uint8_t i;
			unsigned mask;
			for(i = 0; i < SENSORS ; i++){
    			mask = pinMasks[i];
        		if(printFlag & mask){
        			//makes sure the signal is valid by checking bits 1 - 6 match 9 - 14
        			extracted = codes[i] & 0x1F;
        			if(extracted != (codes[i] & 0x1F00) >> 8){
        				continue;
        			}
        
		        	payload[0] = '0' + i;
					payload[1] = ':';
					payload[2] = extracted & 16 ? '1' : '0';
					payload[3] = extracted & 8 ? '1' : '0';
					payload[4] = extracted & 4 ? '1' : '0';
					payload[5] = extracted & 2 ? '1' : '0';
					payload[6] = extracted & 1 ? '1' : '0';
					payload[7] = '\n';
					payload[8] = 0;
					len = 8;
					/* prints full message
					for(bit = 1 << 15, x = 2; x < 18; x++ ){ //2 to 18 = 16 bits
						payload[x] = (codes[i] & bit) ? '1' : '0';
						bit >>= 1;
					}
					*/
					
					pru_rpmsg_send(&transport, dst, src, payload, len);
        	
        		}		
        	}
        	//clear print flags
			printFlag = 0;
        }
    //     else{  //print "no" if seen nothing in 10 seconds to confirm stil running
    //     	noPrint += RESOLUTION;
    //     	if(noPrint > 10000000){
    //     		payload[0] = 'n' ;
				// payload[1] = 'o';
				// payload[2] = '\n';
				// payload[3] = 0;
				// len = 4;
				// pru_rpmsg_send(&transport, dst, src, payload, len);
    //     		noPrint = 0;
    //     	}
        	
    //     }
    
    //Check for message
    	if(__R31 & HOST_INT){
    		/* Clear the event status */
			CT_INTC.SICR_bit.STS_CLR_IDX = FROM_ARM_HOST;
			if (pru_rpmsg_receive(&transport, &src, &dst, payload, &len) == PRU_RPMSG_SUCCESS) {
				if(len >= 4 && payload[0] == 's' && payload[1] == 't' && payload[2] == 'o' && payload[3] == 'p'){
					//got stop command
					goto waitForMessage;
				}
			} else{
			  CT_INTC.SICR_bit.STS_CLR_IDX = FROM_ARM_HOST;
			}
			
		}
    
    	
		while( CT_IEP.TMR_CNT < SAMPLE_PERIOD);
	
	}
}
