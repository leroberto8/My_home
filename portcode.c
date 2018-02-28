/***********************************************************************************

				PIC32MX Starter Kit Example Code - PORT Input Output

 ***********************************************************************************
 * FileName:        port_io.c
 * Dependencies:    None
 * Company:         Microchip Technology, Inc.
 *
 * Copyright (c) 2008 Microchip Technology, Inc.
 * Software License Agreement
 *
 * The software supplied herewith by Microchip Technology Incorporated
 * (the “Company”) for its PIC32 Microcontroller is intended
 * and supplied to you, the Company’s customer, for use solely and
 * exclusively on Microchip PIC32 Microcontroller products.
 * The software is owned by the Company and/or its supplier, and is
 * protected under applicable copyright laws. All rights are reserved.
 * Any use in violation of the foregoing restrictions may subject the
 * user to criminal sanctions under applicable laws, as well as to
 * civil liability for the breach of the terms and conditions of this
 * license.
 *
 * THIS SOFTWARE IS PROVIDED IN AN “AS IS” CONDITION. NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 * TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 * IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 * CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 ***************************************************************************
 * Description:
 *			The example code demonstrates simple switch input methods and
 *			methods for configuring pins as outputs.
 *
 *
 *
 *************************************************************************************/

// Adds support for PIC32 Peripheral library functions and macros
#include <p32xxxx.h>
#include <plib.h>
#ifdef __XC32

#endif
#include <plib.h>           /* Include to use PIC32 peripheral libraries      */
#include <stdint.h>         /* For uint32_t definition                        */
#include <stdbool.h>        /* For true/false definition                      */


// Configuration Bits


//  The following is used by the main application
#define SYS_FREQ		(80000000L)
#define CONFIG          (CN_ON)
#define PINS            (CN16_ENABLE)
#define PULLUPS         (CN15_PULLUP_ENABLE | CN16_PULLUP_ENABLE)
#define INTERRUPT       (CHANGE_INT_ON | CHANGE_INT_PRI_2)

                  // IOPORT bit masks can be found in ports.h


unsigned int dummy;

//  port_io application code
int PORT_init(void)
{
     SYSTEMConfigPerformance(80000000);
    AD1PCFG = 0xFFFF;

    DDPCONbits.JTAGEN = 0;
                                   // ce block par moi
    TRISA = 0xff00;

    while (1)
    {
       PORTAINV = 0x34;
    }
    unsigned int last_sw_state = 1;

    // Configure the device for maximum performance, but do not change the PBDIV clock divisor.
	// Given the options, this function will change the program Flash wait states,
	// RAM wait state and enable prefetch cache, but will not change the PBDIV.
    // The PBDIV value is already set via the pragma FPBDIV option above.


     /* TODO Add user clock/system configuration code if appropriate.  */
     SYSTEMConfig(SYS_FREQ, SYS_CFG_ALL);
    /* Initialize I/O and Peripherals for application */
     TRISE = 0x00;
         while(1)

    {
        LATAINV = 0x0000001F;    // Toggle bit 4 every statement
        LATAINV = 0x0000003F;    // Toggle bit 5 second statement
        LATAINV = 0x0000005F;    // Toggle bit 6 and 4
        LATAINV = 0x0000003F;
        LATAINV = 0x0000009F;    // Toggle bit 7 and 4
        LATAINV = 0x0000003F;    //
        LATAINV = 0x0000005F;    // Toggle bit 6 and 4
        LATAINV = 0x0000003F;
    }
   	SYSTEMConfig(SYS_FREQ, SYS_CFG_WAIT_STATES | SYS_CFG_PCACHE);

	// configure IOPORTS PORTD.RD0, RD1 as outputs
	// could also use mPORTDSetPinsDigitalOut(BIT_6 | BIT_7);
    PORTSetPinsDigitalOut(IOPORT_D, BIT_0 | BIT_1);

	// initialize the port pins states = output low
    PORTClearBits(IOPORT_D, BIT_0 | BIT_1);

	// PORTD.RD6, RD7 as inputs
	// could also use mPORTDSetPinsDigitalIn(BIT_6 | BIT_7);
    PORTSetPinsDigitalIn(IOPORT_D, BIT_6 | BIT_7);

	// configure the Change Notice Feature
	// Note: It is recommended to disable vectored interrupts prior to
    // configuring the change notice module, (if they are enabled).
    // The user must read one or more IOPORTs to clear any IO pin
    // change notice mismatch condition, then clear the change notice
    // interrupt flag before re-enabling the vector interrupts.

    // Enable change notice, enable discrete pins and weak pullups
    mCNOpen(CONFIG, PINS, PULLUPS);

    // Read the port to clear any mismatch on change notice pins
    dummy = mPORTDRead();

    // Clear change notice interrupt flag
    ConfigIntCN(INTERRUPT);

    // Ok now to enable multi-vector interrupts
    INTEnableSystemMultiVectoredInt();

	//Initialize the DB_UTILS IO channel
	DBINIT();

	// Display a message
	DBPRINTF("Welcome to the PIC32 PORT input/output example. \n");
	DBPRINTF("The build date and time is ... (" __DATE__ "," __TIME__ ")\n");
	DBPRINTF("Press SW1 to toggle LED1, press SW2 to toggle LED2 \n");


   // loop here polling for SW1, SW2 is handled by Change Notice Interrupt
   while(1)
   {
                        mPORTDSetBits(BIT_1);
                        mPORTDClearBits(BIT_1);
                        mPORTDSetBits(BIT_1);
                        mPORTDClearBits(BIT_1);
                        mPORTDSetBits(BIT_1);
                        mPORTDClearBits(BIT_1);
 	  if(PORTDbits.RD6 == 0)					// 0 = switch is pressed
 	  {
 	  	PORTSetBits(IOPORT_D, BIT_0);			// RED LED = on (same as LATDSET = 0x0001)
 	  	if(last_sw_state == 1)					// display a message only when switch changes state
		{
 	  	    DBPRINTF("Switch SW1 has been pressed. \n");
 	  	    last_sw_state = 0;
 	  	}
 	  }
 	  else										// 1 = switch is not pressed
 	  {
 	  	PORTClearBits(IOPORT_D, BIT_0);			// RED LED = off (same as LATDCLR = 0x0001)
 	  	if(last_sw_state == 0)                 // display a message only when switch changes state
 	  	{
 	  	    DBPRINTF("Switch SW1 has been released. \n");
 	  	    last_sw_state = 1;
        }
 	  }
   };
   //void __ISR(_CHANGE_NOTICE_VECTOR, ipl2) ChangeNotice_Handler(void)


    // Step #1 - always clear the mismatch condition first
    dummy = PORTReadBits(IOPORT_D, BIT_7);

    // Step #2 - then clear the interrupt flag
    mCNClearIntFlag();

    // Step #3 - process the switches
    if(dummy == BIT_7)
    {
        PORTClearBits(IOPORT_D, BIT_1);       // turn off LED2
        DBPRINTF("Switch SW2 has been released. \n");
    }
    else
    {
        PORTSetBits(IOPORT_D, BIT_1);     // turn on LED2
        DBPRINTF("Switch SW2 has been pressed. \n");
    }

    // additional processing here...

PORTA = 0x0000;
PORTB = 0x0000;
PORTC = 0x0000;
PORTD = 0x0000;
PORTE = 0x0000; /*Set analog input/digital input/output modes*/

TRISC = 0b11111011;
TRISD = 0b00001111;
TRISE = 0b11111100;
PR2 = 0x3F;
T2CON = 0b00000100;

printf("EEPROM Ready");
TRISACLR = 0x00FF;
    while(1)
    {    register unsigned int Shift = 0xAAAAAAAA;    // 32 bits alternating
        LATA = Shift & 0x00FF;
        Shift = Shift >> 1;        // 4 instructions per IO toggle, -O0
        LATA = Shift & 0x00FF;
        Shift = Shift >> 1;
        LATA = Shift & 0x00FF;
        Shift = Shift >> 1;
        LATA = Shift & 0x00FF;
        Shift = Shift >> 1;
        LATA = Shift & 0x00FF;
        Shift = Shift >> 1;
        LATA = Shift & 0x00FF;
        Shift = Shift >> 1;
        LATA = Shift & 0x00FF;
        Shift = Shift >> 1;
        LATA = Shift & 0x00FF;
        Shift = Shift >> 1;
        LATA = Shift;            // 3 instructions per IO toggle, -O0
        Shift = Shift >> 1;
        LATA = Shift;            // 34 instructions in loop with -O1
        Shift = Shift >> 1;        //        after setup
        LATA = Shift;
        Shift = Shift >> 1;
        LATA = Shift;
        Shift = Shift >> 1;
        LATA = Shift;
        Shift = Shift >> 1;
        LATA = Shift;
        Shift = Shift >> 1;
        LATA = Shift;
        Shift = Shift >> 1;
        LATA = Shift;
        Shift = Shift >> 1;
        LATA = Shift & 0x00FF;
        Shift = Shift >> 1;
        LATA = Shift & 0x00FF;
        Shift = Shift >> 1;
        LATA = Shift & 0x00FF;
        Shift = Shift >> 1;
        LATA = Shift & 0x00FF;
        Shift = Shift >> 1;
        LATA = Shift & 0x00FF;
        Shift = Shift >> 1;
        LATA = Shift & 0x00FF;
        Shift = Shift >> 1;
        LATA = Shift & 0x00FF;
        Shift = Shift >> 1;
        LATA = Shift & 0x00FF;
        Shift = Shift >> 1;
        LATA = Shift & 0x00FF;
        Shift = Shift >> 1;
        LATA = Shift & 0x00FF;
        Shift = Shift >> 1;
        LATA = Shift & 0x00FF;
        Shift = Shift >> 1;
        LATA = Shift & 0x00FF;
        Shift = Shift >> 1;
        LATA = Shift & 0x00FF;
        Shift = Shift >> 1;
        LATA = Shift & 0x00FF;
        Shift = Shift >> 1;
        LATA = Shift & 0x00FF;
        Shift = Shift >> 1;
        LATA = Shift & 0x00FF;
        Shift = Shift >> 1;
    };
    return(1);
 }




