/*********************************************************************
 *
 *      PIC32MX Oscillator API Example
 *
 *********************************************************************
 * FileName:        osc_basic.c
 *
 * Dependencies:    plib.h
 *
 * Processor:       PIC32
 *
 * Complier:        MPLAB C32
 *                  MPLAB IDE
 * Company:         Microchip Technology Inc.
 *
 * Software License Agreement
 *
 * The software supplied herewith by Microchip Technology Incorporated
 * (the ï¿½Companyï¿½) for its PIC32 Microcontroller is intended
 * and supplied to you, the Companyï¿½s customer, for use solely and
 * exclusively on Microchip PIC32 Microcontroller products.
 * The software is owned by the Company and/or its supplier, and is
 * protected under applicable copyright laws. All rights are reserved.
 * Any use in violation of the foregoing restrictions may subject the
 * user to criminal sanctions under applicable laws, as well as to
 * civil liability for the breach of the terms and conditions of this
 * license.
 *
 * THIS SOFTWARE IS PROVIDED IN AN ï¿½AS ISï¿½ CONDITION. NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 * TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 * IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 * CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 *******************************************************************
 * $Id: osc_basic.c 9390 2008-06-16 23:43:04Z rajbhartin $
 * *****************************************************************
 * The purpose of this example is to demonstrate the use of the
 * Oscillator API lib.
 *
 * Platform: Explorer-16 with PIC32MX PIM
 *
 * Features demonstrated:
 *     - Oscillator configuration and settings using the Oscillator API
 *     - Peripheral Pin Select (PPS)
 *
 * Description:
 *       This example changes the CPU clock source and Peripheral Bus
 *       dividers to reduce power consumption. For the newer PIMS
 *       (PIC32MX - 220F032D, 250F128D, 430F064L), the system clock is
 *       output on the reference clock pin using PPS, so the change in
 *       clock speed can be seen with an oscilloscope.
 *
 * Notes:
 *     - OSCConfig() forces cpu clock source to FRC(no divisor, no PLL),
 *       configures new clock source and then switches to the new
 *       clock source.
 *     - On the PIC32MX 220F032D and 250F128D PIMS, easy access to the
 *       reference clock pin is provided by the test points on top of
 *       the PIM (TP26 in this example)
 ********************************************************************/
#include <plib.h>
#include "p32mx795f512l.h"
//#if defined (__32MX360F512L__) || (__32MX460F512L__) || (__32MX795F512L__) || (__32MX430F064L__) || (__32MX450F256L__) || (__32MX470F512L__)
// Configuration Bit settings
// SYSCLK = 80 MHz (8MHz Crystal / FPLLIDIV * FPLLMUL / FPLLODIV)
// PBCLK = 80 MHz (SYSCLK / FPBDIV)
// Primary Osc w/PLL (XT+,HS+,EC+PLL)
// WDT OFF
// Other options are don't care
//#pragma config FPLLMUL = MUL_20, FPLLIDIV = DIV_2, FPLLODIV = DIV_1, FWDTEN = OFF
//#pragma config POSCMOD = HS, FNOSC = PRIPLL, FPBDIV = DIV_1
#define SYS_FREQ (80000000L)
#define GetSystemClock (25000)
#define SOURCE
#define CONFIG
#define DIVIDER
#define OSCREFConfig()
#if defined (__32MX220F032D__) || (__32MX250F128D__)
// Configuration Bit settings
// SYSCLK = 48 MHz (8MHz Crystal / FPLLIDIV * FPLLMUL / FPLLODIV)
// PBCLK = 48 MHz (SYSCLK / FPBDIV)
// Primary Osc w/PLL (XT+,HS+,EC+PLL)
// WDT OFF
// Other options are don't care
#pragma config FPLLMUL = MUL_24, FPLLIDIV = DIV_2, FPLLODIV = DIV_2, FWDTEN = OFF
#pragma config POSCMOD = HS, FNOSC = PRIPLL, FPBDIV = DIV_1
#define SYS_FREQ (48000000L)
#endif

// Reference clock pin enable, use SYSCLK as the source
//#if defined (__32MX220F032D__) || (__32MX250F128D__) || (__32MX430F064L__) || (__32MX450F256L__) || (__32MX470F512L__)
#define SOURCE OSC_REFOCON_SYSCLK
#define CONFIG (OSC_REFOCON_ON | OSC_REFOCON_OE)
#define DIVIDER 0
//#endif

int OSC_init(void)
{
    // Configure the device for maximum performance but do not change the PBDIV
    // Given the options, this function will change the flash wait states, RAM
    // wait state and enable prefetch cache but will not change the PBDIV.
    // The PBDIV value is already set via the pragma FPBDIV option above.
    SYSTEMConfig(SYS_FREQ, SYS_CFG_WAIT_STATES | SYS_CFG_PCACHE);

    int i;
    //unsigned int GetSystemClock;

SYSTEMConfig(GetSystemClock, SYS_CFG_WAIT_STATES | SYS_CFG_PCACHE);

    mPORTAClearBits(BIT_0);           //Clear bits to ensure the LED is off.
    mPORTASetPinsDigitalOut(BIT_0);   //Set port as output



    while(1)
    {
        i = 100000;
        mPORTAToggleBits(BIT_0);      //Toggle light status.
        while(i--) {}                 //Kill time.
    }
     OSCConfig(OSC_POSC, OSC_PLL_MULT_24, OSC_PLL_POST_256, 0);
      //OSCREFConfig(SOURCE, CONFIG, DIVIDER);
       //PPSOutput(3, RPC1, REFCLKO);
       // clock source.
        OSCConfig(OSC_POSC, OSC_PLL_MULT_24, OSC_PLL_POST_256, 0);
        // Configure the PB bus to run at 1/4 the CPU frequency

	mOSCSetPBDIV( OSC_PB_DIV_4 );

// configure oscillator
	OSCCON = 0b01111100;

	T1CONbits.TCS = 0; // timer input is Fosc/4
	T1CONbits.ON = 1; // bypasses prescaler*/

	IFS0bits.OC1IF = 1;

	// configure ports
	TRISB = 1;
	TRISDbits.TRISD1 = 0;

	// configure power PWM
	IFS2bits.PMPEIF = 0; // 1:1 prescaler, free running
	OC1CON = 0; // disable update to PWM cycle *************

	// set PWM period base
	IPTMR = 0; // Time base of 2
	OSCCON = 0;// clear and a WAIT instruction is executed.


	// set PWM period
	IEC0 = 128;
	IEC0 = 0;

	// set duty
  //const unsigned char value = 32;
	AD1PCFG = 32;
	AD1PCFG = 0;//enables the pin as an analog input pin( FOR eg:ANx)
	AD1PCFG = 50 ;
	AD1PCFG = 0;
	AD1PCFG = 128;
	AD1PCFG = 0;

	OC1CON = 1; // PWM [0:5] pins are enabled in complementry mode
        OC2CON=0;
        OC3CON=0;
        OC4CON=0;
        OC5CON=0;

//PWMCON1 = 0x01; // output overrides via the OVDCON register are synchronized to the PWM time base
        IEC0bits.T2IE = 1;//Enable Timer2 interrupt (PWM time base)
        IEC0bits.T2IE = 0;// Set Timer2 interrupt priority to 0
	I2C3RCV = 1; // configure sychronous serial data as output
	I2C4CON = 1; // configure sychronous serial data as intput
	I2C3TRN = 0; // Disable all faults
        I2C4BRG = 0;
	I2C5CON= 0;
        OC1R = 0; // Time base on, counting up (OUTPUT COMPARE)**********

     PORTDbits.RD2 = 1; // Set pin RD2  to 0
   // #if defined (__32MX220F032D__) || (__32MX250F128D__)
                                                      // Output SYSCLK on reference clock pin

   // #elif defined (__32MX430F064L__) || (__32MX450F256L__) || (__32MX470F512L__)

    //#elif defined (__32MX360F512L__) || (__32MX460F512L__) || (__32MX795F512L__)


        // clock source.
        OSCConfig(OSC_POSC, OSC_PLL_MULT_24, OSC_PLL_POST_256, 0);

        // Configure the PB bus to run at 1/4 the CPU frequency
	mOSCSetPBDIV( OSC_PB_DIV_4 );
//    #endif

    while(1);
}














