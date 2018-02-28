#include <stdio.h>
#include <stdlib.h>
#include "p32xxxx.h"
#include "plib.h"
#define SYSTEM_FREQ_HZ 80000000


const unsigned short pwm_duty_cycles[] = {2500,5000,7500,10000,12500,15000,17500,20000, \
					22500,25000,22500,20000,17500,15000,12500,10000,7500,5000,2500,0};

#define ARRAY_SIZE			            (sizeof(pwm_duty_cycles))
#define _XTAL_FREQ 20000000
#define TMR2PRESCALE 4

#define FORWARD					0
#define REVERSE					1

#define ENABLE_PIN			LATCbits.LATC1
#define DIRECTION_PIN		       LATCbits.LATC2

#define MAX_DUTY                        3999

unsigned int Pwm; // variable to store calculated PWM value
unsigned int Mode; // variable to determine ramp up pwm or ramp down pwm



// configure oscillator
  void OSC_Init(void){
	OSCCON = 0b01111100;

	T1CONbits.TCS = 0; // timer input is Fosc/4
	T1CONbits.ON = 1; // bypasses prescaler

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

}
void PWM_Init(void)
{





	// Allow vector interrupts

        IFS0bits.T2IF=0;
        IEC0bits.T2IE=0;
        TMR2= 0;       // clear count

	//Set Enable and Direction Pins (A2, A3) as digital outputs
	// Initialize as low
	LATC &= 0xFFF; TRISA &= 0xFFF3;

	DIRECTION_PIN = FORWARD;
	Pwm = 0;
	Mode = 1;

	// init OC1 module
	OpenOC1( OC_ON | OC_TIMER2_SRC | OC_PWM_FAULT_PIN_DISABLE, 0, 0);


	// init Timer2 mode and period (PR2) 20 kHz freq ( 1 / 20 kHz = (3999 + 1) / 80MHz * 1

          PR2=128;

 	         TMR2=1;         // set Timer2 Interrupt Priority
	 	IFS0bits.T2IF=0; // clear interrupt flag
		IEC0bits.T2IE=1; // enable timer2 interrupts



	ENABLE_PIN = 1; // Enable the H-bridge

	while(1)
	{


	}

	CloseOC1();

} 

void T2__ISR( void)

{
	if ( Mode )
	{
		if ( Pwm <= MAX_DUTY ) // ramp up pwm

		{
			//mLED_1_On();
			//mLED_2_Off();
                             OC1R = 1;
                            OC1CONbits.SIDL=1; // OCFB=1;
                        OC2CONbits.OCM1 = 1;

                       OC2CONbits.OCM2 = 1;
                        TMR2 = 1;
                       TRISCbits.TRISC2 = 0;

			Pwm ++; // If the duty cycle is not at max, increase

			if (DIRECTION_PIN == FORWARD)
			{
				SetDCOC1PWM(Pwm); // Write new duty cycle
			}
			else
			{
				 //mLED_0_On();
                                 OC2CONbits.OCM0 = 1;
				//SetDCOC1PWM(Pwm);
				SetDCOC1PWM(MAX_DUTY - Pwm);
			}
		}
		else
		{
			Mode = 0; // PWM is at max, change mode to ramp down
		}
	} // end of ramp up pwm
	else

{
		if ( Pwm > 0 ) // ramp down pwm
		{
			//mLED_2_On();
			//mLED_1_Off();
                         OC1R = 0;
                         OC1CONbits.SIDL=0 ; //OCFB?

                       OC2CONbits.OCM1 = 0;
                       OC2CONbits.OCM2 = 0;
                        TMR2 = 0;
                       TRISCbits.TRISC2 = 0;
			Pwm --;         // If the duty cycle is not at min, increase

			if (DIRECTION_PIN == FORWARD)
			{
				SetDCOC1PWM(Pwm); // Write new duty cycle
			}
			else
			{
				//mLED_0_Off();
                               OC2CONbits.OCM0 = 0;

				//SetDCOC1PWM(Pwm);
				SetDCOC1PWM(MAX_DUTY - Pwm);
			}
		}
		else
		{
			Mode = 1; // PWM is at min, change mode to ramp up
			DIRECTION_PIN = !DIRECTION_PIN;
		}
	} // end of ramp down pwm

	// clear interrupt flag and exit
	
       IFS0bits.T2IF=0;
} // T2 Interrupt




/*/SetDCOC1PWM(unsigned int pwm)
{
  if(pwm<3999)
  {
     pwm = ((float)pwm/3998)*Max_Duty;
    PORTCbits.RC3 = pwm & 2;
    PORTCbits.RC2 = pwm & 1;
    PORTCbits.RC13 = pwm >>2;
}*/