

#include <plib.h>
#include "p32mx795f512l.h"
char f_10ms= 0;

void T1__ISRC(void) {
    // Clear the interrupt flag

     IFS0bits.T1IF=0;
     T1CONbits.TGATE=0;
     T1CONbits.ON=0;

    // Update the period

   // set PWM period
	IEC0 = 128;
	IEC0 = 0;


    // Toggle Explorer-16 LEDs

     f_10ms = 1;
}

void T1_Init(void){

T1CONbits.TGATE = (unsigned int)&T1__ISRC; // copi tous les elements de la fonction T1_ISRC
T1CONbits.TCKPS =1;                    //channel source enabled
T1CONbits.TCS |=1;                    // channel is timer 1
IEC0bits.T1IE=0;                        // disable timer interrupt
TMR1=0;                                // clear count
IEC0bits.T1IE = 1;                    // enable interrupt
T1CONbits.ON = 1;                    // turn timer on

}
