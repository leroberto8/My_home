#include <plib.h>
#include "p32mx795f512l.h"
#include "bldc.h"
//#if defined (__32MX360F512L__) || (__32MX460F512L__) || (__32MX795F512L__) || (__32MX430F064L__) || (__32MX450F256L__) || (__32MX470F512L__)


#define HALstate DCH0CONbits.CHPRI
// definicja pinów i Timer steruj?cy

#define T1 PORTCbits.RC1
#define T3 PORTCbits.RC2
#define T5 PORTCbits.RC3
#define T4 PORTCbits.RC4
#define T6 PORTCbits.RC12
#define T2 PORTCbits.RC13
/* deklaracje funkcji*/
 void Halla_Init(void);
void HES_Int(void);

void Halla_Init(void)
{
    TMR2=0; // reset Timer

switch(IC1CON) // read Hall sensor
{
case 1: T1 = 0; // phase 6: 001
        T2 = 0;
        T3 = 0;
        T4 = 0;
        T5 = HALstate;
        T6 = HALstate;
   break;
case 2: T1 = 0; // phase 4: 010
        T2 = HALstate;
        T3 = HALstate;
        T4 = 0;
        T5 = 0;
        T6 = 0;
   break;
case 3: T1 = 0; // phase 5: 011
        T2 = 0;
        T3 = 0;
        T4 = HALstate;
        T5 = HALstate;
        T6 = 0;
   break;
case 4: T1 = HALstate; // phase 2: 100
        T2 = HALstate;
        T3 = 0;
        T4 = 0;
        T5 = 0;
        T6 = 0;
   break;
case 5: T1 = HALstate; // phase 1: 101
        T2 = 0;
        T3 = 0;
        T4 = 0;
        T5 = 0;
        T6 = HALstate;
   break;
 case 6: T1 = 0; // phase 3: 110
         T2 = HALstate;
         T3 = HALstate;
         T4 = 0;
         T5 = 0;
         T6 = 0;
    break;
  default: break; // invalid
  }
/* TU RESRT FLAGI; wlaCZENIA WSZYSTKI PORTC ; WlaCZAMY GLOBALNY PRZEWANIA*/
IFS0bits.OC1IF=1;
IFS2bits.PMPEIF=1;
TRISC=0x0000;
}
void HEs_Init(void)
{

TRISC=20;
TRISD=50;
T1CONbits.TCKPS0=1;// timer runs at x MHz; czas wyliczania Timerów (ils sont au nombre de 3)
T1CONbits.TCKPS1=1;
OC2CONbits.OCM0 = 0;
OC2CONbits.OCM2 = 0;
OC1CONbits.OCM0 = 0;
OC1CONbits.OCM2 = 0;                             //wylaczania timer dotyczacy capture
TMR2=0;//reset Timer
TMR1=1; //start Timer


}