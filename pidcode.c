#include <plib.h>
#include "p32mx795f512l.h"

DWORD ADCValue;
DWORD pv;// wyjsc regulator predkosci
DWORD Tick_old;
DWORD mv;//wyjsc regulator pradu
DWORD Tick_cur;// wartasc pradu chwili k
DWORD Tick_old;//wartasc pradu chwili k-1
DWORD Tick_new;//wartasc pradu chwili k+1
DWORD Period;
 double actualcurrent=0;// wartosc prad mierzona
 double actualspeed=0; // wartosc predkosc mierzona
 double desirecurrent=0;// wartosc prad zadana
 double desirespeed=0;// wartosc predkosc zadana
 double KRw;
 double i;
 double k;
 double  e[3];
void PID_Init (void)
{
typedef volatile struct _PIDstr {

double KRw; /**< wartosc wmocnienie regulator P */

double i; /**< wartosc calkujacy regulator PI */

double k;/*<wartosc wmocnienie regulator  PI*/

DWORD sp; /**< Setpoint in mechanical RPM */

DWORD pv; /**< Process value, dt in sys counts, used for RPM calc */

double e[3]; /**< Error in PI(D) calculations */

DWORD mv; /**< Manipulated value, output of the PI controller */

BYTE HALstate; /**< Current HAL state */

DWORD CMT_CNT; /**< Commutation counter, motor commutated check*/

DWORD CMT_step; /**< Current commutation step */

DWORD RPM; /**< Current motor mechanical RPM */

BYTE Enable; /**< Motor enable, 0 = DISABLE, 1 = ENABLE */

BYTE Direction; /**< Motor rotation direction, Clockwise = CW or Counter
Clockwise = CCW */

BYTE Brake; /**< Motor electrical break, 0 = DISABLE, 1 = ENABLE */

DWORD Period; /**< Switching frequency of the phase drives */

BYTE Poles; /**< Number of Pole-pairs in the motor */
DWORD Tick_cur; /**< Current tick value, for RPM calculations */

DWORD Tick_old; /**< Current tick value, for RPM calculations */

DWORD Tick_new; /**< Current tick value, for RPM calculations */

} PIDstr;

    while((e[0]==0)||(e[1]==0)||(e[2]==0)){


    if(e[0]==0){
        if(actualspeed<desirespeed){
           desirespeed--;
            e[0]= desirespeed-actualspeed;
            pv=KRw*e[0];
        }
        else if(actualspeed>desirespeed){
            desirespeed++;
             e[0]= desirespeed-actualspeed;
            pv=KRw*e[0];
        }

    }
    else if(e[0]=0){
         if(actualcurrent<desirecurrent){
             desirecurrent--;
            Tick_old=desirecurrent;
          Tick_cur=actualcurrent;
         e[1]= desirecurrent-actualcurrent;
         e[2]= e[1]-e[2];
         Tick_new=k*e[1]+Tick_old+Period*e[2]/i;
         mv=k*e[1]+i*e[2];

         }
          if(actualcurrent>desirecurrent){
             desirecurrent++;
            Tick_old=desirecurrent;
          Tick_cur=actualcurrent;
         e[1]= desirecurrent-actualcurrent;
         e[2]= e[1]-e[2];
         Tick_new=k*e[1]+Tick_old+Period*e[2]/i;
         mv=k*e[1]+i*e[2];
          }
    }
        ADCValue=mv;

      }

}
