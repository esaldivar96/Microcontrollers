/*
 * File:   stepMotor.c
 * Author: Enrique
 *
 * Created on 24 de septiembre de 2018, 02:23 PM
 */

#include <xc.h>
#include "PIC18F25k80.h"

#pragma config XINST=OFF
#pragma config MCLRE = OFF      // Master Clear Enable (MCLR Disabled, RG5 Enabled)
#pragma config SOSCSEL = DIG    // SOSC Power Selection and mode Configuration bits (Digital (SCLKI) mode)

#define period_overflowTMR 131  //we start with this value so the overflow of the timer takes 0.25 ms each time

unsigned char secuencia[4] = {0x01,0x02,0x04,0x08};
unsigned int i = 0; 
unsigned char cont = 0;

void interrupt ISR_alta (void)
{   
    if (TMR0IF && TMR0IE)
    {
        TMR0IF = 0;
        TMR0L = period_overflowTMR;
        ++i;
        if (i == 500)                        
        {
            PORTC  = secuencia[(cont++)%4];
            i = 0;
        }
    }  
}

void TIMER_ini()
{
    T0CON = 0b11000001;         //TMR 0 enable, con prescaler 1:4 y base tiempo interna... 2 us
    TMR0L = period_overflowTMR; //empieza en 131 para contar 2us 125 veces (0.25ms)
    TMR0IE = 1;                 //Habilitador local TMR0
    TMR0IP = 1;                 //Alta prioridad
}

void GPIO_ini()
{
    TRISC = 0xF0;               //output C0,C1,C2,C3
}

void main(void)
{
    IPEN = 1;                   //Habilitar niveles de prioridad
    GIEH = 1;                   //Habilitador global interrupciones de alta prioridad
    
    GPIO_ini();                 //initialization of ports
    TIMER_ini();                //initialization of data transmission
    
    while (1)
    {
        ;
    }
}
