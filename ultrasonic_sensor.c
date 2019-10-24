/*
 * File:   ultrasonic_sensor.c
 * Author: Enrique Daniel Saldívar Carranza
 * Created on 11 de septiembre de 2019, 10:47 PM
 * Microcontroller running at 8 MHz
 */

// #pragma config statements should precede project file includes.
#pragma config XINST   = OFF            // Extended instruction set disabled
#pragma config MCLRE   = OFF            // Master Clear Enable (MCLR Disabled, RG5 Enabled)
#pragma config SOSCSEL = DIG            // SOSC Power Selection and mode Configuration bits (Digital (SCLKI) mode)

#include <xc.h>
#include "pic18f25k80.h"

const unsigned int TX_PERIOD = 1000;     // Time to send a new TX message | ms
const unsigned int CONVERT_ASCII = 0x30; //factor used to convert to ASCII

unsigned int cont_tx = 0;
unsigned char message[5] = {0x00,0x00,0x00,0x0A,0x0D}, msg_idx = 0;

void GPIO_ini();
void TIME_ini();
void UART_ini();

void main(void) {
    IPEN = 1;   //high piorities in interrupts
    GIEH = 1;   //global enable on
    
    GPIO_ini();
    TIME_ini();
    UART_ini();
    
    unsigned char has_been_high = 0;
    unsigned char mask = 0;
    unsigned int distance = 0;
    
    while(1){
        
        if (RC2 == 1 && has_been_high == 0) {
            TMR1ON = 1;
            has_been_high = 1;
        }
        else if (RC2 == 0 && has_been_high == 1) {
            distance = 123;
            TMR1ON = 0;
            has_been_high = 0;
            
            //makes data suitable for serial transition and interpretation
            /*for(int i = 2; i >= 0; i--) {
                mask = (distance%10) + CONVERT_ASCII;
                message[i] = i + CONVERT_ASCII;
                distance = distance/10;
            }*/
            message[2] = '1';
            message[1] = '2';
            message[0] = '3';
            
            
            msg_idx = 0;
            PIE1bits.TX1IE = 1;
            TXREG1 = message[msg_idx++];
            
            TMR1 = 0;
        }
        
    }
}

void interrupt Interruptions ()
{   
    // Interruption triggered by TIMER0
    if(TMR0IF == 1 && TMR0IE == 1) {
        TMR0L  = 6;
        TMR0IF = 0;
        cont_tx++;
        
        if(cont_tx == TX_PERIOD) {
            cont_tx = 0;
            TMR2ON = 1;
            RC1 = 1;
        }
    }
    
    if(TMR2IF == 1 && TMR2IE == 1) {
        RC1 = 0;
        TMR2IF = 0;
        TMR2ON = 0;
        TMR2  = 251;
    }
    
    if(PIR1bits.TX1IF == 1 && PIE1bits.TX1IE == 1) {
        PIR1bits.TX1IF = 0;
        
        TXREG1 = message[msg_idx++];
        if(msg_idx == 5) {
            PIE1bits.TX1IE = 0;
            RC0 = ~RC0; //message sent indicator
        }
        
    }
}

void GPIO_ini()
{
    TRISC0 = 0;    //RC0 output 
    TRISC1 = 0;    //RC1 output
    TRISC2 = 1;    //RC2 input
    TRISC6 = 0;    //TX1 output
    RC0 = 0;       //RC0 = 0, means GND will be on the pin
    RC1 = 0;
}

void TIME_ini()
{
    // Timer to trigger TX messages
    T0CON = 0b01000010;    //Disables Timer, 8 bit timer, internal timer FOSC/4, X, we use prescaler, 1/8 - (every count is 4us)
    TMR0L = 6;             //since we are working with 8 bits, we will have 250 counts (1 ms) to overflow
    TMR0IE = 1;            //Local enable, Enables the TMR0 overflow interrupt
    TMR0IP = 1;            //High priority
    TMR0IF = 0;            //Turns off TMR0 flag
    TMR0ON = 1;            //Starts Timer0
    
    T1CON = 0b00101110;    //FOSC/4, prescaler 1/4, SOSC enabled, ignored, 16 bit-operation, disables timer | 2 us per count
    TMR1GE=0;
    
    T2CON = 0b00000001;    //0, no postscale, timer off, prescaler 4 | 2 us per count
    TMR2 = 251;            // 5 counts (10 us) to overflow
    TMR2IE = 1;
    TMR2IP = 1;
    TMR2IF = 0;
}

void UART_ini()
{
    TXSTA1 = 0b00100000;        //dont care (Asynchronous), 8 bit transmission, transmit enable, Asynchronous, Sync Break transmission is completed, low speed, TSR is full 
    RCSTA1bits.SPEN = 1;        //serial port is enable 
    BAUDCON1bits.BRG16 = 0;     //8-bit Baud Rate Generator ? SPBRGx only (Compatible mode), SPBRGHx value is ignored
    SPBRG1 = 12;                //baud rate of 9600 bps, FOSC/[64 (n + 1)]
    
    PIE1bits.TX1IE = 0;         //tx1 interrupt disabled 
    TX1IP = 1;                  //tx1 interrupt high priority
    PIR1bits.TX1IF = 0;         //flag is off
}



