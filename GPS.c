/*
 * File:   GPS.c
 * Author: Enrique Daniel Saldívar Carranza
 *
 * Created on 18 de septiembre de 2018, 08:51 PM
 * Programa que de toda una lista de datos GPS escribe en serial
 * únicamente los datos GPGGA (toda la lista de datos)
 */


#include <xc.h> 
#include "pic18f25k80.h" 
#pragma config XINST = OFF      
#pragma config MCLRE = OFF     // Master Clear Enable (MCLR Disabled, RE3 Enabled) 
#pragma config SOSCSEL = DIG   // SOSC Power Selection and mode Configuration bits (Digital (SCLKI) mode)

unsigned char mens[70] = {}; 
unsigned char i = 0, mess = 0, flag = 0, cont = 0;

void interrupt Interrupciones ()
{
    if(PIR1bits.RC1IF && PIE1bits.RC1IE) 
    {
        mess = RCREG1;
        if (mess == '$'){
            flag = 1;
        }
        if (flag){
          mens[i++] = mess;  
          if((i == 5) && (mens[4] != 'G')){
            flag = 0;
            i = 0;
          }
          if(mens[i-1] == '*'){
            TX1IE = 1;
            TXREG1 = mens[cont++];
            mens[i] = 0;
            i = 0;
            flag = 0;
          }
        } 
    }
    if(TX1IF && TX1IE){ 
        TX1IF = 0;
        TXREG1 = mens[cont++];
        if (mens[cont] == 0){
            TX1IE = 0;
            cont = 0;
        }
    } 
}

void GPIO_ini()
{
    TRISC6 = 0; //tx1 is an output
    TRISC7 = 1; //rx1 is an input
}

void UART_ini()
{
    TXSTA1 = 0b00100000;        //dont care, 8 bit transmission, transmit enable,Asynchronous,Sync Break transmission is completed,low speed,TSR is full 
    RCSTA1 = 0b10010000;        //serial port enable,8bit reception,dont care,enables receiver,Disables address detection, no framing error, no overrun error
    BAUDCON1bits.BRG16 = 0;     // 8-bit Baud Rate Generator ? SPBRGx only (Compatible mode), SPBRGHx value is ignored
    SPBRG1 = 12;                //baud rate of 9600 bps
    TX1IE = 0;                  //tx1 interrupt disabled 
    TX1IP = 1;                  //tx1 interrupt high priority
    TX1IF = 0;                  //flag is off
    RC1IE = 1;                  //turn on reception interrupt
    RC1IP = 1;                  //reception is high priority 
    RC1IF = 0;                  //flag is off
}

void main(void) {
    IPEN = 1;   //high piorities in interrupts
    GIEH = 1;   //global enable on
    
    GPIO_ini(); //initialization of ports
    UART_ini(); //initialization of data transmission

    while(1) 
    { 
        ; 
    } 
}