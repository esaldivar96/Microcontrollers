/*
 * File:   Lidar_I2C.c
 * Author: Enrique Daniel Saldivar Carranza-Orlando Lara Guzmán
 * Created on 30 de septiembre de 2018, 02:42 PM
 * https://openlabpro.com/guide/i2c-module-in-pic18f4550/   sample code for 18f PICs
 * https://www.csd.uoc.gr/~hy428/reading/i2c_spec.pdf   bus specifications used by Lidar
 * http://www.ti.com/lit/ds/symlink/bq76930.pdf I2C protocol routine taken from here (ignore Lidar's datasheet) Pg 32
 */

// #pragma config statements should precede project file includes.
#pragma config XINST = OFF            // Extended instruction set disabled
#pragma config MCLRE = OFF            // Master Clear Enable (MCLR Disabled, RG5 Enabled)
#pragma config SOSCSEL = DIG          // SOSC Power Selection and mode Configuration bits (Digital (SCLKI) mode)
#pragma config WDTEN = ON             // Watchdog Timer (WDT controlled by SWDTEN bit setting)
#pragma config WDTPS = 512            // Watchdog Postscaler (1:512) 2.048s

#include <xc.h>
#include "pic18f25k80.h"

//The stepper motor has a total of 96 steps per revolution
//if you want half stepping just change the values of the sequence array
#define degrees_per_step 375          //this is actually 3.75 degrees per step but we want to avoid floats [degrees*100]
#define visual_degrees 18000          //must be a multiple of 3.75 [degrees*100]
#define hidden_degrees 18000          //must be a multipole of 3.75 [degrees*100]
#define visual_time_per_step 100      //time it takes to complete every step of the visual range [miliseconds]
#define hidden_time_per_step 10       //time it takes to complete every stepp of the hidden range [milisecons]

#define slave_Address_Write 0xC4      //address of slave (Lidar) to write (effective)
#define slave_Address_Read 0xC5       //address of slave (Lidar) to read (effective)
#define ACQ_COMMAND 0x00              //Device Command address (used to indicate what to do)
#define STATUS_Lidar 0X01             //System status (to check if the Lidar is busy)
#define DIS_LB 0x10                   //Distance measured low byte
#define DIS_HB 0x0F                   //Distance measured high byte
#define conver_Ascii 0x30             //factor used to convert to ASCII

/*
 * This program communicates via I2C with the LIDAR-Lite v3 and gives the instruction
 * to take measurements. The distance measured (in cm.) is then 
 * sent via UART to be monitored. If SDA or SCL fail WD is called and the program
 * resets itself.
 * The step-motor that rotates with the Lidar is also controlled
*/

unsigned char rece_status = 0x01, mask = 0, y = 0, mens[10] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0A,0x0D}, i = 0;
unsigned char sequence[]={3,6,12,9};      //0b0011 0b0110 0b1100 0b1001
unsigned char i_mot = 0, deg_range = 0;   //sequence counter, counter to fix measurement range of Lidar
unsigned char visual_degree_counter = 0;
unsigned char hidden_degree_counter = 0;
unsigned char rotation_section = 0;       //this variable defines whether the motor is in the visible range (0) or the hidden range (1)
unsigned char steps = 0;
unsigned char cont_angle = 0;
unsigned char cont_angle_aux = 0;
unsigned int distance = 0, cont_T2 = 0;

void i2c_idle()
{
    RC0 = 0;
    // Wait for Status Idle (i.e. ACKEN, RCEN, PEN, RSEN, SEN) and I2C Bus stop detection
    while (( SSPCON2 & 0x1F ) || ( SSPSTAT & 0x04 ));
    RC0 = 1;
}

void mssp_wait()
{
    RC0 = 0;
    //wait till SSPIF flag is set (completion of event) and turns it off
    while(!PIR1bits.SSPIF);    
    PIR1bits.SSPIF = 0;
    RC0 = 1;
}

void i2c_stop()
{
    //stop routine
    RC0 = 0;              //Sets error on in case stop does not complete its transition
    SSPCON2bits.PEN = 1;  //Initiate Stop condition (generates flag)
    mssp_wait();          //Wait till completion of event
    RC0 = 1;              //Sets error off when stop transition has been made
}

void i2c_nack()
{
    //Sends the no acknowledge
    RC0 = 0;                      //Sets error on in case nack does not complete its transition
    SSPCON2bits.ACKDT=1;          //Set as negative acknowledgement
    SSPCON2bits.ACKEN=1;          //Initiate negative acknowledgement signal
    mssp_wait();                  //Wait till completion of event
    RC0 = 1;                      //Sets error off in case nack completes its transition
}

int i2c_write(unsigned char data)
{
    //sends a byte through the buffer and checks for akn
    RC0 = 0;                         //turn on error
    SSPBUF = data;                   //Input data to buffer (this data will be sent))
    mssp_wait();                     //Waits until completion of transition
    while(SSPCON2bits.ACKSTAT);      //check if acknowledgement signal was received 
    RC0 = 1;                         //tunr off error
}

void i2c_read()
{
    //Performes the read routine
    RC0 = 0;                     //error on in case we get stuck reading data
    SSPCON2bits.RCEN = 1;        //Enables Receive mode for I2C            
    while(!SSPSTATbits.BF);      //Wait till buffer is full
    SSPCON2bits.RCEN = 0;        //diseable reception in Master device 
    rece_status = SSPBUF;        //return received data
    mssp_wait();                 //waits until transition of data has finished
    RC0 = 1;                     //error off when we have finished reading data
}

void get_ms(){
    /* The routine to get a measurement from the Lidar is as follows:
     * 1. Write 0x04 to register 0x00
     * 2. Read register 0x01 until LSB goes low
     * 3. Read high byte of the measuremet in 0x0F
     * 4. Read low byte of the measurement in 0x10 
    */
    
    //*******STARTS I2C COMUNICATION******//
    i = 0;
    // 1.---------------------------------------------------------------
    i2c_idle();
    SSPCON2bits.SEN = 1;                 //start condition is started  
    mssp_wait();                         //Wait till completion of event
    i2c_write(slave_Address_Write);      //Write SLAVE address to write
    i2c_write(ACQ_COMMAND);              //Write REGISTER address
    i2c_write(0x04);                     //Take distance measurement with bias correction              
    i2c_stop();                          //Stop transition
    // 2.---------------------------------------------------------------
    RC0 = 1;                               //turn off error
    rece_status = 0x01;
    while (rece_status & 0x01){            //Check if LSB is 1 or 0 (stay in while until 0)
        i2c_idle();
        SSPCON2bits.SEN = 1;               //start condition is started  
        mssp_wait();                       //wait until completion of event
        i2c_write(slave_Address_Write);    //Write SLAVE address to write
        i2c_write(STATUS_Lidar);                 //Write REGISTER address that we want to read
        i2c_stop();                        //Stop transmission

        i2c_idle();                        
        SSPCON2bits.SEN = 1;               //start condition is started  
        mssp_wait();                       //wait until completion of event
        i2c_write(slave_Address_Read);     //Write SLAVE address to read

        i2c_read();                        //Read from slave
        i2c_nack();                        //Respond with no akn
        i2c_stop();                        //Stop reading
    }
    // 3.---------------------------------------------------------------
    RC0 = 1;                               //turn off error
    i2c_idle();
    SSPCON2bits.SEN = 1;                   //start condition is started  
    mssp_wait();                           //wait until completion of event
    i2c_write(slave_Address_Write);        //Write SLAVE address to write
    i2c_write(DIS_HB);                     //Write REGISTER address that we want to read (high byte)
    i2c_stop();                            //Stop transmission  

    i2c_idle();
    SSPCON2bits.SEN = 1;                   //start condition is started  
    mssp_wait();                           //wait for completion of event
    i2c_write(slave_Address_Read);         //Write SLAVE address to read

    i2c_read();                            //Read from slave
    i2c_nack();                            //Respond with no akn
    i2c_stop();                            //Stop reading
    distance = (unsigned int)(rece_status << 8); //save measurement in high part of int distance
    // 4.---------------------------------------------------------------
    RC0 = 1;                               //Same as step 3. but with low part
    i2c_idle();
    SSPCON2bits.SEN = 1;                       
    mssp_wait();
    i2c_write(slave_Address_Write);          
    i2c_write(DIS_LB);
    i2c_stop();

    i2c_idle();
    SSPCON2bits.SEN = 1;           
    mssp_wait();
    i2c_write(slave_Address_Read);

    i2c_read();
    i2c_nack();
    i2c_stop();
    distance = distance + (unsigned int)(rece_status); //add low part of data to distance
            
    RC0 = 1;                            //turn off error
    //------------------------------------------------------------------
    // Makes data suitable for serial transition and interpretation
    cont_angle_aux = cont_angle;
    for(y = 7; y >= 5; y--){
        mask = (cont_angle_aux%10) + conver_Ascii;
        mens[y] = mask;
        cont_angle_aux = cont_angle_aux/10;
    }
    for(y = 4; y > 0; y--){
        mask = (distance%10) + conver_Ascii;
        mens[y] = mask;
        distance = distance/10;
    }
    mens[0] = distance + conver_Ascii;
    CLRWDT();        //clear the WDT and postscaler counts when executed
    cont_T2 = 0;     //turns off counter for emergency routine
    PIE1bits.TX1IE = 1;
    TXREG1 = mens[i++];
}

void interrupt Interrupciones ()
{
    if(TMR0IF && TMR0IE){              //Interrupt each millisecond
        TMR0IF = 0;
        TMR0 = 6;
        // VISUAL SECTION OF ROTATION ------------------------------------------
        if(0 == rotation_section){
            if(visual_degree_counter++ >= visual_time_per_step){
                visual_degree_counter = 0;
                PORTB = sequence[i_mot++];
                if (10 == deg_range++){
                    deg_range = 0;
                    get_ms();                   //Take a measurement every 5.645°
                }
                if(i_mot > 3) i_mot = 0;        //This resets the sequence for the stepper motor
                if(steps++ >= visual_degrees/degrees_per_step){
                    steps = 0;
                    rotation_section = 1;
                    hidden_degree_counter = 0;
                }
            }
        }
        // NON-VISUAL SECTION OF ROTATION --------------------------------------
        else{
            if(hidden_degree_counter++ >= (hidden_time_per_step)){
                hidden_degree_counter = 0;
                PORTB = sequence[i_mot++];
                if(i_mot > 3) i_mot = 0;    //This resets the sequence for the stepper motor
                if(steps++ >= (hidden_degrees/degrees_per_step)){
                    steps = 0;
                    rotation_section = 0;
                    visual_degree_counter = 0;
                    cont_angle = 0;
                }
            }
        }
    }
    // DATA TRANSMITION --------------------------------------------------------
    if (PIR1bits.TX1IF && PIE1bits.TX1IE){
        PIR1bits.TX1IF = 0;
        TXREG1 = mens[i++];
        if (i == 10){
            PIE1bits.TX1IE = 0;
            cont_angle++;
        }
    }
    // EMERGENGY ROUTINE -------------------------------------------------------
    if (TMR2IF && TMR2IE){
        TMR2IF = 0;
        TMR2 = 6;                   //2 ms to overflow
        cont_T2++;
        if(cont_T2 == 500){         //1 second without activity
            //CALL EMERGENCY ROUTINE
            RC0 = 0;                //Turn on error LED
            cont_T2 = 0;
            TMR0ON = 0;             //Stops Timer0
            TMR2ON = 0;             //Stops Timer2
        }
    }
}

void GPIO_ini()
{
    TRISC = 0xBE;  //RC0,Tx as outputs
    RC0 = 1;       //Starts off
    
    TRISB = 0xF0;             //output for sequence on port b
    PORTB = sequence[i_mot];  //everything starts off
}

void UART_ini()
{
    TXSTA1 = 0b00100000;        //dont care, 8 bit transmission, transmit enable,Asynchronous,Sync Break transmission is completed,low speed,TSR is full 
    RCSTA1 = 0b10010000;        //serial port enable,8bit reception,dont care,enables receiver,Disables address detection, no framing error, no overrun error
    BAUDCON1bits.BRG16 = 0;     //8-bit Baud Rate Generator ? SPBRGx only (Compatible mode), SPBRGHx value is ignored
    SPBRG1 = 12;                //baud rate of 9600 bps, FOSC/[64 (n + 1)]
    
    PIE1bits.TX1IE = 0;         //tx1 interrupt disabled 
    TX1IP = 1;                  //tx1 interrupt high priority
    PIR1bits.TX1IF = 0;         //flag is off
    RC1IE = 0;                  //turn off reception interrupt 
    RC1IF = 0;                  //flag is off
}

void IIC_ini()
{
    SSPSTATbits.SMP = 0;        //High Speed (400 kHz)
    SSPCON1bits.SSPEN = 1;      //Enables the serial port and configures the SDA and SCL pins as the serial port pins 
    SSPCON1bits.SSPM = 0b1000;  //I2C Master mode, clock = Fosc/(4*(SSPADD + 1))
    SSPADD = 0b00001000;        //to generate a clock of 400kHz
    
    PIR1bits.SSPIF = 0;         //Flag is off
    PIE1bits.SSPIE = 0;         //NOT ENABLE
}

void TIME_ini()
{
    T0CON = 0b01000010;    //Disables Timer, 8 bit timer, internal timer FOSC/4, X, we use prescaler, 1/8  
    TMR0L = 6;             //since we are working with 8 bits, we will have 250 counts (1 ms) to overflow
    TMR0IE = 1;            //Local enable, Enables the TMR0 overflow interrupt
    TMR0IP = 1;            //High priority
    TMR0IF = 0;            //Turns off TMR0 flag
    
    T2CON = 0b00000011;    //Timer2 off, prescaler 16, cuentas de 8us.
    TMR2 = 6;              //2 ms to overflow
    TMR2IE = 1;            //Local enable, Enables the TMR2 overflow interrupt
    TMR2IP = 1;            //High priority
    TMR2IF = 0;            //Turns off TMR2 flag
    
    TMR0ON = 1;            //Starts Timer0
    TMR2ON = 1;            //Starts Timer2
}

void WDT_ini()
{
    WDTCONbits.SWDTEN = 1; //Watchdog Timer is on
}

void main(void) {
    IPEN = 1;   //high piorities in interrupts
    GIEH = 1;   //global enable on
    
    GPIO_ini();
    UART_ini();
    IIC_ini();
    TIME_ini();
    WDT_ini();
    
    while(1);
}