#include <xc.h> 

#include "pic18f25k80.h" 

#pragma config XINST = OFF      

#pragma config MCLRE = OFF     // Master Clear Enable (MC LR Disabled, RE3 Enabled) 

 

 

#define direccional_derecho LATA3 

#define direccional_izquierdo LATA5 

#define cuartos LATA7 

#define freno LATA6 

#define frontal LATC3 

 

 

#define velocity_factor 62241                     //divide freq by 15, multiply by wheel perimeter, convert to km/hr, se truncaron los decimales 

 

 

#define distance_factor_numerator 100                                                              //converting 4 seconds to  

#define distance_factor_denominator 9 

 

 

#define adc_numerator 100   

#define adc_denominator 255 

 

 

unsigned int t_viejo=0, counter_data = 0,counter_bluetooth = 0,counter_luces = 0,counter_adc = 0; 

unsigned char send_data[4] = {255,0,0,0};                                                          //ID, velocidad [km/h],distancia[dm],nivel de bateria [%] 

unsigned char TMR3_overflows = 0,temp_duty = 0, counter_dist = 0, counter_tx = 0, i = 0; 

bit flag_dir_der = 0, flag_dir_izq = 0, freno_flag = 0; 

 

 

 

 

void interrupt ISR_alta() 

{ 

    if(PIR1bits.RC1IF && PIE1bits.RC1IE) 

    { 

        counter_bluetooth = 0; 

        if(((( RCREG1 & 0b00111111)+36) < temp_duty) && (freno_flag == 1)) 

        { 

            freno_flag = 0; 

            freno = 0;                                        //freno on 

        } 

        else if(freno_flag) 

        { 

            freno_flag = 0; 

            freno = 1;                                        //freno off 

        } 

        flag_dir_der = (RCREG1 & 0b01000000) >> 6;            //cellphone send dutyCycle + 64 

        flag_dir_izq = (RCREG1 & 0b10000000) >> 7;            //cellphone sends dutyCycle + 128 

        if((RCREG1 & 0b00111111) != 63){ 

            CCPR2L = (RCREG1 & ((unsigned)0b00111111))+ 36;   //duty cycle 

            temp_duty = CCPR2L; 

        } 

        else{ 

            cuartos = ~cuartos; 

            frontal = ~frontal;  

        } 

    } 

    if((CCP5IF == 1) && (CCP5IE == 1)) 

    { 

        CCP5IF = 0;  

        if(CCPR5 != t_viejo) 

        { 

            send_data[1] = (unsigned)velocity_factor/(CCPR5 - t_viejo);     //velocity in km/h 

            t_viejo = CCPR5; 

        } 

        TMR3_overflows = 0; 

    } 

      

    if(TMR0IF == 1 && TMR0IE == 1) 

    { 

        TMR0IF = 0; 

        TMR0 = 6; 

        if(counter_data++ == 500) 

        { 

            counter_data = 0; 

            if(counter_dist++ == 7){ 

                counter_dist = 0; 

                send_data[2] = (unsigned)(send_data[1]*distance_factor_numerator)/distance_factor_denominator;  

                                                                              //distance in dm 

            } 

            if(counter_tx++ == 1){ 

                counter_tx = 0; 

                TXREG1 = send_data[i++]; 

            } 

            if(i>3) i = 0; 

            if(!freno_flag) freno_flag = 1;                                   //minimum time for there to be a change in the state of the brakelight 

        } 

        if(counter_luces++ == 300) 

        { 

            counter_luces = 0; 

            if(flag_dir_der) direccional_derecho = ~direccional_derecho;      //intermitente 

            else direccional_derecho = 1;                                     //direccional apagado 

            if(flag_dir_izq) direccional_izquierdo = ~direccional_izquierdo;  //intermitente 

            else direccional_izquierdo = 1;                                   //direccional apagado 

        } 

                                                                              //if no data has been recieved in 1500 ms then the dutycycle = 0 

        if(counter_bluetooth++ >= 1500) CCPR2L = 0;                           //stops supply voltage to motor duty cycle = 0 

        if(counter_adc++ == 30000){                                           //reads adc every 30 seconds 

            counter_adc = 0; 

            GO_DONE = 1; 

        } 

    } 

     

    if(TMR3IF == 1 && TMR3IE == 1) 

    { 

        TMR3IF = 0; 

        //this will keep track of the amount of overflows of the timer if the frequency at ccp5 is 0 

        if(TMR3_overflows++ >= 3) send_data[1] = 0; 

    } 

     

    if(ADIF == 1 && ADIE == 1) 

    { 

        ADIF = 0; 

        send_data[3] =  ((unsigned int)(ADRESH)*adc_numerator)/adc_denominator; 

    } 

} 

 

 

void timer_ini() 

{ 

    //time delay for different features 

     

    T0CON = 0b11000010;   // oscilating at 250000 hz, each count is 4us 

    TMR0IF = 0;           //flag 

    TMR0IE = 1;           //local enable 

    TMR0IP = 1;           //High priority 

    TMR0 = 6;             //we need 250 counts for an over flow so that 1ms has passed. 

     

    //accelerator 

     

    T2CON = 0b00000111;   //prescaler is 16 and period of pwm at 1ms 

     

    //speedometer 

     

    T3CON = 0b00101011;   //prescaler of 4 , timer oscilating at 500khz 

    TMR3GE = 0; 

    TMR3IF = 0; 

    TMR3IE = 1; 

    TMR3IP = 1; 

} 

 

 

void GPIO_ini() 

{ 

    TRISC7 = 1;            // input for rx 

    TRISC6 = 0;            //output for tx 

    TRISB5= 1;             //input ccp5 on bit 5 

    TRISA = 0b00000110;    //RA7-RA3 output for lights, RA2 input for adc reference, RA1 adc input, RA0 cvreff  

    TRISC3 = 0;            //output for frontal light 

    TRISC2 = 0;            //output for pwm 

    RA3 = 1; 

    RA5 = 1; 

    RA6 = 1; 

    RA7 = 1; 

    RC3 = 1; 

} 

 

 

void UART_ini() 

{ 

    // serial communication 

     

    TXSTA1 = 0b00100100;       //baudrate 9600 

    RCSTA1 = 0b10010000; 

    BAUDCON1bits.BRG16 = 0; 

    SPBRG1 = 51; 

     

    PIE1bits.RC1IE = 1;        //turn on reception 

    RC1IP = 1;                 //reception is high priority 

} 

 

 

void PWM_ini() 

{ 

    //Accelerator 

     

    PR2 = 124;               //period of pwm equal to 1ms, prescaler of timer 2 needs to be 16 

    CCP2CON = 0b00111111;    //PWM mode with with bit 5 and 4 equal to T2CON <1:0> for comparator 

    C2TSEL = 0;              //timer 2 for ccp2 

    CCPR2L = 0;              //so that motor starts in an off state 

} 

 

 

void speed_ini() 

{ 

    //speedometer 

     

    CCP5CON = 0b00000101;   //capture mode, every rising edge 

    C5TSEL = 1;             //based off timer 3 

     

    CCP5IF = 0; 

    CCP5IE = 1;             //habilitador local 

    CCP5IP = 1;             //alta prioridad 

} 

 

 

void adc_ini() 

{ 

    ADIE = 1;               //Enables the A/D interrupt 

    ADIP = 1;               //A/D Converter Interrupt Priority bit 

    ADIF = 0;               //turn off adc module flag 

     

    ANSEL0 = 1; 

    ANSEL1 = 1; 

    ANSEL2 = 1; 

     

    CVRCON = 0b11011010;    //4.1 V at VCref ya que tenemos un AVDD = 5.0 V. 

     

    ADCON0 = 0x05;          //configure A/D module, TURNED ON THE MODULE and selected channel 1 

                            //to turn on module, GO_DONE = 1 

    ADCON1 = 0b00001011;    //upper reference is vdd and lower reference is AN2 

    ADCON2 = 0b00010001; 

     

    GO_DONE = 1;            //inicia conversion 

} 

 

 

void perif_ini ()  

{ 

    IPEN = 1;        //select priorities on interrupts 

    GIEH = 1;        //global enable 

     

    GPIO_ini();      //ports management 

    timer_ini();     //timers needed for perifels 

    UART_ini();      //serial communication 

    PWM_ini();       //accelerator 

    speed_ini();     //speedometer 

    adc_ini();       //adc for bettery level 

} 

 

 

void main(void)  

{ 

    perif_ini(); 

    while(1); 

} 

 