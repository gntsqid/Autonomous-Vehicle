/**
* main_462_final.c for pwm, wheel pulse counter, sonar testing
* K. Zierhut
*/
//
// Updated 16-Nov-2023 **** state machine completed and debug code removed
// Modified 11/24/24 - Steven Lang: Added missing dependencies and fixed errors to get first build running
// Modified 11/29/24 - Steven Lang: Added printing of current run_mode and added condition to stop when both sensors read under 40 cm
//
// ********************************************************************


// INCLUDES

//#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "utils/uartstdio.c"
#include "inc/tm4c123gh6pm.h"
#include "i_trig.h"

// DEFINES

const uint32_t clock_rate=16000000;

#define one_micro_second 16
#define one_second 16000000
#define one_tenth_sec 1600000
#define one_milli_sec 16000
#define five_ms 80000
#define thirty_ms 480000
#define thirty_five_ms 560000
#define forty_ms 640000
#define fifty_ms 800000
#define pi 3.1415926
#define ST_max 0x00ffffff

int const half_max=10000; // for PWM
int const max_delta_PWM=500;
int const turn_PWM_amount=4000;
int const left_sonar=1;
int const right_sonar=2;
const int MAX_PWM = 16000;
int i_atan2(int x,int y);
int i_sin(int a);
int i_cos(int a);

// FUNCTIONS

//
//****************************************************************
//
// Configure the UART and its pins. This must be called before UARTprintf().
//! UART0, connected to the Virtual Serial Port and running at
//! 115,200, 8-N-1, is used to display messages from this application.
//
//*****************************************************************
//
void ConfigureUART(void) {

    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA); // Enable the GPIO Peripheral used by the UART.
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0); // Enable UART0

    // Configure GPIO Pins for UART mode.
    MAP_GPIOPinConfigure(GPIO_PA0_U0RX);
    MAP_GPIOPinConfigure(GPIO_PA1_U0TX);
    MAP_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    // Use the internal 16MHz oscillator as the UART clock source.
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
    // Initialize the UART for console I/O. Assumes uP clock is at 16 Mhz
    UARTStdioConfig(0, 115200, clock_rate);
}

void Config_Ports() {
    // initialize PORT A
    SYSCTL_RCGCGPIO_R |= 0x01; // enable clock for PORTA
    GPIO_PORTA_DEN_R |= 0x30; // enable digital bits 4,5 on PORTA
    GPIO_PORTA_DIR_R |= 0x30; // make bits 4,5 trig output pins

    // Initialize port B
    SYSCTL_RCGCGPIO_R |= 0x02; // enable clock for PORTB
    GPIO_PORTB_DEN_R |= 0xcc; // digital enable bits 2,3,6,7 on PORTB
    GPIO_PORTB_DIR_R &= ~0xcc; // make bits 2,3,6,7 as input pins

    // Initialize PORTF
    SYSCTL_RCGCGPIO_R |= 0x20; // enable clock for PORTF

    // Unlock Port F
    GPIO_PORTF_LOCK_R = GPIO_LOCK_KEY;
    GPIO_PORTF_CR_R |= 0x01;
    GPIO_PORTF_LOCK_R = 0;

    // Initialize port F
    GPIO_PORTF_DEN_R |= 0x1f; // digital enable bits 0..4 on PORTF
    GPIO_PORTF_DIR_R |= 0x0e; // make bits 1..3 as output pins
    GPIO_PORTF_PUR_R |= 0x11; // select bits 0,4 pullup

    // left = sw1
    #define Left_button 0x10
    // right = sw2
    #define Right_button 0x01
}

uint16_t measure_sonar(int sonar_port) {
    // 1 for left, 2 for right
    if (sonar_port<1 || sonar_port>2) return 0;
        int pulse, rise, fall;

    // assume systick running from init less than 50 ms before
    // input=1 means trigger on pin A4, echo on B6
    // input=2 means trigger on pin A5, echo on B7
    // send 10 uS pulse to start sensor
    // pulse=NVIC_ST_CURRENT_R; // pulse out time
    GPIO_PORTA_DATA_R |= sonar_port<<4;
    SysCtlDelay(10 * one_micro_second);
    GPIO_PORTA_DATA_R &= ~0x30;

    // code to measure echo response
    pulse=NVIC_ST_CURRENT_R;
    while (( (pulse-NVIC_ST_CURRENT_R)<five_ms) && (GPIO_PORTB_DATA_R & (sonar_port<<6))==0);

    if ((GPIO_PORTB_DATA_R & (sonar_port<<6))==0) return 500;
        rise=NVIC_ST_CURRENT_R; // got leading edge
    while (((rise-NVIC_ST_CURRENT_R)<thirty_five_ms) && (GPIO_PORTB_DATA_R & (sonar_port<<6))!=0);

    if ((GPIO_PORTB_DATA_R & (sonar_port<<6))!=0) return 0;
        fall=NVIC_ST_CURRENT_R; // got leading edge

    // UARTprintf("pulse to rise = %d and rise to fall = %d\n",pulse-rise,rise-fall);

    while ((pulse-NVIC_ST_CURRENT_R)<forty_ms);

    return ((rise-fall)*17+8000)/16000; // s.b. delta*170*100/16000000
}

void SysTick_Init() {
    NVIC_ST_CTRL_R = 0; // disable SysTick
    NVIC_ST_RELOAD_R = ST_max;
    NVIC_ST_CURRENT_R = 0;
    NVIC_ST_CTRL_R = 0x05; // use core clock
}

// bin_to_hex()
void bin_to_hex(uint16_t n,char hex_str[]) {
    const char lookup[16] = {'0','1','2','3','4','5','6','7','8','9','a','b','c','d','e','f'};
    hex_str[0]=lookup[(n>>12) & 0x0f];
    hex_str[1]=lookup[(n>> 8) & 0x0f];
    hex_str[2]=lookup[(n>> 4) & 0x0f];
    hex_str[3]=lookup[(n ) & 0x0f];
    hex_str[4]=0;
}

// Configure counters for tracking wheels
void Config_Counters() {
    // Init Counters for feedback from wheels.
    // B2 is Timer 3A
    // B3 is timer 3B
    SYSCTL_RCGCTIMER_R |= (1<<3); /* enable clock to Timer Block 3 */
    SYSCTL_RCGCGPIO_R |= (1<<1); /* enable clock to PORTB */
    GPIO_PORTB_DIR_R &= ~(0x0c); /* set PB2,PB3 as input pins */
    GPIO_PORTB_PUR_R |= (0x0c); /* and a pullup */
    GPIO_PORTB_DEN_R |= (0x0c); /* set PB2,PB3 a digital pin */
    GPIO_PORTB_AFSEL_R |= (0x0c); /* enable alternate function on PB2,PB3 */
    GPIO_PORTB_PCTL_R &= ~0x0000FF00; /* configure PB2 as T3CCP0 and PB3 as T3CCP1
    */
    GPIO_PORTB_PCTL_R |= 0x00007700; /* PMCx functions 7 */
    TIMER3_CTL_R &= ~(1<<0); /* disable TIMER3A during setup */
    TIMER3_CFG_R |= (1<<2); /* configure as 16-bit timer mode */
    TIMER3_TAMR_R = 0x13; /* up-counter, edge count, capture mode */
    TIMER3_TAMATCHR_R = 0xFFFF; /* set the count limit */
    TIMER3_TAPMR_R = 0x00; /* set prescaler to none */
    TIMER3_CTL_R |= (3<<2); /* capture both edges */
    TIMER3_CTL_R |= (1<<0); /* enable Timer3A */
    TIMER3_CTL_R &= ~(1<<8); /* disable TIMER3B during setup */
    TIMER3_TBMR_R = 0x13; /* up-counter, edge count, capture mode */
    TIMER3_TBMATCHR_R = 0xFFFF; /* set the count limit */
    TIMER3_TBPMR_R = 0x00; /* set prescaler to none */
    TIMER3_CTL_R |= (3<<10); /* capture both edges */
    TIMER3_CTL_R |= (1<<8); /* enable Timer3A */
}

uint16_t Read_Left() { // wheel pulse counter
    return TIMER3_TAR_R;
}

uint16_t Read_Right() { // wheel pulse counter
    return TIMER3_TBR_R;
}

/* Initialize Clock settings for PWM and GPIO PORT */
// Init PWM to one selected pin at 1 Khz and variable duty cycle initially at 0%.
Init_PWM() {
    SYSCTL_RCGCPWM_R |= 2; /* Enable clock to PWM1 module */
    SYSCTL_RCGCGPIO_R |= 0x01; /* Enable system clock to PORTA */
    SYSCTL_RCC_R &= ~0x00100000; /* Directly feed clock to PWM1 module without pre-
    divider */
    /* Setting of PA6, PA7 pins for M1PWM2, M1PWM3 channel output pin */
    GPIO_PORTA_AFSEL_R |= (3<<6); /* PA6, PA7 select alternate function */
    GPIO_PORTA_PCTL_R &= ~0xff000000; /*set PA6, PA7 */
    GPIO_PORTA_PCTL_R |= 0x55000000; /* to be PMCx function 5 */
    GPIO_PORTA_DEN_R |= (3<<6); /* set PA6, PA7 as a digital pin */
    // M1PWM2 to pin PA6
    // M1PWM3 to pin PA7
    // PWM1 channel 2 setting; 100% = 16000; freq = 1000 Hz
    PWM1_1_CTL_R &= ~(1<<0); /* Disable Generator 1 counter */
    PWM1_1_CTL_R &= ~(1<<1); /* select down count mode of counter 1*/
    PWM1_1_GENA_R = 0x000000c8; /* CLR PWM output when counter reloaded and SET when
    matches PWMCMPA */
    PWM1_1_GENB_R = 0x00000c08; /* CLR PWM output when counter reloaded and SET when
    matches PWMCMPB */
    PWM1_1_LOAD_R = MAX_PWM; /* set load value for 1kHz (16MHz/16000) */
    PWM1_1_CMPA_R = 0; /* set A duty cycle to 0% by loading of 0 to
    PWM1CMPA */
    PWM1_1_CMPB_R = 0; /* set B duty cycle to 0% by loading of 0 to
    PWM1CMPB */
    PWM1_1_CTL_R = 1; /* Enable Generator 1 counter */
    PWM1_ENABLE_R = 0x0c; /* Enable PWM1 channel outputs 2 and 3 */
}

void Set_PWM_out_left(int width) {
    if (width<0) PWM1_1_CMPA_R=0;
    else if (width>MAX_PWM) PWM1_1_CMPA_R=MAX_PWM;
    else
        PWM1_1_CMPA_R=width;
}

void Set_PWM_out_right(int width) {
    if (width<0) PWM1_1_CMPB_R=0;
    else if (width>MAX_PWM) PWM1_1_CMPB_R=MAX_PWM;
    else
    PWM1_1_CMPB_R=width;
}

// Display status of testing
void display_info(int ld, int rd, int a, int da, int dr, int cr, int mode) {
    UARTprintf("wh count L=%d, wh count R=%d, PWM L=%d%%, PWM R=%d%%, L dist=%d cm, R dist=%d cm, ang=%d, des ang=%d, down_r=%d, cross_r=%d, current-mode = %d\n", Read_Left(), Read_Right(), (100*PWM1_1_CMPA_R+8000)/16000, (100*PWM1_1_CMPB_R+8000)/16000, ld, rd, a, da, dr, cr, mode );
}

int main(void)
{
    // Set the clocking to run directly from the crystal. MEANS 16 MHz
    //MAP_SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
    // SYSCTL_OSC_MAIN);
    ConfigureUART(); // Initialize the UART to 115200
    MAP_FPULazyStackingEnable(); // init FPU stuff
    Config_Ports(); // Configure I/O ports
    Config_Counters(); // initialize wheel counters
    Init_PWM(); // Init PWM outputs
    SysTick_Init(); // make sure SysTick clock is running

    // We are finished initializing. Announce yourself and run the test

    UARTprintf("\nCOMP 462 complete car testing\n");

    // working data

    int left_dist, right_dist,start_time;
    int old_left_wheel=0,old_right_wheel=0;
    int new_left_wheel,new_right_wheel,delta_wheels,angle;
    int desired_angle;
    int pass_counter=0;
    int left_PWM_desired, right_PWM_desired;
    int old_left_PWM=0, old_right_PWM=0;
    int delta_left_PWM, delta_right_PWM;
    float down_range_dist=0, cross_range_dist=0;
    int run_mode=0; // (0= POR, green light, 1= dual button start blue light for 1 sec, 2=running and green light, 3=ended and white)
    int mode_counter;

    while (1) {
        SysTick_Init();
        //
        // mode dependent stuff
        //
        switch (run_mode) {
            // POR, green light, wait for two button start
            case 0:
                GPIO_PORTF_DATA_R = 8;
                if ((GPIO_PORTF_DATA_R & 0x11)==0) { run_mode=1; mode_counter=20; }
                    break;

            // dual button start blue light for 1 sec
            case 1:
                GPIO_PORTF_DATA_R = 4;
                if (mode_counter-- == 0) run_mode=2;
                    break;

            // =running and red light
            case 2:
                GPIO_PORTF_DATA_R = 2;
                if (down_range_dist>900.0 && left_dist < 40 && right_dist < 40) run_mode=3; // done after 900 cm = 9 meters
                    break;

            // =ended and white light
            case 3:
                GPIO_PORTF_DATA_R = 0x0e;
                break;
        } // end mode switch

        // first half cycle
        start_time=NVIC_ST_CURRENT_R;

        // real left sonar

        left_dist=measure_sonar( left_sonar);
        //left_dist= (GPIO_PORTF_DATA_R & Left_button) ? 500 : 75; //******************* debug button simulates 75 cm

        // idle window
        // second half cycle
        // wait for second half cycle

        while ((start_time-NVIC_ST_CURRENT_R)<fifty_ms) ;

        // read right sonar

        right_dist=measure_sonar(right_sonar);
        //right_dist= (GPIO_PORTF_DATA_R & Right_button) ? 500 : 75; //*********************** debug button simulates 75 cm

        // read wheel counters

        new_left_wheel=Read_Left();
        new_right_wheel=Read_Right();

        // consider a change here that sets both wheel counters to zero every cycle until voltage is applied to both motors.


        delta_wheels= (new_left_wheel-old_left_wheel) + (new_right_wheel - old_right_wheel) ;
        //delta_wheels=3; // ****************************** debug

        // get approx degrees
        // wheel base about 15 cm
        // so smallest inc delta from wheels is 360 * 26 cm / 40 / 15

        angle=((new_left_wheel-new_right_wheel)*2+2)/5; // /2.5 rounded
        //angle=-3; // ****************************************************** debug

        // calculations go here

        // compute travel this cycle

        // calculate down and cross range travel
        // wheel circumference (rev) about 8 cm * pi = 26 cm
        // wheel count = 40 per rev each or 80 total

        down_range_dist +=i_cos(angle)*delta_wheels*26./8000.; // div by 100 for scaling of sin and cos
        cross_range_dist+=i_sin(angle)*delta_wheels*26./8000.;

        // desired "steer to" angle starts at straight ahead

        desired_angle=0; // default

        if (cross_range_dist> 5) desired_angle=-20;
        if (cross_range_dist<-5) desired_angle= 20;

        if (left_dist <85) desired_angle = 45;
        if (right_dist<85) desired_angle = -45;

        // calculate PWM outputs

        // start at 50%

        left_PWM_desired = half_max;
        right_PWM_desired = half_max;

        // compute desired PWM amount

        if((desired_angle-angle)> 2) left_PWM_desired+=turn_PWM_amount;
        if((desired_angle-angle)<-2) right_PWM_desired+=turn_PWM_amount;

        // apply limited rate of change of PWM
        // left
        delta_left_PWM=left_PWM_desired-old_left_PWM;
        if (delta_left_PWM> max_delta_PWM) delta_left_PWM= max_delta_PWM;
        if (delta_left_PWM<-max_delta_PWM) delta_left_PWM=-max_delta_PWM;
        old_left_PWM=left_PWM_desired=old_left_PWM+delta_left_PWM;
        if (run_mode!=2) old_left_PWM=left_PWM_desired=0;
        Set_PWM_out_left(left_PWM_desired);

        // right
        delta_right_PWM=right_PWM_desired-old_right_PWM;
        if (delta_right_PWM> max_delta_PWM) delta_right_PWM= max_delta_PWM;

        if (delta_right_PWM<-max_delta_PWM) delta_right_PWM=-max_delta_PWM;
            old_right_PWM=right_PWM_desired=old_right_PWM+delta_right_PWM;
        if (run_mode!=2) old_right_PWM=right_PWM_desired=0;
            Set_PWM_out_right(right_PWM_desired);

        // synchronize


        // display data every x passes
            // best to default to 5, using 10 for testing

        if(pass_counter++ > 10) {
            display_info(left_dist,right_dist,angle,desired_angle, down_range_dist,
            cross_range_dist,
            run_mode);
            pass_counter=0;
        }

        // age data
        old_left_wheel= new_left_wheel;
        old_right_wheel=new_right_wheel;

        // wait for end of cycle

        while ((start_time-NVIC_ST_CURRENT_R)<one_tenth_sec) ;
    } // end of forever loop
} // end of main program
