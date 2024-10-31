/**
* main.c for pwm and wheel pulse counter testing
*/
// Updated 7-Nov-2022
//
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
#include "inc/tm4c123gh6pm.h"
//
//****************************************************************
//
const uint32_t clock_rate=16000000;
//
//****************************************************************
//
// Some defines
//
#define one_micro_second 16
#define one_second 16000000
#define one_tenth_sec 1600000
#define one_milli_sec 16000
#define pi 3.1415926
//
//****************************************************************
//
// Configure the UART and its pins. This must be called before UARTprintf().
//! UART0, connected to the Virtual Serial Port and running at
//! 115,200, 8-N-1, is used to display messages from this application.
//
//*****************************************************************
//
// Enable the GPIO Peripheral used by the UART.
void ConfigureUART(void) 
{ 
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);               // Enable UART0
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);               // Configure GPIO Pins for UART mode.
  MAP_GPIOPinConfigure(GPIO_PA0_U0RX);
  MAP_GPIOPinConfigure(GPIO_PA1_U0TX);
  MAP_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1); // Use the internal 16MHz oscillator as the UART clock source.
  UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);              // Initialize the UART for console I/O. Assumes uP clock is at 16 Mhz
  UARTStdioConfig(0, 115200, clock_rate);
}
//
// ****************************************************
//
void Config_Ports() {
  // Ports Used:
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

  #define Right_button 0x01   // right = sw2
}
//
//****************************************************************
//
void Delay(int count)
{
  while(count--);
}
//
//****************************************************************
//
void SysTick_Init() {
  NVIC_ST_CTRL_R = 0; // disable SysTick
  NVIC_ST_RELOAD_R = 0x00ffffff;
  NVIC_ST_CURRENT_R = 0;
  NVIC_ST_CTRL_R = 0x05; // use core clock
}
//
//**************************************************************
//
// bin_to_hex()
void bin_to_hex(uint16_t n,char hex_str[]) {
  const char lookup[16] = {'0','1','2','3','4','5','6','7','8','9','a','b','c','d','e','f'};
  hex_str[0]=lookup[(n>>12) & 0x0f];
  hex_str[1]=lookup[(n>> 8) & 0x0f];
  hex_str[2]=lookup[(n>> 4) & 0x0f];
  hex_str[3]=lookup[(n ) & 0x0f];
  hex_str[4]=0;
}
//
//*****************************************************************
//
// Configure counters for tracking wheels
//
//*****************************************************************
//
void Config_Counters() {
  // Init Counters for feedback from wheels.
  // B0 is Timer 2
  // B2 is timer 3
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
// **************************************************************
//
uint16_t ReadLeft() {
  return TIMER3_TAR_R;
}
//
// **************************************************************
//
uint16_t ReadRight() {
  return TIMER3_TBR_R;
}
//
// **************************************************************
//
/* Initialize Clock settings for PWM and GPIO PORT */
// Init PWM to one selected pin at 1 Khz and variable duty cycle initially at 0%.
//
Init_PWM() {
  SYSCTL_RCGCPWM_R |= 2; /* Enable clock to PWM1 module */
  SYSCTL_RCGCGPIO_R |= 0x01; /* Enable system clock to PORTA */
  SYSCTL_RCC_R &= ~0x00100000; /* Directly feed clock to PWM1 module without pre-
  divider */
  /* Setting of PA6, PA7 pins for M1PWM2, M1PWM3 channel output pin */
  GPIO_PORTA_AFSEL_R |= (3<<6); /* PA6, PA7 sets a alternate function */
  GPIO_PORTA_PCTL_R &= ~0xff000000; /*set PA6, PA7 */
  GPIO_PORTA_PCTL_R |= 0x55000000; /* to be PA6, PA7 PWM PMCx function 5 */
  GPIO_PORTA_DEN_R |= (3<<6); /* set PA6, PA& as a digital pin */
  // M1PWM2 to pin PA6
  // M1PWM3 to pin PA7
  // PWM1 channel 2 setting; 100% = 16000; freq = 1000 Hz
  PWM1_1_CTL_R &= ~(1<<0); /* Disable Generator 1 counter */
  PWM1_1_CTL_R &= ~(1<<1); /* select down count mode of counter 1*/
  PWM1_1_GENA_R = 0x000000c8; /* CLR PWM output when counter reloaded and SET when
  matches PWMCMPA */
  PWM1_1_GENB_R = 0x00000c08; /* CLR PWM output when counter reloaded and SET when
  matches PWMCMPB */
  PWM1_1_LOAD_R = 16000; /* set load value for 1kHz (16MHz/16000) */
  PWM1_1_CMPA_R = 0; /* set A duty cycle to 0% by loading of 0 to
  PWM1CMPA */
  PWM1_1_CMPB_R = 0; /* set B duty cycle to 0% by loading of 0 to
  PWM1CMPB */
  PWM1_1_CTL_R = 1; /* Enable Generator 1 counter */
  PWM1_ENABLE_R = 0x0c; /* Enable PWM1 channel outputs 2 and 3 */
}
//
//*************************************************************
//
void SetPWMLeft(int width) {
  PWM1_1_CMPA_R=width;
}
//
//**************************************************************
//
void SetPWMRight(int width) {
  PWM1_1_CMPB_R=width;
}
//
//*********************************************************************
//
// Display status of testing
//
//********************************************************************
//
void display_info() {
  UARTprintf("wheel counter L=%d wheel counter R=%d PWM L=%d%% PWM R=%d%%\n",
    ReadLeft(),
    ReadRight(),
    (100*PWM1_1_CMPA_R+8000)/16000,
    (100*PWM1_1_CMPB_R+8000)/16000
    );
}
//
//**********************************************************************
//
// main()
//
//*********************************************************************
//
int main(void)
{
  // Set the clocking to run directly from the crystal. MEANS 16 MHz
  MAP_SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);
  ConfigureUART(); // Initialize the UART to 115200
  MAP_FPULazyStackingEnable(); // init FPU stuff
  Config_Ports(); // Configure I/O ports
  Config_Counters(); // initialize wheel counters
  Init_PWM(); // Init PWM outputs
  SysTick_Init(); // make sure SysTick clock is running
  //
  // *************************************************************
  //
  // We are finished initializing. Announce yourself and run the test
  //
  UARTprintf("\nCOMP 462 pwm and counter testing\n");
  //
  // *************************************************************
  int i;
  //
  GPIO_PORTF_DATA_R = 0x08; // set Green
  SetPWMLeft(0); // set duty cycle to (real) 0%
  SysCtlDelay(one_second);
  display_info();

  GPIO_PORTF_DATA_R = 0x04; // blue on
  // 0% / 0%
  SetPWMLeft(10); // set duty cycle to (close) 0%
  SetPWMRight(10); // set duty cycle to (close) 0%
  SysCtlDelay(one_second);
  display_info();

  // 50% / 0%
  SetPWMLeft(8000); // set duty cycle to (close) 50%
  SetPWMRight(10); // set duty cycle to (close) 0%
  SysCtlDelay(5*one_tenth_sec);
  display_info();

  // 50% / 50%
  SetPWMLeft(8000); // set duty cycle to 50%
  SetPWMRight(8000); // set duty cycle to 50%
  SysCtlDelay(one_second);
  display_info();

  // 100% / 100%
  SetPWMLeft(15990); // set duty cycle to 100%
  SetPWMRight(15900); // set duty cycle to 100%
  SysCtlDelay(one_second);
  display_info();

  // 50% / 50%
  SetPWMLeft(8000); // set duty cycle to 50%
  SetPWMRight(8000); // set duty cycle to 50%
  SysCtlDelay(one_second);
  display_info();

  // 0% / 50%
  SetPWMLeft(10); // set duty cycle to 0%
  SetPWMRight(8000); // set duty cycle to 50%
  SysCtlDelay(one_second);
  display_info();

  // 0% / 0%
  SetPWMLeft(10); // set duty cycle to 0%
  SetPWMRight(10); // set duty cycle to 50%
  SysCtlDelay(one_second);
  display_info();
  for (i=1; i<=20;i++) {
    SetPWMLeft(i*799); /// set duty cycle to n
    SetPWMRight(i*799); // set duty cycle to 50%
    SysCtlDelay(100*one_milli_sec);
    display_info();
  }
  for (i=19; i>=1;i--) {
    SetPWMLeft(i*799); /// set duty cycle to n
    SetPWMRight(i*799); // set duty cycle to 50%
    SysCtlDelay(100*one_milli_sec);
    display_info();
  }
  SetPWMLeft(10); // set duty cycle to 0%
  GPIO_PORTF_DATA_R = 0x00; // blue off
  while (1) {
    SysCtlDelay(250*one_milli_sec);
    display_info();
  } // end of forever loop
} // end of main program
