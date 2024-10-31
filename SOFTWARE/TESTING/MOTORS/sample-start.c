/*
######################################################################
*/
// Sample C code to setup pulse counter for sensing wheel motion
SYSCTL_RCGCTIMER_R |= (1<<3); /* enable clock to Timer Block 3 */
SYSCTL_RCGCGPIO_R |= (1<<1); /* enable clock to PORTB */
GPIO_PORTB_DIR_R &= ~(1<<2); /* set PB2 an input pin */
GPIO_PORTB_DEN_R |= (1<<2); /* set PB2 a digital pin */
GPIO_PORTB_B_AFSEL_R |= (1<<2); /* enable alternate function on PB2 */
GPIO_PORTB_PCTL_R &= ~0x00000F00; /* configure PB2 as T3CCP0 pin */
GPIO_PORTB_PCTL_R |= 0x00000700;
TIMER3_CTL_R &= ~(1<<0); /* disable TIMER3A during setup */
TIMER3_CFG_R |= (1<<2); /* configure as 16-bit timer mode */
TIMER3_TAMR_R = 0x13; /* up-counter, edge count, capture mode */
TIMER3_TAMATCHR_R = 0xFFFF; /* set the count limit */
//TIMER3_TAPMR_R = 0xFF; /* set prescaler to 0xFF */
TIMER3_TAPMR_R = 0x00; /* set prescaler to none */
TIMER3_CTL_R |= ~(1<<3)|~(1<<2); /* capture the rising edge */
TIMER3_CTL_R |= (1<<0); /* enable Timer0A */
// = Timer3A_countCapture() // function will return the value of the counter
// = TIMER3_TAR_R

/*
######################################################################
*/
