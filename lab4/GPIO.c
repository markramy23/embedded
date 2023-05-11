#include "tm4c123gh6pm.h"

#include "GPIO.h"
void GPIO_initPORTF()	
{
SYSCTL_RCGCGPIO_R |= 0X20;
while ((SYSCTL_PRGPIO_R & 0X20) == 0);

GPIO_PORTF_LOCK_R = 0X4C4F434B;
GPIO_PORTF_CR_R = 0X0E;
GPIO_PORTF_AMSEL_R = 0X00;
GPIO_PORTF_PCTL_R = 0X00000000;
GPIO_PORTF_DIR_R = 0X0E;
GPIO_PORTF_AFSEL_R = 0X00;
GPIO_PORTF_DEN_R = 0X0E;
GPIO_PORTF_DATA_R &= ~0x0E;
}
void led_on(unsigned char data)
{
GPIO_PORTF_DATA_R |= data;
}
void led_off(void)
{
GPIO_PORTF_DATA_R = 0x00;
}

