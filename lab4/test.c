
#include <stdio.h>
#include <string.h>
#include "tm4c123gh6pm.h"
#include <TM4C123.h>
/////////////////////////////intializing///////////////////
void init_portf(void)
{
SYSCTL_RCGCGPIO_R |= 0X20;
while ((SYSCTL_PRGPIO_R & 0X20) == 0)
;
GPIO_PORTF_LOCK_R = 0X4C4F434B;
GPIO_PORTF_CR_R = 0X0E;
GPIO_PORTF_AMSEL_R = 0X00;
GPIO_PORTF_PCTL_R = 0X00000000;
GPIO_PORTF_DIR_R = 0X0E;
GPIO_PORTF_AFSEL_R = 0X00;
GPIO_PORTF_DEN_R = 0X0E;
GPIO_PORTF_DATA_R &= ~0x0E;
}
void init_uart(void)
{
SYSCTL_RCGCUART_R |= 0X0001;
SYSCTL_RCGCGPIO_R |= 0X0001;
UART0_CTL_R &= ~(0x0001);
UART0_IBRD_R = 104;
UART0_FBRD_R = 11;
UART0_LCRH_R = 0x70;
UART0_CTL_R |= 0x301;
GPIO_PORTA_AFSEL_R |= 0x03;
GPIO_PORTA_PCTL_R = (GPIO_PORTA_PCTL_R & 0xFFFFFF00) +
0x00000011;
GPIO_PORTA_DEN_R |= 0x03;
GPIO_PORTA_AMSEL_R &= ~(0X03);
}
void init_systick(void)
{
NVIC_ST_CTRL_R = 0;
NVIC_ST_RELOAD_R = 0x00FFFFFF;
NVIC_ST_CURRENT_R = 0;
NVIC_ST_CTRL_R = 0x05;
}
///////////////////////done


/***********functions************/
void send(char c)
{
while ((UART0_FR_R & 0x20) != 0)
;
UART0_DR_R = c;
}
char rece(void)
{
while ((UART0_FR_R & 0x10) != 0)
;
return (char)(UART0_DR_R & 0xFF);
}
void led_on(unsigned char data)
{
GPIO_PORTF_DATA_R = data;
}
void led_off(void)
{
GPIO_PORTF_DATA_R = 0x00;
}
void wait_1ms(void)
{
NVIC_ST_RELOAD_R = 16000 - 1;
NVIC_ST_CURRENT_R = 0;
while ((NVIC_ST_CTRL_R & 0x00010000) == 0)
;
}
void delay(int t)
{
int i = 0;
for (i = 0; i < t; i++)
wait_1ms();
}
/***************input outpu from pc*************************/
void get_char(char *inp)
{
char c;
int i;
for (i = 0; i < 2; i++)
{
c = rece();
if (c != 0x0D)
{
inp[i] = c;
send(inp[i]);
}


else
break;
}
}
void print(char *s)
{
while (*s)
{
send(*s);
s++;
}
}
/***************main***********************/
int main()
{
char inp[2] = {0};
init_portf();
init_uart();
init_systick();
while (1)
{
print("A|B|D: \n");
get_char(inp);
if (strcmp(inp, "A") == 0)
{
led_off();
delay(6000);
led_on(0x02);
memset(inp, 0, 2);
}
else if (strcmp(inp, "B") == 0)
{
led_off();
delay(30000);
led_on(0x04);
memset(inp, 0, 2);
}
else if (strcmp(inp, "D") == 0)
{
led_off();
delay(120000);
led_on(0x08);
memset(inp, 0, 2);
}
print("\n");
 }
}