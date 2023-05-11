#include "tm4c123gh6pm.h"

#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include "LCD.h"
#include "tm4c123gh6pm.h"
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include "GPIO.h"


volatile char GPS[80];
char GPS_logname[]="$GPRMC,";
char GPS_formated[12][20];
char*token;
float currentLong, currentLat , speed, finalLat=3003.85572 ,finalLong=3116.78872;
const double EARTH_RADIUS = 6371000;
const double PI=3.14159265359;

void UART0_write(char data);
///////////GPIO/////////////////////
//void GPIO_initPORTF()	
//{
//SYSCTL_RCGCGPIO_R |= 0X20;
//while ((SYSCTL_PRGPIO_R & 0X20) == 0);

//GPIO_PORTF_LOCK_R = 0X4C4F434B;
//GPIO_PORTF_CR_R = 0X0E;
//GPIO_PORTF_AMSEL_R = 0X00;
//GPIO_PORTF_PCTL_R = 0X00000000;
//GPIO_PORTF_DIR_R = 0X0E;
//GPIO_PORTF_AFSEL_R = 0X00;
//GPIO_PORTF_DEN_R = 0X0E;
//GPIO_PORTF_DATA_R &= ~0x0E;
//}
//void led_on(unsigned char data)
//{
//GPIO_PORTF_DATA_R = data;
//}
//void led_off(void)
//{
//GPIO_PORTF_DATA_R = 0x00;
//}


////////////Uart////////////////////
void UART_Init(){
	SET_BIT(SYSCTL_RCGCUART_R,1);
	SET_BIT(SYSCTL_RCGCGPIO_R,1);
	CLR_BIT(UART1_CTL_R,0);
	SET(UART1_IBRD_R,520);
	SET(UART1_FBRD_R,53);
	SET(UART1_LCRH_R,0x0070);
	SET(UART1_CTL_R,0x0301);
	SET_BITS(GPIO_PORTB_AFSEL_R,0x03);
	SET_BITS(GPIO_PORTB_PCTL_R,0x00000011);
	SET_BITS(GPIO_PORTB_DEN_R,0x03);
	CLR(GPIO_PORTB_AMSEL_R,0x03);
}
void send(char c)
{
while ((UART7_FR_R & 0x20) != 0)
;
UART1_DR_R = c;
}

char rece(void)
{
while ((UART7_FR_R & 0x10) != 0)
;
return (char)(UART7_DR_R & 0xFF);
}




void GPS_read(){



char recievedChar;
//check for correct log
	char fillGPScounter = 0;
char flag=1;
	char i;
	char c;

do{
	flag=1;
	
	for( i=0;i<7;i++){
		c=rece();
		if(c!= GPS_logname[i]){
			flag=0;
			break;}
		//UART0_write(c);
		}
	}while(flag==0);

	//store the log
	do{
		
		
		recievedChar= rece();
		//UART0_write(recievedChar);
		GPS[fillGPScounter++]=recievedChar;
		//UART0_write(GPS[fillGPScounter-1]);
		
	}while(recievedChar!='*');
	
	

}

void GPS_format(){
	char noOfTokenStrings=0;
	token = strtok(GPS , ",");
	do{
		strcpy(GPS_formated[noOfTokenStrings],token);
		token = strtok(NULL,",");
		noOfTokenStrings++;
	}while(token!=NULL);

	if(strcmp(GPS_formated[1],"A")==0){
		if(strcmp(GPS_formated[3],"N")==0)
			currentLat= atof(GPS_formated[2]);
		else 
			currentLat = -atof(GPS_formated[2]);
		if(strcmp(GPS_formated[5],"E")==0)
			currentLong = atof(GPS_formated[4]);
		else 
			currentLong= -atof(GPS_formated[4]);
		speed=atof(GPS_formated[6]);
		
}
}
float toDegree(float angle){
	int degree=(int)angle/100;
	float minutes = angle - (float)degree*100;
	return(degree+(minutes/60));
}

float toRad(float angle){
	return angle*(PI/180);}

float GPS_getDistance(float currentLong, float currentLat , float destLong,float destLat){
	float currentLongRad = toRad(toDegree(currentLong)); // TODO: momken nkhaly toRad() btakhod raw gps output w thawello radian 3la tool badal toRad(toDegree())
	float currentLatRad = toRad(toDegree(currentLat));
	float destLongRad = toRad(toDegree(destLong));
	float destLatRad = toRad(toDegree(destLat));
	
	float longDiff = destLongRad - currentLongRad;
	float latDiff = destLatRad - currentLatRad ;
		
	float a = pow(sin(latDiff/2),2)+cos(currentLatRad)*cos(destLatRad)*pow(sin(longDiff/2),2);
	double c = 2*atan2(sqrt(a),sqrt(1-a));
	return 6371000*c;
	
	}
	





void UART7_INIT(){
 SYSCTL_RCGCUART_R |= 0x80; //pin 7 is E
 SYSCTL_RCGCGPIO_R |= 0x10; //pin 4 is E

UART7_CTL_R &= ~ 0x01;
UART7_IBRD_R = 104;  
UART7_FBRD_R = 11;
UART7_LCRH_R = 0x70  ;
UART7_CTL_R =0x301 ;
  
GPIO_PORTE_AFSEL_R |= 0x3;
GPIO_PORTE_PCTL_R |= 0x11;
GPIO_PORTE_DEN_R |= 0x03;
GPIO_PORTE_AMSEL_R &= ~0x03;  
}
//void RGBLED_Init(void){
//SYSCTL_RCGCGPIO_R |= 0x20;             //Port F Clock enable
//while((SYSCTL_PRGPIO_R & 0x20)==0){};  //Delay
//GPIO_PORTF_DIR_R |= 0x0E;              //Enable Output
//GPIO_PORTF_AFSEL_R &= ~(0x0E);         //No alternate function
//GPIO_PORTF_PCTL_R &= ~(0x0000FFF0);    //Clear PCTL bit
//GPIO_PORTF_DEN_R |= 0x0E;              //Enable Digital Pins 3 2 1 
//GPIO_PORTF_AMSEL_R &= ~(0x0E);         //Disable Analog Mode
//GPIO_PORTF_DATA_R &= ~(0x0E);          //Initialize LEDS to be off  
//}

//void RGB_set(uint8_t mask){
//  mask &= 0x0E;
//  GPIO_PORTF_DATA_R |= mask;
//}
//void RGB_clear(uint8_t mask){
//  mask &= 0x0E;
//  GPIO_PORTF_DATA_R &= ~mask;
//}


//uint8_t UART0_Available(){
//  return ((UART0_FR_R &UART_FR_RXFE)==UART_FR_RXFE)? 0:1;
//}
//char UART0_read(){
//   while(UART0_Available() !=1){};
//    return (char) (UART0_DR_R & 0xFF);
//    }

//void UART1_write(char data){
//  while((UART1_FR_R & UART_FR_TXFF)==UART_FR_TXFF){};
//    UART1_DR_R = data;
//}

//void getCommand(char *str,uint8_t maxLen){
//  char c;
//  int8_t i;
//  for(i =0;i<maxLen;i++){
//    c = UART0_read();
//    if(c=='\n' || c=='\r'){ break;
//    }
//    else str[i]=c;
//    UART1_write(c);
//    
//  }
//  
//}
//void printstr(char *str){
//  while(*str){
//    UART1_write(*str);
//    str++;
//  }
//}
////////////////testinggg/////////////////////
void UART0_INIT(){
 SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;
 SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R0;

UART0_CTL_R &= ~ UART_CTL_UARTEN;
UART0_IBRD_R = 104;  
UART0_FBRD_R = 11;
UART0_LCRH_R |= (UART_LCRH_WLEN_8 | UART_LCRH_FEN)  ;
UART0_CTL_R = (UART_CTL_RXE | UART_CTL_TXE | UART_CTL_UARTEN);
  
GPIO_PORTA_AFSEL_R |= 0x03;
GPIO_PORTA_PCTL_R = (GPIO_PORTA_PCTL_R &= ~0xFF)|(GPIO_PCTL_PA1_U0TX|GPIO_PCTL_PA0_U0RX);
GPIO_PORTA_DEN_R |= 0x03;
GPIO_PORTA_AMSEL_R &= ~0x03;  
}
void UART0_write(char data){
  while((UART0_FR_R & UART_FR_TXFF)==UART_FR_TXFF){};
    UART0_DR_R = data;
}

uint8_t UART0_Available(){
  return ((UART0_FR_R &UART_FR_RXFE)==UART_FR_RXFE)? 0:1;
}
void printstr(char *str){
  while(*str){
    UART0_write(*str);
    str++;
  }
}

char UART0_read(){
   while(UART0_Available() !=1){};
    return (char) (UART0_DR_R & 0xFF);
    }
void RGBLED_Init(void){
SYSCTL_RCGCGPIO_R |= 0x20;             //Port F Clock enable
while((SYSCTL_PRGPIO_R & 0x20)==0){};  //Delay
GPIO_PORTF_DIR_R |= 0x0E;              //Enable Output
GPIO_PORTF_AFSEL_R &= ~(0x0E);         //No alternate function
GPIO_PORTF_PCTL_R &= ~(0x0000FFF0);    //Clear PCTL bit
GPIO_PORTF_DEN_R |= 0x0E;              //Enable Digital Pins 3 2 1 
GPIO_PORTF_AMSEL_R &= ~(0x0E);         //Disable Analog Mode
GPIO_PORTF_DATA_R &= ~(0x0E);          //Initialize LEDS to be off  
}
void RGB_set(uint8_t mask){
  mask &= 0x0E;
  GPIO_PORTF_DATA_R |= mask;
}




int main()

{
volatile	float flag;
	volatile float dist=0;//variable to save distance after calling function that calculate distance between two points
	float lat_arr[3];/*array to save latitude */
	float long_arr[3];/*array to save current Longitude*/
volatile	float displacement;
    int counter=0;/*variable used as acounter to increased every cycle of distance calc (every second) */
	UART0_INIT();
	UART7_INIT();	/*initialize the uart of the gps*/	
	GPIO_initPORTF(); // initialize port f for leds
	SysTick_Init(); //initialize systick for delays
	LCD_init(); /*initialize the gpio port of the lcd currently B*/
	
		
while(1){
	
		GPS_read();
		GPS_format();
		
		 long_arr[counter] = currentLong;
		 lat_arr[counter] = currentLat;
		if(counter!=0)
			{
				flag=GPS_getDistance(long_arr[counter-1],lat_arr[counter-1],long_arr[counter],lat_arr[counter]);
				if(	flag<1000){
				if(speed>0.5){
				dist += flag;
				LCD_Clear();/*clear screen to not overwrite on screen*/
				LCD_displayfloat(dist);
				/*display distance on lcd*/
				}
			}
		}
				displacement = GPS_getDistance(long_arr[counter], lat_arr[counter],finalLong,finalLat);
				
				
				//see 
				if(displacement>=100)
				{ 		
					led_off();
			        led_on(0x02);/*turn on red led when distance >100*/   
			    }
				
				if(displacement>=0 && displacement<5)
				{
					led_off();
					led_on(0x08);
					//SysTick_Wait1ms(3000);								/*turn on green led when 0<distance <1*/
				}	
				if(displacement>=5 && displacement<100)
				{
					led_off();
					led_on(0x02);
					led_on(0x08); //turn on yellow led when 1<distance<100
				}	
		    
			if(counter==2){
				counter=0;
				long_arr[0] = long_arr[2];
				lat_arr[0] = lat_arr[2];
			}
			counter++;/*increase counter by 1*/

		
	}
	
}