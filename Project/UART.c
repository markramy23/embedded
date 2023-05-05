#include "tm4c123gh6pm.h"
#include "bit_utilies.h"
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include "UART.h"
void UART_Init(){
	SET_BIT(SYSCTL_RCGCUART_R,1);
	SET_BIT(SYSCTL_RCGCGPIO_R,1);
	CLR_BIT(UART1_CTL_R,0);
	SET(UART1_IBRD_R,104);
	SET(UART1_FBRD_R,11);
	SET(UART1_LCRH_R,0x0070);
	SET(UART1_CTL_R,0x0301);
	SET_BITS(GPIO_PORTB_AFSEL_R,0x03);
	SET_BITS(GPIO_PORTB_PCTL_R,0x00000011);
	SET_BITS(GPIO_PORTB_DEN_R,0x03);
	CLR(GPIO_PORTB_AMSEL_R,0x03);
}
void send(char c)
{
while ((UART1_FR_R & 0x20) != 0)
;
UART1_DR_R = c;
}

char rece(void)
{
while ((UART1_FR_R & 0x10) != 0)
;
return (char)(UART1_DR_R & 0xFF);
}

char GPS[80];
char GPS_logname[]="$GPRMC,";
char GPS_formated[12][20];
char*token;
float currentLong, currentLat , speed, finalLat=1052.563787;
const double EARTH_RADIUS = 6371000;
const double PI=3.14159265359;


void GPS_read(){



char recievedChar;
//check for correct log
	
char flag=1;
	char i;
do{
	flag=1;
	
	for( i=0;i<7;i++){
		if(rece() != GPS_logname[i]){
			flag=0;
			break;}
		}
	}while(flag==0);

	//store the log
	do{
		char fillGPScounter = 0;
		recievedChar= rece();
		GPS[fillGPScounter++]=recievedChar;
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
	float currentLongRad = toRad(toDegree(currentLong));
	float currentLatRad = toRad(toDegree(currentLat));
	float destLongRad = toRad(toDegree(destLong));
	float destLatRad = toRad(toDegree(destLat));
	
	float longDiff = destLongRad - currentLongRad;
	float latDiff = destLatRad - currentLatRad ;
		
	float a = pow(sin(latDiff/2),2)+cos(currentLatRad)*cos(destLatRad)*pow(sin(longDiff/2),2);
	double c = 2*atan2(sqrt(a),sqrt(1-a));
	return EARTH_RADIUS*c;
	
	}
	


