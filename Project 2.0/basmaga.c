//#include "stdint.h"
//#include "math.h"
//#include "tm4c123gh6pm.h"
//#include "stdlib.h"
//#include <stdbool.h>
//#include <stdio.h>
//#include <string.h>
//#define PI 3.14159265
//#define R 6371000

//void SystemInit(){}
//void readGPSModule();
//double Distance;
//double Dis;
//unsigned char c;
//int num[10];
//double lat = 0;
//double lon = 0;
//int flag = 0;
//char longitudeIs[20],latitudeIs[20];
//char dist[20];

//double pastlat=0;
//double pastlon=0;
//// Mili seconds delay function
//void delay_ms(int n)
//{
//    int i,j;
//    for(i=0;i<n;i++)
//        for(j=0;j<3180;j++)
//        {}
//}

//// Micro seconds delay function
//void delay_us(int n)
//{
//    int i,j;
//    for(i=0;i<n;i++)
//        for(j=0;j<3;j++)
//        {}
//}


//void LCD_command(unsigned char com)
//{

//    GPIO_PORTA_DATA_R=0;        //Rs=0 send command, Rw=0 write, E=0 initially enable=0
//    delay_ms(1);
//    GPIO_PORTA_DATA_R |=0x40;   // E=1 (low to high) without affecting other pins
//    delay_ms(1);
//    GPIO_PORTB_DATA_R=com;      // send command to the screen
//    delay_us(1);
//    GPIO_PORTA_DATA_R=0;        // E=0 agian
//    delay_us(1);

//}



//void LCD_char(unsigned char data)
//{
//    GPIO_PORTA_DATA_R=0x80;        //Rs=1 send data, Rw=0 write,E=0 initially enable=0
//    delay_ms(1);
//    GPIO_PORTA_DATA_R |=0x40;      // E=1 (low to high) without affecting other pins
//    delay_ms(1);
//    GPIO_PORTB_DATA_R=data;        // send data to the screen
//    delay_us(1);
//    GPIO_PORTA_DATA_R=0;           //Rs=0 Rw=0 E=0(back to the beginning)
//    delay_us(50);
//}


//void LCD_string(char *str) // to print string (pointer to elements of array of char)
//{
//    int i;
//    for(i=0;str[i]!=0;i++)  /* Send each char of string  */
//    {
//        LCD_char(str[i]);  /* Call LCD data write */
//        delay_ms(20);
//    }
//}

//void LCD_integers(int data)
//{
//    int p;
//    int k=0;
//    while(data>0)
//    {
//        num[k]=data%10;
//        data=data/10;
//        k++;
//    }
//    k--;
//    for (p=k;p>=0;p--)
//    {
//        c=num[p]+48;

//        GPIO_PORTA_DATA_R=0x80;       //Rs=1 send data, Rw=0 write,E=0 initially enable=0
//        delay_ms(1);
//        GPIO_PORTA_DATA_R |=0x40;      // E=1 (low to high) without affecting other pins
//        delay_ms(1);
//        GPIO_PORTB_DATA_R=c;           // send data to the screen
//        delay_us(1);
//        GPIO_PORTA_DATA_R=0;           //Rs=0 Rw=0 E=0(back to the beginning)
//        delay_us(50);
//    }
//}


//void LCD_separating_double(double deci)
//{
//    int integer_part=deci;
//    int decimal_part=(deci - integer_part)*100000;

//    LCD_integers(integer_part);
//    LCD_char('.');
//    LCD_integers(decimal_part);
//}


//// To convert from degrees to radian
//float deg_to_rad(float deg){
//    return (deg * PI / 180);
//}

//// Measuring the distance by longitude and latitude
//float Total_Distance(float long1, float long2, float lat1, float lat2)
//{
//    float dlong = deg_to_rad(long2 - long1);
//    float dlat  = deg_to_rad(lat2 - lat1);
//    float phi1 = deg_to_rad(lat1);
//    float phi2 = deg_to_rad(lat2);
//    // Haversine formula
//    float a = pow(sin((0.5 * dlat)), 2) + cos(phi1) * cos(phi2) * pow(sin((0.5 * dlong)), 2);
//    float d = 2 * R * asin(sqrt(a));
//    Dis = Dis + d;
//    return Dis;
//}


//void init()
//{
//    volatile uint32_t delay;

//    SYSCTL_RCGCGPIO_R |=0x03;    //clock register for ports A B
//    delay=1;

//    GPIO_PORTA_LOCK_R=0x4C4F434B;   //lock for ports A B
//    GPIO_PORTA_CR_R=0xE0;
//    //GPIO_PORTB_LOCK_R=0x4C4F434B;
//    GPIO_PORTB_CR_R=0xFF;


//    GPIO_PORTA_AFSEL_R=0;       //initializing portA  5 6 7
//    GPIO_PORTA_PCTL_R=0;
//    GPIO_PORTA_AMSEL_R=0;
//    GPIO_PORTA_DIR_R|=0xE0;
//    GPIO_PORTA_DEN_R|=0xE0;
//    GPIO_PORTA_PUR_R=0;
//    GPIO_PORTB_AFSEL_R=0;       //initializing portB pins 0-8 for LCD
//    GPIO_PORTB_PCTL_R=0;
//    GPIO_PORTB_AMSEL_R=0;
//    GPIO_PORTB_DIR_R=0xFF;
//    GPIO_PORTB_DEN_R=0xFF;
//    GPIO_PORTB_PUR_R=0;




//        SYSCTL_RCGCUART_R |=0x0020;    // ACTIVATE UART5
//        SYSCTL_RCGCGPIO_R |=0x0010;    // enable the clock of port E4,5

//        UART5_CTL_R &=~(0x01);
//        UART5_IBRD_R =104;
//        UART5_FBRD_R =11;
//        UART5_LCRH_R |=0x0070;
//        UART5_CTL_R |=0X0301;

//        GPIO_PORTE_DEN_R |=0x0030;      // D    E4,5
//        GPIO_PORTE_AMSEL_R &=~0x0030;       //  D
//        GPIO_PORTE_AFSEL_R |=0x0030;
//      GPIO_PORTE_PCTL_R =(GPIO_PORTE_PCTL_R&0xFF00FFFF)+0x00110000;           // working I/O ports OR UART


//    delay_ms(20);
//    LCD_command(0x38);
//    delay_us(50);
//    LCD_command(0x0F);
//    delay_ms(50);
//    LCD_command(0x06);
//    delay_ms(50);
//    LCD_command(0x01);
//    delay_ms(25);


//    SYSCTL_RCGCGPIO_R |=0x20;    // enable the clock of port
//    delay=1;
//    GPIO_PORTF_LOCK_R=0x4C4F434B;
//    GPIO_PORTF_CR_R=0x1F;        // enable pins
//    GPIO_PORTF_DIR_R=0x0E;      // pin I/O  0/1
//    GPIO_PORTF_DEN_R=0x1F;      // D
//    GPIO_PORTF_AMSEL_R =0;      // A
//    GPIO_PORTF_AFSEL_R =0;
//    GPIO_PORTF_PCTL_R =0;       // working I/O ports OR UART
//    GPIO_PORTF_PUR_R=0x11;      // for switches
//}

//int main()
//{
//    //SCB->CPACR |= ((3UL << 10*2) | (3UL << 11*2) );

//    init();








//    while(1)
//    {

//        if(Distance<100){

//        readGPSModule();



//                                              //LCD_command(0xC0);
//                                             // LCD_string(latitudeIs);
//                                              if(flag == 0) flag = 1;
//                                              else

//                                              Distance += Total_Distance(pastlon, lon, pastlat,lat);
//                                              pastlat = lat;
//                                              pastlon = lon;
//                                              sprintf(dist,"%f",Distance);
//                                              LCD_command(0x01);
//                                              LCD_string(dist);

//                                              delay_ms(1000);
//        }


//        if (Distance>=100)
//                    {
//                        GPIO_PORTF_DATA_R=0x02;
//                    }
//                    else
//                    {
//                        GPIO_PORTF_DATA_R=0;
//                    }

//    }

//}
//void readGPSModule(){
//    int check = 0;
//    char GPS_values[100],*token,parseValue[12][20];
//    double latitude=0.0,longitude=0.0,seconds=0.0,minutes=0.0,Lat=0.0,Lon=0.0;
//    char m0,m1,m2,m3,m4,m5,m6,m7;
//    const char comma[2] = ",";



//    int index=0,degrees;
//    start:
//    //is the incoming data is $GPRMC?
//    while((UART5_FR_R&0x0010) !=0);
//    m0 = UART5_DR_R&0xFF;
//    if(m0 =='$'){
//        while((UART5_FR_R&0x0010) !=0);
//        m1 = UART5_DR_R &0xFF;
//        if(m1 == 'G'){
//            while((UART5_FR_R&0x0010) !=0);
//            m2 = UART5_DR_R&0xFF;
//            if(m2 == 'P'){
//                while((UART5_FR_R&0x0010) !=0);
//                m3 = UART5_DR_R&0xFF;
//                if(m3 == 'R'){
//                    while((UART5_FR_R&0x0010) !=0);
//                    m4 = UART5_DR_R&0xFF;
//                    if(m4 == 'M'){
//                        while((UART5_FR_R&0x0010) !=0);
//                        m5 = UART5_DR_R&0xFF;
//                        if(m5 == 'C'){
//                            while((UART5_FR_R&0x0010) !=0);
//                            m6 = UART5_DR_R&0xFF;

//                            if(m6 == ','){
//                                while((UART5_FR_R&0x0010) !=0);
//                                m7 = UART5_DR_R&0xFF;

//                                //assign the data to the GPSValues array. read up to the last data checksum (like checksum: A*60)
//                                while(m7 !='*'){
//                                    GPS_values[index] = m7; //20
//                                    while((UART5_FR_R&0x0010) !=0);
//                                    m7 = UART5_DR_R&0xFF;
//                                    index++;
//                                }
//                                //Separating the data in the GPSValues array by comma
//                                index = 0;
//                                token = strtok(GPS_values,comma);
//                                while( token != NULL ) {
//                                    strcpy(parseValue[index], token);
//                                    token = strtok(NULL,comma);
//                                    index++;}

//                                //Data valid if parseValue[1] = A - not valid if =V
//                                if(strcmp(parseValue[1],"A")==0){
//                                    check = 1;
//                                    latitude=atof(parseValue[2]);
//                                    longitude=atof(parseValue[4]);

//                                    //latitude calculation
//                                    degrees=latitude/100;
//                                    minutes=latitude-(double)(degrees*100);
//                                    seconds=minutes/60.00;
//                                    Lat=degrees+seconds;

//                                    sprintf(latitudeIs,"%f", Lat);
//                                    //print("%s\n",latitudeIs);

//                                    //longitude calculation
//                                    degrees=longitude/100;
//                                    minutes=longitude-(double)(degrees*100);
//                                    seconds=minutes/60.00;
//                                    Lon=degrees+seconds;

//                                    sprintf(longitudeIs,"%f",Lon);

//                                    lat = Lat;
//                                    lon = Lon;


//                                    printf("lat = %s\n",latitudeIs);
//                                    printf("lon = %s\n\n\n",longitudeIs);
//                                   // LCD_command(0x01);
//                                   // LCD_string(longitudeIs);



//                                }


//                                else{

//                                    printf("\Error\n");

//                                }
//                            }
//                        }
//                    }
//                }
//            }
//        }
//    }
//    if (check == 0) { goto start; }
//}
