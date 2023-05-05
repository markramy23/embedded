

void UART_Init();	
char rece(void);
void GPS_read();
void GPS_format();
float toDegree(float angle);
float toRad(float angle);
float GPS_getDistance(float currentLong, float currentLat , float destLong,float destLat);