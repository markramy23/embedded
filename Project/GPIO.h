#define GPIO_SW_1 0
#define GPIO_SW_2 1
#define GPIO_SW_NOT_PRESSED 1
#define GPIO_RED_LED 0
#define GPIO_GREEN_LED 1
#define GPIO_BLUE_LED 2


#define GPIO_LED_ON 1
#define GPIO_LED_OFF 0

void GPIO_initPORTA();
void GPIO_initPORTF();
void GPIO_setLEdValue(unsigned char ledColor , unsigned char ledState);
void GPIO_PORTB_setPort(unsigned char sevenSegmentValue);
void GPIO_setSevenSegment(unsigned char valueOfSevenSegment);
