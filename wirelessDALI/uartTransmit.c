#define F_CPU 16000000
#define USART_BAUDRATE 9600
#define UBRR_VALUE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)
#define VREF 5

#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

volatile uint8_t count = 0;

volatile uint8_t flag = 0;

static uint8_t offbyte1 = 0b11111110;
static uint8_t offbyte2 = 0b00000000;

static uint8_t onbyte1 = 0b11111110;
static uint8_t onbyte2 = 0b11111110;

static uint8_t off1 = 0b01010101;
static uint8_t off2 = 0b01010110;
static uint8_t off3 = 0b10101010;
static uint8_t off4 = 0b10101010;

static uint8_t on1 = 0b01010110;
static uint8_t on2 = 0b01010101;
static uint8_t on3 = 0b01010110;
static uint8_t on4 = 0b01010101;

//Include Libraries
void uart_init(){
	
	UBRR0H |= (unsigned char)((104)>>8); //sets the baud rate to 9600bps
	UBRR0L |= (unsigned char)(104);
	UCSR0B |= (1<<TXEN0); //enables UART transmitter
	UCSR0C |= (1<<UCSZ01)|(1<<UCSZ00); //sets character size to 8 data bits

}

//UART transmitter  function
void uart_transmit(uint8_t myValue){

	while (!((1<<UDRE0)&&(UCSR0A))); //wait until the transmit register is ready
	UDR0 = myValue;//once ready, store next value for transmission
}

void InitADC()
{
	// Select VREF=AVcc
	ADMUX |= (1<<REFS0);
	
	//select ADC channel with safety mask
	ADMUX = (ADMUX & 0xF0);
	
	//set pre-scaler to 128 and enable ADC and enable ADC interrupt
	ADCSRA |= (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0)|(1<<ADEN)|(1<<ADATE)|(1<<ADIE);
	ADCSRB &= ~((1<<ADTS2)|(1<<ADTS1)|(1<<ADTS0));
}

ISR(ADC_vect){
	if(tempCount >= 100){
		tempCount = 0;
	} else if(tempCount == 75) {
		uint16_t val = ADC;
		if(flag) {
			if(count < 15){
				data[count] = val > 0;
				count++;
				} else {
				data[count] = val > 0;
				count = 0;
				for(int i=0;i<16;i++){
					printf("%x ", data[i]);
				}
				printf("\n\r");
				flag = 0;
				ADCSRA &= ~(1<<ADEN);
				EIMSK |= (1 << INT0);
			}
		} else if(val == 0){
			data[count] = val > 0;
			count++;
			flag = 1;
		}
	}
	
	tempCount++;
}


ISR(INT0_vect){
	EIMSK &= ~(1 << INT0);
	if(flag){
// 		uart_transmit(on1);
// 		_delay_ms(50);
// 		uart_transmit(on2);
// 		_delay_ms(50);
// 		uart_transmit(on3);
// 		_delay_ms(50);
// 		uart_transmit(on4);
		uart_transmit(onbyte1);
		_delay_ms(50);
		uart_transmit(onbyte2);
		_delay_ms(50);
	} else {
// 		uart_transmit(on1);
// 		_delay_ms(50);
// 		uart_transmit(on2);
// 		_delay_ms(50);
// 		uart_transmit(on3);
// 		_delay_ms(50);
// 		uart_transmit(on4);
		uart_transmit(offbyte1);
		_delay_ms(50);
		uart_transmit(offbyte2);
		_delay_ms(50);
	}
	flag = !flag;
}

int main()
{
	DDRC &= ~(1 << PINC0);
	
	EICRA |= (1 << ISC01);
	EICRA &= ~(1 << ISC00);
	EIMSK |= (1 << INT0);
	EIFR |= (1 << INTF0);
	sei();
	
	//initialize ADC
	InitADC();
	
	//Initialize USART0
	uart_init();
	
	//ADCSRA |= (1<<ADSC);
	
	while(1)
	{
		_delay_ms(10000);
		if (!(EIMSK & (1 << INT0))){
			EIMSK |= (1 << INT0);	
		}
		//reading potentiometer value and recalculating to Ohms
		//potval = ReadADC(5);
		
		//sending potentiometer value to terminal
		
		//approximate 1s
	}
}