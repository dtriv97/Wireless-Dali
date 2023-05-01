/*
 //* wirelessDALI.c
 //*
 //* Created: 3/09/2019 10:50:11 AM
 //* Author : Dhairya Trivedi & Thanushan Thanababu
 //
*/

#define F_CPU 16000000
#define SDI PORTB3
#define SDO PORTB4
#define SCK PORTB5
#define CSCON PORTD1
#define CSDAT PORTD0

#define RESET_PIN PORTD4

#define IRQ1 PIND3

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

volatile RX_RECIEVED = 0;

//Setting up GCON register for RX/TX mode---------------------------------------------
uint8_t GCON_REG_ADDR = 0b00000000;

uint8_t GCON_REG_TX_VAL = 0b10001000;
uint8_t GCON_REG_RX_VAL = 0b01101000;
uint8_t GCON_REG_SLEEP_VAL = 0b00001000;
uint8_t GCON_REG_STDBY_VAL = 0b00101000;
uint8_t GCON_REG_FREQ_VAL = 0b01001000;

//Setting up registers for Buffered RX/TX mode---------------------------------------------
uint8_t DMOD_REG_ADDR = 0b00000010; //for buffered mode
uint8_t DMOD_REG_VAL = 0b10101000;

uint8_t FIFO_C_REG_ADDR = 0b00001010;
uint8_t FIFO_C_REG_VAL = 0b11000001;

uint8_t FTXRX_I_REG_ADDR = 0b00011010;
uint8_t FTXRX_I_REG_VAL = 0b10111000;	//need to OR with current value of this register

uint8_t FTPR_I_REG_ADDR = 0b00011100;
uint8_t FTPR_I_REG_VAL = 0b00010001;

uint8_t FIFO_CONF_REG_ADDR = 0b00111110;
uint8_t FIFO_CONF_REG_VAL = 0b01000000;

uint8_t SYNC_REG_ADDR = 0b00100100;
uint8_t SYNC_REG_VAL = 0b00111000;

uint8_t SYNC_V07_REG_ADDR = 0b00110010;
uint8_t SYNC_V07_REG_VAL = 0b11111111;

uint8_t SYNC_V31_REG_ADDR = 0b00101100;
uint8_t SYNC_V31_REG_VAL = 0b11111111;

void resetChip(){
	PORTD |= (1 << RESET_PIN);
	_delay_us(100);
	PORTD &= ~(1 << RESET_PIN);
}

void SPI_MasterInit(void)
{
	/* Set MOSI and SCK output, all others input */
	DDRB = ((1 << DDB3) | (1 << DDB5));
	/* Enable SPI, Master, set clock rate f_Clk/16 */
	SPCR0 = (1 << SPE) | (1 << MSTR) | (1 << SPR0);
}

void SPI_MasterTransmit(uint8_t cData)
{
	/* Start transmission */
	SPDR0 = cData;
	/* Wait for transmission complete */
	while (!(SPSR0 & (1 << SPIF)));
}

void SPI_MasterRead() {
	return SPI_MasterTransmit(0x00);
}

void setupSleepMode(){
	PORTD &= ~(1 << CSCON);
	SPI_MasterTransmit(GCON_REG_ADDR);
	SPI_MasterTransmit(GCON_REG_SLEEP_VAL);
	PORTD |= (1 << CSCON);
}

void setupTransmitMode(){
	PORTD &= ~(1 << CSCON);
	SPI_MasterTransmit(GCON_REG_ADDR);
	SPI_MasterTransmit(GCON_REG_TX_VAL);
	PORTD |= (1 << CSCON);
}

void setupRecieveMode() {
	PORTD &= ~(1 << CSCON);
	SPI_MasterTransmit(GCON_REG_ADDR);
	SPI_MasterTransmit(GCON_REG_RX_VAL);
	PORTD |= (1 << CSCON);
}

void setupStandbyMode(){
	PORTD &= ~(1 << CSCON);
	SPI_MasterTransmit(GCON_REG_ADDR);
	SPI_MasterTransmit(GCON_REG_STDBY_VAL);
	PORTD |= (1 << CSCON);
}

void setupFreqMode(){
	PORTD &= ~(1 << CSCON);
	SPI_MasterTransmit(GCON_REG_ADDR);
	SPI_MasterTransmit(GCON_REG_FREQ_VAL);
	PORTD |= (1 << CSCON);
}

void setupBufferedMode(){
	PORTD &= ~(1 << CSCON);
	
	SPI_MasterTransmit(DMOD_REG_ADDR);
	SPI_MasterTransmit(DMOD_REG_VAL);
	
	SPI_MasterTransmit(FIFO_C_REG_ADDR);
	SPI_MasterTransmit(FIFO_C_REG_VAL);
	
	SPI_MasterTransmit(FTXRX_I_REG_ADDR);
	SPI_MasterTransmit(FTXRX_I_REG_VAL);
	
	SPI_MasterTransmit(FTPR_I_REG_ADDR);
	SPI_MasterTransmit(FTPR_I_REG_VAL);
	
	SPI_MasterTransmit(SYNC_REG_ADDR);
	SPI_MasterTransmit(SYNC_REG_VAL);
	
	SPI_MasterTransmit(SYNC_V07_REG_ADDR);
	SPI_MasterTransmit(SYNC_V07_REG_VAL);
	
	SPI_MasterTransmit(SYNC_V31_REG_ADDR);
	SPI_MasterTransmit(SYNC_V31_REG_VAL);
	
	SPI_MasterTransmit(FIFO_CONF_REG_ADDR);
	SPI_MasterTransmit(FIFO_CONF_REG_VAL);
	
	PORTD |= (1 << CSCON);
}

void readGCONReg(){
	PORTD &= ~(1 << CSCON);
	SPI_MasterTransmit(0b01000000 | FIFO_C_REG_ADDR);
	SPI_MasterTransmit(0b10000000);
	PORTD |= (1 << CSCON);
}

void sendData(uint8_t data){
	PORTD &= ~(1 << CSDAT);
	SPI_MasterTransmit(data);
	PORTD |= (1 << CSDAT);
}

ISR(INT1_vect){
	RX_RECIEVED = 1;
}

int main(void)
{	
	DDRD |= (1 << CSDAT);
	PORTD |= (1 << CSDAT);
	
	DDRD |= ((1 << CSCON) | (1 << RESET_PIN));
	PORTD |= (1 << CSCON);
	PORTD &= ~(1 << RESET_PIN);
	
	DDRD &= ~(1 << DDD3);
	
	EICRA |= (1 << ISC10);
	EIMSK |= (1 << INT1);
	EIFR |= (1 << INTF1);
	sei();
	
	SPI_MasterInit();
	resetChip();
	
	_delay_ms(10);
	
	setupStandbyMode();
	_delay_us(100);

	setupBufferedMode();
	_delay_ms(2);	
	
	while (1)
    {
 		setupRecieveMode();
		_delay_ms(1);
		
		while(!RX_RECIEVED);
		RX_RECIEVED = 0;
		
		setupStandbyMode();
		_delay_ms(1);

		PORTD &= ~(1 << CSDAT);
		SPI_MasterTransmit(0b00000000);
		PORTD |= (1 << CSDAT);
		_delay_ms(2);
		
		setupSleepMode();
		_delay_ms(50);
		readGCONReg();
		_delay_ms(100);
	}
}
