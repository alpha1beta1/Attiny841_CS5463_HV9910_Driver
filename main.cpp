/*
 * Attiny841_CS5463_HV9910B_05022019.cpp
 *
 * Created: 2/6/2019 9:47:25 AM
 * Author : Aykan ANKARA
 */ 

#define F_CPU 8000000UL


#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>



/************************************************************************/
/*			ATTINY841 Baudrate and UBRR Definition                      */
/************************************************************************/
#define BAUD 9600
#define BRC ((F_CPU/16/BAUD)-1)



/************************************************************************/
/*			CS5463 Software SPI Definition                               */
/************************************************************************/

//CS
#define ss_cs5463		PORTA7
//CLK
#define clk_cs5463		PORTA4
//CS5460 Data Out (DO)
#define do_cs5463		PORTA6
//CS5460 Data In (DI)
#define di_cs5463		PORTA5
//CS5460 Reset
#define rst_cs5463		PORTA0
//Clock Delay for CS5463
#define clk_delay_cs5463			_delay_us(0.4)
#define cs_read_start_bit_delay		_delay_us(0.03)
#define clk_delay					_delay_us(1)



/************************************************************************/
/*			HV9910B Software PWM Configuration                          */
/************************************************************************/

//HV9910D PWM Out
#define pwm_hv9910_on		PORTB |= (1<<PORTB1)
#define pwm_hv9910_off		PORTB &= ~(1<<PORTB1)





/************************************************************************/
/*					CS5463 Maximum Current Value for HV9910B            */
/************************************************************************/
#define FIFTY_PERC_DUTY				8000	//The maximum duty cycle must be restricted to less
//than 50% (16000 meets 650 Hz) in order to prevent sub-harmonic oscillations
//and open loop instability.
#define TWO_UP_TWENTY_FOUR			16777216	// 2^24 (24-bit CS5463 current values)
#define MAX_VAL						16000000	//measured max value with 10K and 301 Ohm Parallel resistance for Iin+
#define MIN_VAL						200000		// measured min value with 10K and 301 Ohm Parallel resistance  Iin+
#define ACT_STEP_VALUE_HV9910		round((MAX_VAL - MIN_VAL) / FIFTY_PERC_DUTY) //Determining actual interval for CS5463 at max 50% PWM duty cycle
#define STEPS_H9910					round(TWO_UP_TWENTY_FOUR / FIFTY_PERC_DUTY)		//Determining interval for CS5463 at max 50%  PWM duty cycle




/************************************************************************/
/*                   CS5463 Maximum Current Value for MAX5483           */
/************************************************************************/
#define LMV							78			//Least Meaningful Value for MAX5483 at 7.04 VDC
#define MMV							100			//Most Meaningful Value for MAX5483
#define TWO_UP_TWENTY_FOUR			16777216	// 2^24
#define DIFFERENCE					round(TWO_UP_TWENTY_FOUR / (LMV))	//Determining Interval for CS5463



enum CS5463_register_t{
	//Register Page 0 (CS5463)
	CONFIG = 0,
	CONTROL = 28,
	CURRENT_RMS = 11,
	CURRENT_GAIN = 2,
	CYCLE_COUNT = 5,
	EPSILON = 13,
	INST_CURR = 7,
	MASK_INTERRUPT = 26,
	MODE = 18,
	POWER = 9,
	STATUS = 15,
	TEMPERATURE = 2,
	CURRENT_AC_OFFSET = 16,
	VOLTAGE_AC_OFFSET = 17,
	VOLTAGE_RMS = 12,
	
	//COMMANDS (CS5463)
	Read =	0b00000000,
	Write =	0b01000000,
	START_CONTINUOUS = 0xE8,
	SYNC0 = 0b11111110, //SYNC 0 Command: Last char of a serial port re-initialization sequence
	SYNC1 = 0b11111111, //SYNC 1 Command: Used during reads and serial port initialization.
	CALIBRATION_Cur_Volt_DC_OFFSET = 0b11011001,
	CALIBRATION_Cur_Volt_DC_GAIN = 0b11011010,
	CALIBRATION_Cur_Volt_AC_GAIN = 0b11011110,
	CALIBRATION_Cur_Volt_AC_OFFSET = 0b11011101,
};


uint8_t spiSend[3]; //Used for allocation of Max5483 meaningful bits
uint8_t control = 0x80; //Used for writing each bit
uint16_t value_Map_CS5463_Max5483; //Mapping value from CS5463 to MAX5483
uint16_t value_Map_CS5463_HV9910;	//Mapping value from CS5463 to HV9910
uint32_t value_CONFIG;	//Configuration Register value of CS5463 (Register Address Bits: "00000")
uint32_t value_STATUS;	//Status Register value of CS5463 (Register Address Bits: "01111")
uint32_t value_CURRENT_RMS; //Current Register value of CS5463 (Register Address Bits: "01011")
uint32_t value_CURRENT_GAIN; //Current Gain Register value of CS5463 (Register Address Bits: "00010")
uint32_t value_CURRENT_AC_OFFSET; //Current AC Offset Register value of CS5463
uint32_t value_CYCLE_COUNT; //cycle Count Register value of CS5463 ()
uint32_t value_INST_CURR; //Instantaneous Current value of CS5463
uint32_t value_EPSILON; // Epsilon Register value of CS5463


ISR(TIMER1_COMPA_vect)
{
	pwm_hv9910_on;
}

ISR(TIMER1_COMPB_vect)
{
	pwm_hv9910_off;
}



void Cs5463_SetUp();
void SPI_Cs5463_Write(uint8_t);
void SPISetUpSoftware();
uint32_t SPI_CS5463_Read_Write(uint8_t);
uint16_t Map_CS5463_HV9910B_Values(uint32_t);
void USART_Init();
unsigned char USART_Receive();
void USART_Send(uint8_t);
void USART_PutValue(uint32_t);
void PWM_SetUp();
void PWM_Write(uint16_t freq, uint16_t duty);


int main(void)
{
   
	USART_Init();		//Configure all required registers to enable USART communication
	SPISetUpSoftware();	//Set all required Data Direction Registers of Attiny44A
	Cs5463_SetUp();		//Set initial value of all selected Registers of CS5463
	PWM_SetUp();		//Configure all required registers to enable PWM communincation
	
	
	while (1) 
    {
		
		do
		{
			value_STATUS = SPI_CS5463_Read_Write(STATUS<<1); //Read STATUS Register of CS5463 and send this value to Attiny44A
			_delay_ms(100);
		} while (!(value_STATUS & 0x800000));
		
		
		
		
		value_CONFIG = SPI_CS5463_Read_Write(CONFIG<<1); //Read Configuration Register default value which must be 0x000001
		_delay_ms(100);
		
		
		value_CYCLE_COUNT = SPI_CS5463_Read_Write(CYCLE_COUNT<<1); //Read Counter Register default value which must be 0x000FA0
		_delay_ms(100);
		
		
		value_CURRENT_RMS = SPI_CS5463_Read_Write(Read | CURRENT_RMS<<1); //Read CUURENT_RMS Register of CS5463 and send this value to Attiny44A
		_delay_ms(100);
		
		
		value_CURRENT_GAIN = SPI_CS5463_Read_Write(CURRENT_GAIN<<1); //Read CUURENT_GAIN Register of CS5463 and send this value to Attiny44A
		_delay_ms(100);
		
		value_CURRENT_AC_OFFSET = SPI_CS5463_Read_Write(CURRENT_AC_OFFSET<<1); //Read CUURENT_GAIN Register of CS5463 and send this value to Attiny44A
		_delay_ms(100);
		
		value_INST_CURR = SPI_CS5463_Read_Write(INST_CURR<<1); //Read CUURENT_GAIN Register of CS5463 and send this value to Attiny44A
		_delay_ms(100);
		
		value_Map_CS5463_HV9910 = Map_CS5463_HV9910B_Values(value_CURRENT_RMS);
		
		PWM_Write(16000, value_Map_CS5463_HV9910);
		
		USART_PutValue(value_CONFIG);
		USART_PutValue(0x0A);
		_delay_ms(500);
		//USART_PutValue(value_STATUS);
		//_delay_ms(1000);
		//USART_PutValue(value_CYCLE_COUNT);
		//_delay_ms(1000);
		//USART_PutValue(value_CURRENT_GAIN);
		//_delay_ms(1000);
		//USART_PutValue(value_CURRENT_AC_OFFSET);
		//_delay_ms(1000);
		//USART_PutValue(value_INST_CURR);
		//_delay_ms(1000);
		USART_PutValue(value_CURRENT_RMS);
		USART_PutValue(0x0A);
		_delay_ms(500);
		//USART_PutValue(value_Map_CS5463_HV9910);
		//_delay_ms(1000);
		
		
    }
}




void Cs5463_SetUp()
{
	// Sync Commands
	PORTA &= ~(1<<rst_cs5463); // Set Low CS5460 RESET Pin Out
	_delay_us(0.1);
	PORTA |= (1<<rst_cs5463); // Set high CS5460 RESET Pin Out
	_delay_us(0.1);
	
	
	/************************************************************************/
	/*			Start Sending Sync Commands (IMPORTANT!!!)                  */
	/************************************************************************/
	clk_delay_cs5463;
	
	PORTA &= ~(1<<clk_cs5463);
	PORTA &= ~(1<<do_cs5463);
	
	SPI_Cs5463_Write(SYNC1);
	SPI_Cs5463_Write(SYNC1);
	SPI_Cs5463_Write(SYNC1);
	SPI_Cs5463_Write(SYNC0);
	
	PORTA &= ~(1<<clk_cs5463);
	PORTA &= ~(1<<do_cs5463);
	
	
	clk_delay_cs5463;
	clk_delay;
	
		
	
	/************************************************************************/
	/*			Setting Config Register										*/
	/************************************************************************/
	clk_delay_cs5463;
	
	PORTA &= ~(1<<clk_cs5463);
	PORTA &= ~(1<<do_cs5463);
	
	SPI_Cs5463_Write(Write | CONFIG<<1);
	SPI_Cs5463_Write(0x00);
	SPI_Cs5463_Write(0x00);
	SPI_Cs5463_Write(0x02);
	
	PORTA &= ~(1<<clk_cs5463);
	PORTA &= ~(1<<do_cs5463);
	
	
	clk_delay_cs5463;
	clk_delay;
	
	
	
	/************************************************************************/
	/*			Start Setting Mode Register                                 */
	/************************************************************************/
	clk_delay_cs5463;
	
	PORTA &= ~(1<<clk_cs5463);
	PORTA &= ~(1<<do_cs5463);
	
	SPI_Cs5463_Write(Write | MODE<<1); 
	SPI_Cs5463_Write(0x00); //Sets All Pass and activate Automatic Frequency generation
	SPI_Cs5463_Write(0x00);
	SPI_Cs5463_Write(0x00);
	
	PORTA &= ~(1<<clk_cs5463);
	PORTA &= ~(1<<do_cs5463);
	
	
	clk_delay_cs5463;
	clk_delay;
	
	
	
	/************************************************************************/
	/*			Setting Interrupt Mask Register                             */
	/************************************************************************/
	clk_delay_cs5463;
	
	PORTA &= ~(1<<clk_cs5463);
	PORTA &= ~(1<<do_cs5463);
	
	SPI_Cs5463_Write(Write | MASK_INTERRUPT<<1);
	SPI_Cs5463_Write(0x00); 
	SPI_Cs5463_Write(0x00);
	SPI_Cs5463_Write(0x00);
	
	PORTA &= ~(1<<clk_cs5463);
	PORTA &= ~(1<<do_cs5463);
	
	
	clk_delay_cs5463;
	clk_delay;
	
	
	/************************************************************************/
	/*			Setting AC Offset Registers                                */
	/************************************************************************/
	
	// Voltage AC Offset
	clk_delay_cs5463;
	
	PORTA &= ~(1<<clk_cs5463);
	PORTA &= ~(1<<do_cs5463);
	
	SPI_Cs5463_Write(Write | VOLTAGE_AC_OFFSET<<1);
	SPI_Cs5463_Write(0x00);
	SPI_Cs5463_Write(0x00);
	SPI_Cs5463_Write(0x00);
	
	PORTA &= ~(1<<clk_cs5463);
	PORTA &= ~(1<<do_cs5463);
	
	
	clk_delay_cs5463;
	clk_delay;
	
	
	// Current AC Offset
	clk_delay_cs5463;
	
	PORTA &= ~(1<<clk_cs5463);
	PORTA &= ~(1<<do_cs5463);
	
	SPI_Cs5463_Write(Write | CURRENT_AC_OFFSET<<1);
	SPI_Cs5463_Write(0x00);
	SPI_Cs5463_Write(0x00);
	SPI_Cs5463_Write(0x00);
	
	PORTA &= ~(1<<clk_cs5463);
	PORTA &= ~(1<<do_cs5463);
	
	
	clk_delay_cs5463;
	clk_delay;
	
	
	
	/************************************************************************/
	/*        Voltage & Current DC & AC Offset Calibration                  */
	/************************************************************************/
	clk_delay_cs5463;
	
	PORTA &= ~(1<<clk_cs5463);
	PORTA &= ~(1<<do_cs5463);
	
	SPI_Cs5463_Write(CALIBRATION_Cur_Volt_DC_OFFSET);
	SPI_Cs5463_Write(SYNC0);
	SPI_Cs5463_Write(SYNC0);
	SPI_Cs5463_Write(SYNC0);
	
	PORTA &= ~(1<<clk_cs5463);
	PORTA &= ~(1<<do_cs5463);
	
	
	clk_delay_cs5463;
	clk_delay;
	
	/****************************************************************************/
	
	clk_delay_cs5463;
	
	PORTA &= ~(1<<clk_cs5463);
	PORTA &= ~(1<<do_cs5463);
	
	SPI_Cs5463_Write(CALIBRATION_Cur_Volt_AC_OFFSET);
	SPI_Cs5463_Write(SYNC0);
	SPI_Cs5463_Write(SYNC0);
	SPI_Cs5463_Write(SYNC0);
	
	PORTA &= ~(1<<clk_cs5463);
	PORTA &= ~(1<<do_cs5463);
		
	clk_delay_cs5463;
	clk_delay;
		
	
	/************************************************************************/
	/*			Voltage & Current DC & AC Gain Calibration                  */
	/************************************************************************/
	clk_delay_cs5463;
	
	PORTA &= ~(1<<clk_cs5463);
	PORTA &= ~(1<<do_cs5463);
	
	SPI_Cs5463_Write(CALIBRATION_Cur_Volt_DC_GAIN);
	SPI_Cs5463_Write(SYNC0);
	SPI_Cs5463_Write(SYNC0);
	SPI_Cs5463_Write(SYNC0);
	
	PORTA &= ~(1<<clk_cs5463);
	PORTA &= ~(1<<do_cs5463);
		
	clk_delay_cs5463;
	clk_delay;
		
	
	/****************************************************************************/
	
	clk_delay_cs5463;
	
	PORTA &= ~(1<<clk_cs5463);
	PORTA &= ~(1<<do_cs5463);
	
	SPI_Cs5463_Write(CALIBRATION_Cur_Volt_AC_GAIN);
	SPI_Cs5463_Write(SYNC0);
	SPI_Cs5463_Write(SYNC0);
	SPI_Cs5463_Write(SYNC0);
	
	PORTA &= ~(1<<clk_cs5463);
	PORTA &= ~(1<<do_cs5463);
	
	
	clk_delay_cs5463;
	clk_delay;
	
	
	
	
	
	
	/************************************************************************/
	/*			Setting Cycle Count Register								*/
	/************************************************************************/
	clk_delay_cs5463;
	
	PORTA &= ~(1<<clk_cs5463);
	PORTA &= ~(1<<do_cs5463);
	
	
	SPI_Cs5463_Write(Write | CYCLE_COUNT<<1); //0x000FA0 -> 4000
	SPI_Cs5463_Write(0x00); 
	SPI_Cs5463_Write(0x0F);
	SPI_Cs5463_Write(0xA0);
	
	PORTA &= ~(1<<clk_cs5463);
	PORTA &= ~(1<<do_cs5463);
	
	
	clk_delay_cs5463;
	clk_delay;
	
	
	/************************************************************************/
	/*			Setting Control Register                                    */
	/************************************************************************/
	clk_delay_cs5463;
	
	PORTA &= ~(1<<clk_cs5463);
	PORTA &= ~(1<<do_cs5463);
	
	
	SPI_Cs5463_Write(Write | CONTROL<<1); 
	SPI_Cs5463_Write(0x00);
	SPI_Cs5463_Write(0x00);
	SPI_Cs5463_Write(0x04); 
	
	PORTA &= ~(1<<clk_cs5463);
	PORTA &= ~(1<<do_cs5463);
	
	
	clk_delay_cs5463;
	clk_delay;
	
	/************************************************************************/
	/*			Start Continuous                                            */
	/************************************************************************/
	clk_delay_cs5463;
	
	PORTA &= ~(1<<clk_cs5463);
	PORTA &= ~(1<<do_cs5463);
	
	SPI_Cs5463_Write(START_CONTINUOUS);
	SPI_Cs5463_Write(SYNC0);
	SPI_Cs5463_Write(SYNC0);
	SPI_Cs5463_Write(SYNC0);
	
	
	PORTA &= ~(1<<clk_cs5463);
	PORTA &= ~(1<<do_cs5463);
	
	
	clk_delay_cs5463;
	clk_delay;
	
}




void SPI_Cs5463_Write(uint8_t data)
{
	for(uint8_t i=0;i<8;i++)
	{
		if((data & control) == control)
		PORTA |= (1<<do_cs5463);
		else
		PORTA &= ~(1<<do_cs5463);
		
		
		control >>= 1;
		
		PORTA |= (1<<clk_cs5463); //Clock Set High
		clk_delay_cs5463;
		
		PORTA &= ~(1<<clk_cs5463); //Clock Set Low
		clk_delay_cs5463;
	}
	
	
	control = 0x80;
}



uint32_t SPI_CS5463_Read_Write(uint8_t data)
{
	
	uint32_t read_data = 0x00;
	
	for(uint8_t i=0;i<8;i++)  //Send Command Byte (8 bit) to CS5463
	{
		if((data & control) == control)
		PORTA |= (1<<do_cs5463);
		else
		PORTA &= ~(1<<do_cs5463);
		control >>= 1;
		
		PORTA |= (1<<clk_cs5463); //Clock Set High
		clk_delay_cs5463;
		
		read_data <<=1;
		if(bit_is_set(PINA, PINA5)) //Read each bit coming from CS5463
		read_data |= 0x01;
		else
		read_data |= 0x00;
		
		PORTA &= ~(1<<clk_cs5463); //Clock Set Low
		
		clk_delay_cs5463;
	}
	
	control = 0x80;
	
	for(uint8_t j=0;j<24;j++)
	{
		
		PORTA |= (1<<do_cs5463); //Send binary "1" to CS5463 during 24 bit cycle (3 times SYNC1)
		
		PORTA |= (1<<clk_cs5463); //Clock Set High
		clk_delay_cs5463;
		
		read_data <<=1;
		if(bit_is_set(PINA, PINA5)) //Read each bit coming from CS5463
			read_data |= 0x01;
		else
			read_data |= 0x00;
		
		PORTA &= ~(1<<clk_cs5463); //Clock Set Low
		
		clk_delay_cs5463;
	}
	
	read_data &= (0x00FFFFFF); //Take only meaningful 24 bit of 32 bit coming from CS5463
	
	return read_data;
}


void SPISetUpSoftware()
{
	//Set all outputs
	DDRA |= (1<<DDA1) | (1<<DDA2) | (1<<DDA3) | (1<<DDA4) | (1<<DDA6);
	DDRB |= (1<<DDB0) | (1<<DDB1);
	
	//Set all Inputs
	DDRA &= ~(1<<DDA5);
	DDRA &= ~(1<<DDA7);
	PORTA &= ~(1<<di_cs5463);
	PORTA |=  (1<<do_cs5463) | (1<<clk_cs5463) | (1<<rst_cs5463);
	
	
}


uint16_t Map_CS5463_HV9910B_Values(uint32_t data)
{
	uint16_t Read_HV9910 = 0;
	Read_HV9910 = (uint16_t)(round(data / ACT_STEP_VALUE_HV9910));
	return Read_HV9910;
}

void USART_Init(){
	UBRR0H = (uint8_t)(BRC >> 8);
	UBRR0L = (uint8_t)(BRC);
	UCSR0B = (1 << RXEN0) | (1 << TXEN0);
	UCSR0C = (3 << UCSZ00);
}

unsigned char USART_Receive(){
	while(!(UCSR0A & (1 << RXC0)));
	return UDR0;
}

void USART_Send(uint8_t data){
	
	while(!(UCSR0A & (1 << UDRE0)));
	UDR0 = data;
}

void USART_PutValue(uint32_t value)
{
	uint8_t temp;
	temp = (value & 0x00FF0000) >> 16;
	USART_Send(temp);
	temp = (value & 0x0000FF00) >> 8;
	USART_Send(temp);
	temp = (value & 0x000000FF);
	USART_Send(temp);
}


void PWM_SetUp()
{
	DDRB |= (1<<DDB1);
	TCCR1B |= (1<<CS10) | (1<<WGM12);		//No Prescaler, CTC Mode
	TIMSK1 |= (1<<OCIE1A) | (1<<OCIE1B);
	
}

void PWM_Write(uint16_t freq, uint16_t duty)
{
	OCR1A = freq;
	OCR1B = duty;
	sei();
}


 