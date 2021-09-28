#include "TM1637_lib.h"

#ifndef F_CPU
#define F_CPU 1000000UL
#endif

#define CLK		PD4   // ----> [CLK]	
#define DIO		PD5   // ----> [DIO]	
                      //       ATTiny2313

#include <stdlib.h>
#include <avr/io.h>
#include <stdio.h>
#include <util/delay.h>

const uint8_t digitHEX[] = {0x3f, 0x06, 0x5b, 0x4f,
	0x66, 0x6d, 0x7d, 0x07,
	0x7f, 0x6f, 0x00, 0x40
};

void TM1637_init(void){
    DDRD |= (1 << CLK);	// Set port as output
	DDRD |= (1 << DIO);	// Set port as output
} 

uint8_t digToHEX(uint8_t digit){
	return digitHEX[digit];
}      
void TM1637_start(void){
    PORTD |= (1 << CLK);
    PORTD |= (1 << DIO);
    PORTD &= ~(1 << DIO);
    PORTD &= ~(1 << CLK);
}  
void TM1637_stop(void){
    PORTD &= ~(1 << CLK);
    PORTD &= ~(1 << DIO);
    PORTD |= (1 << CLK);
    PORTD |= (1 << DIO);    
}            
void TM1637_writeByte(uint8_t byte){
    uint8_t i, count1;
	for (i = 0; i < 8; i++) //sent 8bit data
	{
		PORTD &= ~(1 << CLK);
		if (byte & 0x01) PORTD |= (1 << DIO); //LSB first
		else PORTD &= ~(1 << DIO);
		byte >>= 1;
		PORTD |= (1 << CLK);

	}
	PORTD &= ~(1 << CLK); //wait for the ACK
	PORTD |= (1 << DIO);
	PORTD |= (1 << CLK);
	DDRD &= ~(1 << DIO);

	_delay_us(50);
	uint8_t ack = (PIND & (1 << DIO));
	if (ack == 0)
	{
		DDRD |= (1 << DIO);
		PORTD &= ~(1 << DIO);
	}
	_delay_us(50);
	DDRD |= (1 << DIO);
	_delay_us(50);

//	return ack;
    
}
void TM1637_sendByte(uint8_t BitAddr, int8_t sendData){
    TM1637_start();          //start signal sent to GyverTM1637 from MCU
	TM1637_writeByte(ADDR_FIXED);//
	TM1637_stop();           //
	TM1637_start();          //
	TM1637_writeByte(BitAddr | 0xc0); //
	TM1637_writeByte(sendData);//
	TM1637_stop();            //
	TM1637_start();          //
	TM1637_writeByte(0x88 + 0x10);//
	TM1637_stop();           //
}
void TM1637_sendArray(uint8_t sendData[]){
    TM1637_start();          //start signal sent to GyverTM1637 from MCU
	TM1637_writeByte(ADDR_AUTO);//
	TM1637_stop();           //
	TM1637_start();          //
	TM1637_writeByte(0xc0);//
	for (uint8_t i = 0; i < 4; i ++) {
		TM1637_writeByte(sendData[i]);        //
	}
	TM1637_stop();           //
	TM1637_start();          //
	TM1637_writeByte(0x88 + 0x10);//
	TM1637_stop();           //
}

void TM1637_write(int number){
    if ((number > 9999) || (number < -999)) {
        uint8_t data[4] = {0x40, 0x40, 0x40, 0x40};
        TM1637_sendArray((uint8_t *)data);
    } else {
        uint8_t data[4];        
        if (number > 999) {
          for (uint8_t i = 0; i < 4; i++) {
            data[3 - i] = digitHEX[number % 10];
            number /= 10; 
          }
        } else if (number > 99) {
          for (uint8_t i = 0; i < 4; i++) {
            data[3 - i] = digitHEX[number % 10];
            number /= 10; 
          }
          data[0] = 0x00;
        } else if (number > 9) {
          for (uint8_t i = 0; i < 4; i++) {
            data[3 - i] = digitHEX[number % 10];
            number /= 10; 
          }
          data[0] = 0x00;
          data[1] = 0x00;
        } else {
          data[0] = 0x00;
          data[1] = 0x00;
          data[2] = 0x00;
          data[3] = digitHEX[number % 10];
        }
        TM1637_sendArray((uint8_t *)data);   
        
    }
}

