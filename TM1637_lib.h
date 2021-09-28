/*
 * Cамая лучшая библиотека для TM1637
 */


#include <avr/io.h>

    #define CLK		PD4   // ----> [CLK]	
    #define DIO		PD5   // ----> [DIO]	
                          //       ATTiny2313

#define ADDR_AUTO  0x40
#define ADDR_FIXED 0x44
#define STARTADDR  0xc0


void TM1637_init(void);
uint8_t digToHEX(uint8_t digit);       
void TM1637_start(void);  
void TM1637_stop(void);            
void TM1637_writeByte(uint8_t byte);
void TM1637_sendByte(uint8_t BitAddr, int8_t sendData);
void TM1637_sendArray(uint8_t sendData[]);
void TM1637_write(int data);
