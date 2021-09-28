#define F_CPU 1000000UL

#include <avr/io.h>
#include <stdio.h>
#include <util/delay.h>
#include <avr/interrupt.h>
//#include <util/atomic.h>
//#include <avr/pgmspace.h>
//#include <math.h>

#include "TM1637_lib.h"

#define MOT_DELAY 2
#define MOT_A PB2
#define MOT_B PB3
#define MOT_C PB4
#define MOT_D PB5
#define ENC_A PD2
#define ENC_B PD3
#define ENC_SW PD6


uint16_t msec = 0, sec = 0;      
int shift = 0;
uint8_t stateDrive = 0;

//===================================ПРЕРЫВАНИЯ================================================
ISR(INT0_vect) {
  if (PIND & (1 << ENC_B)) {
    if (shift < 9999) {if (PIND & (1 << ENC_SW)) {shift += 1;} else {shift += 5;}}
  } else {
    if (shift > 0) {if (PIND & (1 << ENC_SW)) {shift -= 1;} else {shift -= 5;}}
  }
}

//===================================НАСТРОЙКИ==================================================
void setup(void) {  
    //настройка INT0
  GIMSK |= (1 << INT0); //вкл. внешнее прерывание INT0
  MCUCR |= (1 << ISC01) | (1 << ISC00); //прерывание по переднему фронту
    //настройка таймера
  //TCCR1A |=
    //настройка ног
  DDRD &= ~((1 << ENC_A) | (1 << ENC_B) | (1 << ENC_SW)); //ноги от энкодера на вход
  DDRB |= (1 << MOT_A) | (1 << MOT_B) | (1 << MOT_C) | (1 << MOT_D);
  PORTB |= ((1 << MOT_A) | (1 << MOT_B) | (1 << MOT_C) | (1 << MOT_D));
    //включаем прерывания
  sei(); 
    
  TM1637_init();
}

//==================================ПРОЦЕДУРЫ==================================================
int incState(void){
  stateDrive++;
  if (stateDrive > 3) stateDrive = 0;
  return stateDrive;
}

int decState(void){
  stateDrive--;
  if (stateDrive < 0) stateDrive = 3;
  return stateDrive;
}

void command (int commandNumber){
  switch (commandNumber) {
  case 0 : 
    PORTB |= ((1 << MOT_A) | (1 << MOT_B) | (1 << MOT_C) | (1 << MOT_D));
    PORTB &= ~(1 << MOT_A);
    _delay_ms(MOT_DELAY);
    PORTB |= ((1 << MOT_A) | (1 << MOT_B) | (1 << MOT_C) | (1 << MOT_D));
    stateDrive = 0;
    break;
  case 1 : 
    PORTB |= ((1 << MOT_A) | (1 << MOT_B) | (1 << MOT_C) | (1 << MOT_D));
    PORTB &= ~(1 << MOT_C);
    _delay_ms(MOT_DELAY);
    PORTB |= ((1 << MOT_A) | (1 << MOT_B) | (1 << MOT_C) | (1 << MOT_D));
    stateDrive = 1;
    break;
  case 2 : 
    PORTB |= ((1 << MOT_A) | (1 << MOT_B) | (1 << MOT_C) | (1 << MOT_D));
    PORTB &= ~(1 << MOT_B);
    _delay_ms(MOT_DELAY);
    PORTB |= ((1 << MOT_A) | (1 << MOT_B) | (1 << MOT_C) | (1 << MOT_D));
    stateDrive = 2;
    break;
  case 3 : 
    PORTB |= ((1 << MOT_A) | (1 << MOT_B) | (1 << MOT_C) | (1 << MOT_D));
    PORTB &= ~(1 << MOT_D);
    _delay_ms(MOT_DELAY);
    PORTB |= ((1 << MOT_A) | (1 << MOT_B) | (1 << MOT_C) | (1 << MOT_D));
    stateDrive = 3;
    break;
  }
}

void oneStepForward(void){
  command (incState());
}

void oneStepBack(void){
  command (decState());
}

//===================================ГЛАВНАЯ ПРОГРАММА==========================================
void main(void) {
    setup();
    
    while (1) {
      TM1637_write(shift); 
        if (shift > 0) {
          oneStepForward();
          shift--;
          _delay_ms(1);
        }       
        //shift++;
        //PORTB ^= (1 << MOT_A);



    }
}






