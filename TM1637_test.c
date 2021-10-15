#define F_CPU 1000000UL

#include <avr/io.h>
#include <stdio.h>
#include <util/delay.h>
#include <avr/interrupt.h>
//#include <util/atomic.h>
//#include <avr/pgmspace.h>
//#include <math.h>

#include "TM1637_lib.h"

#define MOT_DELAY 15
#define MOT_A PB2
#define MOT_B PB3
#define MOT_C PB4
#define MOT_D PB5
#define ENC_A PD2
#define ENC_B PD3
#define ENC_SW PD6


uint32_t msec = 0, sec = 0;      
int shift = 0, counter = 0;
int8_t stateDrive = 0, lastSwState = 0, mode = 1, colon = 0, direction = 1, interruptDetected = 0;

//===================================ПРЕРЫВАНИЯ================================================
ISR(INT0_vect) {
  interruptDetected = 1;
  if (mode){
    if (PIND & (1 << ENC_B)) {
      if (shift < 9999) {if (PIND & (1 << ENC_SW)) {shift += 1;} else {shift += 5;}}
    } else {
      if (shift > -999) {if (PIND & (1 << ENC_SW)) {shift -= 1;} else {shift -= 5;}}
    }
  } else {
    if (PIND & (1 << ENC_B)) {
      if (direction < 2) direction++;
    } else {
      if (direction > 0) direction--;
    }
  }
}

ISR(TIMER1_COMPA_vect){
  msec++;
}

ISR(PCINT_vect){}

//===================================НАСТРОЙКИ==================================================
void setup(void) {  
    //настройка INT0
  GIMSK = (1 << INT0); //вкл. внешнее прерывание INT0
  MCUCR = (1 << ISC01) | (1 << ISC00); //прерывание по переднему фронту
    //настройка таймера
  TCCR1A = 0x00;
  TCCR1B = (1 << WGM12) | (1 << CS10);
  TIMSK = (1 << OCIE1A) | (0 << ICIE1);//TODO: здесь какая-то лажа!!!!!!!
  OCR1AH = 0x03;       //считаем до 1000
  OCR1AL = 0xE8;
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
  counter++;
}

void oneStepBack(void){
  command (decState());
  counter--;
}


//===================================ГЛАВНАЯ ПРОГРАММА==========================================
void main(void) {
  setup();  
  TM1637_write(9999,1);
  _delay_ms(2000);
  TM1637_write(0,1);
  while (1) {
//-----------кнопка энкодера----------
    if ((lastSwState == 0) && (!(PIND & (1 << ENC_SW)))){
      lastSwState = 1;
    }
    if ((lastSwState == 1) && (PIND & (1 << ENC_SW))){
//      if (interruptDetected ){
        mode = 0x01;
//      }
      lastSwState = 0;
      interruptDetected = 0;
    }

// -----------управление------------
    if (mode){    
      if (msec & (1 << 12)) {colon ^= 1;}
      TM1637_write(interruptDetected, colon);    
    } else {
      if (direction == 1){
        uint8_t arr[4] = {0x00, 0x80, 0x00, 0x00};
        TM1637_sendArray(arr);
      } else if (direction == 2) {
        oneStepForward();
        uint8_t arr[4] = {0x00, 0x80, 0x40, 0x40};
        TM1637_sendArray(arr);
      } else if (direction == 0) {
        oneStepBack();
        uint8_t arr[4] = {0x40, 0xC0, 0x00, 0x00};
        TM1637_sendArray(arr);
      } 
    }   
  }
}






