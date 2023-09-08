#define F_CPU 1000000UL

#include <avr/io.h>
#include <stdio.h>
#include <util/delay.h>
#include <avr/interrupt.h>
//#include <util/atomic.h>
//#include <avr/pgmspace.h>
//#include <math.h>

#include "TM1637_lib.h"
#include "EEPROM_lib.h"

#define MOT_DELAY_ON 20
#define MOT_DELAY_OFF 2
#define STEP_TIME 1000     //задержка между шагами для ведения 

#define MOT_A PB2
#define MOT_B PB3
#define MOT_C PB4
#define MOT_D PB5
#define ENC_A PD2
#define ENC_B PD3
#define ENC_SW PD6

uint32_t    msec = 0, 
            lastTimeStep = 0;      
int16_t     period = 0,


uint16_t    sec = 0,  
            lastTimeChange = 0,
            counter = 0;
            
int8_t      stateDrive = 0, 
            lastSwState = 0, 
            mode = 1, 
            colon = 0, 
            direction = 1, 
            interruptDetected = 0;

//===================================ПРЕРЫВАНИЯ================================================
ISR(INT0_vect) {
  if (mode){
    if (PIND & (1 << ENC_B)) {
      if (PIND & (1 << ENC_SW)) {(period < 9999) ? period ++ : period = 9999;} 
      else {(period < 9990) ? period += 10 : period = 9999;}
    } else {
      if (PIND & (1 << ENC_SW)) {(period > 0) ? period-- : period = 0;}
      else {(period > 10) ? period -= 10 : period = 0;}
    }
    lastTimeChange = sec;
    interruptDetected = 1;
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

ISR(WDT_OVERFLOW_vect){
  //sec++;
  colon ^= 1;
}

//===================================НАСТРОЙКИ==================================================
void setup(void) {  
    //настройка INT0
  GIMSK = (1 << INT0); //вкл. внешнее прерывание INT0
  MCUCR = (1 << ISC01) | (1 << ISC00); //прерывание по переднему фронту
    //настройка таймера
  TCCR1A = 0x00;
  TCCR1B = (1 << WGM12) | (1 << CS10);
  TIMSK = (1 << OCIE1A);
  OCR1AH = 0x03;       //считаем до 1000
  OCR1AL = 0xE8;
    //настроим watchdog как очень медленный таймер
  WDTCSR |= (1 << WDIE) | (1 << WDP2) | (1 << WDP0);    //прерывание каждые 0,5 сек
    //настройка ног
  DDRD &= ~((1 << ENC_A) | (1 << ENC_B) | (1 << ENC_SW)); //ноги от энкодера на вход
  DDRB |= (1 << MOT_A) | (1 << MOT_B) | (1 << MOT_C) | (1 << MOT_D);
  PORTB |= ((1 << MOT_A) | (1 << MOT_B) | (1 << MOT_C) | (1 << MOT_D));
    //включаем прерывания
  sei();   

  TM1637_init();

  period = (EEPROM_read(0) << 8) | EEPROM_read(1);
}

//==================================ПРОЦЕДУРЫ==================================================
int incState(void){
  stateDrive++;
  if (stateDrive > 7) stateDrive = 0;
  return stateDrive;
}

int decState(void){
  stateDrive--;
  if (stateDrive < 0) stateDrive = 7;
  return stateDrive;
}

void command (int commandNumber){
    
    /*      -1 - все обмотки выклоючены, двигатель остановлен.
     * st   0   1   2   3   4   5   6   7
     *      A   AC  C   CB  B   BD  D   DA  - на какие вывода подано напряжение
     * A|   1   1   0   0   0   0   0   1   - Обмотка 1
     * B|   0   0   0   1   1   1   0   0   -
     * C|   0   1   1   1   0   0   0   0   - Обмотка 2
     * D|   0   0   0   0   0   1   1   1   -
     */
    
  switch (commandNumber) {
  case -1 : 
    PORTB |= ((1 << MOT_A) | (1 << MOT_B) | (1 << MOT_C) | (1 << MOT_D));    
    break;    
  case 0 : 
    PORTB &= ~(1 << MOT_A);
    PORTB |= ((1 << MOT_B) | (1 << MOT_C) | (1 << MOT_D));
    stateDrive = 0;
    break;
  case 1 : 
    PORTB &= ~(1 << MOT_A);
    PORTB &= ~(1 << MOT_C);
    PORTB |= ((1 << MOT_B) | (1 << MOT_D));
    stateDrive = 1;
    break;    
  case 2 : 
    PORTB &= ~(1 << MOT_C);
    PORTB |= ((1 << MOT_A) | (1 << MOT_B) | (1 << MOT_D));    
    stateDrive = 2;
    break;
  case 3 : 
    PORTB &= ~(1 << MOT_C);
    PORTB &= ~(1 << MOT_B);
    PORTB |= ((1 << MOT_A) | (1 << MOT_D));  
    stateDrive = 3;
    break;
  case 4 : 
    PORTB &= ~(1 << MOT_B);
    PORTB |= ((1 << MOT_A) | (1 << MOT_C) | (1 << MOT_D));    
    stateDrive = 4;
    break;
  case 5 : 
    PORTB &= ~(1 << MOT_B);
    PORTB &= ~(1 << MOT_D);
    PORTB |= ((1 << MOT_A) | (1 << MOT_C));  
    stateDrive = 5;
    break;
  case 6 : 
    PORTB &= ~(1 << MOT_D);
    PORTB |= ((1 << MOT_A) | (1 << MOT_B) | (1 << MOT_C));  
    stateDrive = 6;
    break;
  case 7 : 
    PORTB &= ~(1 << MOT_D);
    PORTB &= ~(1 << MOT_A);
    PORTB |= ((1 << MOT_B) | (1 << MOT_C));    
    stateDrive = 7;
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

void stopStepper(void){
    command(-1);
}

//===================================ГЛАВНАЯ ПРОГРАММА==========================================
void main(void) {
  setup();  
  while (1) {  
  sec = msec/1000;
//-----------кнопка энкодера----------
    if ((lastSwState == 0) && (!(PIND & (1 << ENC_SW)))){
      lastSwState = 1;
      interruptDetected = 0;
    }
    if ((lastSwState == 1) && (PIND & (1 << ENC_SW))){
      if (interruptDetected == 0){
        mode ^= 1;
      }
      lastSwState = 0;
      interruptDetected = 0;
    }
//-------------запись в EEPROM----------------    
    if ((sec - lastTimeShiftChange > 2) && (interruptDetected == 1)) {
      EEPROM_write(0, (period >> 8));
      EEPROM_write(1, (period & 0xFF));
      interruptDetected = 0;
    }

// -----------управление------------
    if (mode){  
      if (STEP_TIME + shift < 0){ shift = -STEP_TIME; }  
      if ((msec - lastTimeStep) >= (STEP_TIME + shift)) {           
        oneStepForward();                                 
        lastTimeStep = msec;                    
      }
      TM1637_write(STEP_TIME + shift, colon);    
    } else {
      if (direction == 1){
        stopStepper();  
//        uint8_t arr[4] = {0x00, 0x80, 0x00, 0x00};
//        TM1637_sendArray(arr);
      } else if (direction == 2) {
        oneStepForward();
//        uint8_t arr[4] = {0x00, 0x80, 0x40, 0x40};
//        TM1637_sendArray(arr);
      } else if (direction == 0) {
        oneStepBack();
//        uint8_t arr[4] = {0x40, 0xC0, 0x00, 0x00};
//        TM1637_sendArray(arr);
      } 
      TM1637_write(counter, colon);
    }   
  }
}






