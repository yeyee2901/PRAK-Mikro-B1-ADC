#ifndef AVR_DIGITAL_CLOCK
#define AVR_DIGITAL_CLOCK

#include <Arduino.h>

extern volatile int detik,
                    menit,
                    jam;
extern int detik_last;
extern char jam_buffer[9];

extern uint8_t clear_flag; 


void update_jam();
void init_jam();


#endif