#include <Arduino.h>
#include <AVR_digitalclock.hpp>
#include <stdio.h>

volatile int    detik = 0,
                menit = 0,
                jam = 0;

int detik_last;
char jam_buffer[9];

uint8_t clear_flag = 0; 


void init_jam(){
  // Initialize Timer-1 (16-bit)
  // - Overflow Interrupt Mode
  // - Overflow setiap 1 detik
  // - F_CPU = 1MHz
  // - prescaler = 64
  TCCR1A = 0;
  TIMSK = (1 << TOIE1);
  TCNT1 = 49900;
  TCCR1B = (1 << CS11) | (1 << CS10);
}

void update_jam(){
      
  // update nilai detik, menit, jam
  if(jam >= 23 && menit >= 59 && detik >= 60){
    jam = 0;
    menit = 0;
    detik = 0;
  }
  else if(menit >= 59 && detik >= 60){
    jam++;
    menit = 0;
    detik = 0;
  }
  if(detik >= 60){
    menit++;
    detik = 0;
  }

  // pecah satuan & puluhan, lalu simpan di char buffer
  sprintf(jam_buffer, "%d%d:%d%d:%d%d",
          jam/10, jam%10,
          menit/10, menit%10,
          detik/10, detik%10
  );
  
  // update nilai detik last
  detik_last = detik;
}