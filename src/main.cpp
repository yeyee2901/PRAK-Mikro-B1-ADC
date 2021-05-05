#include <Arduino.h>
#include <driver_LCD16x2.hpp>
#include <AVR_digitalclock.hpp>
#include <stdio.h>

#define DIFFERENTIAL_INPUT  0
#define SINGLE_ENDED        1
#define ADC_CH0             0b00000
#define ADC_DIFF_CH_0p_1n   0b10000

// GLOBAL VARIABLES ----------------------------------------
char ADC_ASCII[5];
char voltage_buf[12];




// FUNCTION PROTOTYPE --------------------------------------
void init_ADC();
uint16_t baca_ADC(int channel);
double dec2volt(uint16_t ADC_value, uint8_t channel_type);
void volt2ASCII(double voltage_value);
void volt2temp_ASCII(double voltage_value);
void dec2ASCII(uint16_t ADC_value); // for testing purpose




// MAIN FUNCTION -------------------------------------------
void setup() {

  // Init LCD
  lcd_init(&PORTC, &PORTD, &DDRC, &DDRD, PD0, PD1);
  lcd_command(CLEAR_DISPLAY);

  // Init Jam Digital
  init_jam();

  // Init ADC
  init_ADC();

  // global interrupt enable
  sei();
  delay(2000);
}

void loop() {
  // ambil data ADC
  unsigned int ADC_result = baca_ADC(ADC_CH0);

  // ubah nilai ADC ke ASCII, nilai ASCII tsb disimpan di buffer char array
  // 'ADC_ASCII'
  dec2ASCII(ADC_result);
  lcd_setpos(0,0);
  lcd_string(ADC_ASCII);

  // ubah nilai ADC ke tegangan (dalam VOLT), kalikan 1000 agar menjadi mV
  // hasil konversi ke ASCII disimpan di buffer char array 'voltage_buf'
  double voltage = dec2volt(ADC_result, SINGLE_ENDED);
  volt2ASCII(voltage);
  lcd_setpos(0, 5);

  // Update jam
  if(detik_last != detik){
    update_jam();
    lcd_setpos(1,0);
    lcd_string(jam_buffer);
  }
}



// ISR ----------------------------------------------
// Overflow setiap 1 detik, reset TCNT1 ke 49900
ISR(TIMER1_OVF_vect){
  detik++;
  TCNT1 = 49900;
}




// FUNCTION DEFINITION ------------------------------
void init_ADC(){
  // Prescaler = 8
  // dengan clock CPU 1 MHz, maka clock ADC = 1 MHz / 8 = 125 kHz
  // Enable Auto trigger mode
  ADCSRA |= (1 << ADPS0) | (1 << ADPS1) | (1 << ADATE);

  // reference voltage -> AREF = AVCC (board di build dengan AREF terhubung AVCC)
  // untuk board tsb, reference voltage tidak bisa diubah. 
  // set mode data ADC right adjusted
  ADMUX = (1 << REFS0);
  ADMUX &= ~(1 << ADLAR);

  // Auto trigger source = Free running
  SFIOR &= ~((1 << ADTS2) | (1 << ADTS1) | (1 << ADTS0));

  // Enable ADC & start the first conversion
  ADCSRA |= (1 << ADEN) | (1 << ADSC);
}

uint16_t baca_ADC(int channel){
  uint16_t ADC_bufferL;
  uint8_t ADC_bufferH;
  uint8_t channel_mask = channel;

  // select channel ADC sesuai kebutuhan
  ADMUX &= 0xE0;
  ADMUX |= channel_mask;

  // clear ADC interrupt flag
  ADCSRA |= (1 << ADIF);

  // tunggu ADIF bernilai 1
  while((ADCSRA & (1 << ADIF)) == 0);

  // ambil low nibble terlebih dahulu, agar nilai ADCH & ADCL
  // di lock selama proses fungsi baca_ADC() ini
  ADC_bufferL = ADCL;
  ADC_bufferH = ADCH;
  ADC_bufferL |= (ADC_bufferH << 8);

  // ambil high nibble (ADCH) yang sudah digeser ke kiri 8x, lalu di OR
  // dengan hasil low nibble (ADCL)
  // ADC_buffer |= (ADCH << 8);

  // clear interrupt flag
  ADCSRA |= (1 << ADIF);

  return ADC_bufferL;
}

// HELPER FUNCTIONS ---------------------
void dec2ASCII(uint16_t ADC_value){
  sprintf(ADC_ASCII, "%d%d%d%d",
    ADC_value / 1000,
    (ADC_value / 100) % 10,
    (ADC_value / 10) % 10,
    ADC_value % 10
  );
}

double dec2volt(uint16_t ADC_value, uint8_t channel_type)
{
  double LSB;
  if (channel_type == SINGLE_ENDED){
    LSB = 5.0 / 1024.0;
  }
  else {
    LSB = 5.0 / 512.0;
  }

  return ADC_value * LSB;
}

void volt2ASCII(double voltage_value)
{
  double voltage_mV = voltage_value * 1000;
  unsigned int voltage_integer = voltage_mV;
  unsigned int voltage_comma_values = (voltage_mV - voltage_integer) * 100;

  sprintf(voltage_buf, "%d%d%d%d.%d%dmV", 
           voltage_integer / 1000,
          (voltage_integer / 100) % 10,
          (voltage_integer / 10) % 10,
           voltage_integer % 10,
          (voltage_comma_values / 10) % 10,
           voltage_comma_values % 10);
}