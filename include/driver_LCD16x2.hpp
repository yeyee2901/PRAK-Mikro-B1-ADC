#ifndef LCD_DRIVER_H
#define LCD_DRIVER_H

#include <Arduino.h>

// GLOBAL MACROS
#define FUNCTION_SET    0x3C
#define DISPLAY_OFF     0x08
#define CLEAR_DISPLAY   0x01
#define ENTRY_MODE      0x06
#define RETURN_HOME     0x02
#define DISPLAY_ON      0x0C
#define CURSOR_ON       0x0E

// GLOBAL VARIABLE DECLARATIONS
extern volatile uint8_t *LCD_DATA_PORT, *LCD_data_dir, 
                        *LCD_CONTROL_PORT, *LCD_control_dir;
extern uint8_t LCD_RS_pin, LCD_EN_pin;


void lcd_init(volatile uint8_t *data_port, volatile uint8_t *control_port, 
              volatile uint8_t *data_dir, volatile uint8_t *control_dir,
              uint8_t RS_pin, uint8_t EN_pin);

void lcd_setpos(uint8_t row, uint8_t col);
void lcd_string(char *char_array_pointer);
void lcd_command(uint8_t command);
void lcd_char(char data);

#endif