#include "driver_LCD16x2.hpp"

// global definitions
volatile uint8_t *LCD_DATA_PORT, *LCD_data_dir, 
                 *LCD_CONTROL_PORT, *LCD_control_dir;
uint8_t LCD_RS_pin, LCD_EN_pin;

/* data port is 8-bit, control port is 2-bit
 initializes LCD with the selected port in 8-bit mode
 parameter: 
 - PORTx ADDRESS (ex: &PORTA),
 - DDRx ADDRESS (ex: &DDRA),
 - pin number (either integer / pin defines such as PC0, PA7, etc
   defined in the pin definitions, the same as datasheet)
*/
void lcd_init(volatile uint8_t *data_port, volatile uint8_t *control_port, 
              volatile uint8_t *data_dir, volatile uint8_t *control_dir,
              uint8_t RS_pin, uint8_t EN_pin)
{
    // assign ports
    LCD_DATA_PORT       = data_port;
    LCD_CONTROL_PORT    = control_port;
    LCD_data_dir        = data_dir;
    LCD_control_dir     = control_dir;
    LCD_EN_pin          = EN_pin;
    LCD_RS_pin          = RS_pin;

    // set ports as outputs
    *LCD_data_dir       = 0xFF;
    *LCD_control_dir    |= (1 << RS_pin) | (1 << EN_pin);

    // request reset sequence to HD44780
    lcd_command(0x30);         
    delay(4);        
    lcd_command(0x30);
    delayMicroseconds(100);    
    lcd_command(0x30);
    delayMicroseconds(100);     

    // set LCD function set (8-bit mode, 2 line, font 5x10 dot)
    lcd_command(FUNCTION_SET); 
    delayMicroseconds(54);

    // turn off display first
    lcd_command(DISPLAY_OFF);  
    delayMicroseconds(54);

    // clear display
    lcd_command(CLEAR_DISPLAY);
    delay(3);

    // set entry mode (cursor incremental)
    lcd_command(ENTRY_MODE);  
    delayMicroseconds(54);

    // turn on the display (cursor off)
    lcd_command(DISPLAY_ON);  
    delayMicroseconds(54);
}

/* set LCD position
 row 0, row 1,
 col 0 - 15
 see datasheet for DDRAM addressing
*/
void lcd_setpos(uint8_t row, uint8_t col){
    uint8_t LCD_position;

    // safety
    if(row > 1){
        row = 1;
    }

    // set DDRAM address hex command is 0x80
    LCD_position = (row*0x40 + col) | 0x80;
    lcd_command(LCD_position);
    delay(1);
}

/* Send a string to display in LCD.
Common use:
- lcd_string("Test");
- char *str_test = "Test 2"; lcd_string(str_test);
- char buf[10]; sprintf(buf, "Test %d", 3); lcd_string(buf);

OUTPUT:
- Test
- Test 2
- Test 3
*/
void lcd_string(char *char_array_pointer){

    // while
    while( *char_array_pointer > 0 ){
        lcd_char(*char_array_pointer++);
    }
}

/*
Send commands to LCD. List of commands are explained in datasheets.
Beware that "Function Set" command can only be executed by HD44780
during initialization only.
most used lcd_command argument:
- CLEAR_DISPLAY
- DISPLAY_ON (cursor OFF)
- CURSOR_ON
*/
void lcd_command(uint8_t command){

    // set to command mode
    *LCD_CONTROL_PORT &= ~(1 << LCD_RS_pin);

    // ready the data
    *LCD_DATA_PORT = command;

    // pulse enable pin
    *LCD_CONTROL_PORT |= (1 << LCD_EN_pin);
    delayMicroseconds(10);
    *LCD_CONTROL_PORT &= ~(1 << LCD_EN_pin);
    
}

/* 
 Send a single character to the LCD to be displayed.
 Before using this function, make sure to set the LCD position
 first.
*/ 
void lcd_char(char data){

    // set to data mode
    *LCD_CONTROL_PORT |= (1 << LCD_RS_pin);

    // ready the data
    *LCD_DATA_PORT = data;

    // pulse enable pin
    *LCD_CONTROL_PORT |= (1 << LCD_EN_pin);
    delayMicroseconds(10);
    *LCD_CONTROL_PORT &= ~(1 << LCD_EN_pin);
    delay(1);
}