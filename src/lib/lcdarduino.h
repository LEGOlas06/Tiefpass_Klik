//
// Created by werne on 09.01.2023.
//

#ifndef MEGA2560_LCD_DISPLAY_LCDARDUINO_H
#define MEGA2560_LCD_DISPLAY_LCDARDUINO_H

#include <inttypes.h>
#include <avr/io.h>
#include <avr/pgmspace.h>

#define XTAL 16000000              /**< clock frequency in Hz, used to calculate delay timer */
#define LCD_CONTROLLER_KS0073 0    /**< Use 0 for HD44780 controller, 1 for KS0073 controller */

/**
 *  @name  Definitions for Display Size
 *  Change these definitions to adapt setting to your display
 */
#define LCD_LINES           2     /**< number of visible lines of the display */
#define LCD_DISP_LENGTH    16     /**< visibles characters per line of the display */
#define LCD_LINE_LENGTH  0x40     /**< internal line length of the display    */
#define LCD_START_LINE1  0x00     /**< DDRAM address of first char of line 1 */
#define LCD_START_LINE2  0x40     /**< DDRAM address of first char of line 2 */
#define LCD_START_LINE3  0x14     /**< DDRAM address of first char of line 3 */
#define LCD_START_LINE4  0x54     /**< DDRAM address of first char of line 4 */
#define LCD_WRAP_LINES      0     /**< 0: no wrap, 1: wrap at end of visibile line */

#if defined(__AVR_ATmega328P__)  // Arduino UNO, NANO
    #define LCD_DATA0_PORT   PORTD        /**< port for 4bit data bit 0 */
    #define LCD_DATA1_PORT   PORTD        /**< port for 4bit data bit 1 */
    #define LCD_DATA2_PORT   PORTD        /**< port for 4bit data bit 2 */
    #define LCD_DATA3_PORT   PORTD        /**< port for 4bit data bit 3 */
    #define LCD_DATA0_PIN    4            /**< pin for 4bit data bit 0  */
    #define LCD_DATA1_PIN    5            /**< pin for 4bit data bit 1  */
    #define LCD_DATA2_PIN    6            /**< pin for 4bit data bit 2  */
    #define LCD_DATA3_PIN    7            /**< pin for 4bit data bit 3  */
    #define LCD_RS_PORT      PORTB        /**< port for RS line         */
    #define LCD_RS_PIN       0            /**< pin  for RS line         */
    #define LCD_E_PORT       PORTB        /**< port for Enable line     */
    #define LCD_E_PIN        1            /**< pin  for Enable line     */
#endif

#if defined(__AVR_ATmega2560__)  // Arduino MEGA2560
    #define LCD_DATA0_PORT   PORTG        /**< port for 4bit data bit 0 */
    #define LCD_DATA1_PORT   PORTE        /**< port for 4bit data bit 1 */
    #define LCD_DATA2_PORT   PORTH        /**< port for 4bit data bit 2 */
    #define LCD_DATA3_PORT   PORTH        /**< port for 4bit data bit 3 */
    #define LCD_DATA0_PIN    5            /**< pin for 4bit data bit 0  */
    #define LCD_DATA1_PIN    3            /**< pin for 4bit data bit 1  */
    #define LCD_DATA2_PIN    3            /**< pin for 4bit data bit 2  */
    #define LCD_DATA3_PIN    4           /**< pin for 4bit data bit 3  */
    #define LCD_RS_PORT      PORTH        /**< port for RS line         */
    #define LCD_RS_PIN       5            /**< pin  for RS line         */
    #define LCD_E_PORT       PORTH        /**< port for Enable line     */
    #define LCD_E_PIN        6            /**< pin  for Enable line     */
#endif
/**
 *  @name Definitions for LCD command instructions
 *  The constants define the various LCD controller instructions which can be passed to the
 *  function lcd_command(), see HD44780 data sheet for a complete description.
 */

/* instruction register bit positions, see HD44780U data sheet */
#define LCD_CLR               0      /* DB0: clear display                  */
#define LCD_HOME              1      /* DB1: return to home position        */
#define LCD_ENTRY_MODE        2      /* DB2: set entry mode                 */
#define LCD_ENTRY_INC         1      /*   DB1: 1=increment, 0=decrement     */
#define LCD_ENTRY_SHIFT       0      /*   DB2: 1=display shift on           */
#define LCD_ON                3      /* DB3: turn lcd/cursor on             */
#define LCD_ON_DISPLAY        2      /*   DB2: turn display on              */
#define LCD_ON_CURSOR         1      /*   DB1: turn cursor on               */
#define LCD_ON_BLINK          0      /*     DB0: blinking cursor ?          */
#define LCD_MOVE              4      /* DB4: move cursor/display            */
#define LCD_MOVE_DISP         3      /*   DB3: move display (0-> cursor) ?  */
#define LCD_MOVE_RIGHT        2      /*   DB2: move right (0-> left) ?      */
#define LCD_FUNCTION          5      /* DB5: function set                   */
#define LCD_FUNCTION_8BIT     4      /*   DB4: set 8BIT mode (0->4BIT mode) */
#define LCD_FUNCTION_2LINES   3      /*   DB3: two lines (0->one line)      */
#define LCD_FUNCTION_10DOTS   2      /*   DB2: 5x10 font (0->5x7 font)      */
#define LCD_CGRAM             6      /* DB6: set CG RAM address             */
#define LCD_DDRAM             7      /* DB7: set DD RAM address             */
#define LCD_BUSY              7      /* DB7: LCD is busy                    */

/* set entry mode: display shift on/off, dec/inc cursor move direction */
#define LCD_ENTRY_DEC            0x04   /* display shift off, dec cursor move dir */
#define LCD_ENTRY_DEC_SHIFT      0x05   /* display shift on,  dec cursor move dir */
#define LCD_ENTRY_INC_           0x06   /* display shift off, inc cursor move dir */
#define LCD_ENTRY_INC_SHIFT      0x07   /* display shift on,  inc cursor move dir */

/* display on/off, cursor on/off, blinking char at cursor position */
#define LCD_DISP_OFF             0x08   /* display off                            */
#define LCD_DISP_ON              0x0C   /* display on, cursor off                 */
#define LCD_DISP_ON_BLINK        0x0D   /* display on, cursor off, blink char     */
#define LCD_DISP_ON_CURSOR       0x0E   /* display on, cursor on                  */
#define LCD_DISP_ON_CURSOR_BLINK 0x0F   /* display on, cursor on, blink char      */

/* move cursor/shift display */
#define LCD_MOVE_CURSOR_LEFT     0x10   /* move cursor left  (decrement)          */
#define LCD_MOVE_CURSOR_RIGHT    0x14   /* move cursor right (increment)          */
#define LCD_MOVE_DISP_LEFT       0x18   /* shift display left                     */
#define LCD_MOVE_DISP_RIGHT      0x1C   /* shift display right                    */

/* function set: set interface data length and number of display lines */
#define LCD_FUNCTION_4BIT_1LINE  0x20   /* 4-bit interface, single line, 5x7 dots */
#define LCD_FUNCTION_4BIT_2LINES 0x28   /* 4-bit interface, dual line,   5x7 dots */
#define LCD_FUNCTION_8BIT_1LINE  0x30   /* 8-bit interface, single line, 5x7 dots */
#define LCD_FUNCTION_8BIT_2LINES 0x38   /* 8-bit interface, dual line,   5x7 dots */

#define LCD_MODE_DEFAULT     ((1<<LCD_ENTRY_MODE) | (1<<LCD_ENTRY_INC) )

/*
 * Initialisiert das Display und setzt den Cursor
 * @param dispAttr LCD_DISP_OFF display on
 *                 LCD_DISP_ON display off, cursor off
 *                 LCD_DISP_ON_CURSOR display on, cursor on
 *                 LCD_DISP_ON_CURSOR_BLINK display on, cursor on flashing
 * @return  none
 */
extern void lcd_init(uint8_t dispAttr);

/*
 * Löscht das Display und setzt den Cursor auf den Ursprung
 */
extern void lcd_clrscr(void);

/*
 * Setzt den Cursor auf den Ursprung
 */
extern void lcd_home(void);

/*
 * Setzt den Cursor auf die angegebene Position
 */
extern void lcd_gotoxy(uint8_t x, uint8_t y);

/*
 * Setzt ein Zeichen an die aktuelle Cursorposition
 */
extern void lcd_putc(char c);

/*
 * Setzt eine String an die aktuelle Cursorposition
 */
extern void lcd_puts(const char *s);

/*
 * Setzt einen String an die aktuelle Cursorposition
 * vom Programmspeicher ohne Auto-Linefeed
 */
extern void lcd_puts_p(const char *progmem_s);

/*
 * Sendet eine Befehl an das LCD-Display
 */
extern void lcd_command(uint8_t cmd);

/*
 * Sendet Daten an das LCD-Display
*/
extern void lcd_data(uint8_t data);

/*
 * Makros für Programmspeicherzugriffe
*/
#define lcd_puts_P(__s)         lcd_puts_p(PSTR(__s))

#define button_ok    1
#define button_up    2
#define button_down  3
#define button_left  4
#define button_right 5

/**
 * Bestimmt welcher Button am LCD-Display gedrückt wurde
 */
unsigned char get_button();
/**
 * Bestimmt welcher Button am LCD-Display gedrückt wurde wenn button der
 * Wert vom ADC0 ist!
 */
unsigned char parse_button(unsigned char button);

#endif //MEGA2560_LCD_DISPLAY_LCDARDUINO_H
