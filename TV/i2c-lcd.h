#ifndef I2C_LCD_H
#define I2C_LCD_H

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Th�ng s? LCD 20x4 module PCF8574 (d?a ch? write = 0x4E = 0x27<<1) */
#ifndef LCD_ADDR_7BIT
#define LCD_ADDR_7BIT   0x27      /* n?u jumper/bi?n tr? kh�c th� d?i ? d�y */
#endif
#define LCD_ADDR        (LCD_ADDR_7BIT << 1)   /* HAL d�ng d?a ch? 8-bit */

#define LCD_COLS        20
#define LCD_ROWS        4

/* API co b?n */
void lcd_init(void);                                   /* Kh?i t?o LCD (4-bit via I2C)    */
void lcd_send_cmd(uint8_t cmd);                        /* G?i l?nh t?i LCD                 */
void lcd_send_data(uint8_t data);                      /* G?i 1 byte d? li?u t?i LCD       */
void lcd_send_string(const char *str);                 /* G?i chu?i k?t th�c '\0'          */
void lcd_put_cur(uint8_t row, uint8_t col);            /* �?t con tr?: row=0..3, col=0..19 */
void lcd_clear(void);                                  /* X�a m�n h�nh, con tr? v? (0,0)   */

#ifdef __cplusplus
}
#endif
#endif /* I2C_LCD_H */
