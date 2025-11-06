/** I2C LCD for HD44780 via PCF8574 (20x4 ready)
 *  Phù h?p tr?c ti?p v?i main.c hi?n t?i c?a b?n.
 */
#include "i2c-lcd.h"
#include "stm32f1xx_hal.h"   /* ho?c stm32xxxx_hal.h tuong ?ng series b?n dùng */

extern I2C_HandleTypeDef hi2c1;  /* hi2c1 du?c t?o trong main.c */

/* Bit di?u khi?n trên PCF8574 (tu? wiring):
   - LCD_BL: Backlight
   - LCD_EN: Enable
   - LCD_RS: Register Select (0=cmd, 1=data)
*/
#define LCD_BL  0x08
#define LCD_EN  0x04
#define LCD_RS  0x01

/* DDRAM offset cho 20x4 (HD44780) */
static const uint8_t LCD_ROW_OFFS[4] = {0x00, 0x40, 0x14, 0x54};

/* G?i 1 n?a byte (high nibble dã n?m ? bit[7:4]) v?i xung EN */
static void lcd_write4(uint8_t hi_nibble_with_ctrl)
{
    uint8_t t[2];
    t[0] = hi_nibble_with_ctrl | LCD_EN;  /* EN=1 */
    t[1] = hi_nibble_with_ctrl;           /* EN=0 */
    HAL_I2C_Master_Transmit(&hi2c1, LCD_ADDR, t, 2, 100);
}

/* G?i 1 byte ? ch? d? 4-bit (hi nibble tru?c, r?i lo nibble) */
static void lcd_send_u8(uint8_t val, uint8_t rs)
{
    uint8_t hi = (val & 0xF0) | LCD_BL | (rs ? LCD_RS : 0x00);
    uint8_t lo = ((val << 4) & 0xF0) | LCD_BL | (rs ? LCD_RS : 0x00);
    lcd_write4(hi);
    lcd_write4(lo);
}

void lcd_send_cmd(uint8_t cmd)
{
    /* RS=0 (command) */
    lcd_send_u8(cmd, 0);
}

void lcd_send_data(uint8_t data)
{
    /* RS=1 (data) */
    lcd_send_u8(data, 1);
}

void lcd_clear(void)
{
    /* L?nh clear display (0x01) dúng chu?n & nhanh g?n cho m?i kích c? */
    lcd_send_cmd(0x01);
    HAL_Delay(2);  /* ~1.53ms theo datasheet */
}

/* Ð?t con tr? theo hàng/c?t (h? tr? d? 4 hàng) */
void lcd_put_cur(uint8_t row, uint8_t col)
{
    if (row >= LCD_ROWS) row = LCD_ROWS - 1;
    if (col >= LCD_COLS) col = LCD_COLS - 1;
    uint8_t addr = LCD_ROW_OFFS[row] + col;
    lcd_send_cmd(0x80 | addr);  /* Set DDRAM address */
}

void lcd_init(void)
{
    /* Trình t? kh?i t?o 4-bit theo datasheet (an toàn & tuong thích) */
    HAL_Delay(50);  /* >40ms sau khi c?p ngu?n */

    /* Ép 8-bit mode 3 l?n b?ng high nibble 0x30 (ch? g?i hi-nibble) */
    lcd_write4(0x30 | LCD_BL); HAL_Delay(5);
    lcd_write4(0x30 | LCD_BL); HAL_Delay(1);
    lcd_write4(0x30 | LCD_BL); HAL_Delay(1);

    /* Vào 4-bit mode (0x20) - cung ch? hi-nibble */
    lcd_write4(0x20 | LCD_BL); HAL_Delay(5);

    /* Function set: 4-bit, 2-line (áp d?ng cho 20x4), font 5x8 */
    lcd_send_cmd(0x28); HAL_Delay(1);

    /* Display OFF */
    lcd_send_cmd(0x08); HAL_Delay(1);

    /* Clear display */
    lcd_send_cmd(0x01); HAL_Delay(2);

    /* Entry mode: tang c?t, không shift */
    lcd_send_cmd(0x06); HAL_Delay(1);

    /* Display ON, cursor OFF, blink OFF */
    lcd_send_cmd(0x0C); HAL_Delay(1);

    /* Con tr? v? (0,0) là m?c d?nh sau clear; có th? set l?i n?u mu?n */
}

void lcd_send_string(const char *str)
{
    while (*str) {
        lcd_send_data((uint8_t)*str++);
    }
}
