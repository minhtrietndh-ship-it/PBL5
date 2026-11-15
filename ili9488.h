#ifndef ILI9488_H
#define ILI9488_H

#include <stdint.h>

#define LCD_WIDTH   320
#define LCD_HEIGHT  480

void ili9488_init(void);

void ili9488_fill_screen(
    uint8_t r,
    uint8_t g,
    uint8_t b
);

void ili9488_fill_rect(
    int x,
    int y,
    int w,
    int h,
    uint8_t r,
    uint8_t g,
    uint8_t b
);

void ili9488_draw_pixel(
    int x,
    int y,
    uint8_t r,
    uint8_t g,
    uint8_t b
);

/*
 * Gửi một vùng ảnh RGB565 xuống ILI9488.
 * Hàm này rất hữu ích để nối với LVGL sau này.
 *
 * rgb565:
 *   mỗi pixel = 16 bit RGB565
 *
 * x1,y1,x2,y2:
 *   vùng cần cập nhật
 */
void ili9488_flush_rgb565(
    int x1,
    int y1,
    int x2,
    int y2,
    const uint16_t *rgb565
);

#endif