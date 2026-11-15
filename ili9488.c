#include "ili9488.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stddef.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/spi_master.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_heap_caps.h"

#define TAG "ILI9488"

// =====================================================
// GPIO
// =====================================================

#define LCD_SCLK    6
#define LCD_MOSI    7
#define LCD_MISO    2
#define LCD_CS      5
#define LCD_DC      3
#define LCD_RST     10

// BL nối trực tiếp 3.3V
#define LCD_BL      -1

// =====================================================
// SPI
// =====================================================

#define LCD_HOST        SPI2_HOST
#define LCD_SPI_FREQ_HZ (10 * 1000 * 1000)

// Số dòng gửi mỗi block
#define BUFFER_LINES    20

static spi_device_handle_t lcd_spi = NULL;


// =====================================================
// GPIO INIT
// =====================================================

static void lcd_gpio_init(void)
{
    // DC
    gpio_reset_pin(LCD_DC);

    ESP_ERROR_CHECK(
        gpio_set_direction(
            LCD_DC,
            GPIO_MODE_OUTPUT
        )
    );

    // RESET
    gpio_reset_pin(LCD_RST);

    ESP_ERROR_CHECK(
        gpio_set_direction(
            LCD_RST,
            GPIO_MODE_OUTPUT
        )
    );

    gpio_set_level(LCD_DC, 1);
    gpio_set_level(LCD_RST, 1);
}


// =====================================================
// SPI INIT
// =====================================================

static void lcd_spi_init(void)
{
    spi_bus_config_t buscfg = {
        .mosi_io_num = LCD_MOSI,
        .miso_io_num = LCD_MISO,
        .sclk_io_num = LCD_SCLK,

        .quadwp_io_num = -1,
        .quadhd_io_num = -1,

        .max_transfer_sz =
            LCD_WIDTH * BUFFER_LINES * 3
    };

    ESP_ERROR_CHECK(
        spi_bus_initialize(
            LCD_HOST,
            &buscfg,
            SPI_DMA_CH_AUTO
        )
    );

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = LCD_SPI_FREQ_HZ,

        // SPI MODE 0
        .mode = 0,

        .spics_io_num = LCD_CS,

        .queue_size = 1
    };

    ESP_ERROR_CHECK(
        spi_bus_add_device(
            LCD_HOST,
            &devcfg,
            &lcd_spi
        )
    );

    ESP_LOGI(
        TAG,
        "SPI initialized: %d Hz",
        LCD_SPI_FREQ_HZ
    );
}


// =====================================================
// RESET
// =====================================================

static void lcd_reset(void)
{
    ESP_LOGI(TAG, "LCD reset");

    gpio_set_level(LCD_RST, 1);

    vTaskDelay(
        pdMS_TO_TICKS(20)
    );

    gpio_set_level(LCD_RST, 0);

    vTaskDelay(
        pdMS_TO_TICKS(120)
    );

    gpio_set_level(LCD_RST, 1);

    vTaskDelay(
        pdMS_TO_TICKS(120)
    );
}


// =====================================================
// WRITE COMMAND
// =====================================================

static void lcd_write_cmd(uint8_t cmd)
{
    gpio_set_level(LCD_DC, 0);

    spi_transaction_t t = {
        .length = 8,
        .tx_buffer = &cmd
    };

    ESP_ERROR_CHECK(
        spi_device_polling_transmit(
            lcd_spi,
            &t
        )
    );
}


// =====================================================
// WRITE DATA
// =====================================================

static void lcd_write_data(
    const uint8_t *data,
    size_t len
)
{
    if (len == 0) {
        return;
    }

    gpio_set_level(LCD_DC, 1);

    spi_transaction_t t = {
        .length = len * 8,
        .tx_buffer = data
    };

    ESP_ERROR_CHECK(
        spi_device_polling_transmit(
            lcd_spi,
            &t
        )
    );
}


// =====================================================
// COMMAND + DATA
// =====================================================

static void lcd_write_cmd_data(
    uint8_t cmd,
    const uint8_t *data,
    size_t len
)
{
    spi_device_acquire_bus(
        lcd_spi,
        portMAX_DELAY
    );

    // COMMAND
    gpio_set_level(LCD_DC, 0);

    spi_transaction_t t_cmd = {
        .flags = SPI_TRANS_CS_KEEP_ACTIVE,
        .length = 8,
        .tx_buffer = &cmd
    };

    ESP_ERROR_CHECK(
        spi_device_polling_transmit(
            lcd_spi,
            &t_cmd
        )
    );

    // DATA
    if (len > 0) {

        gpio_set_level(LCD_DC, 1);

        spi_transaction_t t_data = {
            .length = len * 8,
            .tx_buffer = data
        };

        ESP_ERROR_CHECK(
            spi_device_polling_transmit(
                lcd_spi,
                &t_data
            )
        );
    }

    spi_device_release_bus(lcd_spi);
}


// =====================================================
// ILI9488 INIT
// =====================================================

static void ili9488_controller_init(void)
{
    ESP_LOGI(
        TAG,
        "ILI9488 controller initialization"
    );

    // -------------------------------------------------
    // Sleep Out
    // -------------------------------------------------

    lcd_write_cmd(0x11);

    vTaskDelay(
        pdMS_TO_TICKS(120)
    );


    // -------------------------------------------------
    // Positive Gamma
    // -------------------------------------------------

    const uint8_t gamma_pos[] = {
        0x00,
        0x03,
        0x09,
        0x08,
        0x16,
        0x0A,
        0x3F,
        0x78,
        0x4C,
        0x09,
        0x0A,
        0x08,
        0x16,
        0x1A,
        0x0F
    };

    lcd_write_cmd_data(
        0xE0,
        gamma_pos,
        sizeof(gamma_pos)
    );


    // -------------------------------------------------
    // Negative Gamma
    // -------------------------------------------------

    const uint8_t gamma_neg[] = {
        0x00,
        0x16,
        0x19,
        0x03,
        0x0F,
        0x05,
        0x32,
        0x45,
        0x46,
        0x04,
        0x0E,
        0x0D,
        0x35,
        0x37,
        0x0F
    };

    lcd_write_cmd_data(
        0xE1,
        gamma_neg,
        sizeof(gamma_neg)
    );


    // -------------------------------------------------
    // Power Control 1
    // -------------------------------------------------

    const uint8_t c0[] = {
        0x17,
        0x15
    };

    lcd_write_cmd_data(
        0xC0,
        c0,
        sizeof(c0)
    );


    // -------------------------------------------------
    // Power Control 2
    // -------------------------------------------------

    const uint8_t c1[] = {
        0x41
    };

    lcd_write_cmd_data(
        0xC1,
        c1,
        sizeof(c1)
    );


    // -------------------------------------------------
    // VCOM
    // -------------------------------------------------

    const uint8_t c5[] = {
        0x00,
        0x12,
        0x80
    };

    lcd_write_cmd_data(
        0xC5,
        c5,
        sizeof(c5)
    );


    // -------------------------------------------------
    // MADCTL
    //
    // 0x48:
    // MX + BGR
    // -------------------------------------------------

    const uint8_t madctl[] = {
        0x48
    };

    lcd_write_cmd_data(
        0x36,
        madctl,
        sizeof(madctl)
    );


    // -------------------------------------------------
    // Pixel Format
    //
    // 0x66 = 18-bit RGB666
    // -------------------------------------------------

    const uint8_t pixfmt[] = {
        0x66
    };

    lcd_write_cmd_data(
        0x3A,
        pixfmt,
        sizeof(pixfmt)
    );


    // -------------------------------------------------
    // Interface Mode
    // -------------------------------------------------

    const uint8_t b0[] = {
        0x00
    };

    lcd_write_cmd_data(
        0xB0,
        b0,
        sizeof(b0)
    );


    // -------------------------------------------------
    // Frame Rate
    // -------------------------------------------------

    const uint8_t b1[] = {
        0xA0
    };

    lcd_write_cmd_data(
        0xB1,
        b1,
        sizeof(b1)
    );


    // -------------------------------------------------
    // Display Inversion
    // -------------------------------------------------

    const uint8_t b4[] = {
        0x02
    };

    lcd_write_cmd_data(
        0xB4,
        b4,
        sizeof(b4)
    );


    // -------------------------------------------------
    // Display Function Control
    // -------------------------------------------------

    const uint8_t b6[] = {
        0x02,
        0x02,
        0x3B
    };

    lcd_write_cmd_data(
        0xB6,
        b6,
        sizeof(b6)
    );


    // -------------------------------------------------
    // Entry Mode
    // -------------------------------------------------

    const uint8_t b7[] = {
        0xC6
    };

    lcd_write_cmd_data(
        0xB7,
        b7,
        sizeof(b7)
    );


    // -------------------------------------------------
    // Adjust Control 3
    // -------------------------------------------------

    const uint8_t f7[] = {
        0xA9,
        0x51,
        0x2C,
        0x82
    };

    lcd_write_cmd_data(
        0xF7,
        f7,
        sizeof(f7)
    );


    // -------------------------------------------------
    // Normal Display
    // -------------------------------------------------

    lcd_write_cmd(0x13);


    // -------------------------------------------------
    // Display ON
    // -------------------------------------------------

    lcd_write_cmd(0x29);

    vTaskDelay(
        pdMS_TO_TICKS(25)
    );

    ESP_LOGI(
        TAG,
        "ILI9488 controller ready"
    );
}


// =====================================================
// SET ADDRESS WINDOW
// =====================================================

static void lcd_set_window(
    uint16_t x0,
    uint16_t y0,
    uint16_t x1,
    uint16_t y1
)
{
    uint8_t data[4];

    // -------------------------------------------------
    // Column Address Set
    // -------------------------------------------------

    data[0] = (x0 >> 8) & 0xFF;
    data[1] = x0 & 0xFF;

    data[2] = (x1 >> 8) & 0xFF;
    data[3] = x1 & 0xFF;

    lcd_write_cmd_data(
        0x2A,
        data,
        4
    );


    // -------------------------------------------------
    // Page Address Set
    // -------------------------------------------------

    data[0] = (y0 >> 8) & 0xFF;
    data[1] = y0 & 0xFF;

    data[2] = (y1 >> 8) & 0xFF;
    data[3] = y1 & 0xFF;

    lcd_write_cmd_data(
        0x2B,
        data,
        4
    );
}


// =====================================================
// WRITE PIXEL DATA
//
// Không gửi 0x2C ở đây.
// 0x2C phải được gửi đúng 1 lần trước cả vùng.
// =====================================================

static void lcd_write_pixel_data(
    const uint8_t *data,
    size_t byte_count
)
{
    if (byte_count == 0) {
        return;
    }

    gpio_set_level(LCD_DC, 1);

    spi_transaction_t t = {
        .length = byte_count * 8,
        .tx_buffer = data
    };

    ESP_ERROR_CHECK(
        spi_device_polling_transmit(
            lcd_spi,
            &t
        )
    );
}


// =====================================================
// FILL RECT
//
// ILI9488 SPI:
// 1 pixel = 3 byte
// R G B
// =====================================================

void ili9488_fill_rect(
    int x,
    int y,
    int w,
    int h,
    uint8_t r,
    uint8_t g,
    uint8_t b
)
{
    // -------------------------------------------------
    // Clip
    // -------------------------------------------------

    if (x < 0) {
        w += x;
        x = 0;
    }

    if (y < 0) {
        h += y;
        y = 0;
    }

    if (x + w > LCD_WIDTH) {
        w = LCD_WIDTH - x;
    }

    if (y + h > LCD_HEIGHT) {
        h = LCD_HEIGHT - y;
    }

    if (w <= 0 || h <= 0) {
        return;
    }


    // -------------------------------------------------
    // Set drawing area
    // -------------------------------------------------

    lcd_set_window(
        x,
        y,
        x + w - 1,
        y + h - 1
    );


    // -------------------------------------------------
    // MEMORY WRITE
    //
    // CHỈ GỌI MỘT LẦN
    // -------------------------------------------------

    lcd_write_cmd(0x2C);


    // -------------------------------------------------
    // DMA BUFFER
    // -------------------------------------------------

    const int lines = BUFFER_LINES;

    size_t buffer_size =
        (size_t)w * lines * 3;

    uint8_t *buffer =
        heap_caps_malloc(
            buffer_size,
            MALLOC_CAP_DMA
        );

    if (buffer == NULL) {

        ESP_LOGE(
            TAG,
            "Cannot allocate DMA buffer"
        );

        return;
    }


    // -------------------------------------------------
    // Prepare one block
    // -------------------------------------------------

    for (size_t i = 0;
         i < (size_t)w * lines;
         i++) {

        buffer[i * 3 + 0] = r;
        buffer[i * 3 + 1] = g;
        buffer[i * 3 + 2] = b;
    }


    // -------------------------------------------------
    // Send rows
    // -------------------------------------------------

    for (int row = 0; row < h; row += lines) {

        int current_lines = lines;

        if (row + current_lines > h) {
            current_lines = h - row;
        }

        size_t pixel_count =
            (size_t)w * current_lines;

        size_t byte_count =
            pixel_count * 3;

        lcd_write_pixel_data(
            buffer,
            byte_count
        );
    }


    free(buffer);
}


// =====================================================
// FILL SCREEN
// =====================================================

void ili9488_fill_screen(
    uint8_t r,
    uint8_t g,
    uint8_t b
)
{
    ili9488_fill_rect(
        0,
        0,
        LCD_WIDTH,
        LCD_HEIGHT,
        r,
        g,
        b
    );
}


// =====================================================
// DRAW PIXEL
// =====================================================

void ili9488_draw_pixel(
    int x,
    int y,
    uint8_t r,
    uint8_t g,
    uint8_t b
)
{
    if (x < 0 ||
        x >= LCD_WIDTH ||
        y < 0 ||
        y >= LCD_HEIGHT) {

        return;
    }


    lcd_set_window(
        x,
        y,
        x,
        y
    );


    lcd_write_cmd(0x2C);


    uint8_t pixel[3];

    pixel[0] = r;
    pixel[1] = g;
    pixel[2] = b;

    lcd_write_pixel_data(
        pixel,
        3
    );
}


// =====================================================
// FLUSH RGB565
//
// Dùng sau này cho LVGL.
// LVGL thường dùng RGB565.
//
// RGB565:
//
// RRRRRGGG GGGBBBBB
//
// Chuyển sang RGB666/RGB888:
//
// R5 -> R6
// G6 -> G6
// B5 -> B6
// =====================================================

void ili9488_flush_rgb565(
    int x1,
    int y1,
    int x2,
    int y2,
    const uint16_t *rgb565
)
{
    if (rgb565 == NULL) {
        return;
    }

    if (x1 < 0)
        x1 = 0;

    if (y1 < 0)
        y1 = 0;

    if (x2 >= LCD_WIDTH)
        x2 = LCD_WIDTH - 1;

    if (y2 >= LCD_HEIGHT)
        y2 = LCD_HEIGHT - 1;

    if (x1 > x2 || y1 > y2) {
        return;
    }

    int width =
        x2 - x1 + 1;

    int height =
        y2 - y1 + 1;


    // Set drawing area
    lcd_set_window(
        x1,
        y1,
        x2,
        y2
    );


    // Start memory write
    lcd_write_cmd(0x2C);


    // Buffer RGB666
    size_t max_pixels =
        (size_t)width * BUFFER_LINES;

    uint8_t *buffer =
        heap_caps_malloc(
            max_pixels * 3,
            MALLOC_CAP_DMA
        );

    if (buffer == NULL) {

        ESP_LOGE(
            TAG,
            "Cannot allocate RGB565 buffer"
        );

        return;
    }


    // -------------------------------------------------
    // Convert từng block
    // -------------------------------------------------

    for (int row = 0;
         row < height;
         row += BUFFER_LINES) {

        int current_lines =
            BUFFER_LINES;

        if (row + current_lines > height) {
            current_lines =
                height - row;
        }

        size_t pixels =
            (size_t)width * current_lines;


        for (size_t i = 0;
             i < pixels;
             i++) {

            uint16_t color =
                rgb565[
                    (size_t)row * width + i
                ];

            // RGB565
            uint8_t r5 =
                (color >> 11) & 0x1F;

            uint8_t g6 =
                (color >> 5) & 0x3F;

            uint8_t b5 =
                color & 0x1F;


            // 5 bit -> 8 bit
            uint8_t r8 =
                (r5 << 3) |
                (r5 >> 2);

            uint8_t g8 =
                (g6 << 2) |
                (g6 >> 4);

            uint8_t b8 =
                (b5 << 3) |
                (b5 >> 2);


            buffer[i * 3 + 0] = r8;
            buffer[i * 3 + 1] = g8;
            buffer[i * 3 + 2] = b8;
        }


        lcd_write_pixel_data(
            buffer,
            pixels * 3
        );
    }


    free(buffer);
}


// =====================================================
// PUBLIC INIT
// =====================================================

void ili9488_init(void)
{
    ESP_LOGI(
        TAG,
        "================================"
    );

    ESP_LOGI(
        TAG,
        "ILI9488 initialization"
    );

    ESP_LOGI(
        TAG,
        "ESP32-C3 / SPI / RGB666"
    );

    ESP_LOGI(
        TAG,
        "================================"
    );


    lcd_gpio_init();

    lcd_spi_init();

    lcd_reset();

    ili9488_controller_init();


    ESP_LOGI(
        TAG,
        "ILI9488 ready"
    );
}