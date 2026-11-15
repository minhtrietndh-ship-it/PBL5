#include "decoder.h"
#include "eye_state.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <string.h>
#include "freertos/semphr.h"

// Khai báo thực tế để cấp phát bộ nhớ
int cmd_flag = 0; 
char signal[10] = {'\0'}; // Khởi tạo mảng ký tự
int type_flag=0;
int keyboard_inv=0;
extern int scrollup;
extern int scrolldown;
extern int enable_scroll;
extern SemaphoreHandle_t gpio_semaphore;
signal_t sig=SIGNAL_NONE;
void signal_decoder_task(void *pvParameters) {
    TickType_t press_time = 0;
    TickType_t release_time = 0;
    TickType_t scroll_start_time = 0;
    int scroll_timing_active = 0;
    int last_state = 1; 
    int last_e0_state = 1;
    int double_blink_count = 0;
    TickType_t first_fall_time = 0;
    const TickType_t min_double_ticks = pdMS_TO_TICKS(150);
    const TickType_t max_double_ticks = pdMS_TO_TICKS(600);
    const TickType_t debounce_ticks   = pdMS_TO_TICKS(30); // Chống dội 30ms

    while (1) {
        TickType_t now = xTaskGetTickCount();
        int e0 = eye_state();
        if (e0 == 0 && last_e0_state == 1) {
            if (double_blink_count == 0) {
                // Nháy lần 1
                first_fall_time = now;
                double_blink_count = 1;
            } 
            else if (double_blink_count == 1) {
                // Kiểm tra chống dội (tránh xung nhiễu quá sát < 30ms)
                if ((now - first_fall_time) >= debounce_ticks) {
                    TickType_t delta_ticks = now - first_fall_time;

                    if (delta_ticks >= min_double_ticks && delta_ticks <= max_double_ticks) {
                        // Thỏa mãn điều kiện nháy đúp từ 150ms - 600ms -> Gửi Semaphore
                        if (gpio_semaphore != NULL) {
                            xSemaphoreGive(gpio_semaphore);
                            //printf(">>> Nháy đúp hợp lệ (%lu ms)! Đã gửi Semaphore.\n", (unsigned long)pdTICKS_TO_MS(delta_ticks));
                        }
                        // Reset bộ đếm sau khi thành công
                        double_blink_count = 0;
                        first_fall_time = 0;
                    } 
                    else if (delta_ticks > max_double_ticks) {
                        // Quá thời gian 600ms, coi sườn âm này là lần 1 của chu kỳ mới
                        first_fall_time = now;
                        double_blink_count = 1;
                    } 
                    else {
                        // Quá nhanh (< 150ms), dịch mốc thời gian sang sườn này
                        first_fall_time = now;
                        double_blink_count = 1;
                    }
                }
            }
        }
        // Timeout tự động reset bộ đếm nháy đúp nếu quá 600ms mà không thấy sườn âm thứ 2
        if (double_blink_count == 1 && (now - first_fall_time) > max_double_ticks) {
            double_blink_count = 0;
            first_fall_time = 0;
        }
        last_e0_state = e0;

        if (enable_scroll) {
            int e0 = eye_state();
            int e1 = eye_state1();


            if ((e0 == 0) && (e1 == 1)) {
                if (!scroll_timing_active) {
                    scroll_start_time = xTaskGetTickCount();
                    scroll_timing_active = 1;
                } else if ((xTaskGetTickCount() - scroll_start_time) >= pdMS_TO_TICKS(1000)) {
                    scrollup = 1;
                    //printf("scrollup\n");
                    press_time = xTaskGetTickCount(); // Cập nhật thời gian nhấn khi scrollup được kích hoạt

                }
            }
            else if ((e0 == 1) && (e1 == 0)) {
                if (!scroll_timing_active) {
                    scroll_start_time = xTaskGetTickCount();
                    scroll_timing_active = 1;
                } else if ((xTaskGetTickCount() - scroll_start_time) >= pdMS_TO_TICKS(1000)) {
                    scrolldown = 1;
                    //printf("scrolldown\n");
                    press_time = xTaskGetTickCount(); // Cập nhật thời gian nhấn khi scrolldown được kích hoạt
                }
            } 
            else {
                scroll_timing_active = 0;
                scrollup = 0;
                scrolldown = 0;
            }
        } else {
            scroll_timing_active = 0;
            scrollup = 0;
            scrolldown = 0;
        }
        if (!scrollup && !scrolldown) {
        int current_state = eye_state();
        if (current_state == 0 && last_state == 1) {
            press_time = xTaskGetTickCount();
        } 
        // 2. Phát hiện thời điểm mở mắt trở lại (chuyển từ nhắm sang mở)
        else if (current_state == 1 && last_state == 0) {
            release_time = xTaskGetTickCount();

            // Tính thời gian nhắm mắt (đổi ra mili-giây)
            TickType_t duration_ticks = release_time - press_time;
            uint32_t duration_ms = pdTICKS_TO_MS(duration_ticks);
            if ((duration_ms >=500)&&(duration_ms<=2000)){
                type_flag=1;
            }
            if ((duration_ms>2000)&&(duration_ms<3000)){
              keyboard_inv=1;

            }

            
            // Phân loại tín hiệu theo yêu cầu mã Morse (CN3)
            if (duration_ms >= 1000 && duration_ms <= 2000) {
                //strcpy(signal, "."); // Nhắm trong khoảng 1s - 2s là dấu chấm (.)

                sig = SIGNAL_DOT;
                cmd_flag = 1; // Đặt cờ lệnh khi nhận được tín hiệu chấm


                printf(".\n"); // Nhắm trong khoảng 1s - 2s là dấu chấm (.)
            } 
            else if (duration_ms > 2000 && duration_ms <= 3000) {
                //strcpy(signal, "_"); // Nhắm trên 2s là dấu gạch ngang (_)
                sig = SIGNAL_DASH;
                cmd_flag = 1; // Đặt cờ lệnh khi nhận được tín hiệu gạch ngang
                printf("_\n"); // Nhắm trên 2s là dấu gạch ngang (_)[cite: 3]
            }
            else if (duration_ms > 3000) {
                printf("clear\n"); // Thời gian nhắm quá dài, không hợp lệ
                sig = SIGNAL_CLEAR;
                cmd_flag = 1; // Đặt cờ lệnh khi nhận được tín hiệu xóa
            }
        }

        last_state = current_state;
    }  else {sig = SIGNAL_NONE;}

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}