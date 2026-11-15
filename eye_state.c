#include "eye_state.h"
#include "driver/gpio.h"

// Định nghĩa các chân GPIO kết nối với cảm biến
#define SENSOR_PIN  GPIO_NUM_4  // Chân cho mắt thứ nhất
#define SENSOR_PIN1 GPIO_NUM_0  // Chân cho mắt thứ hai

void eye_sensor_init(void) {
    // 1. Cấu hình SENSOR_PIN làm chân ngắt (bắt sườn âm 1 -> 0)
    gpio_config_t io_conf_sensor = {
        .pin_bit_mask = (1ULL << SENSOR_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLDOWN_DISABLE,      
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE      // Kích hoạt ngắt sườn âm (1 -> 0)
    };
    gpio_config(&io_conf_sensor);

    // 2. Cấu hình SENSOR_PIN1 làm đầu vào thông thường (không ngắt)
    gpio_config_t io_conf_sensor1 = {
        .pin_bit_mask = (1ULL << SENSOR_PIN1),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLDOWN_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE        // Không dùng ngắt
    };
    gpio_config(&io_conf_sensor1);

    // 3. Cài đặt dịch vụ ngắt GPIO (chỉ gọi 1 lần duy nhất trong toàn

    // 4. Đăng ký hàm xử lý ngắt (ISR handler) cho SENSOR_PIN (nếu bạn dùng hàm ngắt riêng)
    // gpio_isr_handler_add(SENSOR_PIN, gpio_isr_handler, NULL);
}

// Hàm đọc trạng thái mắt 1 (GPIO 3)
int eye_state(void) {
    return gpio_get_level(SENSOR_PIN);
}

// Hàm đọc trạng thái mắt 2 (GPIO 0)
int eye_state1(void) {
    return gpio_get_level(SENSOR_PIN1);
}