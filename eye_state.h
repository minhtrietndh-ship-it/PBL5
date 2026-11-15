#ifndef EYE_STATE_H
#define EYE_STATE_H
#define SENSOR_PIN GPIO_NUM_4  // Chân cho mắt thứ nhất
#define SENSOR_PIN1 GPIO_NUM_0  // Chân cho mắt thứ hai
#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Khởi tạo chân GPIO 4 cho cảm biến
 */
void eye_sensor_init(void);

/**
 * @brief Đọc trạng thái cảm biến
 * @return int Trả về 0 hoặc 1
 */
int eye_state(void);
int eye_state1(void);

#ifdef __cplusplus
}
#endif

#endif // EYE_STATE_H