#ifndef SIGNAL_DECODER_H
#define SIGNAL_DECODER_H


extern char signal[10]; // Biến toàn cục để lưu tín hiệu nháy mắt
extern int cmd_flag; // Biến toàn cục để lưu trạng thái lệnh
typedef enum {
    SIGNAL_NONE = 0,
    SIGNAL_DOT,     // Dấu chấm (.)
    SIGNAL_DASH,    // Dấu gạch (_)
    SIGNAL_CLEAR
} signal_t;
extern signal_t sig; // Biến toàn cục để lưu trạng thái tín hiệu
#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Task xử lý giải mã tín hiệu nháy mắt thành mã Morse (. hoặc _)
 */
void signal_decoder_task(void *pvParameters);

#ifdef __cplusplus
}
#endif

#endif // SIGNAL_DECODER_H