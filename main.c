//nham 3s- thoat
//
//interupt to double click
//save tin nhan
//ban phim 2
//nimble
//Sleep
//exit
//
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/event_groups.h"
#include "driver/gpio.h"
#include "eye_state.h"
#include "decoder.h"
#include "string.h"
#include "freertos/timers.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "mqtt_client.h"
#include "WiFi.h"
#include "nvs_flash.h"  
#include "driver/uart.h"
#include "ili9488.h"
#include "lvgl.h"
#include "esp_timer.h"
#include "esp_bt.h"
#if CONFIG_BT_NIMBLE_ENABLED
#include "host/ble_hs.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#else
#include "esp_bt_defs.h"
#if CONFIG_BT_BLE_ENABLED
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_defs.h"
#endif
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#endif
#include "esp_hidd.h"
#include "esp_hid_gap.h"
#include "eye_state.h"
#include "esp_now_protocol.h"
#include "esp_now.h"
static const char *TAG = "MAIN";
static const char *TAG2 = "A7680C_TEST";
static const char *TAG3 = "HID_DEV_DEMO";
#define TIME_MIN_US     (150 * 1000)    // 150 ms
#define TIME_MAX_US     (600 * 1000)    // 600 ms
#define DEBOUNCE_US     (20 * 1000)
SemaphoreHandle_t gpio_semaphore = NULL;
#define MAX_INBOX_MSG 10
#define INBOX_MSG_LEN 100

typedef struct {
    char content[INBOX_MSG_LEN];
    bool is_read; // false: Tin mới, true: Đã đọc
} mqtt_inbox_t;

mqtt_inbox_t inbox[MAX_INBOX_MSG];
int inbox_count = 0;      // Tổng số tin nhắn đang có
int inbox_index = 0;      // Vị trí con trỏ khi cuộn màn hình
int last_inbox_index = -1; // Dùng cho lvgl_task để cập nhật UI
int mailbox_index = 0;      // Vị trí đang trỏ trong danh sách hộp thư

/*// Biến lưu trữ trạng thái ngắt
static volatile int64_t s_first_fall_time = 0;
static volatile uint8_t s_fall_counter = 0;
// Hàm ISR xử lý ngắt GPIO
static void IRAM_ATTR gpio_isr_handler(void *arg) {
    int64_t current_time = esp_timer_get_time(); // Lấy thời gian hiện tại (µs)
    
    // Chống dội (debounce): Bỏ qua nếu các xung diễn ra quá gần nhau (< 20ms)
    if (s_first_fall_time != 0 && (current_time - s_first_fall_time) < DEBOUNCE_US && s_fall_counter == 1) {
        return;
    }

    if (s_fall_counter == 0) {
        // Ghi nhận sườn âm thứ nhất
        s_first_fall_time = current_time;
        s_fall_counter = 1;
    } 
    else if (s_fall_counter == 1) {
        // Ghi nhận sườn âm thứ hai
        int64_t delta_time = current_time - s_first_fall_time;

        if (delta_time >= TIME_MIN_US && delta_time <= TIME_MAX_US) {
            // Đạt điều kiện: 2 lần 1->0 trong khoảng 150ms - 600ms
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            xSemaphoreGiveFromISR(gpio_semaphore, &xHigherPriorityTaskWoken);
            
            // Reset bộ đếm
            s_fall_counter = 0;
            s_first_fall_time = 0;
            
            if (xHigherPriorityTaskWoken) {
                portYIELD_FROM_ISR();
            }
        } 
        else if (delta_time > TIME_MAX_US) {
            // Quá thời gian 600ms, coi sườn âm này là sườn thứ nhất của chu kỳ mới
            s_first_fall_time = current_time;
            s_fall_counter = 1;
        } 
        else {
            // Quá nhanh (< 150ms), có thể là nhiễu hoặc bấm lỗi, reset hoặc giữ nguyên tùy chiến lược
            // Ở đây ta dịch chuyển mốc thời gian sang sườn này để bắt đầu lại
            s_first_fall_time = current_time;
            s_fall_counter = 1;
        }
    }
}*/
#include "freertos/queue.h"

//QueueHandle_t health_queue;
typedef struct
{
    TaskHandle_t task_hdl;
    esp_hidd_dev_t *hid_dev;
    uint8_t protocol_mode;
    uint8_t *buffer;
} local_param_t;
#if CONFIG_BT_BLE_ENABLED || CONFIG_BT_NIMBLE_ENABLED
static local_param_t s_ble_hid_param = {0};
static TaskHandle_t combo_task_hdl = NULL; // Quản lý Task riêng để tránh tạo chồng chéo

/* --- 1. REPORT MAPS CHO BÀN PHÍM, CHUỘT VÀ MEDIA --- */
const uint8_t keyboardReportMap[] = {
    0x05, 0x01,       // Usage Page (Generic Desktop)
    0x09, 0x06,       // Usage (Keyboard)
    0xA1, 0x01,       // Collection (Application)

    0x85, 0x01,       // Report ID (1)

    // Modifier: Ctrl, Shift, Alt, GUI
    0x05, 0x07,       // Usage Page (Keyboard)
    0x19, 0xE0,       // Usage Minimum
    0x29, 0xE7,       // Usage Maximum
    0x15, 0x00,       // Logical Minimum
    0x25, 0x01,       // Logical Maximum
    0x75, 0x01,       // Report Size 1 bit
    0x95, 0x08,       // Report Count 8
    0x81, 0x02,       // Input

    // Reserved byte
    0x95, 0x01,
    0x75, 0x08,
    0x81, 0x03,

    // LED Output
    0x95, 0x05,
    0x75, 0x01,
    0x05, 0x08,
    0x19, 0x01,
    0x29, 0x05,
    0x91, 0x02,

    // LED Padding
    0x95, 0x01,
    0x75, 0x03,
    0x91, 0x03,

    // 6 Keycodes
    0x95, 0x06,
    0x75, 0x08,
    0x15, 0x00,
    0x25, 0x65,
    0x05, 0x07,
    0x19, 0x00,
    0x29, 0x65,
    0x81, 0x00,

    0xC0
};
const unsigned char mouseReportMap[] = {
    0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
    0x09, 0x02,                    // USAGE (Mouse)
    0xa1, 0x01,                    // COLLECTION (Application)
    0x85, 0x02,                    //   Report ID (2)
    0x09, 0x01,                    //   USAGE (Pointer)
    0xa1, 0x00,                    //   COLLECTION (Physical)
    0x05, 0x09,                    //     USAGE_PAGE (Button)
    0x19, 0x01,                    //     USAGE_MINIMUM (Button 1)
    0x29, 0x03,                    //     USAGE_MAXIMUM (Button 3)
    0x15, 0x00,                    //     LOGICAL_MINIMUM (0)
    0x25, 0x01,                    //     LOGICAL_MAXIMUM (1)
    0x95, 0x03,                    //     REPORT_COUNT (3)
    0x75, 0x01,                    //     REPORT_SIZE (1)
    0x81, 0x02,                    //     INPUT (Data,Var,Abs)
    0x95, 0x01,                    //     REPORT_COUNT (1)
    0x75, 0x05,                    //     REPORT_SIZE (5)
    0x81, 0x03,                    //     INPUT (Cnst,Var,Abs)
    0x05, 0x01,                    //     USAGE_PAGE (Generic Desktop)
    0x09, 0x30,                    //     USAGE (X)
    0x09, 0x31,                    //     USAGE (Y)
    0x09, 0x38,                    //     USAGE (Wheel)
    0x15, 0x81,                    //     LOGICAL_MINIMUM (-127)
    0x25, 0x7f,                    //     LOGICAL_MAXIMUM (127)
    0x75, 0x08,                    //     REPORT_SIZE (8)
    0x95, 0x03,                    //     REPORT_COUNT (3)
    0x81, 0x06,                    //     INPUT (Data,Var,Rel)
    0xc0,                          //   END_COLLECTION
    0xc0                           // END_COLLECTION
};

const unsigned char mediaReportMap[] = {
    0x05, 0x0C,        // Usage Page (Consumer)
    0x09, 0x01,        // Usage (Consumer Control)
    0xA1, 0x01,        // Collection (Application)
    0x85, 0x03,        //   Report ID (3)
    0x09, 0xE2,        //   Usage (Mute)
    0x09, 0xE9,        //   Usage (Volume Increment)
    0x09, 0xEA,        //   Usage (Volume Decrement)
    0x15, 0x00,        //   Logical Minimum (0)
    0x25, 0x01,        //   Logical Maximum (1)
    0x75, 0x01,        //   Report Size (1)
    0x95, 0x03,        //   Report Count (3)
    0x81, 0x02,        //   Input (Data,Var,Abs)
    0x95, 0x05,        //   Report Count (5)
    0x81, 0x03,        //   Input (Const,Var,Abs)
    0xC0,              // End Collection
};
typedef enum {
    COMP_MODE_SCROLL,       // Chế độ 0: Cuộn trang (Lên / Xuống)
    COMP_MODE_VERTICAL,     // Chế độ 1: Di chuyển chuột dọc (Lên / Xuống)
    COMP_MODE_HORIZONTAL,   // Chế độ 2: Di chuyển chuột ngang (Trái / Phải)
    COMP_MODE_KEYBOARD,     // Chế độ 3: Gõ bàn phím
    COMP_MODE_MAX
} computer_sub_mode_t;
TaskHandle_t sos_task_hdl = NULL;
SemaphoreHandle_t sos_semaphore = NULL;
/*#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
static void node1_espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
#else
static void node1_espnow_recv_cb(const uint8_t *mac_addr, const uint8_t *data, int len)
#endif 
{
    if (len == sizeof(health_data_packet_t)) {
        health_data_packet_t *incoming_data = (health_data_packet_t *)data;
        
        if (incoming_data->msg_type == MSG_TYPE_DATA) {
            // Đẩy gói dữ liệu vào Queue (không chờ - wait = 0)
            xQueueSend(health_queue, incoming_data, 0);
        }
    }
}*/
computer_sub_mode_t computer_mode = COMP_MODE_VERTICAL; // Mặc định là chế độ dọc
// Hàm gửi dữ liệu chuột (Report ID = 2, map_index = 1)
void send_mouse(uint8_t buttons, char dx, char dy, char wheel)
{
    uint8_t buffer[4] = {0};
    buffer[0] = buttons;
    buffer[1] = dx;
    buffer[2] = dy;
    buffer[3] = wheel;
    esp_hidd_dev_input_set(s_ble_hid_param.hid_dev, 1, 2, buffer, 4);
}

// Chuyển ký tự thành mã phím bàn phím
#define USB_HID_MODIFIER_LEFT_SHIFT     0x02
#define CASE(a, b, c) case a: buffer[0] = b; buffer[2] = c; break;

static void char_to_code(uint8_t *buffer, char ch)
{
    if(ch >= 'a' && ch <= 'z') {
        buffer[0] = 0;
        buffer[2] = (uint8_t)(4 + (ch - 'a'));
    } else if(ch >= 'A' && ch <= 'Z') {
        buffer[0] = USB_HID_MODIFIER_LEFT_SHIFT;
        ch = ch - ('A'-'a');
        buffer[2] = (uint8_t)(4 + (ch - 'a'));
    } else if(ch >= '0' && ch <= '9') {
        buffer[0] = 0;
        if(ch == '0') buffer[2] = 39;
        else buffer[2] = (uint8_t)(30 + (ch - '1'));
    } else {
        switch(ch) {
            CASE(' ', 0, 0x2C);
            CASE('\n', 0, 0x28);
            default:
                buffer[0] = 0;
                buffer[2] = 0;
        }
    }
}
void save_incoming_message(const char* data, int data_len) {
    if (data == NULL || data_len <= 0) return;

    // 1. Tính toán kích thước an toàn (Cắt bỏ phần thừa nếu tin nhắn dài hơn sức chứa)
    int copy_len = data_len;
    if (copy_len >= INBOX_MSG_LEN) {
        copy_len = INBOX_MSG_LEN - 1; // Chừa 1 byte cho '\0'
        //printf("Cảnh báo: Tin nhắn quá dài, đã bị cắt bớt!\n");
        ESP_LOGW(TAG, "Cảnh báo: Tin nhắn quá dài, đã bị cắt bớt!");
    }

    // 2. Dùng memcpy thay vì strncpy vì 'data' từ MQTT không có '\0'
    char safe_msg[INBOX_MSG_LEN];
    memcpy(safe_msg, data, copy_len);
    safe_msg[copy_len] = '\0'; // CHỐT CHẶN BẢO VỆ CHUỖI

    // 3. Xử lý xóa tin cũ nếu hộp thư đầy
    if (inbox_count >= MAX_INBOX_MSG) {
        int delete_idx = -1;
        for (int i = 0; i < MAX_INBOX_MSG; i++) {
            if (inbox[i].is_read) {
                delete_idx = i;
                break;
            }
        }
        if (delete_idx == -1) delete_idx = 0;

        for (int i = delete_idx; i < MAX_INBOX_MSG - 1; i++) {
            inbox[i] = inbox[i + 1];
        }
        inbox_count--;
    }

    // 4. Lưu tin an toàn bằng strcpy vì safe_msg đã có '\0'
    strcpy(inbox[inbox_count].content, safe_msg);
    inbox[inbox_count].is_read = false;
    inbox_count++;
    
    ESP_LOGI(TAG, "Đã lưu an toàn tin nhắn: %s", inbox[inbox_count-1].content);
}
// Hàm gửi dữ liệu bàn phím (Report ID = 1, map_index = 0)
void send_keyboard(char c)
{
    uint8_t buffer[8] = {0};
    char_to_code(buffer, c);
    
    // Gửi phím xuống
    esp_hidd_dev_input_set(s_ble_hid_param.hid_dev, 0, 1, buffer, 8);
    vTaskDelay(20 / portTICK_PERIOD_MS);
    
    // Gói tin nhả phím (Release) bắt buộc
    memset(buffer, 0, sizeof(uint8_t) * 8);
    esp_hidd_dev_input_set(s_ble_hid_param.hid_dev, 0, 1, buffer, 8);
}

// Task chạy kết hợp gõ bàn phím và lắc chuột luân phiên
void ble_hid_demo_task_combined(void *pvParameters)
{
    vTaskDelay(2000 / portTICK_PERIOD_MS); // Chờ kết nối Bluetooth ổn định hẳn

    while (1) {
        // Gửi phím 'e'
        send_keyboard('e');
        ESP_LOGI(TAG, "Sent key: e");

        vTaskDelay(1000 / portTICK_PERIOD_MS);

        // Di chuyển chuột nhẹ nhàng để test cả chuột
        send_mouse(0, 0, 5, 0);
        vTaskDelay(30 / portTICK_PERIOD_MS);
        send_mouse(0, 0, -5, 0);

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
#endif
#define PC_KB_ROWS 6
#define PC_KB_COLS 5
#define PC_KB_MAX_ITEMS (PC_KB_ROWS * PC_KB_COLS)

static lv_obj_t *pc_keyboard_btns[PC_KB_MAX_ITEMS];
static int pc_key_index = 0; // Vị trí phím đang trỏ tới khi ở chế độ gõ phím PC

// Mảng ký tự cho bàn phím điều khiển máy tính
static const char *pc_keyboard_keys[PC_KB_MAX_ITEMS] = {
    "a", "b", "c", "d", "e",
    "f", "g", "h", "i", "j",
    "k", "l", "m", "n", "o",
    "p", "q", "r", "s", "t",
    "u", "v", "w", "x", "y",
    "z", " ", "\n", "BS", "OK"
};

/* --- 2. KHAI BÁO REPORT MAPS ARRAY --- */
static esp_hid_raw_report_map_t ble_report_maps[] = {
    {
        .data = keyboardReportMap,
        .len = sizeof(keyboardReportMap)
    },
    {
        .data = mouseReportMap,
        .len = sizeof(mouseReportMap)
    },
    {
        .data = mediaReportMap,
        .len = sizeof(mediaReportMap)
    }
};

static esp_hid_device_config_t ble_hid_config = {
    .vendor_id          = 0x16C0,
    .product_id         = 0x05DF,
    .version            = 0x0100,
    .device_name        = "ESP Combo HID",
    .manufacturer_name  = "Espressif",
    .serial_number      = "1234567890",
    .report_maps        = ble_report_maps,
    .report_maps_len    = 3
};

void ble_hid_task_start_up(void)
{
    if (combo_task_hdl != NULL) {
        return;
    }
}

void ble_hid_task_shut_down(void)
{
    if (combo_task_hdl) {
        vTaskDelete(combo_task_hdl);
        combo_task_hdl = NULL;
    }
}

void sos_emergency_task(void *pvParameters) {
    while (1) {
        // Chờ tín hiệu kích hoạt SOS (chờ vô hạn đến khi có lệnh)
        if (xSemaphoreTake(sos_semaphore, portMAX_DELAY) == pdTRUE) {
            ESP_LOGW("SOS", "!!! PHÁT HIỆN TÍN HIỆU KHẨN CẤP SOS !!! Đang thực thi...");
            ESP_LOGI("SOS", "Đã gửi tín hiệu cầu cứu khẩn cấp thành công.");

        }
        vTaskDelay(1000);
    }
}
static void ble_hidd_event_callback(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
    esp_hidd_event_t event = (esp_hidd_event_t)id;
    esp_hidd_event_data_t *param = (esp_hidd_event_data_t *)event_data;
    static const char *TAG_EVT = "HID_DEV_BLE";

    switch (event) {
    case ESP_HIDD_START_EVENT:
        ESP_LOGI(TAG_EVT, "START");
        esp_hid_ble_gap_adv_start();
        break;
    case ESP_HIDD_CONNECT_EVENT:
        ESP_LOGI(TAG_EVT, "CONNECT");
        break;
    case ESP_HIDD_CONTROL_EVENT:
        if (param->control.control) {
            ble_hid_task_start_up();
        } else {
            ble_hid_task_shut_down();
        }
        break;
    case ESP_HIDD_DISCONNECT_EVENT:
        ESP_LOGI(TAG_EVT, "DISCONNECT");
        ble_hid_task_shut_down();
        esp_hid_ble_gap_adv_start();
        break;
    default:
        break;
    }
}

#if CONFIG_BT_NIMBLE_ENABLED
void ble_hid_device_host_task(void *param)
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}
void ble_store_config_init(void);
#endif

char currentCommand[10] = "";
unsigned long lastSignalTime = 0;
const unsigned long COMMAND_TIMEOUT = 3000;
char exe_command[10] = "";    
bool execute_ready = false;
int scrollup = 0;
int scrolldown = 0;
int enable_scroll = 0; 
int number_index=0;
void run_choice();
void create_message();
void run_start_menu();
#define MQTT_TOPIC  "message"
extern esp_mqtt_client_handle_t client;

#define KEYBOARD_ROWS 6
#define KEYBOARD_COLS 5
#define KEYBOARD_MAX_ITEMS (KEYBOARD_ROWS * KEYBOARD_COLS)
static lv_obj_t *keyboard_btns[KEYBOARD_MAX_ITEMS]; 
static lv_obj_t *txt_box_global = NULL;             
#define TXD_PIN (GPIO_NUM_18) // Chân nối với RX của A7680C
#define RXD_PIN (GPIO_NUM_19) // Chân nối với TX của A7680C
#define UART_PORT_NUM UART_NUM_1
#define BUF_SIZE (1024)
#define MAX_CONTACTS 5     // Số lượng liên hệ tối đa lưu trong NVS
#define MAX_NAME_LEN 32    // Độ dài tối đa của tên
#define MAX_PHONE_LEN 16   // Độ dài tối đa của số điện thoại
static int cached_contact_count = 0;
typedef struct {
    char name[MAX_NAME_LEN];
    char phone[MAX_PHONE_LEN];
} Contact_t;
static Contact_t cached_contacts[MAX_CONTACTS];
Contact_t current_num;
extern int type_flag;
int key_dir=0;
extern int keyboard_inv;


// Danh sách các phím bao gồm Số, Chữ cái cơ bản, Dấu câu và Phím điều khiển
static const char *keyboard_keys[KEYBOARD_MAX_ITEMS] = {
    "A", "B", "C", "D",
    "E", "F", "G", "H", "I", "J", "K",
    ".", ",", "L", "M", "N", "P", "Q",
    "R", "S", "T", "U", "W", "X", "Y",
    "Z", "CLR", "enter", "LUU", "___"
};
#define MAX_LEN 100
char message_buffer[MAX_LEN] = "";
enum MENU {
    START_MENU,
    CHOISE,
    CHOOSE_NUMBER_MESSAGE,
    CREATE_MESSAGE,
    MAIL_BOX,
    NOTE,
    COMPUTER,
    SOS,
    SETTING
} menu_state = START_MENU;
TimerHandle_t timeout_timer;
enum CHOICE{
    CHOICE1,
    CHOICE2,
    CHOICE3,
    CHOICE4,
    CHOICE5,
    CHOICE_MAX_ITEMS
} choice_state = CHOICE1;
int choice_index = 0;
int key_index=0;
static int start_menu_index = 0;
#define START_MENU_MAX_ITEMS 6
static lv_obj_t *start_menu_btns[START_MENU_MAX_ITEMS];

// Mảng ánh xạ index sang các trạng thái menu tương ứng
static const enum MENU start_menu_states[START_MENU_MAX_ITEMS] = {
    CHOISE,
    CREATE_MESSAGE,
    MAIL_BOX,
    NOTE,
    COMPUTER,
    SOS,
    SETTING
};

static const char *start_menu_names[START_MENU_MAX_ITEMS] = {
    "CHOICE", "CREATE_MESSAGE", "MAIL_BOX", "NOTE", "COMPUTER", "SOS", "SETTING"
};

static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf1[LCD_WIDTH * 5];
static lv_disp_drv_t disp_drv;


void init_a7680c_uart(void) {
    uart_config_t uart_config = {
        .baud_rate = 115200, 
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    
    uart_param_config(UART_PORT_NUM, &uart_config);
    uart_set_pin(UART_PORT_NUM, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);
    ESP_LOGI(TAG2, "Đã cấu hình UART cho A7680C thành công.");
}

void send_at_command(const char* cmd) {
    char buffer[128];
    int len = snprintf(buffer, sizeof(buffer), "%s\r\n", cmd);
    uart_write_bytes(UART_PORT_NUM, buffer, len);
    ESP_LOGI(TAG2, "Đã gửi: %s", cmd);
}
static uint8_t node2_mac[6] = {0x34, 0xCD, 0xB0, 0xE9, 0x96, 0xC8}; 
static enum MENU last_sent_menu_state = -1;

// Hàm gửi lệnh Cho phép / Dừng gửi dữ liệu
void request_health_data(uint8_t command) {
    cmd_packet_t cmd_pkt = {
        .msg_type = MSG_TYPE_CMD,
        .command = command
    };
    esp_now_send(node2_mac, (uint8_t *)&cmd_pkt, sizeof(cmd_pkt));
    esp_err_t result = esp_now_send(node2_mac, (uint8_t *)&cmd_pkt, sizeof(cmd_pkt));
if (result == ESP_OK) {
    ESP_LOGI("ESP_NOW", "Node 1: Đã bắn lệnh sang Node 2 thành công!");
} else {
    ESP_LOGE("ESP_NOW", "Node 1: Gửi thất bại, mã lỗi: 0x%X", result);
}
}

// Gọi hàm này trong Task xử lý Menu của bạn
/*void check_and_control_health_stream() {
    if (menu_state != last_sent_menu_state) {
        if (menu_state == HEALTH_MENU) {
            // Khi vào Menu Health -> Ra lệnh BẬT
            request_health_data(CMD_START_STREAM);
        } 
        else {
            // BẤT KỲ KHI NÀO thoát khỏi Menu Health sang menu khác -> Ra lệnh TẮT
            request_health_data(CMD_STOP_STREAM);
        }
        last_sent_menu_state = menu_state;
    }
}*/
// Task đọc phản hồi từ module A7680C trả về
void a7680c_read_task(void *pvParameters) {
    uint8_t *data = (uint8_t *)malloc(BUF_SIZE);
    while (1) {
        int length = uart_read_bytes(UART_PORT_NUM, data, BUF_SIZE, pdMS_TO_TICKS(100));
        if (length > 0) {
            data[length] = '\0';
            ESP_LOGI(TAG2, "Phản hồi từ module:\n%s", (char *)data);
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    free(data);
}
// Hàm gửi tin nhắn SMS
void send_sms(const char* phone_number, const char* message) {
    char cmd_buffer[128];

    // Bước 1: Chuyển sang chế độ Text Mode
    send_at_command("AT+CMGF=1");
    vTaskDelay(pdMS_TO_TICKS(500));

    // Bước 2: Gửi lệnh AT+CMGS kèm số điện thoại
    snprintf(cmd_buffer, sizeof(cmd_buffer), "AT+CMGS=\"%s\"", phone_number);
    uart_write_bytes(UART_PORT_NUM, cmd_buffer, strlen(cmd_buffer));
    uart_write_bytes(UART_PORT_NUM, "\r\n", 2);
    ESP_LOGI("SMS", "Đang gửi yêu cầu đến số: %s", phone_number);
    
    // Đợi module phản hồi ký tự '>'
    vTaskDelay(pdMS_TO_TICKS(500));

    // Bước 3: Gửi nội dung tin nhắn kết thúc bằng Ctrl+Z (0x1A)
    uart_write_bytes(UART_PORT_NUM, message, strlen(message));
    char ctrl_z = 0x1A; // Ký tự Ctrl+Z báo hiệu hoàn tất nội dung tin nhắn
    uart_write_bytes(UART_PORT_NUM, &ctrl_z, 1);
    
    ESP_LOGI("SMS", "Đã gửi nội dung và lệnh kết thúc (Ctrl+Z).");
}

esp_err_t save_contact_list(Contact_t contacts[], int count) {
    nvs_handle_t handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &handle);
    if (err != ESP_ERR_NVS_NOT_FOUND) { // Sửa lại điều kiện kiểm tra lỗi mở NVS chuẩn
        // Nếu mở thất bại hoàn toàn thì return err
    }
    // Thực tế chuẩn hơn:
    err = nvs_open("storage", NVS_READWRITE, &handle);
    if (err != ESP_OK) return err;

    // Lưu tổng số lượng liên hệ
    nvs_set_i32(handle, "cnt_total", count);

    // Lưu từng tên và số điện thoại
    for (int i = 0; i < count; i++) {
        char name_key[32];
        char phone_key[32];

        snprintf(name_key, sizeof(name_key), "name_%d", i);
        snprintf(phone_key, sizeof(phone_key), "phone_%d", i);

        nvs_set_str(handle, name_key, contacts[i].name);
        nvs_set_str(handle, phone_key, contacts[i].phone);
    }

    err = nvs_commit(handle);
    nvs_close(handle);
    return err;
}

// LVGL tick timer callback
static void lv_tick_task(void *arg)
{
    (void)arg;
    lv_tick_inc(1);
}

// Display flush callback
static void lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_p)
{
    int width = area->x2 - area->x1 + 1;
    int height = area->y2 - area->y1 + 1;

    if (width <= 0 || height <= 0) {
        lv_disp_flush_ready(drv);
        return;
    }

    ili9488_flush_rgb565(
        area->x1,
        area->y1,
        area->x2,
        area->y2,
        (const uint16_t *)color_p
    );

    lv_disp_flush_ready(drv);
}

void command_task(void *pvParameters){
    while (1) {
        if (cmd_flag == 1) {
            if (sig == SIGNAL_DOT) {
                strcat(currentCommand, ".");
            } else if (sig == SIGNAL_DASH) {
                strcat(currentCommand, "_");
            }
            else if (sig == SIGNAL_CLEAR) {
                currentCommand[0] = '\0';
            }
            printf("Current Command: %s\n", currentCommand);
            cmd_flag = 0;
            
        }

        if (eye_state() ==0 || eye_state1() == 0) {
            xTimerReset(timeout_timer, 0); // Reset bộ đếm thời gian khi nhắm mắt
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
};

void computer(void){
    // 1. Xử lý chuyển đổi qua lại giữa các chế độ bằng cách chớp mắt 2 lần (Semaphore)
    if (xSemaphoreTake(gpio_semaphore, 0) == pdTRUE) {
        computer_mode = (computer_mode + 1) % COMP_MODE_MAX;
        ESP_LOGI("COMPUTER", "Chuyển sang chế độ: %d", computer_mode);
        
        // Nếu chuyển sang chế độ bàn phím, reset lại index trỏ phím
        if (computer_mode == COMP_MODE_KEYBOARD) {
            pc_key_index = 0;
        }
    }

    switch(computer_mode){
        case COMP_MODE_SCROLL:
            if (scrollup) {
                send_mouse(0, 0, 0, 1);
                vTaskDelay(pdMS_TO_TICKS(50));
            } else if (scrolldown) {
                send_mouse(0, 0, 0, -1);
                vTaskDelay(pdMS_TO_TICKS(50));
            }
            break;

        case COMP_MODE_VERTICAL:
            if (scrollup) {
                send_mouse(0, 0, -10, 0);
                vTaskDelay(pdMS_TO_TICKS(30));
            } else if (scrolldown) {
                send_mouse(0, 0, 10, 0);
                vTaskDelay(pdMS_TO_TICKS(30));
            }
            break;

        case COMP_MODE_HORIZONTAL:
            if (scrollup) {
                send_mouse(0, -10, 0, 0);
                vTaskDelay(pdMS_TO_TICKS(30));
            } else if (scrolldown) {
                send_mouse(0, 10, 0, 0);
                vTaskDelay(pdMS_TO_TICKS(30));
            }
            break;

        case COMP_MODE_KEYBOARD:
            // Di chuyển con trỏ chọn phím dựa trên tín hiệu cuộn lên/xuống
            if (scrollup) {
                pc_key_index = (pc_key_index - 1 + PC_KB_MAX_ITEMS) % PC_KB_MAX_ITEMS;
                ESP_LOGI("PC_KB", "Chọn phím: %s", pc_keyboard_keys[pc_key_index]);
                vTaskDelay(pdMS_TO_TICKS(300));
            } else if (scrolldown) {
                pc_key_index = (pc_key_index + 1) % PC_KB_MAX_ITEMS;
                ESP_LOGI("PC_KB", "Chọn phím: %s", pc_keyboard_keys[pc_key_index]);
                vTaskDelay(pdMS_TO_TICKS(300));
            }

            // Xác nhận chọn phím bằng hành động nhắm mắt (type_flag)
            if (type_flag) {
                const char *selected_key = pc_keyboard_keys[pc_key_index];
                
                if (strcmp(selected_key, "BS") == 0) {
                    // Gửi phím Backspace (Mã ASCII hoặc cấu hình HID tương ứng nếu có)
                    send_keyboard('\b');
                } else if (strcmp(selected_key, "OK") == 0) {
                    send_keyboard('\n');
                } else {
                    // Gửi ký tự đơn thông qua hàm send_keyboard đã có sẵn
                    send_keyboard(selected_key[0]);
                    ESP_LOGI("PC_KB", "Đã gửi ký tự tới PC: %c", selected_key[0]);
                }
                
                type_flag = 0; // Reset cờ
            }
            break;

        default: 
            break;
    }

    // Xử lý Click chuột trái thông thường nếu không phải chế độ bàn phím và có cờ type_flag
    if (type_flag && computer_mode != COMP_MODE_KEYBOARD) {
        send_mouse(1, 0, 0, 0); 
        vTaskDelay(pdMS_TO_TICKS(50));
        send_mouse(0, 0, 0, 0); 
        type_flag = 0; 
    }

    vTaskDelay(pdMS_TO_TICKS(100));
}


void check_and_execute_command() {

    strcpy(exe_command, currentCommand);
    execute_ready = true; 

    currentCommand[0] = '\0';
}

void run_start_menu() {
if (type_flag) {
        // Lưu tạm giá trị menu tiếp theo trước khi reset index
        enum MENU next_menu = start_menu_states[start_menu_index];
        
        type_flag = 0;         // Reset cờ
        start_menu_index = 0;  // Reset về đầu
        
        menu_state = next_menu; // Giao quyền chuyển state sau cùng
        ESP_LOGI(TAG, "Đã chọn Start Menu -> Chuyển đến menu state: %d", menu_state);
        //printf("Đã chọn Start Menu -> Chuyển đến menu state: %d\n", menu_state);
    }
    else if (scrollup) {
        start_menu_index = (start_menu_index - 1 + START_MENU_MAX_ITEMS) % START_MENU_MAX_ITEMS;
        ESP_LOGI(TAG, "Start Menu - Cuộn lên: %d (%s)", start_menu_index, start_menu_names[start_menu_index]);
        vTaskDelay(pdMS_TO_TICKS(300)); // Chống trôi (debounce)
    }
    else if (scrolldown) {
        start_menu_index = (start_menu_index + 1) % START_MENU_MAX_ITEMS;
        ESP_LOGI(TAG, "Start Menu - Cuộn xuống: %d (%s)", start_menu_index, start_menu_names[start_menu_index]);
        vTaskDelay(pdMS_TO_TICKS(300)); // Chống trôi (debounce)
    }
}

int load_contact_list(Contact_t contacts[], int max_count) {
    nvs_handle_t handle;
    if (nvs_open("storage", NVS_READONLY, &handle) != ESP_OK) return 0;

    int32_t count = 0;
    // Đọc tổng số lượng đã lưu
    if (nvs_get_i32(handle, "cnt_total", &count) != ESP_OK) {
        nvs_close(handle);
        return 0;
    }

    if (count > max_count) count = max_count;

    // Đọc từng tên và số điện thoại theo index
    for (int i = 0; i < count; i++) {
        char name_key[32];
        char phone_key[32];

        snprintf(name_key, sizeof(name_key), "name_%d", i);
        snprintf(phone_key, sizeof(phone_key), "phone_%d", i);

        size_t name_len = MAX_NAME_LEN;
        size_t phone_len = MAX_PHONE_LEN;

        nvs_get_str(handle, name_key, contacts[i].name, &name_len);
        nvs_get_str(handle, phone_key, contacts[i].phone, &phone_len);
    }

    nvs_close(handle);
    return count; // Trả về tổng số liên hệ đọc thành công
}

// Hàm thêm 1 liên hệ mới vào danh bạ đang lưu trên NVS
esp_err_t add_single_contact(const char *new_name, const char *new_phone) {
    Contact_t contacts[MAX_CONTACTS];
    
    // Bước 1: Đọc danh bạ hiện tại lên RAM (nếu chưa có thì current_count = 0)
    int current_count = load_contact_list(contacts, MAX_CONTACTS);

    // Bước 2: Kiểm tra xem bộ nhớ danh bạ đã đầy chưa
    if (current_count >= MAX_CONTACTS) {
        ESP_LOGW("CONTACT", "Danh bạ đã đầy, không thể thêm mới!");
        return ESP_ERR_NO_MEM;
    }

    // Bước 3: Gán thông tin mới vào cuối mảng
    snprintf(contacts[current_count].name, MAX_NAME_LEN, "%s", new_name);
    snprintf(contacts[current_count].phone, MAX_PHONE_LEN, "%s", new_phone);
    current_count++; // Tăng số lượng lên 1

    // Bước 4: Lưu ngược toàn bộ danh sách mới xuống NVS
    return save_contact_list(contacts, current_count);
}

void init_contact_cache(void) {
    cached_contact_count = load_contact_list(cached_contacts, MAX_CONTACTS);
    ESP_LOGI("CONTACT", "Đã nạp %d liên hệ vào RAM cache.", cached_contact_count);
}

// 2. Hàm thêm mới: Vừa ghi xuống NVS, vừa cập nhật luôn vào RAM cache
esp_err_t add_and_update_cache(const char *name, const char *phone) {
    // Gọi hàm thêm vào NVS đã viết trước đó
    esp_err_t err = add_single_contact(name, phone);
    
    if (err == ESP_OK) {
        // Cập nhật lại cache trên RAM ngay lập tức để UI nhận dữ liệu mới luôn
        cached_contact_count = load_contact_list(cached_contacts, MAX_CONTACTS);
        ESP_LOGI("CONTACT", "Đã cập nhật lại cache RAM! Tổng số: %d", cached_contact_count);
    }
    return err;
}
void run_choice() {
    // Thay vì check execute_ready và strcmp(exe_command, "."), ta dùng type_flag
    if (type_flag) {
        ESP_LOGI("MENU", "Choice %d selected.", choice_index + 1);
        type_flag = 0; // Reset cờ sau khi xử lý xong
        
        switch (choice_index) {
            case 0:
                esp_mqtt_client_publish(client, MQTT_TOPIC, "Choice 1 selected", 0, 1, 0);
                break;
            case 1:
                esp_mqtt_client_publish(client, MQTT_TOPIC, "Choice 2 selected", 0, 1, 0);
                break;
            case 2:
                esp_mqtt_client_publish(client, MQTT_TOPIC, "Choice 3 selected", 0, 1, 0);
                break;
            case 3:
                esp_mqtt_client_publish(client, MQTT_TOPIC, "Choice 4 selected", 0, 1, 0);
                break;
            case 4:
                esp_mqtt_client_publish(client, MQTT_TOPIC, "Choice 5 selected", 0, 1, 0);
                break;
            default:
                break;
        }
    }
     if (scrollup) {
        choice_index = (choice_index - 1 + CHOICE_MAX_ITEMS) % CHOICE_MAX_ITEMS;
        ESP_LOGI("MENU", "Scrolled up to Choice %d.", choice_index + 1);
        vTaskDelay(pdMS_TO_TICKS(300)); 
    }
    else if (scrolldown) {
        choice_index = (choice_index + 1) % CHOICE_MAX_ITEMS;
        ESP_LOGI("MENU", "Scrolled down to Choice %d.", choice_index + 1);
        vTaskDelay(pdMS_TO_TICKS(300)); 
    }
    
    vTaskDelay(pdMS_TO_TICKS(100));
}
void choose_number_message() {
    if (cached_contact_count <= 0) {
        vTaskDelay(pdMS_TO_TICKS(100));

        return;
    }

    // 2. Chống vượt quá giới hạn nếu danh bạ vừa bị thay đổi/xóa bớt
    if (number_index >= cached_contact_count) {
        number_index = 0;
    }
    if (type_flag){
        current_num=cached_contacts[number_index];
        type_flag=0;
        ESP_LOGI("MENU", "Số đã chọn: %s", current_num.name);


    }
    else if (scrollup) {
        number_index = (number_index - 1 + cached_contact_count) % cached_contact_count;
        //scrollup = 0; // Reset scrollup after processing
        ESP_LOGI("MENU", "Đang cuộn lên thứ %d, số %s", number_index, cached_contacts[number_index].phone);
        vTaskDelay(pdMS_TO_TICKS(500)); 
    }
    else if (scrolldown) {
        number_index = (number_index + 1) % cached_contact_count;
        //scrollup = 0; // Reset scrollup after processing
        ESP_LOGI("MENU", "Đang cuộn xuống thứ %d, số %s", number_index, cached_contacts[number_index].phone);
        vTaskDelay(pdMS_TO_TICKS(500)); 
    }
}

void note(){
    if (xSemaphoreTake(gpio_semaphore, 0) == pdTRUE) {
        key_dir = !key_dir; // Đảo chiều cuộn
        ESP_LOGI("NOTE", "Đã đảo chiều cuộn: %s", key_dir ? "Ngang" : "Dọc");
    }

    else if (type_flag){
        if (key_index<KEYBOARD_MAX_ITEMS){
        ESP_LOGI("NOTE", "Đã gõ phím %s", keyboard_keys[key_index]);}

        switch (key_index){
            case (26):
               int len=strlen(message_buffer);
               if (len>0){
               message_buffer[len-1]='\0';}
               break;
            case (27):
               ESP_LOGI(TAG, "enter");
               break;
            case (28):
               ESP_LOGI(TAG,"SAVED");
               break;
            case (29):
              strcat(message_buffer," ");
              break;
            default:
              if (key_index<KEYBOARD_MAX_ITEMS){
              strcat(message_buffer,keyboard_keys[key_index]);
              printf("message:%s", message_buffer);
              }
              break;
        }
        type_flag=0;
    }

    else if (scrollup) {
        if (key_dir){
        key_index = (key_index - 1 + KEYBOARD_MAX_ITEMS) % KEYBOARD_MAX_ITEMS;
        printf("Cuộn lên KEYBOARD -> Đang trỏ tới phím: [%s] (Index: %d)\n", keyboard_keys[key_index], key_index);
        }
        else if (!key_dir){
            key_index = (key_index - KEYBOARD_COLS + KEYBOARD_MAX_ITEMS) % KEYBOARD_MAX_ITEMS;
            printf("Cuộn dọc LÊN KEYBOARD -> Đang trỏ tới phím: [%s] (Index: %d)\n", keyboard_keys[key_index], key_index);
        }

    }
    else if (scrolldown) {
        if (key_dir){
        key_index = (key_index + 1) % KEYBOARD_MAX_ITEMS;
        printf("Cuộn xuống KEYBOARD -> Đang trỏ tới phím: [%s] (Index: %d)\n", keyboard_keys[key_index], key_index);
    } else if (!key_dir){
        key_index = (key_index + KEYBOARD_COLS) % KEYBOARD_MAX_ITEMS;
        printf("Cuộn dọc XUỐNG KEYBOARD -> Đang trỏ tới phím: [%s] (Index: %d)\n", keyboard_keys[key_index], key_index);
    }
    
}
    vTaskDelay(pdMS_TO_TICKS(300));
  
}

void danh_ba(void){

}

void create_message(void) {
    // 1. Đảo chiều cuộn ngang/dọc khi có tín hiệu chớp mắt 2 lần
    if (xSemaphoreTake(gpio_semaphore, 0) == pdTRUE) {
        key_dir = !key_dir; 
        ESP_LOGI("CREATE_MSG", "Đã đảo chiều cuộn: %s", key_dir ? "Ngang" : "Dọc");
    }

    // 2. Xác nhận chọn phím (Chớp mắt 1 lần)
    else if (type_flag) {
        if (key_index < KEYBOARD_MAX_ITEMS) {
            printf("Đã gõ phím %s\n", keyboard_keys[key_index]);
        }

        switch (key_index) {
            case (26): // Phím CLR (Xóa lùi)
               {
                   int len = strlen(message_buffer);
                   if (len > 0) {
                       message_buffer[len - 1] = '\0';
                   }
               }
               break;

            case (27): // Phím enter (Gửi MQTT)
               ESP_LOGI("CREATE_MSG", "Đang gửi tin nhắn qua MQTT: %s", message_buffer);
               
               // Gửi chuỗi lên topic MQTT
               esp_mqtt_client_publish(client, MQTT_TOPIC, message_buffer, 0, 1, 0);
               
               // Sau khi gửi xong, có thể xóa trắng buffer để soạn tin mới 
               // hoặc tự động thoát về Start Menu tùy ý bạn:
               message_buffer[0] = '\0'; 
               // menu_state = START_MENU; 
               break;

            case (28): // Phím LUU (Trong tin nhắn có thể bỏ qua hoặc báo đang lưu nháp)
               ESP_LOGI("CREATE_MSG", "Lưu nháp tin nhắn...");
               break;

            case (29): // Phím Space (___)
               if (strlen(message_buffer) + 1 < MAX_LEN) {
                   strcat(message_buffer, " ");
               }
               break;

            default: // Các phím chữ cái, số, dấu câu
              if (key_index < KEYBOARD_MAX_ITEMS) {
                  // Chống tràn RAM bằng cách kiểm tra độ dài buffer trước khi ghép
                  if (strlen(message_buffer) + strlen(keyboard_keys[key_index]) < MAX_LEN) {
                      strcat(message_buffer, keyboard_keys[key_index]);
                      printf("message: %s\n", message_buffer);
                  }
              }
              break;
        }
        type_flag = 0; // Reset cờ sau khi xử lý xong
    }

    // 3. Cuộn LÊN
    else if (scrollup) {
        if (key_dir) { // Cuộn ngang
            key_index = (key_index - 1 + KEYBOARD_MAX_ITEMS) % KEYBOARD_MAX_ITEMS;
        } else {       // Cuộn dọc
            key_index = (key_index - KEYBOARD_COLS + KEYBOARD_MAX_ITEMS) % KEYBOARD_MAX_ITEMS;
        }
        printf("Cuộn LÊN -> Trỏ tới phím: [%s]\n", keyboard_keys[key_index]);
    }
    
    // 4. Cuộn XUỐNG
    else if (scrolldown) {
        if (key_dir) { // Cuộn ngang
            key_index = (key_index + 1) % KEYBOARD_MAX_ITEMS;
        } else {       // Cuộn dọc
            key_index = (key_index + KEYBOARD_COLS) % KEYBOARD_MAX_ITEMS;
        }
        printf("Cuộn XUỐNG -> Trỏ tới phím: [%s]\n", keyboard_keys[key_index]);
    }

    vTaskDelay(pdMS_TO_TICKS(300)); // Delay để chống rung/chống dội lệnh cuộn
}

void mailbox_logic(void) {
    if (inbox_count <= 0) {
        // Nếu hộp thư trống, chớp mắt để thoát về Start Menu
        if (type_flag) {
            menu_state = START_MENU;
            type_flag = 0;
            ESP_LOGI("MAILBOX", "Hộp thư trống, quay về Start Menu.");
        }
        return;
    }

    // Đảm bảo index không vượt quá số lượng tin hiện có
    if (mailbox_index >= inbox_count) {
        mailbox_index = inbox_count - 1;
    }

    if (scrollup) {
        mailbox_index = (mailbox_index - 1 + inbox_count) % inbox_count;
        printf("Cuộn LÊN Mailbox -> Index: %d\n", mailbox_index);
        vTaskDelay(pdMS_TO_TICKS(300));
    } 
    else if (scrolldown) {
        mailbox_index = (mailbox_index + 1) % inbox_count;
        printf("Cuộn XUỐNG Mailbox -> Index: %d\n", mailbox_index);
        vTaskDelay(pdMS_TO_TICKS(300));
    }
    else if (type_flag) {
        // CHỚP MẮT CHỌN: Đọc tin nhắn hiện tại và cập nhật trạng thái is_read
        inbox[mailbox_index].is_read = true; 
        
        ESP_LOGI("MAILBOX", "Đã đọc tin [%d]: %s", mailbox_index, inbox[mailbox_index].content);
        
        // (Tùy chọn) Bạn có thể chuyển sang một Sub-state hoặc hiển thị popup nội dung chi tiết.
        // Ở đây hệ thống sẽ tự động cập nhật giao diện LVGL đổi sang màu đã đọc.
        type_flag = 0;
    }
}
void noi_chuyen(void){


}

void vTimeoutCallback(TimerHandle_t xTimer) {
    if (currentCommand[0] != '\0') {
        printf("Hết 3 giây không có tín hiệu! Tự động chốt lệnh.\n");
        
        // Gọi hàm thực thi lệnh
        check_and_execute_command();
        printf("Command executed: %s\n", exe_command);
        // Reset chuỗi sau khi thực thi xong
        currentCommand[0] = '\0';
    }
}

void enable_scr_task(void *pvParameters) {
    while (1) {
        switch (menu_state) {
            case START_MENU:
                enable_scroll = 1;
                break;
            case CHOISE:
                enable_scroll = 1;
                break;
            case CHOOSE_NUMBER_MESSAGE:
                enable_scroll = 1;
                break;
            case NOTE:
                enable_scroll = 1;
                break;
            case COMPUTER:
                enable_scroll = 1;
                break;
            case SOS:
                enable_scroll = 1;
                break;
            case CREATE_MESSAGE:
                enable_scroll = 1;
                break;
            case MAIL_BOX:
                enable_scroll = 1;
                break;
            default:
                enable_scroll = 0;
                break;
        }
     
        vTaskDelay(pdMS_TO_TICKS(100)); // Kiểm tra trạng thái menu mỗi 100ms
    }
} 

void menu_task(void *pvParameters) {
    static enum MENU prev_menu_state = START_MENU;
    while (1) {
        switch (menu_state)
        {
            case START_MENU:
                run_start_menu();
                break;
            case CHOISE:
                run_choice();
                break;
            case CHOOSE_NUMBER_MESSAGE:
                choose_number_message();
                break;
            case CREATE_MESSAGE:
                create_message();
                break;
            case MAIL_BOX:
                mailbox_logic();
                break;
            case NOTE:
                note();
                break;
            case COMPUTER:
                computer();
                break;
            case SOS:
                // Thêm cơ chế cho phép quay về Start Menu nếu muốn (ví dụ dùng type_flag hoặc tín hiệu khác)
                if (type_flag) {
                    menu_state = START_MENU;
                    type_flag = 0;
                    ESP_LOGI("SOS", "Thoát khỏi menu SOS, quay về Start Menu.");
                }
                vTaskDelay(pdMS_TO_TICKS(100)); 
                break;
            default:
                break;
        }
                if (menu_state == SOS && prev_menu_state != SOS) {
            xSemaphoreGive(sos_semaphore);
            ESP_LOGW("SOS", "Đã gửi tín hiệu cầu cứu SOS thành công!");
        }
        if (execute_ready) {
        if ((strcmp(exe_command, "__") == 0)&& menu_state != START_MENU){
            menu_state = START_MENU;
            printf("Returning to START_MENU.\n");
        }
        execute_ready = false;
        exe_command[0] = '\0';
    }
        prev_menu_state = menu_state;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
static void start_ui_init(void)
{
    lv_obj_t *screen = lv_scr_act();
    lv_obj_clean(screen); // Xóa sạch widget cũ khi bắt đầu vào Start Menu

    // Đổi màu nền tối
    lv_obj_set_style_bg_color(screen, lv_color_hex(0x101820), LV_PART_MAIN);

    // Tiêu đề
    lv_obj_t *title = lv_label_create(screen);
    lv_label_set_text(title, "START MENU");
    lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 10);

    // Container chứa các nút dạng lưới (3 hàng x 2 cột)
    lv_obj_t *cont = lv_obj_create(screen);
    lv_obj_set_size(cont, 300, 220);
    lv_obj_align(cont, LV_ALIGN_BOTTOM_MID, 0, -10);
    lv_obj_set_flex_flow(cont, LV_FLEX_FLOW_ROW_WRAP); // Tự động xuống dòng sau mỗi 2 cột
    lv_obj_set_flex_align(cont, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_bg_opa(cont, 0, LV_PART_MAIN); 
    lv_obj_set_style_border_width(cont, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_row(cont, 8, LV_PART_MAIN);       
    lv_obj_set_style_pad_column(cont, 10, LV_PART_MAIN);   

    for (int i = 0; i < START_MENU_MAX_ITEMS; i++) {
        start_menu_btns[i] = lv_btn_create(cont);
        lv_obj_set_size(start_menu_btns[i], 130, 52); 

        lv_obj_t *lbl = lv_label_create(start_menu_btns[i]);
        lv_label_set_text(lbl, start_menu_names[i]);
        lv_obj_set_style_text_font(lbl, &lv_font_montserrat_14, LV_PART_MAIN);
        lv_obj_center(lbl);

        // Highlight nút đang trỏ tới dựa trên start_menu_index
        if (i == start_menu_index) {
            lv_obj_set_style_bg_color(start_menu_btns[i], lv_color_hex(0x61AFEF), LV_PART_MAIN); // Xanh sáng
            lv_obj_set_style_text_color(lbl, lv_color_hex(0x000000), LV_PART_MAIN);
        } else {
            lv_obj_set_style_bg_color(start_menu_btns[i], lv_color_hex(0x3A3F4B), LV_PART_MAIN); // Xám tối
            lv_obj_set_style_text_color(lbl, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
        }
    }
}

static lv_obj_t *mailbox_list = NULL;
static lv_obj_t *mailbox_detail_lbl = NULL; // Khung hiển thị full nội dung ở trên
static lv_obj_t *mailbox_btns_array[MAX_INBOX_MSG];
static void mailbox_ui_init(void) {
    lv_obj_t *screen = lv_scr_act();
    lv_obj_clean(screen);

    // Nền tối
    lv_obj_set_style_bg_color(screen, lv_color_hex(0x101820), LV_PART_MAIN);

    // Tiêu đề
    lv_obj_t *title = lv_label_create(screen);
    lv_label_set_text(title, "INBOX (MAILBOX)");
    lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 5);

    if (inbox_count == 0) {
        lv_obj_t *empty_lbl = lv_label_create(screen);
        lv_label_set_text(empty_lbl, "Hộp thư trống!");
        lv_obj_set_style_text_color(empty_lbl, lv_color_hex(0x888888), LV_PART_MAIN);
        lv_obj_center(empty_lbl);
        return;
    }

    // 1. KHUNG HIỂN THỊ FULL NỘI DUNG Ở PHÍA TRÊN
    mailbox_detail_lbl = lv_label_create(screen);
    lv_obj_set_width(mailbox_detail_lbl, 300);
    lv_label_set_long_mode(mailbox_detail_lbl, LV_LABEL_LONG_WRAP); // Tự động xuống dòng
    lv_obj_set_style_text_font(mailbox_detail_lbl, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_set_style_text_color(mailbox_detail_lbl, lv_color_hex(0x98C379), LV_PART_MAIN); // Màu xanh lá nhạt dễ nhìn
    lv_obj_align(mailbox_detail_lbl, LV_ALIGN_TOP_MID, 0, 25);

    // Hiển thị full nội dung của tin nhắn đang được trỏ tới (mailbox_index)
    if (mailbox_index >= 0 && mailbox_index < inbox_count) {
        lv_label_set_text(mailbox_detail_lbl, inbox[mailbox_index].content);
    } else {
        lv_label_set_text(mailbox_detail_lbl, "...");
    }

    // 2. TẠO DANH SÁCH (LIST) Ở PHÍA DƯỚI
    mailbox_list = lv_list_create(screen);
    lv_obj_set_size(mailbox_list, 320, 140); // Thu nhỏ chiều cao lại để nhường chỗ cho khung trên
    lv_obj_align(mailbox_list, LV_ALIGN_BOTTOM_MID, 0, -5);
    lv_obj_set_style_bg_color(mailbox_list, lv_color_hex(0x21252B), LV_PART_MAIN);

    for (int i = 0; i < inbox_count; i++) {
        char preview[35];
        if (strlen(inbox[i].content) > 22) {
            snprintf(preview, sizeof(preview), "%.22s...", inbox[i].content);
        } else {
            // Đã khắc phục lỗi cảnh báo biên dịch bằng cách giới hạn chuẩn xác độ dài
            snprintf(preview, sizeof(preview), "%.22s", inbox[i].content);
        }

        char display_buf[50];
        if (!inbox[i].is_read) {
            snprintf(display_buf, sizeof(display_buf), "[!] %s", preview);
        } else {
            snprintf(display_buf, sizeof(display_buf), "    %s", preview);
        }

        mailbox_btns_array[i] = lv_list_add_btn(mailbox_list, NULL, display_buf);

        // 2. HIGHLIGHT CŨNG DÙNG BIẾN MẢNG
        if (i == mailbox_index) {
            lv_obj_set_style_bg_color(mailbox_btns_array[i], lv_color_hex(0x61AFEF), LV_PART_MAIN); // Xanh sáng
            lv_obj_set_style_text_color(lv_obj_get_child(mailbox_btns_array[i], 0), lv_color_hex(0x000000), LV_PART_MAIN);
        } else {
            lv_obj_set_style_bg_color(mailbox_btns_array[i], lv_color_hex(0x3A3F4B), LV_PART_MAIN); // Xám
            uint32_t t_color = inbox[i].is_read ? 0x888888 : 0xFFFFFF;
            lv_obj_set_style_text_color(lv_obj_get_child(mailbox_btns_array[i], 0), lv_color_hex(t_color), LV_PART_MAIN);
        }
    }
}

static void mailbox_ui_update(void) {
    if (inbox_count <= 0) return;

    // Cập nhật nội dung full ở khung trên theo vị trí mailbox_index hiện tại
    if (mailbox_detail_lbl != NULL) {
        if (mailbox_index >= 0 && mailbox_index < inbox_count) {
            lv_label_set_text(mailbox_detail_lbl, inbox[mailbox_index].content);
        }
    }

    // Cập nhật lại màu sắc và nhãn của danh sách mà không làm reset vị trí cuộn
    for (int i = 0; i < inbox_count; i++) {
        if (mailbox_btns_array[i] == NULL) continue;

        // Cập nhật lại text nếu trạng thái đọc (is_read) thay đổi
        char preview[35];
        if (strlen(inbox[i].content) > 22) {
            snprintf(preview, sizeof(preview), "%.22s...", inbox[i].content);
        } else {
            snprintf(preview, sizeof(preview), "%.34s", inbox[i].content);
        }

        char display_buf[50];
        if (!inbox[i].is_read) {
            snprintf(display_buf, sizeof(display_buf), "[!] %s", preview);
        } else {
            snprintf(display_buf, sizeof(display_buf), "    %s", preview);
        }
        
        lv_obj_t *lbl = lv_obj_get_child(mailbox_btns_array[i], 0);
        if (lbl != NULL) {
            lv_label_set_text(lbl, display_buf);
        }

        // Highlight dòng đang trỏ tới (mailbox_index)
        if (i == mailbox_index) {
            lv_obj_set_style_bg_color(mailbox_btns_array[i], lv_color_hex(0x61AFEF), LV_PART_MAIN); // Xanh sáng
            if (lbl) lv_obj_set_style_text_color(lbl, lv_color_hex(0x000000), LV_PART_MAIN);
        } else {
            lv_obj_set_style_bg_color(mailbox_btns_array[i], lv_color_hex(0x3A3F4B), LV_PART_MAIN); // Xám
            if (lbl) {
                uint32_t t_color = inbox[i].is_read ? 0x888888 : 0xFFFFFF;
                lv_obj_set_style_text_color(lbl, lv_color_hex(t_color), LV_PART_MAIN);
            }
        }
    }
}
static void start_ui_update(void)
{
    static int prev_index = 0;

    // Đổi màu nút cũ về xám bình thường
    if (prev_index >= 0 && prev_index < START_MENU_MAX_ITEMS) {
        lv_obj_set_style_bg_color(start_menu_btns[prev_index], lv_color_hex(0x3A3F4B), LV_PART_MAIN);
        lv_obj_t *lbl_old = lv_obj_get_child(start_menu_btns[prev_index], 0);
        lv_obj_set_style_text_color(lbl_old, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
    }

    // Đổi màu nút mới sang xanh sáng (Highlight)
    if (start_menu_index >= 0 && start_menu_index < START_MENU_MAX_ITEMS) {
        lv_obj_set_style_bg_color(start_menu_btns[start_menu_index], lv_color_hex(0x61AFEF), LV_PART_MAIN);
        lv_obj_t *lbl_new = lv_obj_get_child(start_menu_btns[start_menu_index], 0);
        lv_obj_set_style_text_color(lbl_new, lv_color_hex(0x000000), LV_PART_MAIN);
    }

    prev_index = start_menu_index;
}
// Hàm 2: CHOICE dạng danh sách (List) có đổi màu mục đang chọn
static void choice_ui(void)
{
    lv_obj_t *screen = lv_scr_act();
    lv_obj_clean(screen); // Xóa sạch widget cũ

    // Đổi màu nền
    lv_obj_set_style_bg_color(screen, lv_color_hex(0x282C34), LV_PART_MAIN);

    // Tiêu đề
    lv_obj_t *title = lv_label_create(screen);
    lv_label_set_text(title, "CHOICE MENU");
    lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 15);

    // Tạo danh sách (List)
    lv_obj_t *list = lv_list_create(screen);
    lv_obj_set_size(list, 320, 220);
    lv_obj_align(list, LV_ALIGN_CENTER, 0, 20);
    lv_obj_set_style_bg_color(list, lv_color_hex(0x21252B), LV_PART_MAIN);

    // Danh sách các lựa chọn
    const char *choices[CHOICE_MAX_ITEMS] = {
        "Choice 1", "Choice 2", "Choice 3", "Choice 4", "Choice 5"
    };

    for (int i = 0; i < CHOICE_MAX_ITEMS; i++) {
        lv_obj_t *btn = lv_list_add_btn(list, NULL, choices[i]);
        
        // Nếu là mục đang được trỏ tới bởi choice_index thì đổi màu nổi bật (Highlight)
        if (i == choice_index) {
            lv_obj_set_style_bg_color(btn, lv_color_hex(0x61AFEF), LV_PART_MAIN); // Xanh dương sáng
            lv_obj_set_style_text_color(lv_obj_get_child(btn, 0), lv_color_hex(0x000000), LV_PART_MAIN);
        } else {
            lv_obj_set_style_bg_color(btn, lv_color_hex(0x3A3F4B), LV_PART_MAIN); // Xám tối
            lv_obj_set_style_text_color(lv_obj_get_child(btn, 0), lv_color_hex(0xFFFFFF), LV_PART_MAIN);
        }
    }
}
static void pc_keyboard_ui_init(void)
{
    lv_obj_t *screen = lv_scr_act();
    lv_obj_clean(screen); 

    lv_obj_set_style_bg_color(screen, lv_color_hex(0x101820), LV_PART_MAIN);

    // Tiêu đề hiển thị chế độ hiện tại
    lv_obj_t *title = lv_label_create(screen);
    lv_label_set_text(title, "COMPUTER - KEYBOARD MODE");
    lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 10);

    // Container chứa các phím bấm dạng lưới
    lv_obj_t *cont = lv_obj_create(screen);
    lv_obj_set_size(cont, 330, 220); 
    lv_obj_align(cont, LV_ALIGN_BOTTOM_MID, 0, -10);
    lv_obj_set_flex_flow(cont, LV_FLEX_FLOW_ROW_WRAP);
    lv_obj_set_flex_align(cont, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_bg_opa(cont, 0, LV_PART_MAIN); 
    lv_obj_set_style_border_width(cont, 0, LV_PART_MAIN);

    for (int i = 0; i < PC_KB_MAX_ITEMS; i++) {
        pc_keyboard_btns[i] = lv_btn_create(cont);
        lv_obj_set_size(pc_keyboard_btns[i], 56, 32);

        lv_obj_t *lbl = lv_label_create(pc_keyboard_btns[i]);
        lv_obj_set_style_text_font(lbl, &lv_font_montserrat_14, LV_PART_MAIN);
        lv_label_set_text(lbl, pc_keyboard_keys[i]);
        lv_obj_center(lbl);

        // Highlight phím đang trỏ tới (pc_key_index)
        if (i == pc_key_index) {
            lv_obj_set_style_bg_color(pc_keyboard_btns[i], lv_color_hex(0x61AFEF), LV_PART_MAIN); // Xanh sáng
            lv_obj_set_style_text_color(lbl, lv_color_hex(0x000000), LV_PART_MAIN);
        } else {
            lv_obj_set_style_bg_color(pc_keyboard_btns[i], lv_color_hex(0x3A3F4B), LV_PART_MAIN); // Xám tối
            lv_obj_set_style_text_color(lbl, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
        }
    }
}

static void pc_keyboard_ui_update(void)
{
    static int prev_pc_key_index = 0;

    // Đổi màu phím cũ về xám
    if (prev_pc_key_index >= 0 && prev_pc_key_index < PC_KB_MAX_ITEMS) {
        lv_obj_set_style_bg_color(pc_keyboard_btns[prev_pc_key_index], lv_color_hex(0x3A3F4B), LV_PART_MAIN);
        lv_obj_t *lbl_old = lv_obj_get_child(pc_keyboard_btns[prev_pc_key_index], 0);
        lv_obj_set_style_text_color(lbl_old, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
    }

    // Đổi màu phím mới sang xanh sáng (Highlight)
    if (pc_key_index >= 0 && pc_key_index < PC_KB_MAX_ITEMS) {
        lv_obj_set_style_bg_color(pc_keyboard_btns[pc_key_index], lv_color_hex(0x61AFEF), LV_PART_MAIN);
        lv_obj_t *lbl_new = lv_obj_get_child(pc_keyboard_btns[pc_key_index], 0);
        lv_obj_set_style_text_color(lbl_new, lv_color_hex(0x000000), LV_PART_MAIN);
    }

    prev_pc_key_index = pc_key_index;
}
static void keyboard_ui_init(void)
{
    lv_obj_t *screen = lv_scr_act();
    lv_obj_clean(screen); // Chỉ xóa 1 lần duy nhất khi vào menu NOTE

    // Đổi màu nền tối
    lv_obj_set_style_bg_color(screen, lv_color_hex(0x101820), LV_PART_MAIN);

    // Tiêu đề nhỏ gọn
    lv_obj_t *title = lv_label_create(screen);
    lv_label_set_text(title, "NOTE KEYBOARD");
    lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 5);

    // Khung hiển thị chuỗi đang gõ (message_buffer)
    txt_box_global = lv_label_create(screen);
    lv_obj_set_width(txt_box_global, 300); // Đặt chiều rộng khung text (tuỳ chỉnh theo màn hình của bạn)
    lv_label_set_long_mode(txt_box_global, LV_LABEL_LONG_WRAP); // Bật tự động xuống dòng
    lv_obj_set_style_text_align(txt_box_global, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN); // Căn lề phải
    lv_label_set_text(txt_box_global, message_buffer[0] != '\0' ? message_buffer : "...");
    lv_obj_set_style_text_color(txt_box_global, lv_color_hex(0x98C379), LV_PART_MAIN); 
    lv_obj_set_style_text_font(txt_box_global, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_align(txt_box_global, LV_ALIGN_TOP_MID, 0, 25);

    // Tạo container chứa bàn phím (6 hàng, 5 cột)
    lv_obj_t *cont = lv_obj_create(screen);
    lv_obj_set_size(cont, 330, 240); 
    lv_obj_align(cont, LV_ALIGN_BOTTOM_MID, 0, -5);
    lv_obj_set_flex_flow(cont, LV_FLEX_FLOW_ROW_WRAP);
    lv_obj_set_flex_align(cont, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_bg_opa(cont, 0, LV_PART_MAIN); 
    lv_obj_set_style_border_width(cont, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(cont, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_row(cont, 6, LV_PART_MAIN);
    lv_obj_set_style_pad_column(cont, 6, LV_PART_MAIN);

    // Khởi tạo 30 phím và lưu vào mảng `keyboard_btns`
    for (int i = 0; i < KEYBOARD_MAX_ITEMS; i++) {
        keyboard_btns[i] = lv_btn_create(cont);
        lv_obj_set_size(keyboard_btns[i], 56, 34);

        lv_obj_t *lbl = lv_label_create(keyboard_btns[i]);
        lv_obj_set_style_text_font(lbl, &lv_font_montserrat_14, LV_PART_MAIN);

        // Gán nhãn ký hiệu
        if (strcmp(keyboard_keys[i], "CLR") == 0) {
            lv_label_set_text(lbl, LV_SYMBOL_BACKSPACE);
        } else if (strcmp(keyboard_keys[i], "enter") == 0) {
            lv_label_set_text(lbl, LV_SYMBOL_OK);
        } else if (strcmp(keyboard_keys[i], "LUU") == 0) {
            lv_label_set_text(lbl, LV_SYMBOL_SAVE);
        } else if (strcmp(keyboard_keys[i], "___") == 0) {
            lv_label_set_text(lbl, "SPC");
        } else {
            lv_label_set_text(lbl, keyboard_keys[i]);
        }

        lv_obj_center(lbl);

        // Đặt trạng thái ban đầu
        if (i == key_index) {
            lv_obj_set_style_bg_color(keyboard_btns[i], lv_color_hex(0x61AFEF), LV_PART_MAIN);
            lv_obj_set_style_text_color(lbl, lv_color_hex(0x000000), LV_PART_MAIN);
        } else {
            lv_obj_set_style_bg_color(keyboard_btns[i], lv_color_hex(0x3A3F4B), LV_PART_MAIN);
            lv_obj_set_style_text_color(lbl, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
        }
    }
}

static void keyboard_ui_update(void)
{
    // Cập nhật chuỗi message đang gõ trên khung text
    if (txt_box_global != NULL) {
        lv_label_set_text(txt_box_global, message_buffer[0] != '\0' ? message_buffer : "...");
    }

    static int prev_key_index = 0;

    // 1. Đổi phím cũ về màu xám bình thường
    if (prev_key_index >= 0 && prev_key_index < KEYBOARD_MAX_ITEMS) {
        lv_obj_set_style_bg_color(keyboard_btns[prev_key_index], lv_color_hex(0x3A3F4B), LV_PART_MAIN);
        lv_obj_t *lbl_old = lv_obj_get_child(keyboard_btns[prev_key_index], 0);
        lv_obj_set_style_text_color(lbl_old, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
    }

    // 2. Đổi phím mới sang màu xanh sáng
    if (key_index >= 0 && key_index < KEYBOARD_MAX_ITEMS) {
        lv_obj_set_style_bg_color(keyboard_btns[key_index], lv_color_hex(0x61AFEF), LV_PART_MAIN);
        lv_obj_t *lbl_new = lv_obj_get_child(keyboard_btns[key_index], 0);
        lv_obj_set_style_text_color(lbl_new, lv_color_hex(0x000000), LV_PART_MAIN);
    }


    prev_key_index = key_index;
}

static void lvgl_task(void *arg)
{
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(1500)); // Delay để đảm bảo các phần cứng khác đã khởi tạo xong
    ESP_LOGI(TAG, "Khoi tao phan cung ILI9488...");
    ili9488_init();

    ESP_LOGI(TAG, "LVGL initialization...");
    lv_init();

    lv_disp_draw_buf_init(&draw_buf, buf1, NULL, LCD_WIDTH * 5);

    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = LCD_WIDTH;
    disp_drv.ver_res = LCD_HEIGHT;
    disp_drv.flush_cb = lvgl_flush_cb;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register(&disp_drv);

    // LVGL tick timer
    const esp_timer_create_args_t timer_args = {
        .callback = &lv_tick_task,
        .name = "lv_tick"
    };
    esp_timer_handle_t lv_tick_timer;
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &lv_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lv_tick_timer, 1000));
    //start_ui(); // Gọi start_ui() để hiển thị START MENU ban đầu
    // Gọi create_ui an toàn bên trong Task này
    keyboard_ui_init();

    ESP_LOGI(TAG, "LVGL ready");
    enum MENU last_menu_state = -1;
    int last_choice_index = -1;
    int last_key_index =-1;
    int last_start_menu_index = -1;
    char last_message_buffer[MAX_LEN]="";
    static int last_pc_key_index = -1;
    int last_mailbox_index = -1;
    static computer_sub_mode_t last_computer_mode = -1;

while (1)
    {
        if (menu_state != last_menu_state || 
            (menu_state == COMPUTER && computer_mode != last_computer_mode) ||
            (menu_state == START_MENU && start_menu_index != last_start_menu_index) ||
            (menu_state == COMPUTER && computer_mode == COMP_MODE_KEYBOARD && pc_key_index != last_pc_key_index) ||
            (menu_state == CHOISE && choice_index != last_choice_index) ||
            (menu_state == MAIL_BOX && mailbox_index != last_mailbox_index) ||
            ((menu_state == NOTE || menu_state == CREATE_MESSAGE) && 
             (key_index != last_key_index || strcmp(message_buffer, last_message_buffer) != 0))
            ) {
            
            if (menu_state == COMPUTER) {
                if (computer_mode == COMP_MODE_KEYBOARD) {
                    if (last_computer_mode != COMP_MODE_KEYBOARD) {
                        pc_keyboard_ui_init(); // Khởi tạo bàn phím lần đầu vào mode
                    } else {
                        pc_keyboard_ui_update(); // Cập nhật highlight phím đang trỏ
                    }
                } else {
                    // Các chế độ cuộn/chuột khác của Computer có thể dùng giao diện chung
                    start_ui_init();
                }
            } else {
                switch (menu_state)
                {
                case START_MENU:
                    if (last_menu_state != START_MENU) {
                        start_ui_init(); // Khởi tạo Start Menu lần đầu vào menu START_MENU
                    } else {
                        start_ui_update(); // Chỉ cập nhật highlight khi cuộn phím bên trong START_MENU
                    }
                    break;
                case CHOISE:
                    choice_ui();
                    break;
                case NOTE:
                    if (last_menu_state != NOTE) {
                    keyboard_ui_init(); // Khởi tạo bàn phím và khung text lần đầu vào menu NOTE
                } else {
                    keyboard_ui_update(); // Chỉ cập nhật highlight khi cuộn phím bên trong NOTE
                }
                break;
                case CREATE_MESSAGE:
                    if (last_menu_state != CREATE_MESSAGE) {
                        keyboard_ui_init(); // Dùng chung giao diện bàn phím của NOTE
                    } else {
                        keyboard_ui_update();
                    }
                    break;
                case MAIL_BOX:
                    if (last_menu_state != MAIL_BOX) {
                        mailbox_ui_init();
                    } else {
                        mailbox_ui_update();
                    }
                    break;
                default:
                    break;
                }
            }
            
            last_menu_state = menu_state;
            last_choice_index = choice_index;
            last_key_index = key_index;
            last_pc_key_index = pc_key_index;
            last_computer_mode = computer_mode;
            last_mailbox_index = mailbox_index;
            strcpy(last_message_buffer, message_buffer);
        }

        lv_timer_handler();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
/*void health_processing_task(void *pvParameters) {
    health_data_packet_t rx_data;

    while (1) {
        if (xQueueReceive(health_queue, &rx_data, portMAX_DELAY) == pdTRUE) {
            printf("Đã lấy từ Queue ra xử lý: %.1f BPM\n", rx_data.bpm);
        }
    }
}*/

void app_main(void) {
eye_sensor_init();
esp_reset_reason_t reason = esp_reset_reason();
ESP_LOGI("RESET", "Nguyen nhân reset chip: %d", reason);
esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "Đang khởi động hệ thống...");

    wifi_init_sta();
    ESP_ERROR_CHECK(esp_now_init());
    //ESP_ERROR_CHECK(esp_now_register_recv_cb(node1_espnow_recv_cb));

    // Khai báo MAC của Node 2 (Cảm biến sức khỏe) để Node 1 có thể gửi lệnh điều khiển
    // Lưu ý: Mảng node2_mac phải được khai báo ở đầu file main.c
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, node2_mac, 6);
    peerInfo.channel = 0;  
    peerInfo.encrypt = false;
    
    // Bỏ qua lỗi nếu peer đã tồn tại (tránh crash khi reset mềm)
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        ESP_LOGW(TAG, "Peer Node 2 đã tồn tại hoặc có lỗi thêm Peer");
    }
    request_health_data(CMD_STOP_STREAM);
    ESP_LOGI(TAG, "Khởi tạo ESP-NOW thành công!");
    // =========================================================================
      ESP_LOGI(TAG, "setting hid gap, mode:%d", HID_DEV_MODE);
    ret = esp_hid_gap_init(HID_DEV_MODE);
    ESP_ERROR_CHECK(ret);

#if CONFIG_BT_BLE_ENABLED || CONFIG_BT_NIMBLE_ENABLED
    ret = esp_hid_ble_gap_adv_init(ESP_HID_APPEARANCE_GENERIC, ble_hid_config.device_name);
    ESP_ERROR_CHECK(ret);

#if CONFIG_BT_BLE_ENABLED
    if ((ret = esp_ble_gatts_register_callback(esp_hidd_gatts_event_handler)) != ESP_OK) {
        ESP_LOGE(TAG, "GATTS register callback failed: %d", ret);
        return;
    }
#endif

    ESP_LOGI(TAG, "setting ble device");
    ESP_ERROR_CHECK(
        esp_hidd_dev_init(&ble_hid_config, ESP_HID_TRANSPORT_BLE, ble_hidd_event_callback, &s_ble_hid_param.hid_dev));
#endif

#if CONFIG_BT_NIMBLE_ENABLED
    ble_store_config_init();
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;
    ret = esp_nimble_enable(ble_hid_device_host_task);
    if (ret) {
        ESP_LOGE(TAG, "esp_nimble_enable failed: %d", ret);
    }
#endif

    ESP_LOGI(TAG, "Hệ thống đã khởi động hoàn tất!");
    timeout_timer = xTimerCreate(
        "CmdTimeoutTimer",
        pdMS_TO_TICKS(3000),
        pdFALSE,                    
        (void *)0,
        vTimeoutCallback           
    );
    //init_a7680c_uart();
    init_contact_cache();
//health_queue = xQueueCreate(5, sizeof(health_data_packet_t));
gpio_semaphore = xSemaphoreCreateBinary();
    if (gpio_semaphore == NULL) {
        printf("Không thể tạo Semaphore!\n");
        return;
    }
sos_semaphore = xSemaphoreCreateBinary();
    if (sos_semaphore == NULL) {
        ESP_LOGE("SOS", "Không thể tạo SOS Semaphore!");
        return;
    }

    // TẠO TASK SOS VỚI ƯU TIÊN CAO (Ví dụ: Priority = 6, trong khi các task khác là 5)
xTaskCreate(sos_emergency_task, "sos_task", 2060, NULL, 6, &sos_task_hdl);
xTaskCreate(signal_decoder_task, "signal_decoder_task", 2048, NULL, 5, NULL);
xTaskCreate(command_task, "command_task", 3072, NULL, 5, NULL);
xTaskCreate(menu_task, "menu_task", 2060, NULL, 5, NULL);
//xTaskCreate(health_processing_task, "health_processing_task", 2048, NULL, 5, NULL);
xTaskCreate(enable_scr_task, "enable_scr_task", 1024, NULL, 5, NULL);
//xTaskCreate(display_task, "display_task", 4096, NULL, 5, NULL);
xTaskCreate(lvgl_task, "lvgl_task", 8192, NULL, 5, NULL);
// 1. Thực hiện thêm liên hệ

// 2. Đọc lại toàn bộ danh bạ để kiểm tra
Contact_t check_list[MAX_CONTACTS];
int total = load_contact_list(check_list, MAX_CONTACTS);

ESP_LOGI("TEST", "--- DANH BẠ HIỆN TẠI TRONG NVS (Tổng: %d) ---", total);
for (int i = 0; i < total; i++) {
    ESP_LOGI("TEST", "[%d] Tên: %s | SĐT: %s", i, check_list[i].name, check_list[i].phone);
}

/*xTaskCreate(a7680c_read_task, "a7680c_read_task", 4096, NULL, 5, NULL);
vTaskDelay(pdMS_TO_TICKS(6000));
    
    send_at_command("AT");           // Kiểm tra kết nối cơ bản
    vTaskDelay(pdMS_TO_TICKS(1000));
    send_at_command("AT+CPIN?");     // Kiểm tra SIM
    vTaskDelay(pdMS_TO_TICKS(1000));
    send_at_command("AT+CSQ");       // Kiểm tra sóng
    send_at_command("AT+CPIN?");     // Phải trả về READY
vTaskDelay(pdMS_TO_TICKS(1000));
send_at_command("AT+CREG?");     // Kiểm tra đăng ký mạng (Mong đợi: +CREG: 0,1 hoặc 0,5)
vTaskDelay(pdMS_TO_TICKS(1000));
//send_at_command("AT+CSQ");       // Kiểm tra sóng (như lúc nãy bạn test đạt 28 là rất khỏe)
    send_sms("+84369979523", "Kiem tra song!");*/
}