#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "mqtt_client.h"

#define MQTT_BROKER "mqtt://broker.hivemq.com"
#define MQTT_TOPIC  "message"
esp_mqtt_client_handle_t client;
// --- CẤU HÌNH TÊN WIFI VÀ MẬT KHẨU CỦA BẠN ---
#define EXAMPLE_ESP_WIFI_SSID      "Viet Tho"
#define EXAMPLE_ESP_WIFI_PASS      "987654321"
#define EXAMPLE_ESP_MAXIMUM_RETRY  5

static const char *TAG = "WIFI_STATION";

/* FreeRTOS event group để báo hiệu khi kết nối thành công */
static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
extern void save_incoming_message(const char* data, int data_len);
static int s_retry_num = 0;
static void mqtt_event_handler(void *handler_args,
                               esp_event_base_t base,
                               int32_t event_id,
                               void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;

    // Khi kết nối thành công tới Broker
    if (event->event_id == MQTT_EVENT_CONNECTED)
    {
        ESP_LOGI(TAG, "MQTT Connected");
        esp_mqtt_client_subscribe(client, MQTT_TOPIC, 1);
    }
    // Khi nhận được tin nhắn từ MQTTX gửi lên Topic
else if (event->event_id == MQTT_EVENT_DATA)
    {
        // Kiểm tra đúng Topic trước
        if (strncmp(event->topic, "message", event->topic_len) == 0) {
            
            // Tạo mảng đệm an toàn (ví dụ 100 byte giống bên main.c)
            char safe_msg[100];
            
            // Tính toán kích thước copy an toàn, không bao giờ vượt quá kích thước mảng
            int copy_len = event->data_len;
            if (copy_len > sizeof(safe_msg) - 1) {
                copy_len = sizeof(safe_msg) - 1;
            }
            
            // Copy đúng phần cho phép
            memcpy(safe_msg, event->data, copy_len);
            safe_msg[copy_len] = '\0'; // Bắt buộc chốt ký tự kết thúc chuỗi
            
            // Truyền chuỗi đã được bảo vệ vào hàm lưu
            save_incoming_message(safe_msg, strlen(safe_msg));
        }
    }
}
void start_mqtt(void)
{
    esp_mqtt_client_config_t cfg = {
        .broker.address.uri = MQTT_BROKER,
    };

    client = esp_mqtt_client_init(&cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
}

// Hàm bắt sự kiện Wi-Fi và IP
static void event_handler(void* arg, esp_event_base_t event_base,
                          int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "Đang kết nối lại với Wi-Fi...");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG, "Kết nối Wi-Fi thất bại");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {

        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Đã nhận được IP:" IPSTR, IP2STR(&event->ip_info.ip));
        start_mqtt(); // Bắt đầu kết nối MQTT khi nhận được IP
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

// Hàm khởi tạo Wi-Fi chế độ Station
void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    // 1. Khởi tạo TCP/IP stack và event loop
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    // 2. Cấu hình thông tin Wi-Fi
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    
    // 3. Khởi động Wi-Fi
    ESP_ERROR_CHECK(esp_wifi_start());
    esp_wifi_set_channel(6, WIFI_SECOND_CHAN_NONE);

    ESP_LOGI(TAG, "wifi_init_sta hoàn tất.");

    // Chờ kết nối thành công hoặc thất bại
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Kết nối thành công đến SSID:%s", EXAMPLE_ESP_WIFI_SSID);
        wifi_second_chan_t second;
        uint8_t primary_chan;
        esp_wifi_get_channel(&primary_chan, &second);
ESP_LOGI(TAG, "Kênh Wi-Fi hiện tại của Node 1 là: %d", primary_chan);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Thất bại khi kết nối đến SSID:%s", EXAMPLE_ESP_WIFI_SSID);
    } else {
        ESP_LOGE(TAG, "Sự kiện không xác định");
    }
}