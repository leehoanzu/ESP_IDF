/*
 * sender.c — Dual Radar Sender PRODUCTION v4.0
 * Kiến trúc Phase Separation (Cấu hình trước, Đọc luồng sau),
 * Xả cạn (Drain) UART chống Ghost Stream, Anti-Binary Poisoning.
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "driver/uart.h"
#include "esp_timer.h"
#include "esp_log.h"

static const char *TAG = "RADAR_TX_v4";

static uint8_t receiverMAC[6] = {0x1C, 0xDB, 0xD4, 0x76, 0x7C, 0x90};

#define RADAR_HALF_GAP_CM    35
#define RADAR_BUF_SIZE       4096
#define MAX_PAYLOAD          240

// Radar 1 (RIGHT)
#define RADAR1_UART      UART_NUM_2
#define RADAR1_RX_PIN    16
#define RADAR1_TX_PIN    17
#define RADAR1_TIME_MS   101  //131

// Radar 2 (LEFT)
#define RADAR2_UART      UART_NUM_1
#define RADAR2_RX_PIN    41
#define RADAR2_TX_PIN    42
#define RADAR2_TIME_MS   131  //101

#define RADAR_ID_RIGHT   0x01
#define RADAR_ID_LEFT    0x02

// ================= BUFFERS =================
static uint8_t radarBuf1[RADAR_BUF_SIZE]; static int radarIdx1 = 0;
static uint8_t radarBuf2[RADAR_BUF_SIZE]; static int radarIdx2 = 0;

// ================= UART QUEUES =================
static QueueHandle_t uart_queue1 = NULL;
static QueueHandle_t uart_queue2 = NULL;

// ================= THREAD-SAFE ESP-NOW =================
static SemaphoreHandle_t espnow_mutex = NULL;
static SemaphoreHandle_t frame_mutex = NULL;

typedef struct {
    uint8_t  data[250];
    size_t   len;
} TxPacket_t;

static QueueHandle_t tx_queue = NULL;

static void espnow_tx_task(void *pvParameters) {
    TxPacket_t pkt;
    while (1) {
        if (xQueueReceive(tx_queue, &pkt, portMAX_DELAY) == pdTRUE) {
            xSemaphoreTake(espnow_mutex, portMAX_DELAY);
            for (int retry = 0; retry < 3; retry++) {
                if (esp_now_send(receiverMAC, pkt.data, pkt.len) == ESP_OK) break;
                vTaskDelay(pdMS_TO_TICKS(5));
            }
            xSemaphoreGive(espnow_mutex);
        }
    }
}

static void queue_espnow_send(const uint8_t *data, size_t len) {
    TxPacket_t pkt;
    if (len > sizeof(pkt.data)) len = sizeof(pkt.data);
    memcpy(pkt.data, data, len);
    pkt.len = len;
    xQueueSend(tx_queue, &pkt, pdMS_TO_TICKS(100));
}

// ================= CRC16 =================
static uint16_t crc16(const uint8_t *data, size_t len) {
    uint16_t crc = 0;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
            else crc <<= 1;
        }
    }
    return crc;
}

// ================= SEND RADAR FRAME =================
static void sendRadarFrame(uint8_t *data, int len, uint8_t radar_id) {
    uint16_t crc = crc16(data, len);
    uint16_t total_len = len + 2;
    int offset = 0;
    uint8_t seq = 0;
    uint8_t pkt[250];

    xSemaphoreTake(frame_mutex, portMAX_DELAY);
    while (offset < len) {
        int chunk = (len - offset > MAX_PAYLOAD) ? MAX_PAYLOAD : (len - offset);
        bool is_last = (offset + chunk == len);

        pkt[0] = 0x01; pkt[1] = seq++;
        pkt[2] = (total_len >> 8) & 0xFF; pkt[3] = total_len & 0xFF;
        pkt[4] = radar_id;

        memcpy(pkt + 5, data + offset, chunk);
        int send_len = 5 + chunk;
        if (is_last) {
            memcpy(pkt + 5 + chunk, &crc, 2);
            send_len += 2;
        }

        queue_espnow_send(pkt, send_len);
        offset += chunk;
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    xSemaphoreGive(frame_mutex);
}

// ================= ANTI-POISONING AT PARSER =================
// Hàm xả cạn UART FIFO để tránh "Ghost Stream" (có timeout an toàn)
static void drain_uart(uart_port_t port) {
    uint8_t dump[256];
    int len;
    do {
        len = uart_read_bytes(port, dump, sizeof(dump), pdMS_TO_TICKS(100));
    } while (len > 0);
    uart_flush_input(port);
}

// Phân tích phản hồi AT an toàn ngay cả khi lẫn rác nhị phân
static bool wait_at_response_safe(uart_port_t port, const char *expected, int timeout_ms) {
    char buf[512] = {0};
    int idx = 0;
    uint64_t start = esp_timer_get_time() / 1000ULL;

    while ((esp_timer_get_time() / 1000ULL - start) < (uint64_t)timeout_ms) {
        uint8_t b;
        if (uart_read_bytes(port, &b, 1, pdMS_TO_TICKS(10)) > 0) {
            char c = (b == 0 || b > 127) ? '?' : (char)b;
            buf[idx++] = c;
            buf[idx] = '\0';
            
            if (idx >= 500) {
                memmove(buf, buf + 250, 250);
                idx = 250;
                buf[idx] = '\0';
            }

            if (strstr(buf, expected) || strstr(buf, "AT+OK") || strstr(buf, "OK")) {
                ESP_LOGI(TAG, "   ✓ Received: OK");
                return true;
            }
            // Bắt ngay lập tức các mã lỗi của radar để không bị dính Timeout
            if (strstr(buf, "ERROR") || strstr(buf, "AT+ERR") || strstr(buf, "Save Para Fail")) {
                ESP_LOGE(TAG, "   ✗ Radar Rejected! Log: %s", buf);
                return false;
            }
        }
    }
    ESP_LOGW(TAG, "   ✗ TIMEOUT! Log: %s", buf);
    return false;
}

static void send_at_production(uart_port_t port, const char *cmd, const char *wait_keyword, int timeout_ms) {
    // [QUAN TRỌNG] Quét sạch dội âm của lệnh cũ trước khi gửi lệnh mới
    uart_flush_input(port); 
    
    uart_write_bytes(port, cmd, strlen(cmd));
    ESP_LOGI(TAG, "→ %s", cmd);

    if (wait_keyword) {
        wait_at_response_safe(port, wait_keyword, timeout_ms);
    } else {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// ================= PROCESS BYTE =================
static int process_byte(uint8_t byte, uint8_t *buf, int *idx, uart_port_t port, uint8_t radar_id) {
    if (*idx >= RADAR_BUF_SIZE - 1) {
        ESP_LOGE(TAG, "[%s] BUFFER OVERRUN → RESET PARSER", (radar_id == RADAR_ID_RIGHT ? "RIGHT" : "LEFT"));
        *idx = 0;
    }
    buf[(*idx)++] = byte;

    if (*idx >= 8) {
        bool hdr = (buf[0]==1 && buf[1]==2 && buf[2]==3 && buf[3]==4 &&
                    buf[4]==5 && buf[5]==6 && buf[6]==7 && buf[7]==8);
        if (!hdr) {
            memmove(buf, buf + 1, *idx - 1);
            (*idx)--;
            return 0;
        }
    } else return 0;

    if (*idx < 12) return 0;

    uint32_t pktLen = (uint32_t)buf[8] | ((uint32_t)buf[9] << 8) |
                      ((uint32_t)buf[10] << 16) | ((uint32_t)buf[11] << 24);

    if (pktLen > RADAR_BUF_SIZE || pktLen < 12) { *idx = 0; return 0; }

    if (*idx < (int)pktLen) return 0;

    // Bật log để quan sát dễ dàng quá trình gửi frame
    ESP_LOGI(TAG, "[%s FRAME FULL] size=%lu bytes", (radar_id == RADAR_ID_RIGHT ? "RIGHT" : "LEFT"), pktLen);

    sendRadarFrame(buf, (int)pktLen, radar_id);

    memmove(buf, buf + pktLen, *idx - pktLen);
    *idx -= pktLen;
    return 1;
}

// ================= UART EVENT TASKS =================
static void radar1_task(void *pv) {
    uint64_t lastPing = 0;
    while (1) {
        uint64_t now = esp_timer_get_time() / 1000;
        if (now - lastPing > 2000) {
            lastPing = now;
            uint8_t ping[5] = {0x01,0,0,0,RADAR_ID_RIGHT};
            queue_espnow_send(ping, 5);
        }

        uint8_t tmp[256];
        int rd = uart_read_bytes(RADAR1_UART, tmp, sizeof(tmp), pdMS_TO_TICKS(10));
        for (int i = 0; i < rd; i++) {
            process_byte(tmp[i], radarBuf1, &radarIdx1, RADAR1_UART, RADAR_ID_RIGHT);
        }
    }
}

static void radar2_task(void *pv) {
    uint64_t lastPing = 0;
    while (1) {
        uint64_t now = esp_timer_get_time() / 1000;
        if (now - lastPing > 2000) {
            lastPing = now;
            uint8_t ping[5] = {0x01,0,0,0,RADAR_ID_LEFT};
            queue_espnow_send(ping, 5);
        }

        uint8_t tmp[256];
        int rd = uart_read_bytes(RADAR2_UART, tmp, sizeof(tmp), pdMS_TO_TICKS(10));
        for (int i = 0; i < rd; i++) {
            process_byte(tmp[i], radarBuf2, &radarIdx2, RADAR2_UART, RADAR_ID_LEFT);
        }
    }
}

// ================= UART INIT =================
static void uart_init_one(uart_port_t port, int tx, int rx) {
    uart_config_t cfg = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_driver_install(port, 4096, 4096, 0, NULL, 0);
    uart_param_config(port, &cfg);
    uart_set_pin(port, tx, rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

static void uart_init(void) {
    uart_init_one(RADAR1_UART, RADAR1_TX_PIN, RADAR1_RX_PIN);
    uart_init_one(RADAR2_UART, RADAR2_TX_PIN, RADAR2_RX_PIN);
}

// ================= WIFI & ESP-NOW INIT =================
static void wifi_init(void) {
    esp_netif_init();
    esp_event_loop_create_default();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_start();
}

static void espnow_init(void) {
    esp_now_init();
    esp_now_peer_info_t peer = {0};
    memcpy(peer.peer_addr, receiverMAC, 6);
    esp_now_add_peer(&peer);

    espnow_mutex = xSemaphoreCreateMutex();
    frame_mutex = xSemaphoreCreateMutex();
    tx_queue = xQueueCreate(32, sizeof(TxPacket_t));
}

// ================= PRODUCTION CONFIG =================
static void radar_at_config(void) {
    char cmdbuf[64];

    ESP_LOGI(TAG, "=== PHASE 1: STOP & DRAIN ===");
    send_at_production(RADAR1_UART, "AT+STOP\n", NULL, 0);
    vTaskDelay(pdMS_TO_TICKS(500));
    send_at_production(RADAR2_UART, "AT+STOP\n", NULL, 0);
    vTaskDelay(pdMS_TO_TICKS(500));
    drain_uart(RADAR1_UART);
    drain_uart(RADAR2_UART);

    ESP_LOGI(TAG, "=== PHASE 2: CONFIG PARTITION & PARAMS ===");
    send_at_production(RADAR1_UART, "AT+TIME=131\n", "OK", 2000);
    send_at_production(RADAR1_UART, "AT+HEIGHT=100\n", "OK", 2000);
    send_at_production(RADAR1_UART, "AT+RANGE=350\n", "OK", 2000);
    send_at_production(RADAR1_UART, "AT+SENS=8\n", "OK", 2000);
    
    // TRẢ VỀ CẤU HÌNH GỐC ĐỂ RADAR KHÔNG BÁO LỖI VÀ CHẠY ỔN ĐỊNH
    send_at_production(RADAR1_UART, "AT+XNEGAD=0\n", "OK", 2000);
    send_at_production(RADAR1_UART, "AT+XPOSID=350\n", "OK", 2000);

    send_at_production(RADAR2_UART, "AT+TIME=101\n", "OK", 2000);
    send_at_production(RADAR2_UART, "AT+HEIGHT=100\n", "OK", 2000);
    send_at_production(RADAR2_UART, "AT+RANGE=350\n", "OK", 2000);
    send_at_production(RADAR2_UART, "AT+SENS=8\n", "OK", 2000);

    send_at_production(RADAR2_UART, "AT+XNEGAD=-350\n", "OK", 2000);
    send_at_production(RADAR2_UART, "AT+XPOSID=0\n", "OK", 2000);

    ESP_LOGI(TAG, "=== PHASE 3: STUDY (non-blocking) ===");
    send_at_production(RADAR1_UART, "AT+STUDY\n", NULL, 0);  // không chờ
    send_at_production(RADAR2_UART, "AT+STUDY\n", NULL, 0);
    vTaskDelay(pdMS_TO_TICKS(8000));   // chỉ chờ 8s khởi động study

    ESP_LOGI(TAG, "=== PHASE 4: START ===");
    // Tương tự, nếu lệnh START có spam binary ngay lập tức, ta không chờ OK mà cứ gửi rồi xả buffer.
    send_at_production(RADAR1_UART, "AT+START\n", NULL, 0);
    vTaskDelay(pdMS_TO_TICKS(1000)); // Đợi radar khởi động phát binary
    send_at_production(RADAR2_UART, "AT+START\n", NULL, 0);
    vTaskDelay(pdMS_TO_TICKS(1000));

    // drain_uart(RADAR1_UART);
    // drain_uart(RADAR2_UART);
    ESP_LOGI(TAG, "=== CONFIG DONE - Radars now in partition mode ===");
}
// ================= MAIN =================
void app_main(void) {
    nvs_flash_init();
    wifi_init();
    uart_init();
    espnow_init();

    ESP_LOGI(TAG, "=== DUAL-RADAR SENDER v4.1 STARTED ===");

    // 0. Khởi tạo tx task trước để queue hoạt động
    xTaskCreate(espnow_tx_task, "espnow_tx", 4096, NULL, 7, NULL);

    // 1. CẤU HÌNH AT TRƯỚC! (Rất quan trọng)
    // Nếu chạy Parser trước, nó sẽ khóa UART và đọc mất các phản hồi "OK" của Radar
    radar_at_config();

    // 2. Khởi động parser SAU khi config xong, bắt đầu đọc dữ liệu Radar liên tục
    xTaskCreate(radar1_task, "radar1", 4096, NULL, 6, NULL);
    xTaskCreate(radar2_task, "radar2", 4096, NULL, 6, NULL);

    while (1) vTaskDelay(pdMS_TO_TICKS(1000));
}