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
#include "driver/gpio.h"

static const char *TAG = "RADAR_TX_v4.5_FAST_STUDY";

// ================= THÊM CHỨC NĂNG BỎ QUA 10 PHÚT =================
#define ENABLE_FAST_STUDY  1     // ← ĐỔI Ở ĐÂY: 1 = bỏ qua 10 phút (dev/test), 0 = chờ đầy đủ (production)

static uint8_t receiverMAC[6] = {0x1C, 0xDB, 0xD4, 0x76, 0x7C, 0x90};

#define RADAR_HALF_GAP_CM    5
#define RADAR_BUF_SIZE       4096
#define MAX_PAYLOAD          240

// ================= GLOBAL WATCHDOG (giữ nguyên) =================
static uint64_t last_valid_frame_time1 = 0; 
static uint64_t last_valid_frame_time2 = 0;
static bool is_resetting1 = false; 
static bool is_resetting2 = false; 
#define RADAR_TIMEOUT_MS 40000 //25000 

// ================= HARD RESET PINS & UART (giữ nguyên) =================
#define RADAR1_RESET_GPIO    36
#define RADAR2_RESET_GPIO    37
#define RADAR1_UART      UART_NUM_2
#define RADAR1_RX_PIN    16
#define RADAR1_TX_PIN    17
#define RADAR2_UART      UART_NUM_1
#define RADAR2_RX_PIN    41
#define RADAR2_TX_PIN    42
#define RADAR_ID_RIGHT   0x01
#define RADAR_ID_LEFT    0x02

// ================= BUFFERS & QUEUES (giữ nguyên) =================
static uint8_t radarBuf1[RADAR_BUF_SIZE]; static int radarIdx1 = 0;
static uint8_t radarBuf2[RADAR_BUF_SIZE]; static int radarIdx2 = 0;
static QueueHandle_t uart_queue1 = NULL;
static QueueHandle_t uart_queue2 = NULL;
static SemaphoreHandle_t espnow_mutex = NULL;
typedef struct { uint8_t data[250]; size_t len; } TxPacket_t;
static QueueHandle_t tx_queue = NULL;

// ================= MULTI-ZONE (3 CỤC L) =================

#define MY_ZONE_ID          1    // ← ĐỔI Ở ĐÂY cho từng board:
                                  // Cục 1 (đầu phòng) = 1
                                  // Cục 2 (giữa)      = 2  
                                  // Cục 3 (cuối phòng)= 3

#define ZONE3_ROTATED       0     // ← Đổi thành 1 nếu zone 3 bạn lắp radar xoay 90° (bẻ cong)

// ================= REMOTE COMMAND CONFIG =================
typedef struct {
    char cmd[150]; // Chỉ cần lưu nội dung lệnh AT, không cần ID Radar nữa
} RemoteCmd_t;

static QueueHandle_t remote_cmd_queue = NULL;
static bool pause_tdm_for_cmd = false; // Cờ tạm dừng TDM

// ================= ESP-NOW, CRC, SEND FRAME, HARD RESET, DRAIN (giữ nguyên) =================
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

// ================= KHAI BÁO TRƯỚC HÀM (FORWARD DECLARATIONS) =================
static void send_at_production(uart_port_t port, const char *cmd, const char *wait_keyword, int timeout_ms);

// ================= ESP-NOW NHẬN LỆNH TỪ RECEIVER =================
static void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    if (len > 4 && data[0] == 0x02) { 
        char *payload = (char *)(data + 1);
        
        // Tạo chuỗi tiền tố để nhận diện Zone của chính board này (Ví dụ: "Z1:")
        char target_prefix[10];
        snprintf(target_prefix, sizeof(target_prefix), "Z%d:", MY_ZONE_ID);

        RemoteCmd_t rcmd;
        memset(&rcmd, 0, sizeof(rcmd));

        // Nếu lệnh gọi đích danh Zone này HOẶC gọi ALL (Tất cả các Zone)
        if (strncmp(payload, target_prefix, 3) == 0) {
            strncpy(rcmd.cmd, payload + 3, sizeof(rcmd.cmd) - 1);
            xQueueSend(remote_cmd_queue, &rcmd, 0);
        } else if (strncmp(payload, "ALL:", 4) == 0) {
            strncpy(rcmd.cmd, payload + 4, sizeof(rcmd.cmd) - 1);
            xQueueSend(remote_cmd_queue, &rcmd, 0);
        }
        // Nếu gọi Zone khác (VD mình là Z1 mà lệnh là Z2) thì làm ngơ (Bỏ qua)
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

static void sendRadarFrame(uint8_t *data, int len, uint8_t radar_id) {
    uint16_t crc = crc16(data, len);
    uint16_t total_len = len + 2;
    int offset = 0;
    uint8_t seq = 0;
    uint8_t pkt[250];

    while (offset < len) {
        int chunk = (len - offset > MAX_PAYLOAD) ? MAX_PAYLOAD : (len - offset);
        bool is_last = (offset + chunk == len);

        pkt[0] = 0x01; 
        pkt[1] = seq++;
        pkt[2] = (total_len >> 8) & 0xFF; 
        pkt[3] = total_len & 0xFF;
        pkt[4] = MY_ZONE_ID;           // ← THÊM: ZONE_ID
        pkt[5] = radar_id;             // ← radar_id cũ dịch sang vị trí 5

        memcpy(pkt + 6, data + offset, chunk);   // ← dịch +1 byte
        int send_len = 6 + chunk;
        if (is_last) {
            memcpy(pkt + 6 + chunk, &crc, 2);
            send_len += 2;
        }

        queue_espnow_send(pkt, send_len);
        offset += chunk;
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

// ================= HARD RESET FUNCTION =================
static void hard_reset_radar(int gpio_num) {
    gpio_set_level(gpio_num, 0);        // kéo thấp reset
    vTaskDelay(pdMS_TO_TICKS(200));
    gpio_set_level(gpio_num, 1);        // thả cao
    ESP_LOGI(TAG, "   HARD RESET GPIO%d → DONE", gpio_num);
    vTaskDelay(pdMS_TO_TICKS(1200));    // chờ module boot sạch (quan trọng!)
}

// ================= ANTI-POISONING (cải tiến) === ==============
static void drain_uart(uart_port_t port) {
    uint8_t dump[256];
    int len;
    do {
        len = uart_read_bytes(port, dump, sizeof(dump), pdMS_TO_TICKS(50));
    } while (len > 0);
    uart_flush_input(port);
    
}

/// ================= HÀM XẢ RÁC KÉP & ĐƠN =================
static void active_delay_and_drain(int delay_ms) {
    uint64_t start = esp_timer_get_time() / 1000;
    while ((esp_timer_get_time() / 1000) - start < delay_ms) {
        // Vừa chờ thời gian trôi qua, vừa liên tục dọn rác
        drain_uart(RADAR1_UART);
        drain_uart(RADAR2_UART);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

static void drain_single_uart(uart_port_t port, int delay_ms) {
    uint64_t start = esp_timer_get_time() / 1000;
    while ((esp_timer_get_time() / 1000) - start < delay_ms) {
        drain_uart(port);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// ================= HÀM THIẾU – BỔ SUNG Ở ĐÂY (QUAN TRỌNG) =================
static bool wait_at_response_safe(uart_port_t port, const char *expected, int timeout_ms) {
    char buf[512] = {0};
    int idx = 0;
    uint64_t start = esp_timer_get_time() / 1000ULL;

    while ((esp_timer_get_time() / 1000ULL - start) < (uint64_t)timeout_ms) {
        uint8_t b;
        if (uart_read_bytes(port, &b, 1, pdMS_TO_TICKS(10)) > 0) {
            if (idx < 511) buf[idx++] = (b < 32 || b > 126) ? '?' : (char)b;
            buf[idx] = '\0';

            if (strstr(buf, expected) || strstr(buf, "AT+OK") || strstr(buf, "OK") || strstr(buf, "{")) {
                ESP_LOGI(TAG, "   ✓ Received: OK");
                return true;
            }
            if (strstr(buf, "ERROR") || strstr(buf, "AT+ERR")) {
                ESP_LOGE(TAG, "   ✗ ERROR: %s", buf);
                return false;
            }
            if (buf[0]=='?' && buf[1]=='?' && buf[2]=='?' && buf[3]=='?') {
                ESP_LOGI(TAG, "   ✓ Binary mode detected after START");
                return true;
            }
        }
    }
    ESP_LOGW(TAG, "   ✗ TIMEOUT! Last: %s", buf);
    return false;
}

// ================= HÀM WAIT STUDY RESPONSE (giữ nguyên, đã tối ưu) =================
static bool wait_for_study_response(uart_port_t port, int timeout_sec) {
    uint8_t buf[16] = {0};
    int idx = 0;
    uint64_t start = esp_timer_get_time() / 1000ULL;

    while ((esp_timer_get_time() / 1000ULL - start) < (uint64_t)timeout_sec * 1000) {
        uint8_t b;
        if (uart_read_bytes(port, &b, 1, pdMS_TO_TICKS(10)) > 0) {
            if (idx < 15) buf[idx++] = b;
            if (idx >= 6 && buf[0]==0x55 && buf[1]==0xAA && buf[2]==0x06 && buf[3]==0x00) {
                uint8_t result = buf[4];
                if ((result == 0xB1 && buf[5] == 0xB7) || (result == 0xA1 && buf[5] == 0xA7)) {
                    ESP_LOGI(TAG, "STUDY RESPONSE: %s", (result == 0xB1) ? "B1 B7 → CẦN HỌC MỚI" : "A1 A7 → ĐÃ HỌC SẴN");
                    return (result == 0xB1);
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    return false;
}

// ================= HELPER FUNCTION =================
static inline const char* radar_id_label(uint8_t id) {
    return (id == RADAR_ID_RIGHT) ? "RIGHT" : "LEFT";
}

// ================= PROCESS BYTE V3 =================
static int process_byte(uint8_t byte, uint8_t *buf, int *idx, uart_port_t port, uint8_t radar_id) {
    if (*idx >= RADAR_BUF_SIZE) {
        ESP_LOGE(TAG, "[%s] BUFFER FULL → HARD RESET PARSER", radar_id_label(radar_id));
        *idx = 0;
        // Không return 0 ở đây để vẫn nạp byte hiện tại vào buffer mới
    }

    buf[(*idx)++] = byte;

    if (*idx < 8) return 0; // Chưa đủ chiều dài tối thiểu của Header

    // === 1. Tìm Header (Chỉ đọc - Cực nhanh, O(N)) ===
    int header_idx = -1;
    // Tối ưu: Nếu 8 byte đầu đã là header, vòng lặp này break ngay lập tức ở i=0
    for (int i = 0; i <= *idx - 8; i++) {
        if (buf[i]==1 && buf[i+1]==2 && buf[i+2]==3 && buf[i+3]==4 &&
            buf[i+4]==5 && buf[i+5]==6 && buf[i+6]==7 && buf[i+7]==8) {
            header_idx = i;
            break;
        }
    }

    // === 2. Xử lý Rác (Chỉ dùng 1 lệnh memmove duy nhất) ===
    if (header_idx > 0) {
        // Có rác phía trước header
        ESP_LOGW(TAG, "[%s] Dropping %d garbage bytes", radar_id_label(radar_id), header_idx);
        memmove(buf, buf + header_idx, *idx - header_idx);
        *idx -= header_idx;
    } else if (header_idx == -1) {
        // Toàn bộ buffer là rác, không có header -> Xóa hết, chỉ giữ lại 7 byte cuối
        memmove(buf, buf + (*idx - 7), 7);
        *idx = 7;
        return 0;
    }

    // Tại thời điểm này, chắc chắn buf[0] đến buf[7] là Header chuẩn.
    
    // === 3. Đọc Length & Xác thực ===
    if (*idx < 12) return 0; // Chờ thêm dữ liệu để đọc Length

    uint32_t pktLen = (uint32_t)buf[8] | ((uint32_t)buf[9]<<8) |
                      ((uint32_t)buf[10]<<16) | ((uint32_t)buf[11]<<24);

    // Xác thực độ dài: 32 bytes (0 người) đến 512 bytes (khoảng 14 người)
    if (pktLen > 512 || pktLen < 32) {
        ESP_LOGE(TAG, "[%s] INVALID LEN %lu → RESET", radar_id_label(radar_id), pktLen);
        *idx = 0;
        return 0;
    }

    // if (*idx < (int)pktLen) return 0; // Chưa nhận đủ toàn bộ frame

    // === 4. Xử lý frame hợp lệ ===
    // (Dùng ESP_LOGD hoặc ESP_LOGI tùy cấu hình log level của bạn)
    // ESP_LOGI(TAG, "[%s FRAME] %lu bytes, %lu persons", 
    //          radar_id_label(radar_id), pktLen, (pktLen-32)/32);

    // sendRadarFrame(buf, (int)pktLen, radar_id);

    // Tìm đoạn cuối của hàm process_byte và thêm:
    if (*idx < (int)pktLen) return 0; 

    // --- CẬP NHẬT WATCHDOG TẠI ĐÂY ---
    uint64_t now = esp_timer_get_time() / 1000;
    if (radar_id == RADAR_ID_RIGHT) last_valid_frame_time1 = now;
    else last_valid_frame_time2 = now;

    ESP_LOGI(TAG, "[%s FRAME] %lu bytes", radar_id_label(radar_id), pktLen);
    sendRadarFrame(buf, (int)pktLen, radar_id);

    // Dọn buffer cho frame tiếp theo
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
            // uint8_t ping[5] = {0x01,0,0,0,RADAR_ID_RIGHT};
            uint8_t ping[6] = {0x01, 0, 0, 0, MY_ZONE_ID, RADAR_ID_RIGHT};
            queue_espnow_send(ping, 6);
        }

        uart_event_t event;
        // Kiểm tra queue nhanh hơn
        if (xQueueReceive(uart_queue1, &event, pdMS_TO_TICKS(10))) {
            if (event.type == UART_DATA) {
                // TĂNG KÍCH THƯỚC BUFFER ĐỌC LÊN 1024
                uint8_t tmp[1024]; 
                size_t sz = event.size > sizeof(tmp) ? sizeof(tmp) : event.size;
                int rd = uart_read_bytes(RADAR1_UART, tmp, sz, 0);
                for (int i = 0; i < rd; i++) {
                    process_byte(tmp[i], radarBuf1, &radarIdx1, RADAR1_UART, RADAR_ID_RIGHT);
                }
            } else if (event.type == UART_FIFO_OVF || event.type == UART_BUFFER_FULL) {
                ESP_LOGE(TAG, "[RIGHT RADAR] UART OVERFLOW OR BUFFER FULL!");
                uart_flush_input(RADAR1_UART);
            }
        }
    }
}

static void radar2_task(void *pv) {
    uint64_t lastPing = 0;
    while (1) {
        uint64_t now = esp_timer_get_time() / 1000;
        if (now - lastPing > 2000) {
            lastPing = now;
            // uint8_t ping[5] = {0x01,0,0,0,RADAR_ID_LEFT};
            uint8_t ping[6] = {0x01, 0, 0, 0, MY_ZONE_ID, RADAR_ID_LEFT};
            queue_espnow_send(ping, 6);
        }

        uart_event_t event;
        if (xQueueReceive(uart_queue2, &event, pdMS_TO_TICKS(10))) {
            if (event.type == UART_DATA) {
                uint8_t tmp[1024];
                size_t sz = event.size > sizeof(tmp) ? sizeof(tmp) : event.size;  // ← SỬA LỖI CŨ
                int rd = uart_read_bytes(RADAR2_UART, tmp, sz, 0);
                for (int i = 0; i < rd; i++) {
                    process_byte(tmp[i], radarBuf2, &radarIdx2, RADAR2_UART, RADAR_ID_LEFT);
                }
            } else if (event.type == UART_FIFO_OVF || event.type == UART_BUFFER_FULL) {
                uart_flush_input(RADAR2_UART);
            }
        }
    }
}

// ================= TASK THỰC THI LỆNH TỪ XA =================
// ================= OTA COMMAND TASK (HỌC LẠI TỪ ĐẦU) =================
static void remote_cmd_task(void *pv) {
    RemoteCmd_t rcmd;
    while(1) {
        if (xQueueReceive(remote_cmd_queue, &rcmd, portMAX_DELAY)) {
            ESP_LOGW(TAG, "⚡ NHẬN LỆNH OTA CHO ZONE %d: %s", MY_ZONE_ID, rcmd.cmd);
            
            pause_tdm_for_cmd = true;       // Khóa TDM không cho nhả xung
            vTaskDelay(pdMS_TO_TICKS(600)); // Đợi TDM kết thúc chu kỳ hiện tại

            // -------------------------------------------------------------
            // BƯỚC 1: ÉP DỪNG RADAR (Để tránh rác UART)
            // -------------------------------------------------------------
            ESP_LOGW(TAG, "--- BƯỚC 1: DỪNG RADAR ---");
            send_at_production(RADAR1_UART, "AT+STOP\n", "OK", 800); 
            send_at_production(RADAR2_UART, "AT+STOP\n", "OK", 800);
            drain_uart(RADAR1_UART); 
            drain_uart(RADAR2_UART);

            // -------------------------------------------------------------
            // BƯỚC 2: NẠP LỆNH MỚI VÀ LƯU VÀO FLASH CỦA RADAR
            // -------------------------------------------------------------
            ESP_LOGW(TAG, "--- BƯỚC 2: NẠP THÔNG SỐ %s", rcmd.cmd);
            send_at_production(RADAR1_UART, rcmd.cmd, "OK", 1500);
            send_at_production(RADAR2_UART, rcmd.cmd, "OK", 1500);
            
            // Bắt buộc phải RESTORE để radar ghi nhớ thông số này vào Flash
            send_at_production(RADAR1_UART, "AT+RESTORE\n", "OK", 1500);
            send_at_production(RADAR2_UART, "AT+RESTORE\n", "OK", 1500);

            // -------------------------------------------------------------
            // BƯỚC 3: REBOOT ĐỂ RADAR ÁP DỤNG THÔNG SỐ VỪA LƯU
            // -------------------------------------------------------------
            ESP_LOGW(TAG, "--- BƯỚC 3: REBOOT ĐỂ ÁP DỤNG ---");
            send_at_production(RADAR1_UART, "AT+RESET\n", "OK", 800);
            send_at_production(RADAR2_UART, "AT+RESET\n", "OK", 800);
            vTaskDelay(pdMS_TO_TICKS(2500)); // Chờ module radar boot xong hệ điều hành
            drain_uart(RADAR1_UART); 
            drain_uart(RADAR2_UART);

            // -------------------------------------------------------------
            // BƯỚC 4: HỌC LẠI MÔI TRƯỜNG (STUDY) VỚI THÔNG SỐ MỚI
            // -------------------------------------------------------------
            ESP_LOGW(TAG, "--- BƯỚC 4: HỌC MÔI TRƯỜNG (STUDY) ---");
            
            uart_write_bytes(RADAR1_UART, "AT+STUDY\n", 9);
            bool need_wait1 = wait_for_study_response(RADAR1_UART, 5);
            
            uart_write_bytes(RADAR2_UART, "AT+STUDY\n", 9);
            bool need_wait2 = wait_for_study_response(RADAR2_UART, 5);

            // Nếu radar yêu cầu học (B1 B7)
            if (need_wait1 || need_wait2) {
                ESP_LOGI(TAG, "→ Đang tiến hành học %s...", ENABLE_FAST_STUDY ? "nhanh (10s)" : "chuẩn (10 phút)");
                active_delay_and_drain(ENABLE_FAST_STUDY ? 10000 : 600000);
                
                // Lưu lại kết quả học môi trường vào Flash
                send_at_production(RADAR1_UART, "AT+RESTORE\n", "OK", 2000);
                send_at_production(RADAR2_UART, "AT+RESTORE\n", "OK", 2000);
            } else {
                ESP_LOGI(TAG, "→ Radar báo đã học sẵn.");
            }

            // -------------------------------------------------------------
            // BƯỚC 5: HOÀN TẤT, ĐƯA VỀ TRẠNG THÁI STOP ĐỂ TDM TIẾP QUẢN
            // -------------------------------------------------------------
            ESP_LOGW(TAG, "--- BƯỚC 5: HOÀN TẤT OTA ---");
            send_at_production(RADAR1_UART, "AT+STOP\n", "OK", 800);
            send_at_production(RADAR2_UART, "AT+STOP\n", "OK", 800);

            // Reset Watchdog Timer để tránh module bị restart oan uổng
            last_valid_frame_time1 = esp_timer_get_time() / 1000;
            last_valid_frame_time2 = esp_timer_get_time() / 1000;

            pause_tdm_for_cmd = false; // Mở khóa cho TDM Task chạy
            ESP_LOGI(TAG, "⚡ ZONE %d TIẾP TỤC TDM VỚI THÔNG SỐ MỚI!", MY_ZONE_ID);

            // =============================================================
            // BƯỚC 6: GỬI PHẢN HỒI (ACK) VỀ CHO RECEIVER
            // =============================================================
            char ack_msg[100];
            snprintf(ack_msg, sizeof(ack_msg), "Da nap xong lenh: %s", rcmd.cmd);
            
            uint8_t ack_pkt[120];
            ack_pkt[0] = 0x03;               // Header 0x03 quy ước là gói tin ACK
            ack_pkt[1] = MY_ZONE_ID;         // Gửi kèm ID của Zone để Receiver biết ai đang báo cáo
            int msg_len = strlen(ack_msg);
            memcpy(ack_pkt + 2, ack_msg, msg_len);
            
            // Đẩy vào hàng đợi để gửi qua ESP-NOW về Receiver
            queue_espnow_send(ack_pkt, msg_len + 2);
        }
    }
}

// ================= UART INIT =================
static void uart_init_one(uart_port_t port, int tx, int rx, QueueHandle_t *queue) {
    uart_config_t cfg = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    // uart_driver_install(port, 4096, 4096, 20, queue, 0);
    // NÊN SỬA THÀNH:
    uart_driver_install(port, 8192, 4096, 50, queue, 0);

    uart_param_config(port, &cfg);
    uart_set_pin(port, tx, rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

static void uart_init(void) {
    uart_init_one(RADAR1_UART, RADAR1_TX_PIN, RADAR1_RX_PIN, &uart_queue1);
    uart_init_one(RADAR2_UART, RADAR2_TX_PIN, RADAR2_RX_PIN, &uart_queue2);
}

static void send_at_production(uart_port_t port, const char *cmd, const char *wait_keyword, int timeout_ms) {
    for (int retry = 0; retry < 3; retry++) {
        uart_flush_input(port);
        uart_write_bytes(port, cmd, strlen(cmd));
        ESP_LOGI(TAG, "→ %s (retry %d)", cmd, retry);

        if (wait_keyword) {
            if (wait_at_response_safe(port, wait_keyword, timeout_ms)) {
                return;  // Thành công
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(100));
            return;
        }

        // Nếu nhận "Save Para Fail" → retry ngay
        // (hàm wait_at_response_safe đã log lỗi, ta chỉ retry)
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    ESP_LOGE(TAG, "✗ FAILED after 3 retries: %s", cmd);
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
    tx_queue = xQueueCreate(32, sizeof(TxPacket_t));
}

// ================= APPLY COMMON PARAMS (giữ nguyên) =================
// ================= SET SCENE PARAMS SAU STUDY (FULL RANGE) =================
// ================= SET SCENE PARAMS TỐI ƯU CHO PHÒNG CHỮ L + OVERLAP 50CM =================
static void set_scene_params(uart_port_t port) {
    // 1. Thông số chung (trần 2.7m, độ nhạy vừa phải)
    const char* COMMON_PARAMS[] = {
        "AT+DEBUG=3\n",
        "AT+HEIGHT=270\n",      // Trần thực tế 2.7m
        "AT+HEIGHTD=270\n",
        "AT+SENS=3\n"
    };
    for (int i = 0; i < sizeof(COMMON_PARAMS)/sizeof(COMMON_PARAMS[0]); i++) {
        send_at_production(port, COMMON_PARAMS[i], "OK", 2000);
    }

    // 2. Boundary riêng theo Zone (đã cộng overlap 50cm để handoff mượt)
#if MY_ZONE_ID == 1
    // Zone 1 (3m9 x 4m8) + chồng lấn 50cm sang Zone 2
    const char* ZONE_PARAMS[] = {
        "AT+RANGE=200\n",
        "AT+TIME=107\n", //101 127
        "AT+XNegaD=-255\n", "AT+XPosiD=255\n",   // 3.9m + 25cm mỗi bên "AT+XNegaD=-215\n", "AT+XPosiD=215\n",
        "AT+YNegaD=-250\n", "AT+YPosiD=250\n"    // 4.8m
    };
#elif MY_ZONE_ID == 2
    // Zone 2 (3m55 x 4m8) + chồng lấn 50cm với cả Zone 1 và Zone 3
    const char* ZONE_PARAMS[] = {
        "AT+RANGE=200\n",
        "AT+TIME=101\n", //101
        "AT+XNegaD=-200\n", "AT+XPosiD=200\n",   // 3.55m + overlap "AT+XNegaD=-200\n", "AT+XPosiD=200\n", 
        "AT+YNegaD=-250\n", "AT+YPosiD=250\n"
    };
#elif MY_ZONE_ID == 3
    // Zone 3 (Đuôi 3m3 x 2m8) + chồng lấn 50cm theo trục Y
    const char* ZONE_PARAMS[] = {
        "AT+RANGE=200\n",
        "AT+TIME=101\n", //101
        "AT+XNegaD=-180\n", "AT+XPosiD=180\n",   // 3.3m ngang
        "AT+YNegaD=-165\n", "AT+YPosiD=165\n"    // 2.8m dọc + overlap 50cm
    };
#else
    const char* ZONE_PARAMS[] = {
        "AT+XNegaD=-200\n", "AT+XPosiD=200\n",
        "AT+YNegaD=-200\n", "AT+YPosiD=200\n"
    };
#endif

    for (int i = 0; i < sizeof(ZONE_PARAMS)/sizeof(ZONE_PARAMS[0]); i++) {
        send_at_production(port, ZONE_PARAMS[i], "OK", 2000);
    }
}

// ================= RADAR_AT_CONFIG – ĐÃ CÓ CHỨC NĂNG BỎ QUA =================
static void radar_at_config(void) {
    ESP_LOGI(TAG, "=== PHASE 0: HARD RESET BOTH RADARS ===");
    hard_reset_radar(RADAR1_RESET_GPIO);
    hard_reset_radar(RADAR2_RESET_GPIO);
    vTaskDelay(pdMS_TO_TICKS(1500));

    // ================= STUDY RIGHT (hoàn thành learning mode trước) =================
    ESP_LOGI(TAG, "=== PHASE 1: STUDY RIGHT ===");
    uart_write_bytes(RADAR1_UART, "AT+STUDY\n", 9);
    bool need_wait = wait_for_study_response(RADAR1_UART, 15);
    if (need_wait) {
        if (ENABLE_FAST_STUDY) {
            ESP_LOGW(TAG, "FAST MODE → chỉ chờ 10s");
            active_delay_and_drain(10000);
        } else {
            ESP_LOGI(TAG, "→ Học môi trường RIGHT 10 phút...");
            active_delay_and_drain(600000);
        }
        send_at_production(RADAR1_UART, "AT+RESTORE\n", "OK", 2000);  // lưu learning
    } else {
        ESP_LOGI(TAG, "→ RIGHT đã học sẵn");
    }

    // ================= STUDY LEFT =================
    ESP_LOGI(TAG, "=== PHASE 2: STUDY LEFT ===");
    uart_write_bytes(RADAR2_UART, "AT+STUDY\n", 9);
    need_wait = wait_for_study_response(RADAR2_UART, 15);
    if (need_wait) {
        if (ENABLE_FAST_STUDY) {
            active_delay_and_drain(10000);
        } else {
            ESP_LOGI(TAG, "→ Học môi trường LEFT 10 phút...");
            active_delay_and_drain(600000);
        }
        send_at_production(RADAR2_UART, "AT+RESTORE\n", "OK", 2000);
    } else {
        ESP_LOGI(TAG, "→ LEFT đã học sẵn");
    }

    // ================= SAU KHI STUDY XONG MỚI SET RANGE (theo scene) =================
    ESP_LOGI(TAG, "=== PHASE 3: SET SCENE PARAMS (FULL RANGE) ===");
    set_scene_params(RADAR1_UART);
    set_scene_params(RADAR2_UART);

    //STOP SAU KHI SET RANGE
    send_at_production(RADAR1_UART, "AT+STOP\n", "OK", 200);
    send_at_production(RADAR2_UART, "AT+STOP\n", "OK", 200);
    

    ESP_LOGI(TAG, "=== CONFIG DONE - STUDY HOÀN TẤT → FULL RANGE + TDM 10s ===");
}

// ================= RADAR RECOVERY CŨNG CÓ CHỨC NĂNG BỎ QUA =================
// ================= RECOVERY (cũng STUDY trước) =================
// static void radar_recovery_config(uart_port_t port, uint8_t radar_id) {
//     ESP_LOGW(TAG, "=== RECOVERY: %s ===", radar_id_label(radar_id));
//     int gpio = (radar_id == RADAR_ID_RIGHT) ? RADAR1_RESET_GPIO : RADAR2_RESET_GPIO;
//     hard_reset_radar(gpio);

//     uart_write_bytes(port, "AT+STUDY\n", 9);
//     bool need_wait = wait_for_study_response(port, 15);
//     if (need_wait) {
//         if (ENABLE_FAST_STUDY) active_delay_and_drain(10000);
//         else active_delay_and_drain(600000);
//         send_at_production(port, "AT+RESTORE\n", "OK", 2000);
//     }
//     set_scene_params(port);   // set range sau study
//     uart_write_bytes(port, "AT+RESET\n", 9);
//     vTaskDelay(pdMS_TO_TICKS(300));
//     ESP_LOGI(TAG, "=== %s RECOVERY COMPLETE (FULL RANGE) ===", radar_id_label(radar_id));
// }

// ================= RECOVERY – THEO ĐÚNG FLOW BẠN GỢI Ý =================
static void radar_recovery_config(uart_port_t port, uint8_t radar_id) {
    ESP_LOGW(TAG, "=== RECOVERY %s START ===", radar_id_label(radar_id));

    // 1. STOP radar trước
    uart_write_bytes(port, "AT+STOP\n", 8);
    drain_uart(port);
    vTaskDelay(pdMS_TO_TICKS(200));

    // 2. Hard reset GPIO
    int gpio = (radar_id == RADAR_ID_RIGHT) ? RADAR1_RESET_GPIO : RADAR2_RESET_GPIO;
    hard_reset_radar(gpio);

    // 3. Wait 2s (boot sạch)
    vTaskDelay(pdMS_TO_TICKS(2000));

    // 4. AT+RESET
    uart_write_bytes(port, "AT+RESET\n", 9);
    vTaskDelay(pdMS_TO_TICKS(300));

    // 5. AT+STUDY
    uart_write_bytes(port, "AT+STUDY\n", 9);
    bool need_wait = wait_for_study_response(port, 15);
    if (need_wait) {
        if (ENABLE_FAST_STUDY) {
            active_delay_and_drain(10000);
        } else {
            active_delay_and_drain(600000);
            send_at_production(port, "AT+RESTORE\n", "OK", 2000);
        }
    }

    // 6. Set params (FULL RANGE)
    set_scene_params(port);

    ESP_LOGI(TAG, "=== RECOVERY %s HOÀN TẤT - READY ===", radar_id_label(radar_id));
}

// ================= TDM, WATCHDOG, PARSER, app_main (GIỮ NGUYÊN HOÀN TOÀN) =================
// radar1_task, radar2_task, radar_tdm_task, radar_watchdog_task, process_byte, 
// uart_init, wifi_init, espnow_init, app_main... giữ nguyên như bản trước.
// ================= WATCHDOG TASK =================
static void radar_watchdog_task(void *pv) {
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(2000)); 
        // Đã xóa dòng if (is_system_resetting) ở đây

        uint64_t now = esp_timer_get_time() / 1000;

        // Xử lý Right Radar
        if (!is_resetting1 && (now - last_valid_frame_time1 > RADAR_TIMEOUT_MS)) {
            ESP_LOGE(TAG, "!!! RIGHT RADAR HUNG - RECOVERING...");
            is_resetting1 = true;
            radar_recovery_config(RADAR1_UART, RADAR_ID_RIGHT);
            last_valid_frame_time1 = esp_timer_get_time() / 1000;
            is_resetting1 = false;
        }

        // Xử lý Left Radar
        if (!is_resetting2 && (now - last_valid_frame_time2 > RADAR_TIMEOUT_MS)) {
            ESP_LOGE(TAG, "!!! LEFT RADAR HUNG - RECOVERING...");
            is_resetting2 = true;
            radar_recovery_config(RADAR2_UART, RADAR_ID_LEFT);
            last_valid_frame_time2 = esp_timer_get_time() / 1000;
            is_resetting2 = false;
        }
    }
}

// ================= TDM SWITCH TASK =================
static void radar_tdm_task(void *pv) {
    ESP_LOGI(TAG, "=== TDM 10s FULL RANGE MODE (CROSS-CHECK + BACKUP) ===");

    while (1) {
        if (!is_resetting1) {
            uart_write_bytes(RADAR1_UART, "AT+START\n", 9);
            ESP_LOGI(TAG, "→ START RIGHT 10s (full range)");
            vTaskDelay(pdMS_TO_TICKS(18000)); //10000

            // uart_write_bytes(RADAR1_UART, "AT+STOP\n", 8);
            send_at_production(RADAR1_UART, "AT+RESET\n", "OK", 200);

            send_at_production(RADAR1_UART, "AT+STOP\n", "OK", 200);

            ESP_LOGW(TAG, "WAIT radar stop...");
            drain_uart(RADAR1_UART);

            vTaskDelay(pdMS_TO_TICKS(200));

            
        } else vTaskDelay(pdMS_TO_TICKS(500));

        if (!is_resetting2) {
            uart_write_bytes(RADAR2_UART, "AT+START\n", 9);
            ESP_LOGI(TAG, "→ START LEFT 10s (full range)");
            vTaskDelay(pdMS_TO_TICKS(18000)); //10000

            send_at_production(RADAR2_UART, "AT+RESET\n", "OK", 200);

            // uart_write_bytes(RADAR2_UART, "AT+STOP\n", 8);
            send_at_production(RADAR2_UART, "AT+STOP\n", "OK", 200);

            ESP_LOGW(TAG, "WAIT radar stop...");
            drain_uart(RADAR2_UART); //phải drain nếu không bị nhiễu.
            
            // send_at_production(RADAR2_UART, "AT+RESET\n", "OK", 300);



            vTaskDelay(pdMS_TO_TICKS(200));
        } else vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// ================= app_main (chỉ update TAG) =================
void app_main(void) {
    nvs_flash_init();
    wifi_init();
    uart_init();
    espnow_init();

    gpio_config_t io_conf = { .mode = GPIO_MODE_OUTPUT, .pin_bit_mask = (1ULL<<RADAR1_RESET_GPIO) | (1ULL<<RADAR2_RESET_GPIO) };
    gpio_config(&io_conf);
    gpio_set_level(RADAR1_RESET_GPIO, 1);
    gpio_set_level(RADAR2_RESET_GPIO, 1);

    ESP_LOGI(TAG, "=== DUAL-RADAR v4.7 - STUDY TRƯỚC → FULL RANGE + TDM 10s ===");

    radar_at_config();

    xTaskCreate(radar1_task, "radar1", 4096, NULL, 6, NULL);
    xTaskCreate(radar2_task, "radar2", 4096, NULL, 6, NULL);
    xTaskCreate(espnow_tx_task, "espnow_tx", 4096, NULL, 7, NULL);
    xTaskCreate(radar_tdm_task, "tdm_switch", 4096, NULL, 5, NULL);
    xTaskCreate(radar_watchdog_task, "watchdog", 3072, NULL, 4, NULL);

    esp_now_register_recv_cb(OnDataRecv);
    remote_cmd_queue = xQueueCreate(10, sizeof(RemoteCmd_t));
    xTaskCreate(remote_cmd_task, "remote_cmd", 4096, NULL, 8, NULL);

    uint64_t startup_now = esp_timer_get_time() / 1000;
    last_valid_frame_time1 = startup_now;
    last_valid_frame_time2 = startup_now;

    while (1) vTaskDelay(pdMS_TO_TICKS(1000));
}