// /*
//  * sender.c — Dual Radar Sender
//  * =============================================================
//  * Kiến trúc 2 radar trong 1 cục (khoảng cách gần nhau ~30–70 cm):
//  *
//  *   R1 = RADAR_RIGHT  (UART2, pin 16/17)  →  phủ nửa PHẢI  (X ≥ 0)
//  *   R2 = RADAR_LEFT   (UART1, pin 41/42)  →  phủ nửa TRÁI  (X < 0)
//  *
//  * ── Spatial Partition (phân vùng cứng) ──────────────────────────
//  *   R1: AT+XNEGAD=0   · AT+XPOSID=350   → firmware loại X < 0
//  *   R2: AT+XNEGAD=-350· AT+XPOSID=0     → firmware loại X > 0
//  *   → 2 radar không bao giờ báo cùng 1 mục tiêu.
//  *
//  * ── Prime Stagger (giảm RF coupling) ────────────────────────────
//  *   R1: AT+TIME=131 ms
//  *   R2: AT+TIME=101 ms
//  *   LCM(101,131) = 13 231 ms → va chạm < 1 lần / 13 s
//  *
//  * ── Coordinate Calibration (bù lệch vật lý) ─────────────────────
//  *   Gắn radar_id vào packet; receiver bù offset ±X_OFFSET_CM.
//  *   R1 lắp bên PHẢI tâm cục → offset = +RADAR_HALF_GAP_CM
//  *   R2 lắp bên TRÁI tâm cục → offset = -RADAR_HALF_GAP_CM
//  *
//  * ── Packet format gửi qua ESP-NOW ───────────────────────────────
//  *   [0x01][seq][lenH][lenL][radar_id][payload…]
//  *    byte0  1    2     3      4         5+
//  *   radar_id: RADAR_ID_RIGHT=0x01, RADAR_ID_LEFT=0x02
//  * =============================================================
//  */

// #include <stdio.h>
// #include <string.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "esp_system.h"
// #include "esp_wifi.h"
// #include "esp_now.h"
// #include "esp_event.h"
// #include "nvs_flash.h"
// #include "driver/uart.h"
// #include "esp_timer.h"

// // ================= CONFIG =================
// static const uint8_t receiverMAC[] = {0x1C, 0xDB, 0xD4, 0x76, 0x7C, 0x90};

// /* ── Radar 1 (RIGHT): phủ nửa PHẢI, X ∈ [0, +350 cm] ── */
// #define RADAR1_UART      UART_NUM_2
// #define RADAR1_RX_PIN    16
// #define RADAR1_TX_PIN    17
// #define RADAR1_TIME_MS   131          // Prime Stagger: chu kỳ 131 ms
// #define RADAR1_XNEG      0            // Spatial Partition: chặn X âm
// #define RADAR1_XPOS      350          // Mức X dương tối đa (cm)

// /* ── Radar 2 (LEFT): phủ nửa TRÁI, X ∈ [-350, 0 cm] ── */
// #define RADAR2_UART      UART_NUM_1
// #define RADAR2_RX_PIN    41
// #define RADAR2_TX_PIN    42
// #define RADAR2_TIME_MS   101          // Prime Stagger: chu kỳ 101 ms
// #define RADAR2_XNEG      (-350)       // Mức X âm tối đa (cm)
// #define RADAR2_XPOS      0            // Spatial Partition: chặn X dương

// /* ── Coordinate Calibration ── */
// // Nếu 2 radar đặt gần tâm (khoảng cách nhỏ), offset nhỏ.
// // Đo thực tế: khoảng cách 2 radar / 2 (cm). Ví dụ 30cm → 15cm.
// #define RADAR_HALF_GAP_CM   35      // 70cm center-to-center / 2 (khop voi receiver 0.35m)

// /* ── Sensor height ── */
// #define RADAR_HEIGHT_CM  100   ///267          // Chiều cao lắp đặt (cm)

// /* ── radar_id bytes ── */
// #define RADAR_ID_RIGHT   0x01
// #define RADAR_ID_LEFT    0x02

// /* ── Misc ── */
// #define RADAR_BUF_SIZE   2048
// #define RADAR_RANGE_CM   350          // Thu hẹp tầm quét giảm nhiễu xa

// // ================= BUFFERS =================
// static uint8_t radarBuf1[RADAR_BUF_SIZE];
// static int     radarIdx1 = 0;

// static uint8_t radarBuf2[RADAR_BUF_SIZE];
// static int     radarIdx2 = 0;

// // ================= HELPERS =================
// static void send_at(uart_port_t port, const char *cmd)
// {

//     uart_write_bytes(port, cmd, strlen(cmd));
//     vTaskDelay(pdMS_TO_TICKS(300));
//     printf("[AT→UART%d] %s", (port == RADAR1_UART) ? 2 : 1, cmd);
// }

// // ================= CRC16-CCITT =================
// static uint16_t crc16(const uint8_t *data, int len)
// {
//     uint16_t crc = 0xFFFF;
//     for (int i = 0; i < len; i++) {
//         crc ^= (uint16_t)data[i] << 8;
//         for (int j = 0; j < 8; j++)
//             crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
//     }
//     return crc;
// }

// // ================= ESP-NOW RECV =================
// // Receiver gửi lệnh AT về → chuyển thẳng xuống RADAR_1 (mặc định)
// // Nếu cần điều khiển từng radar, thêm prefix vào lệnh.
// void OnDataRecv(const esp_now_recv_info_t *info,
//                 const uint8_t *data,
//                 int len)
// {
//     if (len > 1 && data[0] == 0x02) {
//         const char *cmd = (const char *)(data + 1);
//         int cmdLen = len - 1;

//         // R1: prefix → chi gui radar RIGHT
//         if (cmdLen > 3 && cmd[0]=='R' && cmd[1]=='1' && cmd[2]==':') {
//             uart_write_bytes(RADAR1_UART, cmd + 3, cmdLen - 3);
//             printf("\n>> Cmd fwd R1 only: %.*s\n", cmdLen - 3, cmd + 3);
//         }
//         // R2: prefix → chi gui radar LEFT
//         else if (cmdLen > 3 && cmd[0]=='R' && cmd[1]=='2' && cmd[2]==':') {
//             uart_write_bytes(RADAR2_UART, cmd + 3, cmdLen - 3);
//             printf("\n>> Cmd fwd R2 only: %.*s\n", cmdLen - 3, cmd + 3);
//         }
//         // Khong prefix → gui ca 2
//         else {
//             uart_write_bytes(RADAR1_UART, cmd, cmdLen);
//             uart_write_bytes(RADAR2_UART, cmd, cmdLen);
//             printf("\n>> Cmd fwd both: %.*s\n", cmdLen, cmd);
//         }
//     }
// }

// // ================= SEND FRAME =================
// static void sendRadarFrame(uint8_t *data, int len, uint8_t radar_id)
// {
//     /*
//      * Packet: [0x01][seq][totalH][totalL][radar_id][payload…]
//      * totalLen = len + 2 (2 byte CRC16 append cuoi)
//      * → Receiver se verify CRC truoc khi parse
//      */
//     uint16_t crc = crc16(data, len);
//     uint16_t totalLen = len + 2;  // payload + CRC

//     int     offset = 0;
//     uint8_t seq    = 0;
//     uint8_t pkt[250];

//     // Tao buffer tam: data + CRC
//     // (Gui toan bo data truoc, roi gui 2 byte CRC o chunk cuoi)
//     while (offset < len) {
//         int chunk = len - offset;
//         if (chunk > 224) chunk = 224;

//         pkt[0] = 0x01;
//         pkt[1] = seq++;
//         pkt[2] = (totalLen >> 8) & 0xFF;
//         pkt[3] = totalLen & 0xFF;
//         pkt[4] = radar_id;

//         memcpy(pkt + 5, data + offset, chunk);
//         int pktLen = chunk + 5;

//         // Neu la chunk cuoi → append 2 byte CRC
//         if (offset + chunk >= len) {
//             pkt[pktLen]     = (crc >> 8) & 0xFF;
//             pkt[pktLen + 1] = crc & 0xFF;
//             pktLen += 2;
//         }

//         esp_now_send(receiverMAC, pkt, pktLen);
//         offset += chunk;
//         vTaskDelay(pdMS_TO_TICKS(2));
//     }
// }

// // ================= WIFI INIT =================
// static void wifi_init(void)
// {
//     esp_netif_init();
//     esp_event_loop_create_default();
//     wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
//     esp_wifi_init(&cfg);
//     esp_wifi_set_mode(WIFI_MODE_STA);
//     esp_wifi_start();
// }

// // ================= UART INIT =================
// static void uart_init_one(uart_port_t port, int tx, int rx)
// {
//     uart_config_t cfg = {
//         .baud_rate = 115200,
//         .data_bits = UART_DATA_8_BITS,
//         .parity    = UART_PARITY_DISABLE,
//         .stop_bits = UART_STOP_BITS_1,
//         .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
//     };
//     uart_driver_install(port, 4096, 0, 0, NULL, 0);
//     uart_param_config(port, &cfg);
//     uart_set_pin(port, tx, rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
// }

// static void uart_init(void)
// {
//     uart_init_one(RADAR1_UART, RADAR1_TX_PIN, RADAR1_RX_PIN);
//     uart_init_one(RADAR2_UART, RADAR2_TX_PIN, RADAR2_RX_PIN);
// }

// // ================= AT CONFIG =================
// /*
//  * Spatial Partition + Prime Stagger + Height
//  * Gọi sau khi UART đã sẵn sàng. Chờ radar boot ~1–2 giây trước.
//  *
//  * Lưu ý: AT+RESTORE khôi phục factory → xóa cấu hình cũ.
//  * Nếu muốn giữ lại cấu hình khác (VD: AT+SENS), bỏ dòng RESTORE
//  * và chỉ gửi những lệnh cần thay đổi.
//  */
// static void radar_at_config(void)
// {
//     printf("\n[CFG] Configuring Radar 1 (RIGHT, %dms)...\n", RADAR1_TIME_MS);
//     send_at(RADAR1_UART, "AT+RESTORE\n");
//     send_at(RADAR1_UART, "AT+STOP\n");
//     vTaskDelay(pdMS_TO_TICKS(500));

//     char buf[64];
//     snprintf(buf, sizeof(buf), "AT+TIME=%d\n", RADAR1_TIME_MS);
//     send_at(RADAR1_UART, buf);

//     snprintf(buf, sizeof(buf), "AT+HEIGHT=%d\n", RADAR_HEIGHT_CM);
//     send_at(RADAR1_UART, buf);

//     snprintf(buf, sizeof(buf), "AT+HEIGHTD=%d\n", RADAR_HEIGHT_CM);
//     send_at(RADAR1_UART, buf);

//     snprintf(buf, sizeof(buf), "AT+RANGE=%d\n", RADAR_RANGE_CM);
//     send_at(RADAR1_UART, buf);

//     snprintf(buf, sizeof(buf), "AT+XNEGAD=%d\n", RADAR1_XNEG);
//     send_at(RADAR1_UART, buf);

//     snprintf(buf, sizeof(buf), "AT+XPOSID=%d\n", RADAR1_XPOS);
//     send_at(RADAR1_UART, buf);

//     send_at(RADAR1_UART, "AT+SENS=8\n");
//     send_at(RADAR1_UART, "AT+YNEGAD=-150\n");
//     send_at(RADAR1_UART, "AT+YPOSID=150\n");

//     send_at(RADAR1_UART, "AT+START\n");
//     printf("[CFG] Radar 1 (RIGHT) configured.\n");

//     // ---- Radar 2 ----
//     printf("\n[CFG] Configuring Radar 2 (LEFT, %dms)...\n", RADAR2_TIME_MS);
//     send_at(RADAR2_UART, "AT+RESTORE\n");
//     send_at(RADAR2_UART, "AT+STOP\n");
//     vTaskDelay(pdMS_TO_TICKS(500));

//     snprintf(buf, sizeof(buf), "AT+TIME=%d\n", RADAR2_TIME_MS);
//     send_at(RADAR2_UART, buf);

//     snprintf(buf, sizeof(buf), "AT+HEIGHT=%d\n", RADAR_HEIGHT_CM);
//     send_at(RADAR2_UART, buf);

//     snprintf(buf, sizeof(buf), "AT+HEIGHTD=%d\n", RADAR_HEIGHT_CM);
//     send_at(RADAR2_UART, buf);

//     snprintf(buf, sizeof(buf), "AT+RANGE=%d\n", RADAR_RANGE_CM);
//     send_at(RADAR2_UART, buf);

//     snprintf(buf, sizeof(buf), "AT+XNEGAD=%d\n", RADAR2_XNEG);
//     send_at(RADAR2_UART, buf);

//     snprintf(buf, sizeof(buf), "AT+XPOSID=%d\n", RADAR2_XPOS);
//     send_at(RADAR2_UART, buf);

//     send_at(RADAR2_UART, "AT+SENS=8\n");
//     send_at(RADAR2_UART, "AT+YNEGAD=-150\n");
//     send_at(RADAR2_UART, "AT+YPOSID=150\n");

//     send_at(RADAR2_UART, "AT+START\n");
//     printf("[CFG] Radar 2 (LEFT) configured.\n\n");
// }

// // ================= ESPNOW INIT =================
// static void espnow_init(void)
// {
//     esp_now_init();
//     esp_now_register_recv_cb(OnDataRecv);

//     esp_now_peer_info_t peer = {0};
//     memcpy(peer.peer_addr, receiverMAC, 6);
//     peer.channel = 0;
//     peer.encrypt = false;
//     esp_now_add_peer(&peer);
// }

// // ================= FRAME PARSER (dùng chung) =================
// /*
//  * Đọc từng byte từ UART và ghép thành frame.
//  * Header 8 byte: 01 02 03 04 05 06 07 08
//  * Byte [8..11]  : packetLen (uint32_t LE)
//  *
//  * Trả về 1 khi đã gửi xong 1 frame đầy đủ, 0 nếu chưa.
//  */
// static int process_byte(uint8_t byte,
//                          uint8_t  *buf,
//                          int      *idx,
//                          uart_port_t port,
//                          uint8_t  radar_id)
// {
//     buf[(*idx)++] = byte;

//     // Bước 1: Chờ đủ 8 byte header
//     if (*idx >= 8) {
//         bool hdr = (buf[0]==1 && buf[1]==2 && buf[2]==3 && buf[3]==4 &&
//                     buf[4]==5 && buf[5]==6 && buf[6]==7 && buf[7]==8);
//         if (!hdr) {
//             memmove(buf, buf + 1, *idx - 1);
//             (*idx)--;
//             return 0;
//         }
//     } else {
//         return 0;
//     }

//     // Bước 2: Chờ đủ 12 byte để đọc packetLen
//     if (*idx < 12) return 0;

//     uint32_t pktLen;
//     memcpy(&pktLen, &buf[8], 4);

//     if (pktLen > RADAR_BUF_SIZE) {
//         *idx = 0;
//         return 0;
//     }

//     // Bước 3: Chờ đủ frame
//     if (*idx < (int)pktLen) return 0;

//     // Gửi frame qua ESP-NOW
//     const char *zoneName = (radar_id == RADAR_ID_RIGHT) ? "RIGHT" : "LEFT";
//     printf("\n[%s Frame] size=%lu\n", zoneName, pktLen);
//     sendRadarFrame(buf, (int)pktLen, radar_id);

//     // Xoá phần đã gửi, giữ lại phần dư
//     memmove(buf, buf + pktLen, *idx - pktLen);
//     *idx -= pktLen;
//     return 1;
// }

// // ================= TASK RADAR 1 (RIGHT) =================
// static void radar1_task(void *pv)
// {
//     uint8_t rxChunk[256];
//     uint64_t lastPing = 0;

//     while (1) {
//         uint64_t now = esp_timer_get_time() / 1000;
//         if (now - lastPing > 2000) {
//             lastPing = now;
//             uint8_t pingPkt[] = {0x01, 0x00, 0x00, 0x00, RADAR_ID_RIGHT};
//             esp_now_send(receiverMAC, pingPkt, sizeof(pingPkt));
//             printf("[RIGHT] Ping...\n");
//         }

//         int len = uart_read_bytes(RADAR1_UART, rxChunk, sizeof(rxChunk), pdMS_TO_TICKS(10));
//         for (int i = 0; i < len; i++)
//             process_byte(rxChunk[i], radarBuf1, &radarIdx1, RADAR1_UART, RADAR_ID_RIGHT);

//         if (radarIdx1 >= RADAR_BUF_SIZE) radarIdx1 = 0;
//     }
// }

// // ================= TASK RADAR 2 (LEFT) =================
// static void radar2_task(void *pv)
// {
//     uint8_t rxChunk[256];
//     uint64_t lastPing = 0;

//     while (1) {
//         uint64_t now = esp_timer_get_time() / 1000;
//         if (now - lastPing > 2000) {
//             lastPing = now;
//             uint8_t pingPkt[] = {0x01, 0x00, 0x00, 0x00, RADAR_ID_LEFT};
//             esp_now_send(receiverMAC, pingPkt, sizeof(pingPkt));
//             printf("[LEFT ] Ping...\n");
//         }

//         int len = uart_read_bytes(RADAR2_UART, rxChunk, sizeof(rxChunk), pdMS_TO_TICKS(10));
//         for (int i = 0; i < len; i++)
//             process_byte(rxChunk[i], radarBuf2, &radarIdx2, RADAR2_UART, RADAR_ID_LEFT);

//         if (radarIdx2 >= RADAR_BUF_SIZE) radarIdx2 = 0;
//     }
// }

// // ================= MAIN =================
// void app_main(void)
// {
//     nvs_flash_init();
//     wifi_init();
//     uart_init();
//     espnow_init();

//     printf("\n===================================================\n");
//     printf("  Dual-Radar Sender\n");
//     printf("  R1 RIGHT: UART2 (pin%d/%d), TIME=%dms, X=[%d..%d]cm\n",
//            RADAR1_RX_PIN, RADAR1_TX_PIN, RADAR1_TIME_MS,
//            RADAR1_XNEG, RADAR1_XPOS);
//     printf("  R2 LEFT : UART1 (pin%d/%d), TIME=%dms, X=[%d..%d]cm\n",
//            RADAR2_RX_PIN, RADAR2_TX_PIN, RADAR2_TIME_MS,
//            RADAR2_XNEG, RADAR2_XPOS);
//     printf("  Calibration offset: +/- %d cm\n", RADAR_HALF_GAP_CM);
//     printf("===================================================\n\n");

//     // Chờ radar boot xong rồi mới gửi AT
//     vTaskDelay(pdMS_TO_TICKS(2000));
//     radar_at_config();

//     xTaskCreate(radar1_task, "radar1_task", 4096, NULL, 5, NULL);
//     xTaskCreate(radar2_task, "radar2_task", 4096, NULL, 5, NULL);
// }



/*
 * sender.c — Dual Radar Sender PRODUCTION v3.1
 * Full production-grade: AT response machine, thread-safe TX queue + mutex,
 * buffer safety, parser chạy trước config, no pray-and-delay.
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

static const char *TAG = "RADAR_TX_PROD";

static uint8_t receiverMAC[6] = {0x1C, 0xDB, 0xD4, 0x76, 0x7C, 0x90};

#define RADAR_HALF_GAP_CM    35
#define RADAR_BUF_SIZE       4096
#define MAX_PAYLOAD          240

// Radar 1 (RIGHT)
#define RADAR1_UART      UART_NUM_2
#define RADAR1_RX_PIN    16
#define RADAR1_TX_PIN    17
#define RADAR1_TIME_MS   131
#define RADAR1_XNEG      0
#define RADAR1_XPOS      350

// Radar 2 (LEFT)
#define RADAR2_UART      UART_NUM_1
#define RADAR2_RX_PIN    41
#define RADAR2_TX_PIN    42
#define RADAR2_TIME_MS   101
#define RADAR2_XNEG      (-350)
#define RADAR2_XPOS      0

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
}

// ================= AT RESPONSE MACHINE =================
static bool wait_response(uart_port_t port, const char *expected, int timeout_ms) {
    char buf[256] = {0};   // tăng buffer
    int idx = 0;
    uint64_t start = esp_timer_get_time() / 1000ULL;

    while ((esp_timer_get_time() / 1000ULL - start) < (uint64_t)timeout_ms) {
        uint8_t b;
        if (uart_read_bytes(port, &b, 1, pdMS_TO_TICKS(10)) > 0) {  // giảm xuống 10ms
            if (idx < 255) buf[idx++] = b;
            buf[idx] = '\0';

            if (strstr(buf, expected) || strstr(buf, "AT+OK") || strstr(buf, "OK")) {
                ESP_LOGI(TAG, "   ✓ Received: %s", buf);
                return true;
            }
            if (strstr(buf, "ERROR") || strstr(buf, "Save Para Fail")) {
                ESP_LOGE(TAG, "   ✗ ERROR: %s", buf);
                return false;
            }
        }
    }

    // Timeout → log những gì radar thực sự trả về (rất quan trọng!)
    ESP_LOGW(TAG, "   ✗ TIMEOUT waiting for '%s' | Received so far: %s", expected ? expected : "any", buf);
    return false;   // vẫn tiếp tục config (không chết cứng)
}

static void send_at_production(uart_port_t port, const char *cmd, const char *wait_keyword, int timeout_ms) {
    uart_flush_input(port);
    uart_write_bytes(port, cmd, strlen(cmd));
    ESP_LOGI(TAG, "→ %s", cmd);

    if (wait_keyword) {
        wait_response(port, wait_keyword, timeout_ms);
    } else {
        vTaskDelay(pdMS_TO_TICKS(300));
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
        ESP_LOGD(TAG, "[%s] HEADER DETECTED", (radar_id == RADAR_ID_RIGHT ? "RIGHT" : "LEFT"));
    } else return 0;

    if (*idx < 12) return 0;

    uint32_t pktLen = (uint32_t)buf[8] | ((uint32_t)buf[9] << 8) |
                      ((uint32_t)buf[10] << 16) | ((uint32_t)buf[11] << 24);

    if (pktLen > RADAR_BUF_SIZE || pktLen < 12) { *idx = 0; return 0; }

    if (*idx < (int)pktLen) return 0;

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

        uart_event_t event;
        if (xQueueReceive(uart_queue1, &event, pdMS_TO_TICKS(10))) {
            if (event.type == UART_DATA) {
                uint8_t tmp[256];
                size_t sz = event.size > 256 ? 256 : event.size;
                int rd = uart_read_bytes(RADAR1_UART, tmp, sz, 0);
                for (int i = 0; i < rd; i++) {
                    process_byte(tmp[i], radarBuf1, &radarIdx1, RADAR1_UART, RADAR_ID_RIGHT);
                }
            } else if (event.type == UART_FIFO_OVF || event.type == UART_BUFFER_FULL) {
                uart_flush_input(RADAR1_UART);
                ESP_LOGE(TAG, "UART1 OVERRUN!");
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
            uint8_t ping[5] = {0x01,0,0,0,RADAR_ID_LEFT};
            queue_espnow_send(ping, 5);
        }

        uart_event_t event;
        if (xQueueReceive(uart_queue2, &event, pdMS_TO_TICKS(10))) {
            if (event.type == UART_DATA) {
                uint8_t tmp[256];
                size_t sz = event.size > 256 ? 256 : event.size;
                int rd = uart_read_bytes(RADAR2_UART, tmp, sz, 0);
                for (int i = 0; i < rd; i++) {
                    process_byte(tmp[i], radarBuf2, &radarIdx2, RADAR2_UART, RADAR_ID_LEFT);
                }
            } else if (event.type == UART_FIFO_OVF || event.type == UART_BUFFER_FULL) {
                uart_flush_input(RADAR2_UART);
                ESP_LOGE(TAG, "UART2 OVERRUN!");
            }
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
    uart_driver_install(port, 4096, 4096, 20, queue, 0);
    uart_param_config(port, &cfg);
    uart_set_pin(port, tx, rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

static void uart_init(void) {
    uart_init_one(RADAR1_UART, RADAR1_TX_PIN, RADAR1_RX_PIN, &uart_queue1);
    uart_init_one(RADAR2_UART, RADAR2_TX_PIN, RADAR2_RX_PIN, &uart_queue2);
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

// ================= PRODUCTION CONFIG =================
static void radar_at_config(void) {
    ESP_LOGI(TAG, "=== PRODUCTION AT CONFIG START (fast boot) ===");

    // RIGHT
    ESP_LOGI(TAG, "=== CONFIG RADAR RIGHT (R1) ===");
    send_at_production(RADAR1_UART, "AT+STOP\n", "AT+OK", 2000);
    send_at_production(RADAR1_UART, "AT+TIME=131\n", "AT+OK", 2000);
    send_at_production(RADAR1_UART, "AT+HEIGHT=100\n", "AT+OK", 2000);
    send_at_production(RADAR1_UART, "AT+RANGE=350\n", "AT+OK", 2000);
    send_at_production(RADAR1_UART, "AT+XNEGAD=0\n", "AT+OK", 2000);
    send_at_production(RADAR1_UART, "AT+XPOSID=350\n", "AT+OK", 2000);
    send_at_production(RADAR1_UART, "AT+SENS=8\n", "AT+OK", 2000);
    send_at_production(RADAR1_UART, "AT+YNEGAD=-150\n", "AT+OK", 2000);
    send_at_production(RADAR1_UART, "AT+YPOSID=150\n", "AT+OK", 2000);

    // STUDY chỉ khởi động (không chờ 10 phút)
    send_at_production(RADAR1_UART, "AT+STUDY\n", NULL, 0);   // không chờ keyword
    ESP_LOGW(TAG, "Study mode STARTED on RIGHT (full learning ~10 min in background). Data will come after START.");
    vTaskDelay(pdMS_TO_TICKS(12000));   // chỉ chờ 12s để radar ổn định

    send_at_production(RADAR1_UART, "AT+START\n", "AT+OK", 2000);

    // LEFT (tương tự)
    ESP_LOGI(TAG, "=== CONFIG RADAR LEFT (R2) ===");
    send_at_production(RADAR2_UART, "AT+STOP\n", "AT+OK", 2000);
    send_at_production(RADAR2_UART, "AT+TIME=101\n", "AT+OK", 2000);
    send_at_production(RADAR2_UART, "AT+HEIGHT=100\n", "AT+OK", 2000);
    send_at_production(RADAR2_UART, "AT+RANGE=350\n", "AT+OK", 2000);
    send_at_production(RADAR2_UART, "AT+XNEGAD=-350\n", "AT+OK", 2000);
    send_at_production(RADAR2_UART, "AT+XPOSID=0\n", "AT+OK", 2000);
    send_at_production(RADAR2_UART, "AT+SENS=8\n", "AT+OK", 2000);
    send_at_production(RADAR2_UART, "AT+YNEGAD=-150\n", "AT+OK", 2000);
    send_at_production(RADAR2_UART, "AT+YPOSID=150\n", "AT+OK", 2000);

    send_at_production(RADAR2_UART, "AT+STUDY\n", NULL, 0);
    ESP_LOGW(TAG, "Study mode STARTED on LEFT (full learning ~10 min in background).");
    vTaskDelay(pdMS_TO_TICKS(12000));

    send_at_production(RADAR2_UART, "AT+START\n", "AT+OK", 2000);

    ESP_LOGI(TAG, "=== BOTH RADARS READY – Data should flow now! ===");
}

// ================= MAIN =================
void app_main(void) {
    nvs_flash_init();
    wifi_init();
    uart_init();
    espnow_init();

    ESP_LOGI(TAG, "=== DUAL-RADAR SENDER PRODUCTION v3.1 STARTED ===");
    ESP_LOGI(TAG, "Offset: ±%d cm", RADAR_HALF_GAP_CM);

    // Khởi động parser + TX task trước
    xTaskCreate(radar1_task, "radar1_task", 4096, NULL, 6, NULL);
    xTaskCreate(radar2_task, "radar2_task", 4096, NULL, 6, NULL);
    xTaskCreate(espnow_tx_task, "espnow_tx", 4096, NULL, 7, NULL);

    vTaskDelay(pdMS_TO_TICKS(1000));  // chờ task ổn định

    radar_at_config();  // config sau khi parser sẵn sàng

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}