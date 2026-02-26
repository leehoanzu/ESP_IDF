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
// #define RADAR_HALF_GAP_CM   15        // ← Điều chỉnh theo khoảng cách thực

// /* ── Sensor height ── */
// #define RADAR_HEIGHT_CM  267          // Chiều cao lắp đặt (cm)

// /* ── radar_id bytes ── */
// #define RADAR_ID_RIGHT   0x01
// #define RADAR_ID_LEFT    0x02

// /* ── Misc ── */
// #define RADAR_BUF_SIZE   2048
// #define RADAR_RANGE_CM   400

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

// // ================= ESP-NOW RECV =================
// // Receiver gửi lệnh AT về → chuyển thẳng xuống RADAR_1 (mặc định)
// // Nếu cần điều khiển từng radar, thêm prefix vào lệnh.
// void OnDataRecv(const esp_now_recv_info_t *info,
//                 const uint8_t *data,
//                 int len)
// {
//     if (len > 1 && data[0] == 0x02) {
//         // Forward xuống cả 2 radar
//         uart_write_bytes(RADAR1_UART, (const char *)(data + 1), len - 1);
//         uart_write_bytes(RADAR2_UART, (const char *)(data + 1), len - 1);
//         printf("\n>> Cmd fwd both radars: ");
//         fwrite(data + 1, 1, len - 1, stdout);
//         printf("\n");
//     }
// }

// // ================= SEND FRAME =================
// static void sendRadarFrame(uint8_t *data, int len, uint8_t radar_id)
// {
//     /*
//      * Packet: [0x01][seq][lenH][lenL][radar_id][payload…]
//      * payload = radar frame gốc, tối đa 224 byte/chunk
//      */
//     int     offset = 0;
//     uint8_t seq    = 0;
//     uint8_t pkt[250];

//     while (offset < len) {
//         int chunk = len - offset;
//         if (chunk > 224) chunk = 224;      // ESP-NOW max 250; header 5 byte + 224 payload = 229 ✓

//         pkt[0] = 0x01;
//         pkt[1] = seq++;
//         pkt[2] = (len >> 8) & 0xFF;
//         pkt[3] = len & 0xFF;
//         pkt[4] = radar_id;                 // ← Phân biệt LEFT / RIGHT

//         memcpy(pkt + 5, data + offset, chunk);
//         esp_now_send(receiverMAC, pkt, chunk + 5);

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
//     uint8_t b;
//     uint64_t lastPing = 0;

//     while (1) {
//         // Ping mỗi 2 s
//         uint64_t now = esp_timer_get_time() / 1000;
//         if (now - lastPing > 2000) {
//             lastPing = now;
//             uint8_t pingPkt[] = {0x01, 0x00, 0x00, 0x00, RADAR_ID_RIGHT};
//             esp_now_send(receiverMAC, pingPkt, sizeof(pingPkt));
//             printf("[R1] Ping...\n");
//         }

//         int len = uart_read_bytes(RADAR1_UART, &b, 1, pdMS_TO_TICKS(10));
//         if (len > 0)
//             process_byte(b, radarBuf1, &radarIdx1, RADAR1_UART, RADAR_ID_RIGHT);

//         if (radarIdx1 >= RADAR_BUF_SIZE) radarIdx1 = 0;
//     }
// }

// // ================= TASK RADAR 2 (LEFT) =================
// static void radar2_task(void *pv)
// {
//     uint8_t b;
//     uint64_t lastPing = 0;

//     while (1) {
//         uint64_t now = esp_timer_get_time() / 1000;
//         if (now - lastPing > 2000) {
//             lastPing = now;
//             uint8_t pingPkt[] = {0x01, 0x00, 0x00, 0x00, RADAR_ID_LEFT};
//             esp_now_send(receiverMAC, pingPkt, sizeof(pingPkt));
//             printf("[R2] Ping...\n");
//         }

//         int len = uart_read_bytes(RADAR2_UART, &b, 1, pdMS_TO_TICKS(10));
//         if (len > 0)
//             process_byte(b, radarBuf2, &radarIdx2, RADAR2_UART, RADAR_ID_LEFT);

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


#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "driver/uart.h"
#include "esp_timer.h"

// ================= CONFIG =================
uint8_t receiverMAC[] = {0x1C , 0xDB , 0xD4 , 0x76 , 0x7C , 0x90};

#define RADAR_UART       UART_NUM_2
#define RADAR_RX_PIN     16
#define RADAR_TX_PIN     17

// Sửa từ chân 16/17 sang 41/42
// #define RADAR_UART       UART_NUM_1   // Bạn nên đổi sang UART_NUM_1 nếu UART_NUM_2 gặp vấn đề
// #define RADAR_RX_PIN     41           // Chân nhận (RX)
// #define RADAR_TX_PIN     42           // Chân truyền (TX)

#define RADAR_BUF_SIZE   2048
static uint8_t radarBuf[RADAR_BUF_SIZE];
static int radarIndex = 0;

static uint64_t lastPing = 0;

// ================= ESP-NOW RECV =================
void OnDataRecv(const esp_now_recv_info_t *info,
                const uint8_t *data,
                int len)
{
    if (len > 1 && data[0] == 0x02)
    {
        uart_write_bytes(RADAR_UART,
                         (const char *)(data + 1),
                         len - 1);

        printf("\n>> Cmd: ");
        fwrite(data + 1, 1, len - 1, stdout);
        printf("\n");
    }
}

// ================= SEND FRAME =================
void sendRadarFrame(uint8_t *data, int len)
{
    int offset = 0;
    uint8_t pkt[250];
    uint8_t seq = 0;

    while (offset < len)
    {
        int chunk = len - offset;
        if (chunk > 230)
            chunk = 230;

        pkt[0] = 0x01;
        pkt[1] = seq++;
        pkt[2] = (len >> 8) & 0xFF;
        pkt[3] = len & 0xFF;

        memcpy(pkt + 4, data + offset, chunk);
        esp_now_send(receiverMAC, pkt, chunk + 4);

        offset += chunk;
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

// ================= WIFI INIT =================
void wifi_init(void)
{
    esp_netif_init();
    esp_event_loop_create_default();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_start();
}

// ================= UART INIT =================
void uart_init(void)
{
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    uart_driver_install(RADAR_UART, 4096, 0, 0, NULL, 0);
    uart_param_config(RADAR_UART, &uart_config);
    uart_set_pin(RADAR_UART,
                 RADAR_TX_PIN,
                 RADAR_RX_PIN,
                 UART_PIN_NO_CHANGE,
                 UART_PIN_NO_CHANGE);
}

// ================= ESP-NOW INIT =================
void espnow_init(void)
{
    esp_now_init();
    esp_now_register_recv_cb(OnDataRecv);

    esp_now_peer_info_t peer = {0};
    memcpy(peer.peer_addr, receiverMAC, 6);
    peer.channel = 0;
    peer.encrypt = false;

    esp_now_add_peer(&peer);
}

// ================= LOOP TASK =================
void loop_task(void *pv)
{
    uint8_t b;

    while (1)
    {
        // ----- Ping má»—i 2 giÃ¢y -----
        uint64_t now = esp_timer_get_time() / 1000;
        if (now - lastPing > 2000)
        {
            lastPing = now;
            uint8_t pingPkt[] = { 0x01, 0x00, 0x00, 0x00 };
            esp_now_send(receiverMAC, pingPkt, 4);
            printf("\n[System] Ping...\n");
        }

        // ----- Radar Ä‘á»c tá»«ng byte -----
        int len = uart_read_bytes(RADAR_UART,
                                  &b,
                                  1,
                                  pdMS_TO_TICKS(10));

        if (len > 0)
        {
            // In ra mọi byte nhận được từ UART để kiểm tra Radar có sống không
            printf("%02X ", b);

            // DEBUG HEX y chang Arduino
            if (b < 16) printf("0");
            printf("%X ", b);
            if (b == 0x0A) printf("\n");

            radarBuf[radarIndex++] = b;

            if (radarIndex >= 8)
            {
                bool headerMatch =
                    (radarBuf[0] == 1 &&
                     radarBuf[1] == 2 &&
                     radarBuf[2] == 3 &&
                     radarBuf[3] == 4 &&
                     radarBuf[4] == 5 &&
                     radarBuf[5] == 6 &&
                     radarBuf[6] == 7 &&
                     radarBuf[7] == 8);

                if (!headerMatch)
                {
                    memmove(radarBuf,
                            radarBuf + 1,
                            radarIndex - 1);
                    radarIndex--;
                    continue;
                }

                if (radarIndex >= 12)
                {
                    uint32_t packetLen;
                    memcpy(&packetLen,
                           &radarBuf[8],
                           4);

                    uint32_t totalFrameSize = packetLen;

                    if (totalFrameSize > RADAR_BUF_SIZE)
                    {
                        radarIndex = 0;
                        continue;
                    }

                    if (radarIndex >= totalFrameSize)
                    {
                        printf("\n[Frame Sent], Size = %lu\n",
                               totalFrameSize);

                        sendRadarFrame(radarBuf,
                                       totalFrameSize);

                        memmove(radarBuf,
                                radarBuf + totalFrameSize,
                                radarIndex - totalFrameSize);

                        radarIndex -= totalFrameSize;
                    }
                }
            }

            if (radarIndex >= RADAR_BUF_SIZE)
                radarIndex = 0;
        }
    }
}

// ================= MAIN =================
void app_main(void)
{
    nvs_flash_init();
    wifi_init();
    uart_init();
    espnow_init();

    printf("Sender Ready (Debug Hex Mode).\n");

    xTaskCreate(loop_task,
                "loop_task",
                4096,
                NULL,
                5,
                NULL);
}







///////////////////////////////////////test////////////

/*
// sender.c - Dual Radar Sender (Micro Level)
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "driver/uart.h"

// ================= CẤU HÌNH HỆ THỐNG =================
uint8_t receiverMAC[] = {0x1C , 0xDB , 0xD4 , 0x76 , 0x7C , 0x90};

#define UNIT_ID            0x0A       // ID của Cục này (Cục A). Cục thứ 2 sẽ là 0x0B

// Radar RIGHT (0x01) - Quản lý nửa phải
#define RADAR_RIGHT_UART   UART_NUM_2
#define RADAR_RIGHT_RX     16
#define RADAR_RIGHT_TX     17
#define RADAR_RIGHT_TIME   131        // Chu kỳ nguyên tố 1
#define RADAR_RIGHT_ID     0x01

// Radar LEFT (0x02) - Quản lý nửa trái
#define RADAR_LEFT_UART    UART_NUM_1
#define RADAR_LEFT_RX      41
#define RADAR_LEFT_TX      42
#define RADAR_LEFT_TIME    101        // Chu kỳ nguyên tố 2
#define RADAR_LEFT_ID      0x02

#define RADAR_BUF_SIZE     2048

static uint8_t bufRight[RADAR_BUF_SIZE];
static int idxRight = 0;
static uint8_t bufLeft[RADAR_BUF_SIZE];
static int idxLeft = 0;

static void send_at_cmd(uart_port_t port, const char *cmd) {
    uart_write_bytes(port, cmd, strlen(cmd));
    vTaskDelay(pdMS_TO_TICKS(150));
}

// ================= ĐÓNG GÓI ESP-NOW =================
// Chuẩn mới: [0x01][seq][lenH][lenL][UNIT_ID][RADAR_ID][Payload...]
void sendRadarFrame(uint8_t *data, int len, uint8_t radar_id) {
    int offset = 0;
    uint8_t pkt[250];
    static uint8_t seq = 0;

    while (offset < len) {
        int chunk = len - offset;
        if (chunk > 224) chunk = 224;

        pkt[0] = 0x01;
        pkt[1] = seq++;
        pkt[2] = (len >> 8) & 0xFF;
        pkt[3] = len & 0xFF;
        pkt[4] = UNIT_ID;       // [MỞ RỘNG ĐA CỤC] Định danh Cục
        pkt[5] = radar_id;      // Định danh Radar nội bộ

        memcpy(pkt + 6, data + offset, chunk);
        esp_now_send(receiverMAC, pkt, chunk + 6);

        offset += chunk;
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

// ================= XỬ LÝ UART =================
void process_uart_data(uart_port_t port, uint8_t id, uint8_t *buf, int *idx) {
    uint8_t b;
    int len = uart_read_bytes(port, &b, 1, pdMS_TO_TICKS(20));
    if (len > 0) {
        buf[(*idx)++] = b;
        if (*idx >= 8) {
            bool headerMatch = (buf[0]==1 && buf[1]==2 && buf[2]==3 && buf[3]==4 && 
                                buf[4]==5 && buf[5]==6 && buf[6]==7 && buf[7]==8);
            if (!headerMatch) {
                memmove(buf, buf + 1, *idx - 1);
                (*idx)--; return;
            }
            if (*idx >= 12) {
                uint32_t packetLen;
                memcpy(&packetLen, &buf[8], 4);
                if (packetLen > RADAR_BUF_SIZE) { *idx = 0; return; }
                if (*idx >= packetLen) {
                    sendRadarFrame(buf, packetLen, id);
                    memmove(buf, buf + packetLen, *idx - packetLen);
                    *idx -= packetLen;
                }
            }
        }
        if (*idx >= RADAR_BUF_SIZE) *idx = 0;
    }
}

void radar_task(void *pv) {
    while (1) {
        process_uart_data(RADAR_RIGHT_UART, RADAR_RIGHT_ID, bufRight, &idxRight);
        process_uart_data(RADAR_LEFT_UART, RADAR_LEFT_ID, bufLeft, &idxLeft);
    }
}

// ================= CẤU HÌNH PHÂN VÙNG CỨNG =================
void config_radars() {
    printf("Configuring R_RIGHT (Time: 131ms, Partition: X > -50)\n");
    send_at_cmd(RADAR_RIGHT_UART, "AT+STOP\n");
    send_at_cmd(RADAR_RIGHT_UART, "AT+TIME=131\n"); // Chống nhiễu
    send_at_cmd(RADAR_RIGHT_UART, "AT+XNEGAD=-50\n"); // Nới -50 để tạo vùng Hysteresis
    send_at_cmd(RADAR_RIGHT_UART, "AT+XPOSID=350\n");
    send_at_cmd(RADAR_RIGHT_UART, "AT+START\n");

    printf("Configuring R_LEFT (Time: 101ms, Partition: X < 50)\n");
    send_at_cmd(RADAR_LEFT_UART, "AT+STOP\n");
    send_at_cmd(RADAR_LEFT_UART, "AT+TIME=101\n");
    send_at_cmd(RADAR_LEFT_UART, "AT+XNEGAD=-350\n");
    send_at_cmd(RADAR_LEFT_UART, "AT+XPOSID=50\n"); // Nới +50 để tạo vùng Hysteresis
    send_at_cmd(RADAR_LEFT_UART, "AT+START\n");
}

void app_main(void) {
    nvs_flash_init(); esp_netif_init(); esp_event_loop_create_default();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg); esp_wifi_set_mode(WIFI_MODE_STA); esp_wifi_start();
    esp_now_init();
    
    esp_now_peer_info_t peer = {0};
    memcpy(peer.peer_addr, receiverMAC, 6);
    esp_now_add_peer(&peer);

    uart_config_t u_cfg = {.baud_rate = 115200, .data_bits = UART_DATA_8_BITS, .parity = UART_PARITY_DISABLE, .stop_bits = UART_STOP_BITS_1};
    uart_driver_install(RADAR_RIGHT_UART, 4096, 0, 0, NULL, 0);
    uart_param_config(RADAR_RIGHT_UART, &u_cfg);
    uart_set_pin(RADAR_RIGHT_UART, RADAR_RIGHT_TX, RADAR_RIGHT_RX, -1, -1);

    uart_driver_install(RADAR_LEFT_UART, 4096, 0, 0, NULL, 0);
    uart_param_config(RADAR_LEFT_UART, &u_cfg);
    uart_set_pin(RADAR_LEFT_UART, RADAR_LEFT_TX, RADAR_LEFT_RX, -1, -1);

    vTaskDelay(pdMS_TO_TICKS(2000));
    config_radars();
    xTaskCreate(radar_task, "radar_task", 4096, NULL, 5, NULL);
}


*/

/*
 * sender.c — Dual Radar Sender
 * =============================================================
 * Kiến trúc 2 radar trong 1 cục (khoảng cách gần nhau ~30–70 cm):
 *
 *   R1 = RADAR_RIGHT  (UART2, pin 16/17)  →  phủ nửa PHẢI  (X ≥ 0)
 *   R2 = RADAR_LEFT   (UART1, pin 41/42)  →  phủ nửa TRÁI  (X < 0)
 *
 * ── Spatial Partition (phân vùng cứng) ──────────────────────────
 *   R1: AT+XNEGAD=0   · AT+XPOSID=350   → firmware loại X < 0
 *   R2: AT+XNEGAD=-350· AT+XPOSID=0     → firmware loại X > 0
 *   → 2 radar không bao giờ báo cùng 1 mục tiêu.
 *
 * ── Prime Stagger (giảm RF coupling) ────────────────────────────
 *   R1: AT+TIME=131 ms
 *   R2: AT+TIME=101 ms
 *   LCM(101,131) = 13 231 ms → va chạm < 1 lần / 13 s
 *
 * ── Coordinate Calibration (bù lệch vật lý) ─────────────────────
 *   Gắn radar_id vào packet; receiver bù offset ±X_OFFSET_CM.
 *   R1 lắp bên PHẢI tâm cục → offset = +RADAR_HALF_GAP_CM
 *   R2 lắp bên TRÁI tâm cục → offset = -RADAR_HALF_GAP_CM
 *
 * ── Packet format gửi qua ESP-NOW ───────────────────────────────
 *   [0x01][seq][lenH][lenL][radar_id][payload…]
 *    byte0  1    2     3      4         5+
 *   radar_id: RADAR_ID_RIGHT=0x01, RADAR_ID_LEFT=0x02
 * =============================================================
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "driver/uart.h"
#include "esp_timer.h"

// ================= CONFIG =================
static const uint8_t receiverMAC[] = {0x1C, 0xDB, 0xD4, 0x76, 0x7C, 0x90};

/* ── Radar 1 (RIGHT): phủ nửa PHẢI, X ∈ [0, +350 cm] ── */
#define RADAR1_UART      UART_NUM_2
#define RADAR1_RX_PIN    16
#define RADAR1_TX_PIN    17
#define RADAR1_TIME_MS   131          // Prime Stagger: chu kỳ 131 ms
#define RADAR1_XNEG      0            // Spatial Partition: chặn X âm
#define RADAR1_XPOS      350          // Mức X dương tối đa (cm)

/* ── Radar 2 (LEFT): phủ nửa TRÁI, X ∈ [-350, 0 cm] ── */
#define RADAR2_UART      UART_NUM_1
#define RADAR2_RX_PIN    41
#define RADAR2_TX_PIN    42
#define RADAR2_TIME_MS   101          // Prime Stagger: chu kỳ 101 ms
#define RADAR2_XNEG      (-350)       // Mức X âm tối đa (cm)
#define RADAR2_XPOS      0            // Spatial Partition: chặn X dương

/* ── Coordinate Calibration ── */
// Nếu 2 radar đặt gần tâm (khoảng cách nhỏ), offset nhỏ.
// Đo thực tế: khoảng cách 2 radar / 2 (cm). Ví dụ 30cm → 15cm.
#define RADAR_HALF_GAP_CM   15      // 70cm center-to-center / 2

/* ── Sensor height ── */
#define RADAR_HEIGHT_CM  100   ///267          // Chiều cao lắp đặt (cm)

/* ── radar_id bytes ── */
#define RADAR_ID_RIGHT   0x01
#define RADAR_ID_LEFT    0x02

/* ── Misc ── */
#define RADAR_BUF_SIZE   2048
#define RADAR_RANGE_CM   350          // Thu hẹp tầm quét giảm nhiễu xa

// ================= BUFFERS =================
static uint8_t radarBuf1[RADAR_BUF_SIZE];
static int     radarIdx1 = 0;

static uint8_t radarBuf2[RADAR_BUF_SIZE];
static int     radarIdx2 = 0;

// ================= HELPERS =================
static void send_at(uart_port_t port, const char *cmd)
{

    uart_write_bytes(port, cmd, strlen(cmd));
    vTaskDelay(pdMS_TO_TICKS(300));
    printf("[AT→UART%d] %s", (port == RADAR1_UART) ? 2 : 1, cmd);
}

// ================= ESP-NOW RECV =================
// Receiver gửi lệnh AT về → chuyển thẳng xuống RADAR_1 (mặc định)
// Nếu cần điều khiển từng radar, thêm prefix vào lệnh.
void OnDataRecv(const esp_now_recv_info_t *info,
                const uint8_t *data,
                int len)
{
    if (len > 1 && data[0] == 0x02) {
        // Forward xuống cả 2 radar
        uart_write_bytes(RADAR1_UART, (const char *)(data + 1), len - 1);
        uart_write_bytes(RADAR2_UART, (const char *)(data + 1), len - 1);
        printf("\n>> Cmd fwd both radars: ");
        fwrite(data + 1, 1, len - 1, stdout);
        printf("\n");
    }
}

// ================= SEND FRAME =================
static void sendRadarFrame(uint8_t *data, int len, uint8_t radar_id)
{
    /*
     * Packet: [0x01][seq][lenH][lenL][radar_id][payload…]
     * payload = radar frame gốc, tối đa 224 byte/chunk
     */
    int     offset = 0;
    uint8_t seq    = 0;
    uint8_t pkt[250];

    while (offset < len) {
        int chunk = len - offset;
        if (chunk > 224) chunk = 224;      // ESP-NOW max 250; header 5 byte + 224 payload = 229 ✓

        pkt[0] = 0x01;
        pkt[1] = seq++;
        pkt[2] = (len >> 8) & 0xFF;
        pkt[3] = len & 0xFF;
        pkt[4] = radar_id;                 // ← Phân biệt LEFT / RIGHT

        memcpy(pkt + 5, data + offset, chunk);
        esp_now_send(receiverMAC, pkt, chunk + 5);

        offset += chunk;
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

// ================= WIFI INIT =================
static void wifi_init(void)
{
    esp_netif_init();
    esp_event_loop_create_default();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_start();
}

// ================= UART INIT =================
static void uart_init_one(uart_port_t port, int tx, int rx)
{
    uart_config_t cfg = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_driver_install(port, 4096, 0, 0, NULL, 0);
    uart_param_config(port, &cfg);
    uart_set_pin(port, tx, rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

static void uart_init(void)
{
    uart_init_one(RADAR1_UART, RADAR1_TX_PIN, RADAR1_RX_PIN);
    uart_init_one(RADAR2_UART, RADAR2_TX_PIN, RADAR2_RX_PIN);
}

// ================= AT CONFIG =================
/*
 * Spatial Partition + Prime Stagger + Height
 * Gọi sau khi UART đã sẵn sàng. Chờ radar boot ~1–2 giây trước.
 *
 * Lưu ý: AT+RESTORE khôi phục factory → xóa cấu hình cũ.
 * Nếu muốn giữ lại cấu hình khác (VD: AT+SENS), bỏ dòng RESTORE
 * và chỉ gửi những lệnh cần thay đổi.
 */
static void radar_at_config(void)
{
    printf("\n[CFG] Configuring Radar 1 (RIGHT, %dms)...\n", RADAR1_TIME_MS);
    send_at(RADAR1_UART, "AT+RESTORE\n");
    send_at(RADAR1_UART, "AT+STOP\n");
    vTaskDelay(pdMS_TO_TICKS(500));

    char buf[64];
    snprintf(buf, sizeof(buf), "AT+TIME=%d\n", RADAR1_TIME_MS);
    send_at(RADAR1_UART, buf);

    snprintf(buf, sizeof(buf), "AT+HEIGHT=%d\n", RADAR_HEIGHT_CM);
    send_at(RADAR1_UART, buf);

    snprintf(buf, sizeof(buf), "AT+HEIGHTD=%d\n", RADAR_HEIGHT_CM);
    send_at(RADAR1_UART, buf);

    snprintf(buf, sizeof(buf), "AT+RANGE=%d\n", RADAR_RANGE_CM);
    send_at(RADAR1_UART, buf);

    snprintf(buf, sizeof(buf), "AT+XNEGAD=%d\n", RADAR1_XNEG);
    send_at(RADAR1_UART, buf);

    snprintf(buf, sizeof(buf), "AT+XPOSID=%d\n", RADAR1_XPOS);
    send_at(RADAR1_UART, buf);

    send_at(RADAR1_UART, "AT+SENS=8\n");
    send_at(RADAR1_UART, "AT+YNEGAD=-150\n");
    send_at(RADAR1_UART, "AT+YPOSID=150\n");

    send_at(RADAR1_UART, "AT+START\n");
    printf("[CFG] Radar 1 (RIGHT) configured.\n");

    // ---- Radar 2 ----
    printf("\n[CFG] Configuring Radar 2 (LEFT, %dms)...\n", RADAR2_TIME_MS);
    send_at(RADAR2_UART, "AT+RESTORE\n");
    send_at(RADAR2_UART, "AT+STOP\n");
    vTaskDelay(pdMS_TO_TICKS(500));

    snprintf(buf, sizeof(buf), "AT+TIME=%d\n", RADAR2_TIME_MS);
    send_at(RADAR2_UART, buf);

    snprintf(buf, sizeof(buf), "AT+HEIGHT=%d\n", RADAR_HEIGHT_CM);
    send_at(RADAR2_UART, buf);

    snprintf(buf, sizeof(buf), "AT+HEIGHTD=%d\n", RADAR_HEIGHT_CM);
    send_at(RADAR2_UART, buf);

    snprintf(buf, sizeof(buf), "AT+RANGE=%d\n", RADAR_RANGE_CM);
    send_at(RADAR2_UART, buf);

    snprintf(buf, sizeof(buf), "AT+XNEGAD=%d\n", RADAR2_XNEG);
    send_at(RADAR2_UART, buf);

    snprintf(buf, sizeof(buf), "AT+XPOSID=%d\n", RADAR2_XPOS);
    send_at(RADAR2_UART, buf);

    send_at(RADAR2_UART, "AT+SENS=8\n");
    send_at(RADAR2_UART, "AT+YNEGAD=-150\n");
    send_at(RADAR2_UART, "AT+YPOSID=150\n");

    send_at(RADAR2_UART, "AT+START\n");
    printf("[CFG] Radar 2 (LEFT) configured.\n\n");
}

// ================= ESPNOW INIT =================
static void espnow_init(void)
{
    esp_now_init();
    esp_now_register_recv_cb(OnDataRecv);

    esp_now_peer_info_t peer = {0};
    memcpy(peer.peer_addr, receiverMAC, 6);
    peer.channel = 0;
    peer.encrypt = false;
    esp_now_add_peer(&peer);
}

// ================= FRAME PARSER (dùng chung) =================
/*
 * Đọc từng byte từ UART và ghép thành frame.
 * Header 8 byte: 01 02 03 04 05 06 07 08
 * Byte [8..11]  : packetLen (uint32_t LE)
 *
 * Trả về 1 khi đã gửi xong 1 frame đầy đủ, 0 nếu chưa.
 */
static int process_byte(uint8_t byte,
                         uint8_t  *buf,
                         int      *idx,
                         uart_port_t port,
                         uint8_t  radar_id)
{
    buf[(*idx)++] = byte;

    // Bước 1: Chờ đủ 8 byte header
    if (*idx >= 8) {
        bool hdr = (buf[0]==1 && buf[1]==2 && buf[2]==3 && buf[3]==4 &&
                    buf[4]==5 && buf[5]==6 && buf[6]==7 && buf[7]==8);
        if (!hdr) {
            memmove(buf, buf + 1, *idx - 1);
            (*idx)--;
            return 0;
        }
    } else {
        return 0;
    }

    // Bước 2: Chờ đủ 12 byte để đọc packetLen
    if (*idx < 12) return 0;

    uint32_t pktLen;
    memcpy(&pktLen, &buf[8], 4);

    if (pktLen > RADAR_BUF_SIZE) {
        *idx = 0;
        return 0;
    }

    // Bước 3: Chờ đủ frame
    if (*idx < (int)pktLen) return 0;

    // Gửi frame qua ESP-NOW
    const char *zoneName = (radar_id == RADAR_ID_RIGHT) ? "RIGHT" : "LEFT";
    printf("\n[%s Frame] size=%lu\n", zoneName, pktLen);
    sendRadarFrame(buf, (int)pktLen, radar_id);

    // Xoá phần đã gửi, giữ lại phần dư
    memmove(buf, buf + pktLen, *idx - pktLen);
    *idx -= pktLen;
    return 1;
}

// ================= TASK RADAR 1 (RIGHT) =================
static void radar1_task(void *pv)
{
    uint8_t b;
    uint64_t lastPing = 0;

    while (1) {
        // Ping mỗi 2 s
        uint64_t now = esp_timer_get_time() / 1000;
        if (now - lastPing > 2000) {
            lastPing = now;
            uint8_t pingPkt[] = {0x01, 0x00, 0x00, 0x00, RADAR_ID_RIGHT};
            esp_now_send(receiverMAC, pingPkt, sizeof(pingPkt));
            printf("[RIGHT] Ping...\n");
        }

        int len = uart_read_bytes(RADAR1_UART, &b, 1, pdMS_TO_TICKS(10));
        if (len > 0)
            process_byte(b, radarBuf1, &radarIdx1, RADAR1_UART, RADAR_ID_RIGHT);

        if (radarIdx1 >= RADAR_BUF_SIZE) radarIdx1 = 0;
    }
}

// ================= TASK RADAR 2 (LEFT) =================
static void radar2_task(void *pv)
{
    uint8_t b;
    uint64_t lastPing = 0;

    while (1) {
        uint64_t now = esp_timer_get_time() / 1000;
        if (now - lastPing > 2000) {
            lastPing = now;
            uint8_t pingPkt[] = {0x01, 0x00, 0x00, 0x00, RADAR_ID_LEFT};
            esp_now_send(receiverMAC, pingPkt, sizeof(pingPkt));
            printf("[LEFT] Ping...\n");
        }

        int len = uart_read_bytes(RADAR2_UART, &b, 1, pdMS_TO_TICKS(10));
        if (len > 0)
            process_byte(b, radarBuf2, &radarIdx2, RADAR2_UART, RADAR_ID_LEFT);

        if (radarIdx2 >= RADAR_BUF_SIZE) radarIdx2 = 0;
    }
}

// ================= MAIN =================
void app_main(void)
{
    nvs_flash_init();
    wifi_init();
    uart_init();
    espnow_init();

    printf("\n===================================================\n");
    printf("  Dual-Radar Sender\n");
    printf("  R1 RIGHT: UART2 (pin%d/%d), TIME=%dms, X=[%d..%d]cm\n",
           RADAR1_RX_PIN, RADAR1_TX_PIN, RADAR1_TIME_MS,
           RADAR1_XNEG, RADAR1_XPOS);
    printf("  R2 LEFT : UART1 (pin%d/%d), TIME=%dms, X=[%d..%d]cm\n",
           RADAR2_RX_PIN, RADAR2_TX_PIN, RADAR2_TIME_MS,
           RADAR2_XNEG, RADAR2_XPOS);
    printf("  Calibration offset: +/- %d cm\n", RADAR_HALF_GAP_CM);
    printf("===================================================\n\n");

    // Chờ radar boot xong rồi mới gửi AT
    vTaskDelay(pdMS_TO_TICKS(2000));
    radar_at_config();

    xTaskCreate(radar1_task, "radar1_task", 4096, NULL, 5, NULL);
    xTaskCreate(radar2_task, "radar2_task", 4096, NULL, 5, NULL);
}

