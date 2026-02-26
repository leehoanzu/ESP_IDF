// /*
//  * radarreceiver.c — Dual Radar Receiver
//  * =============================================================
//  * Nhận dữ liệu từ 2 radar qua ESP-NOW:
//  *   radar_id = 0x01  →  Radar RIGHT (nửa phải,  X ≥ 0)
//  *   radar_id = 0x02  →  Radar LEFT  (nửa trái,  X < 0)
//  *
//  * ── Coordinate Calibration ──────────────────────────────────
//  *   Radar RIGHT lắp lệch +RADAR_HALF_GAP_CM so với tâm cục
//  *     → x_cal = x_raw + RADAR_HALF_GAP_CM
//  *   Radar LEFT  lắp lệch -RADAR_HALF_GAP_CM so với tâm cục
//  *     → x_cal = x_raw - RADAR_HALF_GAP_CM
//  *   Điều chỉnh RADAR_HALF_GAP_CM cho phù hợp khoảng cách thực.
//  *
//  * ── Packet format nhận từ sender ────────────────────────────
//  *   [0x01][seq][lenH][lenL][radar_id][radar_frame_payload…]
//  *    byte0  1    2     3      4           5+
//  *
//  * ── Hysteresis Boundary (tránh double-count) ────────────────
//  *   Khi người đứng gần ranh giới X=0 (|x_cal| < BOUNDARY_HYST_CM):
//  *   Chỉ in dữ liệu từ radar đang "sở hữu" người đó (last_owner).
//  * =============================================================
//  */

// #include <stdio.h>
// #include <string.h>
// #include <stdlib.h>
// #include <math.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "esp_wifi.h"
// #include "esp_now.h"
// #include "esp_event.h"
// #include "nvs_flash.h"
// #include "esp_netif.h"
// #include "esp_timer.h"
// #include "esp_mac.h"

// // ================= CONFIG =================
// // Điều chỉnh theo khoảng cách vật lý giữa 2 radar / 2 (cm).
// // Ví dụ: 2 radar đặt cách nhau 30 cm → RADAR_HALF_GAP_CM = 15
// #define RADAR_HALF_GAP_CM    15

// // Vùng hysteresis quanh ranh giới X=0 (cm).
// // Trong vùng này không đổi chủ radar, giảm dao động.
// #define BOUNDARY_HYST_CM     30

// // Radar ID
// #define RADAR_ID_RIGHT       0x01
// #define RADAR_ID_LEFT        0x02

// // ================= GLOBAL =================
// static uint8_t senderMAC[6];
// static bool    paired    = false;
// static bool    isStopped = false;

// #define RX_BUF_SIZE 4096

// /* Mỗi radar có buffer riêng để ghép packet multi-chunk */
// static uint8_t rxBuf1[RX_BUF_SIZE];
// static int     rxIdx1 = 0;
// static int     rxTotal1 = 0;   // totalLen kỳ vọng

// static uint8_t rxBuf2[RX_BUF_SIZE];
// static int     rxIdx2 = 0;
// static int     rxTotal2 = 0;

// /* Hysteresis: nhớ radar nào đang sở hữu mỗi người (theo index) */
// #define MAX_PERSONS 8
// static uint8_t last_owner[MAX_PERSONS]; // RADAR_ID_RIGHT / RADAR_ID_LEFT / 0=unknown

// // ================= STRUCT =================
// typedef struct {
//     uint32_t reserved;
//     uint32_t id;
//     float    x, y, z;
//     float    vx, vy, vz;
// } PersonData;   // 32 bytes

// // ================= COORDINATE CALIBRATION =================
// /*
//  * Bù lệch vật lý của từng radar so với tâm cục.
//  * x_raw: tọa độ X từ radar firmware (cm).
//  * radar_id: 0x01=RIGHT, 0x02=LEFT.
//  * Trả về x đã hiệu chỉnh về tâm cục.
//  */
// static float calibrate_x(float x_raw, uint8_t radar_id)
// {
//     if (radar_id == RADAR_ID_RIGHT)
//         return x_raw + RADAR_HALF_GAP_CM;   // radar lắp bên phải tâm → bù trái
//     else
//         return x_raw - RADAR_HALF_GAP_CM;   // radar lắp bên trái tâm → bù phải
// }

// // ================= HYSTERESIS OWNER =================
// /*
//  * Quyết định radar nào hiển thị target i dựa vào vị trí x_cal.
//  * −BOUNDARY_HYST_CM < x_cal < +BOUNDARY_HYST_CM → giữ owner cũ.
//  */
// static bool should_display(uint8_t radar_id, float x_cal, int person_idx)
// {
//     if (person_idx < 0 || person_idx >= MAX_PERSONS)
//         return true; // Mặc định hiển thị nếu vượt giới hạn

//     float abs_x = (x_cal < 0) ? -x_cal : x_cal;

//     if (abs_x >= BOUNDARY_HYST_CM) {
//         // Ngoài vùng hysteresis → assign chắc chắn
//         uint8_t correct_owner = (x_cal >= 0) ? RADAR_ID_RIGHT : RADAR_ID_LEFT;
//         last_owner[person_idx] = correct_owner;
//         return (radar_id == correct_owner);
//     } else {
//         // Trong vùng hysteresis → giữ owner cũ
//         if (last_owner[person_idx] == 0)
//             last_owner[person_idx] = radar_id; // Lần đầu: giao cho radar phát hiện
//         return (radar_id == last_owner[person_idx]);
//     }
// }

// // ================= PARSE RADAR FRAME =================
// static void parseRadarFrame(uint8_t *frame, int len, uint8_t radar_id)
// {
//     // Kiểm tra header: 01 02 03 04 05 06 07 08
//     if (!(frame[0]==1 && frame[1]==2 && frame[2]==3 && frame[3]==4 &&
//           frame[4]==5 && frame[5]==6 && frame[6]==7 && frame[7]==8))
//         return;

//     uint32_t packetLen, frameID, trackLen;
//     memcpy(&packetLen, &frame[8],  4);
//     memcpy(&frameID,   &frame[12], 4);
//     memcpy(&trackLen,  &frame[28], 4);

//     if (packetLen <= 32 || trackLen < sizeof(PersonData)) {
//         printf("[%s] Skip non-person frame\n",
//                (radar_id == RADAR_ID_RIGHT) ? "RIGHT" : "LEFT");
//         return;
//     }

//     if ((uint32_t)len < packetLen) packetLen = len;
//     if (trackLen % sizeof(PersonData) != 0) return;
//     if (32 + trackLen > packetLen) return;

//     uint32_t count = trackLen / sizeof(PersonData);
//     const char *zone = (radar_id == RADAR_ID_RIGHT) ? "RIGHT" : "LEFT";

//     printf("\n[%s] Frame=%lu  Persons=%lu\n", zone, frameID, count);

//     for (int i = 0; i < (int)count; i++) {
//         int off = 32 + i * sizeof(PersonData);
//         if (off + (int)sizeof(PersonData) > len) break;

//         PersonData p;
//         memcpy(&p, frame + off, sizeof(PersonData));

//         // ── Coordinate Calibration ──
//         float x_cal = calibrate_x(p.x, radar_id);

//         // ── Hysteresis Boundary ──
//         // Chỉ in nếu radar này đang là "owner" của target index i
//         if (!should_display(radar_id, x_cal, i)) {
//             printf("  [%s] Person %d  ID=%lu  x_cal=%.0fcm  → skipped (boundary)\n",
//                    zone, i + 1, p.id, x_cal * 100.0f);
//             continue;
//         }

//         printf("  [%s] Person %d:", zone, i + 1);
//         printf("  ID=%-4lu", p.id);
//         printf("  X=%.2fm(cal=%.0fcm)", p.x, x_cal * 100.0f);
//         printf("  Y=%.2f  Z=%.2f", p.y, p.z);
//         printf("  Vx=%.2f Vy=%.2f Vz=%.2f\n", p.vx, p.vy, p.vz);

//         vTaskDelay(pdMS_TO_TICKS(500)); // Giãn cách in từng person để dễ đọc
//     }
// }

// // ================= REASSEMBLE & PARSE =================
// /*
//  * Gọi khi nhận 1 chunk ESP-NOW cho radar_id.
//  * Ghép multi-chunk, khi đủ frame → parseRadarFrame.
//  */
// static void handle_chunk(const uint8_t *data, int len, uint8_t radar_id)
// {
//     // packet: [0x01][seq][lenH][lenL][radar_id][payload…]
//     // Hàm này nhận data từ byte[0] (đã bao gồm header ESP-NOW).
//     // Gọi từ OnDataRecv sau khi xác định radar_id ở byte[4].

//     if (len < 5 || data[0] != 0x01) return;

//     uint8_t seq      = data[1];
//     uint16_t total   = (data[2] << 8) | data[3];
//     // data[4] = radar_id (đã tách trước khi gọi)
//     const uint8_t *payload  = data + 5;
//     int            chunkLen = len - 5;

//     uint8_t *rxBuf;
//     int     *rxIdx;
//     int     *rxTotal;

//     if (radar_id == RADAR_ID_RIGHT) {
//         rxBuf = rxBuf1; rxIdx = &rxIdx1; rxTotal = &rxTotal1;
//     } else {
//         rxBuf = rxBuf2; rxIdx = &rxIdx2; rxTotal = &rxTotal2;
//     }

//     if (seq == 0) {
//         *rxIdx   = 0;
//         *rxTotal = total;
//     }

//     if (*rxIdx + chunkLen <= RX_BUF_SIZE) {
//         memcpy(rxBuf + *rxIdx, payload, chunkLen);
//         *rxIdx += chunkLen;
//     }

//     if (*rxIdx >= *rxTotal && *rxTotal > 0) {
//         if (!isStopped)
//             parseRadarFrame(rxBuf, *rxTotal, radar_id);
//         *rxIdx   = 0;
//         *rxTotal = 0;
//     }
// }

// // ================= SEND COMMAND =================
// static void sendCmd(const char *cmd)
// {
//     if (!paired) return;

//     uint8_t buf[200];
//     buf[0] = 0x02;

//     char temp[200];
//     snprintf(temp, sizeof(temp), "%s\n", cmd);
//     int l = strlen(temp);
//     if (l > 190) l = 190;
//     memcpy(buf + 1, temp, l);
//     esp_now_send(senderMAC, buf, l + 1);
//     printf("Sent: %s", temp);
// }

// // ================= SETUP SEQUENCE =================
// static void runSetup(void)
// {
//     isStopped = false;
//     printf("--- Running Setup Sequence ---\n");
//     printf("(Sender sẽ cấu hình Spatial Partition + Prime Stagger)\n");

//     // Chỉ gửi STOP → sender tự cấu hình AT khi restart
//     sendCmd("AT+STOP"); vTaskDelay(pdMS_TO_TICKS(500));
//     sendCmd("AT+SENS=5"); vTaskDelay(pdMS_TO_TICKS(300));
//     sendCmd("AT+START"); vTaskDelay(pdMS_TO_TICKS(300));
//     // sendCmd("AT+STOP"); vTaskDelay(pdMS_TO_TICKS(500));

//     printf("--- Setup Done ---\n");
// }

// // ================= ESP-NOW RECV =================
// void OnDataRecv(const esp_now_recv_info_t *info,
//                 const uint8_t *data,
//                 int len)
// {
//     // ── Pairing ──
//     if (!paired) {
//         memcpy(senderMAC, info->src_addr, 6);
//         esp_now_peer_info_t peer = {0};
//         memcpy(peer.peer_addr, senderMAC, 6);
//         peer.channel = 0;
//         peer.encrypt = false;
//         if (esp_now_add_peer(&peer) == ESP_OK) {
//             paired = true;
//             printf("--- PAIRED WITH SENDER ---\n");
//         }
//     }

//     if (len < 5 || data[0] != 0x01) return;

//     uint8_t radar_id = data[4];

//     // ── Ping packet (len=5, no payload) ──
//     if (len == 5) {
//         const char *zone = (radar_id == RADAR_ID_RIGHT) ? "RIGHT" : "LEFT";
//         printf("[PING] Radar %s alive\n", zone);
//         return;
//     }

//     // ── Data: ghép và parse ──
//     handle_chunk(data, len, radar_id);
// }

// // ================= CONSOLE TASK =================
// static void console_task(void *pv)
// {
//     char input[200];
//     printf("Commands: SETUP | START | STOP | RESET | <AT command>\n");
//     printf("Use 'R1:<cmd>' or 'R2:<cmd>' to target a specific radar (via sender forward)\n\n");

//     while (1) {
//         if (fgets(input, sizeof(input), stdin)) {
//             input[strcspn(input, "\r\n")] = 0;

//             if (strcmp(input, "SETUP") == 0)
//                 runSetup();

//             else if (strcmp(input, "STOP") == 0) {
//                 isStopped = true;
//                 sendCmd("AT+STOP");
//                 vTaskDelay(pdMS_TO_TICKS(50));
//                 sendCmd("AT+STOP");
//                 printf(">> Sent STOP (both radars)\n");
//             }

//             else if (strcmp(input, "START") == 0) {
//                 isStopped = false;
//                 sendCmd("AT+START");
//                 printf(">> Sent START (both radars)\n");
//             }

//             else if (strcmp(input, "RESET") == 0) {
//                 sendCmd("AT+RESET");
//                 printf(">> Sent RESET\n");
//             }

//             else if (strcmp(input, "STATUS") == 0) {
//                 printf("\n=== Dual Radar Status ===\n");
//                 printf("  Paired: %s\n", paired ? "YES" : "NO");
//                 printf("  Stopped: %s\n", isStopped ? "YES" : "NO");
//                 printf("  R1 rxIdx=%d total=%d\n", rxIdx1, rxTotal1);
//                 printf("  R2 rxIdx=%d total=%d\n", rxIdx2, rxTotal2);
//                 printf("  Calibration offset: +/-%d cm\n", RADAR_HALF_GAP_CM);
//                 printf("  Boundary hysteresis: +/-%d cm\n", BOUNDARY_HYST_CM);
//                 printf("=========================\n");
//             }

//             else if (strlen(input) > 0)
//                 sendCmd(input);
//         }
//         vTaskDelay(pdMS_TO_TICKS(10));
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

// // ================= MAIN =================
// void app_main(void)
// {
//     // Init hysteresis table
//     memset(last_owner, 0, sizeof(last_owner));

//     nvs_flash_init();
//     wifi_init();

//     if (esp_now_init() != ESP_OK) {
//         printf("ESP-NOW init failed\n");
//         return;
//     }

//     uint8_t mac[6];
//     esp_read_mac(mac, ESP_MAC_WIFI_STA);
//     printf("\n==========================================\n");
//     printf("  DUAL-RADAR RECEIVER\n");
//     printf("  MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
//            mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
//     printf("  Radar RIGHT (id=0x01): X ≥ 0  +%dcm offset\n", RADAR_HALF_GAP_CM);
//     printf("  Radar LEFT  (id=0x02): X < 0  -%dcm offset\n", RADAR_HALF_GAP_CM);
//     printf("  Hysteresis zone: |X| < %d cm\n", BOUNDARY_HYST_CM);
//     printf("==========================================\n\n");

//     esp_now_register_recv_cb(OnDataRecv);

//     xTaskCreate(console_task, "console_task", 4096, NULL, 5, NULL);
// }


#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_timer.h"

#include "esp_mac.h" // Thư viện để đọc địa chỉ MAC

// ================= GLOBAL =================
uint8_t senderMAC[6];
bool paired = false;
bool isStopped = false;

#define RX_BUF_SIZE 4096
uint8_t rxBuffer[RX_BUF_SIZE];
int rxIndex = 0;

// ===== Struct Person (32 bytes) =====
typedef struct {
    uint32_t reserved;
    uint32_t id;
    float x;
    float y;
    float z;
    float vx;
    float vy;
    float vz;
} PersonData;

// ===== Parse Radar Frame =====
void parseRadarFrame(uint8_t *frameData, int len)
{
    if (!(frameData[0]==1 && frameData[1]==2 && frameData[2]==3 &&
          frameData[3]==4 && frameData[4]==5 && frameData[5]==6 &&
          frameData[6]==7 && frameData[7]==8))
        return;

    uint32_t packetLen;
    memcpy(&packetLen, &frameData[8], 4);

    uint32_t frameID;
    memcpy(&frameID, &frameData[12], 4);

    uint32_t trackLen;
    memcpy(&trackLen, &frameData[28], 4);

    if (packetLen <= 32 || trackLen < sizeof(PersonData)) {
        printf("[RADAR] Skip non-person frame\n");
        return;
    }

    if (packetLen > (uint32_t)len)
        packetLen = len;

    if (trackLen % sizeof(PersonData) != 0) {
        printf("[RADAR] Invalid trackLen\n");
        return;
    }

    uint32_t personCount = trackLen / sizeof(PersonData);

    if (32 + trackLen > packetLen) {
        printf("[RADAR] Track data exceeds packet\n");
        return;
    }

    printf("\n[RADAR] Frame=%lu PacketLen=%lu Persons=%lu\n",
           frameID, packetLen, personCount);

    for (int i = 0; i < personCount; i++) {

        int offset = 32 + i * sizeof(PersonData);
        if (offset + sizeof(PersonData) > len)
            break;

        PersonData p;
        memcpy(&p, frameData + offset, sizeof(PersonData));

        printf(" Person %d:", i + 1);
        printf("   ID: %lu ", p.id);
        printf("   X: %.2f ", p.x);
        printf("   Y: %.2f ", p.y);
        printf("   Z: %.2f ", p.z);
        printf("   Vx: %.2f ", p.vx);
        printf("   Vy: %.2f ", p.vy);
        printf("   Vz: %.2f\n", p.vz);
    }
    vTaskDelay(pdMS_TO_TICKS(50));
}

// ===== Send Command =====
void sendCmd(const char *cmd)
{
    if (!paired)
        return;

    uint8_t buf[200];
    buf[0] = 0x02;

    char temp[200];
    snprintf(temp, sizeof(temp), "%s\n", cmd);

    int l = strlen(temp);
    if (l > 190)
        l = 190;

    memcpy(buf + 1, temp, l);
    esp_now_send(senderMAC, buf, l + 1);

    printf("Sent: %s", temp);
}

// ===== Setup Radar Config =====
void runSetup()
{
    isStopped = false;

    printf("--- Running Setup Sequence ---\n");

    sendCmd("AT+STOP"); vTaskDelay(pdMS_TO_TICKS(500));
    sendCmd("AT+SENS=3"); vTaskDelay(pdMS_TO_TICKS(300));
    sendCmd("AT+RANGE=100"); vTaskDelay(pdMS_TO_TICKS(200));
    sendCmd("AT+TIME=200"); vTaskDelay(pdMS_TO_TICKS(200));
    sendCmd("AT+HEIGHT=267"); vTaskDelay(pdMS_TO_TICKS(200));
    sendCmd("AT+HEIGHTD=250"); vTaskDelay(pdMS_TO_TICKS(200));
    sendCmd("AT+XNegaD=-100"); vTaskDelay(pdMS_TO_TICKS(200));
    sendCmd("AT+XPosiD=100"); vTaskDelay(pdMS_TO_TICKS(200));
    sendCmd("AT+YNegaD=-100"); vTaskDelay(pdMS_TO_TICKS(200));
    sendCmd("AT+YPosiD=100"); vTaskDelay(pdMS_TO_TICKS(200));
    sendCmd("AT+DEBUG=3"); vTaskDelay(pdMS_TO_TICKS(200));
    sendCmd("AT+START"); vTaskDelay(pdMS_TO_TICKS(300));
    sendCmd("AT+STUDY"); vTaskDelay(pdMS_TO_TICKS(3000));
    sendCmd("AT+STOP"); vTaskDelay(pdMS_TO_TICKS(300));

    printf("--- Setup Done ---\n");
}

// ===== ESP-NOW Receive Callback =====
void OnDataRecv(const esp_now_recv_info_t *info,
                const uint8_t *data,
                int len)
{
    if (!paired) {
        memcpy(senderMAC, info->src_addr, 6);

        esp_now_peer_info_t peer = {0};
        memcpy(peer.peer_addr, senderMAC, 6);
        peer.channel = 0;
        peer.encrypt = false;

        if (esp_now_add_peer(&peer) == ESP_OK) {
            paired = true;
            printf("--- PAIRED WITH SENDER ---\n");
        }
    }

    if (len <= 4 || data[0] != 0x01)
        return;

    uint8_t seq = data[1];
    uint16_t totalLen = (data[2] << 8) | data[3];
    int chunkLen = len - 4;

    if (seq == 0)
        rxIndex = 0;

    if (rxIndex + chunkLen <= RX_BUF_SIZE) {
        memcpy(rxBuffer + rxIndex, data + 4, chunkLen);
        rxIndex += chunkLen;
    }

    if (rxIndex >= totalLen) {

        printf("\n[RAW FRAME HEX]\n");
        for (int i = 0; i < totalLen; i++) {
            if (rxBuffer[i] < 16) printf("0");
            printf("%X ", rxBuffer[i]);
        }
        printf("\n");

        bool isRadar = (rxBuffer[0]==1 && rxBuffer[1]==2 &&
                        rxBuffer[2]==3 && rxBuffer[3]==4 &&
                        rxBuffer[4]==5 && rxBuffer[5]==6 &&
                        rxBuffer[6]==7 && rxBuffer[7]==8);

        if (!isRadar) {
            printf("[MSG TEXT]\n");
            fwrite(rxBuffer, 1, totalLen, stdout);
            printf("\n");
        } else {
            if (!isStopped)
                parseRadarFrame(rxBuffer, totalLen);
        }

        rxIndex = 0;
    }
}

// ===== Console Task (giá»‘ng loop Arduino) =====
void console_task(void *pv)
{
    char input[200];

    while (1)
    {
        if (fgets(input, sizeof(input), stdin) != NULL)
        {
            input[strcspn(input, "\r\n")] = 0;

            if (strcmp(input, "SETUP") == 0)
                runSetup();

            else if (strcmp(input, "STOP") == 0) {
                isStopped = true;
                sendCmd("AT+STOP");
                vTaskDelay(pdMS_TO_TICKS(50));
                sendCmd("AT+STOP");
                vTaskDelay(pdMS_TO_TICKS(50));
                sendCmd("AT+STOP");
                printf(">> Sent STOP (Forced)\n");
            }

            else if (strcmp(input, "START") == 0) {
                isStopped = false;
                sendCmd("AT+START");
                printf(">> Sent START\n");
            }

            else if (strcmp(input, "RESET") == 0) {
                sendCmd("AT+RESET");
                printf(">> Sent RESET\n");
            }

            else if (strlen(input) > 0) {
                sendCmd(input);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ===== WIFI INIT =====
void wifi_init()
{
    esp_netif_init();
    esp_event_loop_create_default();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_start();
}

// ===== MAIN =====
void app_main()
{
    nvs_flash_init();
    wifi_init();

    if (esp_now_init() != ESP_OK) {
        printf("ESP-NOW init failed\n");
        return;
    }

    uint8_t mac[6];
    // Đọc địa chỉ MAC của giao diện Station (STA)
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    
    printf("\n==================================\n");
    printf("RECEIVER MAC ADDRESS: ");
    for (int i = 0; i < 6; i++) {
        printf("%02X%s", mac[i], (i == 5) ? "" : ":");
    }
    printf("\n==================================\n");

    esp_now_register_recv_cb(OnDataRecv);

    printf("Receiver Ready.\n");
    printf("Commands: SETUP, START, STOP, RESET\n");

    xTaskCreate(console_task, "console_task", 4096, NULL, 5, NULL);
}





/////////////////////////////////////////////////////////////////////

/**
// ==============================================================================
// FILE: radarreceiver.c
// DESCRIPTION: Production-Grade Multi-Radar Fusion & Predictive Tracking
// HARDWARE: ESP32-S3 + MinewSemi MS72SF1
// ==============================================================================

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_timer.h"
#include "esp_mac.h"
#include "nvs_flash.h"

// ===== CẤU HÌNH HỆ THỐNG (MICRO & MACRO) =====
#define OFFSET_CM             35.0f   // Bù lệch vật lý 2 radar nội bộ (Micro)
#define HYSTERESIS_ZONE       30.0f   // Vùng trễ +-30cm ranh giới X=0
#define MAX_FUSION_DIST_CM    80.0f   // Bán kính gộp ID tối đa (Macro)
#define MACRO_TIMEOUT_MS      500     // Xóa ID nếu mất tín hiệu toàn hệ thống trong 0.5s
#define MAX_TARGETS           10      // Tracking tối đa 10 người

// Tọa độ lắp đặt của Cục A (UNIT 0x0A) trong không gian phòng (VD: Mặc định là gốc tọa độ)
#define UNIT_A_OFFSET_X       0.0f
#define UNIT_A_OFFSET_Y       0.0f

// Trạng thái State Machine
typedef enum {
    STATE_TENTATIVE,    // Mới xuất hiện, đang chờ xác nhận
    STATE_CONFIRMED,    // Tracking ổn định
    STATE_TRANSFERRING  // Nằm trong ranh giới hoặc đang làm thủ tục Handoff
} TargetState;

// Cấu trúc Quản lý Mục tiêu Toàn cục
typedef struct {
    uint32_t active;
    uint32_t id;                // ID cấp phát toàn cục (Global ID)
    
    // Quyền sở hữu
    uint8_t  owner_unit;        // Cục quản lý (VD: 0x0A)
    uint8_t  owner_radar;       // Radar nội bộ quản lý (0x01: RIGHT, 0x02: LEFT)
    TargetState state;

    // Bộ đếm tách biệt chống nhiễu (Robust Tracking)
    uint8_t confirm_counter;
    uint8_t transfer_counter;

    // Tọa độ & Vận tốc
    float local_x;              // X tương đối với Cục quản lý
    float global_x;             // X tuyệt đối trong phòng
    float global_y;             // Y tuyệt đối trong phòng
    float vx, vy;               // Vận tốc (m/s) từ radar MS72SF1
    
    uint32_t last_update;       // Thời gian cập nhật cuối (ms)
} TrackedTarget;

TrackedTarget targets[MAX_TARGETS];
uint32_t next_global_id = 1;

// Hàm lấy thời gian Delta thực tế (ms)
static uint32_t get_millis() {
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

// ================= THUẬT TOÁN FUSION & PREDICTIVE TRACKING =================
void process_fusion_target(uint8_t unit_id, uint8_t radar_id, float x_raw, float y_raw, float vx, float vy) {
    uint32_t now = get_millis();
    int best_match = -1;
    float min_dist = MAX_FUSION_DIST_CM;

    // 1. Tính toán tọa độ Local (Bù offset vật lý trục X)
    float x_cal = (radar_id == 0x01) ? (x_raw * 100.0f + OFFSET_CM) : (x_raw * 100.0f - OFFSET_CM);
    float y_cm = y_raw * 100.0f;

    // 2. Tính toán tọa độ Global (Cộng thêm tọa độ lắp đặt của Cục)
    // Tạm mặc định dùng UNIT_A. Khi có Unit B, đổi if(unit_id == 0x0B)...
    float g_x = x_cal + UNIT_A_OFFSET_X;
    float g_y = y_cm + UNIT_A_OFFSET_Y;

    // 3. Predictive Matching (Tìm ID bằng Dead Reckoning)
    for (int i = 0; i < MAX_TARGETS; i++) {
        if (targets[i].active) {
            float dt = (now - targets[i].last_update) / 1000.0f; // Đổi ms sang giây
            
            // Vị trí dự đoán (cm) dựa trên vận tốc (m/s)
            float pred_x = targets[i].global_x + (targets[i].vx * dt * 100.0f);
            float pred_y = targets[i].global_y + (targets[i].vy * dt * 100.0f);

            // So sánh khoảng cách với Vị trí dự đoán
            float dx = pred_x - g_x;
            float dy = pred_y - g_y;
            float dist = sqrtf(dx*dx + dy*dy);

            if (dist < min_dist) {
                min_dist = dist;
                best_match = i;
            }
        }
    }

    // 4. Cập nhật và Xử lý State Machine
    if (best_match != -1) {
        TrackedTarget *t = &targets[best_match];
        t->local_x = x_cal; 
        t->global_x = g_x; 
        t->global_y = g_y;
        t->vx = vx; 
        t->vy = vy;
        t->last_update = now;

       
        // Lớp 1: Xác nhận mục tiêu mới
        if (t->state == STATE_TENTATIVE) {
            t->confirm_counter++;
            if (t->confirm_counter >= 2) t->state = STATE_CONFIRMED;
        }

        // Lớp 2: Ranh giới Hysteresis (Đóng băng Ownership)
        if (fabsf(t->local_x) <= HYSTERESIS_ZONE) {
            t->state = STATE_TRANSFERRING;
            t->transfer_counter = 0; 
            return; // Thoát sớm, không xử lý đổi chủ trong vùng này
        }

        // Lớp 3: Quyết định chuyển chủ (Dominance Rule)
        if (t->owner_unit != unit_id || t->owner_radar != radar_id) {
            // Chủ mới đang áp đảo
            t->transfer_counter++;
            if (t->transfer_counter >= 2) {
                t->owner_unit = unit_id;
                t->owner_radar = radar_id;
                t->state = STATE_CONFIRMED;
                t->transfer_counter = 0;
            }
        } else {
            // Vẫn chủ cũ
            t->transfer_counter = 0;
            if(t->state == STATE_TRANSFERRING) t->state = STATE_CONFIRMED;
        }
    } 
    // 5. Khởi tạo Mục tiêu mới
    else {
        for (int i = 0; i < MAX_TARGETS; i++) {
            if (!targets[i].active) {
                targets[i].active = 1;
                targets[i].id = next_global_id++;
                targets[i].owner_unit = unit_id;
                targets[i].owner_radar = radar_id;
                targets[i].state = STATE_TENTATIVE;
                targets[i].confirm_counter = 1;
                targets[i].transfer_counter = 0;
                targets[i].local_x = x_cal;
                targets[i].global_x = g_x;
                targets[i].global_y = g_y;
                targets[i].vx = vx; 
                targets[i].vy = vy;
                targets[i].last_update = now;
                break;
            }
        }
    }
}

// ================= DỌN DẸP TIMEOUT =================
void cleanup_targets() {
    uint32_t now = get_millis();
    for (int i = 0; i < MAX_TARGETS; i++) {
        if (targets[i].active && (now - targets[i].last_update > MACRO_TIMEOUT_MS)) {
            targets[i].active = 0; // Xóa ID do timeout
        }
    }
}

// ================= NHẬN DỮ LIỆU ESP-NOW =================
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    if (len <= 6 || data[0] != 0x01) return; // Bỏ qua gói tin rác
    
    uint8_t unit_id  = data[4]; 
    uint8_t radar_id = data[5]; 
    const uint8_t *payload = data + 6;
    
    uint32_t trackLen;
    memcpy(&trackLen, &payload[28], 4);
    int personCount = trackLen / 32; // Mỗi PersonData dài 32 bytes
    
    for (int i = 0; i < personCount; i++) {
        int offset = 32 + i * 32;
        float x_raw, y_raw, vx, vy;
        
        // Trích xuất tọa độ và vận tốc
        memcpy(&x_raw, &payload[offset + 8], 4);
        memcpy(&y_raw, &payload[offset + 12], 4);
        memcpy(&vx,    &payload[offset + 20], 4); 
        memcpy(&vy,    &payload[offset + 24], 4); 
        
        process_fusion_target(unit_id, radar_id, x_raw, y_raw, vx, vy);
    }
    
    cleanup_targets();
    
    // ================= HIỂN THỊ UI CONSOLE =================
    // printf("\033[H\033[J"); // Clear Console
    printf("=== PREDICTIVE MULTI-RADAR FUSION TRACKING ===\n");
    printf("Max_Fusion_Dist: %.0fcm | Timeout: %dms | Hysteresis: +-%.0fcm\n", 
           MAX_FUSION_DIST_CM, MACRO_TIMEOUT_MS, HYSTERESIS_ZONE);
    printf("--------------------------------------------------------------\n");
    printf("%-6s | %-5s | %-6s | %-8s | %-7s | %-7s | %-7s\n", 
           "ID", "CỤC", "RADAR", "STATE", "X_LOC", "X_GLOB", "Y_GLOB");
    printf("--------------------------------------------------------------\n");
           
    for (int i = 0; i < MAX_TARGETS; i++) {
        if (targets[i].active && targets[i].state != STATE_TENTATIVE) {
            char* state_str = (targets[i].state == STATE_CONFIRMED) ? "CONFIRM" : "TRANSFR";
            char* radar_str = (targets[i].owner_radar == 0x01) ? "RIGHT" : "LEFT ";
            
            printf("ID[%02lu] | 0x%02X  | %-6s | %-8s | %5.0fcm | %5.0fcm | %5.0fcm\n", 
                   targets[i].id, 
                   targets[i].owner_unit, 
                   radar_str, 
                   state_str,
                   targets[i].local_x, 
                   targets[i].global_x, 
                   targets[i].global_y);
        }
    }
}   


void app_main(void) {
    nvs_flash_init();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_start();

    if (esp_now_init() != ESP_OK) {
        printf("ESP-NOW init failed\n");
        return;
    }

    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    
    printf("\n==================================\n");
    printf("RECEIVER MAC ADDRESS: ");
    for (int i = 0; i < 6; i++) {
        printf("%02X%s", mac[i], (i == 5) ? "" : ":");
    }
    printf("\n==================================\n");

    esp_now_register_recv_cb(OnDataRecv);

    printf("Receiver Ready. Waiting for Dual-Radar Data...\n");

    // Giữ FreeRTOS hoạt động
    while(1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
*/





/*
 * radarreceiver.c — Dual Radar Receiver
 * =============================================================
 * Nhận dữ liệu từ 2 radar qua ESP-NOW:
 *   radar_id = 0x01  →  Radar RIGHT (nửa phải,  X ≥ 0)
 *   radar_id = 0x02  →  Radar LEFT  (nửa trái,  X < 0)
 *
 * ── Coordinate Calibration ──────────────────────────────────
 *   Radar RIGHT lắp lệch +RADAR_HALF_GAP_CM so với tâm cục
 *     → x_cal = x_raw + RADAR_HALF_GAP_CM
 *   Radar LEFT  lắp lệch -RADAR_HALF_GAP_CM so với tâm cục
 *     → x_cal = x_raw - RADAR_HALF_GAP_CM
 *   Điều chỉnh RADAR_HALF_GAP_CM cho phù hợp khoảng cách thực.
 *
 * ── Packet format nhận từ sender ────────────────────────────
 *   [0x01][seq][lenH][lenL][radar_id][radar_frame_payload…]
 *    byte0  1    2     3      4           5+
 *
 * ── Hysteresis Boundary (tránh double-count) ────────────────
 *   Khi người đứng gần ranh giới X=0 (|x_cal| < BOUNDARY_HYST_CM):
 *   Chỉ in dữ liệu từ radar đang "sở hữu" người đó (last_owner).
 * =============================================================
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_mac.h"

// ================= CONFIG =================
// Don vi: MET (radar tra ve met)
#define RADAR_HALF_GAP_M     0.35f  // 70cm / 2 = 0.35m
#define BOUNDARY_HYST_M      0.30f  // Vung hysteresis +-0.3m quanh X=0

// Radar ID
#define RADAR_ID_RIGHT       0x01
#define RADAR_ID_LEFT        0x02

// ================= GLOBAL =================
static uint8_t senderMAC[6];
static bool    paired    = false;
static bool    isStopped = false;

// Debug counters
static uint32_t dbg_right_frames = 0;
static uint32_t dbg_left_frames  = 0;

#define RX_BUF_SIZE 4096

/* Mỗi radar có buffer riêng để ghép packet multi-chunk */
static uint8_t rxBuf1[RX_BUF_SIZE];
static int     rxIdx1 = 0;
static int     rxTotal1 = 0;   // totalLen kỳ vọng

static uint8_t rxBuf2[RX_BUF_SIZE];
static int     rxIdx2 = 0;
static int     rxTotal2 = 0;



// ================= STRUCT =================
typedef struct {
    uint32_t reserved;
    uint32_t id;
    float    x, y, z;
    float    vx, vy, vz;
} PersonData;   // 32 bytes

// ================= COORDINATE CALIBRATION =================
static float calibrate_x(float x_raw, uint8_t radar_id)
{
    if (radar_id == RADAR_ID_RIGHT)
        return x_raw + RADAR_HALF_GAP_M;
    else
        return x_raw - RADAR_HALF_GAP_M;
}

// ================= TRACK MANAGER =================
//  State Machine: TENTATIVE -> CONFIRMED -> TRANSFERRING
//  + N-of-M Presence Filter (window=5, confirm>=3, absent<2)
//  + Track Continuity (2 frame lien tiep)
//  + Hard Timeout (500ms)

#define MAX_TRACKS          8
#define TM_WINDOW_SIZE      5
#define TM_CONFIRM_THRESH   3       // hits >= 3/5 -> CONFIRMED
#define TM_ABSENT_THRESH    2       // hits <  2/5 -> remove
#define TM_CONTINUITY_FRAMES 2      // 2 frame lien tiep moi confirm
#define TM_TRACK_TIMEOUT_MS  500    // Timeout cung (ms)
#define TM_STATIC_VEL_THRESH 0.05f  // Nguong van toc tinh (m/s)
#define TM_STATIC_MAX_FRAMES 10     // Static lien tuc N frame -> loai

typedef enum {
    TS_TENTATIVE,       // Moi phat hien, chua xac nhan
    TS_CONFIRMED        // Dang tracking on dinh
} TrackState;

typedef struct {
    uint32_t   id;              // Person ID tu radar
    uint8_t    owner_radar;     // RADAR_ID_RIGHT / LEFT
    TrackState state;
    float      x, y, z;
    float      vx, vy, vz;
    float      x_cal;           // X da hieu chinh
    uint64_t   last_seen_ms;
    int        consec_frames;   // So frame lien tiep thay
    // N-of-M sliding window
    uint8_t    window[TM_WINDOW_SIZE];
    int        window_head;
    int        hit_count;
    int        static_frames;   // Dem frame lien tiep van toc ~0
    bool       ever_moved;      // Da tung di chuyen? (tranh loai nguoi dung yen)
    bool       active;          // Slot dang dung?
} TrackedPerson;

static TrackedPerson s_tracks[MAX_TRACKS];
static int           s_track_count = 0;


// Tao track moi
static TrackedPerson* track_create(uint32_t id, uint8_t radar_id)
{
    if (s_track_count >= MAX_TRACKS) return NULL;
    TrackedPerson *t = &s_tracks[s_track_count++];
    memset(t, 0, sizeof(TrackedPerson));
    t->id = id;
    t->owner_radar = radar_id;
    t->state = TS_TENTATIVE;
    t->active = true;
    return t;
}

// Push hit/miss vao sliding window
static void track_push(TrackedPerson *t, uint8_t hit)
{
    t->hit_count -= t->window[t->window_head];
    t->window[t->window_head] = hit;
    t->hit_count += hit;
    t->window_head = (t->window_head + 1) % TM_WINDOW_SIZE;
}

// Core: goi moi frame, cap nhat toan bo tracks
static void track_manager_update(
    PersonData *persons, int count,
    uint8_t radar_id, uint64_t now_ms)
{
    // --- Buoc 1: Danh dau "hit" cho ID xuat hien ---
    for (int i = 0; i < count; i++) {
        PersonData *p = &persons[i];
        float x_cal = calibrate_x(p->x, radar_id);

        // Tim track hien co (theo ID, bat ky radar nao)
        TrackedPerson *t = NULL;
        for (int j = 0; j < s_track_count; j++) {
            if (s_tracks[j].active && s_tracks[j].id == p->id) {
                t = &s_tracks[j];
                break;
            }
        }
        if (!t) {
            // Chua co -> tao TENTATIVE, gan cho radar phat hien dau tien
            t = track_create(p->id, radar_id);
            if (!t) continue;
        }

        // Cap nhat data
        t->x = p->x;  t->y = p->y;  t->z = p->z;
        t->vx = p->vx; t->vy = p->vy; t->vz = p->vz;
        t->x_cal = x_cal;
        t->last_seen_ms = now_ms;
        t->consec_frames++;
        track_push(t, 1);

        // --- Velocity Filter: loai target tinh (phan xa tuong/ban ghe) ---
        float speed = (p->vx < 0 ? -p->vx : p->vx)
                    + (p->vy < 0 ? -p->vy : p->vy);
        if (speed < TM_STATIC_VEL_THRESH) {
            t->static_frames++;
            // Chi loai neu CHUA BAO GIO di chuyen (ghost/phan xa tinh)
            if (!t->ever_moved && t->static_frames >= TM_STATIC_MAX_FRAMES) {
                // Static qua lau -> ghost target, loai bo
                if (t->state >= TS_CONFIRMED) {
                    const char *zn = (t->owner_radar == RADAR_ID_RIGHT) ? "RIGHT" : "LEFT";
                    printf("[TRACK] ID=%lu STATIC -> removed [%s]\n",
                           (unsigned long)t->id, zn);
                }
                t->active = false;
                // compact: di chuyen slot cuoi vao day
                int idx = (int)(t - s_tracks);
                if (idx < s_track_count - 1)
                    s_tracks[idx] = s_tracks[s_track_count - 1];
                s_track_count--;
                continue; // skip state machine cho target nay
            }
        } else {
            t->static_frames = 0;
            t->ever_moved = true;  // Da di chuyen -> khong bao gio bi loai static
        }

        // --- Lop 2: Track Continuity + State Machine ---
        float abs_x = (x_cal < 0) ? -x_cal : x_cal;

        // Cap nhat ownership theo vi tri (don gian, khong can TRANSFERRING)
        if (abs_x >= BOUNDARY_HYST_M) {
            // Ngoai vung hysteresis -> owner = ben nao x_cal thuoc
            uint8_t pos_owner = (x_cal >= 0) ? RADAR_ID_RIGHT : RADAR_ID_LEFT;
            if (t->owner_radar != pos_owner) {
                const char *from = (t->owner_radar == RADAR_ID_RIGHT) ? "RIGHT" : "LEFT ";
                const char *to   = (pos_owner == RADAR_ID_RIGHT) ? "RIGHT" : "LEFT ";
                printf("  >> ID=%lu HANDOFF [%s] -> [%s]  cal=%+.2fm <<\n",
                       (unsigned long)t->id, from, to, x_cal);
                t->owner_radar = pos_owner;
            }
        }
        // Trong vung hysteresis (abs_x < BOUNDARY_HYST_M) -> giu owner cu, khong doi

        // Chuyen TENTATIVE -> CONFIRMED
        if (t->state == TS_TENTATIVE &&
            t->consec_frames >= TM_CONTINUITY_FRAMES &&
            t->hit_count >= TM_CONFIRM_THRESH) {
            t->state = TS_CONFIRMED;
            const char *z = (t->owner_radar == RADAR_ID_RIGHT) ? "RIGHT" : "LEFT ";
            printf("\n  >> [%s] ID=%lu CONFIRMED  X=%.2fm Y=%.2fm <<\n",
                   z, (unsigned long)t->id, t->x, t->y);
        }

        // In toa do CHI KHI radar nay la owner (tranh duplicate)
        if (radar_id == t->owner_radar) {
            const char *st = (t->state == TS_CONFIRMED) ? "OK  " : "TENT";
            const char *z = (t->owner_radar == RADAR_ID_RIGHT) ? "RIGHT" : "LEFT ";
            printf("  [%s|%s] ID=%-3lu  X=%+6.2f  cal=%+6.2f  Y=%+5.2f  Z=%5.2f"
                   "  Vx=%+5.2f Vy=%+5.2f  hits=%d/%d\n",
                   z, st, (unsigned long)t->id,
                   t->x, t->x_cal, t->y, t->z,
                   t->vx, t->vy, t->hit_count, TM_WINDOW_SIZE);
        }
    }

    // --- Buoc 2: Danh "miss" + timeout cho track khong thay ---
    for (int i = 0; i < s_track_count; i++) {
        TrackedPerson *t = &s_tracks[i];
        if (!t->active) continue;
        if (t->owner_radar != radar_id) continue;

        // Kiem tra co trong danh sach frame nay khong
        bool seen = false;
        for (int j = 0; j < count; j++) {
            if (persons[j].id == t->id) { seen = true; break; }
        }

        if (!seen) {
            track_push(t, 0);
            t->consec_frames = 0;

            // Timeout cung
            bool timed_out = (now_ms - t->last_seen_ms) > TM_TRACK_TIMEOUT_MS;

            if (timed_out || t->hit_count < TM_ABSENT_THRESH) {
                if (t->state >= TS_CONFIRMED) {
                    const char *z = (t->owner_radar == RADAR_ID_RIGHT) ? "RIGHT" : "LEFT ";
                    printf("\n  << [%s] ID=%lu LEFT zone (hits=%d/%d, timeout=%s) >>\n",
                           z, (unsigned long)t->id,
                           t->hit_count, TM_WINDOW_SIZE,
                           timed_out ? "YES" : "NO");
                }
                // Xoa track: swap voi phan tu cuoi
                t->active = false;
                if (i < s_track_count - 1)
                    s_tracks[i] = s_tracks[s_track_count - 1];
                s_track_count--;
                i--; // Re-check slot nay
            }
        }
    }
}

// ================= PARSE RADAR FRAME =================
static void parseRadarFrame(uint8_t *frame, int len, uint8_t radar_id)
{
    if (!(frame[0]==1 && frame[1]==2 && frame[2]==3 && frame[3]==4 &&
          frame[4]==5 && frame[5]==6 && frame[6]==7 && frame[7]==8))
        return;

    uint32_t packetLen, frameID, trackLen;
    memcpy(&packetLen, &frame[8],  4);
    memcpy(&frameID,   &frame[12], 4);
    memcpy(&trackLen,  &frame[28], 4);

    if ((uint32_t)len < packetLen) packetLen = len;
    uint64_t now_ms = esp_timer_get_time() / 1000;
    const char *zone = (radar_id == RADAR_ID_RIGHT) ? "RIGHT" : "LEFT ";

    // Frame khong co nguoi -> van goi track_manager de danh miss
    if (packetLen <= 32 || trackLen < sizeof(PersonData)) {
        printf("--- [%s] Frame #%-5lu | Persons: 0 ---\n", zone, frameID);
        track_manager_update(NULL, 0, radar_id, now_ms);
        return;
    }

    if (trackLen % sizeof(PersonData) != 0) return;
    if (32 + trackLen > packetLen) return;

    uint32_t count = trackLen / sizeof(PersonData);
    printf("--- [%s] Frame #%-5lu | Persons: %lu ---\n", zone, frameID, count);

    // Thu thap PersonData
    PersonData persons[MAX_TRACKS];
    int pcount = 0;
    for (int i = 0; i < (int)count && pcount < MAX_TRACKS; i++) {
        int off = 32 + i * (int)sizeof(PersonData);
        if (off + (int)sizeof(PersonData) > len) break;
        memcpy(&persons[pcount], frame + off, sizeof(PersonData));
        pcount++;
    }

    // Dua vao Track Manager
    track_manager_update(persons, pcount, radar_id, now_ms);
}

// ================= REASSEMBLE & PARSE =================
/*
 * Gọi khi nhận 1 chunk ESP-NOW cho radar_id.
 * Ghép multi-chunk, khi đủ frame → parseRadarFrame.
 */
static void handle_chunk(const uint8_t *data, int len, uint8_t radar_id)
{
    // packet: [0x01][seq][lenH][lenL][radar_id][payload…]
    // Hàm này nhận data từ byte[0] (đã bao gồm header ESP-NOW).
    // Gọi từ OnDataRecv sau khi xác định radar_id ở byte[4].

    if (len < 5 || data[0] != 0x01) return;

    uint8_t seq      = data[1];
    uint16_t total   = (data[2] << 8) | data[3];
    // data[4] = radar_id (đã tách trước khi gọi)
    const uint8_t *payload  = data + 5;
    int            chunkLen = len - 5;

    uint8_t *rxBuf;
    int     *rxIdx;
    int     *rxTotal;

    if (radar_id == RADAR_ID_RIGHT) {
        rxBuf = rxBuf1; rxIdx = &rxIdx1; rxTotal = &rxTotal1;
    } else {
        rxBuf = rxBuf2; rxIdx = &rxIdx2; rxTotal = &rxTotal2;
    }

    if (seq == 0) {
        *rxIdx   = 0;
        *rxTotal = total;
    }

    if (*rxIdx + chunkLen <= RX_BUF_SIZE) {
        memcpy(rxBuf + *rxIdx, payload, chunkLen);
        *rxIdx += chunkLen;
    }

    if (*rxIdx >= *rxTotal && *rxTotal > 0) {
        if (!isStopped) {
            if (radar_id == RADAR_ID_RIGHT) dbg_right_frames++;
            else                            dbg_left_frames++;
            parseRadarFrame(rxBuf, *rxTotal, radar_id);
        }
        *rxIdx   = 0;
        *rxTotal = 0;
    }
}

// ================= SEND COMMAND =================
static void sendCmd(const char *cmd)
{
    if (!paired) return;

    uint8_t buf[200];
    buf[0] = 0x02;

    char temp[200];
    snprintf(temp, sizeof(temp), "%s\n", cmd);
    int l = strlen(temp);
    if (l > 190) l = 190;
    memcpy(buf + 1, temp, l);
    esp_now_send(senderMAC, buf, l + 1);
    printf("Sent: %s", temp);
}

// ================= SETUP SEQUENCE =================
static void runSetup(void)
{
    isStopped = false;
    printf("--- Running Setup Sequence ---\n");
    printf("(Sender sẽ cấu hình Spatial Partition + Prime Stagger)\n");

    // Chỉ gửi STOP → sender tự cấu hình AT khi restart
    sendCmd("AT+STOP"); vTaskDelay(pdMS_TO_TICKS(500));
    sendCmd("AT+SENS=8"); vTaskDelay(pdMS_TO_TICKS(300));
    sendCmd("AT+YNEGAD=-150"); vTaskDelay(pdMS_TO_TICKS(200));
    sendCmd("AT+YPOSID=150"); vTaskDelay(pdMS_TO_TICKS(200));
    sendCmd("AT+START"); vTaskDelay(pdMS_TO_TICKS(300));

    printf("--- Setup Done ---\n");
}

// ================= ESP-NOW RECV =================
void OnDataRecv(const esp_now_recv_info_t *info,
                const uint8_t *data,
                int len)
{
    // ── Pairing ──
    if (!paired) {
        memcpy(senderMAC, info->src_addr, 6);
        esp_now_peer_info_t peer = {0};
        memcpy(peer.peer_addr, senderMAC, 6);
        peer.channel = 0;
        peer.encrypt = false;
        if (esp_now_add_peer(&peer) == ESP_OK) {
            paired = true;
            printf("--- PAIRED WITH SENDER ---\n");
        }
    }

    if (len < 5 || data[0] != 0x01) return;

    uint8_t radar_id = data[4];

    // ── Ping packet (len=5, no payload) ──
    if (len == 5) {
        const char *zone = (radar_id == RADAR_ID_RIGHT) ? "RIGHT" : "LEFT";
        printf("[PING] Radar %s alive\n", zone);
        return;
    }

    // ── Data: ghép và parse ──
    handle_chunk(data, len, radar_id);
}

// ================= CONSOLE TASK =================
static void console_task(void *pv)
{
    char input[200];
    printf("Commands: SETUP | START | STOP | RESET | <AT command>\n");
    printf("Use 'R1:<cmd>' or 'R2:<cmd>' to target a specific radar (via sender forward)\n\n");

    while (1) {
        if (fgets(input, sizeof(input), stdin)) {
            input[strcspn(input, "\r\n")] = 0;

            if (strcmp(input, "SETUP") == 0)
                runSetup();

            else if (strcmp(input, "STOP") == 0) {
                isStopped = true;
                sendCmd("AT+STOP");
                vTaskDelay(pdMS_TO_TICKS(50));
                sendCmd("AT+STOP");
                printf(">> Sent STOP (both radars)\n");
            }

            else if (strcmp(input, "START") == 0) {
                isStopped = false;
                sendCmd("AT+START");
                printf(">> Sent START (both radars)\n");
            }

            else if (strcmp(input, "RESET") == 0) {
                sendCmd("AT+RESET");
                printf(">> Sent RESET\n");
            }

            else if (strcmp(input, "STATUS") == 0) {
                printf("\n=== Dual Radar Status ===\n");
                printf("  Paired: %s\n", paired ? "YES" : "NO");
                printf("  Stopped: %s\n", isStopped ? "YES" : "NO");
                printf("  RIGHT rxIdx=%d total=%d  frames=%lu\n", rxIdx1, rxTotal1, (unsigned long)dbg_right_frames);
                printf("  LEFT  rxIdx=%d total=%d  frames=%lu\n", rxIdx2, rxTotal2, (unsigned long)dbg_left_frames);
                printf("  Tracks: %d\n", s_track_count);
                printf("  Calibration: +/-%.2f m\n", RADAR_HALF_GAP_M);
                printf("  Hysteresis:  +/-%.2f m\n", BOUNDARY_HYST_M);
                printf("=========================\n");
            }

            else if (strlen(input) > 0)
                sendCmd(input);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
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

// ================= MAIN =================
void app_main(void)
{
    // Init track manager
    s_track_count = 0;

    nvs_flash_init();
    wifi_init();

    if (esp_now_init() != ESP_OK) {
        printf("ESP-NOW init failed\n");
        return;
    }

    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    printf("\n==========================================\n");
    printf("  DUAL-RADAR RECEIVER\n");
    printf("  MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    printf("  Radar RIGHT (id=0x01): X >= 0  +%.2fm offset\n", RADAR_HALF_GAP_M);
    printf("  Radar LEFT  (id=0x02): X < 0  -%.2fm offset\n", RADAR_HALF_GAP_M);
    printf("  Hysteresis zone: |X| < %.2f m\n", BOUNDARY_HYST_M);
    printf("==========================================\n\n");

    esp_now_register_recv_cb(OnDataRecv);

    xTaskCreate(console_task, "console_task", 4096, NULL, 5, NULL);
}



