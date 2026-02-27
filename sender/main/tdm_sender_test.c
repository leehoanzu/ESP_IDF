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

// Radar RIGHT (0x01)
#define RADAR1_UART      UART_NUM_2
#define RADAR1_RX_PIN    16
#define RADAR1_TX_PIN    17

// Radar LEFT (0x02)
#define RADAR2_UART      UART_NUM_1
#define RADAR2_RX_PIN    41
#define RADAR2_TX_PIN    42

#define RADAR_ID_RIGHT   0x01
#define RADAR_ID_LEFT    0x02

#define RADAR_BUF_SIZE   2048
static uint8_t radarBuf1[RADAR_BUF_SIZE];
static int radarIndex1 = 0;

static uint8_t radarBuf2[RADAR_BUF_SIZE];
static int radarIndex2 = 0;

static uint64_t lastPing = 0;

// ================= ESP-NOW RECV =================
void OnDataRecv(const esp_now_recv_info_t *info,
                const uint8_t *data,
                int len)
{
    if (len > 1 && data[0] == 0x02)
    {
        // Forward to both radars
        uart_write_bytes(RADAR1_UART, (const char *)(data + 1), len - 1);
        uart_write_bytes(RADAR2_UART, (const char *)(data + 1), len - 1);

        printf("\n>> Cmd: ");
        fwrite(data + 1, 1, len - 1, stdout);
        printf("\n");
    }
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

// ================= SEND FRAME =================
// Hàm này mượn y xì của bản 1 radar nhưng thêm byte radar_id vào vị trí số 4 và tính CRC
void sendRadarFrame(uint8_t *data, int len, uint8_t radar_id)
{
    uint16_t crc = crc16(data, len);
    uint16_t total_len = len + 2; // +2 byte CRC
    int offset = 0;
    uint8_t pkt[250];
    uint8_t seq = 0;

    while (offset < len)
    {
        int chunk = len - offset;
        if (chunk > 230)
            chunk = 230;
            
        bool is_last = (offset + chunk == len);

        pkt[0] = 0x01;
        pkt[1] = seq++;
        pkt[2] = (total_len >> 8) & 0xFF;
        pkt[3] = total_len & 0xFF;
        pkt[4] = radar_id; // Đánh dấu đây là radar trái hay phải

        memcpy(pkt + 5, data + offset, chunk);
        
        int send_len = 5 + chunk;
        if (is_last) {
            memcpy(pkt + 5 + chunk, &crc, 2);
            send_len += 2;
        }
        
        esp_now_send(receiverMAC, pkt, send_len);

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
static void uart_init_one(uart_port_t port, int tx, int rx) {
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    uart_driver_install(port, 4096, 0, 0, NULL, 0);
    uart_param_config(port, &uart_config);
    uart_set_pin(port, tx, rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

void uart_init(void)
{
    uart_init_one(RADAR1_UART, RADAR1_TX_PIN, RADAR1_RX_PIN);
    uart_init_one(RADAR2_UART, RADAR2_TX_PIN, RADAR2_RX_PIN);
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

// ================= CORE PARSER (TỪ BẢN 1 RADAR) =================
// Tách ruột vòng for ở bản 1 radar ra thành 1 hàm process riêng lẻ để dùng lại cho 2 task
void parse_radar_byte(uint8_t b, uint8_t *radarBuf, int *radarIndex, uint8_t radar_id) {
    radarBuf[(*radarIndex)++] = b;

    if (*radarIndex >= 8)
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
                    *radarIndex - 1);
            (*radarIndex)--;
            return; // continue equivalent
        }

        if (*radarIndex >= 12)
        {
            uint32_t packetLen;
            memcpy(&packetLen,
                   &radarBuf[8],
                   4);

            uint32_t totalFrameSize = packetLen;

            if (totalFrameSize > RADAR_BUF_SIZE)
            {
                *radarIndex = 0;
                return; // continue equivalent
            }

            if (*radarIndex >= totalFrameSize)
            {
                const char *rname = (radar_id == RADAR_ID_RIGHT) ? "RIGHT" : "LEFT";
                printf("\n[%s Frame Sent], Size = %lu\n", rname, totalFrameSize);

                sendRadarFrame(radarBuf, totalFrameSize, radar_id);

                memmove(radarBuf,
                        radarBuf + totalFrameSize,
                        *radarIndex - totalFrameSize);

                *radarIndex -= totalFrameSize;
            }
        }
    }

    if (*radarIndex >= RADAR_BUF_SIZE)
        *radarIndex = 0;
}

// ================= LOOP TASK RIGHT =================
void radar1_task(void *pv)
{
    uint8_t tmp[256];
    int idle_count = 0;
    while (1)
    {
        uint64_t now = esp_timer_get_time() / 1000;
        if (now - lastPing > 2000)
        {
            lastPing = now;
            uint8_t pingPkt[] = { 0x01, 0x00, 0x00, 0x00, RADAR_ID_RIGHT };
            esp_now_send(receiverMAC, pingPkt, 5);
        }

        int len = uart_read_bytes(RADAR1_UART, tmp, sizeof(tmp), pdMS_TO_TICKS(10));
        if (len > 0) {
            idle_count = 0;
            for (int i = 0; i < len; i++)
            {
                parse_radar_byte(tmp[i], radarBuf1, &radarIndex1, RADAR_ID_RIGHT);
            }
        } else {
            idle_count++;
            if (idle_count > 10 && radarIndex1 > 0) {
                // Rỗng buffer nếu không nhận được data quá ~100ms (Radar bị STOP)
                radarIndex1 = 0;
            }
        }
    }
}

// ================= LOOP TASK LEFT =================
void radar2_task(void *pv)
{
    uint8_t tmp[256];
    int idle_count = 0;
    while (1)
    {
        int len = uart_read_bytes(RADAR2_UART, tmp, sizeof(tmp), pdMS_TO_TICKS(10));
        if (len > 0) {
            idle_count = 0;
            for (int i = 0; i < len; i++)
            {
                parse_radar_byte(tmp[i], radarBuf2, &radarIndex2, RADAR_ID_LEFT);
            }
        } else {
            idle_count++;
            if (idle_count > 10 && radarIndex2 > 0) {
                // Rỗng buffer nếu không nhận được data quá ~100ms (Radar bị STOP)
                radarIndex2 = 0;
            }
        }
    }
}

// ================= CẤU HÌNH AT CƠ BẢN =================
static void radar_at_config(void) {
    // Chỉ stop, study, start (Logic tối thiểu nhất để đảm bảo không lỗi)
    printf("=== RADAR BASIC AT CONFIG ===\n");
    
    uart_write_bytes(RADAR1_UART, "AT+STOP\n", 8);
    uart_write_bytes(RADAR2_UART, "AT+STOP\n", 8);
    vTaskDelay(pdMS_TO_TICKS(500));
    
    uart_write_bytes(RADAR1_UART, "AT+STUDY\n", 9);
    uart_write_bytes(RADAR2_UART, "AT+STUDY\n", 9);
    vTaskDelay(pdMS_TO_TICKS(8000));
    
    // TDM sẽ lo việc START sau
}

// ================= TDM TASK =================
void tdm_task(void *pv)
{
    printf("--- TDM Task Started: Interleaving every 1 sec ---\n");
    while (1)
    {
        // 1. Phải chạy, Trái dừng
        uart_write_bytes(RADAR1_UART, "AT+START\n", 9);
        uart_write_bytes(RADAR2_UART, "AT+STOP\n", 8);
        vTaskDelay(pdMS_TO_TICKS(1000));

        // 2. Trái chạy, Phải dừng
        uart_write_bytes(RADAR1_UART, "AT+STOP\n", 8);
        uart_write_bytes(RADAR2_UART, "AT+START\n", 9);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// ================= MAIN =================
void app_main(void)
{
    nvs_flash_init();
    wifi_init();
    uart_init();
    espnow_init();

    printf("Sender Ready (Dual Radar Base).\n");
    
    radar_at_config(); // Tạm tắt nếu bạn đã set cứng qua phần mềm hãng

    xTaskCreate(radar1_task, "radar1", 4096, NULL, 5, NULL);
    xTaskCreate(radar2_task, "radar2", 4096, NULL, 5, NULL);
    
    // Tạo TDM task để bật/tắt luân phiên
    xTaskCreate(tdm_task, "tdm_task", 2048, NULL, 5, NULL);
}