

/*
 * radarreceiver.c — Dual Radar Receiver PRODUCTION v3.1
 * Hoàn thiện: Chỉ in khi có thay đổi + tọa độ đầy đủ + sửa hết lỗi compile
 */

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
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"

static const char *TAG = "RADAR_RX";

// MAC helper
#define MACSTR "%02X:%02X:%02X:%02X:%02X:%02X"
#define MAC2STR(a) (a)[0],(a)[1],(a)[2],(a)[3],(a)[4],(a)[5]

// ================= CONFIG =================
#define RADAR_HALF_GAP_M     0.35f
#define BOUNDARY_HYST_M      0.30f

#define RADAR_ID_RIGHT       0x01
#define RADAR_ID_LEFT        0x02

#define RX_BUF_SIZE          4096
#define PARTIAL_TIMEOUT_MS   400

// ================= CRC16 =================
static uint16_t crc16(const uint8_t *data, size_t len) {
    uint16_t crc = 0;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; j++) {
            crc = (crc & 0x8000) ? ((crc << 1) ^ 0x1021) : (crc << 1);
        }
    }
    return crc;
}

// ================= GLOBAL =================
static uint8_t senderMAC[6];
static bool paired = false;
static bool isStopped = false;

static uint8_t rxBuf[2][RX_BUF_SIZE];
static int     rxIdx[2] = {0};
static int     rxTotal[2] = {0};
static uint64_t last_activity[2] = {0};

// ================= TRACK MANAGER =================
typedef struct {
    uint32_t id;
    uint8_t  owner_radar;
    float    x_cal, y;
    float    vx, vy;
    uint64_t last_seen;
    bool     active;
} TrackedPerson;

#define MAX_TRACKS 8
static TrackedPerson tracks[MAX_TRACKS];
static int track_count = 0;

static uint64_t last_print_ms = 0;
static bool     changed = false;        // dirty flag

static float calibrate_x(float x_raw, uint8_t radar_id) {
    return (radar_id == RADAR_ID_RIGHT) ? x_raw + RADAR_HALF_GAP_M : x_raw - RADAR_HALF_GAP_M;
}

static void track_reset(void) {
    memset(tracks, 0, sizeof(tracks));
    track_count = 0;
    changed = true;
}

static TrackedPerson* find_or_create_track(uint32_t id, uint8_t radar_id) {
    for (int i = 0; i < track_count; i++) {
        if (tracks[i].active && tracks[i].id == id) {
            return &tracks[i];
        }
    }
    if (track_count < MAX_TRACKS) {
        TrackedPerson *t = &tracks[track_count++];
        t->id = id;
        t->owner_radar = radar_id;
        t->active = true;
        changed = true;
        return t;
    }
    return NULL;
}

static void update_track(TrackedPerson *t, float x_cal, float y, float vx, float vy) {
    if (fabsf(t->x_cal - x_cal) > 0.05f || fabsf(t->y - y) > 0.05f ||
        fabsf(t->vx - vx) > 0.02f || fabsf(t->vy - vy) > 0.02f) {
        changed = true;
    }
    t->x_cal = x_cal;
    t->y = y;
    t->vx = vx;
    t->vy = vy;
    t->last_seen = esp_timer_get_time() / 1000;
}

static void cleanup_old_tracks(void) {
    uint64_t now = esp_timer_get_time() / 1000;
    for (int i = 0; i < track_count; i++) {
        if (tracks[i].active && (now - tracks[i].last_seen > 2000)) {
            tracks[i].active = false;
            changed = true;
            tracks[i] = tracks[track_count - 1];
            track_count--;
            i--;
        }
    }
}

// ================= PARSE =================
static void parseRadarFrame(uint8_t *frame, int len, uint8_t radar_id) {
    if (len < 32) return;

    uint32_t trackLen;
    memcpy(&trackLen, &frame[28], 4);
    uint32_t count = trackLen / 32;

    ESP_LOGD(TAG, "[%s] Frame | %lu persons", 
             (radar_id == RADAR_ID_RIGHT ? "RIGHT" : "LEFT"), count);

    for (uint32_t i = 0; i < count; i++) {
        int off = 32 + i * 32;
        uint32_t id; float x_raw, y_raw, vx, vy;

        memcpy(&id,    &frame[off + 0],  4);
        memcpy(&x_raw, &frame[off + 8],  4);
        memcpy(&y_raw, &frame[off + 12], 4);
        memcpy(&vx,    &frame[off + 20], 4);
        memcpy(&vy,    &frame[off + 24], 4);

        float x_cal = calibrate_x(x_raw, radar_id);

        TrackedPerson *t = find_or_create_track(id, radar_id);
        if (t) update_track(t, x_cal, y_raw, vx, vy);
    }

    cleanup_old_tracks();
    if (track_count == 0) changed = true;
}

// ================= HANDLE CHUNK (giữ nguyên) =================
static void handle_chunk(const uint8_t *data, int len, uint8_t radar_id) {
    if (len < 5 || data[0] != 0x01) return;

    int idx = (radar_id == RADAR_ID_RIGHT) ? 0 : 1;

    uint64_t now = esp_timer_get_time() / 1000;
    if (rxIdx[idx] > 0 && now - last_activity[idx] > PARTIAL_TIMEOUT_MS) {
        ESP_LOGW(TAG, "Partial frame timeout → clear buffer %s", (idx==0?"RIGHT":"LEFT"));
        rxIdx[idx] = rxTotal[idx] = 0;
    }

    uint8_t seq = data[1];
    uint16_t total = (data[2] << 8) | data[3];
    const uint8_t *payload = data + 5;
    int chunkLen = len - 5;

    if (seq == 0) {
        rxIdx[idx] = 0;
        rxTotal[idx] = total;
    }

    if (rxIdx[idx] + chunkLen <= RX_BUF_SIZE) {
        memcpy(rxBuf[idx] + rxIdx[idx], payload, chunkLen);
        rxIdx[idx] += chunkLen;
        last_activity[idx] = now;
    }

    if (rxIdx[idx] >= rxTotal[idx] && rxTotal[idx] > 0) {
        if (rxTotal[idx] >= 2) {
            uint16_t recv_crc;
            memcpy(&recv_crc, rxBuf[idx] + rxTotal[idx] - 2, 2);
            uint16_t calc_crc = crc16(rxBuf[idx], rxTotal[idx] - 2);

            if (calc_crc == recv_crc) {
                if (!isStopped) parseRadarFrame(rxBuf[idx], rxTotal[idx] - 2, radar_id);
            } else {
                ESP_LOGE(TAG, "CRC ERROR %s! calc=%04X recv=%04X", 
                         (idx==0?"RIGHT":"LEFT"), calc_crc, recv_crc);
            }
        }
        rxIdx[idx] = rxTotal[idx] = 0;
    }
}

// ================= PRINT ONLY WHEN CHANGED =================
static void print_table(void) {
    uint64_t now = esp_timer_get_time() / 1000;
    if (!changed && (now - last_print_ms < 300)) return;

    printf("\033[H\033[J");
    printf("=== DUAL RADAR TRACKING (change only) ===\n");
    printf("Tracks: %d\n", track_count);
    printf("--------------------------------------------------\n");
    printf("ID     Radar   X(m)    Y(m)    Vx     Vy\n");
    printf("--------------------------------------------------\n");

    if (track_count == 0) {
        printf("          --- No persons detected ---\n");
    } else {
        for (int i = 0; i < track_count; i++) {
            if (!tracks[i].active) continue;
            printf("%-6lu %-6s %7.2f %7.2f %6.2f %6.2f\n",
                   tracks[i].id,
                   tracks[i].owner_radar == RADAR_ID_RIGHT ? "RIGHT" : "LEFT",
                   tracks[i].x_cal, tracks[i].y,
                   tracks[i].vx, tracks[i].vy);
        }
    }
    printf("--------------------------------------------------\n");

    last_print_ms = now;
    changed = false;
}

// ================= ON DATA RECV =================
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    if (!paired) {
        memcpy(senderMAC, info->src_addr, 6);
        esp_now_peer_info_t peer = {0};
        memcpy(peer.peer_addr, senderMAC, 6);
        esp_now_add_peer(&peer);
        paired = true;
        ESP_LOGI(TAG, "PAIRED " MACSTR, MAC2STR(senderMAC));
    }

    if (len < 5 || data[0] != 0x01) return;
    handle_chunk(data, len, data[4]);

    print_table();
}

// ================= CONSOLE TASK =================
static void console_task(void *pv) {
    char input[200];
    ESP_LOGI(TAG, "Commands: SETUP | START | STOP | RESET | STATUS | R1:AT+xxx | R2:AT+xxx");

    while (1) {
        if (fgets(input, sizeof(input), stdin)) {
            input[strcspn(input, "\r\n")] = 0;
            if (strlen(input) == 0) continue;

            if (strncmp(input, "R1:", 3) == 0 || strncmp(input, "R2:", 3) == 0) {
                if (!paired) {
                    printf("Not paired yet!\n");
                    continue;
                }
                uint8_t buf[200];
                buf[0] = 0x02;
                int l = snprintf((char*)buf + 1, 198, "%s\n", input);
                esp_now_send(senderMAC, buf, l + 1);
            } else {
                if (!paired) {
                    printf("Not paired yet!\n");
                    continue;
                }
                uint8_t buf[200];
                buf[0] = 0x02;
                int l = snprintf((char*)buf + 1, 198, "%s\n", input);
                esp_now_send(senderMAC, buf, l + 1);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ================= MAIN =================
void app_main(void) {
    nvs_flash_init();
    esp_netif_init();
    esp_event_loop_create_default();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_start();

    esp_now_init();
    esp_now_register_recv_cb(OnDataRecv);

    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    printf("\n=== DUAL-RADAR RECEIVER v3.1 (Change-only + Full Coord) ===\n");
    printf("MAC: " MACSTR "\nOffset: ±%.0f cm\n\n", MAC2STR(mac), RADAR_HALF_GAP_M*100);

    track_reset();

    xTaskCreate(console_task, "console", 4096, NULL, 5, NULL);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));   // chỉ force nhẹ mỗi giây
        // print_table();                  // không cần nữa
    }
}