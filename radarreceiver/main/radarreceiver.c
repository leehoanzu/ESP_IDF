

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
// Đặt 2 cục radar cách nhau bao xa? Ví dụ cách nhau 0.7m -> HALF_GAP là 0.35m
#define RADAR_HALF_GAP_M           0.35f
// Khoảng cách này được dùng để tạo "Phần lấn" ảo bằng phần mềm tại Receiver
#define VIRTUAL_OVERLAP_M          0.30f
#define BOUNDARY_HYST_M            0.30f

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
typedef enum {
    TRACK_TENTATIVE = 0,
    TRACK_CONFIRMED,
    TRACK_TRANSFERRING
} TrackState;

typedef struct {
    uint32_t id;             // Global ID cấp phát bởi ESP32
    uint32_t internal_id;    // ID nội bộ từ giao thức Radar (chỉ 1 byte)
    uint8_t  owner_radar;
    float    x_cal, y, z;
    float    vx, vy, vz;
    uint64_t last_seen;
    bool     active;
    
    // Thuật toán Boundary
    TrackState state;
    int        hit_count;
} TrackedPerson;

#define MAX_TRACKS 8
static TrackedPerson tracks[MAX_TRACKS];
static int track_count = 0;

static uint64_t last_print_ms = 0;
static bool     changed = false;        // dirty flag

static float calibrate_x(float x_raw, uint8_t radar_id) {
    // VIRTUAL OVERLAP SOFTWARE:
    // Vì Cục Radar bị giới hạn phần cứng không cho quét sang nửa Âm (hoặc Dương),
    // Nên ở đây ta dịch chuyển Tọa độ vật lý của Radar lấn sang vùng bên kia 1 đoạn VIRTUAL_OVERLAP_M
    // R1 (Right): X vốn > 0. Dịch toàn bộ trục X sang trái thêm.
    // R2 (Left) : X vốn < 0. Dịch toàn bộ trục X sang phải thêm.
    
    if (radar_id == RADAR_ID_RIGHT) {
        return x_raw + (RADAR_HALF_GAP_M - VIRTUAL_OVERLAP_M);
    } else {
        return x_raw - (RADAR_HALF_GAP_M - VIRTUAL_OVERLAP_M);
    }
}

static void track_reset(void) {
    memset(tracks, 0, sizeof(tracks));
    track_count = 0;
    changed = true;
}

static uint32_t next_global_id = 1;

static TrackedPerson* match_or_create_track(uint32_t internal_id, uint8_t radar_id, float x_cal, float y, float z) {
    // 1. Tìm chính xác theo ID nội bộ và Radar
    for (int i = 0; i < track_count; i++) {
        if (tracks[i].active && tracks[i].internal_id == internal_id && tracks[i].owner_radar == radar_id) {
            return &tracks[i];
        }
    }

    // 2. Lớp 3 - Confidence Weighting & Continuity Matching
    // Nếu không tìm thấy, xem có Track nào cũ đang Transferring hoặc ở gần không (Khoảng cách < 0.4m) 
    // để gộp chung thay vì tạo bóng ma (Ghosting)
    for (int i = 0; i < track_count; i++) {
        // Chỉ ghép nối với Radar ĐỐI DIỆN
        if (tracks[i].active && tracks[i].owner_radar != radar_id) {
            float dx = tracks[i].x_cal - x_cal;
            float dy = tracks[i].y - y;
            float dist = sqrtf(dx*dx + dy*dy);
            
            if (dist < 0.4f) { // Cùng 1 người nhưng radar kia mới phát hiện (siết ngưỡng xuống 40cm an toàn)
                // Nếu track cũ đang Confirm và mình ở xa hơn, không làm gì cả (Chống nhiễu ghép)
                if (tracks[i].state == TRACK_CONFIRMED) {
                    // Cùng 1 người nhưng radar kia đang giữ, vào vùng trễ -> Chuyển state
                    if (fabsf(tracks[i].x_cal) < BOUNDARY_HYST_M) {
                        tracks[i].state = TRACK_TRANSFERRING;
                    }
                    return &tracks[i]; // Trả về track cũ để update
                }
                
                // Thuộc quyền radar mới
                tracks[i].owner_radar = radar_id;
                tracks[i].internal_id = internal_id;
                return &tracks[i];
            }
        }
    }

    // 3. Tạo mới nếu không match được ai
    if (track_count < MAX_TRACKS) {
        TrackedPerson *t = &tracks[track_count++];
        t->id = next_global_id++;
        t->internal_id = internal_id;
        t->owner_radar = radar_id;
        t->active = true;
        t->state = TRACK_TENTATIVE;
        t->hit_count = 1;
        changed = true;
        return t;
    }
    return NULL;
}

static void update_track(TrackedPerson *t, float x_cal, float y, float z, float vx, float vy, float vz) {
    if (fabsf(t->x_cal - x_cal) > 0.05f || fabsf(t->y - y) > 0.05f || fabsf(t->z - z) > 0.05f ||
        fabsf(t->vx - vx) > 0.02f || fabsf(t->vy - vy) > 0.02f || fabsf(t->vz - vz) > 0.02f) {
        changed = true;
    }
    t->x_cal = x_cal;
    t->y = y;
    t->z = z;
    t->vx = vx;
    t->vy = vy;
    t->vz = vz;
    t->last_seen = esp_timer_get_time() / 1000;
    
    // Lớp 2 - Track Continuity Rule: Lên hạng sau >= 3 frames
    if (t->state == TRACK_TENTATIVE) {
        t->hit_count++;
        if (t->hit_count >= 3) {
            t->state = TRACK_CONFIRMED;
            changed = true;
        }
    } else if (t->state == TRACK_TRANSFERRING) {
        // Nếu đã rời khỏi vùng trễ +- 0.3m, chốt lại confirm
        if (fabsf(x_cal) >= BOUNDARY_HYST_M) {
            t->state = TRACK_CONFIRMED;
            changed = true;
        }
    }
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
        
        // Thực tế Radar ID chỉ nằm trong byte đầu tiên (từ 0-255). Các byte tiếp theo (off+1, off+2) chứa 
        // Trạng thái (Move/Static) và Loại đối tượng (Type). Nếu đọc gộp 4 byte làm ID thì sẽ ra số khổng lồ
        uint8_t internal_id = frame[off + 0];
        
        float x_raw, y_raw, z_raw, vx, vy, vz;
        memcpy(&x_raw, &frame[off + 8],  4);
        memcpy(&y_raw, &frame[off + 12], 4);
        memcpy(&z_raw, &frame[off + 16], 4);
        memcpy(&vx,    &frame[off + 20], 4);
        memcpy(&vy,    &frame[off + 24], 4);
        memcpy(&vz,    &frame[off + 28], 4);

        float x_cal = calibrate_x(x_raw, radar_id);

        TrackedPerson *t = match_or_create_track(internal_id, radar_id, x_cal, y_raw, z_raw);
        if (t) update_track(t, x_cal, y_raw, z_raw, vx, vy, vz);
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
    printf("=== DUAL RADAR TRACKING (State Machine) ===\n");
    printf("Tracks: %d\n", track_count);
    printf("------------------------------------------------------------------------\n");
    printf("ID     Radar   State    X(m)    Y(m)    Z(m)    Vx     Vy     Vz\n");
    printf("------------------------------------------------------------------------\n");

    if (track_count == 0) {
        printf("              --- No persons detected ---\n");
    } else {
        for (int i = 0; i < track_count; i++) {
            if (!tracks[i].active) continue;
            
            const char* state_str = "TENT";
            if (tracks[i].state == TRACK_CONFIRMED) state_str = "CONF";
            else if (tracks[i].state == TRACK_TRANSFERRING) state_str = "TRANS";
            
            printf("%-6lu %-6s %-7s %7.2f %7.2f %7.2f %6.2f %6.2f %6.2f\n",
                   tracks[i].id,
                   tracks[i].owner_radar == RADAR_ID_RIGHT ? "RIGHT" : "LEFT",
                   state_str,
                   tracks[i].x_cal, tracks[i].y, tracks[i].z,
                   tracks[i].vx, tracks[i].vy, tracks[i].vz);
        }
    }
    printf("------------------------------------------------------------------------\n");

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