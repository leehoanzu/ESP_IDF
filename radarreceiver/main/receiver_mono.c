/*
 * radarreceiver.c — Dual Radar Receiver v4.3 MONO-TDM
 * Setup: 2 radar sát nhau (10cm), quét Full Range (-300~300), TDM luân phiên
 * Tối ưu: Bỏ Fast Exit, Dùng timeout 1500ms mượt mà cho chu kỳ TDM
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

static const char *TAG = "RADAR_RX_v4.3_MONO_TDM";

// ================= CONFIG =================
#define RADAR_HALF_GAP_M           0.05f
#define BOUNDARY_HYST_M            0.30f   // không dùng nữa nhưng giữ để tương thích

#define RADAR_ID_RIGHT       0x01
#define RADAR_ID_LEFT        0x02

#define RX_BUF_SIZE          4096
#define PARTIAL_TIMEOUT_MS   400

// THÊM 3 DÒNG NÀY ĐỂ TỐI ƯU CHO TDM 18 GIÂY:
#define FUSION_DISTANCE_M    0.6f   // Nới lỏng khoảng cách gộp ID (60cm)
#define FUSION_TIME_MS       3000    // Giữ chờ mục tiêu cũ trong 4 giây để gộp
#define TRACK_TIMEOUT_MS     3000  //4000

#define REVERSE_LEFT_X             0         // ← ĐỔI Ở ĐÂY: 
                                             // 0 = lắp cùng chiều (mặc định)
                                             // 1 = lắp NGƯỢC CHIỀU (hướng đối nhau)

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

static uint8_t rxBuf[2][RX_BUF_SIZE];
static int     rxIdx[2] = {0};
static int     rxTotal[2] = {0};
static uint64_t last_activity[2] = {0};

// ================= TRACK MANAGER =================
typedef enum { TRACK_TENTATIVE, TRACK_CONFIRMED } TrackState;  // BỎ TRANSFERRING

typedef struct {
    uint32_t id;
    uint32_t internal_id_right;
    uint32_t internal_id_left;
    uint8_t  owner_radar;
    float    x_cal, y, z;
    float    vx, vy, vz;
    uint64_t last_seen;
    bool     active;
    TrackState state;
    int      hit_count;
} TrackedPerson;

#define MAX_TRACKS 8
static TrackedPerson tracks[MAX_TRACKS];
static int track_count = 0;

static uint64_t last_print_ms = 0;
static bool     changed = false;

// ================= CALIBRATE =================
// static float calibrate_x(float x_raw, uint8_t radar_id) {
//     return (radar_id == RADAR_ID_RIGHT) ? (x_raw + RADAR_HALF_GAP_M) : (x_raw - RADAR_HALF_GAP_M);
// }

static float calibrate_x(float x_raw, uint8_t radar_id) {
    if (radar_id == RADAR_ID_RIGHT) {
        return x_raw + RADAR_HALF_GAP_M;
    } else {  // LEFT
        float x = x_raw - RADAR_HALF_GAP_M;
        return REVERSE_LEFT_X ? -x : x;   // ← Nếu ngược chiều thì đảo dấu X
    }
}

static void track_reset(void) {
    memset(tracks, 0, sizeof(tracks));
    track_count = 0;
    changed = true;
}

static uint32_t next_global_id = 1;

// static TrackedPerson* match_or_create_track(uint32_t internal_id, uint8_t radar_id, float x_cal, float y, float z) {
//     for (int i = 0; i < track_count; i++) {
//         if (!tracks[i].active) continue;
//         uint32_t my_id = (radar_id == RADAR_ID_RIGHT) ? tracks[i].internal_id_right : tracks[i].internal_id_left;
//         if (my_id == internal_id) return &tracks[i];
//     }

//     for (int i = 0; i < track_count; i++) {
//         if (!tracks[i].active) continue;
//         float dx = tracks[i].x_cal - x_cal;
//         float dy = tracks[i].y - y;
//         if (sqrtf(dx*dx + dy*dy) < 0.8f) {
//             if (radar_id == RADAR_ID_RIGHT) tracks[i].internal_id_right = internal_id;
//             else tracks[i].internal_id_left = internal_id;
//             return &tracks[i];
//         }
//     }

//     if (track_count < MAX_TRACKS) {
//         TrackedPerson *t = &tracks[track_count++];
//         t->id = next_global_id++;
//         t->internal_id_right = (radar_id == RADAR_ID_RIGHT) ? internal_id : 0xFFFFFFFF;
//         t->internal_id_left  = (radar_id == RADAR_ID_LEFT)  ? internal_id : 0xFFFFFFFF;
//         t->owner_radar = radar_id;
//         t->active = true;
//         t->state = TRACK_TENTATIVE;
//         t->hit_count = 1;
//         t->x_cal = x_cal; t->y = y; t->z = z;
//         t->vx = t->vy = t->vz = 0.0f;
//         t->last_seen = esp_timer_get_time() / 1000;
//         changed = true;
//         return t;
//     }
//     return NULL;
// }


// static TrackedPerson* match_or_create_track(uint32_t internal_id, uint8_t radar_id, float x_cal, float y, float z)
// {
//     uint64_t now = esp_timer_get_time() / 1000;

//     // STEP 1: Khớp chính xác internal_id (nhanh nhất)
//     for (int i = 0; i < track_count; i++) {
//         if (!tracks[i].active) continue;
//         uint32_t my_id = (radar_id == RADAR_ID_RIGHT) ? tracks[i].internal_id_right : tracks[i].internal_id_left;
//         if (my_id == internal_id) {
//             return &tracks[i];
//         }
//     }

//     // STEP 2: Fusion không gian + thời gian (cho phép cả same-radar và cross-radar)
//     TrackedPerson *best_match = NULL;
//     float min_dist = FUSION_DISTANCE_M;

//     for (int i = 0; i < track_count; i++) {
//         if (!tracks[i].active) continue;

//         // Bỏ qua nếu track quá cũ
//         if (now - tracks[i].last_seen > FUSION_TIME_MS) continue;

//         float dx = tracks[i].x_cal - x_cal;
//         float dy = tracks[i].y - y;
//         float dist = sqrtf(dx*dx + dy*dy);

//         if (dist < min_dist) {
//             min_dist = dist;
//             best_match = &tracks[i];
//         }
//     }

//     // Nếu tìm thấy track gần → gộp (cập nhật internal_id mới)
//     if (best_match != NULL) {
//         if (radar_id == RADAR_ID_RIGHT) {
//             best_match->internal_id_right = internal_id;
//             best_match->internal_id_left  = 0xFFFFFFFF;
//         } else {
//             best_match->internal_id_left  = internal_id;
//             best_match->internal_id_right = 0xFFFFFFFF;
//         }
//         return best_match;
//     }

//     // STEP 3: Tạo track mới
//     if (track_count < MAX_TRACKS) {
//         TrackedPerson *t = &tracks[track_count++];
//         t->id = next_global_id++;
//         t->internal_id_right = (radar_id == RADAR_ID_RIGHT) ? internal_id : 0xFFFFFFFF;
//         t->internal_id_left  = (radar_id == RADAR_ID_LEFT)  ? internal_id : 0xFFFFFFFF;
//         t->owner_radar = radar_id;
//         t->active = true;
//         t->state = TRACK_TENTATIVE;
//         t->hit_count = 1;
//         t->x_cal = x_cal;
//         t->y = y;
//         t->z = z;
//         t->vx = t->vy = t->vz = 0.0f;
//         t->last_seen = now;

//         changed = true;
//         return t;
//     }
//     return NULL;
// }

static TrackedPerson* match_or_create_track(uint32_t internal_id, uint8_t radar_id, float x_cal, float y, float z)
{
    uint64_t now = esp_timer_get_time() / 1000;

    // === ƯU TIÊN 1: Khớp chính xác internal_id (nhanh) ===
    for (int i = 0; i < track_count; i++) {
        if (!tracks[i].active) continue;
        uint32_t my_id = (radar_id == RADAR_ID_RIGHT) ? tracks[i].internal_id_right : tracks[i].internal_id_left;
        if (my_id == internal_id) return &tracks[i];
    }

    // === ƯU TIÊN 2: Fusion theo VỊ TRÍ 3D (ý tưởng chính của bạn) ===
    TrackedPerson *best_match = NULL;
    float min_dist = FUSION_DISTANCE_M;

    for (int i = 0; i < track_count; i++) {
        if (!tracks[i].active) continue;
        if (now - tracks[i].last_seen > FUSION_TIME_MS) continue;

        // Tính khoảng cách Euclidean 3D (X, Y, Z)
        float dx = tracks[i].x_cal - x_cal;
        float dy = tracks[i].y     - y;
        float dz = tracks[i].z     - z;
        float dist = sqrtf(dx*dx + dy*dy + dz*dz);

        if (dist < min_dist) {
            min_dist = dist;
            best_match = &tracks[i];
        }
    }

    // Nếu tìm thấy vị trí gần → merge (không cần nhảy ID)
    if (best_match != NULL) {
        if (radar_id == RADAR_ID_RIGHT) {
            best_match->internal_id_right = internal_id;
            best_match->internal_id_left  = 0xFFFFFFFF;
        } else {
            best_match->internal_id_left  = internal_id;
            best_match->internal_id_right = 0xFFFFFFFF;
        }
        return best_match;
    }

    // === ƯU TIÊN 3: Tạo track mới ===
    if (track_count < MAX_TRACKS) {
        TrackedPerson *t = &tracks[track_count++];
        t->id = next_global_id++;
        t->internal_id_right = (radar_id == RADAR_ID_RIGHT) ? internal_id : 0xFFFFFFFF;
        t->internal_id_left  = (radar_id == RADAR_ID_LEFT)  ? internal_id : 0xFFFFFFFF;
        t->owner_radar = radar_id;
        t->active = true;
        t->state = TRACK_TENTATIVE;
        t->hit_count = 1;
        t->x_cal = x_cal;
        t->y = y;
        t->z = z;
        t->vx = t->vy = t->vz = 0.0f;
        t->last_seen = now;

        changed = true;
        return t;
    }
    return NULL;
}

// ================= UPDATE TRACK – TDM OPTIMIZED =================
static void update_track(TrackedPerson *t, float x_cal, float y, float z, float vx, float vy, float vz, uint8_t source_radar) {
    t->last_seen = esp_timer_get_time() / 1000;

    // Chuyển owner NGAY LẬP TỨC
    if (t->owner_radar != source_radar) {
        t->owner_radar = source_radar;
        t->state = TRACK_CONFIRMED;
        changed = true;
    } else {
        t->hit_count++;
        if (t->state == TRACK_TENTATIVE && t->hit_count >= 2) {
            t->state = TRACK_CONFIRMED;
            changed = true;
        }
    }

    // Smoothing mạnh (giữ nguyên anti-jitter)
    float alpha = 0.85f;
    t->x_cal = t->x_cal * alpha + x_cal * (1.0f - alpha);
    t->y     = t->y     * alpha + y     * (1.0f - alpha);
    t->z     = t->z     * alpha + z     * (1.0f - alpha);
    t->vx    = t->vx    * 0.92f + vx    * 0.08f;
    t->vy    = t->vy    * 0.92f + vy    * 0.08f;
    t->vz    = t->vz    * 0.92f + vz    * 0.08f;

    // Đánh dấu thay đổi khi thực sự di chuyển > 2cm
    static float last_x[MAX_TRACKS], last_y[MAX_TRACKS];
    int idx = t - tracks;
    if (fabsf(t->x_cal - last_x[idx]) > 0.02f || fabsf(t->y - last_y[idx]) > 0.02f) {
        changed = true;
        last_x[idx] = t->x_cal;
        last_y[idx] = t->y;
    }
}

static void cleanup_old_tracks(void) {
    uint64_t now = esp_timer_get_time() / 1000;
    for (int i = 0; i < track_count; i++) {
        // Dùng TRACK_TIMEOUT_MS (4000ms) thay vì 1500 cứng
        if (tracks[i].active && (now - tracks[i].last_seen > TRACK_TIMEOUT_MS)) { 
            tracks[i].active = false;
            changed = true;
            if (i < track_count - 1) tracks[i] = tracks[track_count - 1];
            track_count--;
            i--;
        }
    }
}

// ================= PARSE (ĐÃ LƯỢC BỎ FAST EXIT DƯ THỪA) =================
static void parseRadarFrame(uint8_t *frame, int len, uint8_t radar_id) {
    if (len < 32) return;
    uint32_t trackLen; memcpy(&trackLen, &frame[28], 4);
    uint32_t count = trackLen / 32;

    for (uint32_t i = 0; i < count; i++) {
        int off = 32 + i * 32;
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
        if (t) update_track(t, x_cal, y_raw, z_raw, vx, vy, vz, radar_id);
    }
    
    cleanup_old_tracks();
    if (track_count == 0) changed = true;
}

static void handle_chunk(const uint8_t *data, int len, uint8_t radar_id) {
    if (len < 5 || data[0] != 0x01) return;
    int idx = (radar_id == RADAR_ID_RIGHT) ? 0 : 1;
    uint64_t now = esp_timer_get_time() / 1000;

    if (rxIdx[idx] > 0 && now - last_activity[idx] > PARTIAL_TIMEOUT_MS) {
        rxIdx[idx] = rxTotal[idx] = 0;
    }

    uint8_t seq = data[1];
    uint16_t total = (data[2] << 8) | data[3];
    const uint8_t *payload = data + 5;
    int chunkLen = len - 5;

    if (seq == 0) { rxIdx[idx] = 0; rxTotal[idx] = total; }

    if (rxIdx[idx] + chunkLen <= RX_BUF_SIZE) {
        memcpy(rxBuf[idx] + rxIdx[idx], payload, chunkLen);
        rxIdx[idx] += chunkLen;
        last_activity[idx] = now;
    }

    if (rxIdx[idx] >= rxTotal[idx] && rxTotal[idx] > 0) {
        if (rxTotal[idx] >= 2) {
            uint16_t recv_crc; memcpy(&recv_crc, rxBuf[idx] + rxTotal[idx] - 2, 2);
            if (crc16(rxBuf[idx], rxTotal[idx] - 2) == recv_crc) {
                parseRadarFrame(rxBuf[idx], rxTotal[idx] - 2, radar_id);
            }
        }
        rxIdx[idx] = rxTotal[idx] = 0;
    }
}

// ================= PRINT =================
static void print_table(void) {
    uint64_t now = esp_timer_get_time() / 1000;
    if (!changed && (now - last_print_ms < 400)) return;

    printf("\033[H\033[J");
    printf("=== DUAL RADAR v4.3 MONO-TDM ===\n");
    printf("Tracks: %d\n", track_count);
    printf("------------------------------------------------------------------------\n");
    printf("ID     Owner   State    X(m)    Y(m)    Z(m)    Vx     Vy     Vz\n");
    printf("------------------------------------------------------------------------\n");

    if (track_count == 0) {
        printf("              --- No persons detected ---\n");
    } else {
        for (int i = 0; i < track_count; i++) {
            if (!tracks[i].active) continue;
            const char* state_str = (tracks[i].state == TRACK_CONFIRMED) ? "CONF" : "TENT";
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

void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    if (!paired) {
        memcpy(senderMAC, info->src_addr, 6);
        esp_now_peer_info_t peer = {0};
        memcpy(peer.peer_addr, senderMAC, 6);
        esp_now_add_peer(&peer);
        paired = true;
        ESP_LOGI(TAG, "PAIRED WITH SENDER " MACSTR, MAC2STR(senderMAC));
    }
    if (len < 5 || data[0] != 0x01) return;
    handle_chunk(data, len, data[4]);
    print_table();
}


// ================= CONSOLE & MAIN =================
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

void app_main(void) {
    nvs_flash_init();
    esp_netif_init();
    esp_event_loop_create_default();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_start();
    
    uint8_t mac_addr[6];

    esp_read_mac(mac_addr, ESP_MAC_WIFI_STA);

    ESP_LOGI(TAG, "MAC: " MACSTR, MAC2STR(mac_addr));

    esp_now_init();
    esp_now_register_recv_cb(OnDataRecv);

    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);

    printf("\n=== DUAL-RADAR RECEIVER v4.3 MONO-TDM ===\n\n");
    track_reset();
    xTaskCreate(console_task, "console", 4096, NULL, 5, NULL);

    while (1) vTaskDelay(pdMS_TO_TICKS(1000));
}


/*
 * radarreceiver.c — Dual Radar Receiver v4.3 MONO-TDM (FUSION v1)
 * Setup: 2 radar sát nhau (~10cm), quét Full Range (-300~300), TDM luân phiên
 * Mục tiêu: tránh 2 ID cho cùng 1 người bằng fusion spatial+temporal
 */

// #include <stdio.h>
// #include <string.h>
// #include <math.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "esp_system.h"
// #include "esp_wifi.h"
// #include "esp_now.h"
// #include "esp_timer.h"
// #include "esp_mac.h"
// #include "esp_log.h"
// #include "nvs_flash.h"
// #include "esp_netif.h"

// static const char *TAG = "RADAR_RX_v4.3_MONO_TDM_FUSION";

// // MAC helper
// #define MACSTR "%02X:%02X:%02X:%02X:%02X:%02X"
// #define MAC2STR(a) (a)[0],(a)[1],(a)[2],(a)[3],(a)[4],(a)[5]

// // ================= CONFIG (tuning) =================
// #define RADAR_HALF_GAP_M           0.05f   // mỗi radar lệch 5cm so với tâm
// #define RADAR_SEPARATION_M         (RADAR_HALF_GAP_M * 2.0f) // ~0.10 m
// #define BOUNDARY_HYST_M            0.30f   // (không dùng)

// #define RADAR_ID_RIGHT       0x01
// #define RADAR_ID_LEFT        0x02

// #define RX_BUF_SIZE          4096
// #define PARTIAL_TIMEOUT_MS   400

// // Fusion parameters (quan trọng: chỉnh theo hiện trường)
// #define MERGE_DIST_M         0.50f    // nếu hai detection cách nhau < 0.5 m -> coi có thể là cùng 1 người
// #define MERGE_TIME_MS        12000    // nếu detection cách nhau trong 12s -> cho phép merge (TDM có chu kỳ 10s)
// #define CONFIRM_HITS         2        // số hit cần từ cùng radar để confirm nếu mới tạo
// #define TRACK_TIMEOUT_MS     1500     // timeout để xóa track (giữ như bạn muốn 1500ms)

// // ================= CRC16 =================
// static uint16_t crc16(const uint8_t *data, size_t len) {
//     uint16_t crc = 0;
//     for (size_t i = 0; i < len; i++) {
//         crc ^= (uint16_t)data[i] << 8;
//         for (int j = 0; j < 8; j++) {
//             crc = (crc & 0x8000) ? ((crc << 1) ^ 0x1021) : (crc << 1);
//         }
//     }
//     return crc;
// }

// // ================= GLOBAL =================
// static uint8_t senderMAC[6];
// static bool paired = false;

// static uint8_t rxBuf[2][RX_BUF_SIZE];
// static int     rxIdx[2] = {0};
// static int     rxTotal[2] = {0};
// static uint64_t last_activity[2] = {0};

// // ================= TRACK MANAGER =================
// typedef enum { TRACK_TENTATIVE, TRACK_CONFIRMED } TrackState;

// typedef struct {
//     uint32_t id;
//     uint32_t internal_id_right;
//     uint32_t internal_id_left;
//     uint8_t  owner_radar;    // RADAR_ID_*
//     float    x_cal, y, z;    // global calibrated coordinates (m)
//     float    vx, vy, vz;
//     uint64_t last_seen;      // ms
//     bool     active;
//     TrackState state;
//     int      hit_count;
// } TrackedPerson;

// #define MAX_TRACKS 8
// static TrackedPerson tracks[MAX_TRACKS];
// static int track_count = 0;

// static uint64_t last_print_ms = 0;
// static bool     changed = false;

// // ================= CALIBRATE =================
// static float calibrate_x(float x_raw, uint8_t radar_id) {
//     // convert radar local x to global x coordinate (center-line)
//     return (radar_id == RADAR_ID_RIGHT) ? (x_raw + RADAR_HALF_GAP_M) : (x_raw - RADAR_HALF_GAP_M);
// }

// static void track_reset(void) {
//     memset(tracks, 0, sizeof(tracks));
//     track_count = 0;
//     changed = true;
// }

// static uint32_t next_global_id = 1;

// // ===== helper: euclidean distance 2D (x,y) =====
// static inline float dist2d(float x1, float y1, float x2, float y2) {
//     float dx = x1 - x2;
//     float dy = y1 - y2;
//     return sqrtf(dx*dx + dy*dy);
// }

// // ================= MATCH OR CREATE TRACK (fusion-aware) =================
// static TrackedPerson* match_or_create_track(uint32_t internal_id, uint8_t radar_id, float x_cal, float y, float z) {
//     uint64_t now = esp_timer_get_time() / 1000;

//     // 1) Fast path: same internal id already mapped
//     for (int i = 0; i < track_count; i++) {
//         if (!tracks[i].active) continue;
//         uint32_t my_id = (radar_id == RADAR_ID_RIGHT) ? tracks[i].internal_id_right : tracks[i].internal_id_left;
//         if (my_id == internal_id) {
//             return &tracks[i];
//         }
//     }

//     // 2) Spatial + temporal matching:
//     // If there exists a track seen recently and located within MERGE_DIST_M, associate to it.
//     for (int i = 0; i < track_count; i++) {
//         if (!tracks[i].active) continue;
//         // time condition: allow match if previous observation not too old (MERGE_TIME_MS)
//         if ((now - tracks[i].last_seen) > MERGE_TIME_MS) continue;
//         float d = dist2d(tracks[i].x_cal, tracks[i].y, x_cal, y);
//         if (d <= MERGE_DIST_M) {
//             // associate internal id to that track
//             if (radar_id == RADAR_ID_RIGHT) tracks[i].internal_id_right = internal_id;
//             else tracks[i].internal_id_left = internal_id;
//             return &tracks[i];
//         }
//     }

//     // 3) If still not found, create new track if capacity
//     if (track_count < MAX_TRACKS) {
//         TrackedPerson *t = &tracks[track_count++];
//         t->id = next_global_id++;
//         t->internal_id_right = (radar_id == RADAR_ID_RIGHT) ? internal_id : 0xFFFFFFFF;
//         t->internal_id_left  = (radar_id == RADAR_ID_LEFT)  ? internal_id : 0xFFFFFFFF;
//         t->owner_radar = radar_id;
//         t->active = true;
//         t->state = TRACK_TENTATIVE;
//         t->hit_count = 1;
//         t->x_cal = x_cal; t->y = y; t->z = z;
//         t->vx = t->vy = t->vz = 0.0f;
//         t->last_seen = now;
//         changed = true;
//         return t;
//     }

//     // no slot available
//     return NULL;
// }

// // ================= UPDATE TRACK – TDM OPTIMIZED =================
// static void update_track(TrackedPerson *t, float x_cal, float y, float z, float vx, float vy, float vz, uint8_t source_radar) {
//     uint64_t now = esp_timer_get_time() / 1000;
//     t->last_seen = now;

//     // If owner changes to the new radar, promote to CONFIRMED
//     if (t->owner_radar != source_radar) {
//         t->owner_radar = source_radar;
//         t->state = TRACK_CONFIRMED;
//         changed = true;
//     } else {
//         t->hit_count++;
//         if (t->state == TRACK_TENTATIVE && t->hit_count >= CONFIRM_HITS) {
//             t->state = TRACK_CONFIRMED;
//             changed = true;
//         }
//     }

//     // strong smoothing to reduce jitter
//     float alpha = 0.85f;
//     t->x_cal = t->x_cal * alpha + x_cal * (1.0f - alpha);
//     t->y     = t->y     * alpha + y     * (1.0f - alpha);
//     t->z     = t->z     * alpha + z     * (1.0f - alpha);
//     t->vx    = t->vx    * 0.92f + vx    * 0.08f;
//     t->vy    = t->vy    * 0.92f + vy    * 0.08f;
//     t->vz    = t->vz    * 0.92f + vz    * 0.08f;

//     // mark changed only if moved > 2cm
//     static float last_x[MAX_TRACKS], last_y[MAX_TRACKS];
//     int idx = t - tracks;
//     if (fabsf(t->x_cal - last_x[idx]) > 0.02f || fabsf(t->y - last_y[idx]) > 0.02f) {
//         changed = true;
//         last_x[idx] = t->x_cal;
//         last_y[idx] = t->y;
//     }
// }

// // ================= CONSOLIDATE DUPLICATES (pairwise merge) =================
// // After parse, attempt to merge tracks that are close in space and time (created separately)
// static void consolidate_duplicates(void) {
//     bool merged_any = false;
//     uint64_t now = esp_timer_get_time() / 1000;

//     for (int i = 0; i < track_count; i++) {
//         if (!tracks[i].active) continue;
//         for (int j = i + 1; j < track_count; j++) {
//             if (!tracks[j].active) continue;
//             // only consider merging if both recently seen (within MERGE_TIME_MS)
//             if ((now - tracks[i].last_seen) > MERGE_TIME_MS && (now - tracks[j].last_seen) > MERGE_TIME_MS) continue;
//             float d = dist2d(tracks[i].x_cal, tracks[i].y, tracks[j].x_cal, tracks[j].y);
//             uint64_t dt = (tracks[i].last_seen > tracks[j].last_seen) ? (tracks[i].last_seen - tracks[j].last_seen) : (tracks[j].last_seen - tracks[i].last_seen);
//             if (d <= MERGE_DIST_M && dt <= MERGE_TIME_MS) {
//                 // merge j into i (keep i)
//                 // adopt missing internal ids
//                 if (tracks[i].internal_id_left == 0xFFFFFFFF && tracks[j].internal_id_left != 0xFFFFFFFF) tracks[i].internal_id_left = tracks[j].internal_id_left;
//                 if (tracks[i].internal_id_right == 0xFFFFFFFF && tracks[j].internal_id_right != 0xFFFFFFFF) tracks[i].internal_id_right = tracks[j].internal_id_right;
//                 // choose owner as the most recently seen
//                 if (tracks[j].last_seen > tracks[i].last_seen) {
//                     tracks[i].owner_radar = tracks[j].owner_radar;
//                     tracks[i].last_seen = tracks[j].last_seen;
//                 }
//                 // merge positions by weighted average (by hit_count)
//                 int total_hits = tracks[i].hit_count + tracks[j].hit_count;
//                 if (total_hits > 0) {
//                     tracks[i].x_cal = (tracks[i].x_cal * tracks[i].hit_count + tracks[j].x_cal * tracks[j].hit_count) / (float) total_hits;
//                     tracks[i].y     = (tracks[i].y     * tracks[i].hit_count + tracks[j].y     * tracks[j].hit_count) / (float) total_hits;
//                     tracks[i].z     = (tracks[i].z     * tracks[i].hit_count + tracks[j].z     * tracks[j].hit_count) / (float) total_hits;
//                 }
//                 tracks[i].hit_count = total_hits;
//                 if (tracks[i].state == TRACK_TENTATIVE && tracks[i].hit_count >= CONFIRM_HITS) tracks[i].state = TRACK_CONFIRMED;
//                 // remove j
//                 tracks[j].active = false;
//                 if (j < track_count - 1) tracks[j] = tracks[track_count - 1];
//                 track_count--;
//                 j--; // re-evaluate swapped element
//                 merged_any = true;
//                 changed = true;
//             }
//         }
//     }

//     (void)merged_any;
// }

// // ================= UPDATE / CLEANUP =================
// static void cleanup_old_tracks(void) {
//     uint64_t now = esp_timer_get_time() / 1000;
//     for (int i = 0; i < track_count; i++) {
//         if (tracks[i].active && (now - tracks[i].last_seen > TRACK_TIMEOUT_MS)) {
//             tracks[i].active = false;
//             changed = true;
//             if (i < track_count - 1) tracks[i] = tracks[track_count - 1];
//             track_count--;
//             i--;
//         }
//     }
// }

// // ================= PARSE (unchanged structure) =================
// static void parseRadarFrame(uint8_t *frame, int len, uint8_t radar_id) {
//     if (len < 32) return;
//     uint32_t trackLen; memcpy(&trackLen, &frame[28], 4);
//     uint32_t count = trackLen / 32;

//     for (uint32_t i = 0; i < count; i++) {
//         int off = 32 + i * 32;
//         uint8_t internal_id = frame[off + 0];

//         float x_raw, y_raw, z_raw, vx, vy, vz;
//         memcpy(&x_raw, &frame[off + 8],  4);
//         memcpy(&y_raw, &frame[off + 12], 4);
//         memcpy(&z_raw, &frame[off + 16], 4);
//         memcpy(&vx,    &frame[off + 20], 4);
//         memcpy(&vy,    &frame[off + 24], 4);
//         memcpy(&vz,    &frame[off + 28], 4);

//         float x_cal = calibrate_x(x_raw, radar_id);

//         TrackedPerson *t = match_or_create_track(internal_id, radar_id, x_cal, y_raw, z_raw);
//         if (t) update_track(t, x_cal, y_raw, z_raw, vx, vy, vz, radar_id);
//     }
    
//     // cleanup old tracks first (remove stale)
//     cleanup_old_tracks();

//     // then attempt to consolidate duplicates (merge tracks that likely represent same person)
//     consolidate_duplicates();

//     if (track_count == 0) changed = true;
// }

// // ================= CHUNK HANDLER (unchanged) =================
// static void handle_chunk(const uint8_t *data, int len, uint8_t radar_id) {
//     if (len < 5 || data[0] != 0x01) return;
//     int idx = (radar_id == RADAR_ID_RIGHT) ? 0 : 1;
//     uint64_t now = esp_timer_get_time() / 1000;

//     if (rxIdx[idx] > 0 && now - last_activity[idx] > PARTIAL_TIMEOUT_MS) {
//         rxIdx[idx] = rxTotal[idx] = 0;
//     }

//     uint8_t seq = data[1];
//     uint16_t total = (data[2] << 8) | data[3];
//     const uint8_t *payload = data + 5;
//     int chunkLen = len - 5;

//     if (seq == 0) { rxIdx[idx] = 0; rxTotal[idx] = total; }

//     if (rxIdx[idx] + chunkLen <= RX_BUF_SIZE) {
//         memcpy(rxBuf[idx] + rxIdx[idx], payload, chunkLen);
//         rxIdx[idx] += chunkLen;
//         last_activity[idx] = now;
//     }

//     if (rxIdx[idx] >= rxTotal[idx] && rxTotal[idx] > 0) {
//         if (rxTotal[idx] >= 2) {
//             uint16_t recv_crc; memcpy(&recv_crc, rxBuf[idx] + rxTotal[idx] - 2, 2);
//             if (crc16(rxBuf[idx], rxTotal[idx] - 2) == recv_crc) {
//                 parseRadarFrame(rxBuf[idx], rxTotal[idx] - 2, radar_id);
//             }
//         }
//         rxIdx[idx] = rxTotal[idx] = 0;
//     }
// }

// // ================= PRINT =================
// static void print_table(void) {
//     uint64_t now = esp_timer_get_time() / 1000;
//     if (!changed && (now - last_print_ms < 400)) return;

//     printf("\033[H\033[J");
//     printf("=== DUAL RADAR v4.3 MONO-TDM (FUSION) ===\n");
//     printf("Tracks: %d\n", track_count);
//     printf("--------------------------------------------------------------------------------\n");
//     printf("ID     Owner   State    X(m)    Y(m)    Z(m)    Hits  IntR   IntL   LastSeen(ms ago)\n");
//     printf("--------------------------------------------------------------------------------\n");

//     if (track_count == 0) {
//         printf("              --- No persons detected ---\n");
//     } else {
//         uint64_t nowms = esp_timer_get_time() / 1000;
//         for (int i = 0; i < track_count; i++) {
//             if (!tracks[i].active) continue;
//             const char* state_str = (tracks[i].state == TRACK_CONFIRMED) ? "CONF" : "TENT";
//             const char* owner = tracks[i].owner_radar == RADAR_ID_RIGHT ? "RIGHT" : "LEFT";
//             int intR = (tracks[i].internal_id_right == 0xFFFFFFFF) ? -1 : (int)tracks[i].internal_id_right;
//             int intL = (tracks[i].internal_id_left  == 0xFFFFFFFF) ? -1 : (int)tracks[i].internal_id_left;
//             printf("%-6lu %-6s %-7s %7.2f %7.2f %7.2f  %3d   %4d   %4d   %6lu\n",
//                    tracks[i].id,
//                    owner,
//                    state_str,
//                    tracks[i].x_cal, tracks[i].y, tracks[i].z,
//                    tracks[i].hit_count,
//                    intR, intL,
//                    (unsigned long)(nowms - tracks[i].last_seen));
//         }
//     }
//     printf("--------------------------------------------------------------------------------\n");

//     last_print_ms = now;
//     changed = false;
// }

// // ================= ESP-NOW RECEIVE CALLBACK =================
// void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
//     if (!paired) {
//         memcpy(senderMAC, info->src_addr, 6);
//         esp_now_peer_info_t peer = {0};
//         memcpy(peer.peer_addr, senderMAC, 6);
//         esp_now_add_peer(&peer);
//         paired = true;
//         ESP_LOGI(TAG, "PAIRED WITH SENDER " MACSTR, MAC2STR(senderMAC));
//     }
//     if (len < 5 || data[0] != 0x01) return;
//     handle_chunk(data, len, data[4]);
//     print_table();
// }

// // ================= CONSOLE & MAIN (unchanged) =================
// static void console_task(void *pv) {
//     char input[200];
//     ESP_LOGI(TAG, "Commands: SETUP | START | STOP | RESET | STATUS | R1:AT+xxx | R2:AT+xxx");

//     while (1) {
//         if (fgets(input, sizeof(input), stdin)) {
//             input[strcspn(input, "\r\n")] = 0;
//             if (strlen(input) == 0) continue;

//             if (strncmp(input, "R1:", 3) == 0 || strncmp(input, "R2:", 3) == 0) {
//                 if (!paired) {
//                     printf("Not paired yet!\n");
//                     continue;
//                 }
//                 uint8_t buf[200];
//                 buf[0] = 0x02;
//                 int l = snprintf((char*)buf + 1, 198, "%s\n", input);
//                 esp_now_send(senderMAC, buf, l + 1);
//             } else {
//                 if (!paired) {
//                     printf("Not paired yet!\n");
//                     continue;
//                 }
//                 uint8_t buf[200];
//                 buf[0] = 0x02;
//                 int l = snprintf((char*)buf + 1, 198, "%s\n", input);
//                 esp_now_send(senderMAC, buf, l + 1);
//             }
//         }
//         vTaskDelay(pdMS_TO_TICKS(10));
//     }
// }

// void app_main(void) {
//     nvs_flash_init();
//     esp_netif_init();
//     esp_event_loop_create_default();

//     wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
//     esp_wifi_init(&cfg);
//     esp_wifi_set_mode(WIFI_MODE_STA);
//     esp_wifi_start();

//     esp_now_init();
//     esp_now_register_recv_cb(OnDataRecv);

//     uint8_t mac[6];
//     esp_read_mac(mac, ESP_MAC_WIFI_STA);

//     printf("\n=== DUAL-RADAR RECEIVER v4.3 MONO-TDM (FUSION) ===\n\n");
//     track_reset();
//     xTaskCreate(console_task, "console", 4096, NULL, 5, NULL);

//     while (1) vTaskDelay(pdMS_TO_TICKS(1000));
// }