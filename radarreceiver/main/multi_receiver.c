// /*
//  * radarreceiver.c — Dual Radar Receiver v4.5 PRO MULTI-ZONE
//  * Setup: Hệ 3 Cục (6 Radar), theo dõi đa mục tiêu không gian 3D
//  * Tối ưu: Spatial & Velocity Gating, Local Prediction, Hysteresis, Ghost Rejection
//  */

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

// static const char *TAG = "RADAR_RX_v4.5_PRO";

// // ================= CONFIG & CONSTANTS =================
// #define RADAR_HALF_GAP_M           0.05f
// #define RADAR_ID_RIGHT             0x01
// #define RADAR_ID_LEFT              0x02
// #define RX_BUF_SIZE                4096
// #define PARTIAL_TIMEOUT_MS         400
// #define MAX_ZONES                  4       // Hỗ trợ Index 1, 2, 3

// // TỐI ƯU THUẬT TOÁN MULTI-TARGET TRACKING
// #define MERGE_DIST2                0.49f   // 0.7m * 0.7m (Tránh dùng sqrtf)
// #define VELOCITY_GATE              0.8f    // Sai lệch tổng vector vận tốc < 0.8 m/s
// #define SYSTEM_TRACK_TIMEOUT_MS    1500    // Xóa track chết sau 1.5 giây

// static const float ZONE_OFFSET_X[MAX_ZONES] = {0.0f,   0.0f,   3.70f,  3.70f};
// static const float ZONE_OFFSET_Y[MAX_ZONES] = {0.0f,   0.0f,   0.0f,   4.65f};

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

// // ================= STRUCT BỘ ĐỆM (ESP-NOW) =================
// static uint8_t senderMAC[6];
// static bool paired = false;

// typedef struct {
//     uint8_t  buf[RX_BUF_SIZE];
//     int      idx;
//     int      total;
//     uint64_t last_activity;
// } RadarRxBuffer;

// static RadarRxBuffer rxState[MAX_ZONES][2];

// // ================= STRUCT THEO DÕI NGƯỜI (TRACK MANAGER) =================
// typedef enum { TRACK_TENTATIVE, TRACK_CONFIRMED, TRACK_TRANSFERRING } TrackState;

// typedef struct {
//     uint32_t id;
//     uint8_t  internal_ids[MAX_ZONES][2];  // TỐI ƯU RAM: uint8_t thay vì uint32_t
//     uint8_t  owner_zone;                  
//     float    x, y, z;                     
//     float    vx, vy, vz;                  
//     uint64_t last_seen;
//     bool     active;
//     TrackState state;
    
//     // Biến chống nhiễu & Ghost Rejection
//     int      hit_count;                   
//     int      miss_count;
//     uint8_t  visibility_mask;             // Đánh dấu bit radar nhìn thấy mục tiêu
//     uint8_t  confidence;                  // Điểm tin cậy (0-100)
// } TrackedPerson;

// #define MAX_TRACKS 8
// static TrackedPerson tracks[MAX_TRACKS];
// static int track_count = 0;
// static uint32_t next_global_id = 1;

// static uint64_t last_print_ms = 0;
// static bool     changed = false;

// // ================= CALIBRATE TỌA ĐỘ VẬT LÝ =================
// static float calibrate_x(float x_raw, uint8_t zone_id, uint8_t radar_id) {
//     float x = (radar_id == RADAR_ID_RIGHT) ? (x_raw + RADAR_HALF_GAP_M) : (x_raw - RADAR_HALF_GAP_M);
//     return x + ZONE_OFFSET_X[zone_id];
// }

// static float calibrate_y(float y_raw, uint8_t zone_id) {
//     return y_raw + ZONE_OFFSET_Y[zone_id];
// }

// static void track_reset(void) {
//     memset(tracks, 0, sizeof(tracks));
//     track_count = 0;
//     changed = true;
// }

// // ================= THUẬT TOÁN GỘP NGƯỜI (DATA ASSOCIATION) =================
// static TrackedPerson* match_or_create_track(uint8_t internal_id, uint8_t zone_id, uint8_t radar_id, 
//                                             float x, float y, float z, float vx, float vy, float vz)
// {
//     uint64_t now = esp_timer_get_time() / 1000;
//     int r_idx = (radar_id == RADAR_ID_RIGHT) ? 0 : 1;

//     // 1. DÒ TÌM ID CŨ (O(N) Nhanh)
//     for (int i = 0; i < track_count; i++) {
//         if (!tracks[i].active) continue;
//         if (tracks[i].internal_ids[zone_id][r_idx] == internal_id) {
//             return &tracks[i];
//         }
//     }

//     // 2. GATING: TÌM KIẾM THEO KHÔNG GIAN & VẬN TỐC
//     TrackedPerson *best_match = NULL;
//     float best_dist2 = MERGE_DIST2;

//     for (int i = 0; i < track_count; i++) {
//         if (!tracks[i].active) continue;

//         // Local Prediction: Dự đoán quỹ đạo 3D (Đã có Clamp dt chống bay xuyên tường)
//         float dt = (now - tracks[i].last_seen) * 0.001f;
//         if (dt > 0.5f) dt = 0.5f; 
        
//         float pred_x = tracks[i].x + tracks[i].vx * dt;
//         float pred_y = tracks[i].y + tracks[i].vy * dt;
//         float pred_z = tracks[i].z + tracks[i].vz * dt;

//         // Axis Gating: Lọc Bounding Box thô
//         float dx = pred_x - x;
//         float dy = pred_y - y;
//         float dz = pred_z - z;
//         if (fabsf(dx) > 0.8f || fabsf(dy) > 0.8f || fabsf(dz) > 1.0f) continue;

//         // Spatial Gating: Tính bình phương khoảng cách
//         float dist2 = dx*dx + dy*dy + dz*dz;
//         if (dist2 > best_dist2) continue;

//         // Velocity Gating: Phân biệt 2 người đi sát nhau bằng Vector Vận tốc
//         float dvx = tracks[i].vx - vx;
//         float dvy = tracks[i].vy - vy;
//         float dvz = tracks[i].vz - vz;
//         float vel_diff = fabsf(dvx) + fabsf(dvy) + fabsf(dvz);
//         if (vel_diff > VELOCITY_GATE) continue;

//         best_match = &tracks[i];
//         best_dist2 = dist2;
//     }

//     // Nếu tìm thấy đồng đội, gộp ID
//     if (best_match != NULL) {
//         best_match->internal_ids[zone_id][r_idx] = internal_id;
//         return best_match;
//     }

//     // 3. TẠO NGƯỜI MỚI
//     if (track_count < MAX_TRACKS) {
//         TrackedPerson *t = &tracks[track_count++];
//         t->id = next_global_id++;
        
//         // Khởi tạo sạch ID toàn hệ thống (0xFF là ID trống)
//         for(int z_idx = 0; z_idx < MAX_ZONES; z_idx++) {
//             t->internal_ids[z_idx][0] = 0xFF;
//             t->internal_ids[z_idx][1] = 0xFF;
//         }
        
//         t->internal_ids[zone_id][r_idx] = internal_id;
//         t->owner_zone = zone_id;
//         t->active = true;
//         t->state = TRACK_TENTATIVE;
        
//         t->hit_count = 1;
//         t->miss_count = 0;
//         t->confidence = 0;
//         t->visibility_mask = 0;
        
//         t->x = x; t->y = y; t->z = z;
//         t->vx = vx; t->vy = vy; t->vz = vz;
//         t->last_seen = now;
//         changed = true;
//         return t;
//     }
//     return NULL;
// }

// // ================= CẬP NHẬT TRẠNG THÁI & GHOST REJECTION =================
// static void update_track(TrackedPerson *t, float x, float y, float z, 
//                          float vx, float vy, float vz, uint8_t source_zone, uint8_t radar_id) 
// {
//     uint64_t now = esp_timer_get_time() / 1000;
//     t->last_seen = now;
//     t->miss_count = 0;

//     // Bộ lọc Kalman-lite (Alpha-Beta Filter)
//     float alpha = 0.8f;
//     t->x = t->x * alpha + x * (1.0f - alpha);
//     t->y = t->y * alpha + y * (1.0f - alpha);
//     t->z = t->z * alpha + z * (1.0f - alpha);
    
//     t->vx = t->vx * 0.9f + vx * 0.1f;
//     t->vy = t->vy * 0.9f + vy * 0.1f;
//     t->vz = t->vz * 0.9f + vz * 0.1f;

//     // ================= 1. VISIBILITY MASK & CONFIDENCE =================
//     int r_idx = (radar_id == RADAR_ID_RIGHT) ? 0 : 1;
//     t->visibility_mask |= (1 << ((source_zone - 1) * 2 + r_idx));

//     int seen_by_radars = __builtin_popcount(t->visibility_mask);
//     int16_t new_conf = t->confidence;

//     if (seen_by_radars >= 2) {
//         new_conf += 5; // Người thật (Xác nhận chéo)
//     } else {
//         new_conf -= 2; // Nghi ngờ Ghost
//     }

//     if (new_conf > 100) new_conf = 100;
//     if (new_conf < 0) new_conf = 0;
//     t->confidence = (uint8_t)new_conf;

//     // ================= 2. ZONE HYSTERESIS =================
//     if (t->owner_zone != source_zone) {
//         t->hit_count++;
//         if (t->hit_count > 5) { 
//             t->owner_zone = source_zone;
//             t->state = (t->confidence > 50) ? TRACK_CONFIRMED : TRACK_TRANSFERRING;
//             t->hit_count = 0;
//         } else {
//             t->state = TRACK_TRANSFERRING;
//         }
//     } else {
//         t->hit_count = 0;
//         if ((t->state == TRACK_TENTATIVE || t->state == TRACK_TRANSFERRING) && t->confidence > 50) {
//             t->state = TRACK_CONFIRMED;
//         }
//     }
    
//     changed = true;
// }

// // ================= DỌN RÁC (MEMORY LEAK PREVENTION) =================
// static void cleanup_old_tracks(void) {
//     uint64_t now = esp_timer_get_time() / 1000;
//     for (int i = 0; i < track_count; i++) {
//         if (tracks[i].active && (now - tracks[i].last_seen > SYSTEM_TRACK_TIMEOUT_MS)) {
//             tracks[i].active = false;
//             changed = true;
//             if (i < track_count - 1) {
//                 tracks[i] = tracks[track_count - 1];
//             }
//             track_count--;
//             i--;
//         }
//     }
// }

// // ================= PHÂN TÍCH FRAME TỪ SENDER =================
// static void parseRadarFrame(uint8_t *frame, int len, uint8_t zone_id, uint8_t radar_id) {
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

//         float x_cal = calibrate_x(x_raw, zone_id, radar_id);
//         float y_cal = calibrate_y(y_raw, zone_id); 

//         TrackedPerson *t = match_or_create_track(internal_id, zone_id, radar_id, x_cal, y_cal, z_raw, vx, vy, vz);
//         if (t) update_track(t, x_cal, y_cal, z_raw, vx, vy, vz, zone_id, radar_id);
//     }
    
//     cleanup_old_tracks();
//     if (track_count == 0) changed = true;
// }

// // ================= XỬ LÝ GÓI TIN ESP-NOW THÔ =================
// static void handle_chunk(const uint8_t *data, int len, uint8_t zone_id, uint8_t radar_id) {
//     if (len < 6 || data[0] != 0x01) return;
//     if (zone_id <= 0 || zone_id >= MAX_ZONES) return; 

//     int r_idx = (radar_id == RADAR_ID_RIGHT) ? 0 : 1;
//     uint64_t now = esp_timer_get_time() / 1000;

//     RadarRxBuffer *rx = &rxState[zone_id][r_idx];

//     if (rx->idx > 0 && (now - rx->last_activity > PARTIAL_TIMEOUT_MS)) {
//         rx->idx = 0; 
//         rx->total = 0;
//     }

//     uint8_t seq = data[1];
//     uint16_t total = (data[2] << 8) | data[3];
//     const uint8_t *payload = data + 6; 
//     int chunkLen = len - 6;

//     if (seq == 0) { 
//         rx->idx = 0; 
//         rx->total = total; 
//     }

//     if (rx->idx + chunkLen <= RX_BUF_SIZE) {
//         memcpy(rx->buf + rx->idx, payload, chunkLen);
//         rx->idx += chunkLen;
//         rx->last_activity = now;
//     }

//     if (rx->idx >= rx->total && rx->total > 0) {
//         if (rx->total >= 2) {
//             uint16_t recv_crc; 
//             memcpy(&recv_crc, rx->buf + rx->total - 2, 2);
//             if (crc16(rx->buf, rx->total - 2) == recv_crc) {
//                 parseRadarFrame(rx->buf, rx->total - 2, zone_id, radar_id);
//             }
//         }
//         rx->idx = 0; 
//         rx->total = 0;
//     }
// }

// // ================= HIỂN THỊ CONSOLE (UI) =================
// static void print_table(void) {
//     uint64_t now = esp_timer_get_time() / 1000;
//     if (!changed && (now - last_print_ms < 400)) return;

//     printf("\033[H\033[J");
//     printf("=== DUAL RADAR v4.5 PRO MULTI-ZONE ===\n");
//     printf("Tracks: %d\n", track_count);
//     printf("--------------------------------------------------------------------------------\n");
//     // Thêm cột CF (Confidence) để bạn dễ debug Ghost
//     printf("ID     Zone    State    X(m)    Y(m)    Z(m)    Vx     Vy     Vz    CF(%%)\n");
//     printf("--------------------------------------------------------------------------------\n");

//     if (track_count == 0) {
//         printf("              --- No persons detected ---\n");
//     } else {
//         for (int i = 0; i < track_count; i++) {
//             if (!tracks[i].active) continue;
//             const char* state_str = (tracks[i].state == TRACK_CONFIRMED) ? "CONF" :
//                         (tracks[i].state == TRACK_TRANSFERRING) ? "TRANS" : "TENT";
            
//             printf("%-6lu Z%-5u %-7s %7.2f %7.2f %7.2f %6.2f %6.2f %6.2f   %3u%%\n",
//                    tracks[i].id,
//                    tracks[i].owner_zone, 
//                    state_str,
//                    tracks[i].x, tracks[i].y, tracks[i].z,
//                    tracks[i].vx, tracks[i].vy, tracks[i].vz,
//                    tracks[i].confidence);
                   
//             // XÓA MASK ĐỂ ĐÁNH GIÁ LẠI Ở CHU KỲ SAU (QUAN TRỌNG)
//             tracks[i].visibility_mask = 0;
//         }
//     }
//     printf("--------------------------------------------------------------------------------\n");

//     last_print_ms = now;
//     changed = false;
// }

// void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
//     if (!paired) {
//         memcpy(senderMAC, info->src_addr, 6);
//         esp_now_peer_info_t peer = {0};
//         memcpy(peer.peer_addr, senderMAC, 6);
//         esp_now_add_peer(&peer);
//         paired = true;
//         ESP_LOGI(TAG, "PAIRED WITH SENDER " MACSTR, MAC2STR(senderMAC));
//     }
    
//     if (len < 6 || data[0] != 0x01) return;
    
//     uint8_t zone_id = data[4];
//     uint8_t radar_id = data[5];
    
//     handle_chunk(data, len, zone_id, radar_id);
//     print_table();
// }

// // ================= UART CONSOLE TASK & MAIN =================
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
    
//     uint8_t mac_addr[6];
//     esp_read_mac(mac_addr, ESP_MAC_WIFI_STA);
//     ESP_LOGI(TAG, "MAC: " MACSTR, MAC2STR(mac_addr));

//     esp_now_init();
//     esp_now_register_recv_cb(OnDataRecv);

//     printf("\n=== DUAL-RADAR RECEIVER v4.5 PRO MULTI-ZONE ===\n\n");
//     track_reset();
//     xTaskCreate(console_task, "console", 4096, NULL, 5, NULL);

//     while (1) vTaskDelay(pdMS_TO_TICKS(1000));
// }



//*
//  * radarreceiver.c — Dual Radar Receiver v4.6 ASYNC PRO
//  * Setup: Hệ 3 Cục (6 Radar) - ESP-IDF v5.5.2
//  * Tối ưu: Async Queue, Natural Fusion, Ghost Rejection
//  */
/*
 * radarreceiver.c — Dual Radar Receiver v4.7 HYBRID (Production Ready)
 * Hệ 3 Cục (6 Radar) - Async + Natural Fusion + Ghost Rejection
 * ESP-IDF v5.5.2
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_timer.h"
#include "esp_mac.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_task_wdt.h"

static const char *TAG = "RADAR_RX_v4.7_HYBRID";

// ================= LOW-PASS FILTER (thêm vào đầu file) =================
#define LP_ALPHA_POS    0.65f     // Mượt vị trí (0.5~0.75)
#define LP_ALPHA_VEL    0.35f     // Mượt vận tốc (nhỏ hơn để nhạy hơn với thay đổi)

// ================= CONFIG & CONSTANTS =================
#define RADAR_HALF_GAP_M           0.05f
#define RADAR_ID_RIGHT             0x01
#define RADAR_ID_LEFT              0x02
#define RX_BUF_SIZE                4096
#define PARTIAL_TIMEOUT_MS         400
#define MAX_ZONES                  4

// ================= GATING TỐI ƯU (đã test overlap 50cm) =================
#define MERGE_DIST2                4.0f      // 2.0m x 2.0m - hút mạnh khi overlap
#define VELOCITY_GATE              1.5f
#define AXIS_GATE_XY               1.2f
#define AXIS_GATE_Z                1.5f
#define SYSTEM_TRACK_TIMEOUT_MS    2000
#define HIT_COUNT_CONFIRM          4         // Giảm để handoff nhanh

// Index:                                       [0]    [1/Z1]  [2/Z2]  [3/Z3]
static const float ZONE_OFFSET_X[MAX_ZONES] = {0.0f,   0.0f,  1.0f,   0.0f}; 
static const float ZONE_OFFSET_Y[MAX_ZONES] = {0.0f,   0.0f,   0.0f,   0.0f};


// Index 0: không dùng | 1=Zone1 | 2=Zone2 | 3=Zone3
// static const float ZONE_OFFSET_X[MAX_ZONES] = {0.0f, 0.0f, 3.70f, 3.70f};
// static const float ZONE_OFFSET_Y[MAX_ZONES] = {0.0f, 0.0f, 0.0f,  4.65f};

typedef struct {
    uint8_t  buf[RX_BUF_SIZE];
    int      idx;
    int      total;
    uint64_t last_activity;
} RadarRxBuffer;

typedef struct {
    uint8_t  zone_id;
    uint8_t  radar_id;
    uint8_t  data[250];
    size_t   len;
} FrameItem_t;

typedef enum { TRACK_TENTATIVE, TRACK_CONFIRMED, TRACK_TRANSFERRING } TrackState;

typedef struct {
    uint32_t id;
    uint8_t  internal_ids[MAX_ZONES][2];
    uint8_t  owner_zone;
    float    x, y, z;
    float    vx, vy, vz;
    uint64_t last_seen;
    bool     active;
    TrackState state;
    int      hit_count;
    int      miss_count;
    uint8_t  visibility_mask;
    uint8_t  confidence;
} TrackedPerson;

// ================= GLOBAL =================
static uint8_t senderMAC[6];
static bool paired = false;

QueueHandle_t frame_queue = NULL;
static RadarRxBuffer rxState[MAX_ZONES][2];

#define MAX_TRACKS 8
static TrackedPerson tracks[MAX_TRACKS];
static int track_count = 0;
static uint32_t next_global_id = 1;

static uint64_t last_print_ms = 0;
static bool     changed = false;

// ================= CRC16 & CALIBRATE =================
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

static float calibrate_x(float x_raw, uint8_t zone_id, uint8_t radar_id) {
    float x = (radar_id == RADAR_ID_RIGHT) ? (x_raw + RADAR_HALF_GAP_M) : (x_raw - RADAR_HALF_GAP_M);
    return x + ZONE_OFFSET_X[zone_id];
}

static float calibrate_y(float y_raw, uint8_t zone_id) {
    return y_raw + ZONE_OFFSET_Y[zone_id];
}

static void track_reset(void) {
    memset(tracks, 0, sizeof(tracks));
    track_count = 0;
    changed = true;
}

// ================= MATCH - NATURAL FUSION + AXIS GATING =================
static TrackedPerson* match_or_create_track(uint8_t internal_id, uint8_t zone_id, uint8_t radar_id,
                                            float world_x, float world_y, float z, float vx, float vy, float vz)
{
    uint64_t now = esp_timer_get_time() / 1000;
    int r_idx = (radar_id == RADAR_ID_RIGHT) ? 0 : 1;

    // Tìm ID cũ
    for (int i = 0; i < track_count; i++) {
        if (tracks[i].active && tracks[i].internal_ids[zone_id][r_idx] == internal_id)
            return &tracks[i];
    }

    // Natural Fusion + Axis Gating
    TrackedPerson *best = NULL;
    float best_dist2 = MERGE_DIST2;

    for (int i = 0; i < track_count; i++) {
        if (!tracks[i].active) continue;

        float dt = fminf((now - tracks[i].last_seen) * 0.001f, 0.5f);
        float pred_x = tracks[i].x + tracks[i].vx * dt;
        float pred_y = tracks[i].y + tracks[i].vy * dt;
        float pred_z = tracks[i].z + tracks[i].vz * dt;

        float dx = pred_x - world_x;
        float dy = pred_y - world_y;
        float dz = pred_z - z;

        if (fabsf(dx) > AXIS_GATE_XY || fabsf(dy) > AXIS_GATE_XY || fabsf(dz) > AXIS_GATE_Z) continue;

        float dist2 = dx*dx + dy*dy + dz*dz;
        if (dist2 >= best_dist2) continue;

        float vel_diff = fabsf(tracks[i].vx - vx) + fabsf(tracks[i].vy - vy);
        if (vel_diff > VELOCITY_GATE) continue;

        best = &tracks[i];
        best_dist2 = dist2;
    }

    if (best) {
        best->internal_ids[zone_id][r_idx] = internal_id;
        return best;
    }

    // Tạo mới
    if (track_count < MAX_TRACKS) {
        TrackedPerson *t = &tracks[track_count++];
        t->id = next_global_id++;
        for(int z_idx = 0; z_idx < MAX_ZONES; z_idx++) {
            t->internal_ids[z_idx][0] = 0xFF;
            t->internal_ids[z_idx][1] = 0xFF;
        }
        t->internal_ids[zone_id][r_idx] = internal_id;
        t->owner_zone = zone_id;
        t->active = true;
        t->state = TRACK_TENTATIVE;
        t->hit_count = 1;
        t->miss_count = 0;
        t->confidence = 0;
        t->visibility_mask = 0;
        t->x = world_x; t->y = world_y; t->z = z;
        t->vx = vx; t->vy = vy; t->vz = vz;
        t->last_seen = now;
        changed = true;
        return t;
    }
    return NULL;
}

// ================= UPDATE TRACK (full ghost rejection từ v4.5) =================
static void update_track(TrackedPerson *t, float x, float y, float z,
                         float vx, float vy, float vz, uint8_t source_zone, uint8_t radar_id)
{
    uint64_t now = esp_timer_get_time() / 1000;
    t->last_seen = now;
    t->miss_count = 0;

    // ================= LOW-PASS FILTER (thông thấp) =================
    // Vị trí - mượt mạnh
    t->x = t->x * LP_ALPHA_POS + x * (1.0f - LP_ALPHA_POS);
    t->y = t->y * LP_ALPHA_POS + y * (1.0f - LP_ALPHA_POS);
    t->z = t->z * LP_ALPHA_POS + z * (1.0f - LP_ALPHA_POS);

    // Vận tốc - mượt nhẹ hơn để vẫn phản hồi nhanh
    t->vx = t->vx * LP_ALPHA_VEL + vx * (1.0f - LP_ALPHA_VEL);
    t->vy = t->vy * LP_ALPHA_VEL + vy * (1.0f - LP_ALPHA_VEL);
    t->vz = t->vz * LP_ALPHA_VEL + vz * (1.0f - LP_ALPHA_VEL);

    // ================= Ghost Rejection & Hysteresis (giữ nguyên) =================
    int r_idx = (radar_id == RADAR_ID_RIGHT) ? 0 : 1;
    t->visibility_mask |= (1 << ((source_zone - 1) * 2 + r_idx));

    int seen_by_radars = __builtin_popcount(t->visibility_mask);
    int16_t new_conf = t->confidence;
    if (seen_by_radars >= 2) {
        new_conf += 5;
    } else {
        new_conf -= 2;
    }
    if (new_conf > 100) new_conf = 100;
    if (new_conf < 0) new_conf = 0;
    t->confidence = (uint8_t)new_conf;

    if (t->owner_zone != source_zone) {
        t->hit_count++;
        if (t->hit_count > HIT_COUNT_CONFIRM) {
            t->owner_zone = source_zone;
            t->state = (t->confidence > 50) ? TRACK_CONFIRMED : TRACK_TRANSFERRING;
            t->hit_count = 0;
        } else {
            t->state = TRACK_TRANSFERRING;
        }
    } else {
        t->hit_count = 0;
        if ((t->state == TRACK_TENTATIVE || t->state == TRACK_TRANSFERRING) && t->confidence > 50) {
            t->state = TRACK_CONFIRMED;
        }
    }
    changed = true;
}

// ================= PARSE TASK (Async) =================
static void parse_task(void *pv) {
    FrameItem_t item;
    while (1) {
        if (xQueueReceive(frame_queue, &item, portMAX_DELAY) == pdTRUE) {
            if (item.zone_id <= 0 || item.zone_id >= MAX_ZONES) continue;

            int r_idx = (item.radar_id == RADAR_ID_RIGHT) ? 0 : 1;
            RadarRxBuffer *rx = &rxState[item.zone_id][r_idx];
            uint64_t now = esp_timer_get_time() / 1000;

            if (rx->idx > 0 && (now - rx->last_activity > PARTIAL_TIMEOUT_MS)) rx->idx = 0;
            if (item.data[1] == 0) { rx->idx = 0; rx->total = (item.data[2] << 8) | item.data[3]; }

            size_t chunkLen = item.len - 6;
            if (rx->idx + chunkLen <= RX_BUF_SIZE) {
                memcpy(rx->buf + rx->idx, item.data + 6, chunkLen);
                rx->idx += chunkLen;
                rx->last_activity = now;
            }

            if (rx->idx >= rx->total && rx->total > 0) {
                uint16_t recv_crc;
                memcpy(&recv_crc, rx->buf + rx->total - 2, 2);
                if (crc16(rx->buf, rx->total - 2) == recv_crc) {
                    uint32_t trackLen;
                    memcpy(&trackLen, &rx->buf[28], 4);
                    int count = trackLen / 32;
                    for (int i = 0; i < count; i++) {
                        int off = 32 + i * 32;
                        float xr, yr, zr, vx, vy, vz;
                        memcpy(&xr, &rx->buf[off+8], 4);
                        memcpy(&yr, &rx->buf[off+12], 4);
                        memcpy(&zr, &rx->buf[off+16], 4);
                        memcpy(&vx, &rx->buf[off+20], 4);
                        memcpy(&vy, &rx->buf[off+24], 4);
                        memcpy(&vz, &rx->buf[off+28], 4);

                        float xc = calibrate_x(xr, item.zone_id, item.radar_id);
                        float yc = calibrate_y(yr, item.zone_id);

                        TrackedPerson *t = match_or_create_track(rx->buf[off], item.zone_id, item.radar_id, xc, yc, zr, vx, vy, vz);
                        if (t) update_track(t, xc, yc, zr, vx, vy, vz, item.zone_id, item.radar_id);
                    }
                }
                rx->idx = 0;
            }

            // Cleanup
            now = esp_timer_get_time() / 1000;
            for (int i = 0; i < track_count; i++) {
                if (tracks[i].active && (now - tracks[i].last_seen > SYSTEM_TRACK_TIMEOUT_MS)) {
                    tracks[i].active = false;
                    changed = true;
                    if (i < track_count - 1) tracks[i] = tracks[track_count - 1];
                    track_count--; i--;
                }
            }

            // Print 400ms
            if (changed && (now - last_print_ms > 400)) {
                printf("\033[H\033[J=== DUAL RADAR v4.7 HYBRID ===\nTracks: %d\n", track_count);
                printf("--------------------------------------------------------------------------------\n");
                printf("ID     Zone    State    X(m)    Y(m)    Z(m)    Vx     Vy     Vz    CF(%%)\n");
                printf("--------------------------------------------------------------------------------\n");
                if (track_count == 0) {
                    printf("              --- No persons detected ---\n");
                } else {
                    for (int i = 0; i < track_count; i++) {
                        if (!tracks[i].active) continue;
                        const char* state_str = (tracks[i].state == TRACK_CONFIRMED) ? "CONF" :
                                                (tracks[i].state == TRACK_TRANSFERRING) ? "TRANS" : "TENT";
                        printf("%-6lu Z%-5u %-7s %7.2f %7.2f %7.2f %6.2f %6.2f %6.2f   %3u%%\n",
                               tracks[i].id, tracks[i].owner_zone, state_str,
                               tracks[i].x, tracks[i].y, tracks[i].z,
                               tracks[i].vx, tracks[i].vy, tracks[i].vz, tracks[i].confidence);
                        tracks[i].visibility_mask = 0;
                    }
                }
                printf("--------------------------------------------------------------------------------\n");
                last_print_ms = now;
                changed = false;
            }
        }
    }
}

// ================= ESP-NOW CALLBACK =================
// void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
//     if (len < 6 || data[0] != 0x01) return;

//     if (!paired) {
//         memcpy(senderMAC, info->src_addr, 6);
//         esp_now_peer_info_t peer = {0};
//         memcpy(peer.peer_addr, senderMAC, 6);
//         esp_now_add_peer(&peer);
//         paired = true;
//         ESP_LOGI(TAG, "PAIRED WITH SENDER " MACSTR, MAC2STR(senderMAC));
//     }

//     FrameItem_t item;
//     item.zone_id = data[4];
//     item.radar_id = data[5];
//     item.len = (len > 250) ? 250 : len;
//     memcpy(item.data, data, item.len);
//     xQueueSend(frame_queue, &item, 0);
// }

// ================= ESP-NOW CALLBACK =================
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    if (len < 1) return;

    // -------------------------------------------------------------
    // 1. NẾU LÀ GÓI TIN PHẢN HỒI OTA (ACK - Header 0x03)
    // -------------------------------------------------------------
    if (data[0] == 0x03 && len >= 2) {
        uint8_t ack_zone = data[1];
        char ack_msg[100];
        int msg_len = len - 2;
        if (msg_len >= sizeof(ack_msg)) msg_len = sizeof(ack_msg) - 1;
        
        memcpy(ack_msg, data + 2, msg_len);
        ack_msg[msg_len] = '\0'; // Kết thúc chuỗi
        
        // In ra màn hình Terminal với màu Xanh Lá (1;32m) cho nổi bật
        printf("\n\033[1;32m[SUCCESS] ZONE %d BÁO CÁO: %s\033[0m\n\n", ack_zone, ack_msg);
        return; // Xử lý xong tin nhắn thì thoát, không đẩy vào hàng đợi Radar
    }

    // -------------------------------------------------------------
    // 2. NẾU LÀ GÓI TIN DỮ LIỆU RADAR (Header 0x01)
    // -------------------------------------------------------------
    if (len < 6 || data[0] != 0x01) return;

    if (!esp_now_is_peer_exist(info->src_addr)) {
        esp_now_peer_info_t peer = {0};
        memcpy(peer.peer_addr, info->src_addr, 6);
        ESP_LOGI(TAG, "New Sender: " MACSTR " (Zone %u)", MAC2STR(info->src_addr), data[4]);
        esp_now_add_peer(&peer);
    }

    memcpy(senderMAC, info->src_addr, 6); 

    FrameItem_t item;
    item.zone_id = data[4];
    item.radar_id = data[5];
    item.len = (len > 250) ? 250 : len;
    memcpy(item.data, data, item.len);
    
    // Đẩy vào hàng đợi để parse_task xử lý
    xQueueSend(frame_queue, &item, 0); 
}

// ================= CONSOLE TASK (AT command) =================
// static void console_task(void *pv) {
//     char input[200];
//     ESP_LOGI(TAG, "Commands: R1:AT+xxx | R2:AT+xxx | STATUS");

//     while (1) {
//         if (fgets(input, sizeof(input), stdin)) {
//             input[strcspn(input, "\r\n")] = 0;
//             if (strlen(input) == 0) continue;

//             if (!paired) {
//                 printf("Not paired yet!\n");
//                 continue;
//             }
//             uint8_t buf[200];
//             buf[0] = 0x02;
//             int l = snprintf((char*)buf + 1, 198, "%s\n", input);
//             esp_now_send(senderMAC, buf, l + 1);
//         }
//         vTaskDelay(pdMS_TO_TICKS(10));
//     }
// }

// ================= CONSOLE TASK (AT command) =================
static void console_task(void *pv) {
    char input[200];
    ESP_LOGI(TAG, "Commands: Z1:AT+xxx | Z2:AT+xxx | Z3:AT+xxx | ALL:AT+xxx");

    // Địa chỉ Broadcast MAC (Gửi cho tất cả mọi thiết bị trong vùng phủ sóng)
    uint8_t broadcast_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

    while (1) {
        if (fgets(input, sizeof(input), stdin)) {
            input[strcspn(input, "\r\n")] = 0;
            if (strlen(input) == 0) continue;

            if (strncmp(input, "Z1:", 3) == 0 || strncmp(input, "Z2:", 3) == 0 || 
                strncmp(input, "Z3:", 3) == 0 || strncmp(input, "ALL:", 4) == 0) {
                
                // Thêm địa chỉ Broadcast vào danh sách Peer nếu chưa có
                if (!esp_now_is_peer_exist(broadcast_mac)) {
                    esp_now_peer_info_t peer = {0};
                    memcpy(peer.peer_addr, broadcast_mac, 6);
                    esp_now_add_peer(&peer);
                }

                uint8_t buf[200];
                buf[0] = 0x02; // Header lệnh điều khiển
                int l = snprintf((char*)buf + 1, 198, "%s\n", input);
                
                // Gửi Broadcast đi
                esp_now_send(broadcast_mac, buf, l + 1);
                ESP_LOGI(TAG, "Đã gửi Broadcast: %s", input);
                
            } else {
                printf("Sai cú pháp! Hãy dùng: Z1:AT+... hoặc ALL:AT+...\n");
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ================= APP_MAIN =================
void app_main(void) {
    nvs_flash_init();
    esp_netif_init();
    esp_event_loop_create_default();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_start();

    // Watchdog 15s
    esp_task_wdt_config_t twdt_config = {
        .timeout_ms = 15000,
        .trigger_panic = true,
        .idle_core_mask = (1 << 0) | (1 << 1)
    };
    esp_task_wdt_deinit();
    esp_task_wdt_init(&twdt_config);

    esp_now_init();
    esp_now_register_recv_cb(OnDataRecv);

    frame_queue = xQueueCreate(20, sizeof(FrameItem_t));
    memset(rxState, 0, sizeof(rxState));
    track_reset();

    xTaskCreate(parse_task, "parse_task", 8192, NULL, 10, NULL);
    xTaskCreate(console_task, "console", 4096, NULL, 5, NULL);

    printf("\n=== DUAL-RADAR RECEIVER v4.7 HYBRID READY ===\n");
}