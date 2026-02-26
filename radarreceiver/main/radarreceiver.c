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
static uint32_t dbg_crc_fail     = 0;

#define RX_BUF_SIZE 4096

/* Mỗi radar có buffer riêng để ghép packet multi-chunk */
static uint8_t rxBuf1[RX_BUF_SIZE];
static int     rxIdx1 = 0;
static int     rxTotal1 = 0;   // totalLen kỳ vọng

static uint8_t rxBuf2[RX_BUF_SIZE];
static int     rxIdx2 = 0;
static int     rxTotal2 = 0;

// Timestamp cua chunk cuoi cung nhan duoc (cho timeout)
static uint64_t rxLastChunkMs1 = 0;
static uint64_t rxLastChunkMs2 = 0;

// ================= CRC16-CCITT =================
static uint16_t crc16(const uint8_t *data, int len)
{
    uint16_t crc = 0xFFFF;
    for (int i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; j++)
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
    }
    return crc;
}



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
    // packet: [0x01][seq][totalH][totalL][radar_id][payload…]
    // totalLen = original_frame + 2 (CRC16)

    if (len < 5 || data[0] != 0x01) return;

    uint8_t seq      = data[1];
    uint16_t total   = (data[2] << 8) | data[3];
    const uint8_t *payload  = data + 5;
    int            chunkLen = len - 5;

    uint8_t  *rxBuf;
    int      *rxIdx;
    int      *rxTotal;
    uint64_t *rxLastMs;

    if (radar_id == RADAR_ID_RIGHT) {
        rxBuf = rxBuf1; rxIdx = &rxIdx1; rxTotal = &rxTotal1;
        rxLastMs = &rxLastChunkMs1;
    } else {
        rxBuf = rxBuf2; rxIdx = &rxIdx2; rxTotal = &rxTotal2;
        rxLastMs = &rxLastChunkMs2;
    }

    uint64_t now_ms = esp_timer_get_time() / 1000;

    // Timeout: partial buffer > 300ms -> abort
    if (*rxIdx > 0 && *rxTotal > 0 && (now_ms - *rxLastMs > 300)) {
        *rxIdx = 0;
        *rxTotal = 0;
    }

    *rxLastMs = now_ms;

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
            // Verify CRC16 (2 byte cuoi cua totalLen)
            int frameLen = *rxTotal - 2;  // tru 2 byte CRC
            if (frameLen > 0) {
                uint16_t rx_crc  = (rxBuf[frameLen] << 8) | rxBuf[frameLen + 1];
                uint16_t calc_crc = crc16(rxBuf, frameLen);

                if (rx_crc == calc_crc) {
                    if (radar_id == RADAR_ID_RIGHT) dbg_right_frames++;
                    else                            dbg_left_frames++;
                    parseRadarFrame(rxBuf, frameLen, radar_id);
                } else {
                    dbg_crc_fail++;
                    const char *z = (radar_id == RADAR_ID_RIGHT) ? "RIGHT" : "LEFT";
                    printf("[CRC_FAIL] Radar %s  expected=0x%04X got=0x%04X  (total fails=%lu)\n",
                           z, calc_crc, rx_crc, (unsigned long)dbg_crc_fail);
                }
            }
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
                printf("  CRC Fails: %lu\n", (unsigned long)dbg_crc_fail);
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


