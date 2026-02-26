/*
 * sender.c ΓÇö Dual Radar Sender
 * =============================================================
 * Kiß║┐n tr├║c 2 radar trong 1 cß╗Ñc (khoß║úng c├ích gß║ºn nhau ~30ΓÇô70 cm):
 *
 *   R1 = RADAR_RIGHT  (UART2, pin 16/17)  ΓåÆ  phß╗º nß╗¡a PHß║óI  (X ΓëÑ 0)
 *   R2 = RADAR_LEFT   (UART1, pin 41/42)  ΓåÆ  phß╗º nß╗¡a TR├üI  (X < 0)
 *
 * ΓöÇΓöÇ Spatial Partition (ph├ón v├╣ng cß╗⌐ng) ΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇ
 *   R1: AT+XNEGAD=0   ┬╖ AT+XPOSID=350   ΓåÆ firmware loß║íi X < 0
 *   R2: AT+XNEGAD=-350┬╖ AT+XPOSID=0     ΓåÆ firmware loß║íi X > 0
 *   ΓåÆ 2 radar kh├┤ng bao giß╗¥ b├ío c├╣ng 1 mß╗Ñc ti├¬u.
 *
 * ΓöÇΓöÇ Prime Stagger (giß║úm RF coupling) ΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇ
 *   R1: AT+TIME=131 ms
 *   R2: AT+TIME=101 ms
 *   LCM(101,131) = 13 231 ms ΓåÆ va chß║ím < 1 lß║ºn / 13 s
 *
 * ΓöÇΓöÇ Coordinate Calibration (b├╣ lß╗çch vß║¡t l├╜) ΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇ
 *   Gß║»n radar_id v├áo packet; receiver b├╣ offset ┬▒X_OFFSET_CM.
 *   R1 lß║»p b├¬n PHß║óI t├óm cß╗Ñc ΓåÆ offset = +RADAR_HALF_GAP_CM
 *   R2 lß║»p b├¬n TR├üI t├óm cß╗Ñc ΓåÆ offset = -RADAR_HALF_GAP_CM
 *
 * ΓöÇΓöÇ Packet format gß╗¡i qua ESP-NOW ΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇ
 *   [0x01][seq][lenH][lenL][radar_id][payloadΓÇª]
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

/* ΓöÇΓöÇ Radar 1 (RIGHT): phß╗º nß╗¡a PHß║óI, X Γêê [0, +350 cm] ΓöÇΓöÇ */
#define RADAR1_UART      UART_NUM_2
#define RADAR1_RX_PIN    16
#define RADAR1_TX_PIN    17
#define RADAR1_TIME_MS   131          // Prime Stagger: chu kß╗│ 131 ms
#define RADAR1_XNEG      0            // Spatial Partition: chß║╖n X ├óm
#define RADAR1_XPOS      350          // Mß╗⌐c X d╞░╞íng tß╗æi ─æa (cm)

/* ΓöÇΓöÇ Radar 2 (LEFT): phß╗º nß╗¡a TR├üI, X Γêê [-350, 0 cm] ΓöÇΓöÇ */
#define RADAR2_UART      UART_NUM_1
#define RADAR2_RX_PIN    41
#define RADAR2_TX_PIN    42
#define RADAR2_TIME_MS   101          // Prime Stagger: chu kß╗│ 101 ms
#define RADAR2_XNEG      (-350)       // Mß╗⌐c X ├óm tß╗æi ─æa (cm)
#define RADAR2_XPOS      0            // Spatial Partition: chß║╖n X d╞░╞íng

/* ΓöÇΓöÇ Coordinate Calibration ΓöÇΓöÇ */
// Nß║┐u 2 radar ─æß║╖t gß║ºn t├óm (khoß║úng c├ích nhß╗Å), offset nhß╗Å.
// ─Éo thß╗▒c tß║┐: khoß║úng c├ích 2 radar / 2 (cm). V├¡ dß╗Ñ 30cm ΓåÆ 15cm.
#define RADAR_HALF_GAP_CM   35      // 70cm center-to-center / 2 (khop voi receiver 0.35m)

/* ΓöÇΓöÇ Sensor height ΓöÇΓöÇ */
#define RADAR_HEIGHT_CM  100   ///267          // Chiß╗üu cao lß║»p ─æß║╖t (cm)

/* ΓöÇΓöÇ radar_id bytes ΓöÇΓöÇ */
#define RADAR_ID_RIGHT   0x01
#define RADAR_ID_LEFT    0x02

/* ΓöÇΓöÇ Misc ΓöÇΓöÇ */
#define RADAR_BUF_SIZE   2048
#define RADAR_RANGE_CM   350          // Thu hß║╣p tß║ºm qu├⌐t giß║úm nhiß╗àu xa

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
    printf("[ATΓåÆUART%d] %s", (port == RADAR1_UART) ? 2 : 1, cmd);
}

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

// ================= ESP-NOW RECV =================
// Receiver gß╗¡i lß╗çnh AT vß╗ü ΓåÆ chuyß╗ân thß║│ng xuß╗æng RADAR_1 (mß║╖c ─æß╗ïnh)
// Nß║┐u cß║ºn ─æiß╗üu khiß╗ân tß╗½ng radar, th├¬m prefix v├áo lß╗çnh.
void OnDataRecv(const esp_now_recv_info_t *info,
                const uint8_t *data,
                int len)
{
    if (len > 1 && data[0] == 0x02) {
        const char *cmd = (const char *)(data + 1);
        int cmdLen = len - 1;

        // R1: prefix ΓåÆ chi gui radar RIGHT
        if (cmdLen > 3 && cmd[0]=='R' && cmd[1]=='1' && cmd[2]==':') {
            uart_write_bytes(RADAR1_UART, cmd + 3, cmdLen - 3);
            printf("\n>> Cmd fwd R1 only: %.*s\n", cmdLen - 3, cmd + 3);
        }
        // R2: prefix ΓåÆ chi gui radar LEFT
        else if (cmdLen > 3 && cmd[0]=='R' && cmd[1]=='2' && cmd[2]==':') {
            uart_write_bytes(RADAR2_UART, cmd + 3, cmdLen - 3);
            printf("\n>> Cmd fwd R2 only: %.*s\n", cmdLen - 3, cmd + 3);
        }
        // Khong prefix ΓåÆ gui ca 2
        else {
            uart_write_bytes(RADAR1_UART, cmd, cmdLen);
            uart_write_bytes(RADAR2_UART, cmd, cmdLen);
            printf("\n>> Cmd fwd both: %.*s\n", cmdLen, cmd);
        }
    }
}

// ================= SEND FRAME =================
static void sendRadarFrame(uint8_t *data, int len, uint8_t radar_id)
{
    /*
     * Packet: [0x01][seq][totalH][totalL][radar_id][payloadΓÇª]
     * totalLen = len + 2 (2 byte CRC16 append cuoi)
     * ΓåÆ Receiver se verify CRC truoc khi parse
     */
    uint16_t crc = crc16(data, len);
    uint16_t totalLen = len + 2;  // payload + CRC

    int     offset = 0;
    uint8_t seq    = 0;
    uint8_t pkt[250];

    // Tao buffer tam: data + CRC
    // (Gui toan bo data truoc, roi gui 2 byte CRC o chunk cuoi)
    while (offset < len) {
        int chunk = len - offset;
        if (chunk > 224) chunk = 224;

        pkt[0] = 0x01;
        pkt[1] = seq++;
        pkt[2] = (totalLen >> 8) & 0xFF;
        pkt[3] = totalLen & 0xFF;
        pkt[4] = radar_id;

        memcpy(pkt + 5, data + offset, chunk);
        int pktLen = chunk + 5;

        // Neu la chunk cuoi ΓåÆ append 2 byte CRC
        if (offset + chunk >= len) {
            pkt[pktLen]     = (crc >> 8) & 0xFF;
            pkt[pktLen + 1] = crc & 0xFF;
            pktLen += 2;
        }

        esp_now_send(receiverMAC, pkt, pktLen);
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
 * Gß╗ìi sau khi UART ─æ├ú sß║╡n s├áng. Chß╗¥ radar boot ~1ΓÇô2 gi├óy tr╞░ß╗¢c.
 *
 * L╞░u ├╜: AT+RESTORE kh├┤i phß╗Ñc factory ΓåÆ x├│a cß║Ñu h├¼nh c┼⌐.
 * Nß║┐u muß╗æn giß╗» lß║íi cß║Ñu h├¼nh kh├íc (VD: AT+SENS), bß╗Å d├▓ng RESTORE
 * v├á chß╗ë gß╗¡i nhß╗»ng lß╗çnh cß║ºn thay ─æß╗òi.
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

// ================= FRAME PARSER (d├╣ng chung) =================
/*
 * ─Éß╗ìc tß╗½ng byte tß╗½ UART v├á gh├⌐p th├ánh frame.
 * Header 8 byte: 01 02 03 04 05 06 07 08
 * Byte [8..11]  : packetLen (uint32_t LE)
 *
 * Trß║ú vß╗ü 1 khi ─æ├ú gß╗¡i xong 1 frame ─æß║ºy ─æß╗º, 0 nß║┐u ch╞░a.
 */
static int process_byte(uint8_t byte,
                         uint8_t  *buf,
                         int      *idx,
                         uart_port_t port,
                         uint8_t  radar_id)
{
    buf[(*idx)++] = byte;

    // B╞░ß╗¢c 1: Chß╗¥ ─æß╗º 8 byte header
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

    // B╞░ß╗¢c 2: Chß╗¥ ─æß╗º 12 byte ─æß╗â ─æß╗ìc packetLen
    if (*idx < 12) return 0;

    uint32_t pktLen;
    memcpy(&pktLen, &buf[8], 4);

    if (pktLen > RADAR_BUF_SIZE) {
        *idx = 0;
        return 0;
    }

    // B╞░ß╗¢c 3: Chß╗¥ ─æß╗º frame
    if (*idx < (int)pktLen) return 0;

    // Gß╗¡i frame qua ESP-NOW
    const char *zoneName = (radar_id == RADAR_ID_RIGHT) ? "RIGHT" : "LEFT";
    printf("\n[%s Frame] size=%lu\n", zoneName, pktLen);
    sendRadarFrame(buf, (int)pktLen, radar_id);

    // Xo├í phß║ºn ─æ├ú gß╗¡i, giß╗» lß║íi phß║ºn d╞░
    memmove(buf, buf + pktLen, *idx - pktLen);
    *idx -= pktLen;
    return 1;
}

// ================= TASK RADAR 1 (RIGHT) =================
static void radar1_task(void *pv)
{
    uint8_t rxChunk[256];
    uint64_t lastPing = 0;

    while (1) {
        uint64_t now = esp_timer_get_time() / 1000;
        if (now - lastPing > 2000) {
            lastPing = now;
            uint8_t pingPkt[] = {0x01, 0x00, 0x00, 0x00, RADAR_ID_RIGHT};
            esp_now_send(receiverMAC, pingPkt, sizeof(pingPkt));
            printf("[RIGHT] Ping...\n");
        }

        int len = uart_read_bytes(RADAR1_UART, rxChunk, sizeof(rxChunk), pdMS_TO_TICKS(10));
        for (int i = 0; i < len; i++)
            process_byte(rxChunk[i], radarBuf1, &radarIdx1, RADAR1_UART, RADAR_ID_RIGHT);

        if (radarIdx1 >= RADAR_BUF_SIZE) radarIdx1 = 0;
    }
}

// ================= TASK RADAR 2 (LEFT) =================
static void radar2_task(void *pv)
{
    uint8_t rxChunk[256];
    uint64_t lastPing = 0;

    while (1) {
        uint64_t now = esp_timer_get_time() / 1000;
        if (now - lastPing > 2000) {
            lastPing = now;
            uint8_t pingPkt[] = {0x01, 0x00, 0x00, 0x00, RADAR_ID_LEFT};
            esp_now_send(receiverMAC, pingPkt, sizeof(pingPkt));
            printf("[LEFT ] Ping...\n");
        }

        int len = uart_read_bytes(RADAR2_UART, rxChunk, sizeof(rxChunk), pdMS_TO_TICKS(10));
        for (int i = 0; i < len; i++)
            process_byte(rxChunk[i], radarBuf2, &radarIdx2, RADAR2_UART, RADAR_ID_LEFT);

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

    // Chß╗¥ radar boot xong rß╗ôi mß╗¢i gß╗¡i AT
    vTaskDelay(pdMS_TO_TICKS(2000));
    radar_at_config();

    xTaskCreate(radar1_task, "radar1_task", 4096, NULL, 5, NULL);
    xTaskCreate(radar2_task, "radar2_task", 4096, NULL, 5, NULL);
}

