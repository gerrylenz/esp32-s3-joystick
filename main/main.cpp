#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <cmath>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "driver/uart.h"
#include "esp_log.h"

#include "usb/usb_host.h"
#include "usb/cdc_acm_host.h"

#include "led_control.hpp"

#include "esp_adc/adc_oneshot.h"

#include <atomic>

// UART-Konfiguration
#define UART_NUM UART_NUM_0
#define UART0_TX_PIN 43
#define UART0_RX_PIN 44
#define BAUDRATE 115200
#define UART_LOG_BUF_SIZE 1024
#define UART_HOST_BUF_SIZE 2048
#define UART_EVENT_QUEUE_LENGTH 40

// Optional: UART0 hardware flow control (Host sendet nur, wenn ESP bereit ist)
// Verkabelung: ESP32 RTS → Host CTS (GND gemeinsam). Host-Software muss CTS respektieren.
// Auf 1 setzen und UART0_RTS_PIN an freien GPIO anpassen.
#define UART0_FLOW_CONTROL 0
#if UART0_FLOW_CONTROL
#define UART0_RTS_PIN 21              // ESP32 RTS-Ausgang → Host CTS-Eingang
#define UART0_CTS_PIN UART_PIN_NO_CHANGE // optional: Host RTS → ESP32 CTS (sonst NC)
#define UART0_RX_FLOW_CTRL_THRESH 64  // HW-FIFO-Füllstand, ab dem RTS aktiv wird
#endif

#define UART_LOG_NUM UART_NUM_1
#define UART_LOG_TX_PIN 17
#define UART_LOG_RX_PIN 18

// Queue-Definitionen
#define LINE_QUEUE_LENGTH 32
#define MAX_LINE_LENGTH 128
#define USB_MAX_CHUNK 128
#define USB_CDC_IN_BUFFER_SIZE 128
#define TX_QUEUE_SEND_TIMEOUT_MS 500

// Smoothieboard (LPC17xx) CDC-ACM — VID/PID ggf. anpassen
#define USB_TARGET_VID 0xFFFF
#define USB_TARGET_PID 0x0005

#define RX_QUEUE_LENGTH 128
#define RX_ITEM_SIZE 128

// USB CDC Device Handle. Schließen nur über claim_cdc_for_close(), damit
// Disconnect und TX-Fehler dasselbe Handle nicht zweimal schließen.
cdc_acm_dev_hdl_t cdc_dev = NULL;
static cdc_acm_dev_hdl_t cdc_claimed = NULL;
static portMUX_TYPE cdc_dev_mux = portMUX_INITIALIZER_UNLOCKED;
static volatile bool usb_in_host_callback = false;

// Struktur für empfangene UART/USB-Pakete
typedef struct
{
    uint8_t data[RX_ITEM_SIZE];
    size_t len;
} rx_packet_t;

//**************************************************************************
// ADC Konfiguration für Joystick (ADC2, Kanäle 2/3)
#define ADC_UNIT ADC_UNIT_2
#define joystick_X_Pin ADC_CHANNEL_2
#define joystick_Y_Pin ADC_CHANNEL_3

adc_channel_t channels[2] = {joystick_X_Pin, joystick_Y_Pin};
adc_oneshot_unit_handle_t adc_handle;
static int adc_raw[2];

//**************************************************************************
// Joystick & Jogging Variablen
#define JOYSTICK_DEADZONE_STOP 22    // Mitte pro Achse (Skala -100..100)
#define JOYSTICK_START_SAMPLES 4     // n×10 ms bestätigen vor erstem Jog
#define JOYSTICK_ADC_HALF_RANGE 1000
#define JOYSTICK_NOISE_MARGIN 15
#define JOYSTICK_X_NOISE_EXTRA 12    // X-Achse: oft mehr ADC-Offset
#define JOYSTICK_ADC_IDLE_MARGIN 6   // Counts über Rausch → gilt als Ruhe
#define JOYSTICK_ADC_START_EXTRA 55  // Counts über Rausch → Anfahren
#define JOYSTICK_CENTER_TRACK_SAMPLES 40 // ~400 ms Ruhe → Nullpunkt nachführen
#define JOYSTICK_TRAVEL_LEARN_MIN 350
#define JOYSTICK_CENTER_SAMPLES 64
#define JOYSTICK_MIN_FEEDRATE 1.0f
#define JOYSTICK_RESPONSE_EXPONENT 1.6f // >1 = langsamer/feiner bei kleinem Ausschlag
#define JOYSTICK_MOTION_SMOOTH_ALPHA 0.35f // 0..1, niedriger = weicher, träger
#define JOG_LOOKAHEAD_SEGMENTS 2.5f
#define JOG_MIN_STEP_MM 0.004f
#define JOYSTICK_DEFAULT_MAX_FEED 4000.0f // Fallback bis GRBL-Config geladen

// --- Stop-Befehl definieren ---
#define GRBL_STOP_CMD 0x85
static bool is_jogging_now = false;
static bool next_jog_true = false;
// Anzahl noch ausstehender ok auf Befehle, die diese Firmware selbst gesendet hat.
static std::atomic<int> local_ok_pending{0};
// Nach Jog-Cancel wegen ALARM/ERROR einmal $X hinterher, erst wenn 0x85 raus ist.
static std::atomic<bool> unlock_after_jog_cancel{false};

int x = 0, y = 0;
static int joystick_center_adc[2];
static int adc_filtered[2];
static int joystick_noise_adc[2] = {JOYSTICK_NOISE_MARGIN, JOYSTICK_NOISE_MARGIN};
static int joystick_range_adc_pos[2] = {JOYSTICK_ADC_HALF_RANGE, JOYSTICK_ADC_HALF_RANGE};
static int joystick_range_adc_neg[2] = {JOYSTICK_ADC_HALF_RANGE, JOYSTICK_ADC_HALF_RANGE};
static bool joystick_calibrated = false;
static float machine_max_feed_rate = JOYSTICK_DEFAULT_MAX_FEED;
static float motion_smooth_x = 0.0f;
static float motion_smooth_y = 0.0f;

// Joystick-Konfiguration
struct joystick_t
{
    bool xHardwareReversed;   // Some joysticks shows reversed values on their X axis.
    bool yHardwareReversed;   // Some joysticks shows reversed values on their Y axis.
} joystick;

// GRBL-Konfigurationspuffer
#define GRBL_MAX_CONFIG_LINES 128
#define GRBL_MAX_LINE_LEN 64
volatile bool grbl_config_active = false;
volatile bool grbl_config_done = false;

typedef struct
{
    char lines[GRBL_MAX_CONFIG_LINES][GRBL_MAX_LINE_LEN];
    int count;
} grbl_config_t;

grbl_config_t raw_cfg;

//**************************************************************************
// FreeRTOS Queues & Semaphoren
static SemaphoreHandle_t usb_tx_lock;
static SemaphoreHandle_t device_disconnected_sem;
static SemaphoreHandle_t ok_sem; // Signal für OK vom GRBL

static QueueHandle_t rx_uart_queue; // Event-Queue für UART
static QueueHandle_t rx_usb_queue;

typedef struct
{
    char data[MAX_LINE_LENGTH];
    size_t len;
    bool expect_ok; // true: die ok-Antwort bleibt beim Host unsichtbar
} tx_item_t;

static QueueHandle_t tx_usb_queue = NULL;
static QueueHandle_t control_queue; // für Stop/Notfall-Bytes

#define CONTROL_QUEUE_LENGTH 32

static volatile uint32_t ctrl_queue_drop_count;
static volatile uint32_t tx_queue_jog_drop_count;
static volatile uint32_t tx_queue_reset_timeout_count;
static volatile uint32_t rx_usb_queue_drop_count;
static volatile uint32_t uart_rx_overflow_count;

//******************************************************************************************
// Joystick Start/Stop Funktionen
static inline void start_jogging(void)
{
    is_jogging_now = true;
    next_jog_true = true;
}
static inline void stop_jogging(void)
{
    is_jogging_now = false;
    // false lassen: joy_task verwirft einen Jog, der den Stopp im ok-Warten überholt hat.
    next_jog_true = false;
    xSemaphoreTake(ok_sem, 0);
    xSemaphoreGive(ok_sem);
}

static void send_jog_cancel(void)
{
    xQueueReset(tx_usb_queue);

    uint8_t stop = GRBL_STOP_CMD;
    if (xQueueSend(control_queue, &stop, pdMS_TO_TICKS(TX_QUEUE_SEND_TIMEOUT_MS)) != pdPASS)
    {
        ctrl_queue_drop_count++;
        ESP_LOGW("jog", "control_queue voll, Jog-Cancel verworfen (#%" PRIu32 ")", ctrl_queue_drop_count);
    }
}

static inline void reset_motion_smoothing(void)
{
    motion_smooth_x = 0.0f;
    motion_smooth_y = 0.0f;
}

static inline void abort_jog_motion(void)
{
    stop_jogging();
    reset_motion_smoothing();
    send_jog_cancel();
}
//******************************************************************************************
// GRBL-Paar Parsing ($Index=Value)
static inline bool parse_grbl_pair(const char *line, int *index, float *value)
{
    if (line[0] != '$')
        return false;

    const char *eq = strchr(line, '=');
    if (!eq)
        return false;

    *index = atoi(line + 1);
    *value = atof(eq + 1);
    return true;
}
//*************************************************************************************
// GRBL Konfigurationszeile hinzufügen
static void grbl_config_add_line(grbl_config_t *cfg, const char *line)
{
    if (cfg->count >= GRBL_MAX_CONFIG_LINES)
        return;

    strncpy(cfg->lines[cfg->count], line, GRBL_MAX_LINE_LEN);
    cfg->lines[cfg->count][GRBL_MAX_LINE_LEN - 1] = 0;
    cfg->count++;
}

static float grbl_max_feed_from_config(const grbl_config_t *cfg)
{
    float max_feed = 0.0f;

    for (int i = 0; i < cfg->count; i++)
    {
        int idx;
        float val;

        if (parse_grbl_pair(cfg->lines[i], &idx, &val) && idx == 110)
            max_feed = val;
    }

    return max_feed;
}

static void apply_grbl_max_feed(const grbl_config_t *cfg)
{
    float feed = grbl_max_feed_from_config(cfg);
    if (feed > 0.0f)
        machine_max_feed_rate = feed;
}
//******************************************************************************************
// UART Initialisierung & Event Task
static void uart_init()
{
    uart_config_t cfg = {};
    cfg.baud_rate = BAUDRATE;
    cfg.data_bits = UART_DATA_8_BITS;
    cfg.parity = UART_PARITY_DISABLE;
    cfg.stop_bits = UART_STOP_BITS_1;
#if UART0_FLOW_CONTROL
#if (UART0_CTS_PIN != UART_PIN_NO_CHANGE)
    cfg.flow_ctrl = UART_HW_FLOWCTRL_CTS_RTS;
#else
    cfg.flow_ctrl = UART_HW_FLOWCTRL_RTS;
#endif
    cfg.rx_flow_ctrl_thresh = UART0_RX_FLOW_CTRL_THRESH;
#else
    cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
#endif
    cfg.source_clk = UART_SCLK_DEFAULT;

    uart_param_config(UART_LOG_NUM, &cfg);
    uart_set_pin(UART_LOG_NUM, UART_LOG_TX_PIN, UART_LOG_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_LOG_NUM, UART_LOG_BUF_SIZE, UART_LOG_BUF_SIZE, 0, NULL, 0);

    esp_err_t ret;
    ret = uart_param_config(UART_NUM, &cfg);
    if (ret != ESP_OK)
        ESP_LOGE("uart_init", "uart_param_config failed: %d", ret);
#if UART0_FLOW_CONTROL
    ret = uart_set_pin(UART_NUM, UART0_TX_PIN, UART0_RX_PIN, UART0_RTS_PIN, UART0_CTS_PIN);
    if (ret != ESP_OK)
        ESP_LOGE("uart_init", "uart_set_pin failed: %d", ret);
    ESP_LOGI("uart_init", "UART0 HW flow control: RTS=GPIO %d, thresh=%d",
             UART0_RTS_PIN, UART0_RX_FLOW_CTRL_THRESH);
#else
    ret = uart_set_pin(UART_NUM, UART0_TX_PIN, UART0_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK)
        ESP_LOGE("uart_init", "uart_set_pin failed: %d", ret);
#endif
    ret = uart_driver_install(UART_NUM, UART_HOST_BUF_SIZE, UART_HOST_BUF_SIZE,
                              UART_EVENT_QUEUE_LENGTH, &rx_uart_queue, 0);
    if (ret != ESP_OK)
        ESP_LOGE("uart_init", "uart_driver_install failed: %d", ret);
    else
        ESP_LOGD("uart_init", "uart_driver_install OK");
    if (rx_uart_queue == NULL)
    {
        ESP_LOGE("uart_init", "uart_queue == NULL after driver install!");
    }
}
// GRBL-Echtzeitbytes: sofort raus, nicht hinter G-Code in der TX-Queue.
static bool is_grbl_realtime(uint8_t b)
{
    switch (b)
    {
    case 0x18: // Ctrl-X Soft-Reset
    case '!':
    case '?':
    case '~':
    case 0x84: // Safety Door
    case 0x85: // Jog Cancel
    case 0x86: // Debug-Report, nur wenn GRBL mit DEBUG gebaut ist
        return true;
    default:
        break;
    }
    if (b >= 0x90 && b <= 0x94) // Feed-Override
        return true;
    if (b >= 0x95 && b <= 0x97) // Rapid-Override
        return true;
    if (b >= 0x99 && b <= 0x9E) // Spindle-Override
        return true;
    if (b == 0xA0 || b == 0xA1) // Coolant
        return true;
    return false;
}

static void enqueue_realtime(uint8_t b)
{
    if (xQueueSend(control_queue, &b, pdMS_TO_TICKS(TX_QUEUE_SEND_TIMEOUT_MS)) != pdPASS)
    {
        ctrl_queue_drop_count++;
        ESP_LOGW("uart_event_task", "control_queue voll, Echtzeit 0x%02X verworfen (#%" PRIu32 ")",
                 b, ctrl_queue_drop_count);
    }
}

static void enqueue_host_chunk(const uint8_t *data, size_t len)
{
    // Echtzeitbytes zuerst, damit ein voller TX-Puffer Feed-Hold nicht aufhält.
    for (size_t i = 0; i < len; i++)
    {
        if (is_grbl_realtime(data[i]))
            enqueue_realtime(data[i]);
    }

    size_t i = 0;
    while (i < len)
    {
        if (is_grbl_realtime(data[i]))
        {
            i++;
            continue;
        }

        tx_item_t item = {};
        while (i < len && item.len < sizeof(item.data) && !is_grbl_realtime(data[i]))
            item.data[item.len++] = (char)data[i++];

        if (item.len == 0)
            continue;
        if (xQueueSend(tx_usb_queue, &item, portMAX_DELAY) != pdPASS)
            ESP_LOGE("uart_event_task", "tx_usb_queue nicht verfügbar");
    }
}

//*************************************************************************************
// UART Event Task
static void uart_event_task(void *pvParameters)
{
    uart_event_t event;

    for (;;)
    {
        if (xQueueReceive(rx_uart_queue, (void *)&event, (TickType_t)portMAX_DELAY))
        {
            switch (event.type)
            {
            case UART_DATA:
            {
                ESP_LOGD("uart_event_task", "[UART DATA]: %d", event.size);
                // Erst den ganzen Event lesen. Sonst bleibt der Rest im Treiber,
                // und Echtzeitbytes am Ende warten hinter einem vollen TX-Puffer.
                size_t remaining = event.size;
                uint8_t chunk[UART_HOST_BUF_SIZE];
                size_t filled = 0;
                while (remaining > 0 && filled < sizeof(chunk))
                {
                    size_t to_read = remaining;
                    if (to_read > sizeof(chunk) - filled)
                        to_read = sizeof(chunk) - filled;

                    int r = uart_read_bytes(UART_NUM, chunk + filled, to_read, portMAX_DELAY);
                    if (r <= 0)
                        break;
                    filled += (size_t)r;
                    remaining -= (size_t)r;
                }
                if (filled > 0)
                    enqueue_host_chunk(chunk, filled);
                while (remaining > 0)
                {
                    size_t to_read = remaining;
                    if (to_read > sizeof(chunk))
                        to_read = sizeof(chunk);
                    int r = uart_read_bytes(UART_NUM, chunk, to_read, portMAX_DELAY);
                    if (r <= 0)
                        break;
                    remaining -= (size_t)r;
                    enqueue_host_chunk(chunk, (size_t)r);
                }
                break;
            }
            case UART_BREAK:
                ESP_LOGD("uart_event_task", "UART RX break detected");
                break;

            case UART_BUFFER_FULL:
                uart_rx_overflow_count++;
                ESP_LOGW("uart_event_task", "UART RX buffer full (#%" PRIu32 ")", uart_rx_overflow_count);
                uart_flush_input(UART_NUM);
                break;

            case UART_FIFO_OVF:
                uart_rx_overflow_count++;
                ESP_LOGW("uart_event_task", "UART RX FIFO overflow (#%" PRIu32 ")", uart_rx_overflow_count);
                uart_flush_input(UART_NUM);
                break;

            case UART_FRAME_ERR:
                ESP_LOGD("uart_event_task", "UART frame error");
                break;

            case UART_PARITY_ERR:
                ESP_LOGD("uart_event_task", "UART parity error");
                break;

            case UART_PATTERN_DET:
                ESP_LOGD("uart_event_task", "UART pattern detected");
                break;

            default:
                ESP_LOGD("uart_event_task", "Unhandled UART event: %d", event.type);
                break;
            }
        }
    }
    vTaskDelete(NULL);
}
//******************************************************************************************
// USB RX Handling
static bool handle_usb_rx(const uint8_t *data, size_t len, void *arg)
{
    if (len == 0)
        return true;

    rx_packet_t pkt;
    if (len > RX_ITEM_SIZE)
        len = RX_ITEM_SIZE;
    memcpy(pkt.data, data, len);
    pkt.len = len;

    if (xQueueSendFromISR(rx_usb_queue, &pkt, NULL) != pdPASS)
        rx_usb_queue_drop_count++;

    return true;
}

static bool line_is_grbl_ok(const char *line)
{
    return line[0] == 'o' && line[1] == 'k' &&
           (line[2] == '\r' || line[2] == '\n' || line[2] == '\0');
}

static bool line_is_grbl_fault(const char *line)
{
    return strstr(line, "ALARM:") || strstr(line, "alarm:") ||
           strstr(line, "ERROR:") || strstr(line, "error:");
}

// true: dieses ok gehört zu $J/$X von uns und geht nicht an den Host.
static bool take_local_ok(void)
{
    int prev = local_ok_pending.fetch_sub(1, std::memory_order_acq_rel);
    if (prev > 0)
        return true;
    local_ok_pending.store(0, std::memory_order_release);
    return false;
}

static void queue_expect_ok(const char *cmd)
{
    tx_item_t item = {};
    size_t n = strlen(cmd);
    if (n > sizeof(item.data))
        n = sizeof(item.data);
    memcpy(item.data, cmd, n);
    item.len = n;
    item.expect_ok = true;
    if (xQueueSend(tx_usb_queue, &item, pdMS_TO_TICKS(TX_QUEUE_SEND_TIMEOUT_MS)) != pdPASS)
    {
        tx_queue_reset_timeout_count++;
        ESP_LOGW("usb_rx", "tx_usb_queue voll, Befehl verworfen (#%" PRIu32 ")",
                 tx_queue_reset_timeout_count);
    }
}

//*************************************************************************************
void process_usb_rx_task(void *pvParameters)
{
    rx_packet_t pkt;
    static char line_buf[256];
    static size_t line_len = 0;

    while (1)
    {
        if (xQueueReceive(rx_usb_queue, &pkt, portMAX_DELAY) == pdPASS)
        {
            for (size_t i = 0; i < pkt.len; i++)
            {
                char c = pkt.data[i];

                if (line_len < sizeof(line_buf) - 1)
                    line_buf[line_len++] = c;
                else
                    line_len = 0;

                if (c == '\n')
                {
                    line_buf[line_len] = '\0';
                    bool is_ok = line_is_grbl_ok(line_buf);

                    // --------------------------
                    // 0) Fehlerprüfung (ALARM/ERROR) während Jog
                    // --------------------------
                    if (is_jogging_now && line_is_grbl_fault(line_buf))
                    {
                        // $X erst in send_control_byte, nach dem Queue-Reset von 0x85.
                        unlock_after_jog_cancel.store(true, std::memory_order_release);
                        abort_jog_motion();
                        ESP_LOGW("usb_rx", "GRBL Fehler erkannt: %s", line_buf);

                        uart_write_bytes(UART_NUM, line_buf, line_len);
                        line_len = 0;
                        continue;
                    }

                    // --------------------------
                    // 1) Befinden wir uns im GRBL-Konfig-Mode?
                    // --------------------------
                    if (grbl_config_active)
                    {
                        if (is_ok)
                        {
                            grbl_config_active = false;
                            grbl_config_done = true;
                            line_len = 0;
                            continue;
                        }

                        if (line_buf[0] == '$')
                        {
                            grbl_config_add_line(&raw_cfg, line_buf);
                        }

                        line_len = 0;
                        continue;
                    }

                    // ok nur schlucken, wenn es die Antwort auf $J oder $X von uns ist.
                    if (is_ok && take_local_ok())
                    {
                        xSemaphoreGive(ok_sem);
                        line_len = 0;
                        continue;
                    }

                    // --------------------------
                    // 4) Zeilen, die mit '$' beginnen
                    // --------------------------
                    if (line_buf[0] == '$')
                    {
                        grbl_config_add_line(&raw_cfg, line_buf);
                        apply_grbl_max_feed(&raw_cfg);

                        ESP_LOGD("usb_rx", "GRBL $-Zeile empfangen: %s", line_buf);
                        ESP_LOGD("grbl", "FeedMax: X=%.3f", machine_max_feed_rate);
                    }

                    // --------------------------
                    // 5) Alle anderen Daten an UART
                    // --------------------------
                    uart_write_bytes(UART_NUM, line_buf, line_len);
                    line_len = 0;
                }
            }
        }
    }
}
//******************************************************************************************
// Liefert das Handle genau einmal. hdl == NULL nimmt das aktuelle cdc_dev.
static cdc_acm_dev_hdl_t claim_cdc_for_close(cdc_acm_dev_hdl_t hdl)
{
    portENTER_CRITICAL(&cdc_dev_mux);
    if (hdl == NULL)
        hdl = cdc_dev;

    cdc_acm_dev_hdl_t claimed = NULL;
    if (hdl != NULL && hdl != cdc_claimed)
    {
        claimed = hdl;
        cdc_claimed = hdl;
        if (cdc_dev == hdl)
            cdc_dev = NULL;
    }
    else if (cdc_dev == hdl)
    {
        cdc_dev = NULL;
    }
    portEXIT_CRITICAL(&cdc_dev_mux);
    return claimed;
}

static void usb_prepare_open(void)
{
    portENTER_CRITICAL(&cdc_dev_mux);
    if (cdc_dev == NULL)
        cdc_claimed = NULL;
    portEXIT_CRITICAL(&cdc_dev_mux);
}

static bool usb_publish_opened(cdc_acm_dev_hdl_t dev)
{
    bool ok = false;
    portENTER_CRITICAL(&cdc_dev_mux);
    if (dev != NULL && cdc_claimed == dev)
    {
        cdc_dev = NULL;
    }
    else
    {
        cdc_dev = dev;
        cdc_claimed = NULL;
        ok = dev != NULL;
    }
    portEXIT_CRITICAL(&cdc_dev_mux);
    return ok;
}

static void usb_close_handle(cdc_acm_dev_hdl_t hdl)
{
    bool locked = false;
    if (!usb_in_host_callback && usb_tx_lock &&
        xSemaphoreTake(usb_tx_lock, portMAX_DELAY) == pdTRUE)
        locked = true;

    cdc_acm_dev_hdl_t dev = claim_cdc_for_close(hdl);

    if (locked && usb_tx_lock)
        xSemaphoreGive(usb_tx_lock);

    if (dev)
        cdc_acm_host_close(dev);
}

static void usb_close_device(void)
{
    usb_close_handle(NULL);
}

static esp_err_t usb_tx(const uint8_t *data, size_t len, uint32_t timeout_ms)
{
    if (usb_tx_lock == NULL || xSemaphoreTake(usb_tx_lock, portMAX_DELAY) != pdTRUE)
        return ESP_ERR_INVALID_STATE;

    portENTER_CRITICAL(&cdc_dev_mux);
    cdc_acm_dev_hdl_t dev = cdc_dev;
    portEXIT_CRITICAL(&cdc_dev_mux);

    esp_err_t err = ESP_ERR_INVALID_STATE;
    if (dev)
        err = cdc_acm_host_data_tx_blocking(dev, data, len, timeout_ms);

    xSemaphoreGive(usb_tx_lock);
    return err;
}

static void recover_usb_link(const char *context, esp_err_t err)
{
    ESP_LOGW("usb_recovery", "%s: %s", context, esp_err_to_name(err));

    stop_jogging();
    local_ok_pending.store(0, std::memory_order_release);
    unlock_after_jog_cancel.store(false, std::memory_order_release);
    if (tx_usb_queue)
        xQueueReset(tx_usb_queue);

    usb_close_device();
    xSemaphoreGive(device_disconnected_sem);
}

static bool send_control_byte(uint8_t ctrl)
{
    if (cdc_dev)
    {
        esp_err_t err = usb_tx(&ctrl, 1, 100);
        if (err != ESP_OK)
        {
            recover_usb_link("CTRL send", err);
            return false;
        }
    }
    if (ctrl == GRBL_STOP_CMD && tx_usb_queue)
        xQueueReset(tx_usb_queue);
    if (ctrl == GRBL_STOP_CMD &&
        unlock_after_jog_cancel.exchange(false, std::memory_order_acq_rel) &&
        cdc_dev)
        queue_expect_ok("$X\n");
    ESP_LOGD("queue2grbl", "Echtzeit 0x%02X", ctrl);
    return true;
}

//******************************************************************************************
// Queue -> USB Task
static void queue2grbl(void *arg)
{
    tx_item_t item;
    uint8_t ctrl;

    while (true)
    {
        if (xQueueReceive(control_queue, &ctrl, 0) == pdPASS)
        {
            send_control_byte(ctrl);
            continue;
        }

        if (xQueueReceive(tx_usb_queue, &item, pdMS_TO_TICKS(5)) != pdPASS)
            continue;

        if (item.len == 0)
            continue;

        if (xQueueReceive(control_queue, &ctrl, 0) == pdPASS)
        {
            if (!send_control_byte(ctrl) || ctrl == GRBL_STOP_CMD)
                continue;
        }

        if (!cdc_dev)
            continue;

        if (item.len < sizeof(item.data))
        {
            char tmp[MAX_LINE_LENGTH + 1];
            memcpy(tmp, item.data, item.len);
            tmp[item.len] = 0;
            ESP_LOGD("queue2grbl", "%s", tmp);
        }

        esp_err_t err = usb_tx((uint8_t *)item.data, item.len, 1000);
        if (err != ESP_OK)
            recover_usb_link("TX send", err);
        else if (item.expect_ok)
            local_ok_pending.fetch_add(1, std::memory_order_release);
    }
}
//*************************************************************************************
// USB Events
static void handle_event(const cdc_acm_host_dev_event_data_t *event, void *user_ctx)
{
    usb_in_host_callback = true;
    switch (event->type)
    {
    case CDC_ACM_HOST_ERROR:
        ESP_LOGE("handle_event", "CDC-ACM error has occurred, err_no = %i", event->data.error);
        recover_usb_link("CDC-ACM error", ESP_FAIL);
        break;
    case CDC_ACM_HOST_DEVICE_DISCONNECTED:
        ESP_LOGD("handle_event", "Device suddenly disconnected");
        stop_jogging();
        local_ok_pending.store(0, std::memory_order_release);
        unlock_after_jog_cancel.store(false, std::memory_order_release);
        grbl_config_done = false;
        if (tx_usb_queue)
            xQueueReset(tx_usb_queue);
        if (rx_usb_queue)
            xQueueReset(rx_usb_queue);
        usb_close_handle(event->data.cdc_hdl);
        xSemaphoreGive(device_disconnected_sem);
        break;
    case CDC_ACM_HOST_SERIAL_STATE:
        // ESP_LOGD("handle_event", "Serial state notif 0x%04X", event->data.serial_state.val);
        break;
    case CDC_ACM_HOST_NETWORK_CONNECTION:
    default:
        ESP_LOGD("handle_event", "Unsupported CDC event: %i", event->type);
        break;
    }
    usb_in_host_callback = false;
}

//*************************************************************************************
// USB Host Task
static void usb_host_task(void *arg)
{
    while (true)
    {
        uint32_t event_flags;
        usb_host_lib_handle_events(portMAX_DELAY, &event_flags);
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS)
        {
            ESP_ERROR_CHECK(usb_host_device_free_all());
        }
    }
}

//*************************************************************************************
// ADC Initialisierung
void adc_init(adc_channel_t *channel, uint8_t numChannels)
{
    adc_oneshot_unit_init_cfg_t unitConfig = {
        .unit_id = ADC_UNIT,
        .clk_src = (adc_oneshot_clk_src_t)0, // 0 = Treiber-Default
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    adc_oneshot_new_unit(&unitConfig, &adc_handle);

    adc_oneshot_chan_cfg_t channelConfig = {
        .atten = ADC_ATTEN_DB_12,    // 150mV - 2450mV
        .bitwidth = ADC_BITWIDTH_12, // resolution 12 bit
    };

    for (int i = 0; i < numChannels; i++)
    {
        adc_oneshot_config_channel(adc_handle, channel[i], &channelConfig);
    }

}
//*************************************************************************************
static void calibrate_joystick_center(void)
{
    ESP_LOGD("joy", "Kalibriere Nullpunkt — Joystick in Mittelstellung lassen...");
    vTaskDelay(pdMS_TO_TICKS(300));

    int64_t sum[2] = {0, 0};
    int min_adc[2] = {4095, 4095};
    int max_adc[2] = {0, 0};

    for (int i = 0; i < JOYSTICK_CENTER_SAMPLES; i++)
    {
        adc_oneshot_read(adc_handle, channels[0], &adc_raw[0]);
        adc_oneshot_read(adc_handle, channels[1], &adc_raw[1]);
        sum[0] += adc_raw[0];
        sum[1] += adc_raw[1];
        if (adc_raw[0] < min_adc[0])
            min_adc[0] = adc_raw[0];
        if (adc_raw[0] > max_adc[0])
            max_adc[0] = adc_raw[0];
        if (adc_raw[1] < min_adc[1])
            min_adc[1] = adc_raw[1];
        if (adc_raw[1] > max_adc[1])
            max_adc[1] = adc_raw[1];
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    joystick_center_adc[0] = (int)(sum[0] / JOYSTICK_CENTER_SAMPLES);
    joystick_center_adc[1] = (int)(sum[1] / JOYSTICK_CENTER_SAMPLES);
    adc_filtered[0] = joystick_center_adc[0];
    adc_filtered[1] = joystick_center_adc[1];
    for (int axis = 0; axis < 2; axis++)
    {
        joystick_range_adc_pos[axis] = JOYSTICK_ADC_HALF_RANGE;
        joystick_range_adc_neg[axis] = JOYSTICK_ADC_HALF_RANGE;
    }

    for (int axis = 0; axis < 2; axis++)
    {
        int spread = (max_adc[axis] - min_adc[axis]) / 2 + JOYSTICK_NOISE_MARGIN;
        if (spread < JOYSTICK_NOISE_MARGIN)
            spread = JOYSTICK_NOISE_MARGIN;
        if (spread > 45)
            spread = 45;
        if (axis == 0)
            spread += JOYSTICK_X_NOISE_EXTRA;
        joystick_noise_adc[axis] = spread;
    }

    joystick_calibrated = true;

    ESP_LOGD("joy", "Nullpunkt ADC X=%d Y=%d, Rausch X=%d Y=%d, Range +X=%d -X=%d +Y=%d -Y=%d",
             joystick_center_adc[0], joystick_center_adc[1],
             joystick_noise_adc[0], joystick_noise_adc[1],
             joystick_range_adc_pos[0], joystick_range_adc_neg[0],
             joystick_range_adc_pos[1], joystick_range_adc_neg[1]);
}

static int adc_abs_delta(int axis)
{
    int delta = adc_filtered[axis] - joystick_center_adc[axis];
    return delta > 0 ? delta : -delta;
}

static bool adc_axis_idle(int axis)
{
    return adc_abs_delta(axis) <= joystick_noise_adc[axis] + JOYSTICK_ADC_IDLE_MARGIN;
}

static bool adc_axis_start(int axis)
{
    return adc_abs_delta(axis) > joystick_noise_adc[axis] + JOYSTICK_ADC_START_EXTRA;
}

static bool joystick_fully_idle(void)
{
    return adc_axis_idle(0) && adc_axis_idle(1);
}

static bool joystick_start_intent(void)
{
    return adc_axis_start(0) || adc_axis_start(1);
}

static void track_joystick_center_idle(int *idle_samples)
{
    if (!joystick_fully_idle())
    {
        *idle_samples = 0;
        return;
    }

    (*idle_samples)++;
    if (*idle_samples < JOYSTICK_CENTER_TRACK_SAMPLES)
        return;

    joystick_center_adc[0] = (joystick_center_adc[0] * 31 + adc_filtered[0]) / 32;
    joystick_center_adc[1] = (joystick_center_adc[1] * 31 + adc_filtered[1]) / 32;
    *idle_samples = JOYSTICK_CENTER_TRACK_SAMPLES;
}

static void learn_joystick_travel(int filtered, int center, int axis)
{
    int delta = filtered - center;
    if (delta == 0)
        return;

    int abs_delta = (delta > 0) ? delta : -delta;
    if (abs_delta < JOYSTICK_TRAVEL_LEARN_MIN)
        return;

    int learned = (abs_delta * 11) / 10;
    if (learned > 2200)
        return;

    if (delta > 0)
    {
        if (learned > joystick_range_adc_pos[axis])
            joystick_range_adc_pos[axis] = learned;
    }
    else if (learned > joystick_range_adc_neg[axis])
    {
        joystick_range_adc_neg[axis] = learned;
    }
}

static int adc_to_axis(int filtered, int center, int axis)
{
    int delta = filtered - center;
    int abs_delta = delta > 0 ? delta : -delta;
    if (abs_delta <= joystick_noise_adc[axis])
        return 0;

    learn_joystick_travel(filtered, center, axis);

    int range = (delta > 0) ? joystick_range_adc_pos[axis] : joystick_range_adc_neg[axis];
    if (range <= 0)
        range = JOYSTICK_ADC_HALF_RANGE;

    delta = (delta * 100) / range;
    if (delta > 100)
        delta = 100;
    if (delta < -100)
        delta = -100;
    return delta;
}

static void adc_read(void)
{
    int raw[2];

    adc_oneshot_read(adc_handle, channels[0], &raw[0]);
    adc_oneshot_read(adc_handle, channels[1], &raw[1]);

    adc_filtered[0] = (adc_filtered[0] * 3 + raw[0]) / 4;
    adc_filtered[1] = (adc_filtered[1] * 3 + raw[1]) / 4;

    if (!joystick_calibrated)
    {
        x = 0;
        y = 0;
        return;
    }

    x = adc_to_axis(adc_filtered[0], joystick_center_adc[0], 0);
    y = adc_to_axis(adc_filtered[1], joystick_center_adc[1], 1);
}
//******************************************************************************************
// USB Connected Callback
static bool on_usb_connected(void)
{
    ESP_LOGD("usb", "Starte GRBL Config Abfrage");

    grbl_config_active = true;
    grbl_config_done = false;

    memset(&raw_cfg, 0, sizeof(raw_cfg)); // Buffer leeren

    // "$$" direkt senden
    esp_err_t err = usb_tx((const uint8_t *)"$$\n", 3, 1000);
    if (err != ESP_OK)
    {
        ESP_LOGW("on_usb_connected", "Sendefehler $$: %s", esp_err_to_name(err));
        grbl_config_active = false;
        return false;
    }

    // Warten bis GRBL alles geschickt hat
    int timeout_ms = 2000;
    while (!grbl_config_done && timeout_ms > 0)
    {
        vTaskDelay(pdMS_TO_TICKS(10));
        timeout_ms -= 10;
    }

    if (!grbl_config_done)
    {
        ESP_LOGW("grbl", "GRBL Konfiguration TIMEOUT");
        grbl_config_active = false;
        return false;
    }

    apply_grbl_max_feed(&raw_cfg);

    ESP_LOGD("grbl", "Konfiguration erfolgreich geladen");
    ESP_LOGD("grbl", "FeedMax X=%.3f", machine_max_feed_rate);
    return true;
}
//******************************************************************************************
// Joystick Normalisierung & Quantisierung
static void configure_joy(void)
{
    joystick.xHardwareReversed = false;
    joystick.yHardwareReversed = true;

    machine_max_feed_rate = JOYSTICK_DEFAULT_MAX_FEED;
    reset_motion_smoothing();
    calibrate_joystick_center();
    adc_read();
}
//******************************************************************************************
// Totzone abziehen und Restbereich 0..100 neu skalieren (kein Sprung an der Totzone)
static int axis_after_deadzone(int v)
{
    if (v > -JOYSTICK_DEADZONE_STOP && v < JOYSTICK_DEADZONE_STOP)
        return 0;

    int sign = (v > 0) ? 1 : -1;
    int abs_v = (v > 0) ? v : -v;
    int remapped = (abs_v - JOYSTICK_DEADZONE_STOP) * 100 / (100 - JOYSTICK_DEADZONE_STOP);
    if (remapped > 100)
        remapped = 100;
    return sign * remapped;
}

// Exponentielle Kennlinie: kleine Ausschläge → deutlich langsamere Bewegung
static float apply_response_curve(float norm)
{
    if (norm == 0.0f)
        return 0.0f;

    float sign = (norm > 0.0f) ? 1.0f : -1.0f;
    float magnitude = powf(fabsf(norm), JOYSTICK_RESPONSE_EXPONENT);
    return sign * magnitude;
}

// IIR-Glättung der Bewegungsachsen (weniger Ruckeln bei feinen Korrekturen)
static float smooth_motion_axis(float target, float *state)
{
    if (fabsf(target) < 0.001f)
    {
        *state *= (1.0f - JOYSTICK_MOTION_SMOOTH_ALPHA);
        if (fabsf(*state) < 0.001f)
            *state = 0.0f;
        return *state;
    }

    *state = *state * (1.0f - JOYSTICK_MOTION_SMOOTH_ALPHA) + target * JOYSTICK_MOTION_SMOOTH_ALPHA;
    return *state;
}

static void read_joystick_axes(int *lx, int *ly)
{
    adc_read();
    *lx = joystick.xHardwareReversed ? -x : x;
    *ly = joystick.yHardwareReversed ? -y : y;
}

static bool joystick_motion_axes(int lx, int ly, float *sx, float *sy)
{
    int mx = axis_after_deadzone(lx);
    int my = axis_after_deadzone(ly);

    if (mx != 0 && !adc_axis_start(0))
        mx = 0;
    if (my != 0 && !adc_axis_start(1))
        my = 0;

    float tx = apply_response_curve((float)mx / 100.0f);
    float ty = apply_response_curve((float)my / 100.0f);

    *sx = smooth_motion_axis(tx, &motion_smooth_x);
    *sy = smooth_motion_axis(ty, &motion_smooth_y);
    return fabsf(*sx) > 0.001f || fabsf(*sy) > 0.001f;
}
//******************************************************************************************
// Joystick Task
static void joy_task(void *arg)
{
    const TickType_t JOG_INTERVAL = pdMS_TO_TICKS(10);
    const float LOOKAHEAD = JOG_LOOKAHEAD_SEGMENTS;

    TickType_t last_wake = xTaskGetTickCount();
    const float interval_s = (float)JOG_INTERVAL / (float)configTICK_RATE_HZ;
    int start_confirm = 0;
    int idle_track_samples = 0;

    while (true)
    {
        vTaskDelayUntil(&last_wake, JOG_INTERVAL);

        int lx = 0;
        int ly = 0;
        read_joystick_axes(&lx, &ly);
        track_joystick_center_idle(&idle_track_samples);

        if (cdc_dev == NULL)
        {
            start_confirm = 0;
            idle_track_samples = 0;
            if (is_jogging_now)
                stop_jogging();
            continue;
        }

        if (joystick_fully_idle())
        {
            start_confirm = 0;
            if (is_jogging_now)
                abort_jog_motion();
            continue;
        }

        if (!is_jogging_now)
        {
            if (!joystick_start_intent())
            {
                start_confirm = 0;
                continue;
            }
            start_confirm++;
            if (start_confirm < JOYSTICK_START_SAMPLES)
                continue;
        }

        float sx = 0.0f;
        float sy = 0.0f;
        if (!joystick_motion_axes(lx, ly, &sx, &sy))
        {
            if (is_jogging_now)
                abort_jog_motion();
            continue;
        }

        float max_mm_s = machine_max_feed_rate / 60.0f;
        float vx = sx * max_mm_s;
        float vy = sy * max_mm_s;

        float cmd_x = vx * interval_s * LOOKAHEAD;
        float cmd_y = vy * interval_s * LOOKAHEAD;

        if (fabsf(cmd_x) < JOG_MIN_STEP_MM && fabsf(cmd_y) < JOG_MIN_STEP_MM)
            continue;

        float F = fmaxf(fabsf(vx), fabsf(vy)) * 60.0f;
        if (F < JOYSTICK_MIN_FEEDRATE)
            continue;
        if (F > machine_max_feed_rate)
            F = machine_max_feed_rate;

        start_jogging();
        if (!next_jog_true)
            continue;

        if (xSemaphoreTake(ok_sem, (TickType_t)100) != pdTRUE)
            continue;
        if (!next_jog_true || !is_jogging_now)
        {
            xSemaphoreGive(ok_sem);
            continue;
        }

        read_joystick_axes(&lx, &ly);
        if (!joystick_motion_axes(lx, ly, &sx, &sy))
        {
            xSemaphoreGive(ok_sem);
            abort_jog_motion();
            continue;
        }

        vx = sx * max_mm_s;
        vy = sy * max_mm_s;
        cmd_x = vx * interval_s * LOOKAHEAD;
        cmd_y = vy * interval_s * LOOKAHEAD;
        if (fabsf(cmd_x) < JOG_MIN_STEP_MM && fabsf(cmd_y) < JOG_MIN_STEP_MM)
        {
            xSemaphoreGive(ok_sem);
            continue;
        }
        F = fmaxf(fabsf(vx), fabsf(vy)) * 60.0f;
        if (F < JOYSTICK_MIN_FEEDRATE)
        {
            xSemaphoreGive(ok_sem);
            continue;
        }
        if (F > machine_max_feed_rate)
            F = machine_max_feed_rate;

        char cmd[128];
        int n = snprintf(cmd, sizeof(cmd),
                         "$J=G91 X%.3f Y%.3f F%.0f\n",
                         cmd_x, cmd_y, F);

        if (n <= 0 || (size_t)n > MAX_LINE_LENGTH)
        {
            xSemaphoreGive(ok_sem);
            continue;
        }

        tx_item_t item = {};
        item.len = (size_t)n;
        item.expect_ok = true;
        memcpy(item.data, cmd, item.len);

        if (xQueueSend(tx_usb_queue, &item, 0) != pdPASS)
        {
            tx_queue_jog_drop_count++;
            ESP_LOGW("joy_task", "tx_usb_queue voll, Jog verworfen (#%" PRIu32 ")", tx_queue_jog_drop_count);
            xSemaphoreGive(ok_sem);
            continue;
        }
    }
}
//******************************************************************************************
// USB Connect Loop
static void usb_connect_loop(void *arg)
{
    const cdc_acm_host_device_config_t dev_cfg = {
        .connection_timeout_ms = 5000,
        .out_buffer_size = USB_MAX_CHUNK,
        .in_buffer_size = USB_CDC_IN_BUFFER_SIZE,
        .event_cb = handle_event,
        .data_cb = handle_usb_rx,
        .user_arg = NULL};

    while (true)
    {
        ESP_LOGD("usb_connect_loop", "Versuche USB-Gerät zu öffnen...");
        usb_prepare_open();
        cdc_acm_dev_hdl_t opened = NULL;
        esp_err_t err = cdc_acm_host_open(USB_TARGET_VID, USB_TARGET_PID, 0, &dev_cfg, &opened);

        if (err != ESP_OK)
        {
            // Gerät nicht da, warten und erneut versuchen
            ESP_LOGD("usb_connect_loop", "Kein Gerät, retry in 500ms");
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }
        if (!usb_publish_opened(opened))
        {
            ESP_LOGW("usb_connect_loop", "USB-Gerät direkt nach Open getrennt");
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        cdc_acm_host_desc_print(opened);
        vTaskDelay(pdMS_TO_TICKS(100));

        cdc_acm_line_coding_t line_coding;
        line_coding.dwDTERate = 115200;
        line_coding.bDataBits = 8;
        line_coding.bParityType = 0;
        line_coding.bCharFormat = 0;
        ESP_ERROR_CHECK(cdc_acm_host_line_coding_set(opened, &line_coding));
        ESP_ERROR_CHECK(cdc_acm_host_line_coding_get(opened, &line_coding));
        ESP_LOGD("usb_connect_loop", "Line Get: Rate: %" PRIu32 ", Stop bits: %" PRIu8 ", Parity: %" PRIu8 ", Databits: %" PRIu8 "",
                 line_coding.dwDTERate, line_coding.bCharFormat, line_coding.bParityType, line_coding.bDataBits);

        ESP_ERROR_CHECK(cdc_acm_host_set_control_line_state(opened, true, false));

        ESP_LOGD("usb_connect_loop", "USB-Gerät verbunden");
        led_set_color(0, 0, 255); // Blau = Gerät verbunden

        if (!on_usb_connected())
        {
            led_set_color(255, 0, 0);
            usb_close_device();
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        // Control-Line-Reset für GRBL-Gerät
        cdc_acm_host_set_control_line_state(opened, true, true);
        vTaskDelay(pdMS_TO_TICKS(100));
        cdc_acm_host_set_control_line_state(opened, true, false);
        vTaskDelay(pdMS_TO_TICKS(500));

        // Veraltete Disconnect-Signale verwerfen. Ein echtes Ereignis danach
        // bleibt in der Semaphore bzw. an cdc_dev == NULL sichtbar.
        while (xSemaphoreTake(device_disconnected_sem, 0) == pdTRUE)
        {
        }
        if (cdc_dev != NULL)
            xSemaphoreTake(device_disconnected_sem, portMAX_DELAY);

        stop_jogging();
        local_ok_pending.store(0, std::memory_order_release);
        unlock_after_jog_cancel.store(false, std::memory_order_release);
        grbl_config_done = false;
        if (tx_usb_queue)
            xQueueReset(tx_usb_queue);
        led_set_color(255, 0, 0); // Rot = Gerät getrennt
        ESP_LOGD("usb_connect_loop", "USB-Gerät getrennt — Reconnect in 500ms");
        usb_close_device();
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
//******************************************************************************************
// Main
extern "C" void app_main()
{
    uart_init();

    led_strip_init();
    adc_init(channels, 2);
    configure_joy();

    // -----------------------
    // Queues & Semaphoren
    ok_sem = xSemaphoreCreateBinary();
    xSemaphoreGive(ok_sem);

    control_queue = xQueueCreate(CONTROL_QUEUE_LENGTH, sizeof(uint8_t));
    assert(control_queue);

    rx_usb_queue = xQueueCreate(RX_QUEUE_LENGTH, sizeof(rx_packet_t));
    assert(rx_usb_queue);

    tx_usb_queue = xQueueCreate(LINE_QUEUE_LENGTH, sizeof(tx_item_t));
    assert(tx_usb_queue);

    usb_tx_lock = xSemaphoreCreateMutex();
    assert(usb_tx_lock);

    device_disconnected_sem = xSemaphoreCreateBinary();
    assert(device_disconnected_sem);

    // -----------------------
    // USB Host Init
    const usb_host_config_t host_config = {
        .skip_phy_setup = false,
        .root_port_unpowered = false,
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
        .enum_filter_cb = NULL,
        .fifo_settings_custom = {
            .nptx_fifo_lines = 0,
            .ptx_fifo_lines = 0,
            .rx_fifo_lines = 0,
        },
        .peripheral_map = 0,
    };
    ESP_ERROR_CHECK(usb_host_install(&host_config));
    ESP_ERROR_CHECK(cdc_acm_host_install(NULL));

    // -----------------------
    // Tasks starten
    xTaskCreate(usb_connect_loop, "usb_connect_loop", 4096, nullptr, 10, nullptr);
    xTaskCreate(usb_host_task, "usb_host_task", 4096, NULL, 20, NULL);
    xTaskCreate(uart_event_task, "uart_event_task", 8192, NULL, 10, NULL);
    xTaskCreate(process_usb_rx_task, "process_usb_rx_task", 4096, NULL, 12, NULL);
    xTaskCreate(queue2grbl, "queue2grbl", 8192, NULL, 10, NULL);
    xTaskCreate(joy_task, "joy_task", 4096, NULL, 10, NULL);

}