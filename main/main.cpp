#include <stdio.h>
#include <string.h>
#include <memory>
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

#include "esp_timer.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"

#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif

// UART-Konfiguration
#define UART_NUM UART_NUM_0
#define UART0_TX_PIN 43
#define UART0_RX_PIN 44
#define BAUDRATE 115200
const int UART_BUF_SIZE = 1024;

#define UART_LOG_NUM UART_NUM_1
#define UART_LOG_TX_PIN 17
#define UART_LOG_RX_PIN 18

// Queue-Definitionen
#define LINE_QUEUE_LENGTH 10
#define MAX_LINE_LENGTH 128
#define USB_MAX_CHUNK 128

#define RX_QUEUE_LENGTH 64
#define RX_ITEM_SIZE 128

// USB CDC Device Handle
cdc_acm_dev_hdl_t cdc_dev = NULL;

// Struktur für empfangene UART/USB-Pakete
typedef struct
{
    uint8_t data[RX_ITEM_SIZE];
    size_t len;
} rx_packet_t;

//**************************************************************************
// ADC Konfiguration für Joystick
// ADC1 Kanäle für X- und Y-Achse
#define ADC_UNIT ADC_UNIT_2
#define ADC_ATTEN ADC_ATTEN_DB_12
#define joystick_X_Pin ADC_CHANNEL_2
#define joystick_Y_Pin ADC_CHANNEL_3

adc_channel_t channels[2] = {joystick_X_Pin, joystick_Y_Pin};
adc_oneshot_unit_handle_t adc_handle;
static int adc_raw[2];
static int voltage[2];

// ADC Kalibrierungs-Handles
adc_cali_handle_t adc_cali_chan0_handle = NULL;
adc_cali_handle_t adc_cali_chan1_handle = NULL;
static bool adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle);

//**************************************************************************
// Joystick & Jogging Variablen
// --- Stop-Befehl definieren ---
#define GRBL_STOP_CMD 0x85
static bool is_jogging_now = false;
static bool filter_jog_ok = false;
static bool next_jog_true = false;
static int64_t jog_filter_until_us = 0; // Zeitmarke in µs

int x = 0, y = 0, percent = 0;

// Maschinen-Konfiguration
struct machine_t
{
    char jogUnit[128]; // Jogging unit "G21" for mm or "G20" for inches.
    float maxFeedRate; // Setting up the value of maximum feedrate the machine will go with when the joystick is fully pushed a side for jogging.
} machine;

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

// GRBL Settings Struktur
typedef struct
{
    float steps_per_mm[3];    // $100, $101, $102
    float max_rate[3];        // $110, $111, $112
    float accel[3];           // $120, $121, $122
    float max_travel[3];      // $130, $131, $132
    float junction_deviation; // $11
    float arc_tolerance;      // $12
    float idle_lock_time;     // $1
    int invert_mask;          // $3
    int step_pulse_us;        // $0
    float feed_rate;          // letzter bekannter Feed (optional)
} grbl_settings_t;

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
} tx_item_t;

static QueueHandle_t tx_usb_queue = NULL;
static QueueHandle_t control_queue; // für Stop/Notfall-Bytes

#define CONTROL_QUEUE_LENGTH 8

//******************************************************************************************
// Joystick Start/Stop Funktionen
static inline void start_jogging(void)
{
    // setze Flags atomar-ish (volatile reicht für einfache usage)
    is_jogging_now = true;
    filter_jog_ok = false; // keine Filterung während aktivem Jogging
    next_jog_true = true;
}
static inline void stop_jogging(void)
{
    // Jogging beendet — aktiviere kurzzeitigen Filter für Nachlauf-ok
    is_jogging_now = false;
    next_jog_true = false;
    xSemaphoreTake(ok_sem, 0);
    const int FILTER_MS = 1000; // Dauer, in der Nachschwinger-ok ignoriert werden
    jog_filter_until_us = esp_timer_get_time() + (int64_t)FILTER_MS * 1000LL;
    xSemaphoreGive(ok_sem);
    filter_jog_ok = true;
    next_jog_true = true;
}
//******************************************************************************************
// GRBL-Paar Parsing ($Index=Value)
static inline bool parse_grbl_pair(const char *line, int *index, float *value)
{
    if (line[0] != '$')
        return false;

    char *eq = strchr(line, '=');
    if (!eq)
        return false;

    *index = atoi(line + 1);
    *value = atof(eq + 1);
    return true;
}
//*************************************************************************************
// GRBL Konfigurationszeile hinzufügen
void grbl_config_add_line(grbl_config_t *cfg, const char *line)
{
    if (cfg->count >= GRBL_MAX_CONFIG_LINES)
        return;

    strncpy(cfg->lines[cfg->count], line, GRBL_MAX_LINE_LEN);
    cfg->lines[cfg->count][GRBL_MAX_LINE_LEN - 1] = 0;
    cfg->count++;
}
//******************************************************************************************
// GRBL Config parsen
void grbl_parse_config(grbl_config_t *cfg, grbl_settings_t *out)
{
    memset(out, 0, sizeof(*out));

    for (int i = 0; i < cfg->count; i++)
    {
        char *line = cfg->lines[i];

        int idx;
        float val;

        if (!parse_grbl_pair(line, &idx, &val))
            continue;

        switch (idx)
        {
        case 0:
            out->step_pulse_us = (int)val;
            break;
        case 1:
            out->idle_lock_time = val;
            break;
        case 3:
            out->invert_mask = (int)val;
            break;

        case 11:
            out->junction_deviation = val;
            break;
        case 12:
            out->arc_tolerance = val;
            break;

        // Steps/mm
        case 100:
            out->steps_per_mm[0] = val;
            break;
        case 101:
            out->steps_per_mm[1] = val;
            break;
        case 102:
            out->steps_per_mm[2] = val;
            break;

        // Max feed rate
        case 110:
            out->max_rate[0] = val;
            break;
        case 111:
            out->max_rate[1] = val;
            break;
        case 112:
            out->max_rate[2] = val;
            break;

        // Acceleration
        case 120:
            out->accel[0] = val;
            break;
        case 121:
            out->accel[1] = val;
            break;
        case 122:
            out->accel[2] = val;
            break;

        // Travel limits
        case 130:
            out->max_travel[0] = val;
            break;
        case 131:
            out->max_travel[1] = val;
            break;
        case 132:
            out->max_travel[2] = val;
            break;

        default:
            break; // alles andere ignorieren
        }
    }
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
    cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    cfg.source_clk = UART_SCLK_DEFAULT;

    uart_param_config(UART_LOG_NUM, &cfg);
    uart_set_pin(UART_LOG_NUM, UART_LOG_TX_PIN, UART_LOG_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_LOG_NUM, UART_BUF_SIZE, UART_BUF_SIZE, 0, NULL, 0);

    esp_err_t ret;
    ret = uart_param_config(UART_NUM, &cfg);
    if (ret != ESP_OK)
        ESP_LOGE("uart_init", "uart_param_config failed: %d", ret);
    ret = uart_set_pin(UART_NUM, UART0_TX_PIN, UART0_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK)
        ESP_LOGE("uart_init", "uart_set_pin failed: %d", ret);
    ret = uart_driver_install(UART_NUM, UART_BUF_SIZE, UART_BUF_SIZE, 20, &rx_uart_queue, 0);
    if (ret != ESP_OK)
        ESP_LOGE("uart_init", "uart_driver_install failed: %d", ret);
    else
        ESP_LOGI("uart_init", "uart_driver_install OK");
    if (rx_uart_queue == NULL)
    {
        ESP_LOGE("uart_init", "uart_queue == NULL after driver install!");
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
            // bzero(dtmp, UART_BUF_SIZE);

            switch (event.type)
            {
            case UART_DATA:
            {
                ESP_LOGD("uart_event_task", "[UART DATA]: %d", event.size);
                tx_item_t item;

                size_t to_read = event.size;
                if (to_read > sizeof(item.data))
                    to_read = sizeof(item.data);

                int r = uart_read_bytes(UART_NUM, (uint8_t *)item.data, to_read, portMAX_DELAY);
                if (r > 0)
                {
                    item.len = (size_t)r;
                    if (xQueueSend(tx_usb_queue, &item, pdMS_TO_TICKS(10)) != pdPASS)
                    {
                        ESP_LOGW("uart_event_task", "tx_usb_queue voll, drop uart data");
                    }
                }
                break;
            }
            case UART_BREAK:
                ESP_LOGI("uart_event_task", "UART RX break detected");
                break;

            case UART_BUFFER_FULL:
                ESP_LOGW("uart_event_task", "UART RX buffer full");
                uart_flush_input(UART_NUM);
                xQueueReset(tx_usb_queue);
                break;

            case UART_FIFO_OVF:
                ESP_LOGW("uart_event_task", "UART RX FIFO overflow");
                uart_flush_input(UART_NUM);
                xQueueReset(tx_usb_queue);
                break;

            case UART_FRAME_ERR:
                ESP_LOGW("uart_event_task", "UART frame error");
                break;

            case UART_PARITY_ERR:
                ESP_LOGW("uart_event_task", "UART parity error");
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
        ESP_LOGW("handle_usb_rx", "RX queue full, drop packet");

    return true;
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
                    bool is_ok = strstr(line_buf, "ok") != NULL;
                    int64_t now = esp_timer_get_time();

                    // --------------------------
                    // 0) Fehlerprüfung (ALARM/ERROR)
                    // --------------------------
                    if (is_jogging_now)
                    {
                        if (strstr(line_buf, "ALARM:") || strstr(line_buf, "ERROR:"))
                        {
                            stop_jogging();

                            // Queue leeren
                            xQueueReset(tx_usb_queue);

                            // Optional: Stop-Befehl an GRBL senden
                            uint8_t stop_cmd = GRBL_STOP_CMD;
                            xQueueSend(control_queue, &stop_cmd, 0);

                            // GRBL Reset senden ($X) nach kurzer Verzögerung
                            vTaskDelay(pdMS_TO_TICKS(50));
                            tx_item_t reset_item;
                            const char *reset_cmd = "$X\n";
                            reset_item.len = strlen(reset_cmd);
                            memcpy(reset_item.data, reset_cmd, reset_item.len);
                            if (xQueueSend(tx_usb_queue, &reset_item, 0) != pdPASS)
                            {
                                ESP_LOGI("usb_rx", "tx_usb_queue voll, Reset-Befehl verworfen");
                            }

                            ESP_LOGI("usb_rx", "GRBL Fehler erkannt: %s", line_buf);

                            // Optional: kleinen Delay einfügen, damit GRBL Reset verarbeitet
                            vTaskDelay(pdMS_TO_TICKS(50));

                            uart_write_bytes(UART_NUM, line_buf, line_len);
                            line_len = 0;
                            continue; // nächste Zeile
                        }
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
                    // --------------------------
                    // 2) Jog läuft aktuell
                    // --------------------------
                    if (is_jogging_now && is_ok)
                    {
                        xSemaphoreGive(ok_sem); // wichtig!
                        line_len = 0;
                        continue;
                    }

                    // --------------------------
                    // 3) Jog wurde gerade gestoppt
                    //    → alle ok ignorieren
                    // --------------------------
                    if (filter_jog_ok)
                    {
                        if (now < jog_filter_until_us && is_ok)
                        {
                            // IGNORIEREN
                            line_len = 0;
                            continue;
                        }
                        else
                        {
                            // Zeitfenster abgelaufen → deaktivieren
                            filter_jog_ok = false;
                        }
                    }

                    // --------------------------
                    // 4) Zeilen, die mit '$' beginnen
                    // --------------------------
                    if (line_buf[0] == '$')
                    {
                        grbl_config_add_line(&raw_cfg, line_buf);
                        // Konfiguration jetzt parsen
                        grbl_settings_t settings;
                        grbl_parse_config(&raw_cfg, &settings);
                        machine.maxFeedRate = settings.max_rate[0]; // X-Achse als Referenz

                        // optional Log
                        ESP_LOGI("usb_rx", "GRBL $-Zeile empfangen: %s", line_buf);
                        ESP_LOGI("grbl", "FeedMax: X=%.3f", machine.maxFeedRate);
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
// Queue -> USB Task
static void queue2grbl(void *arg)
{
    tx_item_t item;
    uint8_t ctrl;

    while (true)
    {
        vTaskDelay(pdMS_TO_TICKS(10));

        // STOP bytes mit höherer Priorität (nicht blockierend prüfen)
        if (xQueueReceive(control_queue, &ctrl, 0) == pdPASS)
        {
            xSemaphoreTake(usb_tx_lock, portMAX_DELAY);
            if (cdc_dev)
            {
                cdc_acm_host_data_tx_blocking(cdc_dev, &ctrl, 1, 1000);
            }
            xSemaphoreGive(usb_tx_lock);
            ESP_LOGI("queue2grbl", "STOP sofort gesendet");
            xQueueReset(tx_usb_queue);
            continue;
        }

        // Warte auf ein tx-Item
        if (xQueueReceive(tx_usb_queue, &item, pdMS_TO_TICKS(10)) != pdPASS)
            continue;

        size_t len = item.len;
        if (len == 0)
            continue;

        xSemaphoreTake(usb_tx_lock, portMAX_DELAY);
        // nur zum loggen --------------------------------

        char tmp[MAX_LINE_LENGTH + 1];
        memcpy(tmp, item.data, item.len);
        tmp[item.len] = 0;
        ESP_LOGI("queue2grbl", "%s", tmp);

        if (cdc_dev)
        {
            esp_err_t err = cdc_acm_host_data_tx_blocking(cdc_dev, (uint8_t *)item.data, item.len, 1000);
            if (err != ESP_OK)
            {
                ESP_LOGI("queue2grbl", "Sendefehler (err=%d)", err);
                break;
            }
        }
        xSemaphoreGive(usb_tx_lock);
    }
}
//*************************************************************************************
// USB Events
static void handle_event(const cdc_acm_host_dev_event_data_t *event, void *user_ctx)
{
    switch (event->type)
    {
    case CDC_ACM_HOST_ERROR:
        ESP_LOGE("handle_event", "CDC-ACM error has occurred, err_no = %i", event->data.error);
        break;
    case CDC_ACM_HOST_DEVICE_DISCONNECTED:
        ESP_LOGI("handle_event", "Device suddenly disconnected");
        ESP_ERROR_CHECK(cdc_acm_host_close(event->data.cdc_hdl));
        xSemaphoreGive(device_disconnected_sem);
        if (rx_usb_queue)
            xQueueReset(rx_usb_queue);
        break;
    case CDC_ACM_HOST_SERIAL_STATE:
        // ESP_LOGI("handle_event", "Serial state notif 0x%04X", event->data.serial_state.val);
        break;
    case CDC_ACM_HOST_NETWORK_CONNECTION:
    default:
        ESP_LOGW("handle_event", "Unsupported CDC event: %i", event->type);
        break;
    }
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
int map(int x, int in_min, int in_max, int out_min, int out_max)
{
    return ((x - in_min) * (out_max - out_min)) / (in_max - in_min) + out_min;
}
//*************************************************************************************
// ADC Initialisierung & Kalibrierung
void adc_init(adc_channel_t *channel, uint8_t numChannels)
{
    adc_oneshot_unit_init_cfg_t unitConfig = {
        .unit_id = ADC_UNIT, // ADC1
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

    //-------------ADC1 Calibration Init---------------//

    adc_calibration_init(ADC_UNIT, channels[0], ADC_ATTEN_DB_12, &adc_cali_chan0_handle);
    adc_calibration_init(ADC_UNIT, channels[1], ADC_ATTEN_DB_12, &adc_cali_chan1_handle);
}
//*************************************************************************************
// ADC Calibration
static bool adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated)
    {
        ESP_LOGI("adc_calibration_init", "scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .chan = channel,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK)
        {
            calibrated = true;
        }
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated)
    {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Line Fitting");
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK)
        {
            calibrated = true;
        }
    }
#endif

    *out_handle = handle;
    if (ret == ESP_OK)
    {
        ESP_LOGI("adc_calibration_init", "Calibration Success");
    }
    else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated)
    {
        ESP_LOGW("adc_calibration_init", "eFuse not burnt, skip software calibration");
    }
    else
    {
        ESP_LOGE("adc_calibration_init", "Invalid arg or no memory");
    }

    return calibrated;
}
//*************************************************************************************
static void adc_read(void)
{

    adc_oneshot_read(adc_handle, channels[0], &adc_raw[0]); // read channel 0
    adc_cali_raw_to_voltage(adc_cali_chan0_handle, adc_raw[0], &voltage[0]);

    adc_oneshot_read(adc_handle, channels[1], &adc_raw[1]); // read channel 1
    adc_cali_raw_to_voltage(adc_cali_chan1_handle, adc_raw[1], &voltage[1]);

    x = map(adc_raw[0], 0, 4095, -100, 100);
    y = map(adc_raw[1], 0, 4095, -100, 100);
}
//*************************************************************************************
// GRBL Config lesen
bool grbl_read_config(grbl_config_t *cfg)
{
    cfg->count = 0;

    // kleines Delay, GRBL muss bereit sein
    vTaskDelay(pdMS_TO_TICKS(200));

    // $$ senden
    const char *cmd = "$$\n";
    if (xQueueSend(tx_usb_queue, (void *)cmd, pdMS_TO_TICKS(50)) != pdPASS)
        return false;

    int64_t start = esp_timer_get_time();

    while (true)
    {
        char rxline[128] = {0};

        // 1 Sekunde Timeout insgesamt
        if (esp_timer_get_time() - start > 1000000)
        {
            ESP_LOGW("grbl_cfg", "Timeout beim Lesen der GRBL-Konfiguration");
            break;
        }

        // 100 ms auf neue Zeilen warten
        if (xQueueReceive(rx_usb_queue, rxline, pdMS_TO_TICKS(100)) != pdPASS)
            continue;

        // nur Zeilen beachten, die mit '$' anfangen
        if (rxline[0] != '$')
            continue;

        // "ok" bedeutet Ende
        if (strncmp(rxline, "ok", 2) == 0)
            break;

        // Zeile speichern
        if (cfg->count < 64)
        {
            strncpy(cfg->lines[cfg->count], rxline, 63);
            cfg->count++;
        }
    }

    if (cfg->count == 0)
    {
        ESP_LOGW("grbl_cfg", "Keine GRBL-Config empfangen.");
        return false;
    }

    return true;
}
//******************************************************************************************
// USB Connected Callback
void on_usb_connected()
{
    ESP_LOGI("usb", "Starte GRBL Config Abfrage");

    grbl_config_active = true;
    grbl_config_done = false;

    memset(&raw_cfg, 0, sizeof(raw_cfg)); // Buffer leeren

    // "$$" direkt senden
    esp_err_t err = cdc_acm_host_data_tx_blocking(cdc_dev, (uint8_t *)"$$\n", 3, 1000);
    if (err != ESP_OK)
    {
        ESP_LOGW("on_usb_connected", "Sendefehler $$");
        return;
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
        return;
    }

    // Konfiguration jetzt parsen
    grbl_settings_t settings;
    grbl_parse_config(&raw_cfg, &settings);

    machine.maxFeedRate = settings.max_rate[0]; // X-Achse als Referenz

    ESP_LOGI("grbl", "Konfiguration erfolgreich geladen");
    ESP_LOGI("grbl", "FeedMax: X=%.3f  Y=%.3f  Z=%.3f",
             settings.max_rate[0],
             settings.max_rate[1],
             settings.max_rate[2]);
    ESP_LOGI("grbl", "Steps/mm: X=%.3f  Y=%.3f  Z=%.3f",
             settings.steps_per_mm[0],
             settings.steps_per_mm[1],
             settings.steps_per_mm[2]);
}
//******************************************************************************************
// Joystick Normalisierung & Quantisierung
static void configure_joy(void)
{
    joystick.xHardwareReversed = true;
    joystick.yHardwareReversed = false;

    strcpy(machine.jogUnit, "G21");
    machine.maxFeedRate = 4000;
    adc_read();
}
//******************************************************************************************
// --- 10-Stufen Quantisierung
static float quantize_10(int v)
{
    if (v > -20 && v < 20) // Deadzone = 20
        return 0.0f;

    // in 10 Bereiche pro Seite teilen
    // -100..100 -> -10..+10
    int q = v / 10; // integer Abtastung
    if (q > 10)
        q = 10;
    if (q < -10)
        q = -10;

    return (float)q / 10.0f; // zurück nach -1..+1
}
//******************************************************************************************
// Joystick Task
static void joy_task(void *arg)
{
    // --- Konfiguration ---
    const TickType_t JOG_INTERVAL = pdMS_TO_TICKS(10); // 10 ms
    const TickType_t ADC_INTERVAL = pdMS_TO_TICKS(50); // ADC alle 50 ms
    const float STEP_RES = 0.1f;                       // Schrittauflösung in mm

    float send_x = 0;
    float send_y = 0;
    float F = 0;

    TickType_t last_wake = xTaskGetTickCount();
    TickType_t last_adc_time = 0;

    int lx = 0, ly = 0;

    while (true)
    {
        vTaskDelayUntil(&last_wake, JOG_INTERVAL);

        TickType_t now = xTaskGetTickCount();

        // --- ADC lesen ---
        if ((now - last_adc_time) >= ADC_INTERVAL)
        {
            adc_read();
            last_adc_time = now;

            lx = joystick.xHardwareReversed ? -x : x;
            ly = joystick.yHardwareReversed ? -y : y;
        }

        // --- Quantisierung 11 Stufen ---
        float sx = quantize_10(lx);
        float sy = quantize_10(ly);

        // --- Deadzone ---
        if (sx == 0.0f && sy == 0.0f)
        {
            if (is_jogging_now)
            {
                stop_jogging();
                uint8_t stop = GRBL_STOP_CMD;
                xQueueReset(tx_usb_queue);
                xQueueSend(control_queue, &stop, 0);
            }
            continue;
        }

        // --- Achsgeschwindigkeit (mm/s) ---
        float max_mm_s = machine.maxFeedRate / 60.0f;
        float vx = sx * max_mm_s;
        float vy = sy * max_mm_s;

        float interval_s = (float)JOG_INTERVAL * (float)portTICK_PERIOD_MS / 1000.0f;

        // --- Schritte pro Intervall (ganzzahlig, STEP_RES) ---
        send_x = roundf((vx * interval_s) / STEP_RES) * STEP_RES;
        send_y = roundf((vy * interval_s) / STEP_RES) * STEP_RES;

        // --- Feedrate berechnen ---
        float Fx = (fabsf(send_x) / interval_s) * 60.0f;
        float Fy = (fabsf(send_y) / interval_s) * 60.0f;
        F = fmaxf(Fx, Fy);

        if (F > machine.maxFeedRate)
            F = machine.maxFeedRate;

        if (F < 500.0f)
            F = 500; // Minimum Feedrate

        start_jogging();
        if (!next_jog_true)
            continue;

        // --- SENDEN ---
        if (xSemaphoreTake(ok_sem, (TickType_t)100) == pdTRUE &&
            next_jog_true && is_jogging_now)
        {
            // längere Schritte erzeugen, um Lookahead auszunutzen
            // z.B. min. 1 mm pro Befehl, multipliziert durch Achsstufe
            float cmd_x = send_x;
            float cmd_y = send_y;

            if (fabsf(cmd_x) < STEP_RES)
                cmd_x = (cmd_x > 0) ? STEP_RES : (cmd_x < 0 ? -STEP_RES : 0);
            if (fabsf(cmd_y) < STEP_RES)
                cmd_y = (cmd_y > 0) ? STEP_RES : (cmd_y < 0 ? -STEP_RES : 0);

            char cmd[128];
            int n = snprintf(cmd, sizeof(cmd),
                             "$J=G91 X%.3f Y%.3f F%.0f\n",
                             cmd_x, cmd_y, F);

            tx_item_t item;
            item.len = n;
            memcpy(item.data, cmd, n);

            xQueueSend(tx_usb_queue, &item, 0);
        }
    }
}
//******************************************************************************************
// USB Connect Loop
static void usb_connect_loop(void *arg)
{
    const uint16_t VID = 0xFFFF; // anpassen
    const uint16_t PID = 0x0005; // anpassen

    const cdc_acm_host_device_config_t dev_cfg = {
        .connection_timeout_ms = 5000,
        .out_buffer_size = USB_MAX_CHUNK,
        .in_buffer_size = 64,
        .event_cb = handle_event,
        .data_cb = handle_usb_rx,
        .user_arg = NULL};

    while (true)
    {
        ESP_LOGI("usb_connect_loop", "Versuche USB-Gerät zu öffnen...");
        esp_err_t err = cdc_acm_host_open(VID, PID, 0, &dev_cfg, &cdc_dev);

        if (err != ESP_OK)
        {
            // Gerät nicht da, warten und erneut versuchen
            ESP_LOGI("usb_connect_loop", "Kein Gerät, retry in 500ms");
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        cdc_acm_host_desc_print(cdc_dev);
        vTaskDelay(pdMS_TO_TICKS(100));

        cdc_acm_line_coding_t line_coding;
        line_coding.dwDTERate = 115200;
        line_coding.bDataBits = 8;
        line_coding.bParityType = 0;
        line_coding.bCharFormat = 0;
        ESP_ERROR_CHECK(cdc_acm_host_line_coding_set(cdc_dev, &line_coding));
        ESP_ERROR_CHECK(cdc_acm_host_line_coding_get(cdc_dev, &line_coding));
        ESP_LOGI("usb_connect_loop", "Line Get: Rate: %"PRIu32", Stop bits: %"PRIu8", Parity: %"PRIu8", Databits: %"PRIu8"",
                 line_coding.dwDTERate, line_coding.bCharFormat, line_coding.bParityType, line_coding.bDataBits);

        ESP_ERROR_CHECK(cdc_acm_host_set_control_line_state(cdc_dev, true, false));

        ESP_LOGI("usb_connect_loop", "USB-Gerät verbunden");
        led_set_color(0, 0, 255); // Blau = Gerät verbunden
        on_usb_connected();

        // Control-Line-Reset für GRBL-Gerät
        cdc_acm_host_set_control_line_state(cdc_dev, true, true);
        vTaskDelay(pdMS_TO_TICKS(100));
        cdc_acm_host_set_control_line_state(cdc_dev, true, false);
        vTaskDelay(pdMS_TO_TICKS(500));

        // Warte bis Gerät getrennt wird
        xSemaphoreTake(device_disconnected_sem, portMAX_DELAY);
        cdc_dev = NULL;
        led_set_color(255, 0, 0); // Rot = Gerät getrennt
        ESP_LOGW("usb_connect_loop", "USB-Gerät getrennt");
    }
}
//******************************************************************************************
// Main
extern "C" void app_main()
{
    esp_log_level_set("*", ESP_LOG_NONE);
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

    // tx_usb_queue = xQueueCreate(LINE_QUEUE_LENGTH, MAX_LINE_LENGTH);
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
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
    };
    ESP_ERROR_CHECK(usb_host_install(&host_config));
    ESP_ERROR_CHECK(cdc_acm_host_install(NULL));

    // -----------------------
    // Tasks starten
    xTaskCreate(usb_connect_loop, "usb_connect_loop", 4096, nullptr, 10, nullptr);
    xTaskCreate(usb_host_task, "usb_host_task", 4096, NULL, 20, NULL);
    xTaskCreate(uart_event_task, "uart_event_task", 8192, NULL, 10, NULL);
    xTaskCreate(process_usb_rx_task, "process_usb_rx_task", 4096, NULL, 10, NULL);
    xTaskCreate(queue2grbl, "queue2grbl", 8192, NULL, 10, NULL);
    xTaskCreate(joy_task, "joy_task", 4096, NULL, 10, NULL);

}