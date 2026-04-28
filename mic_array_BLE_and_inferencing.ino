#define EIDSP_QUANTIZE_FILTERBANK   0
#include <rcmarkowitz-project-1_inferencing.h>
#include "driver/i2s.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Wire.h>
#include <string.h>

#define XMOS_ADDR 0x2C

#define GPO_SERVICER_RESID            20
#define GPO_SERVICER_RESID_LED_EFFECT 12
#define GPO_SERVICER_RESID_DOA        19
#define GPO_DOA_READ_NUM_BYTES        4

// === BLE CONFIG ===
BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;
uint32_t value = 0;

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

void write_led_effect(uint8_t effect);

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      write_led_effect(4);
    };
    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      write_led_effect(3);
    }
};

QueueHandle_t command_queue;

// ==== AUDIO CONFIG ====
#define I2S_PORT            I2S_NUM_0
#define I2S_WS              7
#define I2S_SD              43
#define I2S_SCK             8

#define SAMPLE_RATE         48000
#define I2S_SAMPLE_BITS     32
#define SAMPLE_BUFFER_SIZE  2048

// ==== SLIDING WINDOW CONFIG ====
#define SLICES_PER_WINDOW   3
#define SLICE_SIZE          (EI_CLASSIFIER_RAW_SAMPLE_COUNT / SLICES_PER_WINDOW)

// ==== CONSECUTIVE-AGREEMENT CONFIG ====
#define CONSECUTIVE_REQUIRED 2

// ==== BUFFERS ====
static int32_t  i2s_samples[SAMPLE_BUFFER_SIZE];
static int16_t  slice_buffer_a[SLICE_SIZE];
static int16_t  slice_buffer_b[SLICE_SIZE];
static int16_t *write_slice = slice_buffer_a;
static int16_t *read_slice  = slice_buffer_b;
static int16_t  rolling_buffer[EI_CLASSIFIER_RAW_SAMPLE_COUNT];

static volatile uint32_t slice_count = 0;
static volatile bool     slice_ready = false;

static bool record_status = true;
static bool debug_nn      = false;

// ==== DOA POLLING BUFFER ====
#define DOA_BUF_SIZE           64
#define DOA_BUCKET_DEGREES     5
#define DOA_WINDOW_MS          1500
#define VAD_MAJORITY_THRESHOLD 0.3f  // at least 30% of window samples must have VAD=1

typedef struct {
    uint16_t angle;
    uint8_t  vad;
    uint32_t timestamp_ms;
} doa_sample_t;

typedef struct {
    uint16_t angle;     // 0xFFFF if no valid angle
    bool     speech_detected;
} doa_result_t;

static doa_sample_t  doa_poll_buf[DOA_BUF_SIZE];
static uint8_t       doa_poll_head  = 0;
static uint8_t       doa_poll_count = 0;
static SemaphoreHandle_t doa_mutex;

// ==== FUNCTION DECLARATIONS ====
static void audio_inference_callback(uint32_t n_bytes);
static void capture_samples(void *arg);
static void ble_sender_task(void *arg);
static void doa_poll_task(void *arg);
static int  microphone_audio_signal_get_data(size_t offset, size_t length, float *out_ptr);
static int  i2s_init();
static void i2s_deinit();
static doa_result_t get_doa_result();

void setup() {
    Serial.begin(115200);
    delay(3000);
    Wire.begin();
    Serial.println("XVF3800 Keyword Spotting - Sliding Window");
    Serial.print("Deploy version: ");
    Serial.println(EI_CLASSIFIER_PROJECT_DEPLOY_VERSION);
    Serial.print("Slice size: ");
    Serial.println(SLICE_SIZE);
    Serial.print("Raw Sample Count: ");
    Serial.println(EI_CLASSIFIER_RAW_SAMPLE_COUNT);

    memset(rolling_buffer, 0, sizeof(rolling_buffer));

    if (i2s_init() != 0) {
        Serial.println("ERR: I2S init failed");
        while(1);
    }

    write_led_effect(3);

    doa_mutex = xSemaphoreCreateMutex();

    BLEDevice::init("ESP32-S3");
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());
    BLEService *pService = pServer->createService(SERVICE_UUID);
    pCharacteristic = pService->createCharacteristic(
                        CHARACTERISTIC_UUID,
                        BLECharacteristic::PROPERTY_READ   |
                        BLECharacteristic::PROPERTY_WRITE  |
                        BLECharacteristic::PROPERTY_NOTIFY |
                        BLECharacteristic::PROPERTY_INDICATE
                    );
    pCharacteristic->addDescriptor(new BLE2902());
    pService->start();
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(false);
    pAdvertising->setMinPreferred(0x0);
    BLEDevice::startAdvertising();
    Serial.println("Waiting a client connection to notify...");

    command_queue = xQueueCreate(5, sizeof(const char *));
    xTaskCreate(ble_sender_task, "BLESender", 4096, NULL, 1, NULL);
    xTaskCreate(capture_samples, "CaptureSamples", 4096, NULL, 1, NULL);
    xTaskCreate(doa_poll_task,   "DoAPoll",        2048, NULL, 1, NULL);
    Serial.println("Listening...");
}

void loop() {
    if (!deviceConnected && oldDeviceConnected) {
        delay(500);
        pServer->startAdvertising();
        Serial.println("start advertising");
        oldDeviceConnected = deviceConnected;
    }
    if (deviceConnected && !oldDeviceConnected) {
        oldDeviceConnected = deviceConnected;
    }

    if (!slice_ready) {
        delay(1);
        return;
    }
    slice_ready = false;

    memmove(rolling_buffer,
            rolling_buffer + SLICE_SIZE,
            sizeof(int16_t) * (EI_CLASSIFIER_RAW_SAMPLE_COUNT - SLICE_SIZE));

    memcpy(rolling_buffer + (EI_CLASSIFIER_RAW_SAMPLE_COUNT - SLICE_SIZE),
           read_slice,
           sizeof(int16_t) * SLICE_SIZE);

    signal_t signal;
    signal.total_length = EI_CLASSIFIER_RAW_SAMPLE_COUNT;
    signal.get_data     = &microphone_audio_signal_get_data;

    ei_impulse_result_t result = {0};
    EI_IMPULSE_ERROR r = run_classifier(&signal, &result, debug_nn);

    if (r != EI_IMPULSE_OK) {
        ei_printf("ERR: Classifier failed (%d)\n", r);
        return;
    }

    static unsigned long last_prediction_ms = 0;
    static const char   *pending_label      = "";
    static int           consecutive_count  = 0;

    // Find the highest-scoring command this window (skip noise and unknown)
    const char *top_label = NULL;
    float       top_score = 0.0f;
    size_t      top_ix    = 0;

    for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT - 2; ix++) {
        float score = result.classification[ix].value;
        if (score > top_score) {
            top_score = score;
            top_label = result.classification[ix].label;
            top_ix    = ix;
        }
    }

    float noise_score = result.classification[EI_CLASSIFIER_LABEL_COUNT - 2].value;
    float unk_score   = result.classification[EI_CLASSIFIER_LABEL_COUNT - 1].value;

    bool passed_threshold = (top_label != NULL) &&
                            (top_score > 0.7f) &&
                            ((noise_score + unk_score) <= 0.1f);

    if (passed_threshold) {
        // Track consecutive agreement across windows
        if (pending_label != NULL && strcmp(top_label, pending_label) == 0) {
            consecutive_count++;
        } else {
            pending_label     = top_label;
            consecutive_count = 1;
        }

        if (consecutive_count >= CONSECUTIVE_REQUIRED &&
            millis() - last_prediction_ms > 1500) {

            last_prediction_ms = millis();

            doa_result_t doa = get_doa_result();
            if (doa.speech_detected) {
                ei_printf("Prediction: %s (", top_label);
                ei_printf_float(top_score);
                ei_printf(") [x%d]\n", consecutive_count);

                if (strcmp(top_label, "ComeHere") == 0) {
                    char cmd_buf[32];
                    if (doa.angle != 0xFFFF) {
                        snprintf(cmd_buf, sizeof(cmd_buf), "ComeHere:%u", doa.angle);
                    } else {
                        snprintf(cmd_buf, sizeof(cmd_buf), "ComeHere:-1");
                    }
                    const char *cmd = cmd_buf;
                    xQueueSend(command_queue, &cmd, 0);
                } else {
                    const char *cmd = top_label;
                    xQueueSend(command_queue, &cmd, 0);
                }
            }

            // Reset after firing so we require fresh agreement for the next command
            pending_label     = "";
            consecutive_count = 0;
        }
    } else {
        // No strong prediction this window, decay the streak
        pending_label     = "";
        consecutive_count = 0;
    }

#if EI_CLASSIFIER_HAS_ANOMALY == 1
    ei_printf("Anomaly: ");
    ei_printf_float(result.anomaly);
    ei_printf("\n");
#endif
}

static doa_result_t get_doa_result() {
    doa_result_t out = { 0xFFFF, false };

    if (xSemaphoreTake(doa_mutex, pdMS_TO_TICKS(10)) != pdTRUE) return out;

    uint32_t now = millis();
    uint8_t count = doa_poll_count;

    doa_sample_t local[DOA_BUF_SIZE];
    uint8_t local_count = 0;

    for (uint8_t i = 0; i < count; i++) {
        uint8_t idx = (doa_poll_head + DOA_BUF_SIZE - count + i) % DOA_BUF_SIZE;
        doa_sample_t *s = &doa_poll_buf[idx];
        if (now - s->timestamp_ms <= DOA_WINDOW_MS) {
            local[local_count++] = *s;
        }
    }

    xSemaphoreGive(doa_mutex);

    if (local_count == 0) return out;

    // Check VAD majority
    uint8_t vad_count = 0;
    for (uint8_t i = 0; i < local_count; i++) {
        if (local[i].vad == 1) vad_count++;
    }
    float vad_ratio = (float)vad_count / (float)local_count;
    out.speech_detected = (vad_ratio >= VAD_MAJORITY_THRESHOLD);

    // Find majority angle regardless — only used if speech_detected is true
    uint16_t best_angle = 0xFFFF;
    uint8_t  best_count = 0;

    for (uint8_t i = 0; i < local_count; i++) {
        uint8_t matches = 0;
        for (uint8_t j = 0; j < local_count; j++) {
            int diff = abs((int)local[i].angle - (int)local[j].angle);
            if (diff > 180) diff = 360 - diff;
            if (diff <= DOA_BUCKET_DEGREES) matches++;
        }
        if (matches > best_count) {
            best_count = matches;
            best_angle = local[i].angle;
        }
    }
    out.angle = best_angle;

    return out;
}

static void doa_poll_task(void *arg) {
    while (true) {
        uint16_t doa_values[2] = {0};
        uint8_t status = 0xFF;

        bool success = read_doa_values((uint8_t *)doa_values, &status);

        if (success) {
            doa_sample_t sample;
            sample.angle        = doa_values[0];
            sample.vad          = (uint8_t)doa_values[1];
            sample.timestamp_ms = millis();

            if (xSemaphoreTake(doa_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                doa_poll_buf[doa_poll_head] = sample;
                doa_poll_head = (doa_poll_head + 1) % DOA_BUF_SIZE;
                if (doa_poll_count < DOA_BUF_SIZE) doa_poll_count++;
                xSemaphoreGive(doa_mutex);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

static void audio_inference_callback(uint32_t n_bytes) {
    static uint8_t decimate = 0;
    static int32_t dc_offset = 0;

    for (uint32_t i = 0; i < n_bytes / sizeof(int32_t); i++) {
        decimate++;
        if (decimate < 3) continue;
        decimate = 0;

        dc_offset += (i2s_samples[i] - dc_offset) >> 8;
        int32_t centered = i2s_samples[i] - dc_offset;
        int16_t val = (int16_t)(centered >> 16);

        write_slice[slice_count++] = val;

        if (slice_count >= SLICE_SIZE) {
            slice_count = 0;
            int16_t *tmp = write_slice;
            write_slice  = read_slice;
            read_slice   = tmp;
            slice_ready  = true;
        }
    }
}

static void capture_samples(void *arg) {
    size_t bytes_read;
    while (record_status) {
        i2s_read(I2S_PORT,
                 (char *)i2s_samples,
                 SAMPLE_BUFFER_SIZE * sizeof(int32_t),
                 &bytes_read,
                 portMAX_DELAY);

        if (bytes_read > 0) {
            audio_inference_callback(bytes_read);
        } else {
            ei_printf("ERR: I2S read failed\n");
        }
    }
    vTaskDelete(NULL);
}

static int microphone_audio_signal_get_data(size_t offset, size_t length, float *out_ptr) {
    numpy::int16_to_float(&rolling_buffer[offset], out_ptr, length);
    return 0;
}

static int i2s_init() {
    i2s_config_t i2s_config = {
        .mode                 = (i2s_mode_t)(I2S_MODE_SLAVE | I2S_MODE_RX),
        .sample_rate          = SAMPLE_RATE,
        .bits_per_sample      = (i2s_bits_per_sample_t)I2S_SAMPLE_BITS,
        .channel_format       = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_I2S,
        .intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count        = 8,
        .dma_buf_len          = 512,
        .use_apll             = false,
        .tx_desc_auto_clear   = false,
        .fixed_mclk           = 0
    };

    i2s_pin_config_t pin_config = {
        .bck_io_num   = I2S_SCK,
        .ws_io_num    = I2S_WS,
        .data_out_num = -1,
        .data_in_num  = I2S_SD
    };

    esp_err_t err;
    err = i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
    if (err != ESP_OK) return err;

    err = i2s_set_pin(I2S_PORT, &pin_config);
    if (err != ESP_OK) return err;

    return i2s_zero_dma_buffer(I2S_PORT);
}

static void i2s_deinit() {
    i2s_driver_uninstall(I2S_PORT);
}

void ble_sender_task(void *arg) {
    const char *received_command;
    while (true) {
        if (xQueueReceive(command_queue, &received_command, portMAX_DELAY)) {
            if (deviceConnected) {
                pCharacteristic->setValue(received_command);
                pCharacteristic->notify();
            }
        }
    }
}

void write_led_effect(uint8_t effect) {
    Wire.beginTransmission(XMOS_ADDR);
    Wire.write(GPO_SERVICER_RESID);
    Wire.write(GPO_SERVICER_RESID_LED_EFFECT);
    Wire.write(1);
    Wire.write(effect);
    Wire.endTransmission();
}

bool read_doa_values(uint8_t *buffer, uint8_t *status) {
    const uint8_t resid = GPO_SERVICER_RESID;
    const uint8_t cmd = GPO_SERVICER_RESID_DOA | 0x80;
    const uint8_t read_len = GPO_DOA_READ_NUM_BYTES;

    Wire.beginTransmission(XMOS_ADDR);
    Wire.write(resid);
    Wire.write(cmd);
    Wire.write(read_len + 1);
    uint8_t result = Wire.endTransmission();

    if (result != 0) {
        Serial.print("I2C Write Error: ");
        Serial.println(result);
        return false;
    }

    Wire.requestFrom(XMOS_ADDR, (uint8_t)(read_len + 1));
    if (Wire.available() < read_len + 1) {
        Serial.println("I2C Read Error: Not enough data received.");
        return false;
    }

    *status = Wire.read();
    for (uint8_t i = 0; i < read_len; i++) {
        buffer[i] = Wire.read();
    }

    return true;
}