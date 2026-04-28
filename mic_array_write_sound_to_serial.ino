#include "driver/i2s.h"

#define I2S_PORT        I2S_NUM_0
#define I2S_WS          7
#define I2S_SD          43
#define I2S_SCK         8
#define SAMPLE_RATE     48000
#define I2S_SAMPLE_BITS 32
#define SAMPLE_BUFFER_SIZE 2048

static int32_t i2s_samples[SAMPLE_BUFFER_SIZE];

void setup() {
    Serial.begin(115200);
    delay(2000);

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

    i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
    i2s_set_pin(I2S_PORT, &pin_config);
    i2s_zero_dma_buffer(I2S_PORT);
}

void loop() {
    size_t bytes_read = 0;
    static uint8_t decimate = 0;
    static int32_t dc_offset = 0;

    i2s_read(I2S_PORT,
             (char *)i2s_samples,
             SAMPLE_BUFFER_SIZE * sizeof(int32_t),
             &bytes_read,
             portMAX_DELAY);

    uint32_t samples_read = bytes_read / sizeof(int32_t);

    for (uint32_t i = 0; i < samples_read; i++) {
        decimate++;
        if (decimate < 3) continue;
        decimate = 0;

        dc_offset += (i2s_samples[i] - dc_offset) >> 8;
        int32_t centered = i2s_samples[i] - dc_offset;
        int16_t val = (int16_t)(centered >> 16);

        Serial.write((uint8_t *)&val, 2);
    }
}