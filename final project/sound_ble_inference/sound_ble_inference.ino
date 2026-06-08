/*
  Edge Impulse sound inference with BLE result notifications.

  This sketch is meant for an Edge Impulse Arduino library export, not the
  firmware-only .bin deployment.
*/

#include <Arduino.h>
#include <ArduinoBLE.h>
#include <PDM.h>
#include <stdarg.h>
#include <stdio.h>

#include <Sound_Detection_inferencing.h>

#if defined(EI_CLASSIFIER_SENSOR) && EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_MICROPHONE
#error "This sketch expects an Edge Impulse microphone/audio classifier."
#endif

#define BLE_DEVICE_NAME "EI-Sound-BLE"
#define BLE_PAYLOAD_LEN 244
#define SERIAL_BAUD_RATE 115200
#define SERIAL_WAIT_MS 30000
#define PDM_CAPTURE_SAMPLE_RATE 16000
#define INFERENCE_GAP_MS 5000

BLEService inferenceService("19B10000-E8F2-537E-4F6C-D104768A1214");
BLEStringCharacteristic topResultCharacteristic(
    "19B10001-E8F2-537E-4F6C-D104768A1214",
    BLERead | BLENotify,
    64);
BLEStringCharacteristic scoresCharacteristic(
    "19B10002-E8F2-537E-4F6C-D104768A1214",
    BLERead | BLENotify,
    BLE_PAYLOAD_LEN);
BLEUnsignedIntCharacteristic inferenceCountCharacteristic(
    "19B10003-E8F2-537E-4F6C-D104768A1214",
    BLERead | BLENotify);

typedef struct {
    int16_t *buffer;
    int16_t resample_group[4];
    uint8_t buf_ready;
    uint8_t resample_group_count;
    uint32_t buf_count;
    uint32_t capture_count;
    uint32_t n_samples;
    uint32_t capture_samples;
} inference_t;

static inference_t inference;
static signed short sampleBuffer[2048];
static volatile bool record_ready = false;
static bool debug_nn = false;
static unsigned int inference_count = 0;

static bool setup_ble(void);
static void poll_ble_for(uint32_t duration_ms);
static void publish_ble_result(const ei_impulse_result_t &result);
static bool microphone_inference_start(uint32_t n_samples);
static bool microphone_inference_record(void);
static int microphone_audio_signal_get_data(size_t offset, size_t length, float *out_ptr);
static void microphone_inference_end(void);
static void pdm_data_ready_inference_callback(void);
static void print_inference_result(const ei_impulse_result_t &result);
void ei_printf(const char *format, ...);

void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);

    Serial.begin(SERIAL_BAUD_RATE);
    uint32_t serial_wait_start = millis();
    while (!Serial && (millis() - serial_wait_start) < SERIAL_WAIT_MS) {
        digitalWrite(LED_BUILTIN, (millis() / 250) % 2);
        delay(10);
    }
    digitalWrite(LED_BUILTIN, LOW);

    ei_printf("Edge Impulse sound inference with BLE notifications\r\n");
    ei_printf("Model audio rate: %d Hz, PDM capture rate: %d Hz\r\n",
              EI_CLASSIFIER_FREQUENCY,
              PDM_CAPTURE_SAMPLE_RATE);

    if (!microphone_inference_start(EI_CLASSIFIER_RAW_SAMPLE_COUNT)) {
        uint32_t last_status = 0;
        while (1) {
            if ((millis() - last_status) >= 1000) {
                ei_printf("Failed to allocate audio buffer or start PDM microphone\r\n");
                last_status = millis();
            }
            BLE.poll();
            digitalWrite(LED_BUILTIN, (millis() / 500) % 2);
            delay(100);
        }
    }

    if (!setup_ble()) {
        uint32_t last_status = 0;
        while (1) {
            if ((millis() - last_status) >= 1000) {
                ei_printf("Failed to initialize BLE\r\n");
                last_status = millis();
            }
            digitalWrite(LED_BUILTIN, (millis() / 500) % 2);
            delay(100);
        }
    }

    ei_printf("BLE advertising as %s\r\n", BLE_DEVICE_NAME);
    ei_printf("Subscribe to characteristic 19B10001-E8F2-537E-4F6C-D104768A1214 for top result notifications\r\n");
}

void loop()
{
    BLE.poll();

    ei_printf("Starting inferencing in %d seconds...\r\n", INFERENCE_GAP_MS / 1000);
    poll_ble_for(INFERENCE_GAP_MS);

    ei_printf("Recording...\r\n");
    if (!microphone_inference_record()) {
        ei_printf("ERR: Failed to record audio\r\n");
        return;
    }
    ei_printf("Recording done\r\n");

    signal_t signal;
    signal.total_length = EI_CLASSIFIER_RAW_SAMPLE_COUNT;
    signal.get_data = &microphone_audio_signal_get_data;

    ei_impulse_result_t result = { 0 };
    EI_IMPULSE_ERROR run_result = run_classifier(&signal, &result, debug_nn);
    if (run_result != EI_IMPULSE_OK) {
        ei_printf("ERR: Failed to run classifier (%d)\r\n", run_result);
        return;
    }

    print_inference_result(result);
    publish_ble_result(result);
}

static bool setup_ble(void)
{
    if (!BLE.begin()) {
        return false;
    }

    BLE.setLocalName(BLE_DEVICE_NAME);
    BLE.setDeviceName(BLE_DEVICE_NAME);
    BLE.setAdvertisedService(inferenceService);

    inferenceService.addCharacteristic(topResultCharacteristic);
    inferenceService.addCharacteristic(scoresCharacteristic);
    inferenceService.addCharacteristic(inferenceCountCharacteristic);
    BLE.addService(inferenceService);

    topResultCharacteristic.writeValue("waiting,0.000");
    scoresCharacteristic.writeValue("waiting");
    inferenceCountCharacteristic.writeValue((unsigned int)0);

    BLE.advertise();
    return true;
}

static void poll_ble_for(uint32_t duration_ms)
{
    uint32_t start = millis();
    while ((millis() - start) < duration_ms) {
        BLE.poll();
        delay(10);
    }
}

static size_t advance_used(size_t used, int written, size_t capacity)
{
    if (written <= 0) {
        return used;
    }

    size_t available = capacity - used;
    if ((size_t)written >= available) {
        return capacity - 1;
    }

    return used + (size_t)written;
}

static void publish_ble_result(const ei_impulse_result_t &result)
{
    const char *best_label = result.classification[0].label;
    float best_value = result.classification[0].value;

    for (uint16_t ix = 1; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
        if (result.classification[ix].value > best_value) {
            best_label = result.classification[ix].label;
            best_value = result.classification[ix].value;
        }
    }

    inference_count++;

    char top_payload[64];
    snprintf(top_payload, sizeof(top_payload), "%u,%s,%.3f",
             inference_count, best_label, best_value);
    topResultCharacteristic.writeValue(top_payload);

    char scores_payload[BLE_PAYLOAD_LEN + 1];
    size_t used = 0;
    int written = snprintf(scores_payload, sizeof(scores_payload), "%u", inference_count);
    used = advance_used(used, written, sizeof(scores_payload));

    for (uint16_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
        written = snprintf(scores_payload + used, sizeof(scores_payload) - used,
                           "%c%s=%.3f",
                           ix == 0 ? ':' : ',',
                           result.classification[ix].label,
                           result.classification[ix].value);
        used = advance_used(used, written, sizeof(scores_payload));
    }

#if EI_CLASSIFIER_HAS_ANOMALY == 1
    written = snprintf(scores_payload + used, sizeof(scores_payload) - used,
                       ",anomaly=%.3f", result.anomaly);
    used = advance_used(used, written, sizeof(scores_payload));
#endif

    scores_payload[sizeof(scores_payload) - 1] = '\0';
    scoresCharacteristic.writeValue(scores_payload);
    inferenceCountCharacteristic.writeValue(inference_count);
    BLE.poll();
}

static bool microphone_inference_start(uint32_t n_samples)
{
    uint32_t capture_samples = (uint32_t)(((uint64_t)n_samples * PDM_CAPTURE_SAMPLE_RATE) / EI_CLASSIFIER_FREQUENCY);

    inference.buffer = (int16_t *)malloc(n_samples * sizeof(int16_t));
    if (inference.buffer == NULL) {
        return false;
    }

    inference.buf_count = 0;
    inference.capture_count = 0;
    inference.n_samples = n_samples;
    inference.capture_samples = capture_samples;
    inference.buf_ready = 0;
    inference.resample_group_count = 0;

    PDM.onReceive(&pdm_data_ready_inference_callback);
    PDM.setBufferSize(sizeof(sampleBuffer));

    if (!PDM.begin(1, PDM_CAPTURE_SAMPLE_RATE)) {
        microphone_inference_end();
        return false;
    }

    PDM.setGain(127);
    record_ready = false;

    return true;
}

static bool microphone_inference_record(void)
{
    inference.buf_ready = 0;
    inference.buf_count = 0;
    inference.capture_count = 0;
    inference.resample_group_count = 0;
    record_ready = true;

    while (inference.buf_ready == 0) {
        BLE.poll();
        static uint32_t last_record_status = 0;
        if ((millis() - last_record_status) >= 1000) {
            ei_printf("Recording samples: %lu/%lu\r\n",
                      (unsigned long)inference.capture_count,
                      (unsigned long)inference.capture_samples);
            last_record_status = millis();
        }
        delay(10);
    }

    record_ready = false;
    return true;
}

static int microphone_audio_signal_get_data(size_t offset, size_t length, float *out_ptr)
{
    numpy::int16_to_float(&inference.buffer[offset], out_ptr, length);
    return 0;
}

static void microphone_inference_end(void)
{
    PDM.end();
    free(inference.buffer);
    inference.buffer = NULL;
}

static void pdm_data_ready_inference_callback(void)
{
    int bytes_available = PDM.available();
    if (bytes_available > (int)sizeof(sampleBuffer)) {
        bytes_available = sizeof(sampleBuffer);
    }

    int bytes_read = PDM.read((char *)&sampleBuffer[0], bytes_available);
    int samples_read = bytes_read / 2;

    if (!record_ready || inference.buf_ready) {
        return;
    }

    for (int i = 0; i < samples_read; i++) {
        inference.capture_count++;
        inference.resample_group[inference.resample_group_count++] = sampleBuffer[i];

        if (inference.resample_group_count == 4) {
            if ((inference.buf_count + 3) <= inference.n_samples) {
                int32_t s0 = inference.resample_group[0];
                int32_t s1 = inference.resample_group[1];
                int32_t s2 = inference.resample_group[2];
                int32_t s3 = inference.resample_group[3];

                inference.buffer[inference.buf_count++] = (int16_t)s0;
                inference.buffer[inference.buf_count++] = (int16_t)((2 * s1 + s2) / 3);
                inference.buffer[inference.buf_count++] = (int16_t)((s2 + 2 * s3) / 3);
            }

            inference.resample_group_count = 0;
        }

        if (inference.buf_count >= inference.n_samples || inference.capture_count >= inference.capture_samples) {
            inference.buf_ready = 1;
            break;
        }
    }
}

static void print_inference_result(const ei_impulse_result_t &result)
{
    ei_printf("Timing: DSP %d ms, inference %d ms, anomaly %d ms\r\n",
              result.timing.dsp,
              result.timing.classification,
              result.timing.anomaly);

    ei_printf("#Classification predictions:\r\n");
    for (uint16_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
        ei_printf("    %s: %.6f\r\n",
                  result.classification[ix].label,
                  result.classification[ix].value);
    }

#if EI_CLASSIFIER_HAS_ANOMALY == 1
    ei_printf("    anomaly score: %.6f\r\n", result.anomaly);
#endif
}

void ei_printf(const char *format, ...)
{
    static char print_buf[1024];

    va_list args;
    va_start(args, format);
    int written = vsnprintf(print_buf, sizeof(print_buf), format, args);
    va_end(args);

    if (written <= 0) {
        return;
    }

    size_t len = (written < (int)sizeof(print_buf)) ? (size_t)written : sizeof(print_buf) - 1;
    Serial.write((const uint8_t *)print_buf, len);
    Serial.flush();
}
