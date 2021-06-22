/* Edge Impulse ingestion SDK
 * Copyright (c) 2021 EdgeImpulse Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/* Include ----------------------------------------------------------------- */
#include "ei_microphone.h"
//#include "ei_device_nordic_nrf52.h"
//#include "ei_classifier_porting.h"

//#include "ei_config_types.h"
//#include "sensor_aq_mbedtls_hs256.h"
//#include "sensor_aq_none.h"

#include <zephyr.h>
#include <nrfx_pdm.h>
#include <string.h>

/* Read nordic id from the registers */
#if ((CONFIG_SOC_NRF52840 == 1) || \
       (CONFIG_SOC_NRF52840_QIAA == 1))
#define PDM_CLK_PIN                         36// 32+4 = p1.04
#define PDM_DIN_PIN                         37// 32+5 = p1.05
#elif ((CONFIG_SOC_NRF5340_CPUAPP == 1) || \
       (CONFIG_SOC_NRF5340_CPUAPP_QKAA == 1))
#define PDM_CLK_PIN                         37// 32+5 = p1.05
#define PDM_DIN_PIN                         38// 32+6 = p1.06
#else 
#error "Unsupported build target was chosen!"
#endif

/* Audio sampling config */
#define AUDIO_SAMPLING_FREQUENCY            16000
#define AUDIO_SAMPLES_PER_MS                (AUDIO_SAMPLING_FREQUENCY / 1000)
#define AUDIO_DSP_SAMPLE_LENGTH_MS          16
#define AUDIO_DSP_SAMPLE_RESOLUTION         (sizeof(short))
#define AUDIO_DSP_SAMPLE_BUFFER_SIZE        (AUDIO_SAMPLES_PER_MS * AUDIO_DSP_SAMPLE_LENGTH_MS * AUDIO_DSP_SAMPLE_RESOLUTION) //4096

/* Buffers for receiving PDM mic data */
static int16_t pdm_buffer_temp[2][AUDIO_DSP_SAMPLE_BUFFER_SIZE] = {0};
int16_t *current_buff;
volatile bool write_data = false;
bool microphone_started = false;

/** Status and control struct for inferencing struct */
typedef struct {
    int16_t *buffers[2];
    uint8_t buf_select;
    uint8_t buf_ready;
    uint32_t buf_count;
    uint32_t n_samples;
} inference_t;

//extern ei_config_t *ei_config_get_config();

///* Dummy functions for sensor_aq_ctx type */
//static size_t ei_write(const void*, size_t size, size_t count, EI_SENSOR_AQ_STREAM*)
//{
//    return count;
//}

//static int ei_seek(EI_SENSOR_AQ_STREAM*, long int offset, int origin)
//{
//    return 0;
//}

/* Private variables ------------------------------------------------------- */
static bool record_ready = false;

static volatile inference_t inference;

//static unsigned char ei_mic_ctx_buffer[1024];
//static sensor_aq_signing_ctx_t ei_mic_signing_ctx;
//static sensor_aq_mbedtls_hs256_ctx_t ei_mic_hs_ctx;
//static sensor_aq_ctx ei_mic_ctx = {
//    { ei_mic_ctx_buffer, 1024 },
//    &ei_mic_signing_ctx,
//    &ei_write,
//    &ei_seek,
//    NULL,
//};

/* Private functions ------------------------------------------------------- */

/**
 * @brief      Inference audio callback, store samples in ram buffer
 *             Signal when buffer is full, and swap buffers
 * @param      buffer   Pointer to source buffer
 * @param[in]  n_bytes  Number of bytes to write
 */
static void audio_buffer_inference_callback(void *buffer, uint32_t n_bytes)
{
    int16_t *samples = (int16_t *)buffer;

    for(uint32_t i = 0; i < (n_bytes >> 1); i++) {
        inference.buffers[inference.buf_select][inference.buf_count++] = samples[i];

        if(inference.buf_count >= inference.n_samples) {
            inference.buf_select ^= 1;
            inference.buf_count = 0;
            inference.buf_ready = 1;
        }
    }
}

/**
 * @brief      PDM receive data handler
 * @param[in]  p_evt  pdm event structure
 */
static void pdm_data_handler(nrfx_pdm_evt_t const * p_evt)
{
    nrfx_err_t err = NRFX_SUCCESS;
    static uint8_t buf_toggle = 0;

    if(p_evt->error != 0){
        printk("PDM handler error ocured\n");
        printk("pdm_data_handler error: %d, %d  \n", p_evt->error, p_evt->buffer_requested);
        return;
    }
    if(true == p_evt->buffer_requested){
        buf_toggle ^= 1;
        err = nrfx_pdm_buffer_set(pdm_buffer_temp[buf_toggle], AUDIO_DSP_SAMPLE_BUFFER_SIZE);
        if(err != NRFX_SUCCESS){
            printk("PDM buffer init error: %d\n", err);
        }
    }
    if(p_evt->buffer_released != NULL){
            write_data = true;
            current_buff = pdm_buffer_temp[buf_toggle];
    }
}

/**
 * @brief      Capture 2 channel pdm data every 100 ms.
 *             Waits for new data to be ready.
 *             Creates a 1 channel pdm array and calls callback function
 * @param[in]  callback  Callback needs to handle the audio samples
 */
static void get_dsp_data(void (*callback)(void *buffer, uint32_t n_bytes))
{
    //TODO: check the number of bytes
    //printk("1 write_data:%d", write_data);
    if(write_data ==true){
            //printk("Calling inference callback");
            callback((void *)current_buff, sizeof(pdm_buffer_temp)/2);
        write_data = false;
    }
    //printk("2 write_data:%d \r\n", write_data);
}

/* Public functions -------------------------------------------------------- */

/**
 * @brief      Set the PDM mic to +34dB
 */
void ei_microphone_init(void)
{
    nrfx_err_t err;

    /* PDM driver configuration */
    nrfx_pdm_config_t config_pdm = NRFX_PDM_DEFAULT_CONFIG(PDM_CLK_PIN, PDM_DIN_PIN);
    config_pdm.clock_freq = NRF_PDM_FREQ_1280K;
    config_pdm.ratio = NRF_PDM_RATIO_80X;
    config_pdm.edge = NRF_PDM_EDGE_LEFTRISING;
    config_pdm.gain_l = NRF_PDM_GAIN_MAXIMUM;
    config_pdm.gain_r = NRF_PDM_GAIN_MAXIMUM;

    /* PDM interrupt configuration necessary for Zephyr */
#if ((CONFIG_SOC_NRF52840 == 1) || \
       (CONFIG_SOC_NRF52840_QIAA == 1))
    IRQ_DIRECT_CONNECT(PDM_IRQn, 6, nrfx_pdm_irq_handler, 0);
#elif ((CONFIG_SOC_NRF5340_CPUAPP == 1) || \
       (CONFIG_SOC_NRF5340_CPUAPP_QKAA == 1))
    IRQ_DIRECT_CONNECT(PDM0_IRQn, 6, nrfx_pdm_irq_handler, 0);
#else 
#error "Unsupported build target was chosen!"
#endif

    err = nrfx_pdm_init(&config_pdm, pdm_data_handler);
    if(err != NRFX_SUCCESS){
        printk("PDM init error: %d\n", err);
    }
    else{
        printk("PDM init OK\n");
    }
}

bool ei_microphone_inference_start(uint32_t n_samples)
{
    nrfx_err_t err;

    inference.buffers[0] = (int16_t *)malloc(n_samples * sizeof(int16_t));

    if(inference.buffers[0] == NULL) {
        return false;
    }

    inference.buffers[1] = (int16_t *)malloc(n_samples * sizeof(int16_t));

    if(inference.buffers[1] == NULL) {
        free(inference.buffers[0]);
        return false;
    }

    inference.buf_select = 0;
    inference.buf_count  = 0;
    inference.n_samples  = n_samples;
    inference.buf_ready  = 0;

    err = nrfx_pdm_start();
    if(err != NRFX_SUCCESS){
        return false;
    }

    microphone_started = true;

    return true;
}

bool ei_microphone_inference_record(void)
{
    while (inference.buf_ready == 0) {
        get_dsp_data(&audio_buffer_inference_callback);
    };
 
    inference.buf_ready = 0;

    return true;
}

/**
 * @brief      Reset buffer counters for non-continuous inferecing
 */
void ei_microphone_inference_reset_buffers(void)
{
    inference.buf_ready = 0;
    inference.buf_count = 0;
}

/**
 * Get raw audio signal data
 */
int ei_microphone_audio_signal_get_data(size_t offset, size_t length, float *out_ptr)
{
    size_t i;

    for(i = 0; i < length; i++) {
        *(out_ptr + i) = (float)inference.buffers[inference.buf_select ^ 1][offset + i]
        / ((float)(1 << 15));
    }

    return 0;
}

bool ei_microphone_inference_end(void)
{
    uint32_t nrfx_err;
    record_ready = false;

    nrfx_err = nrfx_pdm_stop();
    if(nrfx_err != NRFX_SUCCESS)
    {
        printk("PDM Could not stop PDM sampling, error = %d", nrfx_err);
    }
    else
    {
        microphone_started = false;
    }

    free(inference.buffers[0]);
    free(inference.buffers[1]);
    return true;
}

bool ei_microphone_inference_started(void)
{
    return microphone_started;
}
