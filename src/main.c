/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
//#include <model-parameters/model_metadata.h>
//#include <edge_impulse/src/edge_impulse_project/model-parameters/model_metadata.h>
//#include "edge_impulse/src/edge_impulse_project/model-parameters/model_metadata.h"

//#include <ei_run_classifier.h>
//#include "model-parameters/model_metadata.h"

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)
#define LED3_NODE DT_ALIAS(led3)

#if DT_NODE_HAS_STATUS(LED0_NODE, okay)
#define LED0_LABEL	DT_GPIO_LABEL(LED0_NODE, gpios)
#define LED0_PIN	DT_GPIO_PIN(LED0_NODE, gpios)
#define LED0_FLAGS	DT_GPIO_FLAGS(LED0_NODE, gpios)
#else
/* A build error here means your board isn't set up to blink an LED. */
#error "Unsupported board: led0 devicetree alias is not defined"
#define LED0_LABEL	""
#define LED0_PIN	0
#define LED0_FLAGS	0
#endif

#if DT_NODE_HAS_STATUS(LED3_NODE, okay)
#define LED3_LABEL	DT_GPIO_LABEL(LED3_NODE, gpios)
#define LED3_PIN	DT_GPIO_PIN(LED3_NODE, gpios)
#define LED3_FLAGS	DT_GPIO_FLAGS(LED3_NODE, gpios)
#else
/* A build error here means your board isn't set up to blink an LED. */
#error "Unsupported board: led3 devicetree alias is not defined"
#define LED3_LABEL	""
#define LED3_PIN	0
#define LED3_FLAGS	0
#endif



#include <ei_wrapper.h>
//#include <ei_microphone.h>
#include "include/ei_microphone.h"
#include <string.h>

//#include "input_data.h"

#define FRAME_ADD_INTERVAL_MS	100

static size_t frame_surplus;
volatile bool prediction_running = false;

const struct device *led0_dev;
const struct device *led3_dev;


static void result_ready_cb(int err)
{
	if (err) {
		printk("Result ready callback returned error (err: %d)\n", err);
		return;
	}

	const char *label;
	float value;
	float anomaly;

	err = ei_wrapper_get_classification_results(&label, &value, &anomaly);

	if (err) {
		printk("Cannot get classification results (err: %d)", err);
	} else {
		printk("\nClassification results\n");
		printk("======================\n");
		printk("Label: %s\n", label);
		printk("Value: %.2f\n", value);
		if (ei_wrapper_classifier_has_anomaly()) {
			printk("Anomaly: %.2f\n", anomaly);
		}

                if (0 == strcmp("help", label))
                {		
                    gpio_pin_set(led0_dev, LED0_PIN, 1);
                }
                else if (0 == strcmp("nectarine", label))
                {
                    gpio_pin_set(led3_dev, LED3_PIN, 1);
                }

	}

	if (frame_surplus > 0) {
		err = ei_wrapper_start_prediction(0, 1);
		if (err) {
			printk("Cannot restart prediction (err: %d)\n", err);
                        prediction_running = false;
		} else {
			printk("Prediction restarted...\n");
                        prediction_running = true;
		}

		frame_surplus--;
	}
        else
        {
            prediction_running = false;
        }
}


#define EI_CLASSIFIER_INTERVAL_MS                0.0625
#define EI_CLASSIFIER_RAW_SAMPLE_COUNT           16000
#define EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME      1
#define EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE       (EI_CLASSIFIER_RAW_SAMPLE_COUNT * EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME)

float mic_buffer[EI_CLASSIFIER_RAW_SAMPLE_COUNT] = {0};
//float mic_buffer[5];

static void run_nn(void)
{
    // summary of inferencing settings (from model_metadata.h)
    printk("Inferencing settings:\n");
    printk("\tInterval: %f", (float)EI_CLASSIFIER_INTERVAL_MS);
    printk("ms.\n");
    printk("\tFrame size: %d\n", EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE);
    printk("\tSample length: %d ms.\n", EI_CLASSIFIER_RAW_SAMPLE_COUNT / 16);
    //printk("\tNo. of classes: %d\n", sizeof(ei_classifier_inferencing_categories) /
    //                                        sizeof(ei_classifier_inferencing_categories[0]));

    if (ei_microphone_inference_started())
    {
        ei_microphone_inference_end();
    }

    if (ei_microphone_inference_start(EI_CLASSIFIER_RAW_SAMPLE_COUNT) == false) {
        printk("ERR: Failed to setup audio sampling\r\n");
        return;
    }

    printk("Starting inferencing\n");

    while (1) {
        if (prediction_running)
        {
            // Wait for prediction to finish
            return;
        }
        printk("Starting inferencing in 2 s...\n");

        int err;
        bool cancelled = false;
        err = ei_wrapper_clear_data(&cancelled);
        if (0 != err)
        {
            printk("ei wrapper clear data failed, err_code:%d \n", err);
        }
        if (cancelled)
        {
            printk("Prediction was cancelled");
        }

        // instead of wait_ms, we'll wait on the signal, this allows threads to cancel us...
        k_msleep(2000);

        //if(ei_user_invoke_stop()) {
        //    printk("Inferencing stopped by user\r\n");
        //    break;
        //}

        printk("Recording...\n");

        ei_microphone_inference_reset_buffers();
        bool m = ei_microphone_inference_record();
        if (!m) {
            printk("ERR: Failed to record audio...\n");
            break;
        }

        printk("Recording done\n");

        //signal_t signal;
        //signal.total_length = EI_CLASSIFIER_RAW_SAMPLE_COUNT;
        //signal.get_data = &ei_microphone_audio_signal_get_data;
        //ei_impulse_result_t result = {0};

        //EI_IMPULSE_ERROR r = run_classifier(&signal, &result, debug);
        //if (r != EI_IMPULSE_OK) {
        //    printk("ERR: Failed to run classifier (%d)\n", r);
        //    break;
        //}

        //// print the predictions
        //printk("Predictions (DSP: %d ms., Classification: %d ms., Anomaly: %d ms.): \n",
        //    result.timing.dsp, result.timing.classification, result.timing.anomaly);
        //for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
        //    printk("    %s: \t", result.classification[ix].label);
        //    printk(result.classification[ix].value);
        //    printk("\r\n");
        //}

        ei_microphone_audio_signal_get_data(0, EI_CLASSIFIER_RAW_SAMPLE_COUNT, mic_buffer);


	if (ARRAY_SIZE(mic_buffer) < ei_wrapper_get_window_size()) {
		printk("Not enough input data\n");
		return;
	}

	if (ARRAY_SIZE(mic_buffer) % ei_wrapper_get_frame_size() != 0) {
		printk("Improper number of input samples\n");
		return;
	}

	size_t cnt = 0;

	err = ei_wrapper_add_data(&mic_buffer[cnt],
				  ei_wrapper_get_window_size());
	if (err) {
		printk("Cannot provide input data (err: %d)\n", err);
		printk("Increase CONFIG_EI_WRAPPER_DATA_BUF_SIZE\n");
		return;
	}
	cnt += ei_wrapper_get_window_size();

	err = ei_wrapper_start_prediction(0, 0);
	if (err) {
		printk("Cannot start prediction (err: %d)\n", err);
	} else {
		printk("Prediction started...\n");
                prediction_running = true;
	}

	/* Predictions for additional data are triggered in the result ready
	 * callback. The prediction start can be triggered before the input
	 * data is provided. In that case the prediction is started right
	 * after the prediction window is filled with data.
	 */         

	frame_surplus = (ARRAY_SIZE(mic_buffer) - ei_wrapper_get_window_size())
			/ ei_wrapper_get_frame_size();


	while (cnt < ARRAY_SIZE(mic_buffer)) {
		err = ei_wrapper_add_data(&mic_buffer[cnt],
					  ei_wrapper_get_frame_size());
		if (err) {
			printk("Cannot provide input data (err: %d)\n", err);
			printk("Increase CONFIG_EI_WRAPPER_DATA_BUF_SIZE\n");
			return;
		}
		cnt += ei_wrapper_get_frame_size();

		//k_sleep(K_MSEC(FRAME_ADD_INTERVAL_MS));
	}


#if EI_CLASSIFIER_HAS_ANOMALY == 1
        printk("    anomaly score: ");
        printk(result.anomaly);
        printk("\r\n");
#endif

        //if(ei_user_invoke_stop()) {
        //    printk("Inferencing stopped by user\r\n");
        //    break;
        //}
    }

    ei_microphone_inference_end();
}

void main(void)
{

    ///* Initialize board uart */
    //if(uart_init() != 0){
    //    ei_printf("Init uart on board error occured\r\n");
    //}

    ///** Initialize development board LEDs */
    //if(BOARD_ledInit() != 0){
    //    ei_printf("Init LEDs on board error occured\r\n");
    //}

    ///* Initialize Zephyr flash device */
    //create_flash_device();

    ///* Initialize Edge Impuls sensors and commands */
    //ei_init();

    //while(1){
    //    ei_main();
    //}

	//bool led_is_on = true;
	int ret;

	led0_dev = device_get_binding(LED0_LABEL);
	if (led0_dev == NULL) {
		return;
	}
        
        led3_dev = device_get_binding(LED3_LABEL);
	if (led3_dev == NULL) {
		return;
	}

	ret = gpio_pin_configure(led0_dev, LED0_PIN, GPIO_OUTPUT_ACTIVE | LED0_FLAGS);
	if (ret < 0) {
		return;
	}

        ret = gpio_pin_configure(led3_dev, LED3_PIN, GPIO_OUTPUT_ACTIVE | LED3_FLAGS);
	if (ret < 0) {
		return;
	}

        /* Setup the microphone sensor */
        ei_microphone_init();

	int err = ei_wrapper_init(result_ready_cb);

	if (err) {
		printk("Edge Impulse wrapper failed to initialize (err: %d)\n",
		       err);
		return;
	};

	//if (ARRAY_SIZE(input_data) < ei_wrapper_get_window_size()) {
	//	printk("Not enough input data\n");
	//	return;
	//}

	//if (ARRAY_SIZE(input_data) % ei_wrapper_get_frame_size() != 0) {
	//	printk("Improper number of input samples\n");
	//	return;
	//}

	//size_t cnt = 0;

	/* input_data is defined in input_data.h file. */
	//err = ei_wrapper_add_data(&input_data[cnt],
	//			  ei_wrapper_get_window_size());
	//if (err) {
	//	printk("Cannot provide input data (err: %d)\n", err);
	//	printk("Increase CONFIG_EI_WRAPPER_DATA_BUF_SIZE\n");
	//	return;
	//}
	//cnt += ei_wrapper_get_window_size();

	//err = ei_wrapper_start_prediction(0, 0);
	//if (err) {
	//	printk("Cannot start prediction (err: %d)\n", err);
	//} else {
	//	printk("Prediction started...\n");
	//}

	/* Predictions for additional data are triggered in the result ready
	 * callback. The prediction start can be triggered before the input
	 * data is provided. In that case the prediction is started right
	 * after the prediction window is filled with data.
	 */
	//frame_surplus = (ARRAY_SIZE(input_data) - ei_wrapper_get_window_size())
	//		/ ei_wrapper_get_frame_size();

	//while (cnt < ARRAY_SIZE(input_data)) {
	//	err = ei_wrapper_add_data(&input_data[cnt],
	//				  ei_wrapper_get_frame_size());
	//	if (err) {
	//		printk("Cannot provide input data (err: %d)\n", err);
	//		printk("Increase CONFIG_EI_WRAPPER_DATA_BUF_SIZE\n");
	//		return;
	//	}
	//	cnt += ei_wrapper_get_frame_size();

	//	k_sleep(K_MSEC(FRAME_ADD_INTERVAL_MS));
	//}

        gpio_pin_set(led0_dev, LED0_PIN, 0);
        gpio_pin_set(led3_dev, LED3_PIN, 0);
	while (1) {
                run_nn();

		//gpio_pin_set(led0_dev, LED0_PIN, (int)led_is_on);
		//gpio_pin_set(led3_dev, LED3_PIN, (int)!led_is_on);
		//led_is_on = !led_is_on;
		//k_msleep(SLEEP_TIME_MS);
                k_yield();
	}
}
