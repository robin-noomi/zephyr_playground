#include <zephyr.h>
//#include "ei_run_classifier.h"
#include "numpy.hpp"

#include <sys/printk.h>
#include "ei_device_nordic_nrf52.h"
#include "ei_zephyr_flash_commands.h"
#include "ei_main.h"
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>

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


const struct device *led0_dev;
const struct device *led3_dev;


int main(void)
{
    int ret;

    led0_dev = device_get_binding(LED0_LABEL);
    if (led0_dev == NULL) {
            return -1;
    }

    led3_dev = device_get_binding(LED3_LABEL);
    if (led3_dev == NULL) {
            return -1;
    }

    ret = gpio_pin_configure(led0_dev, LED0_PIN, GPIO_OUTPUT_ACTIVE | LED0_FLAGS);
    if (ret < 0) {
            return ret;
    }

    ret = gpio_pin_configure(led3_dev, LED3_PIN, GPIO_OUTPUT_ACTIVE | LED3_FLAGS);
    if (ret < 0) {
            return ret;
    }

        
    //gpio_pin_set(led0_dev, LED0_PIN, 1);
    //gpio_pin_set(led3_dev, LED3_PIN, 1);

    /* This is needed so that output of printf 
       is output immediately without buffering 
    */
    setvbuf(stdout, NULL, _IONBF, 0);

    /* Initialize board uart */
    if(uart_init() != 0){
        ei_printf("Init uart on board error occured\r\n");
    }

    /** Initialize development board LEDs */
    if(BOARD_ledInit() != 0){
        ei_printf("Init LEDs on board error occured\r\n");
    }

    /* Initialize Zephyr flash device */
    create_flash_device();

    /* Initialize Edge Impuls sensors and commands */
    ei_init();

    while(1){
        ei_main();
    }
}
