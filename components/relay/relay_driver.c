#include <stdbool.h>
#include <ulp_lp_core_gpio.h>
#include <soc/gpio_num.h>
#include "hal/gpio_ll.h"
#include "soc/gpio_struct.h"
#include "soc/io_mux_reg.h"

#include "relay_driver.h"

/* LP GPIOs are 0-7; GPIO 8+ are HP-only and need the HP GPIO peripheral */
#define IS_LP_GPIO(n) ((n) >= GPIO_NUM_0 && (n) <= GPIO_NUM_7)

void relay_driver_init(int gpio_num)
{
    if (IS_LP_GPIO(gpio_num)) {
        ulp_lp_core_gpio_init(gpio_num);
        ulp_lp_core_gpio_output_enable(gpio_num);
        ulp_lp_core_gpio_input_disable(gpio_num);
        ulp_lp_core_gpio_set_output_mode(gpio_num, RTCIO_LL_OUTPUT_NORMAL);
    } else {
        /* Route GPIO matrix output to the physical pad, then enable output */
        PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[gpio_num], PIN_FUNC_GPIO);
        gpio_ll_output_enable(&GPIO, gpio_num);
    }
}

void relay_driver_set_power(int gpio_num, bool power)
{
    if (IS_LP_GPIO(gpio_num)) {
        ulp_lp_core_gpio_set_level(gpio_num, power);
    } else {
        gpio_ll_set_level(&GPIO, gpio_num, power);
    }
}
