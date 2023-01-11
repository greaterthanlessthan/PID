#include "freertos/FreeRTOS.h"
#include <freertos/task.h>
#include <freertos/timers.h>
#include <freertos/portmacro.h> // not actually necessary but makes intellisense shut up

#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "esp_system.h"

#include "pwm_slow.h"

#define LOG_MEM_ALLOC_ERROR                                               \
    ESP_LOGE(TAG, "Could not allocate memory, see %s, line %d", __FILE__, \
             __LINE__);

static char *TAG = "SLOW_PWM";

static void
init_gpio(int pin)
{
    // zero-initialize the config structure.
    gpio_config_t io_conf = {};
    // disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    // set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    // bit mask of the pins that you want to set
    io_conf.pin_bit_mask = (1ULL << pin);
    // disable pull-down mode
    io_conf.pull_down_en = 0;
    // disable pull-up mode
    io_conf.pull_up_en = 0;
    // configure GPIO with the given settings
    gpio_config(&io_conf);
};

// static bool timer_alarm_handler_gpio_low(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *pin)
//{
//     int *gpio_num = (int *)pin;
//     gpio_set_level(*gpio_num, 0);
//     return 0;
// }
//
// static bool timer_alarm_handler_gpio_high(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *pin)
//{
//     int *gpio_num = (int *)pin;
//     gpio_set_level(*gpio_num, 1);
//     return 0;
// }

static void slow_pwm_task(void *_params)
{
    pwm_slow_params *params = (pwm_slow_params *)_params;
    TickType_t xLastWakeTime = xTaskGetTickCount();

    uint16_t period_high;
    uint16_t period_low;

    init_gpio(params->pwm_pin);

    //    gptimer_handle_t gptimer = NULL;
    //    gptimer_config_t timer_config = {
    //        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
    //        .direction = GPTIMER_COUNT_UP,
    //        .resolution_hz = 100000, // 100kHz, 1 tick = 10us
    //    };
    //    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer));
    //
    //    gptimer_alarm_config_t alarm_config = {
    //        .alarm_count = 50000,                // period = 0.5s @resolution 100kHz
    //        .flags.auto_reload_on_alarm = false, // disable auto-reload
    //    };
    //    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_config));
    //
    //    gptimer_event_callbacks_t cbs = {
    //        .on_alarm = timer_alarm_handler_gpio_low, // register user callback
    //    };
    //    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &cbs, &params->pwm_pin));
    //
    //    gptimer_alarm_config_t alarm_config2 = {
    //        .reload_count = 0,
    //        .alarm_count = 100000,              // period = 0.5s @resolution 100kHz
    //        .flags.auto_reload_on_alarm = true, // disable auto-reload
    //    };
    //    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_config2));
    //
    //    gptimer_event_callbacks_t cbs2 = {
    //        .on_alarm = timer_alarm_handler_gpio_high, // register user callback
    //    };
    //    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &cbs2, &params->pwm_pin));
    //
    //    ESP_ERROR_CHECK(gptimer_enable(gptimer));
    //    ESP_ERROR_CHECK(gptimer_start(gptimer));
    //
    //    ESP_LOGI(TAG, "Started the timer");

    for (;;)
    {
        gpio_set_level(params->pwm_pin, 1);

        ESP_LOGV(TAG, "Task has %d words remaining in stack", uxTaskGetStackHighWaterMark(NULL));

        // calculate how long GPIO should remain high and how long it should remain low
        period_high = (params->duty_cycle / 100.0) * params->period;

        // round up to minimum period or round down to 0
        if (period_high < params->min_pulse_width)
        {
            period_high = period_high > (params->min_pulse_width / 2) ? params->min_pulse_width : 0;
        }

        period_low = params->period - period_high;

        // pause for set amount of time while GPIO pin high
        if (pdMS_TO_TICKS(period_high) > 0)
        {
            ESP_LOGD(TAG, "GPIO high for %d ms", period_high);
            xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(period_high));
            xLastWakeTime = xTaskGetTickCount();
        }

        gpio_set_level(params->pwm_pin, 0);

        if (pdMS_TO_TICKS(period_low) > 0)
        {
            // pause for set amount of time while GPIO low
            ESP_LOGD(TAG, "GPIO low for %d ms", period_low);
            xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(period_low));
            xLastWakeTime = xTaskGetTickCount();
        }
    }
};

void start_slow_pwm_task(pwm_slow_params *params)
{
    BaseType_t task_ret;

    // start task
    task_ret = xTaskCreate(slow_pwm_task, "slow_pwm_task", 2000,
                           (void *)params, 20, NULL);
    if (task_ret == pdPASS)
    {
        ESP_LOGI(TAG, "Started the slow PWM task");
    }
    else
    {
        LOG_MEM_ALLOC_ERROR
    }
};