#include <stdio.h>
#include "esp_log.h"
#include "driver/mcpwm_prelude.h"
#include "pwm.h"

static const char *TAG = "PWM";

static mcpwm_cmpr_handle_t comparator = NULL;

void pwm_init(int pulse_gpio, uint32_t timer_resolution, uint32_t timer_period)
{
    // Create the new timer
    mcpwm_timer_handle_t timer = NULL;
    mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = timer_resolution,
        .period_ticks = timer_period,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));

    // Create the MCPWM operator
    mcpwm_oper_handle_t oper = NULL;
    mcpwm_operator_config_t operator_config = {
        .group_id = 0, // operator must be in the same group to the timer
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &oper));

    // Connect the timer and operator
    ESP_LOGI(TAG, "Connect timer and operator");
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timer));

    // Create comparator and generator from the operator
    ESP_LOGI(TAG, "Create comparator and generator from the operator");
    mcpwm_comparator_config_t comparator_config = {
        .flags.update_cmp_on_tez = true,
    };
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &comparator_config, &comparator));

    mcpwm_gen_handle_t generator = NULL;
    mcpwm_generator_config_t generator_config = {
        .gen_gpio_num = pulse_gpio,
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &generator_config, &generator));

    // Define what the generator should do
    ESP_LOGI(TAG, "Set generator action on timer and compare event");
    // go high on counter empty
    ESP_ERROR_CHECK(mcpwm_generator_set_actions_on_timer_event(generator,
                                                               MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                                                                            MCPWM_TIMER_EVENT_EMPTY,
                                                                                            MCPWM_GEN_ACTION_HIGH),
                                                               MCPWM_GEN_TIMER_EVENT_ACTION_END()));
    // go low on compare threshold
    ESP_ERROR_CHECK(mcpwm_generator_set_actions_on_compare_event(generator,
                                                                 MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                                                                                comparator,
                                                                                                MCPWM_GEN_ACTION_LOW),
                                                                 MCPWM_GEN_COMPARE_EVENT_ACTION_END()));

    // Set an initial compare value that will result in no output
    set_pulse_width(timer_period);

    // Start the timer
    ESP_LOGI(TAG, "Enable and start timer");
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));
};

void set_pulse_width(uint32_t cmp_ticks)
{
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, cmp_ticks));
};