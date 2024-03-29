#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_system.h"
#include "soc/gpio_struct.h"
#include <esp_log.h>
#include "nvs_flash.h"

#include "max31856.h"
#include "PID.h"
#include "adc.h"
#include "pwm.h"

#include "pwm_slow.h"
#include "wifi.h"

#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK 18
#define PIN_CS 5
#define MAX_SPI_HOST VSPI_HOST

#define DRDY_PIN 21

#define PWM_PULSE_GPIO 22                  // GPIO connects to the PWM signal line
#define PWM_TIMEBASE_RESOLUTION_HZ 1000000 // 1MHz, 1us per tick
#define PWM_TIMEBASE_PERIOD 20000          // 20000 ticks, 20ms

// Because I want folding dammit
#pragma GCC diagnostic ignored "-Wunknown-pragmas"

static const char *TAG = "MAIN";

void update_pwm_with_cv(void *_pid);
void receive_temp_to_pv(void *_pid);

pwm_slow_params pwm_params = {
    .period = 300,
    .duty_cycle = 0,
    .min_pulse_width = 17,
    .pwm_pin = PWM_PULSE_GPIO,
};

void app_main(void)
{
    // Setup the temperature reading from SPI chip
#pragma region

    // Bus cfg
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 0,
    };

    // Device cfg
    spi_device_interface_config_t devcfg = max31856_device_interface(PIN_CS);
    static spi_device_handle_t max_spi;

    // Initialize the SPI bus
    esp_err_t ret = spi_bus_initialize(MAX_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);
    ret = spi_bus_add_device(MAX_SPI_HOST, &devcfg, &max_spi);
    ESP_ERROR_CHECK(ret);

    // install gpio isr service
    ret = gpio_install_isr_service(GPIO_INTR_NEGEDGE);
    ESP_ERROR_CHECK(ret);

    // Begin the temperature reading task
    max31856_start_drdy_pin_task(DRDY_PIN, &max_spi);

    uint8_t *r;

    // Set open wire detection
    r = max31856_read_register(&max_spi, RW_REG_CR0, 1);
    *r = *r | CR0_OCFAULT0;
    max31856_write_register(&max_spi, RW_REG_CR0, r, 1);

    // Set TC to type K
    r = max31856_read_register(&max_spi, RW_REG_CR1, 1);
    *r = *r | TYPE_K;
    max31856_write_register(&max_spi, RW_REG_CR1, r, 1);

    // automatic conversion is set in DRDY task

#pragma endregion

    // init the ADC channel
    // todo: improve the ADC oneshot library
    // adc_init();

    // init the PWM channel
    // pwm_init(PWM_PULSE_GPIO, PWM_TIMEBASE_RESOLUTION_HZ, PWM_TIMEBASE_PERIOD);

    start_slow_pwm_task(&pwm_params);

    pid_controller_struct pid = create_pid_struct();

    start_pid_task(&pid);

    pid.ki = 0.001;
    pid.kp = 0.3;
    pid.IntegratorLimits.ClampEnable = 1;
    pid.IntegratorLimits.LimitHi = 5;
    pid.IntegratorLimits.LimitLo = -5;
    pid.Init = false;
    pid.Setpoint = 60;
    pid.PVSPCallback = &receive_temp_to_pv;
    pid.CVCallback = &update_pwm_with_cv;

    // Initialize NVS
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    wifi_init_sta();

    while (true)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));

        // adc_u1_ch4();

        ESP_LOGI(TAG, "Temperature: %.1f °C", pid.ProcessValue);
        ESP_LOGI(TAG, "Control Value: %.1f", pid.ControlValue);
    }
}

// callback function of the PID loop
void receive_temp_to_pv(void *_pid)
{
    pid_controller_struct *pid = (pid_controller_struct *)_pid;
    xQueueReceive(MAX31856_TEMP_READ_QUEUE, &pid->ProcessValue, 0);
};

// callback function of the PID loop
void update_pwm_with_cv(void *_pid)
{
    pid_controller_struct *pid = (pid_controller_struct *)_pid;
    float pulse_width = 10.0 * (pid->ControlValue > 0 ? pid->ControlValue : 0);
    pulse_width = pulse_width > 100.0 ? 100.0 : pulse_width;
    ESP_LOGD(TAG, "Setting pulse width to %.1f", pulse_width);
    // set_pulse_width(pulse_width);
    pwm_params.duty_cycle = pulse_width;
};