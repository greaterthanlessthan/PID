#include <stdint.h>
void pwm_init(int pulse_gpio, uint32_t timer_resolution, uint32_t timer_period);
void set_pulse_width(uint32_t cmp_ticks);