typedef struct pwm_slow_params
{
    int period;          // period in ms
    float duty_cycle;    // % of time GPIO is high
    int min_pulse_width; // minimum time for a pulse in ms
    int pwm_pin;
} pwm_slow_params;

void start_slow_pwm_task(pwm_slow_params *params);