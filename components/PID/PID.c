#include "freertos/FreeRTOS.h"
#include <freertos/event_groups.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <freertos/timers.h>
#include <freertos/portmacro.h> // not actually necessary but makes intellisense shut up

#include <esp_log.h>

#include "PID.h"

#include <float.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>

#define LOG_MEM_ALLOC_ERROR                                                   \
    ESP_LOGE(PID_TAG, "Could not allocate memory, see %s, line %d", __FILE__, \
             __LINE__);

static const char *PID_TAG = "PID";

/*
 * Provide a pid_controller_struct with defaults set
 */
pid_controller_struct
create_pid_struct()
{
    pid_limits lims =
        {
            .ClampEnable = 0,
            .LimitHi = FLT_MAX,
            .LimitLo = FLT_MIN,
            .AtHiLimit = 0,
            .AtLoLimit = 0,

            .ROC_ClampEnable = 0,
            .ROC_LimitHi = FLT_MAX,
            .ROC_LimitLo = FLT_MIN,
            .ROC_AtHiLimit = 0,
            .ROC_AtLoLimit = 0,
        };

    pid_controller_struct pid_struct =
        {
            .Setpoint = 0.0,
            .ProcessValue = 0.0,
            .ControlValue = 0.0,

            .SetpointLimits = lims,
            .ProcessValueLimits = lims,
            .ControlValueLimits = lims,

            .Init = 0,
            .InitValue = 0,

            .kp = 0,
            .ki = 0,
            .kd = 0,

            .Error = 0,
            .LoopDirection = 0,
            .ErrorDeadband = 0,

            .IntegratorValue = 0,
            .IntegratorLimits = lims,
            .SamplingRate = 20,

            .PVSPCallback = NULL,
            .CVCallback = NULL,
        };

    return pid_struct;
}

/*
 * Calculate the integral value using the trapezoidal rule.
 * The area is multiplied by ki when being added to the integrator.
 */
static float
trapezoidal_rule(uint16_t delta_t, float error, float error_previous)
{
    return 0.5 * (delta_t / 1000.0) * (error + error_previous);
}

/*
 * Set above/below limits flags, and clamp value if enabled
 */
static float
calculate_limits(pid_limits *limits, float input)
{
    limits->AtHiLimit = input > limits->LimitHi;
    limits->AtLoLimit = input < limits->LimitLo;

    if (!limits->ClampEnable)
    {
        return input;
    }
    if (!limits->AtHiLimit && !limits->AtLoLimit)
    {
        return input;
    }

    return limits->AtHiLimit ? limits->LimitHi : limits->LimitLo;
}

// todo: change %/s to u/s
/*
 * Set above/below rate of change flags, and clamp value if enabled
 */
static float
calculate_roc_limts(pid_limits *lim, float inp,
                    float inp_prev, uint16_t delta_t)
{
    // calculate rate of change in %/s
    float roc_perc_s = ((inp - inp_prev) / inp_prev) * 100.0 * (1000.0 / delta_t);
    float value;

    // Set status
    lim->ROC_AtHiLimit = roc_perc_s > lim->ROC_LimitHi;
    lim->ROC_LimitLo = roc_perc_s < lim->ROC_LimitLo;

    // Return if not limited
    if (!lim->ROC_ClampEnable)
    {
        return inp;
    }
    if (!lim->ROC_AtHiLimit && !lim->ROC_AtLoLimit)
    {
        return inp;
    }

    // Calculate limited value to return
    roc_perc_s = lim->ROC_AtHiLimit ? lim->ROC_LimitHi : lim->ROC_LimitLo;
    value = (roc_perc_s * inp_prev) / (100.0 * (1000.0 / delta_t)) + inp_prev;

    return value;
}

static float
calculate_both_limits(pid_limits *lim, float inp,
                      float inp_prev, uint16_t delta_t)
{
    inp = calculate_limits(lim, inp);
    return calculate_roc_limts(lim, inp, inp_prev, delta_t);
}

/*
 * PID controller FreeRTOS task
 */
static void pid_controller_task(void *pid_arg)
{
    pid_controller_struct *pid = (pid_controller_struct *)pid_arg;

    float last_err = 0.0;
    float last_pv = 0.0;
    float last_sp = 0.0;
    float last_cv = 0.0;

    float p_component;
    float d_component;

    float cv;
    float sp;
    float pv;

    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;)
    {
        // pause for set amount of time for sampling
        ESP_LOGD(PID_TAG, "Delaying for %" PRId16 " ms", pid->SamplingRate);
        xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(pid->SamplingRate));
        xLastWakeTime = xTaskGetTickCount();

        if (pid->PVSPCallback != NULL)
        {
            (*pid->PVSPCallback)(pid);
        }

        // process value processing
        // set the function-scope pv
        pv = calculate_both_limits(&pid->ProcessValueLimits,
                                   pid->ProcessValue,
                                   last_pv,
                                   pid->SamplingRate);
        last_pv = pv;

        // setpoint processing
        // set the function-scope sp
        sp = calculate_both_limits(&pid->ProcessValueLimits,
                                   pid->Setpoint,
                                   last_sp,
                                   pid->SamplingRate);
        last_sp = sp;

        // calculate the error
        if (pid->LoopDirection)
        {
            // Cooling loop
            pid->Error = pv - sp;
        }
        else
        { // Heating loop
            pid->Error = sp - pv;
        }

        // Apply the deadband
        if (fabs(pid->Error) < pid->ErrorDeadband)
            pid->Error = 0.0;

        // calculate the proportional component
        p_component = pid->kp * pid->Error;

        // calculate the integral component
        // get the new timeslice integrator value
        pid->IntegratorValue += pid->ki * trapezoidal_rule(pid->SamplingRate, pid->Error, last_err);
        // limit the integrator so that windup doesn't get too large
        pid->IntegratorValue = calculate_limits(&pid->IntegratorLimits, pid->IntegratorValue);

        // calculate the derivative component
        d_component = pid->kd * (pid->Error - last_err);

        // set last err now that it has been used
        last_err = pid->Error;

        // sum the components
        cv = p_component + pid->IntegratorValue + d_component;

        // apply limits before writing to control value
        pid->ControlValue = calculate_both_limits(&pid->ControlValueLimits,
                                                  cv,
                                                  last_cv,
                                                  pid->SamplingRate);
        last_cv = pid->ControlValue;

        if (pid->CVCallback != NULL)
        {
            (*pid->CVCallback)(pid);
        }
        ESP_LOGV(PID_TAG, "Task has %d words remaining in stack", uxTaskGetStackHighWaterMark(NULL));
    }
}

void start_pid_task(pid_controller_struct *pid)
{
    BaseType_t task_ret;

    // start gpio task
    task_ret = xTaskCreate(pid_controller_task, "pid_controller_task", 2000,
                           (void *)pid, 20, NULL);
    if (task_ret == pdPASS)
    {
        ESP_LOGI(PID_TAG, "Started the PID task");
    }
    else
    {
        LOG_MEM_ALLOC_ERROR
    }
}