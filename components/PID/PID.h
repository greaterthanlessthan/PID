#include <stdbool.h>
#include <stdint.h>

/**
 * Limit struct. Limits the value and rate of change for several variables in the PID struct
 */
typedef struct pid_limits
{
    // value
    bool ClampEnable; // 0=do not impose limits, 1=impose limits
    float LimitHi;    // maximum value of the variable
    float LimitLo;    // minimum value of the variable
    bool AtHiLimit;   // 1=at or above high limit
    bool AtLoLimit;   // 1=at or below low limit

    // rate of change
    bool ROC_ClampEnable; // 0=do not impose ROC limits, 1=impose limits
    float ROC_LimitHi;    // maximum rate of change of the variable
    float ROC_LimitLo;    // minimum rate of change of the variable
    bool ROC_AtHiLimit;   // 1=at or above high rate of change limit
    bool ROC_AtLoLimit;   // 1=at or above low rate of chagne limit
} pid_limits;

/**
 * PID Struct. Contains parameters for the PID controller
 */
typedef struct pid_controller_struct
{
    // inputs and output
    float Setpoint;                       // memory address of the setpoint
    float ProcessValue;                   // memory address of the process value
    float ControlValue;                   // memory address of the control value
    struct pid_limits SetpointLimits;     // limits for the setpoint
    struct pid_limits ProcessValueLimits; // sanity checking
    struct pid_limits ControlValueLimits; // limits for the control value

    // initalization
    bool Init;       // 1=clear integrator, hold CV to init_value
    float InitValue; // value of CV when init=1

    // gains
    float kp; // proportional gain
    float ki; // integral gain
    float kd; // derivative gain

    // error calculation
    float Error;         // calculated error
    bool LoopDirection;  // direction. 0 is err=sp-pv (heating loop), 1 is err=pv-sp (cooling loop)
    float ErrorDeadband; // deadband. When absolute error is less than this value, output does not change

    // integrator
    float IntegratorValue;              // value of the integrator
    struct pid_limits IntegratorLimits; // limits for the integrator
    uint16_t SamplingRate;              // sampling rate in ms

    // PV callback
    // points to a function that is called whenever the PID cycle begins
    // the function must return nothing and take pid_controller_struct as an argument
    // intended to receive the input and setpoint
    void (*PVSPCallback)(void *);

    // CV callback
    // points to a function that is called whenever the PID cycle completes
    // the function must return nothing and take pid_controller_struct as an argument
    // intended to update the output
    void (*CVCallback)(void *);
} pid_controller_struct;

/**
 * Create a pid_controller_struct with default values and memory addresses defined for inputs and outputs
 */
pid_controller_struct
create_pid_struct();

/**
 * Begin the PID task
 */
void start_pid_task(pid_controller_struct *pid);