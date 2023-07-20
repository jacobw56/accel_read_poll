/**
 * @file    main.c
 * @author  Walter Jacob - Overkill Projects, LLC.
 * @date    2023
 *
 * @brief   Main apllication file.
 *
 * @details This application polls a sensor (LSM6DS33). Very simple
 *          implementation, but not very portable. Change SENSOR_POLL_INTERVAL
 *          to modify the sample rate, keeping in mind that Nyquist tells us
 *          that we are only capturing signals up to 1/2 the sample rate, and
 *          that the sensor has a maximum sample rate of 6.664k for the accel
 *          and 1.666k for the gyro.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "app_timer.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_gpio.h"

#define SENSOR_POLL_INTERVAL APP_TIMER_TICKS(10) /**< Timer interval for sensor polling */

APP_TIMER_DEF(m_sensor_poll_timer);

static bool m_poll_sensor;

/**@brief Sensor polling timer callback.
 *
 * @details Kicks off a sensor read.
 */
static void sensor_poll_timer_timeout_handler(void *p_context)
{
    m_poll_sensor = true;
}

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{

    // Initialize timer module.
    uint32_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_sensor_poll_timer, APP_TIMER_MODE_REPEATED, sensor_poll_timer_timeout_handler);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for starting timers.
 */
static void timers_start(void)
{
    uint32_t err_code;
    err_code = app_timer_start(m_sensor_poll_timer, SENSOR_POLL_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for the Power manager.
 */
static void log_init(void)
{
    uint32_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        // NOTE: We could shutdown RAM regions here, etc.

        // Wait for an event.
        __WFE();
        // Clear the internal event register.
        __SEV();
        __WFE();
    }
}

static void read_sensor(void)
{
    if (m_poll_sensor == true)
    {
        m_poll_sensor = false;
        // Read sensor
    }
}

/**@brief Function for application main entry.
 */
int main(void)
{
    log_init();
    timers_init();

    m_poll_sensor = false;

    timers_start();

    // Enter main loop.
    for (;;)
    {
        read_sensor();
        idle_state_handle();
    }
}

/* These are for arm toolchain 11.3, which screwed up these defs
 * BEGIN arm toolchain fix
 */
void _close(void)
{
}
void _lseek(void)
{
}
void _read(void)
{
}
void _write(void)
{
}
/* END arm toolchain fix */
