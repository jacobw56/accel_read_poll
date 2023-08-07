/**
 * @file    main.c
 * @author  Walter Jacob - Overkill Projects, LLC.
 * @date    2023
 *
 * @brief   Main application file.
 *
 * @details This application polls a sensor (LSM6DSO32). Very simple
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
#include "nrfx_clock.h"
#include "nrfx_spim.h"

#include "sensor.h"

#define SPI_CS_PIN NRF_GPIO_PIN_MAP(0, 27)
#define SPI_SCK_PIN NRF_GPIO_PIN_MAP(1, 8)
#define SPI_MOSI_PIN NRF_GPIO_PIN_MAP(0, 7)
#define SPI_MISO_PIN NRF_GPIO_PIN_MAP(0, 26)

#define SENSOR_POLL_INTERVAL APP_TIMER_TICKS(10) /**< Timer interval for sensor polling */

APP_TIMER_DEF(m_sensor_poll_timer);

static nrfx_spim_t m_spi = NRFX_SPIM_INSTANCE(3);
static sensor_t m_sensor;

/**@brief Clock event callback.
 *
 * @param[in] event     Clock event to handle
 */
void clock_event_handler(nrfx_clock_evt_type_t event)
{
    switch (event)
    {
    case NRFX_CLOCK_EVT_HFCLK_STARTED:
        break;
    case NRFX_CLOCK_EVT_LFCLK_STARTED:
        /* Could use this to start app_timer */
        break;
    case NRFX_CLOCK_EVT_CTTO:
        break;
    case NRFX_CLOCK_EVT_CAL_DONE:
        break;
    default:
        break;
    }
}

/**@brief Sensor polling timer callback.
 *
 * @details Kicks off a sensor read.
 */
static void sensor_poll_timeout_handler(void *p_context)
{
    sensor_poll();
}

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates the application timers.
 */
static void timers_init(void)
{
    ret_code_t ret;

    // Init the low-freq clock source for app_timer
    ret = nrfx_clock_init(clock_event_handler);
    APP_ERROR_CHECK(ret);

    if (nrfx_clock_lfclk_is_running() == false)
    {
        nrfx_clock_lfclk_start();
    }

    // Initialize timer module.
    ret = app_timer_init();
    APP_ERROR_CHECK(ret);

    ret = app_timer_create(&m_sensor_poll_timer, APP_TIMER_MODE_REPEATED, sensor_poll_timeout_handler);
    APP_ERROR_CHECK(ret);
}

/**@brief Function for starting timers.
 */
static void timers_start(void)
{
    ret_code_t ret;
    ret = app_timer_start(m_sensor_poll_timer, SENSOR_POLL_INTERVAL, NULL);
    APP_ERROR_CHECK(ret);
}

/**@brief Function to initialize the log subsystem.
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

/**@brief Function to initialize the SPI peripheral for this application.
 *
 * @retval NRFX error codes.
 */
static nrfx_err_t spi_init(void)
{
    nrfx_spim_config_t cfg = NRFX_SPIM_DEFAULT_CONFIG;
    cfg.sck_pin = SPI_SCK_PIN;
    cfg.mosi_pin = SPI_MOSI_PIN;
    cfg.miso_pin = SPI_MISO_PIN;
    cfg.ss_pin = SPI_CS_PIN;
    cfg.frequency = NRF_SPIM_FREQ_1M; // might be too fast for breadboards...
    cfg.use_hw_ss = true;

    return nrfx_spim_init(&m_spi, &cfg, NULL, NULL);
}

/**@brief Function to configure the sensor for this application.
 *
 * @retval NRFX error codes.
 */
static nrfx_err_t sensor_config(void)
{
    nrfx_err_t ret;

    sensor_config_t cfg = SENSOR_DEFAULT_CONFIG;

    ret = sensor_init(&m_sensor, &cfg, &m_spi);
    if (ret != NRFX_SUCCESS)
        return ret;

    ret = sensor_block_data_update(&m_sensor);
    if (ret != NRFX_SUCCESS)
        return ret;

    return NRFX_SUCCESS;
}

/**@brief Function for application main entry.
 */
int main(void)
{
    log_init();
    APP_ERROR_CHECK(spi_init());
    APP_ERROR_CHECK(sensor_config());

    timers_init();
    timers_start();

    // Enter main loop.
    for (;;)
    {
        sensor_process(&m_sensor);
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
