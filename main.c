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

#include "ble.h"
#include "ble_err.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_dis.h"
#include "ble_conn_params.h"
#include "ble_sensor.h"

#include "nrf_sdm.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"

#include "peer_manager.h"
#include "peer_manager_handler.h"

#include "fds.h"

#include "nrf_ble_gatt.h"
#include "nrf_ble_lesc.h"
#include "nrf_ble_qwr.h"
#include "ble_conn_state.h"
#include "nrf_pwr_mgmt.h"

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

#define DEVICE_NAME "Accel_poll"             /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME "OverkillProjects" /**< Manufacturer. Will be passed to Device Information Service. */
#define MODEL_NUMBER 1                       /**< Model number. Will be passed to Device Information Service. */
#define APP_ADV_INTERVAL 300                 /**< The advertising interval (in units of 0.625 ms. This value corresponds to 187.5 ms). */

#define APP_ADV_DURATION 18000 /**< The advertising duration (180 seconds) in units of 10 milliseconds. */

#define APP_BLE_CONN_CFG_TAG 1  /**< A tag identifying the SoftDevice BLE configuration. */
#define APP_BLE_OBSERVER_PRIO 3 /**< Application's BLE observer priority. You shouldn't need to modify this value. */

#define BATTERY_LEVEL_MEAS_INTERVAL APP_TIMER_TICKS(2000) /**< Battery level measurement interval (ticks). */
#define MIN_BATTERY_LEVEL 81                              /**< Minimum simulated battery level. */
#define MAX_BATTERY_LEVEL 100                             /**< Maximum simulated 7battery level. */
#define BATTERY_LEVEL_INCREMENT 1                         /**< Increment between each simulated battery level measurement. */

#define HEART_RATE_MEAS_INTERVAL APP_TIMER_TICKS(1000) /**< Heart rate measurement interval (ticks). */
#define MIN_HEART_RATE 140                             /**< Minimum heart rate as returned by the simulated measurement function. */
#define MAX_HEART_RATE 300                             /**< Maximum heart rate as returned by the simulated measurement function. */
#define HEART_RATE_INCREMENT 10                        /**< Value by which the heart rate is incremented/decremented for each call to the simulated measurement function. */

#define RR_INTERVAL_INTERVAL APP_TIMER_TICKS(300) /**< RR interval interval (ticks). */
#define MIN_RR_INTERVAL 100                       /**< Minimum RR interval as returned by the simulated measurement function. */
#define MAX_RR_INTERVAL 500                       /**< Maximum RR interval as returned by the simulated measurement function. */
#define RR_INTERVAL_INCREMENT 1                   /**< Value by which the RR interval is incremented/decremented for each call to the simulated measurement function. */

#define SENSOR_CONTACT_DETECTED_INTERVAL APP_TIMER_TICKS(5000) /**< Sensor Contact Detected toggle interval (ticks). */

#define MIN_CONN_INTERVAL MSEC_TO_UNITS(400, UNIT_1_25_MS) /**< Minimum acceptable connection interval (0.4 seconds). */
#define MAX_CONN_INTERVAL MSEC_TO_UNITS(650, UNIT_1_25_MS) /**< Maximum acceptable connection interval (0.65 second). */
#define SLAVE_LATENCY 0                                    /**< Slave latency. */
#define CONN_SUP_TIMEOUT MSEC_TO_UNITS(4000, UNIT_10_MS)   /**< Connection supervisory timeout (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY APP_TIMER_TICKS(5000) /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY APP_TIMER_TICKS(30000) /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT 3                       /**< Number of attempts before giving up the connection parameter negotiation. */

#define LESC_DEBUG_MODE 0 /**< Set to 1 to use LESC debug keys, allows you to use a sniffer to inspect traffic. */

#define SEC_PARAM_BOND 1                               /**< Perform bonding. */
#define SEC_PARAM_MITM 0                               /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC 1                               /**< LE Secure Connections enabled. */
#define SEC_PARAM_KEYPRESS 0                           /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES BLE_GAP_IO_CAPS_NONE /**< No I/O capabilities. */
#define SEC_PARAM_OOB 0                                /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE 7                       /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE 16                      /**< Maximum encryption key size. */

#define DEAD_BEEF 0xDEADBEEF /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

BLE_SENSOR_DEF(m_ble_sensor);
NRF_BLE_GATT_DEF(m_gatt);           /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);             /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising); /**< Advertising module instance. */
APP_TIMER_DEF(m_sensor_poll_timer);

static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID; /**< Handle of the current connection. */

static ble_uuid_t m_adv_uuids[] = /**< Universally unique service identifiers. */
    {
        {BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE},
        {BLE_UUID_BATTERY_SERVICE, BLE_UUID_TYPE_BLE},
};

static nrfx_spim_t m_spi = NRFX_SPIM_INSTANCE(3);
static sensor_t m_sensor;

/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num   Line number of the failing ASSERT call.
 * @param[in] file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t *p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**@brief Function for handling Service errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
void service_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

/**@brief Clear bond information from persistent storage.
 */
static void delete_bonds(void)
{
    ret_code_t err_code;

    NRF_LOG_INFO("Erase bonds!");

    err_code = pm_peers_delete();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for starting advertising.
 */
void advertising_start(bool erase_bonds)
{
    if (erase_bonds == true)
    {
        delete_bonds();
        // Advertising is started by PM_EVT_PEERS_DELETE_SUCCEEDED event.
    }
    else
    {
        ret_code_t err_code;

        err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
        APP_ERROR_CHECK(err_code);
    }
}

/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    ret_code_t err_code;
    ble_gap_conn_params_t gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_UNKNOWN);
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}

/**
 * @brief GATT generic Event handler.
 *
 * @param[in] p_gatt  GATT struct that contains the GATT module data.
 * @param[in] p_evt   GATT event struct.
 */
static void gatt_evt_handler(nrf_ble_gatt_t *p_gatt, nrf_ble_gatt_evt_t const *p_evt)
{
    switch (p_evt->evt_id)
    {
    case NRF_BLE_GATT_EVT_DATA_LENGTH_UPDATED:
        NRF_LOG_INFO("GATT DATA LENGTH on connection 0x%x changed to %d.",
                     p_evt->conn_handle,
                     p_evt->params.data_length);
        break;

    case NRF_BLE_GATT_EVT_ATT_MTU_UPDATED:
        NRF_LOG_INFO("GATT ATT MTU on connection 0x%x changed to %d.",
                     p_evt->conn_handle,
                     p_evt->params.att_mtu_effective);
        break;

    default:
        break;
    }
}

/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

/**
 * @brief BLE sensor service GATT indication callback.
 *
 * @details Used by characteristics with indicate/noifty attribute when an
 *          indicate/notify is received.
 *
 * @param[in] p_activity    BLE sensor instance.
 * @param[in] p_evt         GATT indication event.
 */
void ble_sensor_indication_handler(ble_sensor_t *p_sensor, ble_sensor_evt_t *p_evt)
{
    switch (p_evt->evt_type)
    {
    case BLE_SENSOR_EVT_INDICATION_ENABLED:
        NRF_LOG_INFO("Sensor indications enabled.");
        break;

    case BLE_SENSOR_EVT_INDICATION_CONFIRMED:
        NRF_LOG_INFO("Sensor indications confirmed.");
        break;

    case BLE_SENSOR_EVT_NOTIFICATION_ENABLED:
        NRF_LOG_INFO("Sensor notifications enabled.");
        break;

    case BLE_SENSOR_EVT_INDICATION_NOTIFICATION_DISABLED:
        NRF_LOG_INFO("Sensor indications and notifications disabled.");
        break;

    default:
        // No implementation needed.
        break;
    }
}

/**@brief Function for initializing services that will be used by the application.
 *
 * @details Initialize the Heart Rate, Battery and Device Information services.
 */
static void services_init(void)
{
    ret_code_t err_code;
    ble_dis_init_t dis_init;
    ble_sensor_init_t sensor_init;
    nrf_ble_qwr_init_t qwr_init = {0};

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

    // Initialize Device Information Service.
    memset(&dis_init, 0, sizeof(dis_init));

    dis_init.dis_char_rd_sec = SEC_OPEN;

    ble_srv_ascii_to_utf8(&dis_init.manufact_name_str, (char *)MANUFACTURER_NAME);
    ble_srv_ascii_to_utf8(&dis_init.model_num_str, (char *)MODEL_NUMBER);

    err_code = ble_dis_init(&dis_init);
    APP_ERROR_CHECK(err_code);

    // Initialize the custom sensor service
    memset(&sensor_init, 0, sizeof(sensor_init));

    sensor_init.error_handler = service_error_handler;
    sensor_init.indication_evt_handler = ble_sensor_indication_handler;
    err_code = ble_sensor_init(&m_ble_sensor, &sensor_init);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t *p_evt)
{
    ret_code_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}

/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    ret_code_t err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail = false;
    cp_init.evt_handler = on_conn_params_evt;
    cp_init.error_handler = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const *p_evt)
{
    pm_handler_on_pm_evt(p_evt);
    pm_handler_disconnect_on_sec_failure(p_evt);
    pm_handler_flash_clean(p_evt);

    switch (p_evt->evt_id)
    {
    case PM_EVT_PEERS_DELETE_SUCCEEDED:
        advertising_start(false);
        break;

    default:
        break;
    }
}

/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    ret_code_t err_code;

    /* Could prepare wakeup buttons here. */

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    switch (ble_adv_evt)
    {
    case BLE_ADV_EVT_FAST:
        NRF_LOG_INFO("Fast advertising.");
        /* Could set/toggle(PWM) an LED here. */
        break;

    case BLE_ADV_EVT_IDLE:
        sleep_mode_enter(); // No connection means we can just head off to sleep.
        break;

    default:
        break;
    }
}

/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const *p_ble_evt, void *p_context)
{
    ret_code_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
    case BLE_GAP_EVT_CONNECTED:
        NRF_LOG_INFO("Connected.");

        /* Could set an indication LED for connected status here. */

        m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
        err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
        APP_ERROR_CHECK(err_code);

        err_code = pm_conn_secure(p_ble_evt->evt.gap_evt.conn_handle, false);
        if (err_code != NRF_ERROR_BUSY)
        {
            APP_ERROR_CHECK(err_code);
        }

        break;

    case BLE_GAP_EVT_DISCONNECTED:
        NRF_LOG_INFO("Disconnected, reason %d.",
                     p_ble_evt->evt.gap_evt.params.disconnected.reason);
        m_conn_handle = BLE_CONN_HANDLE_INVALID;
        break;

    case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
    {
        NRF_LOG_DEBUG("PHY update request.");
        ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
        err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
        APP_ERROR_CHECK(err_code);
    }
    break;

    case BLE_GATTC_EVT_TIMEOUT:
        // Disconnect on GATT Client timeout event.
        NRF_LOG_DEBUG("GATT Client Timeout.");
        err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                         BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        APP_ERROR_CHECK(err_code);
        break;

    case BLE_GATTS_EVT_TIMEOUT:
        // Disconnect on GATT Server timeout event.
        NRF_LOG_DEBUG("GATT Server Timeout.");
        err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                         BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        APP_ERROR_CHECK(err_code);
        break;

    case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
        NRF_LOG_DEBUG("BLE_GAP_EVT_SEC_PARAMS_REQUEST");
        break;

    case BLE_GAP_EVT_AUTH_KEY_REQUEST:
        NRF_LOG_INFO("BLE_GAP_EVT_AUTH_KEY_REQUEST");
        break;

    case BLE_GAP_EVT_LESC_DHKEY_REQUEST:
        NRF_LOG_INFO("BLE_GAP_EVT_LESC_DHKEY_REQUEST");
        break;

    case BLE_GAP_EVT_AUTH_STATUS:
        NRF_LOG_INFO("BLE_GAP_EVT_AUTH_STATUS: status=0x%x bond=0x%x lv4: %d kdist_own:0x%x kdist_peer:0x%x",
                     p_ble_evt->evt.gap_evt.params.auth_status.auth_status,
                     p_ble_evt->evt.gap_evt.params.auth_status.bonded,
                     p_ble_evt->evt.gap_evt.params.auth_status.sm1_levels.lv4,
                     *((uint8_t *)&p_ble_evt->evt.gap_evt.params.auth_status.kdist_own),
                     *((uint8_t *)&p_ble_evt->evt.gap_evt.params.auth_status.kdist_peer));
        break;

    default:
        // No implementation needed.
        break;
    }
}

/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    ret_code_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}

/**@brief Function for the Peer Manager initialization.
 */
static void peer_manager_init(void)
{
    ble_gap_sec_params_t sec_param;
    ret_code_t err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_param.bond = SEC_PARAM_BOND;
    sec_param.mitm = SEC_PARAM_MITM;
    sec_param.lesc = SEC_PARAM_LESC;
    sec_param.keypress = SEC_PARAM_KEYPRESS;
    sec_param.io_caps = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob = SEC_PARAM_OOB;
    sec_param.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc = 1;
    sec_param.kdist_own.id = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id = 1;

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    ret_code_t err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    init.advdata.name_type = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance = true;
    init.advdata.flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    init.advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.advdata.uuids_complete.p_uuids = m_adv_uuids;

    init.config.ble_adv_fast_enabled = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout = APP_ADV_DURATION;

    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}

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

/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
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
    bool erase_bonds = false; // Get a button or process to update this if you ever want to reset bond data.

    log_init();
    timers_init();
    power_management_init();
    ble_stack_init();
    gap_params_init();
    gatt_init();
    advertising_init();
    services_init();
    conn_params_init();
    peer_manager_init();

    APP_ERROR_CHECK(spi_init());
    APP_ERROR_CHECK(sensor_config());

    timers_start();
    advertising_start(erase_bonds);

    // Enter main loop.
    for (;;)
    {
        sensor_process(&m_sensor); /* TODO: glue BLE sensor driver to this driver? */
        ble_sensor_data_update(&m_ble_sensor, &m_sensor.accel.data);
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
