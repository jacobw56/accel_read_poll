/**
 * @file    ble_sensor.h
 * @author  Walter Jacob - Overkill Projects, LLC.
 * @date    2023
 *
 * @brief   BLE Sensor service
 *
 * @details Contains the sensor data.
 */
#ifndef BLE_SENSOR_H__
#define BLE_SENSOR_H__

#include "ble.h"
#include "ble_srv_common.h"
#include "nrf_sdh_ble.h"
#include "sdk_common.h"
#include "sensor.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define BLE_SENSOR_OBSERVER_PRIO 2
#define SENSOR_DATA_LENGTH 6

/**@brief Macro for defining a BLE Sensor service instance.
 *
 * @param   _name  Name of the instance.
 * @hideinitializer
 */
#define BLE_SENSOR_DEF(_name)                      \
    static ble_sensor_t _name;                     \
    NRF_SDH_BLE_OBSERVER(_name##_obs,              \
                         BLE_SENSOR_OBSERVER_PRIO, \
                         ble_sensor_on_ble_evt,    \
                         &_name)

// Sensor service:                  D000D001-6D69-6E64-8711-73746978FEEB
// Sensor data characteristic:      D000D002-6D69-6E64-8711-73746978FEEB
// Sensor ODR characteristic:       D000D003-6D69-6E64-8711-73746978FEEB
// Base UUID:                       D0000000-6D69-6E64-8711-73746978FEEB
#define BLE_UUID_SENSOR_SERVICE_BASE_UUID                                                              \
    {                                                                                                  \
        0xED, 0xFE, 0x78, 0x69, 0x74, 0x73, 0x11, 0x87, 0x64, 0x6E, 0x69, 0x6D, 0x00, 0x00, 0x00, 0xD0 \
    } /**< Sensor service base UUID. */

// Service & characteristics UUIDs
#define BLE_UUID_SENSOR_SERVICE_UUID 0xD001   /**< Sensor service UUID. */
#define BLE_UUID_SENSOR_DATA_CHAR_UUID 0xD002 /**< Sensor data characteristic UUID. */
#define BLE_UUID_SENSOR_ODR_CHAR_UUID 0xD003  /**< Sensor ODR characteristic UUID. */

    /**@brief Activity service event type. */
    typedef enum
    {
        BLE_SENSOR_EVT_INDICATION_ENABLED,
        BLE_SENSOR_EVT_INDICATION_CONFIRMED,
        BLE_SENSOR_EVT_NOTIFICATION_ENABLED,
        BLE_SENSOR_EVT_INDICATION_NOTIFICATION_DISABLED,
    } ble_sensor_evt_type_t;

    /**@brief Sensor service event. */
    typedef struct
    {
        ble_sensor_evt_type_t evt_type;
    } ble_sensor_evt_t;

    // Forward declaration of the ble_sensor_t type.
    typedef struct ble_sensor_s ble_sensor_t;

    /**@brief Sensor service indication event handler type. */
    typedef void (*ble_sensor_indication_handler_t)(ble_sensor_t *p_sensor, ble_sensor_evt_t *p_evt);

    /**@brief Sensor service init structure. */
    typedef struct
    {
        ble_sensor_indication_handler_t indication_evt_handler;
        ble_srv_error_handler_t error_handler;
    } ble_sensor_init_t;

    /**@brief Sensor service structure. */
    struct ble_sensor_s
    {
        ble_sensor_indication_handler_t indication_evt_handler;
        ble_srv_error_handler_t error_handler;

        uint16_t conn_handle;
        uint16_t service_handle;
        uint8_t uuid_type;

        sensor_data_t data;
        uint8_t odr;

        ble_gatts_char_handles_t data_handles;
        ble_gatts_char_handles_t odr_handles;
    };

    /**@brief Function for initializing the Sensor service.
     *
     * @param[out]  p_sensor       Sensor service structure.
     * @param[in]   p_sensor_init  Information needed to initialize the service.
     *
     * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code from the SD.
     */
    nrfx_err_t ble_sensor_init(ble_sensor_t *p_sensor, const ble_sensor_init_t *p_sensor_init);

    /**@brief Function for handling the Application's BLE Stack events.
     *
     * @details Handles all events from the BLE stack of interest to the Device service.
     *
     * @param[in]   p_ble_evt   Event received from the BLE stack.
     * @param[in]   p_context   Device service structure.
     */
    void ble_sensor_on_ble_evt(ble_evt_t const *p_ble_evt, void *p_context);

    /**
     * @brief Function for updating the data characteristic.
     *
     * @param[in]   p_sensor  BLE sensor driver.
     * @param[in]   data      Sensor data.
     *
     * @retval    NRF_SUCCESS             If the request was added successfully.
     * @retval    NRF_ERROR_NULL          Any parameter was NULL.
     * @retval    NRF_ERROR_NO_MEM        There was no room in the queue or in the data pool.
     * @retval    NRF_ERROR_INVALID_PARAM If \p conn_handle is not registered or type of request -
     *                                    \p p_req is not valid or if the activity driver is null.
     * @retval    NRF_ERROR_INVALID_STATE if indication is not enabled.
     */
    nrfx_err_t ble_sensor_data_update(ble_sensor_t *p_sensor,
                                      sensor_data_t const *data);

    /**
     * @brief Function for updating the ODR characteristic.
     *
     * @param[in]   p_sensor  BLE sensor driver.
     * @param[in]   odr       Sensor ODR.
     *
     * @retval    NRF_SUCCESS             If the request was added successfully.
     * @retval    NRF_ERROR_NULL          Any parameter was NULL.
     * @retval    NRF_ERROR_NO_MEM        There was no room in the queue or in the data pool.
     * @retval    NRF_ERROR_INVALID_PARAM If \p conn_handle is not registered or type of request -
     *                                    \p p_req is not valid or if the activity driver is null.
     * @retval    NRF_ERROR_INVALID_STATE if indication is not enabled.
     */
    nrfx_err_t ble_sensor_odr_update(ble_sensor_t *p_sensor,
                                     uint8_t const *odr);

    /**
     * @brief Function populates the indication enabled flag.
     *
     * @param[in]     conn_handle             Connection handle.
     * @param[in]     p_sensor                BLE sensor driver.
     * @param[in,out] p_indication_enabled    Indication enabled flag.
     *
     * @retval    NRF_SUCCESS Successfully retrieved the value of the attribute.
     * @retval    NRF_ERROR_INVALID_ADDR Invalid pointer supplied.
     * @retval    NRF_ERROR_NOT_FOUND Attribute not found.
     * @retval    NRF_ERROR_INVALID_PARAM Invalid attribute offset supplied.
     * @retval    BLE_ERROR_INVALID_CONN_HANDLE Invalid connection handle supplied on a system attribute.
     * @retval    BLE_ERROR_GATTS_SYS_ATTR_MISSING System attributes missing, use @ref sd_ble_gatts_sys_attr_set to set them to a known value.
     */
    nrfx_err_t ble_sensor_is_indication_enabled(uint16_t conn_handle,
                                                ble_sensor_t *p_sensor,
                                                bool *p_indication_enabled);

    /**
     * @brief Function populates the notification enabled flag.
     *
     * @param[in]     conn_handle             Connection handle.
     * @param[in]     p_sensor                BLE sensor driver.
     * @param[in,out] p_notification_enabled  Notification enabled flag.
     *
     * @retval    NRF_SUCCESS Successfully retrieved the value of the attribute.
     * @retval    NRF_ERROR_INVALID_ADDR Invalid pointer supplied.
     * @retval    NRF_ERROR_NOT_FOUND Attribute not found.
     * @retval    NRF_ERROR_INVALID_PARAM Invalid attribute offset supplied.
     * @retval    BLE_ERROR_INVALID_CONN_HANDLE Invalid connection handle supplied on a system attribute.
     * @retval    BLE_ERROR_GATTS_SYS_ATTR_MISSING System attributes missing, use @ref sd_ble_gatts_sys_attr_set to set them to a known value.
     */
    nrfx_err_t ble_sensor_is_notification_enabled(uint16_t conn_handle,
                                                  ble_sensor_t *p_sensor,
                                                  bool *p_notification_enabled);

/** @} */
#ifdef __cplusplus
}
#endif

#endif /* BLE_SENSOR_H__ */