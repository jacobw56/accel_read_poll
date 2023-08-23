#include <string.h>
#include "ble_sensor.h"

static uint8_t const sensor_data_name[] = "Sensor Data";
static uint8_t const sensor_odr_name[] = "Sensor ODR";

/* Static function prototypes */
static nrfx_err_t ble_sensor_data_add(ble_sensor_t *p_sensor);
static nrfx_err_t ble_sensor_odr_add(ble_sensor_t *p_sensor);
static void ble_sensor_on_connect(ble_sensor_t *p_sensor, ble_evt_t const *p_ble_evt);
static void ble_sensor_on_disconnect(ble_sensor_t *p_sensor, ble_evt_t const *p_ble_evt);
static void ble_sensor_on_write(ble_sensor_t *p_sensor, ble_evt_t const *p_ble_evt);
static void ble_sensor_on_hvc(ble_sensor_t *p_sensor, ble_evt_t const *p_ble_evt);

static void ble_sensor_on_connect(ble_sensor_t *p_sensor, ble_evt_t const *p_ble_evt)
{
}

static void ble_sensor_on_disconnect(ble_sensor_t *p_sensor, ble_evt_t const *p_ble_evt) {}

static void ble_sensor_on_cccd_write(ble_sensor_t *p_sensor, ble_gatts_evt_write_t const *p_evt_write)
{
    if (p_evt_write->len == 2)
    {
        // CCCD written, update indication state
        if (p_sensor->indication_evt_handler != NULL)
        {
            ble_sensor_evt_t evt;

            if (ble_srv_is_indication_enabled(p_evt_write->data))
            {
                evt.evt_type = BLE_SENSOR_EVT_INDICATION_ENABLED;
            }
            else if (ble_srv_is_notification_enabled(p_evt_write->data))
            {
                evt.evt_type = BLE_SENSOR_EVT_NOTIFICATION_ENABLED;
            }
            else
            {
                evt.evt_type = BLE_SENSOR_EVT_INDICATION_NOTIFICATION_DISABLED;
            }

            p_sensor->indication_evt_handler(p_sensor, &evt);
        }
    }
}

/**@brief Function for handling the Write event.
 *
 * @param[in]   p_sensor        Sensor service structure.
 * @param[in]   p_ble_evt       Event received from the BLE stack.
 */
static void ble_sensor_on_write(ble_sensor_t *p_sensor, ble_evt_t const *p_ble_evt)
{
    ble_gatts_evt_write_t const *p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    if (p_evt_write->handle == p_sensor->data_handles.cccd_handle)
    {
        ble_sensor_on_cccd_write(p_sensor, p_evt_write);
    }
}

/**@brief Function for handling the HVC event.
 *
 * @details Handles HVC events from the BLE stack.
 *
 * @param[in]   p_sensor        Sensor service structure.
 * @param[in]   p_ble_evt       Event received from the BLE stack.
 */
static void ble_sensor_on_hvc(ble_sensor_t *p_sensor, ble_evt_t const *p_ble_evt)
{
    ble_gatts_evt_hvc_t const *p_hvc = &p_ble_evt->evt.gatts_evt.params.hvc;

    if (p_hvc->handle == p_sensor->data_handles.value_handle)
    {
        ble_sensor_evt_t evt;

        evt.evt_type = BLE_SENSOR_EVT_INDICATION_CONFIRMED;
        p_sensor->indication_evt_handler(p_sensor, &evt);
    }
}

static nrfx_err_t ble_sensor_data_add(ble_sensor_t *p_sensor)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t attr_char_value;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_md_t attr_md;
    ble_uuid_t ble_uuid;

    memset(&char_md, 0, sizeof(char_md));
    memset(&cccd_md, 0, sizeof(cccd_md));
    memset(&attr_md, 0, sizeof(attr_md));
    memset(&attr_char_value, 0, sizeof(attr_char_value));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc = BLE_GATTS_VLOC_STACK;

    char_md.char_props.read = 1;
    char_md.char_props.write = 0;
    char_md.char_props.notify = 1;
    char_md.char_props.indicate = 1;
    char_md.p_char_user_desc = sensor_data_name;
    char_md.char_user_desc_size = sizeof(sensor_data_name);
    char_md.char_user_desc_max_size = sizeof(sensor_data_name);
    char_md.p_char_pf = NULL;
    char_md.p_user_desc_md = NULL;
    char_md.p_cccd_md = &cccd_md;
    char_md.p_sccd_md = NULL;

    // Define the Device Serial Characteristic UUID
    ble_uuid.type = p_sensor->uuid_type;
    ble_uuid.uuid = BLE_UUID_SENSOR_DATA_CHAR_UUID;

    // Set permissions on the Characteristic value
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);

    // Attribute Metadata settings
    attr_md.vloc = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth = 0;
    attr_md.wr_auth = 0;
    attr_md.vlen = 0;

    // Attribute Value settings
    attr_char_value.p_uuid = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len = sizeof(sensor_data_t);
    attr_char_value.init_offs = 0;
    attr_char_value.max_len = sizeof(sensor_data_t);
    attr_char_value.p_value = (uint8_t *)(&p_sensor->data);

    return sd_ble_gatts_characteristic_add(p_sensor->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_sensor->data_handles);
}

static nrfx_err_t ble_sensor_odr_add(ble_sensor_t *p_sensor)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t attr_char_value;
    ble_gatts_attr_md_t attr_md;
    ble_uuid_t ble_uuid;

    memset(&char_md, 0, sizeof(char_md));
    memset(&attr_md, 0, sizeof(attr_md));
    memset(&attr_char_value, 0, sizeof(attr_char_value));

    char_md.char_props.read = 1;
    char_md.char_props.write = 0;
    char_md.p_char_user_desc = sensor_odr_name;
    char_md.char_user_desc_size = sizeof(sensor_odr_name);
    char_md.char_user_desc_max_size = sizeof(sensor_odr_name);
    char_md.p_char_pf = NULL;
    char_md.p_user_desc_md = NULL;
    char_md.p_cccd_md = NULL;
    char_md.p_sccd_md = NULL;

    // Define the Device Serial Characteristic UUID
    ble_uuid.type = p_sensor->uuid_type;
    ble_uuid.uuid = BLE_UUID_SENSOR_ODR_CHAR_UUID;

    // Set permissions on the Characteristic value
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

    // Attribute Metadata settings
    attr_md.vloc = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth = 0;
    attr_md.wr_auth = 0;
    attr_md.vlen = 0;

    // Attribute Value settings
    attr_char_value.p_uuid = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len = sizeof(uint8_t);
    attr_char_value.init_offs = 0;
    attr_char_value.max_len = sizeof(uint8_t);
    attr_char_value.p_value = (uint8_t *)&p_sensor->odr;

    return sd_ble_gatts_characteristic_add(p_sensor->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_sensor->odr_handles);
}

nrfx_err_t ble_sensor_init(ble_sensor_t *p_sensor, ble_sensor_init_t const *p_sensor_init)
{
    VERIFY_PARAM_NOT_NULL(p_sensor);
    VERIFY_PARAM_NOT_NULL(p_sensor_init);

    nrfx_err_t err_code;
    ble_uuid_t ble_uuid;

    // Initialize service structure
    p_sensor->conn_handle = BLE_CONN_HANDLE_INVALID;

    // Initialize service structure.
    p_sensor->indication_evt_handler = p_sensor_init->indication_evt_handler;
    p_sensor->error_handler = p_sensor_init->error_handler;

    // Add service UUID
    ble_uuid128_t base_uuid = {BLE_UUID_SENSOR_SERVICE_BASE_UUID};
    err_code = sd_ble_uuid_vs_add(&base_uuid, &p_sensor->uuid_type);
    if (err_code != NRF_SUCCESS)
        return err_code;

    // Set up the UUID for the service (base + service-specific)
    ble_uuid.type = p_sensor->uuid_type;
    ble_uuid.uuid = BLE_UUID_SENSOR_SERVICE_UUID;

    // Set up and add the service
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_sensor->service_handle);
    if (err_code != NRF_SUCCESS)
        return err_code;

    // Add the different characteristics in the service
    err_code = ble_sensor_data_add(p_sensor);
    if (err_code != NRF_SUCCESS)
        return err_code;

    err_code = ble_sensor_odr_add(p_sensor);
    if (err_code != NRF_SUCCESS)
        return err_code;

    return NRF_SUCCESS;
}

void ble_sensor_on_ble_evt(ble_evt_t const *p_ble_evt, void *p_context)
{
    ble_sensor_t *p_sensor = (ble_sensor_t *)p_context;

    switch (p_ble_evt->header.evt_id)
    {
    case BLE_GAP_EVT_CONNECTED:
        ble_sensor_on_connect(p_sensor, p_ble_evt);
        break;

    case BLE_GAP_EVT_DISCONNECTED:
        ble_sensor_on_disconnect(p_sensor, p_ble_evt);
        break;

    case BLE_GATTS_EVT_WRITE:
        ble_sensor_on_write(p_sensor, p_ble_evt);
        break;

    case BLE_GATTS_EVT_HVC:
        ble_sensor_on_hvc(p_sensor, p_ble_evt);
        break;

    default:
        break;
    }
}

nrfx_err_t ble_sensor_data_update(ble_sensor_t *p_sensor, sensor_data_t const *p_data)
{
    nrfx_err_t ret;

    if (p_sensor == NULL)
        return NRF_ERROR_INVALID_PARAM;

    bool indication_enabled;
    ret = ble_sensor_is_indication_enabled(p_sensor->conn_handle, p_sensor, &indication_enabled);
    if (ret != NRFX_SUCCESS)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    bool notification_enabled;
    ret = ble_sensor_is_notification_enabled(p_sensor->conn_handle, p_sensor, &notification_enabled);
    if (ret != NRFX_SUCCESS)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    uint16_t len = (uint16_t)sizeof(sensor_data_t);

    ble_gatts_hvx_params_t hvx_params;

    memset(&hvx_params, 0, sizeof(hvx_params));

    hvx_params.handle = p_sensor->data_handles.value_handle;
    hvx_params.offset = 0;
    hvx_params.p_len = &len;
    hvx_params.p_data = (uint8_t *)p_data;

    if (indication_enabled)
    {
        hvx_params.type = BLE_GATT_HVX_INDICATION;
    }
    else if (notification_enabled)
    {
        hvx_params.type = BLE_GATT_HVX_NOTIFICATION;
    }

    return sd_ble_gatts_hvx(p_sensor->conn_handle,
                            &hvx_params);
}

nrfx_err_t ble_sensor_odr_update(ble_sensor_t *p_sensor,
                                 uint8_t const *odr)
{
    if (p_sensor == NULL)
        return NRFX_ERROR_INVALID_PARAM;

    ble_gatts_value_t gatts_value;

    // Initialize value struct.
    memset(&gatts_value, 0, sizeof(gatts_value));

    gatts_value.len = sizeof(uint8_t);
    gatts_value.offset = 0;
    gatts_value.p_value = (uint8_t *)odr;

    // Update database.
    return sd_ble_gatts_value_set(BLE_CONN_HANDLE_INVALID,
                                  p_sensor->odr_handles.value_handle,
                                  &gatts_value);
}

nrfx_err_t ble_sensor_is_indication_enabled(uint16_t conn_handle,
                                            ble_sensor_t *p_sensor,
                                            bool *p_indication_enabled)
{
    nrfx_err_t err_code;
    uint8_t cccd_value_buf[BLE_CCCD_VALUE_LEN];
    ble_gatts_value_t gatts_value;

    // Initialize value struct.
    memset(&gatts_value, 0, sizeof(gatts_value));

    gatts_value.len = BLE_CCCD_VALUE_LEN;
    gatts_value.offset = 0;
    gatts_value.p_value = cccd_value_buf;

    err_code = sd_ble_gatts_value_get(conn_handle,
                                      p_sensor->data_handles.cccd_handle,
                                      &gatts_value);
    if (err_code == NRF_SUCCESS)
    {
        *p_indication_enabled = ble_srv_is_indication_enabled(cccd_value_buf);
    }
    if (err_code == BLE_ERROR_GATTS_SYS_ATTR_MISSING)
    {
        *p_indication_enabled = false;
        return NRF_SUCCESS;
    }
    return err_code;
}

nrfx_err_t ble_sensor_is_notification_enabled(uint16_t conn_handle,
                                              ble_sensor_t *p_sensor,
                                              bool *p_notification_enabled)
{
    nrfx_err_t err_code;
    uint8_t cccd_value_buf[BLE_CCCD_VALUE_LEN];
    ble_gatts_value_t gatts_value;

    // Initialize value struct.
    memset(&gatts_value, 0, sizeof(gatts_value));

    gatts_value.len = BLE_CCCD_VALUE_LEN;
    gatts_value.offset = 0;
    gatts_value.p_value = cccd_value_buf;

    err_code = sd_ble_gatts_value_get(conn_handle,
                                      p_sensor->data_handles.cccd_handle,
                                      &gatts_value);
    if (err_code == NRF_SUCCESS)
    {
        *p_notification_enabled = ble_srv_is_notification_enabled(cccd_value_buf);
    }
    if (err_code == BLE_ERROR_GATTS_SYS_ATTR_MISSING)
    {
        *p_notification_enabled = false;
        return NRF_SUCCESS;
    }
    return err_code;
}