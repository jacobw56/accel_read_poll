/**
 * @file    sensor.h
 * @author  Walter Jacob - Overkill Projects, LLC.
 * @date    2023
 *
 * @brief   Simple sensor driver.
 *
 * @details A simple (not very portable) API for making reads from a sensor.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "nrfx_spim.h"
#include "lsm6dso32.h"

/** @brief Union holding the appropriate sensor data struct (accel or gyro). */
typedef union
{
    struct
    {
        int x;
        int y;
        int z;
    };
    struct
    {
        int pitch;
        int roll;
        int yaw;
    };
} sensor_data_t;

/** @brief Subsensor struct containing info about the accel or gyro. */
typedef struct
{
    sensor_data_t data;
    uint8_t fs;
    uint8_t odr;
} sensor_subsensor_t;

/** @brief Configuration structure of the SPIM driver instance. */
typedef struct
{
    uint8_t accel_fs;
    uint8_t accel_odr;
    uint8_t gyro_fs;
    uint8_t gyro_odr;
} sensor_config_t;

#define SENSOR_DEFAULT_CONFIG                     \
    {                                             \
        .accel_fs = LSM6DSO32_ACCEL_RANGE_8_G,    \
        .accel_odr = LSM6DSO32_RATE_208_HZ,       \
        .gyro_fs = LSM6DSO32_GYRO_RANGE_1000_DPS, \
        .gyro_odr = LSM6DSO32_RATE_208_HZ,        \
    }

/** @brief Structure of the SPIM driver instance. */
typedef struct
{
    nrfx_drv_state_t state;
    sensor_subsensor_t accel;
    sensor_subsensor_t gyro;
} sensor_t;

/**
 * @brief Function for initializing the sensor.
 *
 * @param[in] sensor Pointer to the sensor driver structure.
 * @param[in] spi    Pointer to the SPI peripheral being used by the sensor.
 *
 * @retval NRFX_SUCCESS             The procedure is successful.
 * @retval NRFX_ERROR_BUSY          The driver is not ready for a new transfer.
 * @retval NRFX_ERROR_NOT_SUPPORTED The provided parameters are not supported.
 * @retval NRFX_ERROR_INTERNAL      The sensor returned an invalid WHO_AM_I value.
 * @retval NRFX_ERROR_INVALID_ADDR  The provided buffers are not placed in the Data
 *                                  RAM region.
 */
nrfx_err_t sensor_init(sensor_t *sensor, sensor_config_t *config, nrfx_spim_t *spi);

/**
 * @brief Function for reading a register from the sensor.
 *
 * @param[in] sensor Pointer to the sensor driver structure.
 * @param[in] result Pointer to the result buffer.
 * @param[in] reg    Address of the register to be read.
 *
 * @retval NRFX_SUCCESS             The procedure is successful.
 * @retval NRFX_ERROR_BUSY          The driver is not ready for a new transfer.
 * @retval NRFX_ERROR_NOT_SUPPORTED The provided parameters are not supported.
 * @retval NRFX_ERROR_INVALID_ADDR  The provided buffers are not placed in the Data
 *                                  RAM region.
 */
nrfx_err_t sensor_read_reg(sensor_t *sensor, uint8_t *result, uint8_t reg);

/**
 * @brief Function for writing a register from the sensor.
 *
 * @param[in] sensor Pointer to the sensor driver structure.
 * @param[in] result The value to write to the register.
 * @param[in] reg    Address of the register to be written.
 *
 * @retval NRFX_SUCCESS             The procedure is successful.
 * @retval NRFX_ERROR_BUSY          The driver is not ready for a new transfer.
 * @retval NRFX_ERROR_NOT_SUPPORTED The provided parameters are not supported.
 * @retval NRFX_ERROR_INVALID_ADDR  The provided buffers are not placed in the Data
 *                                  RAM region.
 */
nrfx_err_t sensor_write_reg(sensor_t *sensor, uint8_t value, uint8_t reg);

/**
 * @brief Function for rebooting the sensor.
 *
 * @param[in] sensor Pointer to the sensor driver structure.
 *
 * @retval NRFX_SUCCESS             The procedure is successful.
 * @retval NRFX_ERROR_BUSY          The driver is not ready for a new transfer.
 * @retval NRFX_ERROR_NOT_SUPPORTED The provided parameters are not supported.
 * @retval NRFX_ERROR_INVALID_ADDR  The provided buffers are not placed in the Data
 *                                  RAM region.
 */
nrfx_err_t sensor_reboot(sensor_t *sensor);

/**
 * @brief Function for setting the sensor to block-data updates.
 *
 * @param[in] sensor Pointer to the sensor driver structure.
 *
 * @retval NRFX_SUCCESS             The procedure is successful.
 * @retval NRFX_ERROR_BUSY          The driver is not ready for a new transfer.
 * @retval NRFX_ERROR_NOT_SUPPORTED The provided parameters are not supported.
 * @retval NRFX_ERROR_INVALID_ADDR  The provided buffers are not placed in the Data
 *                                  RAM region.
 */
nrfx_err_t sensor_block_data_update(sensor_t *sensor);

/**
 * @brief Function for setting the sensor to continuous-data updates.
 *
 * @param[in] sensor Pointer to the sensor driver structure.
 *
 * @retval NRFX_SUCCESS             The procedure is successful.
 * @retval NRFX_ERROR_BUSY          The driver is not ready for a new transfer.
 * @retval NRFX_ERROR_NOT_SUPPORTED The provided parameters are not supported.
 * @retval NRFX_ERROR_INVALID_ADDR  The provided buffers are not placed in the Data
 *                                  RAM region.
 */
nrfx_err_t sensor_continuous_data_update(sensor_t *sensor);

/**
 * @brief Function for performing a software reset on the sensor.
 *
 * @param[in] sensor Pointer to the sensor driver structure.
 *
 * @retval NRFX_SUCCESS             The procedure is successful.
 * @retval NRFX_ERROR_BUSY          The driver is not ready for a new transfer.
 * @retval NRFX_ERROR_NOT_SUPPORTED The provided parameters are not supported.
 * @retval NRFX_ERROR_INVALID_ADDR  The provided buffers are not placed in the Data
 *                                  RAM region.
 */
nrfx_err_t sensor_software_reset(sensor_t *sensor);

/**
 * @brief Function for setting the data rate for the accelerometer.
 *
 * @param[in] sensor Pointer to the sensor driver structure.
 * @param[in] odr    A valid ODR value. @ref lsm6dso32_data_rate_t
 *
 * @retval NRFX_SUCCESS             The procedure is successful.
 * @retval NRFX_ERROR_BUSY          The driver is not ready for a new transfer.
 * @retval NRFX_ERROR_NOT_SUPPORTED The provided parameters are not supported.
 * @retval NRFX_ERROR_INVALID_ADDR  The provided buffers are not placed in the Data
 *                                  RAM region.
 */
nrfx_err_t sensor_set_accel_odr(sensor_t *sensor, uint8_t odr);

/**
 * @brief Function for setting the full scale range for the accelerometer.
 *
 * @param[in] sensor Pointer to the sensor driver structure.
 * @param[in] fs     A valid ODR value. @ref lsm6dso32_accel_range_t
 *
 * @retval NRFX_SUCCESS             The procedure is successful.
 * @retval NRFX_ERROR_BUSY          The driver is not ready for a new transfer.
 * @retval NRFX_ERROR_NOT_SUPPORTED The provided parameters are not supported.
 * @retval NRFX_ERROR_INVALID_ADDR  The provided buffers are not placed in the Data
 *                                  RAM region.
 */
nrfx_err_t sensor_set_accel_fs(sensor_t *sensor, uint8_t fs);

/**
 * @brief Function for setting the data rate for the gyroscope.
 *
 * @param[in] sensor Pointer to the sensor driver structure.
 * @param[in] odr    A valid ODR value. @ref lsm6dso32_data_rate_t
 *
 * @retval NRFX_SUCCESS             The procedure is successful.
 * @retval NRFX_ERROR_BUSY          The driver is not ready for a new transfer.
 * @retval NRFX_ERROR_NOT_SUPPORTED The provided parameters are not supported.
 * @retval NRFX_ERROR_INVALID_ADDR  The provided buffers are not placed in the Data
 *                                  RAM region.
 */
nrfx_err_t sensor_set_gyro_odr(sensor_t *sensor, uint8_t odr);

/**
 * @brief Function for setting the full scale range for the gyroscope.
 *
 * @param[in] sensor Pointer to the sensor driver structure.
 * @param[in] fs     A valid ODR value. @ref lsm6dso32_gyro_range_t
 *
 * @retval NRFX_SUCCESS             The procedure is successful.
 * @retval NRFX_ERROR_BUSY          The driver is not ready for a new transfer.
 * @retval NRFX_ERROR_NOT_SUPPORTED The provided parameters are not supported.
 * @retval NRFX_ERROR_INVALID_ADDR  The provided buffers are not placed in the Data
 *                                  RAM region.
 */
nrfx_err_t sensor_set_gyro_fs(sensor_t *sensor, uint8_t fs);

/**
 * @brief Function for setting the sensor as ready to poll.
 */
void sensor_poll(void);

/**
 * @brief Function for reading the sensor.
 *
 * @details Checks the poll sensor flag, which needs to be reset here.
 *
 * @param[in] sensor Pointer to the sensor driver structure.
 */
void sensor_process(sensor_t *sensor);
