/**
 * @file    sensor.c
 * @author  Walter Jacob - Overkill Projects, LLC.
 * @date    2023
 *
 * @brief   Simple sensor driver.
 *
 * @details A simple (not very portable) API for making reads from a sensor.
 */

#include "sensor.h"

#define ACCEL_DATA_SIZE_BYTES 6
#define GYRO_DATA_SIZE_BYTES 6

static nrfx_spim_t *p_spi;
static bool m_poll_sensor;

static nrfx_err_t read_reg(sensor_t *sensor, uint8_t *result, size_t result_size, uint8_t reg)
{
    uint8_t rx_buff[12];
    uint8_t reg_addr = reg | (1 << 7);

    nrfx_spim_xfer_desc_t xfer = {
        .p_tx_buffer = &reg_addr,
        .tx_length = 1,
        .p_rx_buffer = rx_buff,
        .rx_length = result_size,
    };

    nrfx_err_t ret = nrfx_spim_xfer(p_spi, &xfer, 0);
    if (ret != NRFX_SUCCESS)
    {
        return ret;
    }

    memcpy(result, &rx_buff[1], result_size);

    return NRFX_SUCCESS;
}

static unsigned int fs_map[4] = {4, 32, 8, 16};
static nrfx_err_t read_accel(sensor_t *sensor)
{
    nrfx_err_t ret;
    uint8_t accel_data[ACCEL_DATA_SIZE_BYTES];

    ret = read_reg(sensor, accel_data, ACCEL_DATA_SIZE_BYTES, LSM6DSO32_OUTX_L_A);
    if (ret != NRFX_SUCCESS)
        return ret;

    // Convert accel values to milli-g
    sensor->accel.data.x = (int)(((int)((int16_t) * ((int16_t *)(&accel_data[0]))) * (int)fs_map[sensor->accel.fs] * (int)1000) / (int)32768);
    sensor->accel.data.y = (int)(((int)((int16_t) * ((int16_t *)(&accel_data[2]))) * (int)fs_map[sensor->accel.fs] * (int)1000) / (int)32768);
    sensor->accel.data.z = (int)(((int)((int16_t) * ((int16_t *)(&accel_data[4]))) * (int)fs_map[sensor->accel.fs] * (int)1000) / (int)32768);

    return NRFX_SUCCESS;
}

static nrfx_err_t read_gyro(sensor_t *sensor)
{
    nrfx_err_t ret;
    uint8_t gyro_data[GYRO_DATA_SIZE_BYTES];

    ret = read_reg(sensor, gyro_data, GYRO_DATA_SIZE_BYTES, LSM6DSO32_OUTX_L_G);
    if (ret != NRFX_SUCCESS)
        return ret;

    // Convert gyro values to degrees per second
    sensor->gyro.data.x = (int)(((int)((int16_t) * ((int16_t *)(&gyro_data[0]))) * (int)fs_map[sensor->gyro.fs]) / (int)32768);
    sensor->gyro.data.y = (int)(((int)((int16_t) * ((int16_t *)(&gyro_data[2]))) * (int)fs_map[sensor->gyro.fs]) / (int)32768);
    sensor->gyro.data.z = (int)(((int)((int16_t) * ((int16_t *)(&gyro_data[4]))) * (int)fs_map[sensor->gyro.fs]) / (int)32768);

    return NRFX_SUCCESS;
}

nrfx_err_t sensor_read_reg(sensor_t *sensor, uint8_t *result, uint8_t reg)
{
    return read_reg(sensor, result, 1, reg);
}

nrfx_err_t sensor_write_reg(sensor_t *sensor, uint8_t value, uint8_t reg)
{
    uint8_t tx_buff[2] = {
        reg,
        value,
    };

    nrfx_spim_xfer_desc_t xfer = {
        .p_tx_buffer = tx_buff,
        .tx_length = 2,
        .p_rx_buffer = NULL,
        .rx_length = 0,
    };

    return nrfx_spim_xfer(p_spi, &xfer, 0);
}

nrfx_err_t sensor_reboot(sensor_t *sensor)
{
    uint8_t ctrl3;
    nrfx_err_t ret = sensor_read_reg(sensor, &ctrl3, (uint8_t)LSM6DSO32_CTRL3_C);
    if (ret != NRFX_SUCCESS)
        return ret;

    ctrl3 |= LSM6DSO32_CTRL3_C_BOOT_MSK;
    return sensor_write_reg(sensor, ctrl3, (uint8_t)LSM6DSO32_CTRL3_C);
}

nrfx_err_t sensor_block_data_update(sensor_t *sensor)
{
    uint8_t ctrl3;
    nrfx_err_t ret = sensor_read_reg(sensor, &ctrl3, (uint8_t)LSM6DSO32_CTRL3_C);
    if ((ret != NRFX_SUCCESS) || (ctrl3 & LSM6DSO32_CTRL3_C_BDU_MSK))
    {
        return ret;
    }

    ctrl3 |= LSM6DSO32_CTRL3_C_BDU_MSK;
    return sensor_write_reg(sensor, ctrl3, (uint8_t)LSM6DSO32_CTRL3_C);
}

nrfx_err_t sensor_continuous_data_update(sensor_t *sensor)
{
    uint8_t ctrl3;
    nrfx_err_t ret = sensor_read_reg(sensor, &ctrl3, (uint8_t)LSM6DSO32_CTRL3_C);
    if ((ret != NRFX_SUCCESS) || (~ctrl3 & LSM6DSO32_CTRL3_C_BDU_MSK))
    {
        return ret;
    }

    ctrl3 &= ~LSM6DSO32_CTRL3_C_BDU_MSK;
    return sensor_write_reg(sensor, ctrl3, (uint8_t)LSM6DSO32_CTRL3_C);
}

nrfx_err_t sensor_software_reset(sensor_t *sensor)
{
    uint8_t ctrl3;
    nrfx_err_t ret = sensor_read_reg(sensor, &ctrl3, (uint8_t)LSM6DSO32_CTRL3_C);
    if (ret != NRFX_SUCCESS)
    {
        return ret;
    }

    ctrl3 |= LSM6DSO32_CTRL3_C_SW_RESET_MSK;
    return sensor_write_reg(sensor, ctrl3, (uint8_t)LSM6DSO32_CTRL3_C);
}

nrfx_err_t sensor_set_accel_odr(sensor_t *sensor, uint8_t odr)
{
    uint8_t ctrl1;
    nrfx_err_t ret = sensor_read_reg(sensor, &ctrl1, (uint8_t)LSM6DSO32_CTRL1_XL);
    if ((ret != NRFX_SUCCESS) || (((ctrl1 & LSM6DSO32_CTRL1_XL_ODR_XL_MSK) >> LSM6DSO32_CTRL1_XL_ODR_XL_POS) == odr))
    {
        return ret;
    }

    ctrl1 &= ~LSM6DSO32_CTRL1_XL_ODR_XL_MSK;
    ctrl1 |= (odr << LSM6DSO32_CTRL1_XL_ODR_XL_POS);
    ret = sensor_write_reg(sensor, ctrl1, (uint8_t)LSM6DSO32_CTRL1_XL);
    if (ret != NRFX_SUCCESS)
    {
        return ret;
    }

    sensor->accel.odr = odr;
    return NRFX_SUCCESS;
}

nrfx_err_t sensor_set_accel_fs(sensor_t *sensor, uint8_t fs)
{
    uint8_t ctrl1;
    nrfx_err_t ret = sensor_read_reg(sensor, &ctrl1, (uint8_t)LSM6DSO32_CTRL1_XL);
    if ((ret != NRFX_SUCCESS) || (((ctrl1 & LSM6DSO32_CTRL1_XL_FS_XL_MSK) >> LSM6DSO32_CTRL1_XL_FS_XL_POS) == fs))
    {
        return ret;
    }

    ctrl1 &= ~LSM6DSO32_CTRL1_XL_FS_XL_MSK;
    ctrl1 |= (fs << LSM6DSO32_CTRL1_XL_FS_XL_POS);
    ret = sensor_write_reg(sensor, ctrl1, (uint8_t)LSM6DSO32_CTRL1_XL);
    if (ret != NRFX_SUCCESS)
    {
        return ret;
    }

    sensor->accel.fs = fs;
    return NRFX_SUCCESS;
}

nrfx_err_t sensor_set_gyro_odr(sensor_t *sensor, uint8_t odr)
{
    uint8_t ctrl2;
    nrfx_err_t ret = sensor_read_reg(sensor, &ctrl2, (uint8_t)LSM6DSO32_CTRL2_G);
    if ((ret != NRFX_SUCCESS) || (((ctrl2 & LSM6DSO32_CTRL2_G_ODR_G_MSK) >> LSM6DSO32_CTRL2_G_ODR_G_POS) == odr))
    {
        return ret;
    }

    ctrl2 &= ~LSM6DSO32_CTRL2_G_ODR_G_MSK;
    ctrl2 |= (odr << LSM6DSO32_CTRL2_G_ODR_G_POS);
    ret = sensor_write_reg(sensor, ctrl2, (uint8_t)LSM6DSO32_CTRL2_G);
    if (ret != NRFX_SUCCESS)
    {
        return ret;
    }

    sensor->gyro.odr = odr;
    return NRFX_SUCCESS;
}

nrfx_err_t sensor_set_gyro_fs(sensor_t *sensor, uint8_t fs)
{
    uint8_t ctrl2;
    nrfx_err_t ret = sensor_read_reg(sensor, &ctrl2, (uint8_t)LSM6DSO32_CTRL2_G);
    if ((ret != NRFX_SUCCESS) || (((ctrl2 & LSM6DSO32_CTRL2_G_FS_G_MSK) >> LSM6DSO32_CTRL2_G_FS_G_POS) == fs))
    {
        return ret;
    }

    ctrl2 &= ~LSM6DSO32_CTRL2_G_FS_G_MSK;
    ctrl2 |= (fs << LSM6DSO32_CTRL2_G_FS_G_POS);
    ret = sensor_write_reg(sensor, ctrl2, (uint8_t)LSM6DSO32_CTRL2_G);
    if (ret != NRFX_SUCCESS)
    {
        return ret;
    }

    sensor->gyro.fs = fs;
    return NRFX_SUCCESS;
}

nrfx_err_t sensor_init(sensor_t *sensor, sensor_config_t *config, nrfx_spim_t *spi)
{
    ASSERT(sensor->state != NRFX_DRV_STATE_INITIALIZED);
    p_spi = spi;

    // Verify WHO_AM_I
    uint8_t whoami;
    nrfx_err_t ret = sensor_read_reg(sensor, &whoami, LSM6DSO32_WHOAMI);
    if (ret != NRFX_SUCCESS)
        return ret;

    if (whoami != LSM6DSO32_WHO_AM_I_VALUE)
        return NRFX_ERROR_INTERNAL;

    // Configure the sensor
    ret = sensor_set_accel_fs(sensor, config->accel_fs);
    if (ret != NRFX_SUCCESS)
        return ret;

    ret = sensor_set_accel_odr(sensor, config->accel_odr);
    if (ret != NRFX_SUCCESS)
        return ret;

    ret = sensor_set_gyro_fs(sensor, config->gyro_fs);
    if (ret != NRFX_SUCCESS)
        return ret;

    ret = sensor_set_gyro_odr(sensor, config->gyro_odr);
    if (ret != NRFX_SUCCESS)
        return ret;

    m_poll_sensor = false;
    sensor->state = NRFX_DRV_STATE_INITIALIZED;
    return NRFX_SUCCESS;
}

void sensor_poll(void)
{
    m_poll_sensor = true;
}

void sensor_process(sensor_t *sensor)
{
    if (m_poll_sensor == true)
    {
        m_poll_sensor = false;
        // Read sensor
        read_accel(sensor);
        read_gyro(sensor);
    }
}
