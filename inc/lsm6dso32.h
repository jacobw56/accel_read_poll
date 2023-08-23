/**
 * @file    lsm6dso32.h
 * @author  Walter Jacob - Overkill Projects, LLC.
 * @date    2023
 *
 * @brief   LSM6DSO32 register definitions.
 *
 * @details LSM6DSO32 register definitions.
 */
#ifndef LSM6DSO32_H__
#define LSM6DSO32_H__

#define LSM6DSO32_I2CADDR_DEFAULT 0x6A ///< LSM6DSO32 default I2C address

/* REGISTERS */
#define LSM6DSO32_FUNC_CFG_ACCESS 0x1    ///< Enable embedded functions register
#define LSM6DSO32_PIN_CTRL 0x2           ///< Pin control register
#define LSM6DSO32_FIFO_CTRL1 0x07        ///< FIFO configuration register 1
#define LSM6DSO32_FIFO_CTRL2 0x08        ///< FIFO configuration register 2
#define LSM6DSO32_FIFO_CTRL3 0x09        ///< FIFO configuration register 3
#define LSM6DSO32_FIFO_CTRL4 0x0A        ///< FIFO configuration register 4
#define LSM6DSO32_COUNTER_BDR_REG1 0x0B  ///< Counter batch data rate register 1
#define LSM6DSO32_COUNTER_BDR_REG2 0x0C  ///< Counter batch data rate register 2
#define LSM6DSO32_INT1_CTRL 0x0D         ///< Interrupt control for INT 1
#define LSM6DSO32_INT2_CTRL 0x0E         ///< Interrupt control for INT 2
#define LSM6DSO32_WHOAMI 0x0F            ///< Chip ID register
#define LSM6DSO32_CTRL1_XL 0x10          ///< Main accelerometer config register
#define LSM6DSO32_CTRL2_G 0x11           ///< Main gyro config register
#define LSM6DSO32_CTRL3_C 0x12           ///< Main configuration register
#define LSM6DSO32_CTRL4_C 0x13           ///< Configuration register
#define LSM6DSO32_CTRL5_C 0x14           ///< Configuration register
#define LSM6DSO32_CTRL6_C 0x15           ///< Configuration register
#define LSM6DSO32_CTRL7_G 0x16           ///< High and low pass for gyro
#define LSM6DSO32_CTRL8_XL 0x17          ///< High and low pass for accel
#define LSM6DSO32_CTRL8_XL 0x17          ///< I3C and DEN register for accel
#define LSM6DSO32_CTRL10_C 0x19          ///< Main timestamp register
#define LSM6DSO32_ALL_INT_SRC 0x1A       ///< Source register for all interrupts
#define LSM6DSO32_WAKEUP_SRC 0x1B        ///< Wake-up interrupt source register
#define LSM6DSO32_TAP_SRC 0x1C           ///< Tap source register
#define LSM6DSO32_D6D_SRC 0x1D           ///< Portrait, landscape, face-up and face-down source register
#define LSM6DSO32_STATUS_REG 0X1E        ///< Status register
#define LSM6DSO32_OUT_TEMP_L 0x20        ///< First temp data register
#define LSM6DSO32_OUT_TEMP_H 0x21        ///< Second temp data register
#define LSM6DSO32_OUTX_L_G 0x22          ///< First gyro pitch data register
#define LSM6DSO32_OUTX_H_G 0x23          ///< Second gyro pitch data register
#define LSM6DSO32_OUTY_L_G 0x24          ///< First gyro roll data register
#define LSM6DSO32_OUTY_H_G 0x25          ///< Second gyro roll data register
#define LSM6DSO32_OUTZ_L_G 0x26          ///< First gyro yaw data register
#define LSM6DSO32_OUTZ_H_G 0x27          ///< Second gyro yaw data register
#define LSM6DSO32_OUTX_L_A 0x28          ///< First accel x data register
#define LSM6DSO32_OUTX_H_A 0x29          ///< Second accel x data register
#define LSM6DSO32_OUTY_L_A 0x2A          ///< First accel y data register
#define LSM6DSO32_OUTY_H_A 0x2B          ///< Second accel y data register
#define LSM6DSO32_OUTZ_L_A 0x2C          ///< First accel z data register
#define LSM6DSO32_OUTZ_H_A 0x2D          ///< Second accel z data register
#define LSM6DSO32_FIFO_STATUS1 0x3A      ///< FIFO status register 1
#define LSM6DSO32_FIFO_STATUS2 0x3B      ///< FIFO status register 2
#define LSM6DSO32_TIMESTAMP0 0x40        ///< Timestamp output register 1
#define LSM6DSO32_TIMESTAMP1 0x41        ///< Timestamp output register 2
#define LSM6DSO32_TIMESTAMP2 0x42        ///< Timestamp output register 3
#define LSM6DSO32_TIMESTAMP3 0x43        ///< Timestamp output register 4
#define LSM6DSO32_STEPCOUNTER 0x4B       ///< 16-bit step counter
#define LSM6DSO32_TAP_CFG0 0x56          ///< Tap/pedometer configuration register 0
#define LSM6DSO32_TAP_CFG1 0x57          ///< Tap/pedometer configuration register 1
#define LSM6DSO32_TAP_CFG2 0x58          ///< Tap/pedometer configuration register 2
#define LSM6DSO32_TAP_THS_6D 0x59        ///< Portrait/landscape position and tap function threshold register
#define LSM6DSO32_INT_DUR2 0x5A          ///< Tap recognition function setting register
#define LSM6DSO32_WAKEUP_THS 0x5B        ///< Single and double-tap function threshold register
#define LSM6DSO32_WAKEUP_DUR 0x5C        ///< Free-fall, wakeup, timestamp and sleep mode duration
#define LSM6DSO32_FREE_FALL 0x5D         ///< Free-fall function duration setting register
#define LSM6DSO32_MD1_CFG 0x5E           ///< Functions routing on INT1 register
#define LSM6DSO32_MD2_CFG 0x5F           ///< Functions routing on INT2 register
#define LSM6DSO32_FIFO_DATA_OUT_TAG 0x78 ///< FIFO tag register
#define LSM6DSO32_FIFO_DATA_OUT_X_L 0x79 ///< FIFO data output x low byte
#define LSM6DSO32_FIFO_DATA_OUT_X_H 0x7A ///< FIFO data output x high byte
#define LSM6DSO32_FIFO_DATA_OUT_Y_L 0x7B ///< FIFO data output y low byte
#define LSM6DSO32_FIFO_DATA_OUT_Y_H 0x7C ///< FIFO data output y high byte
#define LSM6DSO32_FIFO_DATA_OUT_Z_L 0x7D ///< FIFO data output z low byte
#define LSM6DSO32_FIFO_DATA_OUT_Z_H 0x7E ///< FIFO data output z high byte

/* FIFO_CTRL1 bit fields */
#define LSM6DSO32_FIFO_CTRL1_WTM_MSK 0xFF ///< FIFO watermark threshold mask, in conjunction with WTM8 in FIFO_CTRL2

/* FIFO_CTRL2 bit fields */
#define LSM6DSO32_FIFO_CTRL2_WTM8_POS 0                          ///< FIFO watermark threshold position, bit 8, in conjunction with WTM[0:7] in FIFO_CTRL1
#define LSM6DSO32_FIFO_CTRL2_WTM8_MSK (1 << FIFO_CTRL2_WTM8_POS) ///< FIFO watermark threshold mask, bit 8, in conjunction with WTM[0:7] in FIFO_CTRL1

/* FIFO_CTRL3 bit fields */
#define LSM6DSO32_FIFO_CTRL3_BDR_XL_POS 0
#define LSM6DSO32_FIFO_CTRL3_BDR_XL_MSK (0xf << FIFO_CTRL3_BDR_XL_POS)
#define LSM6DSO32_FIFO_CTRL3_BDR_GY_POS 4
#define LSM6DSO32_FIFO_CTRL3_BDR_GY_MSK (0xf << FIFO_CTRL3_BDR_GY_POS)

/* INT1_CTRL bit fields */
#define LSM6DSO32_INT1_CTRL_INT1_DRDY_XL_POS 0
#define LSM6DSO32_INT1_CTRL_INT1_DRDY_XL_MSK (1 << LSM6DSO32_INT1_CTRL_INT1_DRDY_XL_POS)
#define LSM6DSO32_INT1_CTRL_INT1_DRDY_G_POS 1
#define LSM6DSO32_INT1_CTRL_INT1_DRDY_G_MSK (1 << LSM6DSO32_INT1_CTRL_INT1_DRDY_G_POS)
#define LSM6DSO32_INT1_CTRL_INT1_BOOT_POS 2
#define LSM6DSO32_INT1_CTRL_INT1_BOOT_MSK (1 << LSM6DSO32_INT1_CTRL_INT1_BOOT_POS)
#define LSM6DSO32_INT1_CTRL_INT1_FIFO_TH_POS 3
#define LSM6DSO32_INT1_CTRL_INT1_FIFO_TH_MSK (1 << LSM6DSO32_INT1_CTRL_INT1_FIFO_TH_MSK)
#define LSM6DSO32_INT1_CTRL_INT1_FIFO_OVR_POS 4
#define LSM6DSO32_INT1_CTRL_INT1_FIFO_OVR_MSK (1 << LSM6DSO32_INT1_CTRL_INT1_FIFO_OVR_POS)
#define LSM6DSO32_INT1_CTRL_INT1_FIFO_FULL_POS 5
#define LSM6DSO32_INT1_CTRL_INT1_FIFO_FULL_MSK (1 << LSM6DSO32_INT1_CTRL_INT1_FIFO_FULL_POS)
#define LSM6DSO32_INT1_CTRL_INT1_CNT_BDR_POS 6
#define LSM6DSO32_INT1_CTRL_INT1_CNT_BDR_MSK (1 << LSM6DSO32_INT1_CTRL_INT1_CNT_BDR_POS)
#define LSM6DSO32_INT1_CTRL_DEN_DRDY_FLAG_POS 7
#define LSM6DSO32_INT1_CTRL_DEN_DRDY_FLAG_MSK (1 << LSM6DSO32_INT1_CTRL_DEN_DRDY_FLAG_POS)

/* INT1_CTRL bit fields */
#define LSM6DSO32_INT2_CTRL_INT2_DRDY_XL_POS 0
#define LSM6DSO32_INT2_CTRL_INT2_DRDY_XL_MSK (1 << LSM6DSO32_INT2_CTRL_INT2_DRDY_XL_POS)
#define LSM6DSO32_INT2_CTRL_INT2_DRDY_G_POS 1
#define LSM6DSO32_INT2_CTRL_INT2_DRDY_G_MSK (1 << LSM6DSO32_INT2_CTRL_INT2_DRDY_G_POS)
#define LSM6DSO32_INT2_CTRL_INT2_DRDY_TEMP_POS 2
#define LSM6DSO32_INT2_CTRL_INT2_DRDY_TEMP_MSK (1 << LSM6DSO32_INT2_CTRL_INT2_DRDY_TEMP_POS)
#define LSM6DSO32_INT2_CTRL_INT2_FIFO_TH_POS 3
#define LSM6DSO32_INT2_CTRL_INT2_FIFO_TH_MSK (1 << LSM6DSO32_INT2_CTRL_INT2_FIFO_TH_POS)
#define LSM6DSO32_INT2_CTRL_INT2_FIFO_OVR_POS 4
#define LSM6DSO32_INT2_CTRL_INT2_FIFO_OVR_MSK (1 << LSM6DSO32_INT2_CTRL_INT2_FIFO_OVR_POS)
#define LSM6DSO32_INT2_CTRL_INT2_FIFO_FULL_POS 5
#define LSM6DSO32_INT2_CTRL_INT2_FIFO_FULL_MSK (1 << LSM6DSO32_INT2_CTRL_INT2_FIFO_FULL_POS)
#define LSM6DSO32_INT2_CTRL_INT2_CNT_BDR_POS 6
#define LSM6DSO32_INT2_CTRL_INT2_CNT_BDR_MSK (1 << LSM6DSO32_INT2_CTRL_INT2_CNT_BDR_POS)

/* WHO_AM_I value */
#define LSM6DSO32_WHO_AM_I_VALUE 0x6C ///< LSM6DSO32 default device id from WHOAMI

/* CTRL1_XL bit fields */
#define LSM6DSO32_CTRL1_XL_LPF2_XL_EN_POS 1
#define LSM6DSO32_CTRL1_XL_LPF2_XL_EN_MSK (1 << LSM6DSO32_CTRL1_XL_LPF2_XL_EN_POS)
#define LSM6DSO32_CTRL1_XL_FS_XL_POS 2
#define LSM6DSO32_CTRL1_XL_FS_XL_MSK (3 << LSM6DSO32_CTRL1_XL_FS_XL_POS)
#define LSM6DSO32_CTRL1_XL_ODR_XL_POS 4
#define LSM6DSO32_CTRL1_XL_ODR_XL_MSK (0xf << LSM6DSO32_CTRL1_XL_ODR_XL_POS)

/* CTRL2_G bit fields */
#define LSM6DSO32_CTRL2_G_FS_125_POS 1
#define LSM6DSO32_CTRL2_G_FS_125_MSK (1 << LSM6DSO32_CTRL2_G_FS_125_POS)
#define LSM6DSO32_CTRL2_G_FS_G_POS 2
#define LSM6DSO32_CTRL2_G_FS_G_MSK (3 << LSM6DSO32_CTRL2_G_FS_G_POS)
#define LSM6DSO32_CTRL2_G_ODR_G_POS 4
#define LSM6DSO32_CTRL2_G_ODR_G_MSK (0xf << LSM6DSO32_CTRL2_G_ODR_G_POS)

/* CTRL3_C bit fields */
#define LSM6DSO32_CTRL3_C_SW_RESET_POS 0
#define LSM6DSO32_CTRL3_C_SW_RESET_MSK (1 << LSM6DSO32_CTRL3_C_SW_RESET_POS)
#define LSM6DSO32_CTRL3_C_IF_INC_POS 2
#define LSM6DSO32_CTRL3_C_IF_INC_MSK (1 << LSM6DSO32_CTRL3_C_IF_INC_POS)
#define LSM6DSO32_CTRL3_C_SIM_POS 3
#define LSM6DSO32_CTRL3_C_SIM_MSK (1 << LSM6DSO32_CTRL3_C_SIM_POS)
#define LSM6DSO32_CTRL3_C_PP_OD_POS 4
#define LSM6DSO32_CTRL3_C_PP_OD_MSK (1 << LSM6DSO32_CTRL3_C_PP_OD_POS)
#define LSM6DSO32_CTRL3_C_H_LACTIVE_POS 5
#define LSM6DSO32_CTRL3_C_H_LACTIVE_MSK (1 << LSM6DSO32_CTRL3_C_H_LACTIVE_POS)
#define LSM6DSO32_CTRL3_C_BDU_POS 6
#define LSM6DSO32_CTRL3_C_BDU_MSK (1 << LSM6DSO32_CTRL3_C_BDU_POS)
#define LSM6DSO32_CTRL3_C_BOOT_POS 7
#define LSM6DSO32_CTRL3_C_BOOT_MSK (1 << LSM6DSO32_CTRL3_C_BOOT_POS)

/* CTRL4_C bit fields */
#define LSM6DSO32_CTRL4_C_LPF1_SEL_G_POS 1
#define LSM6DSO32_CTRL4_C_LPF1_SEL_G_MSK (1 << LSM6DSO32_CTRL4_C_LPF1_SEL_G_POS)
#define LSM6DSO32_CTRL4_C_I2C_DISABLE_POS 2
#define LSM6DSO32_CTRL4_C_I2C_DISABLE_MSK (1 << LSM6DSO32_CTRL4_C_I2C_DISABLE_POS)
#define LSM6DSO32_CTRL4_C_DRDY_MASK_POS 3
#define LSM6DSO32_CTRL4_C_DRDY_MASK_MSK (1 << LSM6DSO32_CTRL4_C_DRDY_MASK_POS)
#define LSM6DSO32_CTRL4_C_INT2_ON_INT1_POS 5
#define LSM6DSO32_CTRL4_C_INT2_ON_INT1_MSK (1 << LSM6DSO32_CTRL4_C_INT2_ON_INT1_POS)
#define LSM6DSO32_CTRL4_C_SLEEP_G_POS 6
#define LSM6DSO32_CTRL4_C_SLEEP_G_MSK (1 << LSM6DSO32_CTRL4_C_SLEEP_G_POS)

/* CTRL5_C bit fields */
#define LSM6DSO32_CTRL5_C_ST_XL_POS 0
#define LSM6DSO32_CTRL5_C_ST_XL_MSK (3 << LSM6DSO32_CTRL5_C_ST_XL_POS)
#define LSM6DSO32_CTRL5_C_ST_G_POS 2
#define LSM6DSO32_CTRL5_C_ST_G_MSK (3 << LSM6DSO32_CTRL5_C_ST_G_POS)
#define LSM6DSO32_CTRL5_C_ROUNDING_POS 5
#define LSM6DSO32_CTRL5_C_ROUNDING_MSK (3 << LSM6DSO32_CTRL5_C_ROUNDING_POS)
#define LSM6DSO32_CTRL5_C_XL_ULP_EN_POS 7
#define LSM6DSO32_CTRL5_C_XL_ULP_EN_MSK (1 << LSM6DSO32_CTRL5_C_XL_ULP_EN_POS)

/* CTRL6_C bit fields */
#define LSM6DSO32_CTRL6_C_FTYPE_POS 0
#define LSM6DSO32_CTRL6_C_FTYPE_MSK (7 << LSM6DSO32_CTRL6_C_FTYPE_POS)
#define LSM6DSO32_CTRL6_C_USR_OFF_W_POS 3
#define LSM6DSO32_CTRL6_C_USR_OFF_W_MSK (1 << LSM6DSO32_CTRL6_C_USR_OFF_W_POS)
#define LSM6DSO32_CTRL6_C_XL_HM_MODE_POS 4
#define LSM6DSO32_CTRL6_C_XL_HM_MODE_MSK (1 << LSM6DSO32_CTRL6_C_XL_HM_MODE_POS)
#define LSM6DSO32_CTRL6_C_LVL2_EN_POS 5
#define LSM6DSO32_CTRL6_C_LVL2_EN_MSK (1 << LSM6DSO32_CTRL6_C_LVL2_EN_POS)
#define LSM6DSO32_CTRL6_C_LVL1_EN_POS 6
#define LSM6DSO32_CTRL6_C_LVL1_EN_MSK (1 << LSM6DSO32_CTRL6_C_LVL1_EN_POS)
#define LSM6DSO32_CTRL6_C_TRIG_EN_POS 7
#define LSM6DSO32_CTRL6_C_TRIG_EN_MSK (1 << LSM6DSO32_CTRL6_C_TRIG_EN_POS)

/* CTRL7_G bit fields */
#define LSM6DSO32_CTRL7_G_USR_OFF_ON_OUT_POS 1
#define LSM6DSO32_CTRL7_G_USR_OFF_ON_OUT_MSK (1 << LSM6DSO32_CTRL7_G_USR_OFF_ON_OUT_POS)
#define LSM6DSO32_CTRL7_G_HPM_G_POS 4
#define LSM6DSO32_CTRL7_G_HPM_G_MSK (3 << LSM6DSO32_CTRL7_G_HPM_G_POS)
#define LSM6DSO32_CTRL7_G_HP_EN_G_POS 6
#define LSM6DSO32_CTRL7_G_HP_EN_G_MSK (1 << LSM6DSO32_CTRL7_G_HP_EN_G_POS)
#define LSM6DSO32_CTRL7_G_G_HM_MODE_POS 7
#define LSM6DSO32_CTRL7_G_G_HM_MODE_MSK (1 << LSM6DSO32_CTRL7_G_G_HM_MODE_POS)

/* CTRL8_XL bit fields */
#define LSM6DSO32_CTRL8_XL_LOW_PASS_ON_6D_POS 0
#define LSM6DSO32_CTRL8_XL_LOW_PASS_ON_6D_MSK (1 << LSM6DSO32_CTRL8_XL_LOW_PASS_ON_6D_POS)
#define LSM6DSO32_CTRL8_XL_HP_SLOPE_XL_EN_POS 2
#define LSM6DSO32_CTRL8_XL_HP_SLOPE_XL_EN_MSK (1 << LSM6DSO32_CTRL8_XL_HP_SLOPE_XL_EN_POS)
#define LSM6DSO32_CTRL8_XL_FASTSETTL_MODE_XL_POS 3
#define LSM6DSO32_CTRL8_XL_FASTSETTL_MODE_XL_MSK (1 << LSM6DSO32_CTRL8_XL_FASTSETTL_MODE_XL_POS)
#define LSM6DSO32_CTRL8_XL_HP_REF_MODE_XL_POS 4
#define LSM6DSO32_CTRL8_XL_HP_REF_MODE_XL_MSK (1 << LSM6DSO32_CTRL8_XL_HP_REF_MODE_XL_POS)
#define LSM6DSO32_CTRL8_XL_HPCF_CL_POS 5
#define LSM6DSO32_CTRL8_XL_HPCF_CL_MSK (7 << LSM6DSO32_CTRL8_XL_HPCF_CL_POS)

/* STATUS_REG bit fields */
#define LSM6DSO32_STATUS_REG_XLDA_POS 0
#define LSM6DSO32_STATUS_REG_XLDA_MSK (1 << LSM6DSO32_STATUS_REG_XLDA_POS)
#define LSM6DSO32_STATUS_REG_GDA_POS 1
#define LSM6DSO32_STATUS_REG_GDA_MSK (1 << LSM6DSO32_STATUS_REG_GDA_POS)
#define LSM6DSO32_STATUS_REG_TDA_POS 2
#define LSM6DSO32_STATUS_REG_TDA_MSK (1 << LSM6DSO32_STATUS_REG_TDA_POS)

/** FIFO mode selection */
typedef enum fifo_mode
{
    LSM6DSO32_FIFO_MODE_BYPASS,
    LSM6DSO32_FIFO_MODE_FIFO,
    LSM6DSO32_FIFO_MODE_CONTINUOUS_TO_FIFO = 3,
    LSM6DSO32_FIFO_MODE_BYPASS_TO_CONTINUOUS,
    LSM6DSO32_FIFO_MODE_CONTINUOUS = 6,
    LSM6DSO32_FIFO_MODE_BYPASS_TO_FIFO,
} lsm6dso32_fifo_mode_t;

/** The accelerometer data rate */
typedef enum data_rate
{
    LSM6DSO32_RATE_SHUTDOWN,
    LSM6DSO32_RATE_12_5_HZ,
    LSM6DSO32_RATE_26_HZ,
    LSM6DSO32_RATE_52_HZ,
    LSM6DSO32_RATE_104_HZ,
    LSM6DSO32_RATE_208_HZ,
    LSM6DSO32_RATE_416_HZ,
    LSM6DSO32_RATE_833_HZ,
    LSM6DSO32_RATE_1_66K_HZ,
    LSM6DSO32_RATE_3_33K_HZ,
    LSM6DSO32_RATE_6_66K_HZ,
} lsm6dso32_data_rate_t;

/** The accelerometer data range */
typedef enum accel_range
{
    LSM6DSO32_ACCEL_RANGE_4_G,
    LSM6DSO32_ACCEL_RANGE_32_G,
    LSM6DSO32_ACCEL_RANGE_8_G,
    LSM6DSO32_ACCEL_RANGE_16_G
} lsm6dso32_accel_range_t;

/** The gyro data range */
typedef enum gyro_range
{
    LSM6DSO32_GYRO_RANGE_125_DPS = 0b0010,
    LSM6DSO32_GYRO_RANGE_250_DPS = 0b0000,
    LSM6DSO32_GYRO_RANGE_500_DPS = 0b0100,
    LSM6DSO32_GYRO_RANGE_1000_DPS = 0b1000,
    LSM6DSO32_GYRO_RANGE_2000_DPS = 0b1100,
    ISM330DHCX_GYRO_RANGE_4000_DPS = 0b0001
} lsm6dso32_gyro_range_t;

/** The high pass filter bandwidth */
typedef enum hpf_range
{
    LSM6DSO32_HPF_ODR_DIV_50 = 0,
    LSM6DSO32_HPF_ODR_DIV_100 = 1,
    LSM6DSO32_HPF_ODR_DIV_9 = 2,
    LSM6DSO32_HPF_ODR_DIV_400 = 3,
} lsm6dso32_hp_filter_t;

#endif /* LSM6DSO32_H__ */