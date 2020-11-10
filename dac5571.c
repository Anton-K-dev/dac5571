/**
 * @copyright Private code.
 *
 * @file    dac5571.c
 * @author  Anton K.
 * @date    30 Oct 2020
 *
 * @details Driver for TI-DAC5571 Source code
 */

#include <stddef.h>
#include <string.h>

#include "dac5571.h"

#define DAC5571_RESOLUTION_MASK     0xFF

#define DAC5571_REG_BYTE_COUNT              2
#define DAC5571_REG_DATA_HALF_BYTE_OFFSET   4
#define DAC5571_REG_MODE_OFFSET             4

struct {
    dac5571_dev_t dev;
    unsigned char is_init:1;
} dac5571_handle;

/**@brief Driver device structure verification
 *
 * @param[IN] dev - device control structure, @see dac5571_dev_t
 *
 * @return result code, @see DAC5571_RET_CODE_T
 */
static DAC5571_RET_CODE_T __dac5571_verify_device(const dac5571_dev_t * const dev)
{
    //! Verify I2C communication functions
    if (dev->i2c_read == NULL || dev->i2c_write == NULL)
        return DAC5571_RET_CODE_ERROR_INVALID_PARAM;

    //! Verify I2C address
    if (dev->i2c_addr != DAC5571_ADDR_BROADCAST
        && dev->i2c_addr != DAC5571_ADDR_SINGLECAST_A0_GND
        && dev->i2c_addr != DAC5571_ADDR_SINGLECAST_A0_VDD)
        return DAC5571_RET_CODE_ERROR_INVALID_PARAM;

    //! Verify reference voltage
    if (dev->ref_voltage != DAC5571_REF_VOLT_3_3V && dev->ref_voltage != DAC5571_REF_VOLT_5V)
        return DAC5571_RET_CODE_ERROR_INVALID_PARAM;

    return DAC5571_RET_CODE_SUCCESS;
}

DAC5571_RET_CODE_T dac5571_init(const dac5571_dev_t * const dev)
{
    DAC5571_RET_CODE_T ret_code;

    //! Check does driver already initialized
    if (dac5571_handle.is_init) {
        ret_code = DAC5571_RET_CODE_ERROR_INVALID_STATE;
        goto __ret;
    }

    //! Verify parameters
    ret_code = __dac5571_verify_device(dev);
    if (ret_code != DAC5571_RET_CODE_SUCCESS)
        goto __ret;

    //! Copy device data to internal structure
    memcpy(&dac5571_handle.dev, dev, sizeof(dac5571_dev_t));

    dac5571_handle.is_init = 1;

__ret:
    return ret_code;
}

DAC5571_RET_CODE_T dac5571_deinit(void)
{
    //! Check does driver has been initialized
    if (dac5571_handle.is_init) {
        dac5571_handle.is_init = 0;
        return DAC5571_RET_CODE_SUCCESS;
    } else {
        return DAC5571_RET_CODE_ERROR_INVALID_STATE;
    }
}


DAC5571_RET_CODE_T dac5571_force_mode(const dac5571_dev_t * const dev, DAC5571_MODE_T mode)
{
    DAC5571_RET_CODE_T ret_code;
    dac5571_dev_t *dev_ptr;
    unsigned char data[2];

    //! Verify mode
    if (mode >= DAC5571_MODE_COUNT) {
        ret_code = DAC5571_RET_CODE_ERROR_INVALID_PARAM;
        goto __ret;
    }

    //! Verify the way of device structure usage
    if (dac5571_handle.is_init) {
        //! Use internal device structure
        dev_ptr = &dac5571_handle.dev;
    } else if (dev != NULL) {
        //! Verify parameters
        ret_code = __dac5571_verify_device(dev);
        if (ret_code != DAC5571_RET_CODE_SUCCESS)
            goto __ret;

        //! Use input device structure
        dev_ptr = (dac5571_dev_t *)dev;
    } else {
        ret_code = DAC5571_RET_CODE_ERROR_INVALID_STATE;
        goto __ret;
    }

    data[0] = mode << DAC5571_REG_MODE_OFFSET;
    data[1] = 0;

    ret_code = dev_ptr->i2c_write(dev_ptr->i2c_addr, data, DAC5571_REG_BYTE_COUNT);
    if (ret_code != DAC5571_RET_CODE_SUCCESS)
        goto __ret;

__ret:
    return ret_code;
}

DAC5571_RET_CODE_T dac5571_force_data(const dac5571_dev_t * const dev, unsigned short mV)
{
    DAC5571_RET_CODE_T ret_code;
    dac5571_dev_t *dev_ptr;
    unsigned char data[2];

    //! Verify the way of device structure usage
    if (dac5571_handle.is_init) {
        //! Use internal device structure
        dev_ptr = &dac5571_handle.dev;
    } else if (dev != NULL) {
        //! Verify parameters
        ret_code = __dac5571_verify_device(dev);
        if (ret_code != DAC5571_RET_CODE_SUCCESS)
            goto __ret;

        //! Use input device structure
        dev_ptr = (dac5571_dev_t *)dev;
    } else {
        ret_code = DAC5571_RET_CODE_ERROR_INVALID_STATE;
        goto __ret;
    }

    //! Verify input voltage value
    if (mV > dev_ptr->ref_voltage) {
        ret_code = DAC5571_RET_CODE_ERROR_INVALID_PARAM;
        goto __ret;
    }

    //! Calculate data bits
    data[0] = mV * DAC5571_RESOLUTION_MASK / dev_ptr->ref_voltage;

    //! Data control and voltage composition
    data[1] = (data[0] & 0x0F) << DAC5571_REG_DATA_HALF_BYTE_OFFSET;
    data[0] = (data[0] >> DAC5571_REG_DATA_HALF_BYTE_OFFSET) | (DAC5571_MODE_NORMAL << DAC5571_REG_MODE_OFFSET);

    ret_code = dev_ptr->i2c_write(dev_ptr->i2c_addr, data, DAC5571_REG_BYTE_COUNT);
    if (ret_code != DAC5571_RET_CODE_SUCCESS)
        goto __ret;

__ret:
    return ret_code;
}


