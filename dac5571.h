/**
 * @copyright Private code.
 *
 * @file    dac5571.h
 * @author  Anton K.
 * @date    30 Oct 2020
 *
 * @details Driver for TI-DAC5571 Header file
 */

#ifndef DAC5571_DAC5571_H_
#define DAC5571_DAC5571_H_

/**@brief Driver return code type
 */
typedef enum {
    DAC5571_RET_CODE_SUCCESS,
    DAC5571_RET_CODE_ERROR_INVALID_STATE,
    DAC5571_RET_CODE_ERROR_INVALID_PARAM,
    DAC5571_RET_CODE_ERROR_COMMUNICATION
} DAC5571_RET_CODE_T;

/**@brief Address type. DAC5571 allowed to broadcast control and data bytes
 *  via I2C interface to synchronous several chips on the single line.
 */
typedef enum {
    DAC5571_ADDR_BROADCAST = 0x48,
    DAC5571_ADDR_SINGLECAST_A0_GND = 0x4C,
    DAC5571_ADDR_SINGLECAST_A0_VDD = 0x4D,
} DAC5571_ADDR_T;

/**@brief Reference voltage
 */
typedef enum {
    DAC5571_REF_VOLT_3_3V = 3300,
    DAC5571_REF_VOLT_5V = 5000
} DAC5571_REF_VOLT_T;

/**@brief Control modes
 */
typedef enum {
    DAC5571_MODE_NORMAL,
    DAC5571_MODE_PWD_1K,
    DAC5571_MODE_PWD_100K,
    DAC5571_MODE_PWD_HZ,

    DAC5571_MODE_COUNT
} DAC5571_MODE_T;

/**@brief I2C communication function pointer type
 *
 * @param[IN] addr - I2C address
 * @param[IN] data - data buffer pointer
 * @param[IN] len - data buffer length
 *
 * @return result code of communication.
 *  Success should be as DAC5571_RET_CODE_SUCCESS, otherwise DAC5571_RET_CODE_ERROR_COMMUNICATION
 */
typedef DAC5571_RET_CODE_T (* dac5571_i2c_com_fptr_t)(unsigned char addr, unsigned char *data, unsigned short len);

/**@brief Device driver configuration structure type
 */
typedef struct {
    const dac5571_i2c_com_fptr_t i2c_read;
    const dac5571_i2c_com_fptr_t i2c_write;
    const DAC5571_ADDR_T i2c_addr;
    const DAC5571_REF_VOLT_T ref_voltage;
} dac5571_dev_t;

/**@brief Device initialization
 * @note After using this function, driver allowed to use other functions
 *      with NULL pointer instead of (dac5571_dev_t *)
 *
 * @param[IN] dev - device control structure, @see dac5571_dev_t
 *
 * @return
 */
DAC5571_RET_CODE_T dac5571_init(const dac5571_dev_t * const dev);

/**@brief Device deinitialization function
 *
 * @return DAC5571_RET_CODE_SUCCESS - if driver was initialized before,
 *         DAC5571_RET_CODE_ERROR_INVALID_STATE - otherwise
 */
DAC5571_RET_CODE_T dac5571_deinit(void);

/**@brief Forcing device mode
 *
 * @param[IN] dev - device control structure, @see dac5571_dev_t.
 *                  NULL pointer acceptable if dac5571_init() has been executed before.
 * @param[IN] mode - device mode, @see DAC5571_MODE_T
 *
 * @return result code, @see DAC5571_RET_CODE_T
 */
DAC5571_RET_CODE_T dac5571_force_mode(const dac5571_dev_t * const dev, DAC5571_MODE_T mode);

/**@brief Forcing to output voltage data
 *  @note This function automatically forced DAC5571_MODE_NORMAL
 *
 * @param[IN] dev - device control structure, @see dac5571_dev_t.
 *                  NULL pointer acceptable if dac5571_init() has been executed before.
 * @param[IN] mV - output mVoltage data
 *
 * @return result code, @see DAC5571_RET_CODE_T
 */
DAC5571_RET_CODE_T dac5571_force_data(const dac5571_dev_t * const dev, unsigned short mV);


#endif /* DAC5571_DAC5571_H_ */
