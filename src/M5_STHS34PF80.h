/*
 * SPDX-FileCopyrightText: 2024 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#ifndef __M5UnitTMOS_H__
#define __M5UnitTMOS_H__

#include <Wire.h>
#include "sths34pf80_api/sths34pf80_reg.h"

// define a standard i2c address (7 bit) macro
#define STHS34PF80_I2C_ADDRESS (STHS34PF80_I2C_ADD >> 1)

#define kMaxTransferBuffer 32

// What we use for transfer chunk size
const static uint16_t kChunkSize = kMaxTransferBuffer;

class M5_STHS34PF80 {
   private:
    TwoWire *_wire;
    uint8_t _addr;
    uint8_t _scl;
    uint8_t _sda;

    bool ping(uint8_t address);
    int writeRegisterRegion(uint8_t address, uint8_t offset, const uint8_t *data, uint16_t length);
    int writeRegisterRegion(uint8_t address, uint8_t offset, uint8_t data, uint16_t length);
    int readRegisterRegion(uint8_t addr, uint8_t reg, uint8_t *data, uint16_t numBytes);

    static int32_t read(void *, uint8_t, uint8_t *, uint16_t);
    static int32_t write(void *, uint8_t, const uint8_t *, uint16_t);
    static void delayMS(uint32_t millisec);

   public:
    /**
     * @brief
     *
     * @param wire I2C Port
     * @param addr addr device i2c addr
     * @param sda sda sda pin number
     * @param scl scl scl pin number
     * @return bool true=success / false=fail
     */
    bool begin(TwoWire *wire = &Wire, uint8_t addr = STHS34PF80_I2C_ADDRESS, uint8_t sda = 21, uint8_t scl = 22);

    /**
     * @brief This function begins the examples/communication and sets
     *  the required values to the control registers in order for sensor
     *  use.
     * @return int32_t Error code (false is success, true is failure)
     */
    int32_t init();  // Resets the device and sets the values needed for sensor use

    /**
     * @brief This function determines whether or not the device
     *  is connected to the STHS34PF80. This tests if the device ID
     *  matches what is expected or not.
     * @return int32_t Error code (false is success, true is failure)
     */
    int32_t isConnected();  // Determines connection to device

    /**
     * @brief Checks to see if the data ready flag is high
     * @return int32_t Data ready code (0 for not ready, 1 for ready)
     */
    int32_t getDataReady(sths34pf80_tmos_drdy_status_t *drdy);  // Returns if the data is ready to be read or not

    /**
     * @brief This function checks the status of the of the device if
     *  there is data ready to be read. This value has 3 flags associated
     *  with it - ambient temperature shock, motion, and presence.
     * @return int32_t Flag data status for the 3 different outputs.
     */
    int32_t getStatus(sths34pf80_tmos_func_status_t *statusVal);  // Returns the status of the device

    /**
     * @brief This function resets the full device
     * @return int32_t Error code (0 no error)
     */
    int32_t reset();  // Set the boot bit, wait 3ms (as per the datasheet), then resets the algorithm

    /**
     * @brief This function returns the presence value (units: cm^-1)
     * TPRESENCE_L (0x3A) and T_PRESENCE_H (0x3B)
     * @param presenceVal Presence value of the device
     * @return int32_t Error code (0 no error)
     */
    int32_t getPresenceValue(int16_t *presenceVal);  // Returns the presence value detected of the device

    /**
     * @brief This function returns the motion value
     * @param motionVal Motion value of the device
     * @return int32_t Error code (0 no error)
     */
    int32_t getMotionValue(int16_t *motionVal);  // Returns the motion value

    /**
     * @brief This function returns the temeparture data.......
     * Note:the data obtained by this function is
     * infrared radiation, which cannot be converted into the actual object temperature. The measured value is relative
     * to the local temperature, and the measurement result may be negative.
     * @param tempVal Value to fill with information
     * @return int32_t Error code (0 no error)
     */
    int32_t getTemperatureData(float *tempVal);  // Returns the raw temperature value read by the device

    /**
     * @brief Returns the device ID of the STHS34PF80.
     * @param devID Device ID to return
     * @return int32_t Error code (0 no error)
     */
    int32_t getDeviceID(uint8_t *devId);  // Returns the ID of the STHS34PF80

    /**
     * @brief Returns the number of averages for the object temperature
     * @param val TObject number to write to register
     * @return int32_t Error code (0 no error)
     */
    int32_t getAverageTObjectNumber(sths34pf80_avg_tobject_num_t *val);  // Returns the number of averages

    /**
     * @brief Sets the number of averages for the object temperature
     * @param num Number of averages; Must be between 0-7
     * @return int32_t Error code (0 no error)
     */
    int32_t setAverageTObjectNumber(sths34pf80_avg_tobject_num_t num);  // Sets the # of averages for object temperature

    /**
     * @brief Returns the number of averages for the ambient temperature
     * @param val TAmbient Average to write to regsiter
     * @return int32_t Error code (0 no error)
     */
    int32_t getAverageTAmbientNumber(
        sths34pf80_avg_tambient_num_t *val);  // Returns the # of averages for ambient temperature

    /**
     * @brief Sets the number of averages for the ambient temperature
     * @param num TAmbient number to send to device
     *  STHS34PF80_AVG_T_8 = 0x0
     *  STHS34PF80_AVG_T_8 = 0x0
     *  STHS34PF80_AVG_T_4 = 0x1
     *  STHS34PF80_AVG_T_2 = 0x2
     *  STHS34PF80_AVG_T_1 = 0x3
     * @return int32_t Error code (0 no error)
     */
    int32_t setAverageTAmbientNumber(
        sths34pf80_avg_tambient_num_t num);  // Sets the # of averages for ambient temperature

    /**
     * @brief Returns the number of averages selected for ambient temperature
     * @param gain Gain to write to device
     * @return int32_t Error code (0 no error)
     */
    int32_t getGainMode(sths34pf80_gain_mode_t *gain);  // Returns the gain mode of the temperature range

    /**
     * @brief Sets the gain mode of the device. This enables the device
     *  to cover a wide operating temperature range for applications
     *  that might be thermally heated inside of the application.
     * @param mode Gain mode to set to device
     *  STHS34PF80_GAIN_WIDE_MODE     = 0x0
     *  STHS34PF80_GAIN_DEFAULT_MODE  = 0x1
     * @return int32_t Error code (0 no error)
     */
    int32_t setGainMode(sths34pf80_gain_mode_t mode);  // Sets the gain mode of the temperature range

    /**
     * @brief Returns the sensitivity value in the embedded linear
     *  algorithm for compensating ambient temperature variations in
     *  the object temperature.
     * @param sense Value to set the sensitivity
     * @return int32_t Error code (0 no error)
     */
    int32_t getTmosSensitivity(float *sense);  // Returns the senstivity of data of the TMOS interface data

    /**
     * @brief Sets the sensitivity value in embedded linear
     *  algorithm for compensating ambient temperature variations in
     *  the object temperature.
     * @param val Sensitivity value to be converted to send to regsiter
     * @return int32_t Error code (0 no error)
     */
    int32_t setTmosSensitivity(float val);  // Sets the sensitivity data for the TMOS interface status

    /**
     * @brief This function returns the output data rate.
     * @param val Value to set the ODR rate
     *  STHS34PF80_TMOS_ODR_OFF
     *  STHS34PF80_TMOS_ODR_AT_0Hz25
     *  STHS34PF80_TMOS_ODR_AT_0Hz50
     *  STHS34PF80_TMOS_ODR_AT_1Hz
     *  STHS34PF80_TMOS_ODR_AT_2Hz
     *  STHS34PF80_TMOS_ODR_AT_4Hz
     *  STHS34PF80_TMOS_ODR_AT_8Hz
     *  STHS34PF80_TMOS_ODR_AT_15Hz
     *  STHS34PF80_TMOS_ODR_AT_30Hz
     * @return int32_t Error code (0 no error)
     */
    int32_t getTmosODR(sths34pf80_tmos_odr_t *val);  // Returns the block data update feature for output registers

    /**
     * @brief Sets the output data rate
     * @param val 0x0 - 0x8
     *  STHS34PF80_TMOS_ODR_OFF
     *  STHS34PF80_TMOS_ODR_AT_0Hz25
     *  STHS34PF80_TMOS_ODR_AT_0Hz50
     *  STHS34PF80_TMOS_ODR_AT_1Hz
     *  STHS34PF80_TMOS_ODR_AT_2Hz
     *  STHS34PF80_TMOS_ODR_AT_4Hz
     *  STHS34PF80_TMOS_ODR_AT_8Hz
     *  STHS34PF80_TMOS_ODR_AT_15Hz
     *  STHS34PF80_TMOS_ODR_AT_30Hz
     * @return int32_t Error code (0 no error)
     */
    int32_t setTmosODR(sths34pf80_tmos_odr_t val);  // Sets the block data update feature

    /**
     * @brief This function enables the block data update feature
     *  for output registers TOBJECT (26h - 27h) and TAMBIENT (28h - 29h).
     * @param val Block data update bit (0 disabled, 1 enabled), -1 for error
     * @return int32_t Error code (0 no error)
     */
    int32_t getBlockDataUpdate(bool *val);  // Enables the block data update feature

    /**
     * @brief This function sets the block data update feature
     *  for output registeres TOBJECT (26h - 27h) and TAMBIENT (28h - 29h).
     *  Block data update bit (0 disabled, 1 enabled)
     * @param val Value to set the block data update bit
     * @return int32_t Error code (0 no error)
     */
    int32_t setBlockDataUpdate(bool val);  // Sets the block data

    /**
     * @brief This function returns the status of the trigger one-shot
     *  acquisition.
     * @param val 0 for idle, 1 for new data set acquired, -1 for error
     *  STHS34PF80_TMOS_IDLE_MODE = 0x0
     *  STHS34PF80_TMOS_ONE_SHOT = 0x1
     * @return int32_t Error code (0 no error)
     */
    int32_t getTmosOneShot(sths34pf80_tmos_one_shot_t *val);  // Returns the state of the trigger one-shot acquisition

    /**
     * @brief This function returns the status of the trigger one-shot
     *  acquistion. It is self-clearing upon completion (1)
     * @param val Value to set to register
     * @return int32_t Error code (0 no error)
     */
    int32_t setTmosOneShot(sths34pf80_tmos_one_shot_t val);  // Sets the trigger one-shot acquisiton

    /**
     * @brief This function enables access to the registers for embedded
     *  functions. Default value: 0
     * @param val Memory bank enabled for embedded or main fucntions
     *  STHS34PF80_MAIN_MEM_BANK = 0x0
     *  STHS34PF80_EMBED_FUNC_MEM_BANK = 0x1
     * @return int32_t Error code (0 no error)
     */
    int32_t getMemoryBank(
        sths34pf80_mem_bank_t *val);  // Returns the state of the access to the registers for embedded functions

    /**
     * @brief This function enables access to the registers for embedded
     *  functions. Default value: 0
     * @param val Value to enable access to the register
     *  STHS34PF80_MAIN_MEM_BANK = 0x0
     *  STHS34PF80_EMBED_FUNC_MEM_BANK = 0x1
     * @return int32_t Error code (0 no error)
     */
    int32_t setMemoryBank(
        sths34pf80_mem_bank_t val);  // Sets the state of access to the registers for embedded functions

    /**
     * @brief This function creates a global reset of the device -
     *  Reboot OTP memory content. Self-clearing upon completion
     * @param val Value to fill for boot OTP
     * @return int32_t Error code (0 no error)
     */
    int32_t getBootOTP(uint8_t *val);  // Returns the reboot OTP memory content value

    /**
     * @brief This function creates a global reset of the device -
     *  Reboot OTP memory content. Self-clearing upon completion
     * @param val Value to set the Boot OTP
     * @return int32_t Error code (0 no error)
     */
    int32_t setBootOTP(uint8_t val);  // Self-clears the boot OTP memory content

    /**
     * @brief This function returns the TMOS function status. This holds
     *  information for the presence deteciton , motion detection, and
     *  ambient shock detection flags. The flags will be raised when
     *  there is detection in any of those fields.
     * @param val Fills with the function status value
     * @return int32_t Error code (0 no error)
     */
    int32_t getTmosFunctionStatus(sths34pf80_tmos_func_status_t *val);  // Returns the struct for using flags to get the
                                                                        // ambient temp, motion, and presence flags

    /**
     * @brief This function returns the raw output value that represents
     *  the amount of infrared radiation emitted from the objects inside
     *  the field of view. It is composed of TOBJECT_H and TOBJECT_L (0x27
     *  and 0x26). The value is expressed as 2's complement
     * @param val Raw value from the TObject registers
     * @return int32_t Error code (0 no error)
     */
    int32_t getTObjectRawValue(int16_t *val);  // Returns the raw value of the TObject Regsiters

    /**
     * @brief This function returns the raw output value for the value that
     *  represents the temperature of the environment in thermal coupling
     *  with the sensor. It is composed of TAMBIENT_H and TAMBIENT_L (0x28
     *  and 0x29). The value is expressed as 2's complement.
     *  Sensitivity = 100 LSB/°C
     * @param val Raw value from TAmbient registers
     * @return int32_t Error code (0 no error)
     */
    int32_t getTAmbientRawValue(int16_t *val);  // Returns the raw value of the TAmbient Registers

    /**
     * @brief This function returns the raw output value for the data that
     *  represents the amount of infrared radiation emitted from the objects
     *  inside the field of view compensated through the embedded algorithm
     *  for compensating ambient temperature varations. The output data is
     *  composed of TOBJ_COMP_H and TOBJ_COMP_L (0x39 and 0x38). The value
     *  is expressed as 2's complement.
     * @param val Raw value from TObj_Comp registers
     * @return int32_t Error code (0 no error)
     */
    int32_t getTObjectCompensatedRawValue(int16_t *val);  // Returns the raw value of the TObject Compensated Registers

    /**
     * @brief This function returns the raw output value for the data that
     *  represents the ambient temperature shock data. It is composed of
     *  TAMB_SHOCK_H and TAMB_SHOCK_L (0x3F and 0x3E). The value is
     *  expressed as 2's complement
     * @param val Raw value from TAmb_Shock registers
     * @return int32_t Error code (0 no error)
     */
    int32_t getTAmbientShockRawValue(int16_t *val);  // Returns the raw value of the TAmbient Shock Registers

    /**
     * @brief This function returns the Low Pass Filter configuration
     *  for motion detection.
     * @param val Low Pass Filter Motion Bandwidth value
     *  STHS34PF80_LPF_ODR_DIV_9   = 0x0
     *  STHS34PF80_LPF_ODR_DIV_20  = 0x1
     *  STHS34PF80_LPF_ODR_DIV_50  = 0x2
     *  STHS34PF80_LPF_ODR_DIV_100 = 0x3
     *  STHS34PF80_LPF_ODR_DIV_200 = 0x4
     *  STHS34PF80_LPF_ODR_DIV_400 = 0x5
     *  STHS34PF80_LPF_ODR_DIV_800 = 0x6
     * @return int32_t Error code (0 no error)
     */
    int32_t getLpfMotionBandwidth(
        sths34pf80_lpf_bandwidth_t *val);  // Returns the low pass filter configuration for motion

    /**
     * @brief This function sets the Low Pass Filter configuration
     *  for Motion detection.
     * @param val LPF Motion Bandwidth configuration
     *  STHS34PF80_LPF_ODR_DIV_9   = 0x0
     *  STHS34PF80_LPF_ODR_DIV_20  = 0x1
     *  STHS34PF80_LPF_ODR_DIV_50  = 0x2
     *  STHS34PF80_LPF_ODR_DIV_100 = 0x3
     *  STHS34PF80_LPF_ODR_DIV_200 = 0x4
     *  STHS34PF80_LPF_ODR_DIV_400 = 0x5
     *  STHS34PF80_LPF_ODR_DIV_800 = 0x6
     * @return int32_t Error code (0 no error)
     */
    int32_t setLpfMotionBandwidth(sths34pf80_lpf_bandwidth_t val);  // Sets the low-pass filter configuration for motion

    /**
     * @brief This function returns the Low Pass Filter configuration
     *  for presence and motion detection.
     * @param val Value to fill with Low Pass Filter configuration
     *  STHS34PF80_LPF_ODR_DIV_9   = 0x0
     *  STHS34PF80_LPF_ODR_DIV_20  = 0x1
     *  STHS34PF80_LPF_ODR_DIV_50  = 0x2
     *  STHS34PF80_LPF_ODR_DIV_100 = 0x3
     *  STHS34PF80_LPF_ODR_DIV_200 = 0x4
     *  STHS34PF80_LPF_ODR_DIV_400 = 0x5
     *  STHS34PF80_LPF_ODR_DIV_800 = 0x6
     * @return int32_t Low Pass Filter Motion Bandwidth value, -1 for error
     */
    int32_t getLpfPresenceMotionBandwidth(
        sths34pf80_lpf_bandwidth_t *val);  // Returns the low pass filter configuration for motion and presence

    /**
     * @brief This function sets the Low Pass Filter configuration
     *  for Presence and Motion detection.
     * @param val Value to set the LPF configuration
     *  STHS34PF80_LPF_ODR_DIV_9   = 0x0
     *  STHS34PF80_LPF_ODR_DIV_20  = 0x1
     *  STHS34PF80_LPF_ODR_DIV_50  = 0x2
     *  STHS34PF80_LPF_ODR_DIV_100 = 0x3
     *  STHS34PF80_LPF_ODR_DIV_200 = 0x4
     *  STHS34PF80_LPF_ODR_DIV_400 = 0x5
     *  STHS34PF80_LPF_ODR_DIV_800 = 0x6
     * @return int32_t Error code (0 no error)
     */
    int32_t setLpfPresenceMotionBandwidth(
        sths34pf80_lpf_bandwidth_t val);  // Sets the low-pass filter configuration for motion and presence

    /**
     * @brief This function returns the Low Pass Filter configuration
     *  for Ambient Temperature Shock detection.
     * @param val Value to fill with the LPF ambient temp shock detection
     *  STHS34PF80_LPF_ODR_DIV_9   = 0x0
     *  STHS34PF80_LPF_ODR_DIV_20  = 0x1
     *  STHS34PF80_LPF_ODR_DIV_50  = 0x2
     *  STHS34PF80_LPF_ODR_DIV_100 = 0x3
     *  STHS34PF80_LPF_ODR_DIV_200 = 0x4
     *  STHS34PF80_LPF_ODR_DIV_400 = 0x5
     *  STHS34PF80_LPF_ODR_DIV_800 = 0x6
     * @return int32_t Low Pass Filter Ambient Temperature Shock Bandwidth value, -1 for error
     */
    int32_t getLpfAmbientTempBandwidth(
        sths34pf80_lpf_bandwidth_t *val);  // Returns the low pass filter config for ambient temperature shock detection

    /**
     * @brief This function sets the Low Pass Filter configuration
     *  for Ambient Temperature Shock detection.
     * @param val Value to set the ambient tempertature shock detection
     *  STHS34PF80_LPF_ODR_DIV_9   = 0x0
     *  STHS34PF80_LPF_ODR_DIV_20  = 0x1
     *  STHS34PF80_LPF_ODR_DIV_50  = 0x2
     *  STHS34PF80_LPF_ODR_DIV_100 = 0x3
     *  STHS34PF80_LPF_ODR_DIV_200 = 0x4
     *  STHS34PF80_LPF_ODR_DIV_400 = 0x5
     *  STHS34PF80_LPF_ODR_DIV_800 = 0x6
     * @return int32_t Error code (0 no error)
     */
    int32_t setLpfAmbientTempBandwidth(sths34pf80_lpf_bandwidth_t val);  // Sets the low-pass filter configuration for
                                                                         // ambient temperature shock detection

    /**
     * @brief This function returns the Low Pass Filter configuration
     *  for Ambient Temperature Shock detection.
     * @param val Value to fill with the LPF presence bandwidth value
     *  STHS34PF80_LPF_ODR_DIV_9   = 0x0
     *  STHS34PF80_LPF_ODR_DIV_20  = 0x1
     *  STHS34PF80_LPF_ODR_DIV_50  = 0x2
     *  STHS34PF80_LPF_ODR_DIV_100 = 0x3
     *  STHS34PF80_LPF_ODR_DIV_200 = 0x4
     *  STHS34PF80_LPF_ODR_DIV_400 = 0x5
     *  STHS34PF80_LPF_ODR_DIV_800 = 0x6
     * @return int32_t Error code (0 no error)
     */
    int32_t getLpfPresenceBandwidth(
        sths34pf80_lpf_bandwidth_t *val);  // Returns the low pass filter config for the presence bandwidth

    /**
     * @brief This function sets the Low Pass Filter configuration
     *  for Presence detection.
     * @param val Value to set the LPF presence bandwidth
     *  STHS34PF80_LPF_ODR_DIV_9   = 0x0
     *  STHS34PF80_LPF_ODR_DIV_20  = 0x1
     *  STHS34PF80_LPF_ODR_DIV_50  = 0x2
     *  STHS34PF80_LPF_ODR_DIV_100 = 0x3
     *  STHS34PF80_LPF_ODR_DIV_200 = 0x4
     *  STHS34PF80_LPF_ODR_DIV_400 = 0x5
     *  STHS34PF80_LPF_ODR_DIV_800 = 0x6
     * @return int32_t Error code (0 no error)
     */
    int32_t setLpfPresenceBandwidth(
        sths34pf80_lpf_bandwidth_t val);  //  // Sets the low-pass filter configuration for presence detection

    /**
     * @brief This function returns the interrupts to be routed.
     *  See page 26, Table 10.9 of the datasheet for more information
     * @param val Value to fill with the types of interrupts to be routed
     *  STHS34PF80_TMOS_INT_HIZ = 0x0
     *  STHS34PF80_TMOS_INT_DRDY = 0x1
     *  STHS34PF80_TMOS_INT_OR = 0x2
     * @return int32_t Error code (0 no error)
     */
    int32_t getTmosRouteInterrupt(sths34pf80_tmos_route_int_t *val);  // Returns the Interrupts to be routed

    /**
     * @brief This function sets the interrupts to be routed.
     *  See page 26, Table 9 of the datasheet for more information
     * @param val Value to set the interrupts to be routed
     *  STHS34PF80_TMOS_INT_HIZ = 0x0
     *  STHS34PF80_TMOS_INT_DRDY = 0x1
     *  STHS34PF80_TMOS_INT_OR = 0x2
     * @return int32_t Error code (0 no error)
     */
    int32_t setTmosRouteInterrupt(sths34pf80_tmos_route_int_t val);  // Sets the Interrutps to be routed

    /**
     * @brief This function returns the selects interrupts output
     * @param val Value to fill with the select interrupt output type
     *  STHS34PF80_TMOS_INT_NONE = 0x0
     *  STHS34PF80_TMOS_INT_TSHOCK = 0x1
     *  STHS34PF80_TMOS_INT_MOTION = 0x2
     *  STHS34PF80_TMOS_INT_TSHOCK_MOTION = 0x3
     *  STHS34PF80_TMOS_INT_PRESENCE = 0x4
     *  STHS34PF80_TMOS_INT_TSHOCK_PRESENCE = 0x5
     *  STHS34PF80_TMOS_INT_MOTION_PRESENCE = 0x6
     *  STHS34PF80_TMOS_INT_ALL = 0x7
     * @return int32_t Error code (0 no error)
     */
    int32_t getTmosInterruptOR(sths34pf80_tmos_int_or_t *val);  // Returns the Interrupts Output type

    /**
     * @brief This function sets the selects interrupts output
     * @param val Value to write to the register. See above defines
     *  for what can be used
     *  STHS34PF80_TMOS_INT_NONE = 0x0
     *  STHS34PF80_TMOS_INT_TSHOCK = 0x1
     *  STHS34PF80_TMOS_INT_MOTION = 0x2
     *  STHS34PF80_TMOS_INT_TSHOCK_MOTION = 0x3
     *  STHS34PF80_TMOS_INT_PRESENCE = 0x4
     *  STHS34PF80_TMOS_INT_TSHOCK_PRESENCE = 0x5
     *  STHS34PF80_TMOS_INT_MOTION_PRESENCE = 0x6
     *  STHS34PF80_TMOS_INT_ALL = 0x7
     * @return int32_t Error code (0 no error)
     */
    int32_t setTmosInterruptOR(sths34pf80_tmos_int_or_t val);  // Sets the interrupts output type

    /**
     * @brief This function returns the interrupt trigger mode
     *  for the push-pull/open-drain selection on INT1 and INT2 pins.
     * @param val Interrupt mode (H or L), -1 for error
     *  STHS34PF80_ACTIVE_HIGH = 0x0
     *  STHS34PF80_ACTIVE_LOW = 0x1
     * @return int32_t Error code (0 no error)
     */
    int32_t getInterruptMode(sths34pf80_int_mode_t *val);  // Returns the interrupt mode currently set on the device

    /**
     * @brief This function sets the interrupt trigger mode
     *  for the push-pull/open-drain selection on INT1 and INT2 pins.
     * @param val Value the user sets to write to the device (0 or 1)
     *  STHS34PF80_ACTIVE_HIGH = 0x0
     *  STHS34PF80_ACTIVE_LOW = 0x1
     * @return int32_t Error code (0 no error)
     */
    int32_t setInterruptMode(sths34pf80_int_mode_t val);  // Sets the interrupt mode of the device

    /**
     * @brief This function returns the data ready mode to pulsed (0)
     *  on the INT pin, or latched (1).
     * @param val Data ready mode (pulsed or latched)
     *  STHS34PF80_DRDY_PULSED = 0x0
     *  STHS34PF80_DRDY_LATCHED = 0x1
     * @return int32_t Error code (0 no error)
     */
    int32_t getDataReadyMode(sths34pf80_drdy_mode_t *val);  // Returns the data ready mode currently set for device

    /**
     * @brief This function sets the data ready mode to be pulsed (0)
     *  on the INT pin, or latched (1).
     * @param val Value the user sets to write to the device (0 or 1)
     *  STHS34PF80_DRDY_PULSED = 0x0
     *  STHS34PF80_DRDY_LATCHED = 0x1
     * @return int32_t Error code (0 no error)
     */
    int32_t setDataReadyMode(sths34pf80_drdy_mode_t val);  // Sets the data ready mode of the device (0 or 1)

    /**
     * @brief This function returns the presence threshold of the device.
     * @param val Presence threshold of the device
     *  Default value is 200 (0x008C)
     * @return int32_t Error code (0 for no error)
     */
    int32_t getPresenceThreshold(uint16_t *val);  // Returns the presence threshold value read

    /**
     * @brief This function sets the presence threshold for the device.
     * @param threshold 15-bit unsigned integer
     *  Default value is 200 (0x00C8).
     * @return int32_t Error code (0 for no error)
     */
    int32_t setPresenceThreshold(uint16_t threshold);  // Sets the presence threshold value of the device

    /**
     * @brief This function returns the motion threshold of the device.
     * @param val Motion threshold
     *  Default value is 200 (0x00C8)
     * @return int32_t Error code (0 for no error)
     */
    int32_t getMotionThreshold(uint16_t *val);  // Returns the motion threshold of the device

    /**
     * @brief This function sets the motion threshold of the device.
     * @param threshold 15-bit unsigned integer
     *  Default value is 200 (0x00C8).
     * @return int32_t Error code (0 for no error)
     */
    int32_t setMotionThreshold(uint8_t threshold);  // Sets the motion threshold value of the device

    /**
     * @brief This function returns the ambient temperature threshold of
     *  the device. Default value is 10 (0x000A).
     * @param val Ambient temperature shock threshold
     * @return int32_t Error code (0 for no error)
     */
    int32_t getTAmbientShockThreshold(uint16_t *val);  // Returns the ambient temperature threshold of the device

    /**
     * @brief This function sets the ambient temperature threshold of the
     *  device. Default value is 10 (0x000A).
     * @param threshold 15-bit unsigned integer
     * @return int32_t Error code (0 for no error)
     */
    int32_t setTAmbientShockThreshold(uint16_t threshold);  // Sets the ambient temperature threshold of the device

    /**
     * @brief This function sets the motion hysteresis of the device.
     * @param val Motion hysteresis
     *  Default value is 0x32.
     * @return int32_t Error code (0 for no error)
     */
    int32_t getMotionHysteresis(uint8_t *val);  // Returns the motion hysterssis of the device

    /**
     * @brief This function returns the motion hysteresis of the device.
     * @param hysteresis 8-bit unsigned integer
     *  Default value is 0x32.
     * @return int32_t Error code (0 for no error)
     */
    int32_t setMotionHysteresis(uint8_t hysteresis);  // Sets the motion hystersis of the device

    /**
     * @brief This function returns the presence hysteresis of the
     *  device. Default value is 0x32.
     * @param val Presence hysteresis
     * @return int32_t Error code (0 for no error)
     */
    int32_t getPresenceHysteresis(uint8_t *val);  // Returns the presence hystersis of the device

    /**
     * @brief This function sets the presence hysteresis of the device.
     *  Default value is 0x32.
     * @param hysteresis 8-bit unsigned integer.
     * @return int32_t Error code (0 for no error)
     */
    int32_t setPresenceHysteresis(uint8_t hysteresis);  // Sets the presence hystersis of the device

    /**
     * @brief This function returns the ambient temperature shock hysteresis
     *  of the device. Default value is 10 (0xA).
     * @param val Ambient temperature shock hysteresis value
     * @return int32_t Error code (0 for no error)
     */
    int32_t getTAmbientShockHysteresis(uint8_t *val);  // Returns the ambient temperature shock hystersis of the device

    /**
     * @brief This function sets the ambient temperature shock hysteresis
     *  of the device. Default value is 10 (0xA).
     * @param hysteresis 15-bit unsigned integer for temp ambient shock hysteresis
     * @return int32_t Error code (0 for no error)
     */
    int32_t setTAmbientShockHysteresis(
        uint16_t hysteresis);  // Sets the ambient temperature shock hystersis of the device

    /**
     * @brief This function returns if the flag is latched or pulsed
     *  on the INT pin. Default value = 0.
     * @param val Latched (0) or Pulsed (1)
     * @return int32_t Error code (0 for no error)
     */
    int32_t getInterruptPulsed(uint8_t *val);  // Returns the result of the algorithms if they are pulsed or not

    /**
     * @brief This function sets the flag for if the result of the
     *  algorithms are pulsed (high for ODR defined) on the INT pin.
     * @param pulse int_pulsed value (0, 1)
     *  Default value = 0
     * @return int32_t Error code (0 no error)
     */
    int32_t setInterruptPulsed(uint8_t pulse);  // Sets the device's result of the algorithms to pulsed or not

    /**
     * @brief This function returns the Tobject compnesation. This is
     *  an embedded linear algorithm for compensating ambient temperature
     *  variations in the object temperature. Default value is 0.
     * @param val Disabled (0) or Enabled (1)
     * @return int32_t Error code (0 no error)
     */
    int32_t getTobjectAlgoCompensation(
        uint8_t *val);  // Returns if the device is using the algorithm for compensating temp variations

    /**
     * @brief This function sets the Tobject compnesation. This is
     *  an embedded linear algorithm for compensating ambient temperature
     *  variations in the object temperature. Default value is 0.
     * @param comp Ambient compensation for object temperature (0, 1)
     * @return int32_t Error code (0 no error)
     */
    int32_t setTobjectAlgoCompensation(uint8_t comp);  // Sets the devices Tobject compensation

    /**
     * @brief This function returns the enable/disable bit for setting
     *  the presence absolute value algorithm
     * @param val Absolute value NOT applied (0) or applied (1)
     * @return int32_t Error code (0 no error)
     */
    int32_t getPresenceAbsValue(
        uint8_t *val);  // Returns if there is absolute value added in the presence detection algorithm

    /**
     * @brief This function returns the enable/disable bit for setting
     *  the presence absolute value algorithm
     * @param val Presence absolute value (0, 1)
     * @return int32_t Error code (0 no error)
     */
    int32_t setPresenceAbsValue(uint8_t val);  // Sets the absolute value added in the presence detection algorithm

    /**
     * @brief This function resets the algorithm whenever parameters
     *  are modified. The user is required to call this after modifying
     *  any parameters.
     * @return int32_t Error code (0 no error)
     */
    int32_t resetAlgo();  // Resets the algo structure of the device

    /**
     * @brief This function writes/sets data to the desired address for the desired
     * number of bytes.
     * @param addr embedded register address
     * @param data embedded register data
     * @param len  embedded register data len
     * @return int32_t Interface status (0 for no error, -1 for error)
     */
    int32_t writeFunctionConfiguration(uint8_t addr, uint8_t *data, uint8_t len);  // Write interface definition

    /**
     * @brief This function returns/gets data from the desired address for the chosen
     * number of bytes. Note that this is limited to a 32 bit value for length.
     * @param addr embedded register address
     * @param data embedded register data
     * @param len  embedded register data len
     * @return int32_t Interface status (0 for no error, -1 for error)
     */
    int32_t readFunctionConfiguration(uint8_t addr, uint8_t *data, uint8_t len);  // Read interface defintions

   protected:
    stmdev_ctx_t sensor;
};

#endif