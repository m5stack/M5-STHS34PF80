/*
 * SPDX-FileCopyrightText: 2024 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#include "M5_STHS34PF80.h"

bool M5_STHS34PF80::begin(TwoWire *wire, uint8_t addr, uint8_t sda, uint8_t scl) {
    if (!_wire) {
        _wire = wire;
        _addr = addr;
        _sda  = sda;
        _scl  = scl;
        _wire->begin(_sda, _scl);
    }
    sensor.read_reg  = M5_STHS34PF80::read;
    sensor.write_reg = M5_STHS34PF80::write;
    sensor.mdelay    = M5_STHS34PF80::delayMS;
    sensor.handle    = this;
    _addr            = addr;

    // call super class begin -- it returns 0 on no error
    return M5_STHS34PF80::init() == 0;
}

int32_t M5_STHS34PF80::read(void *device, uint8_t addr, uint8_t *data, uint16_t numData) {
    return ((M5_STHS34PF80 *)device)->readRegisterRegion(((M5_STHS34PF80 *)device)->_addr, addr, data, numData);
}

int32_t M5_STHS34PF80::write(void *device, uint8_t addr, const uint8_t *data, uint16_t numData) {
    return ((M5_STHS34PF80 *)device)->writeRegisterRegion(((M5_STHS34PF80 *)device)->_addr, addr, data, numData);
}

void M5_STHS34PF80::delayMS(uint32_t millisec) {
    delay(millisec);
}

bool M5_STHS34PF80::ping(uint8_t i2c_address) {
    if (!_wire) return false;

    _wire->beginTransmission(i2c_address);
    return _wire->endTransmission() == 0;
}

int M5_STHS34PF80::writeRegisterRegion(uint8_t i2c_address, uint8_t offset, const uint8_t *data, uint16_t length) {
    _wire->beginTransmission(i2c_address);
    _wire->write(offset);
    _wire->write(data, (int)length);

    return _wire->endTransmission() ? -1 : 0;  // -1 = error, 0 = success
}

int M5_STHS34PF80::writeRegisterRegion(uint8_t i2c_address, uint8_t offset, uint8_t data, uint16_t length) {
    _wire->beginTransmission(i2c_address);
    _wire->write(offset);
    _wire->write(data);

    return _wire->endTransmission() ? -1 : 0;  // -1 = error, 0 = success
}

int M5_STHS34PF80::readRegisterRegion(uint8_t addr, uint8_t reg, uint8_t *data, uint16_t numBytes) {
    uint8_t nChunk;
    uint16_t nReturned;

    if (!_wire) return -1;

    int i;                    // counter in loop
    bool bFirstInter = true;  // Flag for first iteration - used to send register

    while (numBytes > 0) {
        _wire->beginTransmission(addr);

        if (bFirstInter) {
            _wire->write(reg);
            bFirstInter = false;
        }

        if (_wire->endTransmission() != 0) return -1;  // error with the end transmission

        // We're chunking in data - keeping the max chunk to kMaxI2CBufferLength
        nChunk = numBytes > kChunkSize ? kChunkSize : numBytes;

        nReturned = _wire->requestFrom((int)addr, (int)nChunk, (int)true);

        // No data returned, no dice
        if (nReturned == 0) return -1;  // error

        // Copy the retrieved data chunk to the current index in the data segment
        for (i = 0; i < nReturned; i++) {
            *data++ = _wire->read();
        }

        // Decrement the amount of data recieved from the overall data request
        // amount
        numBytes = numBytes - nReturned;

    }  // end while

    return 0;  // Success
}

int32_t M5_STHS34PF80::init() {
    if (isConnected() != 0) {
        return -1;
    }

    reset();  // Set boot bit to 1, delay, then reset algorithm

    // Set temperature object number set average (AVG_TMOS = 32)
    int32_t avgErr = setAverageTObjectNumber(STHS34PF80_AVG_TMOS_32);

    // Set ambient temperature average (AVG_TAMB = 8)
    int32_t tAmbErr = setAverageTAmbientNumber(STHS34PF80_AVG_T_8);

    // Set block data rate update to true
    int32_t blockErr = setBlockDataUpdate(true);

    // Set the data rate (ODR) to 1Hz
    int32_t odrErr = setTmosODR(STHS34PF80_TMOS_ODR_AT_1Hz);

    if (avgErr != 0) {
        return avgErr;
    } else if (tAmbErr != 0) {
        return tAmbErr;
    } else if (blockErr != 0) {
        return blockErr;
    } else if (odrErr != 0) {
        return odrErr;
    }

    // If no errors, return 0
    return 0;
}

int32_t M5_STHS34PF80::isConnected() {
    uint8_t devId = 0;
    int32_t err   = sths34pf80_device_id_get(&sensor, &devId);

    if (devId != STHS34PF80_ID) {
        return -1;
    }
    return err;
}

int32_t M5_STHS34PF80::getDataReady(sths34pf80_tmos_drdy_status_t *drdy) {
    return sths34pf80_tmos_drdy_status_get(&sensor, drdy);
}

int32_t M5_STHS34PF80::getStatus(sths34pf80_tmos_func_status_t *statusVal) {
    return sths34pf80_tmos_func_status_get(&sensor, statusVal);
}

int32_t M5_STHS34PF80::reset() {
    // Set boot bit to 1 in CTRL2 register
    int32_t otpErr = setBootOTP(true);

    // delay 2.50ms to allow for sensor reset (allows for no power cycling)
    sensor.mdelay(3);

    // Reset algorithm for all register values (default values)
    int32_t resetErr = resetAlgo();

    if (otpErr != 0) {
        return otpErr;
    } else if (resetErr != 0) {
        return resetErr;
    } else {
        return 0;
    }
}

int32_t M5_STHS34PF80::getPresenceValue(int16_t *presenceVal) {
    return sths34pf80_tpresence_raw_get(&sensor, presenceVal);
}

int32_t M5_STHS34PF80::getMotionValue(int16_t *motionVal) {
    return sths34pf80_tmotion_raw_get(&sensor, motionVal);
}

int32_t M5_STHS34PF80::getTemperatureData(float *tempVal) {
    int16_t tempValFill = 0;

    // As seen on page 5 of the datasheet - object temperature sensitivity
    int16_t sensitivity = 2000;

    int32_t retVal = sths34pf80_tobject_raw_get(&sensor, &tempValFill);

    // Divide the raw value by the sensitivity
    *tempVal = (float)tempValFill / sensitivity;

    return retVal;
}

int32_t M5_STHS34PF80::getDeviceID(uint8_t *devId) {
    return sths34pf80_device_id_get(&sensor, devId);
}

int32_t M5_STHS34PF80::getAverageTObjectNumber(sths34pf80_avg_tobject_num_t *val) {
    return sths34pf80_avg_tobject_num_get(&sensor, val);
}

int32_t M5_STHS34PF80::setAverageTObjectNumber(sths34pf80_avg_tobject_num_t num) {
    return sths34pf80_avg_tobject_num_set(&sensor, num);
}

int32_t M5_STHS34PF80::getAverageTAmbientNumber(sths34pf80_avg_tambient_num_t *val) {
    return sths34pf80_avg_tambient_num_get(&sensor, val);
}

int32_t M5_STHS34PF80::setAverageTAmbientNumber(sths34pf80_avg_tambient_num_t num) {
    return sths34pf80_avg_tambient_num_set(&sensor, num);
}

int32_t M5_STHS34PF80::getGainMode(sths34pf80_gain_mode_t *gain) {
    return sths34pf80_gain_mode_get(&sensor, gain);
}

int32_t M5_STHS34PF80::setGainMode(sths34pf80_gain_mode_t gain) {
    return sths34pf80_gain_mode_set(&sensor, gain);
}

int32_t M5_STHS34PF80::getTmosSensitivity(float *sense) {
    uint16_t senseFill = 0;
    int32_t err        = sths34pf80_tmos_sensitivity_get(&sensor, &senseFill);

    uint16_t res1 = 2048;
    float res2    = 16;

    *sense = (float)(senseFill - res1) / res2;
    return err;
}

int32_t M5_STHS34PF80::setTmosSensitivity(float val) {
    uint16_t res1 = 2048;
    uint16_t res2 = 16;

    val = (val * res2) + res1;

    uint16_t valReg = val;

    return sths34pf80_tmos_sensitivity_set(&sensor, &valReg);
}

int32_t M5_STHS34PF80::getTmosODR(sths34pf80_tmos_odr_t *val) {
    return sths34pf80_tmos_odr_get(&sensor, val);
}

int32_t M5_STHS34PF80::setTmosODR(sths34pf80_tmos_odr_t val) {
    return sths34pf80_tmos_odr_set(&sensor, val);
}

int32_t M5_STHS34PF80::getBlockDataUpdate(bool *val) {
    return sths34pf80_block_data_update_get(&sensor, (uint8_t *)val);
}

int32_t M5_STHS34PF80::setBlockDataUpdate(bool val) {
    return sths34pf80_block_data_update_set(&sensor, val);
}

int32_t M5_STHS34PF80::getTmosOneShot(sths34pf80_tmos_one_shot_t *val) {
    return sths34pf80_tmos_one_shot_get(&sensor, val);
}

int32_t M5_STHS34PF80::setTmosOneShot(sths34pf80_tmos_one_shot_t val) {
    return sths34pf80_tmos_one_shot_set(&sensor, val);
}

int32_t M5_STHS34PF80::getMemoryBank(sths34pf80_mem_bank_t *val) {
    return sths34pf80_mem_bank_get(&sensor, val);
}

int32_t M5_STHS34PF80::setMemoryBank(sths34pf80_mem_bank_t val) {
    return sths34pf80_mem_bank_set(&sensor, val);
}

int32_t M5_STHS34PF80::getBootOTP(uint8_t *val) {
    return sths34pf80_boot_get(&sensor, val);
}

int32_t M5_STHS34PF80::setBootOTP(uint8_t val) {
    return sths34pf80_boot_set(&sensor, val);
}

int32_t M5_STHS34PF80::getTmosFunctionStatus(sths34pf80_tmos_func_status_t *val) {
    return sths34pf80_tmos_func_status_get(&sensor, val);
}

int32_t M5_STHS34PF80::getTObjectRawValue(int16_t *val) {
    return sths34pf80_tobject_raw_get(&sensor, val);
}

int32_t M5_STHS34PF80::getTAmbientRawValue(int16_t *val) {
    return sths34pf80_tambient_raw_get(&sensor, val);
}

int32_t M5_STHS34PF80::getTObjectCompensatedRawValue(int16_t *val) {
    return sths34pf80_tobj_comp_raw_get(&sensor, val);
}

int32_t M5_STHS34PF80::getTAmbientShockRawValue(int16_t *val) {
    return sths34pf80_tamb_shock_raw_get(&sensor, val);
}

int32_t M5_STHS34PF80::getLpfMotionBandwidth(sths34pf80_lpf_bandwidth_t *val) {
    return sths34pf80_lpf_m_bandwidth_get(&sensor, val);
}

int32_t M5_STHS34PF80::setLpfMotionBandwidth(sths34pf80_lpf_bandwidth_t val) {
    return sths34pf80_lpf_m_bandwidth_set(&sensor, val);
}

int32_t M5_STHS34PF80::getLpfPresenceMotionBandwidth(sths34pf80_lpf_bandwidth_t *val) {
    return sths34pf80_lpf_p_m_bandwidth_get(&sensor, val);
}

int32_t M5_STHS34PF80::setLpfPresenceMotionBandwidth(sths34pf80_lpf_bandwidth_t val) {
    return sths34pf80_lpf_p_m_bandwidth_set(&sensor, val);
}

int32_t M5_STHS34PF80::getLpfAmbientTempBandwidth(sths34pf80_lpf_bandwidth_t *val) {
    return sths34pf80_lpf_a_t_bandwidth_get(&sensor, val);
}

int32_t M5_STHS34PF80::setLpfAmbientTempBandwidth(sths34pf80_lpf_bandwidth_t val) {
    return sths34pf80_lpf_a_t_bandwidth_set(&sensor, val);
}

int32_t M5_STHS34PF80::getLpfPresenceBandwidth(sths34pf80_lpf_bandwidth_t *val) {
    return sths34pf80_lpf_p_bandwidth_get(&sensor, val);
}

int32_t M5_STHS34PF80::setLpfPresenceBandwidth(sths34pf80_lpf_bandwidth_t val) {
    return sths34pf80_lpf_p_bandwidth_set(&sensor, val);
}

int32_t M5_STHS34PF80::getTmosRouteInterrupt(sths34pf80_tmos_route_int_t *val) {
    return sths34pf80_tmos_route_int_get(&sensor, val);
}

int32_t M5_STHS34PF80::setTmosRouteInterrupt(sths34pf80_tmos_route_int_t val) {
    return sths34pf80_tmos_route_int_set(&sensor, val);
}

int32_t M5_STHS34PF80::getTmosInterruptOR(sths34pf80_tmos_int_or_t *val) {
    return sths34pf80_tmos_int_or_get(&sensor, val);
}

int32_t M5_STHS34PF80::setTmosInterruptOR(sths34pf80_tmos_int_or_t val) {
    return sths34pf80_tmos_int_or_set(&sensor, val);
}

int32_t M5_STHS34PF80::getInterruptMode(sths34pf80_int_mode_t *val) {
    return sths34pf80_int_mode_get(&sensor, val);
}

int32_t M5_STHS34PF80::setInterruptMode(sths34pf80_int_mode_t val) {
    return sths34pf80_int_mode_set(&sensor, val);  // Returns the error code
}

int32_t M5_STHS34PF80::getDataReadyMode(sths34pf80_drdy_mode_t *val) {
    return sths34pf80_drdy_mode_get(&sensor, val);
}

int32_t M5_STHS34PF80::setDataReadyMode(sths34pf80_drdy_mode_t val) {
    return sths34pf80_drdy_mode_set(&sensor, val);
}

int32_t M5_STHS34PF80::getPresenceThreshold(uint16_t *val) {
    return sths34pf80_presence_threshold_get(&sensor, val);
}

int32_t M5_STHS34PF80::setPresenceThreshold(uint16_t threshold) {
    return sths34pf80_presence_threshold_set(&sensor, threshold);
}

int32_t M5_STHS34PF80::getMotionThreshold(uint16_t *val) {
    return sths34pf80_motion_threshold_get(&sensor, val);
}

int32_t M5_STHS34PF80::setMotionThreshold(uint8_t threshold) {
    return sths34pf80_motion_threshold_set(&sensor, threshold);
}

int32_t M5_STHS34PF80::getTAmbientShockThreshold(uint16_t *val) {
    return sths34pf80_tambient_shock_threshold_get(&sensor, val);
}

int32_t M5_STHS34PF80::setTAmbientShockThreshold(uint16_t threshold) {
    return sths34pf80_tambient_shock_threshold_set(&sensor, threshold);
}

int32_t M5_STHS34PF80::getMotionHysteresis(uint8_t *val) {
    return sths34pf80_motion_hysteresis_get(&sensor, val);
}

int32_t M5_STHS34PF80::setMotionHysteresis(uint8_t hysteresis) {
    return sths34pf80_motion_hysteresis_set(&sensor, hysteresis);
}

int32_t M5_STHS34PF80::getPresenceHysteresis(uint8_t *val) {
    return sths34pf80_presence_hysteresis_get(&sensor, val);
}

int32_t M5_STHS34PF80::setPresenceHysteresis(uint8_t hysteresis) {
    return sths34pf80_presence_hysteresis_set(&sensor, hysteresis);
}

int32_t M5_STHS34PF80::getTAmbientShockHysteresis(uint8_t *val) {
    return sths34pf80_tambient_shock_hysteresis_get(&sensor, val);
}

int32_t M5_STHS34PF80::setTAmbientShockHysteresis(uint16_t hysteresis) {
    return sths34pf80_tambient_shock_hysteresis_set(&sensor, hysteresis);
}

int32_t M5_STHS34PF80::getInterruptPulsed(uint8_t *val) {
    return sths34pf80_int_or_pulsed_get(&sensor, val);
}

int32_t M5_STHS34PF80::setInterruptPulsed(uint8_t pulse) {
    return sths34pf80_int_or_pulsed_set(&sensor, pulse);
}

int32_t M5_STHS34PF80::getTobjectAlgoCompensation(uint8_t *val) {
    return sths34pf80_tobject_algo_compensation_get(&sensor, val);
}

int32_t M5_STHS34PF80::setTobjectAlgoCompensation(uint8_t comp) {
    return sths34pf80_tobject_algo_compensation_set(&sensor, comp);
}

int32_t M5_STHS34PF80::getPresenceAbsValue(uint8_t *val) {
    return sths34pf80_presence_abs_value_get(&sensor, val);
}

int32_t M5_STHS34PF80::setPresenceAbsValue(uint8_t val) {
    return sths34pf80_presence_abs_value_set(&sensor, val);
}

int32_t M5_STHS34PF80::resetAlgo() {
    return sths34pf80_algo_reset(&sensor);
}

int32_t M5_STHS34PF80::writeFunctionConfiguration(uint8_t addr, uint8_t *data, uint8_t len) {
    return sths34pf80_func_cfg_write(&sensor, addr, data, len);
}

int32_t M5_STHS34PF80::readFunctionConfiguration(uint8_t addr, uint8_t *data, uint8_t len) {
    return sths34pf80_func_cfg_read(&sensor, addr, data, len);
}
