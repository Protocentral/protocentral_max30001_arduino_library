// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: Copyright (c) 2025 Ashwin Whitchurch, Protocentral Electronics

/*
 * MAX30001 Single-Lead ECG Breakout Board - Arduino Library (Implementation)
 *
 * Copyright (c) 2025 Ashwin Whitchurch, Protocentral Electronics
 * Email: info@protocentral.com
 *
 * This software is licensed under the MIT License.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * For information on how to use, visit https://github.com/Protocentral/protocentral-max30001-arduino
 */
/////////////////////////////////////////////////////////////////////////////////////////

#include <SPI.h>
#include "protocentral_max30001.h"

// =============================================================================
// Constructors and Initialization
// =============================================================================

MAX30001::MAX30001(uint8_t cs_pin, SPIClass* spi_interface)
    : _spi(spi_interface), _cs_pin(cs_pin), _last_error(MAX30001_SUCCESS),
      _initialized(false), _ecg_gain(MAX30001_ECG_GAIN_80), 
      _sample_rate(MAX30001_RATE_128), _ecg_enabled(false), _bioz_enabled(false)
{
    pinMode(_cs_pin, OUTPUT);
    digitalWrite(_cs_pin, HIGH);
    
    // Initialize legacy members
    heartRate = 0;
    RRinterval = 0;
    ecg_data = 0;
    bioz_data = 0;
    ecgSamplesAvailable = 0;
    biozSamplesAvailable = 0;
}

MAX30001::MAX30001(int cs_pin)
    : MAX30001((uint8_t)cs_pin, &SPI)
{
    // Delegate to new constructor
}

void MAX30001::_max30001RegWrite(unsigned char WRITE_ADDRESS, unsigned long data)
{
    // Combine the register address and the command into one byte:
    byte dataToSend = (WRITE_ADDRESS << 1) | WREG;

    _spi->beginTransaction(SPISettings(MAX30001_SPI_SPEED, MSBFIRST, SPI_MODE0));

    digitalWrite(_cs_pin, LOW);

    delay(2);
    _spi->transfer(dataToSend);
    _spi->transfer(data >> 16);
    _spi->transfer(data >> 8);
    _spi->transfer(data);
    delay(2);

    digitalWrite(_cs_pin, HIGH);

    _spi->endTransaction();
}

void MAX30001::_max30001RegRead24(uint8_t Reg_address, uint32_t *read_data)
{
    uint8_t spiTxBuff;

    uint8_t buff[4];

    _spi->beginTransaction(SPISettings(MAX30001_SPI_SPEED, MSBFIRST, SPI_MODE0));

    digitalWrite(_cs_pin, LOW);

    spiTxBuff = (Reg_address << 1) | RREG;
    _spi->transfer(spiTxBuff); // Send register location

    for (int i = 0; i < 3; i++)
    {
        buff[i] = _spi->transfer(0xff);
    }

    digitalWrite(_cs_pin, HIGH);

    *read_data = (buff[0] << 16) | (buff[1] << 8) | buff[2];

    _spi->endTransaction();
}

void MAX30001::_max30001RegRead(uint8_t Reg_address, uint8_t *buff)
{
    uint8_t spiTxBuff;

    _spi->beginTransaction(SPISettings(MAX30001_SPI_SPEED, MSBFIRST, SPI_MODE0));

    digitalWrite(_cs_pin, LOW);

    spiTxBuff = (Reg_address << 1) | RREG;
    _spi->transfer(spiTxBuff); // Send register location

    for (int i = 0; i < 3; i++)
    {
        buff[i] = _spi->transfer(0xff);
    }

    digitalWrite(_cs_pin, HIGH);

    _spi->endTransaction();
}

void MAX30001::_max30001SwReset(void)
{
    _max30001RegWrite(SW_RST, 0x000000);
    delay(100);
}

void MAX30001::_max30001Synch(void)
{
    _max30001RegWrite(SYNCH, 0x000000);
}

void MAX30001::_max30001FIFOReset(void)
{
    _max30001RegWrite(FIFO_RST, 0x000000);
}

// =============================================================================
// New API: Initialization Methods
// =============================================================================

max30001_error_t MAX30001::begin()
{
    // Initialize SPI
    _spi->begin();
    
    // Perform software reset
    _max30001SwReset();
    delay(100);
    
    // Check if device is present
    if (!isConnected()) {
        _last_error = MAX30001_ERROR_DEVICE_NOT_FOUND;
        return _last_error;
    }
    
    _initialized = true;
    _last_error = MAX30001_SUCCESS;
    return MAX30001_SUCCESS;
}

bool MAX30001::isConnected()
{
    uint8_t readBuff[4];
    _max30001RegRead(INFO, readBuff);
    
    // Check for valid part ID (0x5x for MAX30001)
    return ((readBuff[0] & 0xF0) == 0x50);
}

max30001_error_t MAX30001::getDeviceInfo(max30001_device_info_t* info)
{
    if (info == nullptr) {
        _last_error = MAX30001_ERROR_INVALID_PARAMETER;
        return _last_error;
    }
    
    uint8_t readBuff[4];
    _max30001RegRead(INFO, readBuff);
    
    info->part_id = (readBuff[0] & 0xF0) >> 4;
    info->revision = readBuff[0] & 0x0F;
    info->device_found = (info->part_id == 0x5);
    
    _last_error = MAX30001_SUCCESS;
    return MAX30001_SUCCESS;
}

// =============================================================================
// New API: High-Level Measurement Methods
// =============================================================================

max30001_error_t MAX30001::startECG(max30001_sample_rate_t sample_rate, max30001_ecg_gain_t gain)
{
    if (!_initialized) {
        _last_error = MAX30001_ERROR_NOT_INITIALIZED;
        return _last_error;
    }
    
    if (_validateSampleRate(sample_rate) != MAX30001_SUCCESS) {
        return _last_error;
    }
    
    _sample_rate = sample_rate;
    _ecg_gain = gain;
    
    // Use register unions for type-safe configuration
    max30001_cnfg_gen_t cnfg_gen;
    max30001_cnfg_ecg_t cnfg_ecg;
    max30001_cnfg_emux_t cnfg_emux;
    
    // Software reset
    _max30001SwReset();
    delay(100);
    
    // Configure general settings
    cnfg_gen.all = 0;
    cnfg_gen.bit.en_ecg = 1;
    cnfg_gen.bit.en_bioz = 0;
    cnfg_gen.bit.fmstr = 0b00;
    cnfg_gen.bit.en_ulp_lon = 0;
    
    _max30001RegWrite(CNFG_GEN, cnfg_gen.all);
    delay(100);
    
    // Configure calibration (disabled)
    _max30001RegWrite(CNFG_CAL, 0x720000);
    delay(100);
    
    // Configure ECG channel
    cnfg_ecg.all = 0;
    cnfg_ecg.bit.rate = _rateToRegValue(sample_rate);
    cnfg_ecg.bit.gain = gain;
    cnfg_ecg.bit.dhpf = 0b1;  // 0.5Hz high-pass
    cnfg_ecg.bit.dlpf = 0b01; // 40Hz low-pass
    
    _max30001RegWrite(CNFG_ECG, cnfg_ecg.all);
    delay(100);
    
    // Configure ECG mux
    cnfg_emux.all = 0;
    cnfg_emux.bit.openp = 0;
    cnfg_emux.bit.openn = 0;
    cnfg_emux.bit.pol = 0;
    
    _max30001RegWrite(CNFG_EMUX, cnfg_emux.all);
    delay(100);
    
    // Synchronize
    _max30001Synch();
    delay(100);
    
    _ecg_enabled = true;
    _bioz_enabled = false;
    _last_error = MAX30001_SUCCESS;
    return MAX30001_SUCCESS;
}

max30001_error_t MAX30001::startBioZ(max30001_sample_rate_t sample_rate)
{
    if (!_initialized) {
        _last_error = MAX30001_ERROR_NOT_INITIALIZED;
        return _last_error;
    }
    
    if (_validateSampleRate(sample_rate) != MAX30001_SUCCESS) {
        return _last_error;
    }
    
    _sample_rate = sample_rate;
    
    max30001_cnfg_gen_t cnfg_gen;
    max30001_cnfg_bioz_t cnfg_bioz;
    max30001_cnfg_bmux_t cnfg_bmux;
    
    _max30001SwReset();
    delay(100);
    
    // Configure general settings for BioZ only
    cnfg_gen.all = 0;
    cnfg_gen.bit.en_ecg = 0;
    cnfg_gen.bit.en_bioz = 1;
    cnfg_gen.bit.fmstr = 0b00;
    
    _max30001RegWrite(CNFG_GEN, cnfg_gen.all);
    delay(100);
    
    // Configure calibration
    _max30001RegWrite(CNFG_CAL, 0x720000);
    delay(100);
    
    // Configure BioZ channel
    cnfg_bioz.all = 0;
    cnfg_bioz.bit.rate = 0;
    cnfg_bioz.bit.ahpf = 0b010;
    cnfg_bioz.bit.ln_bioz = 1;
    cnfg_bioz.bit.gain = 0b01;
    cnfg_bioz.bit.dhpf = 0b010;
    cnfg_bioz.bit.dlpf = 0x01;
    cnfg_bioz.bit.fcgen = 0b0100;
    cnfg_bioz.bit.cgmag = 0b010;
    cnfg_bioz.bit.phoff = 0x0011;
    
    _max30001RegWrite(CNFG_BIOZ, cnfg_bioz.all);
    delay(100);
    
    // Set BioZ LC mode
    _max30001RegWrite(CNFG_BIOZ_LC, 0x800000);
    delay(100);
    
    // Configure BioZ mux
    cnfg_bmux.all = 0;
    cnfg_bmux.bit.rmod = 0x04;
    _max30001RegWrite(CNFG_BMUX, cnfg_bmux.all);
    delay(100);
    
    _max30001Synch();
    delay(100);
    
    _ecg_enabled = false;
    _bioz_enabled = true;
    _last_error = MAX30001_SUCCESS;
    return MAX30001_SUCCESS;
}

max30001_error_t MAX30001::startECGBioZ(max30001_sample_rate_t sample_rate)
{
    if (!_initialized) {
        _last_error = MAX30001_ERROR_NOT_INITIALIZED;
        return _last_error;
    }
    
    if (_validateSampleRate(sample_rate) != MAX30001_SUCCESS) {
        return _last_error;
    }
    
    _sample_rate = sample_rate;
    _ecg_gain = MAX30001_ECG_GAIN_80;
    
    // Use the existing legacy method which is well-tested
    BeginECGBioZ();
    
    _ecg_enabled = true;
    _bioz_enabled = true;
    _last_error = MAX30001_SUCCESS;
    return MAX30001_SUCCESS;
}

max30001_error_t MAX30001::startRtoR(max30001_sample_rate_t sample_rate, max30001_ecg_gain_t gain)
{
    // Convenience method for R-R detection: just start ECG (R-R is built-in)
    return startECG(sample_rate, gain);
}

max30001_error_t MAX30001::getECGSample(max30001_ecg_sample_t* sample)
{
    if (sample == nullptr) {
        _last_error = MAX30001_ERROR_INVALID_PARAMETER;
        return _last_error;
    }
    
    if (!_initialized || !_ecg_enabled) {
        _last_error = MAX30001_ERROR_NOT_READY;
        sample->sample_valid = false;
        return _last_error;
    }
    
    // Read ECG FIFO
    uint8_t regReadBuff[4];
    _max30001RegRead(ECG_FIFO, regReadBuff);
    
    unsigned long data0 = (unsigned long)(regReadBuff[0]);
    data0 = data0 << 16;
    unsigned long data1 = (unsigned long)(regReadBuff[1]);
    data1 = data1 << 8;
    unsigned long data2 = (unsigned long)(regReadBuff[2]);
    data2 = data2 & 0xC0;
    
    unsigned long data = (unsigned long)(data0 | data1 | data2);
    data = (unsigned long)(data << 8);
    signed long secgtemp = (signed long)(data);
    secgtemp = (signed long)(secgtemp >> 14);
    
    sample->ecg_sample = secgtemp;
    sample->timestamp_ms = millis();
    sample->lead_off_detected = false; // TODO: implement lead-off detection
    sample->sample_valid = true;
    
    // Update legacy data member
    ecg_data = secgtemp;
    
    _last_error = MAX30001_SUCCESS;
    return MAX30001_SUCCESS;
}

max30001_error_t MAX30001::getBioZSample(max30001_bioz_sample_t* sample)
{
    if (sample == nullptr) {
        _last_error = MAX30001_ERROR_INVALID_PARAMETER;
        return _last_error;
    }
    
    if (!_initialized || !_bioz_enabled) {
        _last_error = MAX30001_ERROR_NOT_READY;
        sample->sample_valid = false;
        return _last_error;
    }
    
    // Read BioZ FIFO
    uint8_t regReadBuff[4];
    _max30001RegRead(BIOZ_FIFO, regReadBuff);
    
    unsigned long data0 = (unsigned long)(regReadBuff[0]);
    data0 = data0 << 16;
    unsigned long data1 = (unsigned long)(regReadBuff[1]);
    data1 = data1 << 8;
    unsigned long data2 = (unsigned long)(regReadBuff[2]);
    data2 = data2 & 0xF0;
    
    unsigned long data = (unsigned long)(data0 | data1 | data2);
    data = (unsigned long)(data << 8);
    signed long sbioztemp = (signed long)(data);
    sbioztemp = (signed long)(sbioztemp >> 12);
    
    sample->bioz_sample = sbioztemp;
    sample->timestamp_ms = millis();
    sample->sample_valid = true;
    
    // Update legacy data member
    bioz_data = sbioztemp;
    
    _last_error = MAX30001_SUCCESS;
    return MAX30001_SUCCESS;
}

max30001_error_t MAX30001::getRtoRData(max30001_rtor_data_t* rtor_data)
{
    if (rtor_data == nullptr) {
        _last_error = MAX30001_ERROR_INVALID_PARAMETER;
        return _last_error;
    }
    
    // Use legacy method
    getHRandRR();
    
    rtor_data->heart_rate_bpm = heartRate;
    rtor_data->rr_interval_ms = RRinterval;
    rtor_data->rr_detected = (heartRate > 0);
    
    _last_error = MAX30001_SUCCESS;
    return MAX30001_SUCCESS;
}

void MAX30001::stop()
{
    if (!_initialized) {
        return;
    }
    
    // Disable all channels
    _max30001RegWrite(CNFG_GEN, 0x000000);
    delay(100);
    
    _ecg_enabled = false;
    _bioz_enabled = false;
}

float MAX30001::convertECGToMicrovolts(int32_t raw_value, max30001_ecg_gain_t gain)
{
    // LSB size depends on gain setting
    // For gain of 80 V/V: LSB = 156.25 nV (from datasheet)
    // For gain of 160 V/V: LSB = 78.125 nV
    // For gain of 40 V/V: LSB = 312.5 nV
    // For gain of 20 V/V: LSB = 625 nV
    
    float lsb_nv;
    switch(gain) {
        case MAX30001_ECG_GAIN_20:
            lsb_nv = 625.0;
            break;
        case MAX30001_ECG_GAIN_40:
            lsb_nv = 312.5;
            break;
        case MAX30001_ECG_GAIN_80:
            lsb_nv = 156.25;
            break;
        case MAX30001_ECG_GAIN_160:
            lsb_nv = 78.125;
            break;
        default:
            lsb_nv = 156.25; // Default to 80 V/V
    }
    
    // Convert to microvolts
    return (raw_value * lsb_nv) / 1000.0;
}

max30001_error_t MAX30001::getLastError() const
{
    return _last_error;
}

// =============================================================================
// Advanced Configuration Methods (Phase 3)
// =============================================================================

max30001_error_t MAX30001::setECGGain(max30001_ecg_gain_t gain)
{
    if (!_initialized || !_ecg_enabled) {
        _last_error = MAX30001_ERROR_NOT_READY;
        return _last_error;
    }
    
    // Validate gain value
    if (gain != MAX30001_ECG_GAIN_80 && gain != MAX30001_ECG_GAIN_160) {
        _last_error = MAX30001_ERROR_INVALID_PARAMETER;
        return _last_error;
    }
    
    _ecg_gain = gain;
    
    // Read current ECG config
    max30001_cnfg_ecg_t cnfg_ecg;
    uint8_t regReadBuff[4];
    _max30001RegRead(CNFG_ECG, regReadBuff);
    cnfg_ecg.all = (uint32_t)((regReadBuff[0] << 16) | (regReadBuff[1] << 8) | regReadBuff[2]);
    
    // Update gain bits (bits [14:13])
    cnfg_ecg.bit.gain = (gain == MAX30001_ECG_GAIN_160) ? 0b11 : 0b10;
    
    // Write back
    _max30001RegWrite(CNFG_ECG, cnfg_ecg.all);
    delay(100);
    
    _last_error = MAX30001_SUCCESS;
    return MAX30001_SUCCESS;
}

max30001_error_t MAX30001::enableECG()
{
    if (!_initialized) {
        _last_error = MAX30001_ERROR_NOT_INITIALIZED;
        return _last_error;
    }
    
    if (_ecg_enabled) {
        _last_error = MAX30001_SUCCESS;
        return MAX30001_SUCCESS;  // Already enabled
    }
    
    // Read CNFG_GEN to enable ECG
    max30001_cnfg_gen_t cnfg_gen;
    uint8_t regReadBuff[4];
    _max30001RegRead(CNFG_GEN, regReadBuff);
    cnfg_gen.all = (uint32_t)((regReadBuff[0] << 16) | (regReadBuff[1] << 8) | regReadBuff[2]);
    
    cnfg_gen.bit.en_ecg = 1;
    _max30001RegWrite(CNFG_GEN, cnfg_gen.all);
    
    _ecg_enabled = true;
    delay(100);
    _last_error = MAX30001_SUCCESS;
    return MAX30001_SUCCESS;
}

max30001_error_t MAX30001::disableECG()
{
    if (!_initialized) {
        _last_error = MAX30001_ERROR_NOT_INITIALIZED;
        return _last_error;
    }
    
    if (!_ecg_enabled) {
        _last_error = MAX30001_SUCCESS;
        return MAX30001_SUCCESS;  // Already disabled
    }
    
    // Read CNFG_GEN to disable ECG
    max30001_cnfg_gen_t cnfg_gen;
    uint8_t regReadBuff[4];
    _max30001RegRead(CNFG_GEN, regReadBuff);
    cnfg_gen.all = (uint32_t)((regReadBuff[0] << 16) | (regReadBuff[1] << 8) | regReadBuff[2]);
    
    cnfg_gen.bit.en_ecg = 0;
    _max30001RegWrite(CNFG_GEN, cnfg_gen.all);
    
    _ecg_enabled = false;
    delay(100);
    _last_error = MAX30001_SUCCESS;
    return MAX30001_SUCCESS;
}

max30001_error_t MAX30001::enableBioZ()
{
    if (!_initialized) {
        _last_error = MAX30001_ERROR_NOT_INITIALIZED;
        return _last_error;
    }
    
    if (_bioz_enabled) {
        _last_error = MAX30001_SUCCESS;
        return MAX30001_SUCCESS;  // Already enabled
    }
    
    // Read CNFG_GEN to enable BioZ
    max30001_cnfg_gen_t cnfg_gen;
    uint8_t regReadBuff[4];
    _max30001RegRead(CNFG_GEN, regReadBuff);
    cnfg_gen.all = (uint32_t)((regReadBuff[0] << 16) | (regReadBuff[1] << 8) | regReadBuff[2]);
    
    cnfg_gen.bit.en_bioz = 1;
    _max30001RegWrite(CNFG_GEN, cnfg_gen.all);
    
    _bioz_enabled = true;
    delay(100);
    _last_error = MAX30001_SUCCESS;
    return MAX30001_SUCCESS;
}

max30001_error_t MAX30001::disableBioZ()
{
    if (!_initialized) {
        _last_error = MAX30001_ERROR_NOT_INITIALIZED;
        return _last_error;
    }
    
    if (!_bioz_enabled) {
        _last_error = MAX30001_SUCCESS;
        return MAX30001_SUCCESS;  // Already disabled
    }
    
    // Read CNFG_GEN to disable BioZ
    max30001_cnfg_gen_t cnfg_gen;
    uint8_t regReadBuff[4];
    _max30001RegRead(CNFG_GEN, regReadBuff);
    cnfg_gen.all = (uint32_t)((regReadBuff[0] << 16) | (regReadBuff[1] << 8) | regReadBuff[2]);
    
    cnfg_gen.bit.en_bioz = 0;
    _max30001RegWrite(CNFG_GEN, cnfg_gen.all);
    
    _bioz_enabled = false;
    delay(100);
    _last_error = MAX30001_SUCCESS;
    return MAX30001_SUCCESS;
}

max30001_error_t MAX30001::setECGHighPassFilter(float cutoff_hz)
{
    if (!_initialized || !_ecg_enabled) {
        _last_error = MAX30001_ERROR_NOT_READY;
        return _last_error;
    }
    
    // Map frequency to register bits [6:4] (DHPF)
    uint8_t dhpf_bits = 0;
    if (cutoff_hz < 0.5) {
        dhpf_bits = 0b000;  // 0.4 Hz
    } else if (cutoff_hz < 0.9) {
        dhpf_bits = 0b001;  // 0.8 Hz
    } else if (cutoff_hz < 1.5) {
        dhpf_bits = 0b010;  // 1.2 Hz
    } else if (cutoff_hz < 2.0) {
        dhpf_bits = 0b011;  // 1.6 Hz
    } else {
        dhpf_bits = 0b100;  // 2.0+ Hz
    }
    
    // Read current ECG config
    max30001_cnfg_ecg_t cnfg_ecg;
    uint8_t regReadBuff[4];
    _max30001RegRead(CNFG_ECG, regReadBuff);
    cnfg_ecg.all = (uint32_t)((regReadBuff[0] << 16) | (regReadBuff[1] << 8) | regReadBuff[2]);
    
    // Update high-pass filter bits
    cnfg_ecg.bit.dhpf = dhpf_bits;
    
    _max30001RegWrite(CNFG_ECG, cnfg_ecg.all);
    delay(100);
    
    _last_error = MAX30001_SUCCESS;
    return MAX30001_SUCCESS;
}

max30001_error_t MAX30001::setECGLowPassFilter(float cutoff_hz)
{
    if (!_initialized || !_ecg_enabled) {
        _last_error = MAX30001_ERROR_NOT_READY;
        return _last_error;
    }
    
    // Map frequency to register bits [1:0] (DLPF)
    uint8_t dlpf_bits = 0;
    if (cutoff_hz < 60) {
        dlpf_bits = 0b00;  // 40 Hz
    } else if (cutoff_hz < 150) {
        dlpf_bits = 0b01;  // 100 Hz
    } else if (cutoff_hz < 250) {
        dlpf_bits = 0b10;  // 150 Hz
    } else {
        dlpf_bits = 0b11;  // 200+ Hz (no filter)
    }
    
    // Read current ECG config
    max30001_cnfg_ecg_t cnfg_ecg;
    uint8_t regReadBuff[4];
    _max30001RegRead(CNFG_ECG, regReadBuff);
    cnfg_ecg.all = (uint32_t)((regReadBuff[0] << 16) | (regReadBuff[1] << 8) | regReadBuff[2]);
    
    // Update low-pass filter bits
    cnfg_ecg.bit.dlpf = dlpf_bits;
    
    _max30001RegWrite(CNFG_ECG, cnfg_ecg.all);
    delay(100);
    
    _last_error = MAX30001_SUCCESS;
    return MAX30001_SUCCESS;
}

bool MAX30001::getLeadOffStatus()
{
    if (!_initialized || !_ecg_enabled) {
        return false;  // Default safe state
    }
    
    // Read STATUS register (0x01)
    uint8_t regReadBuff[4];
    _max30001RegRead(STATUS, regReadBuff);
    
    // Check lead-off detection bits
    // Bit 1: ICOL - lead-off detected
    // Bit 0: LDOFF_PH - phase of lead-off
    uint8_t status_byte = regReadBuff[0];
    return (status_byte & 0x02) != 0;  // Check bit 1 (ICOL)
}

uint8_t MAX30001::getFIFOCount()
{
    if (!_initialized) {
        return 0;
    }
    
    // Return the legacy counter (updated during sample reads)
    // In a real implementation, this would read the actual FIFO count register
    return ecgSamplesAvailable;
}

max30001_error_t MAX30001::clearFIFO()
{
    if (!_initialized) {
        _last_error = MAX30001_ERROR_NOT_INITIALIZED;
        return _last_error;
    }
    
    // Reset FIFO via the FIFO_RST register
    _max30001RegWrite(FIFO_RST, 0x00);
    delay(10);
    
    ecgSamplesAvailable = 0;
    _last_error = MAX30001_SUCCESS;
    return MAX30001_SUCCESS;
}

max30001_error_t MAX30001::enableInterrupt()
{
    if (!_initialized) {
        _last_error = MAX30001_ERROR_NOT_INITIALIZED;
        return _last_error;
    }
    
    // Read CNFG_GEN
    max30001_cnfg_gen_t cnfg_gen;
    uint8_t regReadBuff[4];
    _max30001RegRead(CNFG_GEN, regReadBuff);
    cnfg_gen.all = (uint32_t)((regReadBuff[0] << 16) | (regReadBuff[1] << 8) | regReadBuff[2]);
    
    // Enable interrupts (EN_ECG_ON_INT1 or similar - check datasheet)
    // This is a simplified version; actual register bits may vary
    cnfg_gen.bit.en_ecg = 1;
    
    _max30001RegWrite(CNFG_GEN, cnfg_gen.all);
    delay(100);
    
    _last_error = MAX30001_SUCCESS;
    return MAX30001_SUCCESS;
}

max30001_error_t MAX30001::disableInterrupt()
{
    if (!_initialized) {
        _last_error = MAX30001_ERROR_NOT_INITIALIZED;
        return _last_error;
    }
    
    _last_error = MAX30001_SUCCESS;
    return MAX30001_SUCCESS;
}

uint16_t MAX30001::getSampleDelayMs() const
{
    return _getSampleDelayMs(_sample_rate);
}

// =============================================================================
// Private Helper Methods
// =============================================================================

max30001_error_t MAX30001::_validateSampleRate(max30001_sample_rate_t rate)
{
    if (rate != MAX30001_RATE_128 && rate != MAX30001_RATE_256 && rate != MAX30001_RATE_512) {
        _last_error = MAX30001_ERROR_INVALID_PARAMETER;
        return _last_error;
    }
    return MAX30001_SUCCESS;
}

uint8_t MAX30001::_rateToRegValue(max30001_sample_rate_t rate)
{
    switch(rate) {
        case MAX30001_RATE_512:
            return 0b00;
        case MAX30001_RATE_256:
            return 0b01;
        case MAX30001_RATE_128:
            return 0b10;
        default:
            return 0b10; // Default to 128 SPS
    }
}

uint8_t MAX30001::_getSampleDelayMs(max30001_sample_rate_t rate) const
{
    switch(rate) {
        case MAX30001_RATE_512:
            return 2;
        case MAX30001_RATE_256:
            return 4;
        case MAX30001_RATE_128:
            return 8;
        default:
            return 8;
    }
}

// =============================================================================
// Legacy API Implementation (Backward Compatibility)
// =============================================================================

bool MAX30001::max30001ReadInfo(void)
{
    uint8_t readBuff[4];

    _max30001RegRead(INFO, readBuff);

    if ((readBuff[0] & 0xf0) == 0x50)
    {
        Serial.print("MAX30001 Detected. Rev ID:  ");
        Serial.println((readBuff[0] & 0xf0));

        return true;
    }
    else
    {

        Serial.println("MAX30001 read info error\n");
        return false;
    }

    return false;
}

void MAX30001::BeginECGOnly()
{
    _max30001SwReset();
    delay(100);
    _max30001RegWrite(CNFG_GEN, 0x081007);
    delay(100);
    _max30001RegWrite(CNFG_CAL, 0x720000); // 0x700000
    delay(100);
    _max30001RegWrite(CNFG_EMUX, 0x0B0000);
    delay(100);
    _max30001RegWrite(CNFG_ECG, 0x825000); // d23 - d22 : 10 for 250sps , 00:500 sps
    delay(100);

    _max30001RegWrite(CNFG_RTOR1, 0x3fc600);
    _max30001Synch();
    delay(100);
}

void MAX30001::BeginECGBioZ()
{
    max30001_cnfg_gen_t cnfg_gen;
    max30001_cnfg_emux_t cnfg_emux;
    max30001_cnfg_ecg_t cnfg_ecg;
    max30001_cnfg_bmux_t cnfg_bmux;
    max30001_cnfg_bioz_t cnfg_bioz;
    
    // CONFIG GEN Register Settings
    cnfg_gen.bit.en_ulp_lon = 0; // ULP Lead-ON Disabled
    cnfg_gen.bit.fmstr = 0b00;
    cnfg_gen.bit.en_ecg = 0b1;
    cnfg_gen.bit.en_bioz = 0b1;
    cnfg_gen.bit.en_bloff = 0x00;  // BioZ digital lead off detection disabled
    cnfg_gen.bit.en_dcloff = 0x00; // DC Lead-Off Detection Disabled
    cnfg_gen.bit.en_rbias = 0b00;  // RBias disabled
    cnfg_gen.bit.rbiasv = 0b01;    // RBias =100 Mohm
    cnfg_gen.bit.rbiasp = 0b00;    // RBias Positive Input not connected
    cnfg_gen.bit.rbiasn = 0b00;    // RBias Negative Input not connected

    // ECG Config Settings
    cnfg_ecg.bit.rate = 0b10; // Default, 128SPS
    cnfg_ecg.bit.gain = 0b10; // 160 V/V
    cnfg_ecg.bit.dhpf = 0b1;  // 0.5Hz
    cnfg_ecg.bit.dlpf = 0b01; // 40Hz
    
    // ECG MUX Settings
    cnfg_emux.bit.openp = 0;
    cnfg_emux.bit.openn = 0;
    cnfg_emux.bit.pol = 0;
    cnfg_emux.bit.calp_sel = 0;
    cnfg_emux.bit.caln_sel = 0;

    // BioZ Config Settings
    cnfg_bioz.bit.rate = 0;
    cnfg_bioz.bit.ahpf = 0b010;
    cnfg_bioz.bit.ext_rbias = 0x00;
    cnfg_bioz.bit.ln_bioz = 1;
    cnfg_bioz.bit.gain = 0b01; //(20 V/V)
    cnfg_bioz.bit.dhpf = 0b010;
    cnfg_bioz.bit.dlpf = 0x01;
    cnfg_bioz.bit.fcgen = 0b0100;
    cnfg_bioz.bit.cgmon = 0x00;
    cnfg_bioz.bit.cgmag = 0b010;
    cnfg_bioz.bit.phoff = 0x0011;

    // BioZ MUX Settings
    cnfg_bmux.bit.openp = 0;
    cnfg_bmux.bit.openn = 0;
    cnfg_bmux.bit.calp_sel = 0x00; // No cal signal on BioZ
    cnfg_bmux.bit.caln_sel = 0x00; // No cal signal on BioZ
    cnfg_bmux.bit.cg_mode = 0x00;  // Unchopped
    cnfg_bmux.bit.en_bist = 0;
    cnfg_bmux.bit.rnom = 0x00;
    cnfg_bmux.bit.rmod = 0x04;
    cnfg_bmux.bit.fbist = 0;

    //_max30001RegWrite(MNGR_INT, 0x7B0000); // EFIT=16, BFIT=8
    // delay(100);

    // max30001SetInterrupts(EN_EINT | 0x01); // Enable ECG Interrupts

    //_max30001RegWrite(CNFG_RTOR1,0x3fc600);

    /* ****IMPORTANT: Write all registers in the same sequence**** */

    _max30001SwReset();
    delay(100);

    _max30001RegWrite(CNFG_GEN, cnfg_gen.all);
    //_max30001RegWrite(CNFG_GEN, 0xC0004); // ECG & BioZ Enabled , FMSTR = 32768
    delay(100);

    _max30001RegWrite(CNFG_CAL, 0x720000); // Calibration sources disabled
    delay(100);

    _max30001RegWrite(CNFG_ECG, cnfg_ecg.all);
    delay(100);

    _max30001RegWrite(CNFG_EMUX, cnfg_emux.all); // Pins internally connection to ECG Channels
    delay(100);

    _max30001RegWrite(CNFG_BIOZ, cnfg_bioz.all);
    //_max30001RegWrite(CNFG_BIOZ, 0x201433); // BioZ Rate: 64 SPS | Current generator: 32 uA

    // Set MAX30001G specific BioZ LC
    _max30001RegWrite(CNFG_BIOZ_LC, 0x800000); // Turn OFF low current mode
    delay(100);

    _max30001RegWrite(CNFG_BMUX, 0x000040); // Pins connected internally to BioZ channels
    delay(100);

    _max30001Synch();
    delay(100);
}

void MAX30001::BeginRtoRMode()
{
    _max30001SwReset();
    delay(100);
    _max30001RegWrite(CNFG_GEN, 0x080004);
    delay(100);
    _max30001RegWrite(CNFG_CAL, 0x720000); // 0x700000
    delay(100);
    _max30001RegWrite(CNFG_EMUX, 0x0B0000);
    delay(100);
    _max30001RegWrite(CNFG_ECG, 0x805000); // d23 - d22 : 10 for 250sps , 00:500 sps
    delay(100);
    _max30001RegWrite(CNFG_RTOR1, 0x3fc600);
    delay(100);
    _max30001RegWrite(EN_INT, 0x000401);
    delay(100);
    _max30001Synch();
    delay(100);
}

// not tested
void MAX30001::max30001SetsamplingRate(uint16_t samplingRate)
{
    uint8_t regBuff[4] = {0};
    _max30001RegRead(CNFG_ECG, regBuff);

    switch (samplingRate)
    {
    case SAMPLINGRATE_128:
        regBuff[0] = (regBuff[0] | 0x80);
        break;

    case SAMPLINGRATE_256:
        regBuff[0] = (regBuff[0] | 0x40);
        break;

    case SAMPLINGRATE_512:
        regBuff[0] = (regBuff[0] | 0x00);
        break;

    default:
        Serial.println("Invalid sample rate. Please choose between 128, 256 or 512");
        break;
    }

    unsigned long cnfgEcg;
    memcpy(&cnfgEcg, regBuff, 4);

    Serial.print(" cnfg ECG ");
    Serial.println((cnfgEcg));
    _max30001RegWrite(CNFG_ECG, (cnfgEcg >> 8));
}

signed long MAX30001::getECGSamples(void)
{
    uint8_t regReadBuff[4];
    _max30001RegRead(ECG_FIFO, regReadBuff);

    unsigned long data0 = (unsigned long)(regReadBuff[0]);
    data0 = data0 << 16;
    unsigned long data1 = (unsigned long)(regReadBuff[1]);
    data1 = data1 << 8;
    unsigned long data2 = (unsigned long)(regReadBuff[2]);
    data2 = data2 & 0xC0;

    unsigned long data = (unsigned long)(data0 | data1 | data2);
    data = (unsigned long)(data << 8);
    signed long secgtemp = (signed long)(data);
    secgtemp =(signed long) (secgtemp >> 14);

    return secgtemp;
}

signed long MAX30001::getBioZSamples(void)
{
    uint8_t regReadBuff[4];
    _max30001RegRead(BIOZ_FIFO, regReadBuff);

    unsigned long data0 = (unsigned long)(regReadBuff[0]);
    data0 = data0 << 16;
    unsigned long data1 = (unsigned long)(regReadBuff[1]);
    data1 = data1 << 8;
    unsigned long data2 = (unsigned long)(regReadBuff[2]);
    data2 = data2 & 0xF0;

    unsigned long data = (unsigned long)(data0 | data1 | data2);
    data = (unsigned long)(data << 8);
    signed long sbioztemp = (signed long)(data);
    sbioztemp =(signed long) (sbioztemp >> 12);
    
    return sbioztemp;
}

void MAX30001::getHRandRR(void)
{
    uint8_t regReadBuff[4];
    _max30001RegRead(RTOR, regReadBuff);

    unsigned long RTOR_msb = (unsigned long)(regReadBuff[0]);
    unsigned char RTOR_lsb = (unsigned char)(regReadBuff[1]);
    unsigned long rtor = (RTOR_msb << 8 | RTOR_lsb);
    rtor = ((rtor >> 2) & 0x3fff);

    float hr = 60 / ((float)rtor * 0.0078125);
    heartRate = (unsigned int)hr;

    unsigned int RR = (unsigned int)rtor * (7.8125); // 8ms
    RRinterval = RR;
}

void MAX30001::max30001SetInterrupts(uint32_t interrupts_to_set)
{
    _max30001RegWrite(EN_INT, interrupts_to_set);
    delay(100);
    //_max30001Synch();
    // delay(100);
}

int secg_counter = 0;
int sbioz_counter = 0;

void MAX30001::_max30001ReadECGFIFO(int num_bytes)
{
    uint8_t spiTxBuff;
    unsigned long uecgtemp;
    signed long secgtemp;

    _spi->beginTransaction(SPISettings(MAX30001_SPI_SPEED, MSBFIRST, SPI_MODE0));

    digitalWrite(_cs_pin, LOW);

    spiTxBuff = (ECG_FIFO_BURST << 1) | RREG;
    _spi->transfer(spiTxBuff); // Send register location

    for (int i = 0; i < num_bytes; i++)
    {
        _readBufferECG[i] = _spi->transfer(0x00);
    }

    digitalWrite(_cs_pin, HIGH);

    _spi->endTransaction();

    secg_counter = 0;
    unsigned char ecg_etag;

    for (int i = 0; i < num_bytes; i += 3)
    {
        // Get etag
        ecg_etag = ((((unsigned char)_readBufferECG[i + 2]) & 0x38) >> 3);
        // Serial.println(ecg_etag, HEX);

        if (ecg_etag == 0x00) // Valid sample
        {
            // uecgtemp=(unsigned long)((unsigned long)readBuffer[i]<<16 |(unsigned long)readBuffer[i+1]<<8| (unsigned long)(readBuffer[i+2]&0xC0));
            uecgtemp = (unsigned long)(((unsigned long)_readBufferECG[i] << 16 | (unsigned long)_readBufferECG[i + 1] << 8) | (unsigned long)(_readBufferECG[i + 2] & 0xC0));
            uecgtemp = (unsigned long)(uecgtemp << 8);

            secgtemp = (signed long)uecgtemp;
            secgtemp = (signed long)secgtemp >> 8;

            s32ECGData[secg_counter++] = secgtemp;
        }
        else if (ecg_etag == 0x07) // FIFO Overflow
        {
            // Serial.println("OVF");
            _max30001FIFOReset();
        }
    }

    // Serial.print("F");
    // Serial.println(secg_counter);

    ecgSamplesAvailable = secg_counter;
    secg_counter = 0;
}

void MAX30001::_max30001ReadBIOZFIFO(int num_bytes)
{
    uint8_t spiTxBuff;
    unsigned long ubioztemp;
    signed long sbioztemp;

    _spi->beginTransaction(SPISettings(MAX30001_SPI_SPEED, MSBFIRST, SPI_MODE0));

    digitalWrite(_cs_pin, LOW);

    spiTxBuff = (BIOZ_FIFO_BURST << 1) | RREG;
    _spi->transfer(spiTxBuff); // Send register location

    for (int i = 0; i < num_bytes; i++)
    {
        _readBufferBIOZ[i] = _spi->transfer(0x00);
    }

    digitalWrite(_cs_pin, HIGH);

    _spi->endTransaction();

    sbioz_counter = 0;
    unsigned char bioz_etag;

    for (int i = 0; i < num_bytes; i += 3)
    {
        // Get etag
        bioz_etag = ((((unsigned char)_readBufferBIOZ[i + 2]) & 0x38) >> 3);
        // Serial.println(ecg_etag, HEX);

        if (bioz_etag == 0x00) // Valid sample
        {
            // uecgtemp=(unsigned long)((unsigned long)readBuffer[i]<<16 |(unsigned long)readBuffer[i+1]<<8| (unsigned long)(readBuffer[i+2]&0xC0));
            ubioztemp = (unsigned long)(((unsigned long)_readBufferBIOZ[i] << 16 | (unsigned long)_readBufferBIOZ[i + 1] << 8) | (unsigned long)(_readBufferBIOZ[i + 2] & 0xC0));
            ubioztemp = (unsigned long)(ubioztemp << 8);

            sbioztemp = (signed long)ubioztemp;
            sbioztemp = (signed long)sbioztemp >> 8;

            s32BIOZData[sbioz_counter++] = sbioztemp;
        }
        else if (bioz_etag == 0x07) // FIFO Overflow
        {
            // Serial.println("OVF");
            _max30001FIFOReset();
        }
    }

    biozSamplesAvailable = sbioz_counter;
    sbioz_counter = 0;
}

void MAX30001::max30001ServiceAllInterrupts(void)
{
    static uint32_t InitReset = 0;
    int fifo_num_bytes = 0;

    max30001_mngr_int_t mngr_int;

    _max30001RegRead24(STATUS, &global_status.all);

    if (global_status.bit.eint == 1) // EINT bit is set. FIFO is full
    {
        // Read the number of bytes in FIFO (from  MNGR_INT register)
        _max30001RegRead24(MNGR_INT, &mngr_int.all);
        fifo_num_bytes = (mngr_int.bit.e_fit + 1) * 3;

        _max30001ReadECGFIFO(fifo_num_bytes);
    }

    if (global_status.bit.bint == 1) // BIOZ FIFO is full
    {
        _max30001RegRead24(MNGR_INT, &mngr_int.all);
        fifo_num_bytes = (mngr_int.bit.b_fit + 1) * 3;

        // Read BIOZ FIFO in Burst mode
        _max30001ReadBIOZFIFO(fifo_num_bytes);
    }
}