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

// FIFO data tags (ETAG = ECG D[5:3], BTAG = BioZ D[2:0])
#define MAX30001_TAG_VALID 0x00
#define MAX30001_TAG_FAST_OR_RANGE 0x01  // ECG: fast-recovery sample, BioZ: over/under range
#define MAX30001_TAG_VALID_EOF 0x02
#define MAX30001_TAG_FAST_OR_RANGE_EOF 0x03
#define MAX30001_TAG_EMPTY 0x06
#define MAX30001_TAG_OVERFLOW 0x07

// R-to-R timing resolution at FMSTR = 32768 Hz (tRTOR = 256 / fMSTR)
#define MAX30001_RTOR_LSB_MS 7.8125f

// =============================================================================
// Sample decoding helpers
// =============================================================================

// ECG FIFO word: D[23:6] = 18-bit two's complement sample, D[5:3] = ETAG
static inline int32_t decodeECG(uint32_t word)
{
    return ((int32_t)(word << 8)) >> 14;
}

// BioZ FIFO word: D[23:4] = 20-bit two's complement sample, D[2:0] = BTAG
static inline int32_t decodeBioZ(uint32_t word)
{
    return ((int32_t)((word & 0xFFFFF0UL) << 8)) >> 12;
}

static inline uint32_t bytesToWord(const volatile uint8_t *b)
{
    return ((uint32_t)b[0] << 16) | ((uint32_t)b[1] << 8) | (uint32_t)b[2];
}

// =============================================================================
// Constructors and Initialization
// =============================================================================

MAX30001::MAX30001(uint8_t cs_pin, SPIClass* spi_interface)
    : _spi(spi_interface), _cs_pin(cs_pin), _last_error(MAX30001_SUCCESS),
      _initialized(false), _ecg_gain(MAX30001_ECG_GAIN_80),
      _sample_rate(MAX30001_RATE_128), _ecg_enabled(false), _bioz_enabled(false)
{
    // Initialize legacy members
    heartRate = 0;
    RRinterval = 0;
    ecg_data = 0;
    bioz_data = 0;
    ecgSamplesAvailable = 0;
    biozSamplesAvailable = 0;
    global_status.all = 0;
}

MAX30001::MAX30001(int cs_pin)
    : MAX30001((uint8_t)cs_pin, &SPI)
{
    // Delegate to new constructor
}

void MAX30001::_max30001RegWrite(unsigned char WRITE_ADDRESS, unsigned long data)
{
    // Combine the register address and the command into one byte:
    byte dataToSend = (WRITE_ADDRESS << 1) | MAX30001_SPI_WRITE;

    _spi->beginTransaction(SPISettings(MAX30001_SPI_SPEED, MSBFIRST, SPI_MODE0));

    digitalWrite(_cs_pin, LOW);

    _spi->transfer(dataToSend);
    _spi->transfer(data >> 16);
    _spi->transfer(data >> 8);
    _spi->transfer(data);

    digitalWrite(_cs_pin, HIGH);

    _spi->endTransaction();
}

void MAX30001::_max30001RegRead24(uint8_t Reg_address, uint32_t *read_data)
{
    uint8_t buff[4];

    _max30001RegRead(Reg_address, buff);

    *read_data = bytesToWord(buff);
}

void MAX30001::_max30001RegRead(uint8_t Reg_address, uint8_t *buff)
{
    uint8_t spiTxBuff;

    _spi->beginTransaction(SPISettings(MAX30001_SPI_SPEED, MSBFIRST, SPI_MODE0));

    digitalWrite(_cs_pin, LOW);

    spiTxBuff = (Reg_address << 1) | MAX30001_SPI_READ;
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
    _max30001RegWrite(MAX30001_REG_SW_RST, 0x000000);
    delay(100);
}

void MAX30001::_max30001Synch(void)
{
    _max30001RegWrite(MAX30001_REG_SYNCH, 0x000000);
}

void MAX30001::_max30001FIFOReset(void)
{
    _max30001RegWrite(MAX30001_REG_FIFO_RST, 0x000000);
}

// =============================================================================
// New API: Initialization Methods
// =============================================================================

max30001_error_t MAX30001::begin()
{
    pinMode(_cs_pin, OUTPUT);
    digitalWrite(_cs_pin, HIGH);

    // Initialize SPI
    _spi->begin();

    // Perform software reset
    _max30001SwReset();

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
    _max30001RegRead(MAX30001_REG_INFO, readBuff);

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
    _max30001RegRead(MAX30001_REG_INFO, readBuff);

    info->part_id = (readBuff[0] & 0xF0) >> 4;
    info->revision = readBuff[0] & 0x0F;
    info->device_found = (info->part_id == 0x5);

    _last_error = MAX30001_SUCCESS;
    return MAX30001_SUCCESS;
}

// =============================================================================
// Channel configuration (shared by all start methods)
// =============================================================================

void MAX30001::_configure(bool ecg, bool bioz, max30001_sample_rate_t sample_rate,
                          max30001_ecg_gain_t gain)
{
    max30001_cnfg_gen_t cnfg_gen;
    max30001_cnfg_ecg_t cnfg_ecg;
    max30001_cnfg_emux_t cnfg_emux;
    max30001_cnfg_bioz_t cnfg_bioz;
    max30001_cnfg_bmux_t cnfg_bmux;
    max30001_cnfg_rtor1_t cnfg_rtor1;
    max30001_mngr_int_t mngr_int;

    _max30001SwReset();

    cnfg_gen.all = 0;
    cnfg_gen.bit.en_ecg = ecg ? 1 : 0;
    cnfg_gen.bit.en_bioz = bioz ? 1 : 0;
    cnfg_gen.bit.fmstr = 0b00;      // 32768 Hz: ECG 512/256/128 sps, BioZ 64 sps
    cnfg_gen.bit.rbiasv = 0b01;     // 100 MOhm (lead bias itself stays disabled)
    if (ecg) {
        cnfg_gen.bit.en_dcloff = 0b01; // DC lead-off on ECGP/ECGN
        cnfg_gen.bit.imag = 0b010;     // 10 nA
        cnfg_gen.bit.vth = 0b00;       // VMID +/- 300 mV
    }
    _max30001RegWrite(MAX30001_REG_CNFG_GEN, cnfg_gen.all);

    _max30001RegWrite(MAX30001_REG_CNFG_CAL, 0x720000); // Calibration sources disabled

    // RRINT is cleared by reading RTOR, so a STATUS read elsewhere cannot drop a beat
    mngr_int.all = 0;
    mngr_int.bit.e_fit = 0b01111;   // ECG FIFO interrupt at 16 samples
    mngr_int.bit.b_fit = 0b011;     // BioZ FIFO interrupt at 4 samples
    mngr_int.bit.clr_rrint = 0b01;
    mngr_int.bit.clr_samp = 1;
    _max30001RegWrite(MAX30001_REG_MNGR_INT, mngr_int.all);

    if (ecg) {
        cnfg_ecg.all = 0;
        cnfg_ecg.bit.rate = _rateToRegValue(sample_rate);
        cnfg_ecg.bit.gain = gain;
        cnfg_ecg.bit.dhpf = 0b1;    // 0.5 Hz high-pass
        cnfg_ecg.bit.dlpf = 0b01;   // ~40 Hz low-pass (28 Hz at 128 sps)
        _max30001RegWrite(MAX30001_REG_CNFG_ECG, cnfg_ecg.all);

        cnfg_emux.all = 0;          // ECGP/ECGN connected, no calibration, normal polarity
        _max30001RegWrite(MAX30001_REG_CNFG_EMUX, cnfg_emux.all);

        // Datasheet defaults for the R-to-R detector, with EN_RTOR set
        cnfg_rtor1.all = 0;
        cnfg_rtor1.bit.wndw = 0b0011;   // 96 ms
        cnfg_rtor1.bit.gain = 0b1111;   // Auto-scale
        cnfg_rtor1.bit.en_rtor = 1;
        cnfg_rtor1.bit.pavg = 0b10;     // 8
        cnfg_rtor1.bit.ptsf = 0b0011;   // 4/16
        _max30001RegWrite(MAX30001_REG_CNFG_RTOR1, cnfg_rtor1.all);
    }

    if (bioz) {
        cnfg_bioz.all = 0;
        cnfg_bioz.bit.rate = 0;         // 64 sps
        cnfg_bioz.bit.ahpf = 0b010;     // 500 Hz
        cnfg_bioz.bit.ext_rbias = 0;
        cnfg_bioz.bit.ln_bioz = 1;
        cnfg_bioz.bit.gain = 0b00;      // 10 V/V: ADC full scale ~6.25 kOhm at 16 uA (20 V/V clipped at ~3.1 kOhm)
        cnfg_bioz.bit.dhpf = 0b01;      // 0.05 Hz (0.5 Hz is a 4th-order filter and removes respiration)
        cnfg_bioz.bit.dlpf = 0b01;      // 4 Hz
        cnfg_bioz.bit.fcgen = 0b0100;   // ~8 kHz
        cnfg_bioz.bit.cgmon = 0;
        cnfg_bioz.bit.cgmag = 0b010;    // 16 uA
        cnfg_bioz.bit.phoff = 0b0011;   // 33.75 deg
        _max30001RegWrite(MAX30001_REG_CNFG_BIOZ, cnfg_bioz.all);

        _max30001RegWrite(MAX30001_REG_CNFG_BIOZ_LC, 0x800000); // High current range (8-96 uA)

        cnfg_bmux.all = 0;              // BIP/BIN connected, no calibration
        cnfg_bmux.bit.rmod = 0b100;     // Built-in modulated resistor load off
        _max30001RegWrite(MAX30001_REG_CNFG_BMUX, cnfg_bmux.all);
    }

    _max30001Synch();

    _sample_rate = sample_rate;
    _ecg_gain = gain;
    _ecg_enabled = ecg;
    _bioz_enabled = bioz;
    ecgSamplesAvailable = 0;
    biozSamplesAvailable = 0;
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

    _configure(true, false, sample_rate, gain);

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

    _configure(false, true, sample_rate, _ecg_gain);

    _last_error = MAX30001_SUCCESS;
    return MAX30001_SUCCESS;
}

max30001_error_t MAX30001::startECGBioZ(max30001_sample_rate_t sample_rate, max30001_ecg_gain_t gain)
{
    if (!_initialized) {
        _last_error = MAX30001_ERROR_NOT_INITIALIZED;
        return _last_error;
    }

    if (_validateSampleRate(sample_rate) != MAX30001_SUCCESS) {
        return _last_error;
    }

    _configure(true, true, sample_rate, gain);

    _last_error = MAX30001_SUCCESS;
    return MAX30001_SUCCESS;
}

max30001_error_t MAX30001::startRtoR(max30001_sample_rate_t sample_rate, max30001_ecg_gain_t gain)
{
    // R-to-R detection is enabled with every ECG start
    return startECG(sample_rate, gain);
}

max30001_error_t MAX30001::getECGSample(max30001_ecg_sample_t* sample)
{
    if (sample == nullptr) {
        _last_error = MAX30001_ERROR_INVALID_PARAMETER;
        return _last_error;
    }

    sample->sample_valid = false;

    if (!_initialized || !_ecg_enabled) {
        _last_error = MAX30001_ERROR_NOT_READY;
        return _last_error;
    }

    uint32_t word;
    _max30001RegRead24(MAX30001_REG_ECG_FIFO, &word);
    uint8_t etag = (word >> 3) & 0x07;

    if (etag == MAX30001_TAG_EMPTY) {
        _last_error = MAX30001_ERROR_NOT_READY;
        return _last_error;
    }

    if (etag == MAX30001_TAG_OVERFLOW) {
        _max30001FIFOReset();
        _last_error = MAX30001_ERROR_FIFO_OVERFLOW;
        return _last_error;
    }

    sample->ecg_sample = decodeECG(word);
    sample->timestamp_ms = millis();
    sample->lead_off_detected = getLeadOffStatus();
    sample->sample_valid = true;

    // Update legacy data member
    ecg_data = sample->ecg_sample;

    _last_error = MAX30001_SUCCESS;
    return MAX30001_SUCCESS;
}

max30001_error_t MAX30001::getBioZSample(max30001_bioz_sample_t* sample)
{
    if (sample == nullptr) {
        _last_error = MAX30001_ERROR_INVALID_PARAMETER;
        return _last_error;
    }

    sample->sample_valid = false;

    if (!_initialized || !_bioz_enabled) {
        _last_error = MAX30001_ERROR_NOT_READY;
        return _last_error;
    }

    uint32_t word;
    _max30001RegRead24(MAX30001_REG_BIOZ_FIFO, &word);
    uint8_t btag = word & 0x07;

    if (btag == MAX30001_TAG_EMPTY) {
        _last_error = MAX30001_ERROR_NOT_READY;
        return _last_error;
    }

    if (btag == MAX30001_TAG_OVERFLOW) {
        _max30001FIFOReset();
        _last_error = MAX30001_ERROR_FIFO_OVERFLOW;
        return _last_error;
    }

    sample->bioz_sample = decodeBioZ(word);
    sample->timestamp_ms = millis();
    sample->out_of_range = (btag == MAX30001_TAG_FAST_OR_RANGE ||
                            btag == MAX30001_TAG_FAST_OR_RANGE_EOF);
    sample->sample_valid = true;

    // Update legacy data member
    bioz_data = sample->bioz_sample;

    _last_error = MAX30001_SUCCESS;
    return MAX30001_SUCCESS;
}

max30001_error_t MAX30001::getRtoRData(max30001_rtor_data_t* rtor_data)
{
    if (rtor_data == nullptr) {
        _last_error = MAX30001_ERROR_INVALID_PARAMETER;
        return _last_error;
    }

    rtor_data->rr_detected = false;

    if (!_initialized || !_ecg_enabled) {
        _last_error = MAX30001_ERROR_NOT_READY;
        return _last_error;
    }

    _readStatus();
    if (global_status.bit.rrint) {
        _readRtoR();  // Reading RTOR clears RRINT
        rtor_data->rr_detected = (RRinterval > 0);
    }

    rtor_data->heart_rate_bpm = heartRate;
    rtor_data->rr_interval_ms = RRinterval;

    _last_error = MAX30001_SUCCESS;
    return MAX30001_SUCCESS;
}

void MAX30001::stop()
{
    if (!_initialized) {
        return;
    }

    // Disable all channels
    _max30001RegWrite(MAX30001_REG_CNFG_GEN, 0x000000);

    _ecg_enabled = false;
    _bioz_enabled = false;
}

float MAX30001::convertECGToMicrovolts(int32_t raw_value, max30001_ecg_gain_t gain)
{
    // VECG = ADC x VREF / (2^17 x ECG_GAIN), VREF = 1000 mV
    // LSB: 20 V/V = 381.47 nV, 40 V/V = 190.73 nV, 80 V/V = 95.37 nV, 160 V/V = 47.68 nV
    float gain_vv;
    switch(gain) {
        case MAX30001_ECG_GAIN_20:
            gain_vv = 20.0f;
            break;
        case MAX30001_ECG_GAIN_40:
            gain_vv = 40.0f;
            break;
        case MAX30001_ECG_GAIN_160:
            gain_vv = 160.0f;
            break;
        case MAX30001_ECG_GAIN_80:
        default:
            gain_vv = 80.0f;
            break;
    }

    return (float)raw_value * (1000000.0f / 131072.0f) / gain_vv;
}

max30001_error_t MAX30001::getLastError() const
{
    return _last_error;
}

// =============================================================================
// Configuration
// =============================================================================

max30001_error_t MAX30001::setECGGain(max30001_ecg_gain_t gain)
{
    if (!_initialized || !_ecg_enabled) {
        _last_error = MAX30001_ERROR_NOT_READY;
        return _last_error;
    }

    if (gain != MAX30001_ECG_GAIN_20 && gain != MAX30001_ECG_GAIN_40 &&
        gain != MAX30001_ECG_GAIN_80 && gain != MAX30001_ECG_GAIN_160) {
        _last_error = MAX30001_ERROR_INVALID_PARAMETER;
        return _last_error;
    }

    max30001_cnfg_ecg_t cnfg_ecg;
    _max30001RegRead24(MAX30001_REG_CNFG_ECG, &cnfg_ecg.all);
    cnfg_ecg.bit.gain = gain;
    _max30001RegWrite(MAX30001_REG_CNFG_ECG, cnfg_ecg.all);

    _ecg_gain = gain;
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

    max30001_cnfg_gen_t cnfg_gen;
    _max30001RegRead24(MAX30001_REG_CNFG_GEN, &cnfg_gen.all);
    cnfg_gen.bit.en_ecg = 1;
    _max30001RegWrite(MAX30001_REG_CNFG_GEN, cnfg_gen.all);

    _ecg_enabled = true;
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

    max30001_cnfg_gen_t cnfg_gen;
    _max30001RegRead24(MAX30001_REG_CNFG_GEN, &cnfg_gen.all);
    cnfg_gen.bit.en_ecg = 0;
    _max30001RegWrite(MAX30001_REG_CNFG_GEN, cnfg_gen.all);

    _ecg_enabled = false;
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

    max30001_cnfg_gen_t cnfg_gen;
    _max30001RegRead24(MAX30001_REG_CNFG_GEN, &cnfg_gen.all);
    cnfg_gen.bit.en_bioz = 1;
    _max30001RegWrite(MAX30001_REG_CNFG_GEN, cnfg_gen.all);

    _bioz_enabled = true;
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

    max30001_cnfg_gen_t cnfg_gen;
    _max30001RegRead24(MAX30001_REG_CNFG_GEN, &cnfg_gen.all);
    cnfg_gen.bit.en_bioz = 0;
    _max30001RegWrite(MAX30001_REG_CNFG_GEN, cnfg_gen.all);

    _bioz_enabled = false;
    _last_error = MAX30001_SUCCESS;
    return MAX30001_SUCCESS;
}

max30001_error_t MAX30001::setECGHighPassFilter(float cutoff_hz)
{
    if (!_initialized || !_ecg_enabled) {
        _last_error = MAX30001_ERROR_NOT_READY;
        return _last_error;
    }

    // The ECG channel has one high-pass option: bypass (DC) or 0.5 Hz
    max30001_cnfg_ecg_t cnfg_ecg;
    _max30001RegRead24(MAX30001_REG_CNFG_ECG, &cnfg_ecg.all);
    cnfg_ecg.bit.dhpf = (cutoff_hz > 0.0f) ? 1 : 0;
    _max30001RegWrite(MAX30001_REG_CNFG_ECG, cnfg_ecg.all);

    _last_error = MAX30001_SUCCESS;
    return MAX30001_SUCCESS;
}

max30001_error_t MAX30001::setECGLowPassFilter(float cutoff_hz)
{
    if (!_initialized || !_ecg_enabled) {
        _last_error = MAX30001_ERROR_NOT_READY;
        return _last_error;
    }

    // DLPF: 00 = bypass, 01 = 40 Hz, 10 = 100 Hz, 11 = 150 Hz.
    // 100 Hz needs >= 256 sps and 150 Hz needs 512 sps; at 128 sps every
    // setting except bypass gives 28.35 Hz. Unsupported settings fall back to 01.
    uint8_t dlpf_bits;
    if (cutoff_hz <= 0.0f) {
        dlpf_bits = 0b00;
    } else if (cutoff_hz < 70.0f) {
        dlpf_bits = 0b01;
    } else if (cutoff_hz < 125.0f) {
        dlpf_bits = 0b10;
    } else {
        dlpf_bits = 0b11;
    }

    max30001_cnfg_ecg_t cnfg_ecg;
    _max30001RegRead24(MAX30001_REG_CNFG_ECG, &cnfg_ecg.all);
    cnfg_ecg.bit.dlpf = dlpf_bits;
    _max30001RegWrite(MAX30001_REG_CNFG_ECG, cnfg_ecg.all);

    _last_error = MAX30001_SUCCESS;
    return MAX30001_SUCCESS;
}

bool MAX30001::getLeadOffStatus()
{
    max30001_leadoff_t leadoff;
    if (getLeadOffDetail(&leadoff) != MAX30001_SUCCESS) {
        return false;
    }
    return leadoff.any;
}

max30001_error_t MAX30001::getLeadOffDetail(max30001_leadoff_t* leadoff)
{
    if (leadoff == nullptr) {
        _last_error = MAX30001_ERROR_INVALID_PARAMETER;
        return _last_error;
    }

    leadoff->any = false;
    leadoff->p_high = leadoff->p_low = leadoff->n_high = leadoff->n_low = false;

    if (!_initialized || !_ecg_enabled) {
        _last_error = MAX30001_ERROR_NOT_READY;
        return _last_error;
    }

    _readStatus();
    leadoff->any = global_status.bit.dcloffint;
    leadoff->p_high = global_status.bit.loff_ph;
    leadoff->p_low = global_status.bit.loff_pl;
    leadoff->n_high = global_status.bit.loff_nh;
    leadoff->n_low = global_status.bit.loff_nl;

    _last_error = MAX30001_SUCCESS;
    return MAX30001_SUCCESS;
}

uint8_t MAX30001::getFIFOCount()
{
    if (!_initialized) {
        return 0;
    }

    // The MAX30001 has no FIFO count register; this is the number of ECG
    // samples decoded by the last burst read (max30001ServiceAllInterrupts)
    return ecgSamplesAvailable;
}

max30001_error_t MAX30001::clearFIFO()
{
    if (!_initialized) {
        _last_error = MAX30001_ERROR_NOT_INITIALIZED;
        return _last_error;
    }

    _max30001FIFOReset();

    ecgSamplesAvailable = 0;
    biozSamplesAvailable = 0;
    _last_error = MAX30001_SUCCESS;
    return MAX30001_SUCCESS;
}

max30001_error_t MAX30001::enableInterrupt()
{
    if (!_initialized) {
        _last_error = MAX30001_ERROR_NOT_INITIALIZED;
        return _last_error;
    }

    // INTB goes low while the ECG FIFO holds >= 16 samples (or the BioZ FIFO
    // >= 4). CMOS output, since the board's TXB0108 does not pass a weak pull-up well.
    uint32_t en_int = MAX30001_EN_INT_EINT | MAX30001_INTB_TYPE_CMOS;
    if (_bioz_enabled) {
        en_int |= MAX30001_EN_INT_BINT;
    }
    _max30001RegWrite(MAX30001_REG_EN_INT, en_int);

    _last_error = MAX30001_SUCCESS;
    return MAX30001_SUCCESS;
}

max30001_error_t MAX30001::disableInterrupt()
{
    if (!_initialized) {
        _last_error = MAX30001_ERROR_NOT_INITIALIZED;
        return _last_error;
    }

    // No interrupt sources; keep INTB driven high
    _max30001RegWrite(MAX30001_REG_EN_INT, MAX30001_INTB_TYPE_CMOS);

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
    return isConnected();
}

void MAX30001::BeginECGOnly()
{
    _max30001SwReset();
    delay(100);
    _max30001RegWrite(MAX30001_REG_CNFG_GEN, 0x081007);
    delay(100);
    _max30001RegWrite(MAX30001_REG_CNFG_CAL, 0x720000); // 0x700000
    delay(100);
    _max30001RegWrite(MAX30001_REG_CNFG_EMUX, 0x0B0000);
    delay(100);
    _max30001RegWrite(MAX30001_REG_CNFG_ECG, 0x825000); // d23 - d22 : 10 for 250sps , 00:500 sps
    delay(100);

    _max30001RegWrite(MAX30001_REG_CNFG_RTOR1, 0x3fc600);
    _max30001Synch();
    delay(100);
}

void MAX30001::BeginECGBioZ()
{
    _configure(true, true, MAX30001_RATE_128, MAX30001_ECG_GAIN_80);
}

void MAX30001::BeginRtoRMode()
{
    _max30001SwReset();
    delay(100);
    _max30001RegWrite(MAX30001_REG_CNFG_GEN, 0x080004);
    delay(100);
    _max30001RegWrite(MAX30001_REG_CNFG_CAL, 0x720000); // 0x700000
    delay(100);
    _max30001RegWrite(MAX30001_REG_CNFG_EMUX, 0x0B0000);
    delay(100);
    _max30001RegWrite(MAX30001_REG_CNFG_ECG, 0x805000); // d23 - d22 : 10 for 250sps , 00:500 sps
    delay(100);
    _max30001RegWrite(MAX30001_REG_CNFG_RTOR1, 0x3fc600);
    delay(100);
    _max30001RegWrite(MAX30001_REG_EN_INT, 0x000401);
    delay(100);
    _max30001Synch();
    delay(100);
}

void MAX30001::max30001SetsamplingRate(uint16_t samplingRate)
{
    max30001_sample_rate_t rate = (max30001_sample_rate_t)samplingRate;
    if (_validateSampleRate(rate) != MAX30001_SUCCESS) {
        return;
    }

    max30001_cnfg_ecg_t cnfg_ecg;
    _max30001RegRead24(MAX30001_REG_CNFG_ECG, &cnfg_ecg.all);
    cnfg_ecg.bit.rate = _rateToRegValue(rate);
    _max30001RegWrite(MAX30001_REG_CNFG_ECG, cnfg_ecg.all);

    _sample_rate = rate;
}

signed long MAX30001::getECGSamples(void)
{
    uint32_t word;
    _max30001RegRead24(MAX30001_REG_ECG_FIFO, &word);

    return decodeECG(word);
}

signed long MAX30001::getBioZSamples(void)
{
    uint32_t word;
    _max30001RegRead24(MAX30001_REG_BIOZ_FIFO, &word);

    return decodeBioZ(word);
}

void MAX30001::getHRandRR(void)
{
    _readRtoR();
}

void MAX30001::_readRtoR(void)
{
    uint32_t rtor_reg;
    _max30001RegRead24(MAX30001_REG_RTOR, &rtor_reg);

    // RTOR D[23:10]: R-to-R interval in units of 7.8125 ms
    uint16_t rtor = (rtor_reg >> 10) & 0x3FFF;
    if (rtor == 0) {
        heartRate = 0;
        RRinterval = 0;
        return;
    }

    float rr_ms = (float)rtor * MAX30001_RTOR_LSB_MS;
    RRinterval = (unsigned int)rr_ms;
    heartRate = (unsigned int)(60000.0f / rr_ms + 0.5f);
}

void MAX30001::max30001SetInterrupts(uint32_t interrupts_to_set)
{
    _max30001RegWrite(MAX30001_REG_EN_INT, interrupts_to_set);
}

void MAX30001::readStatus(void)
{
    _readStatus();
}

void MAX30001::_readStatus(void)
{
    _max30001RegRead24(MAX30001_REG_STATUS, &global_status.all);
}

void MAX30001::_max30001ReadECGFIFO(int num_bytes)
{
    if (num_bytes > (int)sizeof(_readBufferECG)) {
        num_bytes = sizeof(_readBufferECG);
    }

    _spi->beginTransaction(SPISettings(MAX30001_SPI_SPEED, MSBFIRST, SPI_MODE0));

    digitalWrite(_cs_pin, LOW);

    _spi->transfer((MAX30001_REG_ECG_FIFO_BURST << 1) | MAX30001_SPI_READ);

    for (int i = 0; i < num_bytes; i++)
    {
        _readBufferECG[i] = _spi->transfer(0x00);
    }

    digitalWrite(_cs_pin, HIGH);

    _spi->endTransaction();

    int count = 0;

    for (int i = 0; i + 2 < num_bytes; i += 3)
    {
        uint32_t word = bytesToWord(&_readBufferECG[i]);
        uint8_t etag = (word >> 3) & 0x07;

        if (etag == MAX30001_TAG_VALID || etag == MAX30001_TAG_VALID_EOF ||
            etag == MAX30001_TAG_FAST_OR_RANGE || etag == MAX30001_TAG_FAST_OR_RANGE_EOF)
        {
            s32ECGData[count++] = decodeECG(word);
        }
        else if (etag == MAX30001_TAG_OVERFLOW)
        {
            _max30001FIFOReset();
            break;
        }
    }

    ecgSamplesAvailable = count;
}

void MAX30001::_max30001ReadBIOZFIFO(int num_bytes)
{
    if (num_bytes > (int)sizeof(_readBufferBIOZ)) {
        num_bytes = sizeof(_readBufferBIOZ);
    }

    _spi->beginTransaction(SPISettings(MAX30001_SPI_SPEED, MSBFIRST, SPI_MODE0));

    digitalWrite(_cs_pin, LOW);

    _spi->transfer((MAX30001_REG_BIOZ_FIFO_BURST << 1) | MAX30001_SPI_READ);

    for (int i = 0; i < num_bytes; i++)
    {
        _readBufferBIOZ[i] = _spi->transfer(0x00);
    }

    digitalWrite(_cs_pin, HIGH);

    _spi->endTransaction();

    int count = 0;

    for (int i = 0; i + 2 < num_bytes; i += 3)
    {
        uint32_t word = bytesToWord(&_readBufferBIOZ[i]);
        uint8_t btag = word & 0x07;

        if (btag == MAX30001_TAG_VALID || btag == MAX30001_TAG_VALID_EOF ||
            btag == MAX30001_TAG_FAST_OR_RANGE || btag == MAX30001_TAG_FAST_OR_RANGE_EOF)
        {
            s32BIOZData[count++] = decodeBioZ(word);
        }
        else if (btag == MAX30001_TAG_OVERFLOW)
        {
            _max30001FIFOReset();
            break;
        }
    }

    biozSamplesAvailable = count;
}

void MAX30001::max30001ServiceAllInterrupts(void)
{
    max30001_mngr_int_t mngr_int;

    _readStatus();

    if (global_status.bit.eint == 1) // ECG FIFO reached its threshold
    {
        _max30001RegRead24(MAX30001_REG_MNGR_INT, &mngr_int.all);
        _max30001ReadECGFIFO((mngr_int.bit.e_fit + 1) * 3);
    }

    if (global_status.bit.bint == 1) // BioZ FIFO reached its threshold
    {
        _max30001RegRead24(MAX30001_REG_MNGR_INT, &mngr_int.all);
        _max30001ReadBIOZFIFO((mngr_int.bit.b_fit + 1) * 3);
    }
}
