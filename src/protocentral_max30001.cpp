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
      _sample_rate(MAX30001_RATE_128), _ecg_enabled(false), _bioz_enabled(false),
      _bioz_gain(MAX30001_BIOZ_GAIN_10), _bioz_current(MAX30001_BIOZ_CURRENT_16UA),
      _bioz_freq(MAX30001_BIOZ_FREQ_8KHZ), _bioz_ahpf(MAX30001_BIOZ_AHPF_500HZ),
      _bioz_phoff(0b0011), _bioz_dhpf(0b01), _bioz_dlpf(0b01),
      _bioz_ext_rbias(false), _bioz_cgmon(false),
      _bloff_mode(0), _bloff_hi(0xFF), _bloff_lo(0xFF),
      _leadoff_enabled(true), _leadoff_current(MAX30001_LEADOFF_CURRENT_10NA),
      _leadoff_threshold(MAX30001_LEADOFF_THRESHOLD_300MV),
      _fast_mode(MAX30001_FAST_RECOVERY_OFF), _fast_threshold(0x3F),
      _lead_bias(MAX30001_LEAD_BIAS_OFF), _lead_bias_res(MAX30001_LEAD_BIAS_100MOHM),
      _ecg_inverted(false), _status_latched(0)
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

// CNFG_CAL with the calibration sources off (needed for the BioZ test load)
#define MAX30001_CAL_OFF 0x004800UL  // FCAL = 1 Hz, FIFTY = 1, EN_VCAL = 0

uint32_t MAX30001::_genValue(bool ecg, bool bioz, bool lead_on_detect) const
{
    max30001_cnfg_gen_t cnfg_gen;

    cnfg_gen.all = 0;
    cnfg_gen.bit.en_ecg = ecg ? 1 : 0;
    cnfg_gen.bit.en_bioz = bioz ? 1 : 0;
    cnfg_gen.bit.fmstr = 0b00;      // 32768 Hz: ECG 512/256/128 sps, BioZ 64 sps
    cnfg_gen.bit.rbiasv = _lead_bias_res;
    if (lead_on_detect) {
        cnfg_gen.bit.en_ulp_lon = 0b01;
    }
    if (ecg && _leadoff_enabled) {
        cnfg_gen.bit.en_dcloff = 0b01;
        cnfg_gen.bit.imag = _leadoff_current;
        cnfg_gen.bit.vth = _leadoff_threshold;
    }
    if (bioz) {
        cnfg_gen.bit.en_bloff = _bloff_mode;
    }
    if ((_lead_bias == MAX30001_LEAD_BIAS_ECG && ecg) ||
        (_lead_bias == MAX30001_LEAD_BIAS_BIOZ && bioz)) {
        cnfg_gen.bit.en_rbias = _lead_bias;
        cnfg_gen.bit.rbiasp = 1;
        cnfg_gen.bit.rbiasn = 1;
    }
    return cnfg_gen.all;
}

uint32_t MAX30001::_biozValue() const
{
    max30001_cnfg_bioz_t cnfg_bioz;

    cnfg_bioz.all = 0;
    cnfg_bioz.bit.rate = 0;         // 64 sps
    cnfg_bioz.bit.ahpf = _bioz_ahpf;
    cnfg_bioz.bit.ext_rbias = _bioz_ext_rbias ? 1 : 0;
    cnfg_bioz.bit.ln_bioz = 1;
    cnfg_bioz.bit.gain = _bioz_gain;
    cnfg_bioz.bit.dhpf = _bioz_dhpf;
    cnfg_bioz.bit.dlpf = _bioz_dlpf;
    cnfg_bioz.bit.fcgen = _bioz_freq;
    cnfg_bioz.bit.cgmon = _bioz_cgmon ? 1 : 0;
    cnfg_bioz.bit.cgmag = _bioz_current;
    cnfg_bioz.bit.phoff = _bioz_phoff;
    return cnfg_bioz.all;
}

uint32_t MAX30001::_emuxValue() const
{
    max30001_cnfg_emux_t cnfg_emux;

    cnfg_emux.all = 0;              // ECGP/ECGN connected, no calibration
    cnfg_emux.bit.pol = _ecg_inverted ? 1 : 0;
    return cnfg_emux.all;
}

uint32_t MAX30001::_bmuxValue() const
{
    max30001_cnfg_bmux_t cnfg_bmux;

    cnfg_bmux.all = 0;              // BIP/BIN connected, no calibration
    cnfg_bmux.bit.rmod = 0b100;     // Test load off
    return cnfg_bmux.all;
}

uint32_t MAX30001::_mngrDynValue() const
{
    return ((uint32_t)_fast_mode << 22) | ((uint32_t)_fast_threshold << 16) |
           ((uint32_t)_bloff_hi << 8) | (uint32_t)_bloff_lo;
}

uint32_t MAX30001::_rtor1Value() const
{
    max30001_cnfg_rtor1_t cnfg_rtor1;

    cnfg_rtor1.all = 0;
    cnfg_rtor1.bit.wndw = _rtor.window;
    cnfg_rtor1.bit.gain = _rtor.gain;
    cnfg_rtor1.bit.en_rtor = 1;
    cnfg_rtor1.bit.pavg = _rtor.peak_average;
    cnfg_rtor1.bit.ptsf = _rtor.peak_threshold;
    return cnfg_rtor1.all;
}

uint32_t MAX30001::_rtor2Value() const
{
    max30001_cnfg_rtor2_t cnfg_rtor2;

    cnfg_rtor2.all = 0;
    cnfg_rtor2.bit.hoff = _rtor.holdoff_min;
    cnfg_rtor2.bit.ravg = _rtor.interval_average;
    cnfg_rtor2.bit.rhsf = _rtor.holdoff_scale;
    return cnfg_rtor2.all;
}

void MAX30001::_configure(bool ecg, bool bioz, max30001_sample_rate_t sample_rate,
                          max30001_ecg_gain_t gain)
{
    max30001_cnfg_ecg_t cnfg_ecg;
    max30001_mngr_int_t mngr_int;

    _max30001SwReset();

    _max30001RegWrite(MAX30001_REG_CNFG_GEN, _genValue(ecg, bioz, false));
    _max30001RegWrite(MAX30001_REG_CNFG_CAL, MAX30001_CAL_OFF);

    // RRINT is cleared by reading RTOR, so a STATUS read elsewhere cannot drop a beat
    mngr_int.all = 0;
    mngr_int.bit.e_fit = 0b01111;   // ECG FIFO interrupt at 16 samples
    mngr_int.bit.b_fit = 0b011;     // BioZ FIFO interrupt at 4 samples
    mngr_int.bit.clr_rrint = 0b01;
    mngr_int.bit.clr_samp = 1;
    _max30001RegWrite(MAX30001_REG_MNGR_INT, mngr_int.all);
    _max30001RegWrite(MAX30001_REG_MNGR_DYN, _mngrDynValue());

    if (ecg) {
        cnfg_ecg.all = 0;
        cnfg_ecg.bit.rate = _rateToRegValue(sample_rate);
        cnfg_ecg.bit.gain = gain;
        cnfg_ecg.bit.dhpf = 0b1;    // 0.5 Hz high-pass
        cnfg_ecg.bit.dlpf = 0b01;   // ~40 Hz low-pass (28 Hz at 128 sps)
        _max30001RegWrite(MAX30001_REG_CNFG_ECG, cnfg_ecg.all);
        _max30001RegWrite(MAX30001_REG_CNFG_EMUX, _emuxValue());
        _max30001RegWrite(MAX30001_REG_CNFG_RTOR1, _rtor1Value());
        _max30001RegWrite(MAX30001_REG_CNFG_RTOR2, _rtor2Value());
    }

    if (bioz) {
        _max30001RegWrite(MAX30001_REG_CNFG_BIOZ, _biozValue());
        _max30001RegWrite(MAX30001_REG_CNFG_BIOZ_LC, 0x800000); // High current range (8-96 uA)
        _max30001RegWrite(MAX30001_REG_CNFG_BMUX, _bmuxValue());
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

max30001_error_t MAX30001::setECGPolarity(bool inverted)
{
    _ecg_inverted = inverted;

    if (_initialized && _ecg_enabled) {
        _max30001RegWrite(MAX30001_REG_CNFG_EMUX, _emuxValue());
    }

    _last_error = MAX30001_SUCCESS;
    return MAX30001_SUCCESS;
}

max30001_error_t MAX30001::setLeadOffDetection(bool enable, max30001_leadoff_current_t current,
                                               max30001_leadoff_threshold_t threshold)
{
    if (current < MAX30001_LEADOFF_CURRENT_5NA || current > MAX30001_LEADOFF_CURRENT_100NA ||
        threshold > MAX30001_LEADOFF_THRESHOLD_500MV) {
        _last_error = MAX30001_ERROR_INVALID_PARAMETER;
        return _last_error;
    }

    _leadoff_enabled = enable;
    _leadoff_current = current;
    _leadoff_threshold = threshold;

    if (_initialized && (_ecg_enabled || _bioz_enabled)) {
        _max30001RegWrite(MAX30001_REG_CNFG_GEN, _genValue(_ecg_enabled, _bioz_enabled, false));
    }

    _last_error = MAX30001_SUCCESS;
    return MAX30001_SUCCESS;
}

max30001_error_t MAX30001::setRtoRConfig(const max30001_rtor_config_t& config)
{
    if (config.window > 11 || config.gain > 15 || config.peak_average > 3 ||
        config.peak_threshold > 15 || config.holdoff_min > 63 ||
        config.interval_average > 3 || config.holdoff_scale > 7) {
        _last_error = MAX30001_ERROR_INVALID_PARAMETER;
        return _last_error;
    }

    _rtor = config;

    if (_initialized && _ecg_enabled) {
        _max30001RegWrite(MAX30001_REG_CNFG_RTOR1, _rtor1Value());
        _max30001RegWrite(MAX30001_REG_CNFG_RTOR2, _rtor2Value());
    }

    _last_error = MAX30001_SUCCESS;
    return MAX30001_SUCCESS;
}

max30001_error_t MAX30001::setFastRecovery(max30001_fast_recovery_t mode, uint8_t threshold)
{
    if (mode > MAX30001_FAST_RECOVERY_AUTO || threshold > 0x3F) {
        _last_error = MAX30001_ERROR_INVALID_PARAMETER;
        return _last_error;
    }

    _fast_mode = mode;
    _fast_threshold = threshold;

    if (_initialized && (_ecg_enabled || _bioz_enabled)) {
        _max30001RegWrite(MAX30001_REG_MNGR_DYN, _mngrDynValue());
    }

    _last_error = MAX30001_SUCCESS;
    return MAX30001_SUCCESS;
}

max30001_error_t MAX30001::setLeadBias(max30001_lead_bias_t mode,
                                       max30001_lead_bias_resistance_t resistance)
{
    if (mode > MAX30001_LEAD_BIAS_BIOZ || resistance > MAX30001_LEAD_BIAS_200MOHM) {
        _last_error = MAX30001_ERROR_INVALID_PARAMETER;
        return _last_error;
    }

    _lead_bias = mode;
    _lead_bias_res = resistance;

    if (_initialized && (_ecg_enabled || _bioz_enabled)) {
        _max30001RegWrite(MAX30001_REG_CNFG_GEN, _genValue(_ecg_enabled, _bioz_enabled, false));
    }

    _last_error = MAX30001_SUCCESS;
    return MAX30001_SUCCESS;
}

// =============================================================================
// BioZ Configuration
// =============================================================================

// Table 41: highest drive current allowed at each drive frequency
static bool biozCurrentAllowed(max30001_bioz_current_t current, max30001_bioz_freq_t freq)
{
    if (freq <= MAX30001_BIOZ_FREQ_18KHZ) return true;
    if (freq == MAX30001_BIOZ_FREQ_8KHZ) return current <= MAX30001_BIOZ_CURRENT_80UA;
    if (freq == MAX30001_BIOZ_FREQ_4KHZ) return current <= MAX30001_BIOZ_CURRENT_32UA;
    if (freq == MAX30001_BIOZ_FREQ_2KHZ) return current <= MAX30001_BIOZ_CURRENT_16UA;
    return current <= MAX30001_BIOZ_CURRENT_8UA;
}

max30001_error_t MAX30001::_applyBioZ()
{
    if (_initialized && _bioz_enabled) {
        _max30001RegWrite(MAX30001_REG_CNFG_BIOZ, _biozValue());
    }

    _last_error = MAX30001_SUCCESS;
    return MAX30001_SUCCESS;
}

max30001_error_t MAX30001::setBioZGain(max30001_bioz_gain_t gain)
{
    if (gain > MAX30001_BIOZ_GAIN_80) {
        _last_error = MAX30001_ERROR_INVALID_PARAMETER;
        return _last_error;
    }

    _bioz_gain = gain;
    return _applyBioZ();
}

max30001_error_t MAX30001::setBioZCurrent(max30001_bioz_current_t current)
{
    if (current < MAX30001_BIOZ_CURRENT_8UA || current > MAX30001_BIOZ_CURRENT_96UA ||
        !biozCurrentAllowed(current, _bioz_freq)) {
        _last_error = MAX30001_ERROR_INVALID_PARAMETER;
        return _last_error;
    }

    _bioz_current = current;
    return _applyBioZ();
}

max30001_error_t MAX30001::setBioZFrequency(max30001_bioz_freq_t frequency)
{
    if (frequency > MAX30001_BIOZ_FREQ_125HZ || !biozCurrentAllowed(_bioz_current, frequency)) {
        _last_error = MAX30001_ERROR_INVALID_PARAMETER;
        return _last_error;
    }

    _bioz_freq = frequency;
    return _applyBioZ();
}

max30001_error_t MAX30001::setBioZPhaseOffset(uint8_t step)
{
    if (step > 15) {
        _last_error = MAX30001_ERROR_INVALID_PARAMETER;
        return _last_error;
    }

    _bioz_phoff = step;
    return _applyBioZ();
}

max30001_error_t MAX30001::setBioZHighPassFilter(float cutoff_hz)
{
    if (cutoff_hz <= 0.0f) {
        _bioz_dhpf = 0b00;
    } else if (cutoff_hz < 0.25f) {
        _bioz_dhpf = 0b01;
    } else {
        _bioz_dhpf = 0b10;
    }
    return _applyBioZ();
}

max30001_error_t MAX30001::setBioZLowPassFilter(float cutoff_hz)
{
    if (cutoff_hz <= 0.0f) {
        _bioz_dlpf = 0b00;
    } else if (cutoff_hz < 6.0f) {
        _bioz_dlpf = 0b01;
    } else if (cutoff_hz < 12.0f) {
        _bioz_dlpf = 0b10;
    } else {
        _bioz_dlpf = 0b11;
    }
    return _applyBioZ();
}

max30001_error_t MAX30001::setBioZAnalogHighPass(max30001_bioz_ahpf_t ahpf)
{
    if (ahpf > MAX30001_BIOZ_AHPF_BYPASS) {
        _last_error = MAX30001_ERROR_INVALID_PARAMETER;
        return _last_error;
    }

    _bioz_ahpf = ahpf;
    return _applyBioZ();
}

max30001_error_t MAX30001::setBioZExternalBias(bool enable)
{
    _bioz_ext_rbias = enable;
    return _applyBioZ();
}

max30001_error_t MAX30001::setBioZCurrentMonitor(bool enable)
{
    _bioz_cgmon = enable;
    return _applyBioZ();
}

max30001_error_t MAX30001::setBioZLeadOff(bool over_range, bool under_range,
                                          uint8_t over_threshold, uint8_t under_threshold)
{
    // EN_BLOFF: 01 = under range, 10 = over range, 11 = both
    _bloff_mode = (over_range ? 0b10 : 0) | (under_range ? 0b01 : 0);
    _bloff_hi = over_threshold;
    _bloff_lo = under_threshold;

    if (_initialized && (_ecg_enabled || _bioz_enabled)) {
        _max30001RegWrite(MAX30001_REG_MNGR_DYN, _mngrDynValue());
        _max30001RegWrite(MAX30001_REG_CNFG_GEN, _genValue(_ecg_enabled, _bioz_enabled, false));
    }

    _last_error = MAX30001_SUCCESS;
    return MAX30001_SUCCESS;
}

float MAX30001::convertBioZToOhms(int32_t raw_value) const
{
    // BioZ = ADC x VREF / (2^19 x CGMAG x GAIN), VREF = 1 V
    static const uint8_t current_ua[] = {0, 8, 16, 32, 48, 64, 80, 96};
    float gain_vv = (float)(10 << _bioz_gain);

    return (float)raw_value * (1000000.0f / 524288.0f) /
           ((float)current_ua[_bioz_current & 0x07] * gain_vv);
}

// =============================================================================
// Self-Test, Lead-On and Diagnostics
// =============================================================================

max30001_error_t MAX30001::enableECGTestSignal(max30001_cal_freq_t frequency, bool half_millivolt)
{
    if (!_initialized || !_ecg_enabled) {
        _last_error = MAX30001_ERROR_NOT_READY;
        return _last_error;
    }

    if (frequency > MAX30001_CAL_FREQ_0_015HZ) {
        _last_error = MAX30001_ERROR_INVALID_PARAMETER;
        return _last_error;
    }

    // EN_VCAL, bipolar (VMID +/- VMAG), 50 % duty cycle
    uint32_t cal = (1UL << 22) | (1UL << 21) | (half_millivolt ? (1UL << 20) : 0) |
                   ((uint32_t)frequency << 12) | (1UL << 11);
    _max30001RegWrite(MAX30001_REG_CNFG_CAL, cal);

    // Isolate ECGP/ECGN from the electrodes; ECGP = VCALP, ECGN = VCALN
    max30001_cnfg_emux_t cnfg_emux;
    cnfg_emux.all = _emuxValue();
    cnfg_emux.bit.openp = 1;
    cnfg_emux.bit.openn = 1;
    cnfg_emux.bit.calp_sel = 0b10;
    cnfg_emux.bit.caln_sel = 0b11;
    _max30001RegWrite(MAX30001_REG_CNFG_EMUX, cnfg_emux.all);

    _last_error = MAX30001_SUCCESS;
    return MAX30001_SUCCESS;
}

max30001_error_t MAX30001::disableECGTestSignal()
{
    if (!_initialized) {
        _last_error = MAX30001_ERROR_NOT_INITIALIZED;
        return _last_error;
    }

    _max30001RegWrite(MAX30001_REG_CNFG_EMUX, _emuxValue());
    _max30001RegWrite(MAX30001_REG_CNFG_CAL, MAX30001_CAL_OFF);

    _last_error = MAX30001_SUCCESS;
    return MAX30001_SUCCESS;
}

max30001_error_t MAX30001::enableBioZTestLoad(max30001_bioz_rnom_t rnom, max30001_bioz_rmod_t rmod,
                                              max30001_bioz_test_freq_t frequency)
{
    if (!_initialized || !_bioz_enabled) {
        _last_error = MAX30001_ERROR_NOT_READY;
        return _last_error;
    }

    if (rnom > MAX30001_BIOZ_RNOM_625 || frequency > MAX30001_BIOZ_TEST_FREQ_0_0625HZ ||
        (rmod != MAX30001_BIOZ_RMOD_NONE && bioZTestLoadModulationOhms(rnom, rmod) <= 0.0f)) {
        _last_error = MAX30001_ERROR_INVALID_PARAMETER;
        return _last_error;
    }

    // The test load only works with the calibration sources off
    _max30001RegWrite(MAX30001_REG_CNFG_CAL, MAX30001_CAL_OFF);

    // Open BIP/BIN so the body does not load the test resistor
    max30001_cnfg_bmux_t cnfg_bmux;
    cnfg_bmux.all = 0;
    cnfg_bmux.bit.openp = 1;
    cnfg_bmux.bit.openn = 1;
    cnfg_bmux.bit.en_bist = 1;
    cnfg_bmux.bit.rnom = rnom;
    cnfg_bmux.bit.rmod = rmod;
    cnfg_bmux.bit.fbist = frequency;
    _max30001RegWrite(MAX30001_REG_CNFG_BMUX, cnfg_bmux.all);

    _last_error = MAX30001_SUCCESS;
    return MAX30001_SUCCESS;
}

max30001_error_t MAX30001::disableBioZTestLoad()
{
    if (!_initialized) {
        _last_error = MAX30001_ERROR_NOT_INITIALIZED;
        return _last_error;
    }

    _max30001RegWrite(MAX30001_REG_CNFG_BMUX, _bmuxValue());

    _last_error = MAX30001_SUCCESS;
    return MAX30001_SUCCESS;
}

float MAX30001::bioZTestLoadOhms(max30001_bioz_rnom_t rnom)
{
    static const uint16_t nominal[] = {5000, 2500, 1667, 1250, 1000, 833, 714, 625};
    return (rnom <= MAX30001_BIOZ_RNOM_625) ? (float)nominal[rnom] : 0.0f;
}

float MAX30001::bioZTestLoadModulationOhms(max30001_bioz_rnom_t rnom, max30001_bioz_rmod_t rmod)
{
    // Table 36, in tenths of a milliohm; 0 = combination not available
    static const uint16_t modulation[8][3] = {
        {29607, 9806, 2475},
        {7404, 2452, 619},
        {3291, 1090, 275},
        {1851, 613, 0},
        {1185, 392, 0},
        {823, 272, 0},
        {605, 200, 0},
        {463, 153, 0},
    };

    if (rnom > MAX30001_BIOZ_RNOM_625 || rmod > MAX30001_BIOZ_RMOD_2) {
        return 0.0f;
    }
    return (float)modulation[rnom][rmod] / 10000.0f;
}

max30001_error_t MAX30001::startLeadOnDetection(bool interrupt)
{
    if (!_initialized) {
        _last_error = MAX30001_ERROR_NOT_INITIALIZED;
        return _last_error;
    }

    // ULP lead-on detection only runs with both channels off
    _max30001RegWrite(MAX30001_REG_CNFG_GEN, _genValue(false, false, true));
    _ecg_enabled = false;
    _bioz_enabled = false;

    // LONINT is set when the mode is entered; one STATUS read clears it
    _max30001RegRead24(MAX30001_REG_STATUS, &global_status.all);

    _max30001RegWrite(MAX30001_REG_EN_INT, interrupt ? (MAX30001_EN_INT_LONINT | MAX30001_INTB_TYPE_CMOS)
                                                     : MAX30001_INTB_TYPE_CMOS);

    _last_error = MAX30001_SUCCESS;
    return MAX30001_SUCCESS;
}

bool MAX30001::isLeadOn()
{
    if (!_initialized) {
        return false;
    }

    _readStatus();
    return global_status.bit.lonint;
}

max30001_error_t MAX30001::getStatus(max30001_status_flags_t* flags)
{
    if (flags == nullptr) {
        _last_error = MAX30001_ERROR_INVALID_PARAMETER;
        return _last_error;
    }

    if (!_initialized) {
        _last_error = MAX30001_ERROR_NOT_INITIALIZED;
        return _last_error;
    }

    _readStatus();

    max30001_status_t latched;
    latched.all = _status_latched;
    _status_latched = 0;

    flags->ecg_fifo_overflow = latched.bit.eovf;
    flags->bioz_fifo_overflow = latched.bit.bovf;
    flags->ecg_lead_off = latched.bit.dcloffint;
    flags->bioz_over_range = latched.bit.bover;
    flags->bioz_under_range = latched.bit.bundr;
    flags->bioz_drive_off = latched.bit.bcgmon;
    flags->bioz_drive_p_off = latched.bit.bcgmp;
    flags->bioz_drive_n_off = latched.bit.bcgmn;
    flags->fast_recovery = latched.bit.fstint;
    flags->lead_on = latched.bit.lonint;
    flags->pll_unlocked = latched.bit.pllint;

    _last_error = MAX30001_SUCCESS;
    return MAX30001_SUCCESS;
}

void MAX30001::synchronize()
{
    _max30001Synch();
}

static const __FlashStringHelper* registerName(uint8_t reg)
{
    switch (reg) {
        case MAX30001_REG_STATUS:     return F("STATUS");
        case MAX30001_REG_EN_INT:     return F("EN_INT");
        case MAX30001_REG_EN_INT2:    return F("EN_INT2");
        case MAX30001_REG_MNGR_INT:   return F("MNGR_INT");
        case MAX30001_REG_MNGR_DYN:   return F("MNGR_DYN");
        case MAX30001_REG_INFO:       return F("INFO");
        case MAX30001_REG_CNFG_GEN:   return F("CNFG_GEN");
        case MAX30001_REG_CNFG_CAL:   return F("CNFG_CAL");
        case MAX30001_REG_CNFG_EMUX:  return F("CNFG_EMUX");
        case MAX30001_REG_CNFG_ECG:   return F("CNFG_ECG");
        case MAX30001_REG_CNFG_BMUX:  return F("CNFG_BMUX");
        case MAX30001_REG_CNFG_BIOZ:  return F("CNFG_BIOZ");
        case MAX30001_REG_CNFG_BIOZ_LC: return F("CNFG_BIOZ_LC");
        case MAX30001_REG_CNFG_RTOR1: return F("CNFG_RTOR1");
        case MAX30001_REG_CNFG_RTOR2: return F("CNFG_RTOR2");
        default:                      return F("?");
    }
}

void MAX30001::printRegisters(Stream& out)
{
    static const uint8_t regs[] = {
        MAX30001_REG_STATUS, MAX30001_REG_EN_INT, MAX30001_REG_EN_INT2, MAX30001_REG_MNGR_INT,
        MAX30001_REG_MNGR_DYN, MAX30001_REG_INFO, MAX30001_REG_CNFG_GEN, MAX30001_REG_CNFG_CAL,
        MAX30001_REG_CNFG_EMUX, MAX30001_REG_CNFG_ECG, MAX30001_REG_CNFG_BMUX, MAX30001_REG_CNFG_BIOZ,
        MAX30001_REG_CNFG_BIOZ_LC, MAX30001_REG_CNFG_RTOR1, MAX30001_REG_CNFG_RTOR2,
    };

    for (uint8_t i = 0; i < sizeof(regs); i++) {
        uint32_t value;
        if (regs[i] == MAX30001_REG_STATUS) {
            _readStatus();
            value = global_status.all;
        } else {
            _max30001RegRead24(regs[i], &value);
        }

        out.print(F("0x"));
        if (regs[i] < 0x10) out.print('0');
        out.print(regs[i], HEX);
        out.print(' ');
        out.print(registerName(regs[i]));
        out.print(F(" = 0x"));
        for (int8_t shift = 20; shift >= 0; shift -= 4) {
            out.print((value >> shift) & 0x0F, HEX);
        }
        out.println();
    }
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
    // Reading STATUS clears the latched flags, so keep them for getStatus()
    _max30001RegRead24(MAX30001_REG_STATUS, &global_status.all);
    _status_latched |= global_status.all;
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
