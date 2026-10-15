// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: Copyright (c) 2025 Ashwin Whitchurch, Protocentral Electronics
// SPDX-FileCopyrightText: Copyright (c) 2016 Maxim Integrated Products, Inc. (register definitions)

/*
 * MAX30001 Single-Lead ECG Breakout Board - Arduino Library
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
 * This library incorporates register definitions and design patterns from
 * Maxim Integrated Products, Inc., used with permission.
 */

/*******************************************************************************
 * Copyright (C) 2016 Maxim Integrated Products, Inc., All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
 * OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * Except as contained in this notice, the name of Maxim Integrated
 * Products, Inc. shall not be used except as stated in the Maxim Integrated
 * Products, Inc. Branding Policy.
 *
 * The mere transfer of this software does not imply any licenses
 * of trade secrets, proprietary technology, copyrights, patents,
 * trademarks, maskwork rights, or any other form of intellectual
 * property whatsoever. Maxim Integrated Products, Inc. retains all
 * ownership rights.
 *******************************************************************************/

#ifndef protocentral_max30001_h
#define protocentral_max30001_h

#include <Arduino.h>
#include <SPI.h>

#define MAX30001_LIBRARY_VERSION "2.1.0"

// Marks the 1.x-style API; it still works but will be removed in 3.0
#if defined(__GNUC__)
#define MAX30001_DEPRECATED(msg) __attribute__((deprecated(msg)))
#else
#define MAX30001_DEPRECATED(msg)
#endif

// =============================================================================
// SPI Configuration
// =============================================================================
#define MAX30001_SPI_SPEED 1000000
#define MAX30001_SPI_WRITE 0x00
#define MAX30001_SPI_READ 0x01

// FIFO depths (samples)
#define MAX30001_ECG_FIFO_DEPTH 32
#define MAX30001_BIOZ_FIFO_DEPTH 8

// =============================================================================
// Register Addresses
// =============================================================================
#define MAX30001_REG_STATUS          0x01
#define MAX30001_REG_EN_INT          0x02
#define MAX30001_REG_EN_INT2         0x03
#define MAX30001_REG_MNGR_INT        0x04
#define MAX30001_REG_MNGR_DYN        0x05
#define MAX30001_REG_SW_RST          0x08
#define MAX30001_REG_SYNCH           0x09
#define MAX30001_REG_FIFO_RST        0x0A
#define MAX30001_REG_INFO            0x0F
#define MAX30001_REG_CNFG_GEN        0x10
#define MAX30001_REG_CNFG_CAL        0x12
#define MAX30001_REG_CNFG_EMUX       0x14
#define MAX30001_REG_CNFG_ECG        0x15
#define MAX30001_REG_CNFG_BMUX       0x17
#define MAX30001_REG_CNFG_BIOZ       0x18
#define MAX30001_REG_CNFG_BIOZ_LC    0x1A
#define MAX30001_REG_CNFG_RTOR1      0x1D
#define MAX30001_REG_CNFG_RTOR2      0x1E
#define MAX30001_REG_ECG_FIFO_BURST  0x20
#define MAX30001_REG_ECG_FIFO        0x21
#define MAX30001_REG_BIOZ_FIFO_BURST 0x22
#define MAX30001_REG_BIOZ_FIFO       0x23
#define MAX30001_REG_RTOR            0x25
#define MAX30001_REG_NO_OP           0x7F

// EN_INT bits used by the driver
#define MAX30001_EN_INT_EINT 0x800000UL
#define MAX30001_EN_INT_BINT 0x080000UL
#define MAX30001_EN_INT_LONINT 0x000800UL
#define MAX30001_INTB_TYPE_CMOS 0x000001UL

// =============================================================================
// Legacy names (1.x/2.0). Unprefixed names such as STATUS and INFO can clash
// with other libraries: define MAX30001_NO_LEGACY_NAMES before including this
// header to leave them out. They will be removed in 3.0.
// =============================================================================
#ifndef MAX30001_NO_LEGACY_NAMES
#define WREG MAX30001_SPI_WRITE
#define RREG MAX30001_SPI_READ

#define STATUS MAX30001_REG_STATUS
#define EN_INT MAX30001_REG_EN_INT
#define EN_INT2 MAX30001_REG_EN_INT2
#define MNGR_INT MAX30001_REG_MNGR_INT
#define MNGR_DYN MAX30001_REG_MNGR_DYN
#define SW_RST MAX30001_REG_SW_RST
#define SYNCH MAX30001_REG_SYNCH
#define FIFO_RST MAX30001_REG_FIFO_RST
#define INFO MAX30001_REG_INFO
#define CNFG_GEN MAX30001_REG_CNFG_GEN
#define CNFG_CAL MAX30001_REG_CNFG_CAL
#define CNFG_EMUX MAX30001_REG_CNFG_EMUX
#define CNFG_ECG MAX30001_REG_CNFG_ECG
#define CNFG_BMUX MAX30001_REG_CNFG_BMUX
#define CNFG_BIOZ MAX30001_REG_CNFG_BIOZ
#define CNFG_BIOZ_LC MAX30001_REG_CNFG_BIOZ_LC
#define CNFG_RTOR1 MAX30001_REG_CNFG_RTOR1
#define CNFG_RTOR2 MAX30001_REG_CNFG_RTOR2
#define ECG_FIFO_BURST MAX30001_REG_ECG_FIFO_BURST
#define ECG_FIFO MAX30001_REG_ECG_FIFO
#define BIOZ_FIFO_BURST MAX30001_REG_BIOZ_FIFO_BURST
#define BIOZ_FIFO MAX30001_REG_BIOZ_FIFO
#define RTOR MAX30001_REG_RTOR
#define NO_OP MAX30001_REG_NO_OP

#define CLK_PIN 6
#define RTOR_INTR_MASK 0x04

enum EN_INT_bits
{
  EN_EINT = 0x800000,
  EN_ECG_FIFO_OVF = 0x400000,
  EN_ECG_FAST_REC = 0x200000,
  EN_DCLOFFINT = 0x100000,
  EN_BIOZ_FIFO_INT = 0x80000,
  EN_BIOZ_FIFO_OVF = 0x40000,
  EN_BIOZ_OVER_RANGE = 0x20000,
  EN_BIOZ_UNDER_RANGE = 0x10000,
  EN_BIOZ_CG_MON = 0x8000,

  EN_LONINT = 0x800,
  EN_RRINT = 0x400,
  EN_SAMP = 0x200,
  EN_PLLINT = 0x100,

  EN_BCGMP = 0x20,
  EN_BCGMN = 0x10,
  EN_LDOFF_PH = 0x8,
  EN_LDOFF_PL = 0x4,
  EN_LDOFF_NH = 0x2,
  EN_LDOFF_NL = 0x1
};

typedef enum
{
  SAMPLINGRATE_128 = 128,
  SAMPLINGRATE_256 = 256,
  SAMPLINGRATE_512 = 512
} sampRate;
#endif // MAX30001_NO_LEGACY_NAMES

// =============================================================================
// New API: Error Codes and Type Definitions
// =============================================================================

/**
 * @brief Error codes returned by MAX30001 functions
 */
typedef enum {
    MAX30001_SUCCESS = 0,                    ///< Operation completed successfully
    MAX30001_ERROR_INVALID_PARAMETER,        ///< Invalid parameter provided
    MAX30001_ERROR_NOT_INITIALIZED,          ///< Device not initialized
    MAX30001_ERROR_SPI_COMMUNICATION,        ///< SPI communication error
    MAX30001_ERROR_DEVICE_NOT_FOUND,         ///< Device not responding
    MAX30001_ERROR_FIFO_OVERFLOW,            ///< FIFO overflow occurred
    MAX30001_ERROR_NOT_READY                 ///< Data not ready (FIFO empty or channel not running)
} max30001_error_t;

/**
 * @brief Channel identifiers
 */
typedef enum {
    MAX30001_CHANNEL_ECG = 0,
    MAX30001_CHANNEL_BIOZ = 1
} max30001_channel_t;

/**
 * @brief Sample rate options
 */
typedef enum {
    MAX30001_RATE_128 = 128,   ///< 128 samples per second
    MAX30001_RATE_256 = 256,   ///< 256 samples per second
    MAX30001_RATE_512 = 512    ///< 512 samples per second
} max30001_sample_rate_t;

/**
 * @brief ECG gain options
 */
typedef enum {
    MAX30001_ECG_GAIN_20 = 0b00,   ///< 20 V/V
    MAX30001_ECG_GAIN_40 = 0b01,   ///< 40 V/V
    MAX30001_ECG_GAIN_80 = 0b10,   ///< 80 V/V
    MAX30001_ECG_GAIN_160 = 0b11   ///< 160 V/V
} max30001_ecg_gain_t;

/**
 * @brief BioZ gain options
 */
typedef enum {
    MAX30001_BIOZ_GAIN_10 = 0b00,  ///< 10 V/V
    MAX30001_BIOZ_GAIN_20 = 0b01,  ///< 20 V/V
    MAX30001_BIOZ_GAIN_40 = 0b10,  ///< 40 V/V
    MAX30001_BIOZ_GAIN_80 = 0b11   ///< 80 V/V
} max30001_bioz_gain_t;

/**
 * @brief ECG sample data structure
 */
typedef struct {
    int32_t ecg_sample;           ///< Raw ECG sample value
    uint32_t timestamp_ms;        ///< Sample timestamp in milliseconds
    bool lead_off_detected;       ///< Lead-off detection flag
    bool sample_valid;            ///< Sample validity flag
} max30001_ecg_sample_t;

/**
 * @brief BioZ sample data structure
 */
typedef struct {
    int32_t bioz_sample;          ///< Raw BioZ sample value
    uint32_t timestamp_ms;        ///< Sample timestamp in milliseconds
    bool sample_valid;            ///< Sample validity flag
    bool out_of_range;            ///< BioZ over/under range (input saturated)
} max30001_bioz_sample_t;

/**
 * @brief R-R detection data structure
 */
typedef struct {
    uint16_t heart_rate_bpm;      ///< Heart rate in BPM (from the last detected beat)
    uint16_t rr_interval_ms;      ///< R-R interval in milliseconds (from the last detected beat)
    bool rr_detected;             ///< true only when a new beat was detected since the last call
} max30001_rtor_data_t;

/**
 * @brief DC lead-off status (comparator thresholds VMID +/- 300 mV)
 */
typedef struct {
    bool any;                     ///< Lead-off on either electrode (DCLOFFINT)
    bool p_high;                  ///< ECGP above the high threshold
    bool p_low;                   ///< ECGP below the low threshold
    bool n_high;                  ///< ECGN above the high threshold
    bool n_low;                   ///< ECGN below the low threshold
} max30001_leadoff_t;

/**
 * @brief Device information structure
 */
typedef struct {
    uint8_t part_id;              ///< Part ID
    uint8_t revision;             ///< Chip revision
    bool device_found;            ///< Device detection flag
} max30001_device_info_t;

/**
 * @brief BioZ drive current (peak). Higher currents need higher frequencies:
 *        96 uA needs >= 18 kHz, 32 uA >= 4 kHz, 16 uA >= 2 kHz, 8 uA below that
 */
typedef enum {
    MAX30001_BIOZ_CURRENT_8UA = 1,
    MAX30001_BIOZ_CURRENT_16UA = 2,
    MAX30001_BIOZ_CURRENT_32UA = 3,
    MAX30001_BIOZ_CURRENT_48UA = 4,
    MAX30001_BIOZ_CURRENT_64UA = 5,
    MAX30001_BIOZ_CURRENT_80UA = 6,
    MAX30001_BIOZ_CURRENT_96UA = 7
} max30001_bioz_current_t;

/**
 * @brief BioZ drive frequency (FCGEN). Below ~500 Hz the board's 47 nF drive
 *        capacitors limit the current; the datasheet asks for 220 nF at 125 Hz
 */
typedef enum {
    MAX30001_BIOZ_FREQ_128KHZ = 0,   ///< 131072 Hz
    MAX30001_BIOZ_FREQ_80KHZ = 1,    ///< 81920 Hz
    MAX30001_BIOZ_FREQ_40KHZ = 2,    ///< 40960 Hz
    MAX30001_BIOZ_FREQ_18KHZ = 3,    ///< 18204 Hz
    MAX30001_BIOZ_FREQ_8KHZ = 4,     ///< 8192 Hz (default)
    MAX30001_BIOZ_FREQ_4KHZ = 5,     ///< 4096 Hz
    MAX30001_BIOZ_FREQ_2KHZ = 6,     ///< 2048 Hz
    MAX30001_BIOZ_FREQ_1KHZ = 7,     ///< 1024 Hz
    MAX30001_BIOZ_FREQ_500HZ = 8,    ///< 512 Hz
    MAX30001_BIOZ_FREQ_250HZ = 9,    ///< 256 Hz
    MAX30001_BIOZ_FREQ_125HZ = 10    ///< 128 Hz
} max30001_bioz_freq_t;

/**
 * @brief BioZ analog high-pass filter (before demodulation)
 */
typedef enum {
    MAX30001_BIOZ_AHPF_60HZ = 0,
    MAX30001_BIOZ_AHPF_150HZ = 1,
    MAX30001_BIOZ_AHPF_500HZ = 2,    ///< Default
    MAX30001_BIOZ_AHPF_1000HZ = 3,
    MAX30001_BIOZ_AHPF_2000HZ = 4,
    MAX30001_BIOZ_AHPF_4000HZ = 5,
    MAX30001_BIOZ_AHPF_BYPASS = 6
} max30001_bioz_ahpf_t;

/**
 * @brief Built-in BioZ test resistor: nominal value
 */
typedef enum {
    MAX30001_BIOZ_RNOM_5000 = 0,     ///< 5000 Ohm
    MAX30001_BIOZ_RNOM_2500 = 1,     ///< 2500 Ohm
    MAX30001_BIOZ_RNOM_1667 = 2,     ///< 1667 Ohm
    MAX30001_BIOZ_RNOM_1250 = 3,     ///< 1250 Ohm
    MAX30001_BIOZ_RNOM_1000 = 4,     ///< 1000 Ohm
    MAX30001_BIOZ_RNOM_833 = 5,      ///< 833 Ohm
    MAX30001_BIOZ_RNOM_714 = 6,      ///< 714 Ohm
    MAX30001_BIOZ_RNOM_625 = 7       ///< 625 Ohm
} max30001_bioz_rnom_t;

/**
 * @brief Built-in BioZ test resistor: modulation step (see bioZTestLoadModulationOhms())
 */
typedef enum {
    MAX30001_BIOZ_RMOD_0 = 0,        ///< Largest step
    MAX30001_BIOZ_RMOD_1 = 1,
    MAX30001_BIOZ_RMOD_2 = 2,        ///< Smallest step; only with RNOM 5000, 2500 or 1667
    MAX30001_BIOZ_RMOD_NONE = 4      ///< Unmodulated (DC = RNOM)
} max30001_bioz_rmod_t;

/**
 * @brief Built-in BioZ test resistor modulation frequency
 */
typedef enum {
    MAX30001_BIOZ_TEST_FREQ_4HZ = 0,
    MAX30001_BIOZ_TEST_FREQ_1HZ = 1,
    MAX30001_BIOZ_TEST_FREQ_0_25HZ = 2,
    MAX30001_BIOZ_TEST_FREQ_0_0625HZ = 3
} max30001_bioz_test_freq_t;

/**
 * @brief ECG calibration (test) signal frequency
 */
typedef enum {
    MAX30001_CAL_FREQ_256HZ = 0,
    MAX30001_CAL_FREQ_64HZ = 1,
    MAX30001_CAL_FREQ_16HZ = 2,
    MAX30001_CAL_FREQ_4HZ = 3,
    MAX30001_CAL_FREQ_1HZ = 4,
    MAX30001_CAL_FREQ_0_25HZ = 5,
    MAX30001_CAL_FREQ_0_0625HZ = 6,
    MAX30001_CAL_FREQ_0_015HZ = 7
} max30001_cal_freq_t;

/**
 * @brief DC lead-off current
 */
typedef enum {
    MAX30001_LEADOFF_CURRENT_5NA = 1,
    MAX30001_LEADOFF_CURRENT_10NA = 2,   ///< Default
    MAX30001_LEADOFF_CURRENT_20NA = 3,
    MAX30001_LEADOFF_CURRENT_50NA = 4,
    MAX30001_LEADOFF_CURRENT_100NA = 5
} max30001_leadoff_current_t;

/**
 * @brief DC lead-off comparator threshold (VMID +/- value)
 */
typedef enum {
    MAX30001_LEADOFF_THRESHOLD_300MV = 0,   ///< Default
    MAX30001_LEADOFF_THRESHOLD_400MV = 1,
    MAX30001_LEADOFF_THRESHOLD_450MV = 2,
    MAX30001_LEADOFF_THRESHOLD_500MV = 3
} max30001_leadoff_threshold_t;

/**
 * @brief ECG fast recovery (high-pass bypass after saturation)
 */
typedef enum {
    MAX30001_FAST_RECOVERY_OFF = 0,      ///< Default
    MAX30001_FAST_RECOVERY_MANUAL = 1,   ///< On until switched off
    MAX30001_FAST_RECOVERY_AUTO = 2      ///< 500 ms after the ECG output stays past the threshold for 125 ms
} max30001_fast_recovery_t;

/**
 * @brief Internal resistive lead bias. The breakout biases the body through
 *        the RL electrode, so this is only needed when RL is not used
 */
typedef enum {
    MAX30001_LEAD_BIAS_OFF = 0,          ///< Default
    MAX30001_LEAD_BIAS_ECG = 1,          ///< On ECGP/ECGN
    MAX30001_LEAD_BIAS_BIOZ = 2          ///< On BIP/BIN
} max30001_lead_bias_t;

typedef enum {
    MAX30001_LEAD_BIAS_50MOHM = 0,
    MAX30001_LEAD_BIAS_100MOHM = 1,      ///< Default
    MAX30001_LEAD_BIAS_200MOHM = 2
} max30001_lead_bias_resistance_t;

/**
 * @brief R-to-R detector settings, as register codes (datasheet CNFG_RTOR1/2).
 *        The defaults are the datasheet defaults
 */
typedef struct {
    uint8_t window = 3;           ///< WNDW: averaging window (WNDW*2+6) x 8 ms, 0-11 (default 96 ms)
    uint8_t gain = 15;            ///< RGAIN: 2^gain, 15 = auto-scale
    uint8_t peak_average = 2;     ///< PAVG: 0-3 = 2, 4, 8, 16
    uint8_t peak_threshold = 3;   ///< PTSF: threshold = (code+1)/16 of the peak average
    uint8_t holdoff_min = 32;     ///< HOFF: minimum hold-off, code x 8 ms (0-63)
    uint8_t interval_average = 2; ///< RAVG: 0-3 = 2, 4, 8, 16
    uint8_t holdoff_scale = 4;    ///< RHSF: dynamic hold-off = code/8 of the average interval (0-7)
} max30001_rtor_config_t;

/**
 * @brief Status flags, latched since the last getStatus() call
 */
typedef struct {
    bool ecg_fifo_overflow;       ///< EOVF
    bool bioz_fifo_overflow;      ///< BOVF
    bool ecg_lead_off;            ///< DCLOFFINT
    bool bioz_over_range;         ///< BOVER (needs setBioZLeadOff())
    bool bioz_under_range;        ///< BUNDR (needs setBioZLeadOff())
    bool bioz_drive_off;          ///< BCGMON (needs setBioZCurrentMonitor())
    bool bioz_drive_p_off;        ///< BCGMP
    bool bioz_drive_n_off;        ///< BCGMN
    bool fast_recovery;           ///< FSTINT
    bool lead_on;                 ///< LONINT (lead-on detection mode)
    bool pll_unlocked;            ///< PLLINT
} max30001_status_flags_t;

typedef union max30001_status_reg
{
  uint32_t all;

  struct
  {
    uint32_t loff_nl : 1;
    uint32_t loff_nh : 1;
    uint32_t loff_pl : 1;
    uint32_t loff_ph : 1;

    uint32_t bcgmn : 1;
    uint32_t bcgmp : 1;
    uint32_t reserved1 : 1;
    uint32_t reserved2 : 1;

    uint32_t pllint : 1;
    uint32_t samp : 1;
    uint32_t rrint : 1;
    uint32_t lonint : 1;

    uint32_t pedge : 1;
    uint32_t povf : 1;
    uint32_t pint : 1;
    uint32_t bcgmon : 1;

    uint32_t bundr : 1;
    uint32_t bover : 1;
    uint32_t bovf : 1;
    uint32_t bint : 1;

    uint32_t dcloffint : 1;
    uint32_t fstint : 1;
    uint32_t eovf : 1;
    uint32_t eint : 1;

    uint32_t reserved : 8;

  } bit;

} max30001_status_t;

/**
 * @brief CNFG_GEN (0x10)
 */
typedef union max30001_cnfg_gen_reg
{
  uint32_t all;
  struct
  {
    uint32_t rbiasn : 1;
    uint32_t rbiasp : 1;
    uint32_t rbiasv : 2;
    uint32_t en_rbias : 2;
    uint32_t vth : 2;
    uint32_t imag : 3;
    uint32_t ipol : 1;
    uint32_t en_dcloff : 2;
    uint32_t en_bloff : 2;
    uint32_t reserved1 : 1;
    uint32_t en_pace : 1;
    uint32_t en_bioz : 1;
    uint32_t en_ecg : 1;
    uint32_t fmstr : 2;
    uint32_t en_ulp_lon : 2;
    uint32_t reserved : 8;
  } bit;

} max30001_cnfg_gen_t;

/**
 * @brief MNGR_INT (0x04)
 */
typedef union max30001_mngr_int_reg
{
  uint32_t all;

  struct
  {
    uint32_t samp_it : 2;
    uint32_t clr_samp : 1;
    uint32_t clr_pedge : 1;
    uint32_t clr_rrint : 2;
    uint32_t clr_fast : 1;
    uint32_t reserved1 : 1;
    uint32_t reserved2 : 4;
    uint32_t reserved3 : 4;

    uint32_t b_fit : 3;
    uint32_t e_fit : 5;

    uint32_t reserved : 8;

  } bit;

} max30001_mngr_int_t;

/**
 * @brief CNFG_EMUX  (0x14)
 */
typedef union max30001_cnfg_emux_reg
{
  uint32_t all;
  struct
  {
    uint32_t reserved1 : 16;
    uint32_t caln_sel : 2;
    uint32_t calp_sel : 2;
    uint32_t openn : 1;
    uint32_t openp : 1;
    uint32_t reserved2 : 1;
    uint32_t pol : 1;
    uint32_t reserved : 8;
  } bit;

} max30001_cnfg_emux_t;

/**
 * @brief CNFG_ECG   (0x15)
 */
typedef union max30001_cnfg_ecg_reg
{
  uint32_t all;
  struct
  {
    uint32_t reserved1 : 12;
    uint32_t dlpf : 2;
    uint32_t dhpf : 1;
    uint32_t reserved2 : 1;
    uint32_t gain : 2;
    uint32_t reserved3 : 4;
    uint32_t rate : 2;

    uint32_t reserved : 8;
  } bit;

} max30001_cnfg_ecg_t;

/**
 * @brief CNFG_BMUX   (0x17)
 */
typedef union max30001_cnfg_bmux_reg
{
  uint32_t all;
  struct
  {
    uint32_t fbist : 2;
    uint32_t reserved1 : 2;
    uint32_t rmod : 3;
    uint32_t reserved2 : 1;
    uint32_t rnom : 3;
    uint32_t en_bist : 1;
    uint32_t cg_mode : 2;
    uint32_t reserved3 : 2;
    uint32_t caln_sel : 2;
    uint32_t calp_sel : 2;
    uint32_t openn : 1;
    uint32_t openp : 1;
    uint32_t reserved4 : 2;
    uint32_t reserved : 8;
  } bit;

} max30001_cnfg_bmux_t;

/**
 * @brief CNFG_BIOZ   (0x18)
 */
typedef union max30001_bioz_reg
{
  uint32_t all;
  struct
  {
    uint32_t phoff : 4;
    uint32_t cgmag : 3;
    uint32_t cgmon : 1;
    uint32_t fcgen : 4;
    uint32_t dlpf : 2;
    uint32_t dhpf : 2;
    uint32_t gain : 2;
    uint32_t ln_bioz : 1;
    uint32_t ext_rbias : 1;
    uint32_t ahpf : 3;
    uint32_t rate : 1;
    uint32_t reserved : 8;
  } bit;

} max30001_cnfg_bioz_t;

/**
 * @brief CNFG_RTOR1   (0x1D)
 */
typedef union max30001_cnfg_rtor1_reg
{
  uint32_t all;
  struct
  {
    uint32_t reserved1 : 8;
    uint32_t ptsf : 4;
    uint32_t pavg : 2;
    uint32_t reserved2 : 1;
    uint32_t en_rtor : 1;
    uint32_t gain : 4;
    uint32_t wndw : 4;
    uint32_t reserved : 8;
  } bit;

} max30001_cnfg_rtor1_t;

/**
 * @brief CNFG_RTOR2 (0x1E)
 */
typedef union max30001_cnfg_rtor2_reg
{
  uint32_t all;
  struct
  {
    uint32_t reserved1 : 8;
    uint32_t rhsf : 3;
    uint32_t reserved2 : 1;
    uint32_t ravg : 2;
    uint32_t reserved3 : 2;
    uint32_t hoff : 6;
    uint32_t reserved4 : 2;
    uint32_t reserved : 8;
  } bit;

} max30001_cnfg_rtor2_t;

// =============================================================================
// MAX30001 Class Declaration
// =============================================================================

/**
 * @brief Main class for interfacing with the MAX30001 ECG/BioZ sensor
 * 
 * This class provides both a modern high-level API and maintains backward
 * compatibility with the legacy interface.
 */
class MAX30001
{
public:
  // =========================================================================
  // Constructors and Initialization
  // =========================================================================
  
  /**
   * @brief Constructor with chip select pin
   * @param cs_pin SPI chip select pin number
   * @param spi_interface SPI interface to use (default: SPI)
   */
  MAX30001(uint8_t cs_pin, SPIClass* spi_interface = &SPI);
  
  /**
   * @brief Legacy constructor (for backward compatibility)
   * @param cs_pin SPI chip select pin number
   */
  MAX30001(int cs_pin);
  
  /**
   * @brief Initialize the MAX30001 sensor
   * @return Error code indicating success or failure
   */
  max30001_error_t begin();
  
  /**
   * @brief Check if device is connected and responding
   * @return true if device responding, false otherwise
   */
  bool isConnected();
  
  /**
   * @brief Get chip information
   * @param info Pointer to store device info
   * @return Error code
   */
  max30001_error_t getDeviceInfo(max30001_device_info_t* info);
  
  // =========================================================================
  // High-Level Measurement API (New Interface)
  // =========================================================================
  
  /**
   * @brief Start ECG monitoring with specified settings
   * @param sample_rate Desired sample rate (default: 128 SPS)
   * @param gain ECG gain setting (default: 80 V/V)
   * @return Error code
   */
  max30001_error_t startECG(max30001_sample_rate_t sample_rate = MAX30001_RATE_128,
                            max30001_ecg_gain_t gain = MAX30001_ECG_GAIN_80);
  
  /**
   * @brief Start BioZ monitoring with specified settings
   * @param sample_rate Desired sample rate (default: 128 SPS)
   * @return Error code
   */
  max30001_error_t startBioZ(max30001_sample_rate_t sample_rate = MAX30001_RATE_128);
  
  /**
   * @brief Start combined ECG + BioZ monitoring
   * @param sample_rate ECG sample rate (BioZ always runs at 64 SPS)
   * @param gain ECG gain setting (default: 80 V/V)
   * @return Error code
   */
  max30001_error_t startECGBioZ(max30001_sample_rate_t sample_rate = MAX30001_RATE_128,
                                max30001_ecg_gain_t gain = MAX30001_ECG_GAIN_80);
  
  /**
   * @brief Start ECG with R-R interval detection
   * @param sample_rate Desired sample rate (default: 128 SPS)
   * @param gain ECG gain setting (default: 80 V/V)
   * @return Error code (same as startECG; R-R detection is enabled with every ECG start)
   */
  max30001_error_t startRtoR(max30001_sample_rate_t sample_rate = MAX30001_RATE_128,
                             max30001_ecg_gain_t gain = MAX30001_ECG_GAIN_80);
  
  /**
   * @brief Read the next ECG sample from the FIFO
   * @param sample Pointer to store ECG sample data
   * @return MAX30001_SUCCESS, MAX30001_ERROR_NOT_READY if the FIFO is empty,
   *         or MAX30001_ERROR_FIFO_OVERFLOW (the FIFO is reset)
   */
  max30001_error_t getECGSample(max30001_ecg_sample_t* sample);
  
  /**
   * @brief Read the next BioZ sample from the FIFO (64 SPS)
   * @param sample Pointer to store BioZ sample data
   * @return MAX30001_SUCCESS, MAX30001_ERROR_NOT_READY if the FIFO is empty,
   *         or MAX30001_ERROR_FIFO_OVERFLOW (the FIFO is reset)
   */
  max30001_error_t getBioZSample(max30001_bioz_sample_t* sample);
  
  /**
   * @brief Get R-R detection data
   * @param rtor_data Pointer to store R-R data; rr_detected is true only for a new beat
   * @return Error code
   */
  max30001_error_t getRtoRData(max30001_rtor_data_t* rtor_data);
  
  /**
   * @brief Stop all monitoring
   */
  void stop();
  
  /**
   * @brief Convert raw ECG value to microvolts
   * @param raw_value Raw ADC value
   * @param gain Current gain setting
   * @return Value in microvolts
   */
  float convertECGToMicrovolts(int32_t raw_value, max30001_ecg_gain_t gain);
  
  /**
   * @brief Get last error code
   * @return Last error that occurred
   */
  max30001_error_t getLastError() const;
  
  /**
   * @brief Library version string, e.g. "2.1.0"
   */
  static const char* getLibraryVersion() { return MAX30001_LIBRARY_VERSION; }
  
  // =========================================================================
  // Configuration
  // =========================================================================
  
  /**
   * @brief Set ECG gain during runtime
   * @param gain New gain setting (20, 40, 80 or 160 V/V)
   * @return Error code
   */
  max30001_error_t setECGGain(max30001_ecg_gain_t gain);
  
  /**
   * @brief Get current ECG gain setting
   * @return Current ECG gain
   */
  max30001_ecg_gain_t getECGGain() const { return _ecg_gain; }
  
  /**
   * @brief Enable ECG channel (resume acquisition)
   * @return Error code
   */
  max30001_error_t enableECG();
  
  /**
   * @brief Disable ECG channel (pause acquisition)
   * @return Error code
   */
  max30001_error_t disableECG();
  
  /**
   * @brief Check if ECG channel is enabled
   * @return true if ECG is acquiring, false otherwise
   */
  bool isECGEnabled() const { return _ecg_enabled; }
  
  /**
   * @brief Enable BioZ channel (resume acquisition)
   * @return Error code
   */
  max30001_error_t enableBioZ();
  
  /**
   * @brief Disable BioZ channel (pause acquisition)
   * @return Error code
   */
  max30001_error_t disableBioZ();
  
  /**
   * @brief Check if BioZ channel is enabled
   * @return true if BioZ is acquiring, false otherwise
   */
  bool isBioZEnabled() const { return _bioz_enabled; }
  
  /**
   * @brief Set ECG high-pass filter
   * @param cutoff_hz 0 = bypass (DC coupled); any value > 0 selects 0.5 Hz, the only option
   * @return Error code
   */
  max30001_error_t setECGHighPassFilter(float cutoff_hz);
  
  /**
   * @brief Set ECG low-pass filter
   * @param cutoff_hz 0 = bypass, otherwise the nearest of 40, 100 or 150 Hz.
   *        100 Hz needs >= 256 SPS and 150 Hz needs 512 SPS; at 128 SPS the
   *        filter is 28.35 Hz for any value > 0
   * @return Error code
   */
  max30001_error_t setECGLowPassFilter(float cutoff_hz);
  
  /**
   * @brief Get DC lead-off status (enabled with every ECG start, 10 nA)
   * @return true if lead-off detected on either electrode, false if both connected
   */
  bool getLeadOffStatus();
  
  /**
   * @brief Get per-electrode DC lead-off status
   * @param leadoff Pointer to store the status
   * @return Error code
   */
  max30001_error_t getLeadOffDetail(max30001_leadoff_t* leadoff);
  
  /**
   * @brief Number of ECG samples decoded by the last burst read (max30001ServiceAllInterrupts)
   * @note The MAX30001 has no FIFO count register. With getECGSample(), poll until
   *       it returns MAX30001_ERROR_NOT_READY instead
   * @return Sample count
   */
  uint8_t getFIFOCount();
  
  /**
   * @brief Clear/flush the ECG and BioZ FIFOs
   * @return Error code
   */
  max30001_error_t clearFIFO();
  
  /**
   * @brief Enable the INTB output (CMOS, active low)
   *
   * INTB goes low while the ECG FIFO holds >= 16 samples, or the BioZ FIFO
   * >= 4 samples when BioZ is running. Call after a start method, and drain
   * both FIFOs on each interrupt so INTB returns high.
   * @return Error code
   */
  max30001_error_t enableInterrupt();
  
  /**
   * @brief Disable all INTB interrupt sources
   * @return Error code
   */
  max30001_error_t disableInterrupt();
  
  /**
   * @brief Invert the ECG input polarity
   * @param inverted true swaps ECGP and ECGN
   * @return Error code
   */
  max30001_error_t setECGPolarity(bool inverted);
  
  /**
   * @brief Configure DC lead-off detection (applied now and on every ECG start)
   * @param enable false turns the lead-off current sources off
   * @param current Lead-off current
   * @param threshold Comparator threshold
   * @return Error code
   */
  max30001_error_t setLeadOffDetection(bool enable,
                                       max30001_leadoff_current_t current = MAX30001_LEADOFF_CURRENT_10NA,
                                       max30001_leadoff_threshold_t threshold = MAX30001_LEADOFF_THRESHOLD_300MV);
  
  /**
   * @brief Configure the R-to-R detector (applied now and on every ECG start)
   * @param config Detector settings; a default-constructed struct holds the datasheet defaults
   * @return Error code
   */
  max30001_error_t setRtoRConfig(const max30001_rtor_config_t& config);
  
  /**
   * @brief Configure ECG fast recovery
   * @param mode Off, manual or automatic
   * @param threshold Auto mode: ECG output threshold in units of 2048 counts (0-63)
   * @return Error code
   */
  max30001_error_t setFastRecovery(max30001_fast_recovery_t mode, uint8_t threshold = 0x3F);
  
  /**
   * @brief Configure the internal resistive lead bias
   * @param mode Off, on the ECG inputs, or on the BioZ inputs
   * @param resistance Bias resistance
   * @return Error code
   */
  max30001_error_t setLeadBias(max30001_lead_bias_t mode,
                               max30001_lead_bias_resistance_t resistance = MAX30001_LEAD_BIAS_100MOHM);
  
  // =========================================================================
  // BioZ Configuration
  // (settings are kept and applied on every BioZ start; setters also apply
  //  them immediately while BioZ is running)
  // =========================================================================
  
  /**
   * @brief Set BioZ gain
   * @param gain 10, 20, 40 or 80 V/V (default 10 V/V)
   * @return Error code
   */
  max30001_error_t setBioZGain(max30001_bioz_gain_t gain);
  max30001_bioz_gain_t getBioZGain() const { return _bioz_gain; }
  
  /**
   * @brief Set BioZ drive current. Keep current x (body + electrode impedance)
   *        under 90 mV peak at the BIP/BIN pins
   * @param current Drive current (default 16 uA)
   * @return MAX30001_ERROR_INVALID_PARAMETER if the current is not allowed at the drive frequency
   */
  max30001_error_t setBioZCurrent(max30001_bioz_current_t current);
  max30001_bioz_current_t getBioZCurrent() const { return _bioz_current; }
  
  /**
   * @brief Set BioZ drive frequency
   * @param frequency Drive frequency (default 8 kHz)
   * @return MAX30001_ERROR_INVALID_PARAMETER if the current drive current is not allowed at it
   */
  max30001_error_t setBioZFrequency(max30001_bioz_freq_t frequency);
  max30001_bioz_freq_t getBioZFrequency() const { return _bioz_freq; }
  
  /**
   * @brief Set the BioZ demodulation phase offset
   * @param step 0-15, 11.25 deg per step (22.5 deg at 80 kHz, 45 deg at 128 kHz; default 3)
   * @return Error code
   */
  max30001_error_t setBioZPhaseOffset(uint8_t step);
  
  /**
   * @brief Set the BioZ digital high-pass filter (4th-order)
   * @param cutoff_hz 0 = bypass (DC), < 0.25 = 0.05 Hz (default), otherwise 0.5 Hz.
   *        0.5 Hz removes most of the respiration signal
   * @return Error code
   */
  max30001_error_t setBioZHighPassFilter(float cutoff_hz);
  
  /**
   * @brief Set the BioZ digital low-pass filter
   * @param cutoff_hz 0 = bypass, otherwise the nearest of 4 (default), 8 or 16 Hz
   * @return Error code
   */
  max30001_error_t setBioZLowPassFilter(float cutoff_hz);
  
  /**
   * @brief Set the BioZ analog high-pass filter (keep it well below the drive frequency)
   * @param ahpf Cutoff (default 500 Hz)
   * @return Error code
   */
  max30001_error_t setBioZAnalogHighPass(max30001_bioz_ahpf_t ahpf);
  
  /**
   * @brief Use the external 324 kOhm bias resistor on RBIAS for the drive current
   *        (+/-10 % instead of +/-30 %). Only on boards with that resistor fitted
   * @param enable true to use the external resistor
   * @return Error code
   */
  max30001_error_t setBioZExternalBias(bool enable);
  
  /**
   * @brief Enable BioZ over/under-range (AC lead-off) detection
   * @param over_range Flag readings beyond +/- 2048 x over_threshold for 128 ms (2- and 4-electrode)
   * @param under_range Flag readings within +/- 32 x under_threshold for 128 ms (4-electrode only)
   * @return Error code
   */
  max30001_error_t setBioZLeadOff(bool over_range, bool under_range,
                                  uint8_t over_threshold = 0xFF, uint8_t under_threshold = 0xFF);
  
  /**
   * @brief Enable the drive current monitor (detects a DRVP/DRVN electrode
   *        off in 4-electrode BioZ; read with getStatus())
   * @param enable true to enable
   * @return Error code
   */
  max30001_error_t setBioZCurrentMonitor(bool enable);
  
  /**
   * @brief Convert a raw BioZ sample to ohms at the current gain and drive current
   * @param raw_value Raw BioZ sample
   * @return Impedance in ohms (internal bias: +/-30 % absolute accuracy)
   */
  float convertBioZToOhms(int32_t raw_value) const;
  
  // =========================================================================
  // Self-Test, Lead-On and Diagnostics
  // =========================================================================
  
  /**
   * @brief Disconnect the ECG inputs and apply the internal calibration square wave
   * @param frequency Square-wave frequency
   * @param half_millivolt true = +/-0.5 mV, false = +/-0.25 mV (bipolar)
   * @return Error code (ECG must be running)
   */
  max30001_error_t enableECGTestSignal(max30001_cal_freq_t frequency = MAX30001_CAL_FREQ_1HZ,
                                       bool half_millivolt = true);
  max30001_error_t disableECGTestSignal();
  
  /**
   * @brief Disconnect BIP/BIN and switch in the built-in test resistor
   *
   * The reading is RNOM plus a square-wave step. The default 0.05 Hz high-pass
   * removes RNOM itself; call setBioZHighPassFilter(0) to read it.
   * @param rnom Nominal resistance
   * @param rmod Modulation step (MAX30001_BIOZ_RMOD_NONE for DC only)
   * @param frequency Modulation frequency
   * @return Error code (BioZ must be running)
   */
  max30001_error_t enableBioZTestLoad(max30001_bioz_rnom_t rnom,
                                      max30001_bioz_rmod_t rmod = MAX30001_BIOZ_RMOD_0,
                                      max30001_bioz_test_freq_t frequency = MAX30001_BIOZ_TEST_FREQ_1HZ);
  max30001_error_t disableBioZTestLoad();
  
  /**
   * @brief Nominal value of the built-in test resistor, in ohms
   */
  static float bioZTestLoadOhms(max30001_bioz_rnom_t rnom);
  
  /**
   * @brief Modulation step of the built-in test resistor, in ohms (0 if not available)
   */
  static float bioZTestLoadModulationOhms(max30001_bioz_rnom_t rnom, max30001_bioz_rmod_t rmod);
  
  /**
   * @brief Stop both channels and wait for electrode contact in ultra-low-power mode
   *
   * Poll isLeadOn(), or enable the INTB interrupt here, then call a start method.
   * @param interrupt true drives INTB low on lead-on
   * @return Error code
   */
  max30001_error_t startLeadOnDetection(bool interrupt = false);
  
  /**
   * @brief Check for electrode contact in lead-on detection mode
   * @return true once the electrodes are on
   */
  bool isLeadOn();
  
  /**
   * @brief Read status flags latched since the last call (also those seen by
   *        other library calls), then clear them
   * @param flags Pointer to store the flags
   * @return Error code
   */
  max30001_error_t getStatus(max30001_status_flags_t* flags);
  
  /**
   * @brief Restart the ECG/BioZ timing and FIFOs. Issue on all boards together
   *        when several boards share one FCLK, or after changing BioZ settings
   */
  void synchronize();
  
  /**
   * @brief Print every configuration and status register
   * @param out Destination, e.g. Serial
   */
  void printRegisters(Stream& out);
  
  /**
   * @brief Get current sample rate
   * @return Current sample rate setting
   */
  max30001_sample_rate_t getSampleRate() const { return _sample_rate; }
  
  /**
   * @brief Get expected sample delay in milliseconds
   * @return Delay between samples (e.g., 8ms for 128 SPS)
   */
  uint16_t getSampleDelayMs() const;
  
  // =========================================================================
  // Legacy Interface (Backward Compatibility)
  // =========================================================================
  
  // Legacy public data members (deprecated - use the sample structs instead;
  // s32ECGData/s32BIOZData remain the output of max30001ServiceAllInterrupts())
  unsigned int heartRate;
  unsigned int RRinterval;
  signed long ecg_data;
  signed long bioz_data;
  volatile int ecgSamplesAvailable;
  volatile int biozSamplesAvailable;
  signed long s32ECGData[MAX30001_ECG_FIFO_DEPTH];
  signed long s32BIOZData[MAX30001_BIOZ_FIFO_DEPTH];
  
  // Legacy initialization methods
  MAX30001_DEPRECATED("use begin() + startECG()") void BeginECGOnly();
  MAX30001_DEPRECATED("use begin() + startECGBioZ()") void BeginECGBioZ();
  MAX30001_DEPRECATED("use begin() + startRtoR()") void BeginRtoRMode();
  
  // Legacy data acquisition methods
  MAX30001_DEPRECATED("use getECGSample()") signed long getECGSamples(void);
  MAX30001_DEPRECATED("use getBioZSample()") signed long getBioZSamples(void);
  MAX30001_DEPRECATED("use getRtoRData()") void getHRandRR(void);
  
  // Legacy utility methods
  MAX30001_DEPRECATED("use isConnected() / getDeviceInfo()") bool max30001ReadInfo(void);
  MAX30001_DEPRECATED("pass the rate to startECG()") void max30001SetsamplingRate(uint16_t samplingRate);
  MAX30001_DEPRECATED("use enableInterrupt()") void max30001SetInterrupts(uint32_t interrupts);
  MAX30001_DEPRECATED("use getLeadOffDetail() / getRtoRData()") void readStatus(void);
  
  /**
   * @brief Burst-read the ECG/BioZ FIFOs when their interrupt flags are set
   *
   * Results go to s32ECGData[ecgSamplesAvailable] and s32BIOZData[biozSamplesAvailable].
   */
  void max30001ServiceAllInterrupts();

private:
  // =========================================================================
  // Private Members
  // =========================================================================
  
  SPIClass* _spi;
  uint8_t _cs_pin;
  max30001_error_t _last_error;
  bool _initialized;
  max30001_ecg_gain_t _ecg_gain;
  max30001_sample_rate_t _sample_rate;
  bool _ecg_enabled;
  bool _bioz_enabled;
  
  // Settings applied by _configure() and the setters
  max30001_bioz_gain_t _bioz_gain;
  max30001_bioz_current_t _bioz_current;
  max30001_bioz_freq_t _bioz_freq;
  max30001_bioz_ahpf_t _bioz_ahpf;
  uint8_t _bioz_phoff;
  uint8_t _bioz_dhpf;
  uint8_t _bioz_dlpf;
  bool _bioz_ext_rbias;
  bool _bioz_cgmon;
  uint8_t _bloff_mode;
  uint8_t _bloff_hi;
  uint8_t _bloff_lo;
  bool _leadoff_enabled;
  max30001_leadoff_current_t _leadoff_current;
  max30001_leadoff_threshold_t _leadoff_threshold;
  max30001_rtor_config_t _rtor;
  max30001_fast_recovery_t _fast_mode;
  uint8_t _fast_threshold;
  max30001_lead_bias_t _lead_bias;
  max30001_lead_bias_resistance_t _lead_bias_res;
  bool _ecg_inverted;
  uint32_t _status_latched;
  
  max30001_status_t global_status;
  volatile unsigned char _readBufferECG[MAX30001_ECG_FIFO_DEPTH * 3];
  volatile unsigned char _readBufferBIOZ[MAX30001_BIOZ_FIFO_DEPTH * 3];
  
  // =========================================================================
  // Private Methods - Low-Level Hardware Interface
  // =========================================================================
  
  void _max30001ReadECGFIFO(int num_bytes);
  void _max30001ReadBIOZFIFO(int num_bytes);
  void _max30001Synch(void);
  void _max30001RegWrite(unsigned char WRITE_ADDRESS, unsigned long data);
  void _max30001RegRead(uint8_t Reg_address, uint8_t *buff);
  void _max30001RegRead24(uint8_t Reg_address, uint32_t *read_data);
  void _max30001SwReset(void);
  void _max30001FIFOReset(void);
  void _readStatus(void);
  void _readRtoR(void);
  
  // Register values built from the settings above
  uint32_t _genValue(bool ecg, bool bioz, bool lead_on_detect) const;
  uint32_t _biozValue() const;
  uint32_t _emuxValue() const;
  uint32_t _mngrDynValue() const;
  uint32_t _rtor1Value() const;
  uint32_t _rtor2Value() const;
  uint32_t _bmuxValue() const;
  max30001_error_t _applyBioZ();
  
  // Helper methods
  void _configure(bool ecg, bool bioz, max30001_sample_rate_t sample_rate,
                  max30001_ecg_gain_t gain);
  max30001_error_t _validateSampleRate(max30001_sample_rate_t rate);
  uint8_t _rateToRegValue(max30001_sample_rate_t rate);
  uint8_t _getSampleDelayMs(max30001_sample_rate_t rate) const;
};

#endif
