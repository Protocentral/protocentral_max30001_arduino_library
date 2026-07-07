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

// =============================================================================
// SPI Configuration
// =============================================================================
#define MAX30001_SPI_SPEED 1000000

// =============================================================================
// Register Addresses
// =============================================================================
#define WREG 0x00
#define RREG 0x01

#define STATUS 0x01
#define EN_INT 0x02
#define EN_INT2 0x03
#define MNGR_INT 0x04
#define MNGR_DYN 0x05
#define SW_RST 0x08
#define SYNCH 0x09
#define FIFO_RST 0x0A
#define INFO 0x0F
#define CNFG_GEN 0x10
#define CNFG_CAL 0x12
#define CNFG_EMUX 0x14
#define CNFG_ECG 0x15

#define CNFG_BIOZ_LC 0x1A

#define CNFG_BMUX 0x17
#define CNFG_BIOZ 0x18

#define CNFG_RTOR1 0x1D
#define CNFG_RTOR2 0x1E

#define ECG_FIFO_BURST 0x20
#define ECG_FIFO 0x21

#define BIOZ_FIFO_BURST 0x22
#define BIOZ_FIFO 0x23

#define RTOR 0x25
#define NO_OP 0x7F

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
    MAX30001_ERROR_NOT_READY                 ///< Data not ready
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
} max30001_bioz_sample_t;

/**
 * @brief R-R detection data structure
 */
typedef struct {
    uint16_t heart_rate_bpm;      ///< Heart rate in BPM
    uint16_t rr_interval_ms;      ///< R-R interval in milliseconds
    bool rr_detected;             ///< R-R detection flag
} max30001_rtor_data_t;

/**
 * @brief Device information structure
 */
typedef struct {
    uint8_t part_id;              ///< Part ID
    uint8_t revision;             ///< Chip revision
    bool device_found;            ///< Device detection flag
} max30001_device_info_t;

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

// Legacy typedef for backward compatibility
typedef enum
{
  SAMPLINGRATE_128 = 128,
  SAMPLINGRATE_256 = 256,
  SAMPLINGRATE_512 = 512
} sampRate;

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
   * @param sample_rate ECG sample rate (BioZ will be half)
   * @return Error code
   */
  max30001_error_t startECGBioZ(max30001_sample_rate_t sample_rate = MAX30001_RATE_128);
  
  /**
   * @brief Start ECG with R-R interval detection
   * @param sample_rate Desired sample rate (default: 128 SPS)
   * @param gain ECG gain setting (default: 80 V/V)
   * @return Error code (convenience wrapper for startECG with R-R enabled internally)
   */
  max30001_error_t startRtoR(max30001_sample_rate_t sample_rate = MAX30001_RATE_128,
                             max30001_ecg_gain_t gain = MAX30001_ECG_GAIN_80);
  
  /**
   * @brief Get single ECG sample
   * @param sample Pointer to store ECG sample data
   * @return Error code
   */
  max30001_error_t getECGSample(max30001_ecg_sample_t* sample);
  
  /**
   * @brief Get single BioZ sample
   * @param sample Pointer to store BioZ sample data
   * @return Error code
   */
  max30001_error_t getBioZSample(max30001_bioz_sample_t* sample);
  
  /**
   * @brief Enable R-to-R / heart-rate extraction while keeping the ECG/BioZ
   *        stream active.
   * @return Error code
   */
  max30001_error_t enableRtoR();
  
  /**
   * @brief Get R-R detection data
   * @param rtor_data Pointer to store R-R data
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
  
  // =========================================================================
  // Advanced Configuration API (Phase 3)
  // =========================================================================
  
  /**
   * @brief Set ECG gain during runtime
   * @param gain New gain setting (80, 160 V/V options)
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
   * @brief Set ECG high-pass filter cutoff frequency
   * @param cutoff_hz Cutoff frequency in Hz (typically 0.4, 0.8, 1.2 Hz)
   * @return Error code
   */
  max30001_error_t setECGHighPassFilter(float cutoff_hz);
  
  /**
   * @brief Set ECG low-pass filter cutoff frequency
   * @param cutoff_hz Cutoff frequency in Hz (typically 40, 100 Hz)
   * @return Error code
   */
  max30001_error_t setECGLowPassFilter(float cutoff_hz);
  
  /**
   * @brief Get lead-off detection status for both electrodes
   * @return true if lead-off detected on either electrode, false if both connected
   */
  bool getLeadOffStatus();
  
  /**
   * @brief Get FIFO sample count for ECG channel
   * @return Number of samples available in FIFO
   */
  uint8_t getFIFOCount();
  
  /**
   * @brief Clear/flush ECG FIFO buffer
   * @return Error code
   */
  max30001_error_t clearFIFO();
  
  /**
   * @brief Enable INT1 interrupt output
   * @return Error code
   */
  max30001_error_t enableInterrupt();
  
  /**
   * @brief Disable INT1 interrupt output
   * @return Error code
   */
  max30001_error_t disableInterrupt();
  
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
  
  // Legacy public data members (deprecated - use new API methods instead)
  unsigned int heartRate;
  unsigned int RRinterval;
  signed long ecg_data;
  signed long bioz_data;
  volatile int ecgSamplesAvailable;
  volatile int biozSamplesAvailable;
  signed long s32ECGData[128];
  signed long s32BIOZData[128];
  
  // Legacy initialization methods
  void BeginECGOnly();
  void BeginECGBioZ();
  void BeginRtoRMode();
  
  // Legacy data acquisition methods
  signed long getECGSamples(void);
  signed long getBioZSamples(void);
  void getHRandRR(void);
  
  // Legacy utility methods
  bool max30001ReadInfo(void);
  void max30001SetsamplingRate(uint16_t samplingRate);
  void max30001SetInterrupts(uint32_t interrupts);
  void max30001ServiceAllInterrupts();
  void readStatus(void);

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
  
  max30001_status_t global_status;
  volatile unsigned char _readBufferECG[128];
  volatile unsigned char _readBufferBIOZ[128];
  
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
  
  // Helper methods
  max30001_error_t _validateSampleRate(max30001_sample_rate_t rate);
  uint8_t _rateToRegValue(max30001_sample_rate_t rate);
  uint8_t _getSampleDelayMs(max30001_sample_rate_t rate) const;
};

#endif
