# MAX30001 Arduino Library - API Reference

Complete documentation for all public methods in the MAX30001 driver library.

## Table of Contents

1. [Initialization & Device Management](#initialization--device-management)
2. [Measurement Modes](#measurement-modes)
3. [Data Acquisition](#data-acquisition)
4. [Advanced Configuration](#advanced-configuration)
5. [Utility Methods](#utility-methods)
6. [Error Codes](#error-codes)
7. [Data Structures](#data-structures)

---

## Initialization & Device Management

### Constructor

```cpp
MAX30001(uint8_t cs_pin);
MAX30001(uint8_t cs_pin, SPIClass* spi_interface);
```

**Parameters:**
- `cs_pin` - Arduino pin connected to MAX30001 CS line (typically D7)
- `spi_interface` - (Optional) Pointer to SPIClass for custom SPI bus (default: global `SPI`)

**Returns:** Device instance

**Example:**
```cpp
MAX30001 sensor(7);                    // Use default SPI bus, CS=D7
MAX30001 sensor(5, &SPI1);             // Use SPI1 with CS=D5 (for boards with multiple SPI)
```

---

### begin()

```cpp
max30001_error_t begin();
```

**Description:** Initialize the MAX30001 device. Performs SPI communication test and basic chip configuration.

**Returns:** 
- `MAX30001_SUCCESS` on success
- Error code on failure

**Must be called before any measurement functions.**

**Example:**
```cpp
if (sensor.begin() != MAX30001_SUCCESS) {
    Serial.println("Failed to initialize");
    while(1);
}
```

---

### isConnected()

```cpp
bool isConnected();
```

**Description:** Check if device is responding on SPI bus.

**Returns:** 
- `true` if device responds
- `false` if no response

**Example:**
```cpp
if (!sensor.isConnected()) {
    Serial.println("Check SPI connections");
}
```

---

### getDeviceInfo()

```cpp
max30001_error_t getDeviceInfo(max30001_device_info_t* info);
```

**Parameters:**
- `info` - Pointer to `max30001_device_info_t` structure to receive device information

**Returns:** Error code

**Description:** Retrieves chip part ID and revision information.

**Example:**
```cpp
max30001_device_info_t device;
sensor.getDeviceInfo(&device);
Serial.print("Part ID: 0x");
Serial.println(device.part_id, HEX);
Serial.print("Revision: 0x");
Serial.println(device.revision, HEX);
```

---

## Measurement Modes

### startECG()

```cpp
max30001_error_t startECG(max30001_sample_rate_t sample_rate = MAX30001_RATE_128,
                          max30001_ecg_gain_t gain = MAX30001_ECG_GAIN_80);
```

**Parameters:**
- `sample_rate` - Sampling rate: `MAX30001_RATE_128`, `MAX30001_RATE_256`, or `MAX30001_RATE_512`
- `gain` - Gain setting: `MAX30001_ECG_GAIN_80` or `MAX30001_ECG_GAIN_160` (V/V)

**Returns:** Error code

**Description:** Start single-channel ECG acquisition. Also enables built-in R-R detection.

**Example:**
```cpp
sensor.startECG(MAX30001_RATE_128, MAX30001_ECG_GAIN_80);
```

---

### startBioZ()

```cpp
max30001_error_t startBioZ(max30001_sample_rate_t sample_rate = MAX30001_RATE_128);
```

**Parameters:**
- `sample_rate` - Desired sampling rate (BioZ samples at half this rate)

**Returns:** Error code

**Description:** Start bio-impedance measurement (typically used for respiration).

**Note:** If used with ECG, BioZ will sample at half the ECG rate.

**Example:**
```cpp
sensor.startBioZ(MAX30001_RATE_128);  // BioZ at 64 SPS (half of 128)
```

---

### startECGBioZ()

```cpp
max30001_error_t startECGBioZ(max30001_sample_rate_t sample_rate = MAX30001_RATE_128);
```

**Parameters:**
- `sample_rate` - ECG sample rate (BioZ will be half this rate)

**Returns:** Error code

**Description:** Start simultaneous ECG and BioZ acquisition. Recommended for respiration monitoring.

**Example:**
```cpp
sensor.startECGBioZ(MAX30001_RATE_128);  // ECG: 128 SPS, BioZ: 64 SPS
```

---

### startRtoR()

```cpp
max30001_error_t startRtoR(max30001_sample_rate_t sample_rate = MAX30001_RATE_128,
                           max30001_ecg_gain_t gain = MAX30001_ECG_GAIN_80);
```

**Parameters:**
- `sample_rate` - ECG sample rate
- `gain` - ECG gain setting

**Returns:** Error code

**Description:** Convenience method for R-R detection mode (heartbeat detection). Internally calls `startECG()`.

**Example:**
```cpp
sensor.startRtoR(MAX30001_RATE_128, MAX30001_ECG_GAIN_160);
```

---

### stop()

```cpp
void stop();
```

**Description:** Stop all acquisition (ECG, BioZ, and R-R detection).

---

## Data Acquisition

### getECGSample()

```cpp
max30001_error_t getECGSample(max30001_ecg_sample_t* sample);
```

**Parameters:**
- `sample` - Pointer to `max30001_ecg_sample_t` structure to receive sample data

**Returns:** Error code

**Description:** Read next available ECG sample from FIFO.

**Note:** Blocks if no samples available. Call in a loop with small delays for continuous acquisition.

**Example:**
```cpp
max30001_ecg_sample_t ecg;
if (sensor.getECGSample(&ecg) == MAX30001_SUCCESS && ecg.sample_valid) {
    float ecg_mv = sensor.convertECGToMicrovolts(ecg.ecg_sample, MAX30001_ECG_GAIN_80);
    Serial.println(ecg_mv);
}
```

---

### getBioZSample()

```cpp
max30001_error_t getBioZSample(max30001_bioz_sample_t* sample);
```

**Parameters:**
- `sample` - Pointer to `max30001_bioz_sample_t` structure to receive sample data

**Returns:** Error code

**Description:** Read next available BioZ sample from FIFO.

**Important:** BioZ samples at half the ECG rate. In ECG+BioZ mode, skip every other read or hold last value.

**Example:**
```cpp
max30001_bioz_sample_t bioz;
if (sensor.getBioZSample(&bioz) == MAX30001_SUCCESS && bioz.sample_valid) {
    Serial.println(bioz.bioz_sample);
}
```

---

### getRtoRData()

```cpp
max30001_error_t getRtoRData(max30001_rtor_data_t* rtor_data);
```

**Parameters:**
- `rtor_data` - Pointer to `max30001_rtor_data_t` structure to receive R-R data

**Returns:** Error code

**Description:** Get latest R-R interval and calculated heart rate from hardware detector.

**Example:**
```cpp
max30001_rtor_data_t rtor;
if (sensor.getRtoRData(&rtor) == MAX30001_SUCCESS && rtor.rr_detected) {
    Serial.print("Heart Rate: ");
    Serial.print(rtor.heart_rate_bpm);
    Serial.print(" BPM, R-R: ");
    Serial.print(rtor.rr_interval_ms);
    Serial.println(" ms");
}
```

---

## Advanced Configuration

### setECGGain()

```cpp
max30001_error_t setECGGain(max30001_ecg_gain_t gain);
```

**Parameters:**
- `gain` - `MAX30001_ECG_GAIN_80` (80 V/V) or `MAX30001_ECG_GAIN_160` (160 V/V)

**Returns:** Error code

**Description:** Change ECG gain during runtime. Allows adapting to different signal amplitudes.

**Example:**
```cpp
// For larger amplitude signals, reduce to 80 V/V
sensor.setECGGain(MAX30001_ECG_GAIN_80);

// For small amplitude signals, increase to 160 V/V
sensor.setECGGain(MAX30001_ECG_GAIN_160);
```

---

### getECGGain()

```cpp
max30001_ecg_gain_t getECGGain() const;
```

**Returns:** Current ECG gain setting

**Description:** Query the currently configured ECG gain.

---

### enableECG() / disableECG()

```cpp
max30001_error_t enableECG();
max30001_error_t disableECG();
```

**Returns:** Error code

**Description:** Pause/resume ECG acquisition without stopping BioZ.

**Example:**
```cpp
// Temporarily stop ECG
sensor.disableECG();

// Resume ECG
sensor.enableECG();
```

---

### isECGEnabled()

```cpp
bool isECGEnabled() const;
```

**Returns:** `true` if ECG is currently acquiring, `false` otherwise

---

### enableBioZ() / disableBioZ()

```cpp
max30001_error_t enableBioZ();
max30001_error_t disableBioZ();
```

**Returns:** Error code

**Description:** Independently control BioZ acquisition.

---

### isBioZEnabled()

```cpp
bool isBioZEnabled() const;
```

**Returns:** `true` if BioZ is currently acquiring

---

### setECGHighPassFilter()

```cpp
max30001_error_t setECGHighPassFilter(float cutoff_hz);
```

**Parameters:**
- `cutoff_hz` - Cutoff frequency in Hz (typical values: 0.4, 0.8, 1.2, 1.6, 2.0 Hz)

**Returns:** Error code

**Description:** Adjust ECG high-pass filter to remove baseline wander.

**Common Values:**
- `0.5` Hz - Remove very slow drift
- `1.0` Hz - Standard clinical setting
- `2.0` Hz - Remove more baseline variation

---

### setECGLowPassFilter()

```cpp
max30001_error_t setECGLowPassFilter(float cutoff_hz);
```

**Parameters:**
- `cutoff_hz` - Cutoff frequency in Hz (typical values: 40, 100, 150, 200 Hz)

**Returns:** Error code

**Description:** Adjust ECG low-pass filter to remove high-frequency noise.

**Common Values:**
- `40` Hz - Clinical setting (removes power line interference)
- `100` Hz - More detail preserved
- `150` Hz - Minimal filtering

---

### getLeadOffStatus()

```cpp
bool getLeadOffStatus();
```

**Returns:** 
- `true` if lead-off detected (electrode not connected)
- `false` if electrodes connected

**Description:** Check if ECG electrodes are properly connected to skin.

**Example:**
```cpp
if (sensor.getLeadOffStatus()) {
    Serial.println("Warning: Check electrode connections!");
}
```

---

### getFIFOCount()

```cpp
uint8_t getFIFOCount();
```

**Returns:** Number of samples currently in FIFO buffer

**Description:** Get the number of pending samples waiting to be read.

**Example:**
```cpp
if (sensor.getFIFOCount() > 10) {
    Serial.println("FIFO has " + String(sensor.getFIFOCount()) + " samples");
}
```

---

### clearFIFO()

```cpp
max30001_error_t clearFIFO();
```

**Returns:** Error code

**Description:** Reset/clear the ECG FIFO buffer. Useful for discarding stale data after configuration changes.

**Example:**
```cpp
sensor.setECGGain(MAX30001_ECG_GAIN_160);
sensor.clearFIFO();  // Discard data acquired with old gain
```

---

### enableInterrupt() / disableInterrupt()

```cpp
max30001_error_t enableInterrupt();
max30001_error_t disableInterrupt();
```

**Returns:** Error code

**Description:** Control INT1 interrupt output for ISR-based data acquisition.

---

## Utility Methods

### convertECGToMicrovolts()

```cpp
float convertECGToMicrovolts(int32_t raw_value, max30001_ecg_gain_t gain);
```

**Parameters:**
- `raw_value` - Raw ADC value from ECG sample
- `gain` - Gain setting used for acquisition (`MAX30001_ECG_GAIN_80` or `MAX30001_ECG_GAIN_160`)

**Returns:** Converted value in millivolts

**Description:** Convert raw ADC counts to physical ECG signal in millivolts.

**Important:** Must use the same gain value as used during acquisition for accurate conversion.

**Example:**
```cpp
max30001_ecg_sample_t ecg;
sensor.getECGSample(&ecg);

float ecg_mv = sensor.convertECGToMicrovolts(ecg.ecg_sample, MAX30001_ECG_GAIN_80);
Serial.print("ECG: ");
Serial.print(ecg_mv, 2);  // 2 decimal places
Serial.println(" mV");
```

---

### getSampleRate()

```cpp
max30001_sample_rate_t getSampleRate() const;
```

**Returns:** Current sample rate setting

**Description:** Query the configured ECG sample rate.

---

### getSampleDelayMs()

```cpp
uint16_t getSampleDelayMs() const;
```

**Returns:** Delay in milliseconds between samples

**Description:** Get the expected time interval between samples (useful for timing loops).

**Common Values:**
- 128 SPS: 8 ms
- 256 SPS: 4 ms
- 512 SPS: 2 ms

**Example:**
```cpp
uint16_t delay_ms = sensor.getSampleDelayMs();
Serial.print("Expected sample interval: ");
Serial.print(delay_ms);
Serial.println(" ms");
```

---

### getLastError()

```cpp
max30001_error_t getLastError() const;
```

**Returns:** Last error code that occurred

**Description:** Retrieve error code from last operation (useful for debugging).

---

## Error Codes

```cpp
typedef enum {
    MAX30001_SUCCESS = 0,                   // Operation successful
    MAX30001_ERROR_INVALID_PARAMETER,       // Invalid parameter value
    MAX30001_ERROR_NOT_INITIALIZED,         // Device not initialized (call begin() first)
    MAX30001_ERROR_NOT_READY,               // Device not ready for operation
    MAX30001_ERROR_SPI_COMMUNICATION,       // SPI communication failed
    MAX30001_ERROR_DEVICE_NOT_FOUND,        // Device not responding
    MAX30001_ERROR_FIFO_OVERFLOW            // FIFO buffer overflow
} max30001_error_t;
```

---

## Data Structures

### max30001_ecg_sample_t

```cpp
typedef struct {
    int32_t ecg_sample;          // Raw ADC value
    uint32_t timestamp_ms;       // Sample acquisition timestamp
    bool lead_off_detected;      // Electrode connectivity flag
    bool sample_valid;           // Data validity flag
} max30001_ecg_sample_t;
```

---

### max30001_bioz_sample_t

```cpp
typedef struct {
    int32_t bioz_sample;         // Raw BioZ ADC value
    uint32_t timestamp_ms;       // Sample timestamp
    bool sample_valid;           // Data validity flag
} max30001_bioz_sample_t;
```

---

### max30001_rtor_data_t

```cpp
typedef struct {
    uint16_t heart_rate_bpm;     // Calculated heart rate (beats per minute)
    uint16_t rr_interval_ms;     // R-to-R interval in milliseconds
    bool rr_detected;            // Valid R-R detection flag
} max30001_rtor_data_t;
```

---

### max30001_device_info_t

```cpp
typedef struct {
    uint8_t part_id;             // Part ID (typically 0x00 for MAX30001)
    uint8_t revision;            // Chip revision
    bool device_found;           // Device detection flag
} max30001_device_info_t;
```

---

## Enumerations

### Sample Rates

```cpp
typedef enum {
    MAX30001_RATE_128 = 128,     // 128 samples per second
    MAX30001_RATE_256 = 256,     // 256 samples per second
    MAX30001_RATE_512 = 512      // 512 samples per second
} max30001_sample_rate_t;
```

### ECG Gain

```cpp
typedef enum {
    MAX30001_ECG_GAIN_80 = 80,   // 80 V/V gain
    MAX30001_ECG_GAIN_160 = 160  // 160 V/V gain
} max30001_ecg_gain_t;
```

---

## Complete Example

```cpp
#include <SPI.h>
#include <protocentral_max30001.h>

MAX30001 sensor(7);

void setup() {
    Serial.begin(115200);
    SPI.begin();
    
    // Initialize and check
    if (sensor.begin() != MAX30001_SUCCESS) {
        Serial.println("Failed to initialize");
        while(1) delay(1000);
    }
    
    // Get device info
    max30001_device_info_t info;
    sensor.getDeviceInfo(&info);
    Serial.print("Device found: Part ID = 0x");
    Serial.println(info.part_id, HEX);
    
    // Configure and start
    sensor.setECGGain(MAX30001_ECG_GAIN_160);
    sensor.setECGHighPassFilter(0.5);
    sensor.setECGLowPassFilter(40);
    sensor.startECGBioZ(MAX30001_RATE_128);
}

void loop() {
    max30001_ecg_sample_t ecg;
    max30001_bioz_sample_t bioz;
    
    // Acquire ECG
    if (sensor.getECGSample(&ecg) == MAX30001_SUCCESS && ecg.sample_valid) {
        float ecg_mv = sensor.convertECGToMicrovolts(ecg.ecg_sample, MAX30001_ECG_GAIN_160);
        Serial.print("ECG: ");
        Serial.print(ecg_mv, 2);
        Serial.print(" mV");
        
        // Get R-R data
        max30001_rtor_data_t rtor;
        if (sensor.getRtoRData(&rtor) == MAX30001_SUCCESS && rtor.rr_detected) {
            Serial.print(" | HR: ");
            Serial.print(rtor.heart_rate_bpm);
            Serial.print(" BPM");
        }
        
        Serial.println();
    }
    
    delay(8);  // 128 SPS spacing
}
```
