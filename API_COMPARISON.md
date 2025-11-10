# MAX30001 Library API Comparison

## Quick Start Comparison

### Legacy API
```cpp
#include <SPI.h>
#include <protocentral_max30001.h>

#define MAX30001_CS_PIN 7
MAX30001 ecgSensor(MAX30001_CS_PIN);

bool BioZSkipSample = false;

void setup() {
    Serial.begin(57600);
    SPI.begin();
    
    // No error checking possible
    bool ret = ecgSensor.max30001ReadInfo();
    if (ret) {
        Serial.println("MAX 30001 read ID Success");
    } else {
        while (!ret) {
            ret = ecgSensor.max30001ReadInfo();
            Serial.println("Failed to read ID");
            delay(5000);
        }
    }
    
    ecgSensor.BeginECGBioZ();
}

void loop() {
    // Raw signed long values, no context
    signed long ecg_data = ecgSensor.getECGSamples();
    signed long bioz_data;
    
    if (BioZSkipSample == false) {
        bioz_data = ecgSensor.getBioZSamples();
    } else {
        bioz_data = 0x00;
    }
    BioZSkipSample = !BioZSkipSample;
    
    // Manual processing required
    Serial.print(ecg_data);
    Serial.print(",");
    Serial.println(bioz_data);
    
    delay(8);
}
```

### New API
```cpp
#include <SPI.h>
#include <protocentral_max30001.h>

#define MAX30001_CS_PIN 7
MAX30001 ecgSensor(MAX30001_CS_PIN);

bool skipBioZSample = false;

void setup() {
    Serial.begin(115200);
    SPI.begin();
    
    // Error-checked initialization
    if (ecgSensor.begin() != MAX30001_SUCCESS) {
        Serial.println("✗ Failed to initialize");
        while (1);
    }
    Serial.println("✓ MAX30001 initialized");
    
    // Check connection
    if (!ecgSensor.isConnected()) {
        Serial.println("✗ Device not responding");
        while (1);
    }
    
    // Get device info
    max30001_device_info_t info;
    ecgSensor.getDeviceInfo(&info);
    Serial.print("Device: 0x");
    Serial.println(info.part_id, HEX);
    
    // Start with specific configuration
    ecgSensor.startECGBioZ(MAX30001_RATE_128);
}

void loop() {
    max30001_ecg_sample_t ecg;
    max30001_bioz_sample_t bioz;
    
    // Get structured samples with error checking
    if (ecgSensor.getECGSample(&ecg) == MAX30001_SUCCESS) {
        if (ecg.sample_valid) {
            // Automatic conversion to microvolts
            float uv = ecgSensor.convertECGToMicrovolts(
                ecg.ecg_sample, MAX30001_ECG_GAIN_80);
            
            Serial.print(uv, 2);
            Serial.print(" µV, ");
            
            if (!skipBioZSample) {
                if (ecgSensor.getBioZSample(&bioz) == MAX30001_SUCCESS) {
                    Serial.print(bioz.bioz_sample);
                }
            }
            skipBioZSample = !skipBioZSample;
            
            // Lead-off detection included
            if (ecg.lead_off_detected) {
                Serial.println(" [LEAD OFF]");
            } else {
                Serial.println(" [OK]");
            }
        }
    }
    
    delay(8);
}
```

## Feature Comparison Matrix

| Feature | Legacy API | New API |
|---------|-----------|---------|
| **Error Handling** | Boolean or void | Error codes (`max30001_error_t`) |
| **Return Values** | Raw `signed long` | Structured types |
| **Type Safety** | Magic numbers | Named enums |
| **Documentation** | Minimal | Doxygen comments |
| **Configuration** | Fixed in `BeginXXX()` | Flexible parameters |
| **Unit Conversion** | Manual | Built-in (`convertECGToMicrovolts()`) |
| **Metadata** | None | Timestamps, validity flags |
| **Device Info** | Boolean check | Structured info |
| **Lead-off Detection** | Not exposed | Included in samples |
| **Memory Usage** | 45,364 bytes | 48,216 bytes (+6%) |
| **API Stability** | Stable | New, evolving |
| **Backward Compatible** | N/A | ✅ Yes |

## Compilation Results

### Arduino Uno R4 Minima

```
┌─────────────────────────────┬─────────────┬──────────────┐
│ Example                     │ Program     │ Global Vars  │
├─────────────────────────────┼─────────────┼──────────────┤
│ Legacy Example              │ 45,364 B    │ 5,524 B      │
│ New API Example             │ 48,216 B    │ 5,504 B      │
│ Overhead                    │ +2,852 B    │ -20 B        │
│ Percentage                  │ +6.3%       │ -0.4%        │
└─────────────────────────────┴─────────────┴──────────────┘

Memory Available: 262,144 B program / 32,768 B RAM
Both examples use < 20% of available resources
```

## Benefits of New API

### 1. Better Error Handling
```cpp
// Legacy: Silent failure
ecgSensor.BeginECGBioZ();

// New: Explicit error checking
if (ecgSensor.startECGBioZ() != MAX30001_SUCCESS) {
    Serial.println("Error code: " + String(ecgSensor.getLastError()));
}
```

### 2. Structured Data
```cpp
// Legacy: Just a number
signed long ecg = ecgSensor.getECGSamples();

// New: Rich metadata
max30001_ecg_sample_t ecg;
ecgSensor.getECGSample(&ecg);
// Now you have: ecg.ecg_sample, ecg.timestamp_ms, 
//                ecg.lead_off_detected, ecg.sample_valid
```

### 3. Type Safety
```cpp
// Legacy: Magic numbers
ecgSensor.max30001SetsamplingRate(128);  // Is this valid?

// New: Compiler-checked enums
ecgSensor.startECG(MAX30001_RATE_128);  // Type-safe!
```

### 4. Unit Conversion
```cpp
// Legacy: Manual calculation required
float uv = (ecg_data * 156.25) / 1000.0;  // Hardcoded LSB value

// New: Built-in conversion
float uv = ecgSensor.convertECGToMicrovolts(ecg.ecg_sample, 
                                            MAX30001_ECG_GAIN_80);
```

### 5. Configuration Flexibility
```cpp
// Legacy: One size fits all
ecgSensor.BeginECGOnly();

// New: Configurable parameters
ecgSensor.startECG(MAX30001_RATE_512, MAX30001_ECG_GAIN_160);
```

## Migration Path

Existing code continues to work without modification. New projects should use the new API. Gradual migration is supported:

```cpp
// Phase 1: Keep using legacy methods
MAX30001 sensor(7);
sensor.BeginECGBioZ();
signed long ecg = sensor.getECGSamples();

// Phase 2: Start using new initialization
MAX30001 sensor(7);
sensor.begin();  // Better error handling
sensor.BeginECGBioZ();  // Still works
signed long ecg = sensor.getECGSamples();

// Phase 3: Full migration to new API
MAX30001 sensor(7);
sensor.begin();
sensor.startECGBioZ(MAX30001_RATE_128);
max30001_ecg_sample_t ecg;
sensor.getECGSample(&ecg);
```