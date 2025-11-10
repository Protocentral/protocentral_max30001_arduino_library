//////////////////////////////////////////////////////////////////////////////////////////
//
//    Advanced Configuration Demo for MAX30001
//
//    This example demonstrates advanced configuration features including:
//    - Runtime gain adjustment
//    - Channel enable/disable control
//    - Filter configuration (high-pass, low-pass)
//    - Lead-off detection
//    - FIFO management
//    - Interrupt control
//
//    Hardware Setup:
//      Arduino Uno/Mega:
//        - MISO: D12
//        - MOSI: D11
//        - SCLK: D13
//        - CS: D7
//        - VCC: +5V
//        - GND: GND
//
//    Author: Protocentral Electronics
//    Copyright (c) 2025 ProtoCentral
//
//    This software is licensed under the MIT License(http://opensource.org/licenses/MIT).
//
//////////////////////////////////////////////////////////////////////////////////////////

#include <SPI.h>
#include <protocentral_max30001.h>

// Pin Configuration
#define MAX30001_CS_PIN 7

// Create sensor instance
MAX30001 ecgSensor(MAX30001_CS_PIN);

// Configuration state
max30001_ecg_gain_t current_gain = MAX30001_ECG_GAIN_80;
uint32_t config_change_time = 0;
uint32_t last_status_time = 0;

// Constants for timing
#define CONFIG_CHANGE_INTERVAL 5000   // Change config every 5 seconds
#define STATUS_REPORT_INTERVAL 1000   // Report status every 1 second

void setup() {
    Serial.begin(115200);
    while (!Serial) {
        delay(10);
    }
    
    Serial.println("\n===============================================");
    Serial.println("   MAX30001 Advanced Configuration Demo");
    Serial.println("===============================================\n");
    
    // Initialize SPI
    SPI.begin();
    
    Serial.println("Initializing MAX30001...");
    
    // Initialize the sensor
    max30001_error_t result = ecgSensor.begin();
    if (result != MAX30001_SUCCESS) {
        Serial.print("✗ Failed to initialize MAX30001. Error code: ");
        Serial.println(result);
        while (1) {
            delay(1000);
        }
    }
    Serial.println("✓ MAX30001 initialized successfully\n");
    
    // Check connection
    if (!ecgSensor.isConnected()) {
        Serial.println("✗ Device not responding on SPI bus");
        while (1) {
            delay(1000);
        }
    }
    Serial.println("✓ Device connected\n");
    
    // Get device info
    max30001_device_info_t deviceInfo;
    ecgSensor.getDeviceInfo(&deviceInfo);
    Serial.print("Device: Part ID = 0x");
    Serial.print(deviceInfo.part_id, HEX);
    Serial.print(", Revision = 0x");
    Serial.println(deviceInfo.revision, HEX);
    
    // Start ECG acquisition
    Serial.println("\nStarting ECG with advanced configuration options...");
    result = ecgSensor.startECG(MAX30001_RATE_128, MAX30001_ECG_GAIN_80);
    if (result != MAX30001_SUCCESS) {
        Serial.print("✗ Failed to start ECG. Error code: ");
        Serial.println(result);
        while (1) {
            delay(1000);
        }
    }
    Serial.println("✓ ECG started\n");
    
    // Configure filters
    Serial.println("Configuring filters:");
    ecgSensor.setECGHighPassFilter(0.5);  // 0.5 Hz high-pass
    Serial.println("  - High-pass: 0.5 Hz");
    
    ecgSensor.setECGLowPassFilter(40);    // 40 Hz low-pass
    Serial.println("  - Low-pass: 40 Hz");
    
    config_change_time = millis();
    last_status_time = millis();
    
    Serial.println("\n--- Starting demo (configs change every 5 seconds) ---\n");
}

void loop() {
    uint32_t now = millis();
    
    // Get ECG sample
    max30001_ecg_sample_t ecgSample;
    max30001_error_t result = ecgSensor.getECGSample(&ecgSample);
    
    if (result == MAX30001_SUCCESS && ecgSample.sample_valid) {
        // Convert to microvolts
        float ecg_mv = ecgSensor.convertECGToMicrovolts(ecgSample.ecg_sample, current_gain);
        
        // Print occasionally to keep terminal readable
        if ((now - last_status_time) >= STATUS_REPORT_INTERVAL) {
            Serial.print("ECG: ");
            Serial.print(ecg_mv, 2);
            Serial.print(" mV | Gain: ");
            Serial.print(current_gain);
            Serial.print(" V/V | FIFO: ");
            Serial.print(ecgSensor.getFIFOCount());
            Serial.print(" samples | Lead-off: ");
            Serial.println(ecgSensor.getLeadOffStatus() ? "YES" : "NO");
            
            last_status_time = now;
        }
    }
    
    // Demonstrate runtime configuration changes
    if ((now - config_change_time) >= CONFIG_CHANGE_INTERVAL) {
        config_change_time = now;
        demonstrateConfigurationChange();
    }
    
    delay(8);  // ~128 SPS
}

/**
 * @brief Demonstrate different configuration options
 */
void demonstrateConfigurationChange() {
    static uint8_t config_step = 0;
    
    Serial.println("\n--- Configuration Change ---");
    
    switch (config_step) {
        case 0:
            // Change gain to 160 V/V
            Serial.println("Action: Increasing ECG gain to 160 V/V");
            if (ecgSensor.setECGGain(MAX30001_ECG_GAIN_160) == MAX30001_SUCCESS) {
                current_gain = MAX30001_ECG_GAIN_160;
                Serial.println("✓ Gain changed to 160 V/V");
            }
            break;
            
        case 1:
            // Change back to 80 V/V
            Serial.println("Action: Decreasing ECG gain to 80 V/V");
            if (ecgSensor.setECGGain(MAX30001_ECG_GAIN_80) == MAX30001_SUCCESS) {
                current_gain = MAX30001_ECG_GAIN_80;
                Serial.println("✓ Gain changed to 80 V/V");
            }
            break;
            
        case 2:
            // Check and display current configuration
            Serial.println("Action: Reporting current configuration");
            Serial.print("  - ECG enabled: ");
            Serial.println(ecgSensor.isECGEnabled() ? "Yes" : "No");
            Serial.print("  - BioZ enabled: ");
            Serial.println(ecgSensor.isBioZEnabled() ? "Yes" : "No");
            Serial.print("  - Current gain: ");
            Serial.print(ecgSensor.getECGGain());
            Serial.println(" V/V");
            Serial.print("  - Sample rate: ");
            Serial.print(ecgSensor.getSampleRate());
            Serial.println(" SPS");
            Serial.print("  - Sample delay: ");
            Serial.print(ecgSensor.getSampleDelayMs());
            Serial.println(" ms");
            break;
            
        case 3:
            // Change high-pass filter to 1.2 Hz
            Serial.println("Action: Setting high-pass filter to 1.2 Hz");
            if (ecgSensor.setECGHighPassFilter(1.2) == MAX30001_SUCCESS) {
                Serial.println("✓ High-pass filter updated");
            }
            break;
            
        case 4:
            // Change low-pass filter to 100 Hz
            Serial.println("Action: Setting low-pass filter to 100 Hz");
            if (ecgSensor.setECGLowPassFilter(100) == MAX30001_SUCCESS) {
                Serial.println("✓ Low-pass filter updated");
            }
            break;
            
        case 5:
            // Disable ECG temporarily
            Serial.println("Action: Temporarily disabling ECG");
            if (ecgSensor.disableECG() == MAX30001_SUCCESS) {
                Serial.println("✓ ECG disabled");
            }
            break;
            
        case 6:
            // Re-enable ECG
            Serial.println("Action: Re-enabling ECG");
            if (ecgSensor.enableECG() == MAX30001_SUCCESS) {
                Serial.println("✓ ECG re-enabled");
            }
            break;
            
        case 7:
            // Clear FIFO
            Serial.println("Action: Clearing FIFO buffer");
            if (ecgSensor.clearFIFO() == MAX30001_SUCCESS) {
                Serial.println("✓ FIFO cleared");
            }
            break;
            
        case 8:
            // Get last error
            Serial.println("Action: Getting last error status");
            max30001_error_t last_error = ecgSensor.getLastError();
            Serial.print("  - Last error code: ");
            Serial.println(last_error);
            break;
    }
    
    config_step++;
    if (config_step > 8) {
        config_step = 0;  // Cycle back to beginning
    }
    
    Serial.println();
}
