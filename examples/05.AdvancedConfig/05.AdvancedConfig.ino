// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: Copyright (c) 2025 Ashwin Whitchurch, Protocentral Electronics

/*
 * Advanced Configuration Demo for MAX30001
 *
 * Copyright (c) 2025 Ashwin Whitchurch, Protocentral Electronics
 * Email: info@protocentral.com
 *
 * This example demonstrates advanced configuration features including:
 * - Runtime gain adjustment
 * - Channel enable/disable control
 * - Filter configuration (high-pass, low-pass)
 * - Lead-off detection
 * - FIFO management
 *
 * It runs at 512 SPS, the only rate where all three low-pass options
 * (40, 100, 150 Hz) are available.
 *
 * This software is licensed under the MIT License.
 */

//////////////////////////////////////////////////////////////////////////////////////////
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

// Gain setting in V/V, for display
uint16_t gainToVV(max30001_ecg_gain_t gain) {
    switch (gain) {
        case MAX30001_ECG_GAIN_20: return 20;
        case MAX30001_ECG_GAIN_40: return 40;
        case MAX30001_ECG_GAIN_160: return 160;
        default: return 80;
    }
}

// Constants for timing
#define CONFIG_CHANGE_INTERVAL 5000   // Change config every 5 seconds
#define STATUS_REPORT_INTERVAL 1000   // Report status every 1 second

void setup() {
    Serial.begin(115200);
    while (!Serial) {
        delay(10);
    }
    
    Serial.println(F("\n==============================================="));
    Serial.println(F("   MAX30001 Advanced Configuration Demo"));
    Serial.println(F("===============================================\n"));
    
    Serial.println(F("Initializing MAX30001..."));
    
    // Initialize the sensor
    max30001_error_t result = ecgSensor.begin();
    if (result != MAX30001_SUCCESS) {
        Serial.print(F("✗ Failed to initialize MAX30001. Error code: "));
        Serial.println(result);
        while (1) {
            delay(1000);
        }
    }
    Serial.println(F("✓ MAX30001 initialized successfully\n"));
    
    // Get device info
    max30001_device_info_t deviceInfo;
    ecgSensor.getDeviceInfo(&deviceInfo);
    Serial.print(F("Device: Part ID = 0x"));
    Serial.print(deviceInfo.part_id, HEX);
    Serial.print(F(", Revision = 0x"));
    Serial.println(deviceInfo.revision, HEX);
    
    // Start ECG acquisition
    Serial.println(F("\nStarting ECG with advanced configuration options..."));
    result = ecgSensor.startECG(MAX30001_RATE_512, MAX30001_ECG_GAIN_80);
    if (result != MAX30001_SUCCESS) {
        Serial.print(F("✗ Failed to start ECG. Error code: "));
        Serial.println(result);
        while (1) {
            delay(1000);
        }
    }
    Serial.println(F("✓ ECG started\n"));
    
    // Configure filters
    Serial.println(F("Configuring filters:"));
    ecgSensor.setECGHighPassFilter(0.5);  // 0.5 Hz high-pass
    Serial.println(F("  - High-pass: 0.5 Hz"));
    
    ecgSensor.setECGLowPassFilter(40);    // 40 Hz low-pass
    Serial.println(F("  - Low-pass: 40 Hz"));
    
    config_change_time = millis();
    last_status_time = millis();
    
    Serial.println(F("\n--- Starting demo (configs change every 5 seconds) ---\n"));
}

void loop() {
    uint32_t now = millis();
    
    // Drain the ECG FIFO, keeping the newest sample
    max30001_ecg_sample_t ecgSample;
    static int32_t last_ecg = 0;
    
    while (ecgSensor.getECGSample(&ecgSample) == MAX30001_SUCCESS) {
        last_ecg = ecgSample.ecg_sample;
    }
    
    // Print occasionally to keep terminal readable
    if ((now - last_status_time) >= STATUS_REPORT_INTERVAL) {
        float ecg_uv = ecgSensor.convertECGToMicrovolts(last_ecg, current_gain);
        
        Serial.print(F("ECG: "));
        Serial.print(ecg_uv, 1);
        Serial.print(F(" µV | Gain: "));
        Serial.print(gainToVV(current_gain));
        Serial.print(F(" V/V | Lead-off: "));
        Serial.println(ecgSensor.getLeadOffStatus() ? F("YES") : F("NO"));
        
        last_status_time = now;
    }
    
    // Demonstrate runtime configuration changes
    if ((now - config_change_time) >= CONFIG_CHANGE_INTERVAL) {
        config_change_time = now;
        demonstrateConfigurationChange();
    }
    
    delay(10);
}

/**
 * @brief Demonstrate different configuration options
 */
void demonstrateConfigurationChange() {
    static uint8_t config_step = 0;
    
    Serial.println(F("\n--- Configuration Change ---"));
    
    switch (config_step) {
        case 0:
            // Change gain to 160 V/V
            Serial.println(F("Action: Increasing ECG gain to 160 V/V"));
            if (ecgSensor.setECGGain(MAX30001_ECG_GAIN_160) == MAX30001_SUCCESS) {
                current_gain = MAX30001_ECG_GAIN_160;
                Serial.println(F("✓ Gain changed to 160 V/V"));
            }
            break;
            
        case 1:
            // Change back to 80 V/V
            Serial.println(F("Action: Decreasing ECG gain to 80 V/V"));
            if (ecgSensor.setECGGain(MAX30001_ECG_GAIN_80) == MAX30001_SUCCESS) {
                current_gain = MAX30001_ECG_GAIN_80;
                Serial.println(F("✓ Gain changed to 80 V/V"));
            }
            break;
            
        case 2:
            // Check and display current configuration
            Serial.println(F("Action: Reporting current configuration"));
            Serial.print(F("  - ECG enabled: "));
            Serial.println(ecgSensor.isECGEnabled() ? F("Yes") : F("No"));
            Serial.print(F("  - BioZ enabled: "));
            Serial.println(ecgSensor.isBioZEnabled() ? F("Yes") : F("No"));
            Serial.print(F("  - Current gain: "));
            Serial.print(gainToVV(ecgSensor.getECGGain()));
            Serial.println(F(" V/V"));
            Serial.print(F("  - Sample rate: "));
            Serial.print(ecgSensor.getSampleRate());
            Serial.println(F(" SPS"));
            break;
            
        case 3:
            // Bypass the high-pass filter (DC coupled; the only other option is 0.5 Hz)
            Serial.println(F("Action: Bypassing the high-pass filter (DC coupled)"));
            if (ecgSensor.setECGHighPassFilter(0) == MAX30001_SUCCESS) {
                Serial.println(F("✓ High-pass filter bypassed"));
            }
            break;
            
        case 4:
            // Change low-pass filter to 100 Hz
            Serial.println(F("Action: Setting low-pass filter to 100 Hz, high-pass back to 0.5 Hz"));
            if (ecgSensor.setECGLowPassFilter(100) == MAX30001_SUCCESS &&
                ecgSensor.setECGHighPassFilter(0.5) == MAX30001_SUCCESS) {
                Serial.println(F("✓ Filters updated"));
            }
            break;
            
        case 5:
            // Disable ECG temporarily
            Serial.println(F("Action: Temporarily disabling ECG"));
            if (ecgSensor.disableECG() == MAX30001_SUCCESS) {
                Serial.println(F("✓ ECG disabled"));
            }
            break;
            
        case 6:
            // Re-enable ECG
            Serial.println(F("Action: Re-enabling ECG"));
            if (ecgSensor.enableECG() == MAX30001_SUCCESS) {
                Serial.println(F("✓ ECG re-enabled"));
            }
            break;
            
        case 7:
            // Clear FIFO
            Serial.println(F("Action: Clearing FIFO buffer"));
            if (ecgSensor.clearFIFO() == MAX30001_SUCCESS) {
                Serial.println(F("✓ FIFO cleared"));
            }
            break;
            
        case 8:
            // Get last error
            Serial.println(F("Action: Getting last error status"));
            max30001_error_t last_error = ecgSensor.getLastError();
            Serial.print(F("  - Last error code: "));
            Serial.println(last_error);
            break;
    }
    
    config_step++;
    if (config_step > 8) {
        config_step = 0;  // Cycle back to beginning
    }
    
    Serial.println();
}
