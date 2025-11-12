// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: Copyright (c) 2025 Ashwin Whitchurch, Protocentral Electronics

/*
 * Interrupt-Driven ECG Acquisition Demo for MAX30001 (New API)
 *
 * Copyright (c) 2025 Ashwin Whitchurch, Protocentral Electronics
 * Email: info@protocentral.com
 *
 * This example demonstrates interrupt-driven ECG data acquisition using the MAX30001's
 * INT1 output. Instead of polling the device in the main loop, the MAX30001 generates
 * an interrupt whenever new data is available.
 *
 * Benefits of interrupt-driven mode:
 *   - Lower CPU usage (no continuous polling)
 *
 * This software is licensed under the MIT License.
 */

//////////////////////////////////////////////////////////////////////////////////////////
//
//      - More precise timing for real-time applications
//      - Better performance in resource-constrained systems
//      - Enables other tasks while waiting for data
//
//    The INT1 pin triggers when:
//      - ECG FIFO has data available
//      - BioZ FIFO has data available
//      - R-wave detected (if using R-R mode)
//      - Lead-off condition detected
//
//    Output includes:
//      - ECG waveform data
//      - BioZ respiration data
//      - Interrupt count and data acquisition rate
//
//    Software:
//      - Arduino IDE 1.8.x or higher
//      - Serial Monitor set to 115200 baud
//
//    Hardware Setup:
//      Arduino Uno/Mega:
//        - MISO: D12
//        - MOSI: D11
//        - SCLK: D13
//        - CS: D7
//        - INT1: D2 (connect MAX30001 INT1 to Arduino D2)
//        - VCC: +5V
//        - GND: GND
//
//      ESP32:
//        - MISO: GPIO 19
//        - MOSI: GPIO 23
//        - SCLK: GPIO 18
//        - CS: GPIO 5
//        - INT1: GPIO 4 (connect MAX30001 INT1 to ESP32 GPIO 4)
//        - VCC: +5V
//        - GND: GND
//
//    Electrode Connections:
//      - Connect ECG electrodes to ECGP and ECGN
//      - Connect BioZ electrodes to BIP and BIN
//      - No reference electrode (RLD) required for 2-electrode ECG mode
//
//    Author: Protocentral Electronics
//    Copyright (c) 2025 ProtoCentral
//
//    This software is licensed under the MIT License (http://opensource.org/licenses/MIT).
//
//////////////////////////////////////////////////////////////////////////////////////////

#include <SPI.h>
#include <protocentral_max30001.h>

// Pin Configuration
// Change to 5 if using ESP32
#define MAX30001_CS_PIN 7
#define MAX30001_INT_PIN 2  // D2 for Arduino, GPIO4 for ESP32

// Measurement Configuration
#define SAMPLE_RATE MAX30001_RATE_128  // 128 samples per second
#define SAMPLE_DELAY_MS 8              // ~128 SPS

// Create sensor instance
MAX30001 ecgSensor(MAX30001_CS_PIN);

// Interrupt and data tracking
volatile bool dataAvailable = false;
volatile uint32_t interruptCount = 0;
unsigned long lastDataTime = 0;
uint32_t dataCount = 0;
uint32_t samplesProcessed = 0;

// BioZ sampling flag (BioZ samples at half the ECG rate)
bool skipBioZSample = false;
int32_t lastBioZ = 0;

// Timing for performance metrics
unsigned long startTime = 0;
unsigned long lastMetricsTime = 0;
const unsigned long METRICS_INTERVAL = 5000;  // Display metrics every 5 seconds

/**
 * @brief Interrupt service routine for MAX30001 INT1 pin
 * Called whenever MAX30001 has data ready
 */
void handleMAX30001Interrupt() {
    dataAvailable = true;
    interruptCount++;
}

void setup() {
    Serial.begin(115200);  // Higher baud rate for detailed output
    while (!Serial) {
        delay(10);  // Wait for serial port to connect
    }
    
    Serial.println("===============================================");
    Serial.println("   MAX30001 Interrupt-Driven Demo (New API)");
    Serial.println("===============================================\n");
    
    // Initialize SPI
    SPI.begin();
    
    Serial.println("Initializing MAX30001...");
    
    // Initialize the sensor
    max30001_error_t result = ecgSensor.begin();
    if (result != MAX30001_SUCCESS) {
        Serial.print("✗ Failed to initialize MAX30001. Error code: ");
        Serial.println(result);
        Serial.println("\nCheck connections:");
        Serial.println("  - SPI wiring (MISO, MOSI, SCLK, CS)");
        Serial.println("  - Power supply (VCC, GND)");
        Serial.println("  - CS pin configuration");
        while (1) {
            delay(1000);
        }
    }
    Serial.println("✓ MAX30001 initialized successfully");
    
    // Check if device is responding
    if (!ecgSensor.isConnected()) {
        Serial.println("✗ Device not responding on SPI bus");
        Serial.println("  Check CS pin and SPI connections");
        while (1) {
            delay(1000);
        }
    }
    Serial.println("✓ Device connected and responding");
    
    // Get device information
    max30001_device_info_t deviceInfo;
    ecgSensor.getDeviceInfo(&deviceInfo);
    Serial.print("✓ Device Info - Part ID: 0x");
    Serial.print(deviceInfo.part_id, HEX);
    Serial.print(", Revision: 0x");
    Serial.println(deviceInfo.revision, HEX);
    
    Serial.println("\nStarting ECG + BioZ acquisition (interrupt-driven)...");
    
    // Start combined ECG and BioZ monitoring
    result = ecgSensor.startECGBioZ(SAMPLE_RATE);
    if (result != MAX30001_SUCCESS) {
        Serial.print("✗ Failed to start acquisition. Error code: ");
        Serial.println(result);
        while (1) {
            delay(1000);
        }
    }
    Serial.println("✓ ECG and BioZ acquisition started");
    
    // Configure and attach interrupt handler
    pinMode(MAX30001_INT_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(MAX30001_INT_PIN), handleMAX30001Interrupt, FALLING);
    Serial.print("✓ Interrupt handler attached to pin D");
    Serial.println(MAX30001_INT_PIN);
    
    Serial.println("\n" + String("Sample Rate: ") + String(SAMPLE_RATE) + " SPS");
    Serial.println("ECG Gain: 160 V/V");
    Serial.println("BioZ samples at half the ECG rate");
    Serial.println("Acquisition mode: Interrupt-driven (INT1 pin)\n");
    
    startTime = millis();
    lastMetricsTime = startTime;
    lastDataTime = startTime;
}

void loop() {
    // Check if MAX30001 has signaled data ready
    if (dataAvailable) {
        dataAvailable = false;  // Clear flag
        
        max30001_ecg_sample_t ecgSample;
        max30001_bioz_sample_t biozSample;
        int32_t ecg_value = 0;
        int32_t bioz_value = lastBioZ;
        
        // Get ECG sample
        max30001_error_t ecgResult = ecgSensor.getECGSample(&ecgSample);
        
        if (ecgResult == MAX30001_SUCCESS && ecgSample.sample_valid) {
            ecg_value = ecgSample.ecg_sample;
            samplesProcessed++;
            dataCount++;
            
            // Handle BioZ sampling (every other ECG sample)
            if (!skipBioZSample) {
                max30001_error_t biozResult = ecgSensor.getBioZSample(&biozSample);
                
                if (biozResult == MAX30001_SUCCESS && biozSample.sample_valid) {
                    bioz_value = biozSample.bioz_sample;
                    lastBioZ = bioz_value;
                } else {
                    bioz_value = lastBioZ;
                }
            } else {
                bioz_value = lastBioZ;
            }
            
            // Toggle BioZ sampling flag
            skipBioZSample = !skipBioZSample;
            
            // Convert ECG to microvolts for display
            int32_t ecg_microvolts = ecgSensor.convertECGToMicrovolts(ecg_value, MAX30001_ECG_GAIN_160);
            
            // Print sample data (sparse output to avoid overwhelming serial)
            if (dataCount % 16 == 0) {  // Print every 16th sample (~8 per second at 128 SPS)
                Serial.print("ECG: ");
                Serial.print(ecg_microvolts);
                Serial.print(" µV | BioZ: ");
                Serial.print(bioz_value, HEX);
                Serial.print(" | Interrupts: ");
                Serial.print(interruptCount);
                Serial.print(" | Samples: ");
                Serial.println(samplesProcessed);
            }
            
            lastDataTime = millis();
        }
        
        // Display performance metrics
        unsigned long now = millis();
        if ((now - lastMetricsTime) >= METRICS_INTERVAL) {
            unsigned long elapsed = now - startTime;
            float dataRate = (samplesProcessed * 1000.0) / elapsed;
            float intRate = (interruptCount * 1000.0) / elapsed;
            
            Serial.println("\n--- Performance Metrics ---");
            Serial.print("Elapsed time: ");
            Serial.print(elapsed / 1000);
            Serial.println(" seconds");
            Serial.print("Samples processed: ");
            Serial.println(samplesProcessed);
            Serial.print("Data rate: ");
            Serial.print(dataRate, 1);
            Serial.println(" samples/sec");
            Serial.print("Interrupt rate: ");
            Serial.print(intRate, 1);
            Serial.println(" interrupts/sec");
            Serial.println("------------------------\n");
            
            lastMetricsTime = now;
        }
    }
    
    // Idle loop - CPU can perform other tasks while waiting for interrupt
    // This demonstrates the advantage of interrupt-driven mode
}
