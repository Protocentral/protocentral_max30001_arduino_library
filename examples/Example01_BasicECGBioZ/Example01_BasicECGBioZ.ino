// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: Copyright (c) 2025 Ashwin Whitchurch, Protocentral Electronics

/*
 * Basic ECG + BioZ Demo for MAX30001 (New API)
 *
 * Copyright (c) 2025 Ashwin Whitchurch, Protocentral Electronics
 * Email: info@protocentral.com
 *
 * This example demonstrates the new simplified API for acquiring ECG and BioZ data.
 * It uses high-level methods with automatic configuration and error handling.
 * 
 * Data is streamed in ProtoCentral OpenView format for real-time waveform visualization.
 *
 * This software is licensed under the MIT License.
 */

//////////////////////////////////////////////////////////////////////////////////////////
//
//    Software:
//      - Download ProtoCentral OpenView: https://github.com/Protocentral/protocentral_openview
//      - Set baud rate to 57600
//      - Connect to the serial port to see ECG and BioZ waveforms
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
//      ESP32:
//        - MISO: GPIO 19
//        - MOSI: GPIO 23
//        - SCLK: GPIO 18
//        - CS: GPIO 5
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
//    This software is licensed under the MIT License(http://opensource.org/licenses/MIT).
//
//////////////////////////////////////////////////////////////////////////////////////////

#include <SPI.h>
#include <protocentral_max30001.h>

// Pin Configuration
// Change to 5 if using ESP32
#define MAX30001_CS_PIN 7

// Measurement Configuration
#define SAMPLE_RATE MAX30001_RATE_128  // 128 samples per second
#define ECG_GAIN MAX30001_ECG_GAIN_80  // 80 V/V gain
#define SAMPLE_DELAY_MS 8              // ~128 SPS

// ProtoCentral OpenView packet format constants
#define CES_CMDIF_PKT_START_1  0x0A
#define CES_CMDIF_PKT_START_2  0xFA
#define CES_CMDIF_TYPE_DATA    0x02
#define CES_CMDIF_PKT_STOP     0x0B
#define DATA_LEN               0x0C
#define ZERO                   0

// Create sensor instance
MAX30001 ecgSensor(MAX30001_CS_PIN);

// BioZ sampling flag (BioZ samples at half the ECG rate)
bool skipBioZSample = false;
// Hold the last valid BioZ sample so we can resend it on skipped frames
int32_t lastBioZ = 0;

// OpenView packet buffers
volatile char dataPacket[DATA_LEN];
const char dataPacketFooter[2] = {ZERO, CES_CMDIF_PKT_STOP};
const char dataPacketHeader[5] = {CES_CMDIF_PKT_START_1, CES_CMDIF_PKT_START_2, 
                                  DATA_LEN, ZERO, CES_CMDIF_TYPE_DATA};

void setup() {
    Serial.begin(57600);  // ProtoCentral OpenView baud rate
    while (!Serial) {
        delay(10); // Wait for serial port to connect
    }
    
    Serial.println("===============================================");
    Serial.println("   MAX30001 ECG + BioZ Demo (New API)");
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
    
    Serial.println("\nStarting ECG + BioZ acquisition...");
    
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
    
    Serial.println("\n" + String("Sample Rate: ") + String(SAMPLE_RATE) + " SPS");
    Serial.println("ECG Gain: 80 V/V");
    Serial.println("BioZ samples at half the ECG rate");
    Serial.println("\nStreaming data in ProtoCentral OpenView format...");
    Serial.println("Open ProtoCentral OpenView software to visualize waveforms\n");
    
    // Small delay before starting acquisition
    delay(500);
}

/**
 * @brief Send data packet in ProtoCentral OpenView format
 * @param ecg_sample ECG sample value (signed 32-bit)
 * @param bioz_sample BioZ sample value (signed 32-bit)
 * @param bioz_skip Flag indicating if BioZ sample was skipped
 */
void sendDataPacket(int32_t ecg_sample, int32_t bioz_sample, bool bioz_skip) {
    // Pack ECG data (4 bytes, LSB first)
    dataPacket[0] = ecg_sample;
    dataPacket[1] = ecg_sample >> 8;
    dataPacket[2] = ecg_sample >> 16;
    dataPacket[3] = ecg_sample >> 24;
    
    // Pack BioZ data (4 bytes, LSB first)
    dataPacket[4] = bioz_sample;
    dataPacket[5] = bioz_sample >> 8;
    dataPacket[6] = bioz_sample >> 16;
    dataPacket[7] = bioz_sample >> 24;
    
    // BioZ skip flag
    dataPacket[8] = bioz_skip ? 0xFF : 0x00;
    
    // Reserved bytes
    dataPacket[9] = 0x00;
    dataPacket[10] = 0x00;
    dataPacket[11] = 0x00;
    
    // Send packet header
    for (int i = 0; i < 5; i++) {
        Serial.write(dataPacketHeader[i]);
    }
    
    // Send data payload
    for (int i = 0; i < DATA_LEN; i++) {
        Serial.write(dataPacket[i]);
    }
    
    // Send packet footer
    for (int i = 0; i < 2; i++) {
        Serial.write(dataPacketFooter[i]);
    }
}

void loop() {
    max30001_ecg_sample_t ecgSample;
    max30001_bioz_sample_t biozSample;
    int32_t ecg_value = 0;
    // Start with the last known BioZ value so skipped frames repeat the previous value
    int32_t bioz_value = lastBioZ;
    
    // Get ECG sample
    max30001_error_t ecgResult = ecgSensor.getECGSample(&ecgSample);
    
    if (ecgResult == MAX30001_SUCCESS && ecgSample.sample_valid) {
        ecg_value = ecgSample.ecg_sample;
        
        // Handle BioZ sampling (every other ECG sample).
        // If we skip a BioZ sample, reuse the last valid value to avoid sawtooth artifacts.
        if (!skipBioZSample) {
            max30001_error_t biozResult = ecgSensor.getBioZSample(&biozSample);

            if (biozResult == MAX30001_SUCCESS && biozSample.sample_valid) {
                bioz_value = biozSample.bioz_sample;
                lastBioZ = bioz_value; // update held value
            } else {
                // Keep lastBioZ (no update)
                bioz_value = lastBioZ;
            }
        } else {
            // Reuse last valid BioZ value for skipped frames
            bioz_value = lastBioZ;
        }
        
        // Send data in OpenView format
        sendDataPacket(ecg_value, bioz_value, skipBioZSample);
        
        // Toggle BioZ sampling flag
        skipBioZSample = !skipBioZSample;
    }
    
    // Delay between samples (should match sample rate)
    // For 128 SPS: ~8ms between samples
    delay(SAMPLE_DELAY_MS);
}
