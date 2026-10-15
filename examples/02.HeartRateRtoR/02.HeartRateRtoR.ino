// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: Copyright (c) 2025 Ashwin Whitchurch, Protocentral Electronics

/*
 * R-R Interval Detection Demo for MAX30001
 *
 * Copyright (c) 2025 Ashwin Whitchurch, Protocentral Electronics
 * Email: info@protocentral.com
 *
 * This example demonstrates R-R interval (heartbeat timing) detection using the
 * hardware R-R detection feature built into the MAX30001.
 *
 * The MAX30001 has a dedicated R-wave detector that identifies peaks in the ECG
 * signal and calculates the R-R interval (time between consecutive heartbeats).
 * The detector runs on the chip; this sketch only reads the result when the
 * chip flags a new beat.
 *
 * This software is licensed under the MIT License.
 */

//////////////////////////////////////////////////////////////////////////////////////////
//
//    Output (115200 baud), one line per detected beat:
//      - Beat count
//      - R-R interval in milliseconds
//      - Heart rate in beats per minute (BPM)
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
//    The detector needs a few beats to adapt its threshold before the first result.
//
//////////////////////////////////////////////////////////////////////////////////////////

#include <SPI.h>
#include <protocentral_max30001.h>

#define MAX30001_CS_PIN 7
#define SAMPLE_RATE MAX30001_RATE_128

MAX30001 ecgSensor(MAX30001_CS_PIN);

unsigned long lastWaitMessage = 0;
const unsigned long WAIT_MESSAGE_INTERVAL = 2000;
uint32_t beatCount = 0;

void setup() {
    Serial.begin(115200);
    while (!Serial) {
        delay(10);  // Wait for serial port to connect
    }

    Serial.println("===============================================");
    Serial.println("   MAX30001 R-R Detection Demo");
    Serial.println("===============================================\n");

    Serial.println("Initializing MAX30001...");

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

    max30001_device_info_t deviceInfo;
    ecgSensor.getDeviceInfo(&deviceInfo);
    Serial.print("✓ MAX30001 found - Part ID: 0x");
    Serial.print(deviceInfo.part_id, HEX);
    Serial.print(", Revision: 0x");
    Serial.println(deviceInfo.revision, HEX);

    result = ecgSensor.startECG(SAMPLE_RATE, MAX30001_ECG_GAIN_160);
    if (result != MAX30001_SUCCESS) {
        Serial.print("✗ Failed to start ECG. Error code: ");
        Serial.println(result);
        while (1) {
            delay(1000);
        }
    }

    Serial.println("✓ ECG with R-R detection started (128 SPS, 160 V/V)");
    Serial.println("\nWaiting for heartbeats...\n");

    lastWaitMessage = millis();
}

void loop() {
    max30001_ecg_sample_t ecgSample;
    max30001_rtor_data_t rtorData;

    // Keep the ECG FIFO drained so it does not overflow
    while (ecgSensor.getECGSample(&ecgSample) == MAX30001_SUCCESS) {
    }

    if (ecgSensor.getRtoRData(&rtorData) == MAX30001_SUCCESS && rtorData.rr_detected) {
        beatCount++;

        Serial.print("♥ Beat #");
        Serial.print(beatCount);
        Serial.print(" | RR Interval: ");
        Serial.print(rtorData.rr_interval_ms);
        Serial.print(" ms | Heart Rate: ");
        Serial.print(rtorData.heart_rate_bpm);
        Serial.println(" BPM");
    }

    if (beatCount == 0 && (millis() - lastWaitMessage) >= WAIT_MESSAGE_INTERVAL) {
        Serial.println("⋯ Waiting for R-wave detection...");
        lastWaitMessage = millis();
    }

    delay(10);
}
