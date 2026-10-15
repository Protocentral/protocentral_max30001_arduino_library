// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: Copyright (c) 2025 Ashwin Whitchurch, Protocentral Electronics

/*
 * Lead-Off Detection Demo for MAX30001
 *
 * Copyright (c) 2025 Ashwin Whitchurch, Protocentral Electronics
 * Email: info@protocentral.com
 *
 * This example demonstrates DC lead-off detection. Every ECG start enables a
 * 10 nA DC current on ECGP/ECGN; when an electrode comes off, its input drifts
 * past the VMID +/- 300 mV comparator threshold and the chip flags it.
 *
 * This software is licensed under the MIT License.
 */

//////////////////////////////////////////////////////////////////////////////////////////
//
//    Output (115200 baud), twice per second:
//      - ECG value in microvolts
//      - Connection status of each electrode (ECGP / ECGN)
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
//    Detection takes a moment after an electrode is removed or re-attached,
//    while the input settles past the threshold.
//
//////////////////////////////////////////////////////////////////////////////////////////

#include <SPI.h>
#include <protocentral_max30001.h>

#define MAX30001_CS_PIN 7
#define SAMPLE_RATE MAX30001_RATE_128
#define ECG_GAIN MAX30001_ECG_GAIN_80

MAX30001 ecgSensor(MAX30001_CS_PIN);

unsigned long lastStatusUpdate = 0;
const unsigned long STATUS_UPDATE_INTERVAL = 500;  // Update display every 500ms
int32_t lastECG = 0;

void setup() {
    Serial.begin(115200);
    while (!Serial) {
        delay(10);  // Wait for serial port to connect
    }

    Serial.println("===============================================");
    Serial.println("   MAX30001 Lead-Off Detection Demo");
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

    result = ecgSensor.startECG(SAMPLE_RATE, ECG_GAIN);
    if (result != MAX30001_SUCCESS) {
        Serial.print("✗ Failed to start ECG. Error code: ");
        Serial.println(result);
        while (1) {
            delay(1000);
        }
    }

    Serial.println("✓ ECG acquisition started (128 SPS, 80 V/V)");
    Serial.println("✓ DC lead-off detection enabled (10 nA, VMID +/- 300 mV)");
    Serial.println("\nInstructions:");
    Serial.println("  - Connect the ECG electrodes");
    Serial.println("  - Disconnect one or both electrodes to test detection");
    Serial.println("  - Watch the electrode status in the output below\n");

    lastStatusUpdate = millis();
}

void loop() {
    max30001_ecg_sample_t ecgSample;

    while (ecgSensor.getECGSample(&ecgSample) == MAX30001_SUCCESS) {
        lastECG = ecgSample.ecg_sample;
    }

    unsigned long now = millis();
    if ((now - lastStatusUpdate) >= STATUS_UPDATE_INTERVAL) {
        max30001_leadoff_t leadoff;
        ecgSensor.getLeadOffDetail(&leadoff);

        bool ecgpOff = leadoff.p_high || leadoff.p_low;
        bool ecgnOff = leadoff.n_high || leadoff.n_low;

        Serial.print("[ECG] ");
        Serial.print(ecgSensor.convertECGToMicrovolts(lastECG, ECG_GAIN), 1);
        Serial.println(" µV");

        Serial.print("[ECGP] ");
        Serial.println(ecgpOff ? "○ Disconnected" : "● Connected");
        Serial.print("[ECGN] ");
        Serial.println(ecgnOff ? "○ Disconnected" : "● Connected");
        Serial.println("---");

        lastStatusUpdate = now;
    }

    delay(10);
}
