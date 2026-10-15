// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: Copyright (c) 2025 Ashwin Whitchurch, Protocentral Electronics

/*
 * Interrupt-Driven Acquisition Demo for MAX30001
 *
 * Copyright (c) 2025 Ashwin Whitchurch, Protocentral Electronics
 * Email: info@protocentral.com
 *
 * Instead of polling, the MAX30001 pulls its INTB pin low once its FIFOs reach
 * their thresholds (16 ECG samples or 4 BioZ samples). The sketch then reads
 * every buffered sample in one go, which leaves the MCU free between batches.
 *
 * This software is licensed under the MIT License.
 */

//////////////////////////////////////////////////////////////////////////////////////////
//
//    Output (115200 baud):
//      - Batch summary on each interrupt (ECG and BioZ samples read, last values)
//      - Data rate metrics every 5 seconds (expect ~128 ECG and ~64 BioZ samples/s)
//
//    Hardware Setup:
//      Arduino Uno/Mega:
//        - MISO: D12
//        - MOSI: D11
//        - SCLK: D13
//        - CS: D7
//        - INTB: D2 (breakout header pin INTB to Arduino D2)
//        - VCC: +5V
//        - GND: GND
//
//      ESP32: use any GPIO for INTB and set MAX30001_INT_PIN to match
//
//////////////////////////////////////////////////////////////////////////////////////////

#include <SPI.h>
#include <protocentral_max30001.h>

#define MAX30001_CS_PIN 7
#define MAX30001_INT_PIN 2
#define SAMPLE_RATE MAX30001_RATE_128
#define ECG_GAIN MAX30001_ECG_GAIN_80

MAX30001 ecgSensor(MAX30001_CS_PIN);

volatile bool dataAvailable = false;
volatile uint32_t interruptCount = 0;

uint32_t ecgSamplesRead = 0;
uint32_t biozSamplesRead = 0;
unsigned long startTime = 0;
unsigned long lastMetricsTime = 0;
const unsigned long METRICS_INTERVAL = 5000;  // Display metrics every 5 seconds

/**
 * @brief Interrupt service routine for the MAX30001 INTB pin
 */
void handleMAX30001Interrupt() {
    dataAvailable = true;
    interruptCount++;
}

void setup() {
    Serial.begin(115200);
    while (!Serial) {
        delay(10);  // Wait for serial port to connect
    }

    Serial.println("===============================================");
    Serial.println("   MAX30001 Interrupt-Driven Demo");
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

    result = ecgSensor.startECGBioZ(SAMPLE_RATE, ECG_GAIN);
    if (result != MAX30001_SUCCESS) {
        Serial.print("✗ Failed to start acquisition. Error code: ");
        Serial.println(result);
        while (1) {
            delay(1000);
        }
    }

    pinMode(MAX30001_INT_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(MAX30001_INT_PIN), handleMAX30001Interrupt, FALLING);
    ecgSensor.enableInterrupt();

    Serial.println("✓ ECG + BioZ acquisition started (ECG 128 SPS, 80 V/V; BioZ 64 SPS)");
    Serial.print("✓ INTB interrupt on pin D");
    Serial.println(MAX30001_INT_PIN);
    Serial.println();

    startTime = millis();
    lastMetricsTime = startTime;
}

void loop() {
    // INTB stays low until the FIFOs are drained, so also drain if it is already
    // low (an edge may have been missed before the interrupt was attached)
    if (dataAvailable || digitalRead(MAX30001_INT_PIN) == LOW) {
        dataAvailable = false;

        max30001_ecg_sample_t ecgSample;
        max30001_bioz_sample_t biozSample;
        uint8_t ecgCount = 0;
        uint8_t biozCount = 0;
        int32_t lastECG = 0;
        int32_t lastBioZ = 0;

        while (ecgSensor.getECGSample(&ecgSample) == MAX30001_SUCCESS) {
            lastECG = ecgSample.ecg_sample;
            ecgCount++;
        }
        while (ecgSensor.getBioZSample(&biozSample) == MAX30001_SUCCESS) {
            lastBioZ = biozSample.bioz_sample;
            biozCount++;
        }

        ecgSamplesRead += ecgCount;
        biozSamplesRead += biozCount;

        if (ecgCount > 0) {
            Serial.print("Batch: ");
            Serial.print(ecgCount);
            Serial.print(" ECG, ");
            Serial.print(biozCount);
            Serial.print(" BioZ | Last ECG: ");
            Serial.print(ecgSensor.convertECGToMicrovolts(lastECG, ECG_GAIN), 1);
            Serial.print(" µV | Last BioZ: ");
            Serial.println(lastBioZ);
        }
    }

    unsigned long now = millis();
    if ((now - lastMetricsTime) >= METRICS_INTERVAL) {
        float elapsed = (now - startTime) / 1000.0;

        Serial.println("\n--- Performance Metrics ---");
        Serial.print("ECG samples/s:  ");
        Serial.println(ecgSamplesRead / elapsed, 1);
        Serial.print("BioZ samples/s: ");
        Serial.println(biozSamplesRead / elapsed, 1);
        Serial.print("Interrupts/s:   ");
        Serial.println(interruptCount / elapsed, 1);
        Serial.println("---------------------------\n");

        lastMetricsTime = now;
    }
}
