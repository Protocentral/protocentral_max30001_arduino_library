// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: Copyright (c) 2026 Ashwin Whitchurch, Protocentral Electronics

/*
 * Respiration (BioZ) in ohms for the Arduino Serial Plotter
 *
 * Copyright (c) 2026 Ashwin Whitchurch, Protocentral Electronics
 * Email: info@protocentral.com
 *
 * The MAX30001 drives a small AC current through the RA/LA electrodes and
 * measures the voltage. As the chest expands, its impedance changes by a
 * fraction of an ohm; this sketch plots that change in ohms.
 *
 * At start-up it bypasses the high-pass filter for two seconds to measure the
 * total impedance (body + both electrodes). If that is close to the input
 * range, the respiration signal clips - use fresh gel electrodes, or lower the
 * drive current.
 *
 * This software is licensed under the MIT License.
 */

//////////////////////////////////////////////////////////////////////////////////////////
//
//    Output (115200 baud), for Tools -> Serial Plotter:
//      Respiration_ohm  - impedance change in ohms (0.05 Hz high-pass, 4 Hz low-pass)
//      Clipped          - 1 while the BioZ input is out of range
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
//    Electrodes: RA and LA on the chest below the collarbones, RL on the lower
//    left abdomen. Sit still and breathe normally.
//
//////////////////////////////////////////////////////////////////////////////////////////

#include <SPI.h>
#include <protocentral_max30001.h>

#define MAX30001_CS_PIN 7

MAX30001 sensor(MAX30001_CS_PIN);

// Keep drive current x total impedance under 90 mV peak (5.6 kOhm at 16 uA)
const float MAX_IMPEDANCE_OHM = 90e-3 / 16e-6;

float measureTotalImpedance(unsigned long duration_ms)
{
    max30001_bioz_sample_t sample;
    float sum = 0;
    uint16_t count = 0;

    // Let the readings settle after the filter change
    delay(500);
    while (sensor.getBioZSample(&sample) == MAX30001_SUCCESS) {
    }

    unsigned long start = millis();
    while (millis() - start < duration_ms) {
        while (sensor.getBioZSample(&sample) == MAX30001_SUCCESS) {
            sum += sensor.convertBioZToOhms(sample.bioz_sample);
            count++;
        }
    }
    return count ? sum / count : 0;
}

void setup()
{
    Serial.begin(115200);
    while (!Serial) {
        delay(10);
    }

    if (sensor.begin() != MAX30001_SUCCESS) {
        Serial.println(F("MAX30001 not found - check wiring"));
        while (1) {
            delay(1000);
        }
    }

    sensor.startBioZ();

    // Total impedance: bypass the high-pass filter so the DC value is kept
    sensor.setBioZHighPassFilter(0);
    float total = measureTotalImpedance(2000);

    Serial.print(F("Total impedance (body + electrodes): "));
    Serial.print(total, 0);
    Serial.println(F(" ohm"));
    if (total > 0.8 * MAX_IMPEDANCE_OHM) {
        Serial.println(F("Warning: close to the BioZ input range - use fresh gel electrodes"));
        Serial.println(F("or lower the drive current with setBioZCurrent(MAX30001_BIOZ_CURRENT_8UA)"));
    }

    // Respiration: back to the 0.05 Hz high-pass, which keeps breathing (0.1-0.5 Hz)
    sensor.setBioZHighPassFilter(0.05);
    delay(2000);
}

void loop()
{
    max30001_bioz_sample_t sample;

    while (sensor.getBioZSample(&sample) == MAX30001_SUCCESS) {
        Serial.print(F("Respiration_ohm:"));
        Serial.print(sensor.convertBioZToOhms(sample.bioz_sample), 3);
        Serial.print(F(",Clipped:"));
        Serial.println(sample.out_of_range ? 1 : 0);
    }
}
