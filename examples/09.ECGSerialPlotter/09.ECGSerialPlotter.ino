// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: Copyright (c) 2026 Ashwin Whitchurch, Protocentral Electronics

/*
 * ECG in microvolts for the Arduino Serial Plotter
 *
 * Copyright (c) 2026 Ashwin Whitchurch, Protocentral Electronics
 * Email: info@protocentral.com
 *
 * The simplest way to see an ECG: 128 samples per second, in microvolts,
 * with a lead-off flag.
 *
 * This software is licensed under the MIT License.
 */

//////////////////////////////////////////////////////////////////////////////////////////
//
//    Output (115200 baud), for Tools -> Serial Plotter:
//      ECG_uV    - ECG in microvolts (0.5 Hz high-pass, 28 Hz low-pass at 128 SPS)
//      LeadOff   - 1 while an electrode is off
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
//    left abdomen. Power from a battery while electrodes are on a person.
//
//////////////////////////////////////////////////////////////////////////////////////////

#include <SPI.h>
#include <protocentral_max30001.h>

#define MAX30001_CS_PIN 7
#define ECG_GAIN MAX30001_ECG_GAIN_80

MAX30001 sensor(MAX30001_CS_PIN);

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

    sensor.startECG(MAX30001_RATE_128, ECG_GAIN);
}

void loop()
{
    max30001_ecg_sample_t sample;

    while (sensor.getECGSample(&sample) == MAX30001_SUCCESS) {
        Serial.print(F("ECG_uV:"));
        Serial.print(sensor.convertECGToMicrovolts(sample.ecg_sample, ECG_GAIN), 1);
        Serial.print(F(",LeadOff:"));
        Serial.println(sample.lead_off_detected ? 1 : 0);
    }
}
