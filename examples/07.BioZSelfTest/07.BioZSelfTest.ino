// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: Copyright (c) 2026 Ashwin Whitchurch, Protocentral Electronics

/*
 * Self-test for the MAX30001 BioZ and ECG channels - no person needed
 *
 * Copyright (c) 2026 Ashwin Whitchurch, Protocentral Electronics
 * Email: info@protocentral.com
 *
 * BioZ: the chip disconnects BIP/BIN and switches in its built-in test
 * resistor, then steps it by a known amount once a second. The sketch measures
 * the nominal value and the step and compares both with the datasheet values.
 * The step-to-nominal ratio does not depend on the drive-current accuracy, so
 * it is the tighter check.
 *
 * ECG: the chip disconnects ECGP/ECGN and applies its internal calibration
 * square wave; the sketch measures its amplitude.
 *
 * Electrodes can stay connected - the inputs are isolated during the test.
 *
 * This software is licensed under the MIT License.
 */

//////////////////////////////////////////////////////////////////////////////////////////
//
//    Output (115200 baud): measured and expected values, PASS/FAIL per test
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
//////////////////////////////////////////////////////////////////////////////////////////

#include <SPI.h>
#include <protocentral_max30001.h>

#define MAX30001_CS_PIN 7

// 5000 ohm nominal with the largest step (2.96 ohm): 80 mV at 16 uA, inside the input range
#define TEST_RNOM MAX30001_BIOZ_RNOM_5000
#define TEST_RMOD MAX30001_BIOZ_RMOD_0

// The drive current is +/-30 % with the internal bias; the ratio check removes that
const float NOMINAL_TOLERANCE = 0.35;
const float RATIO_TOLERANCE = 0.15;
const float ECG_TOLERANCE = 0.25;

MAX30001 sensor(MAX30001_CS_PIN);

// Smoothed min/max over a few modulation cycles; returns the mean in *mean
float measureStep(bool bioz, unsigned long duration_ms, float* mean)
{
    const uint8_t AVG = 8;
    float window[AVG] = {0};
    uint8_t idx = 0;
    uint16_t count = 0;
    float sum = 0, lo = 1e9, hi = -1e9;

    max30001_ecg_sample_t ecg;
    max30001_bioz_sample_t bz;

    unsigned long start = millis();
    while (millis() - start < duration_ms) {
        float value;
        if (bioz) {
            if (sensor.getBioZSample(&bz) != MAX30001_SUCCESS) continue;
            value = sensor.convertBioZToOhms(bz.bioz_sample);
        } else {
            if (sensor.getECGSample(&ecg) != MAX30001_SUCCESS) continue;
            value = sensor.convertECGToMicrovolts(ecg.ecg_sample, MAX30001_ECG_GAIN_80);
        }

        sum += value;
        window[idx] = value;
        idx = (idx + 1) % AVG;
        if (++count >= AVG) {
            float avg = 0;
            for (uint8_t i = 0; i < AVG; i++) avg += window[i];
            avg /= AVG;
            if (avg < lo) lo = avg;
            if (avg > hi) hi = avg;
        }
    }

    *mean = count ? sum / count : 0;
    return (count > AVG) ? hi - lo : 0;
}

bool report(const __FlashStringHelper* name, float measured, float expected, float tolerance, const __FlashStringHelper* unit)
{
    bool pass = expected > 0 && fabs(measured - expected) <= tolerance * expected;

    Serial.print(pass ? F("PASS  ") : F("FAIL  "));
    Serial.print(name);
    Serial.print(F(": "));
    Serial.print(measured, 3);
    Serial.print(F(" (expected "));
    Serial.print(expected, 3);
    Serial.print(F(" +/- "));
    Serial.print(tolerance * 100, 0);
    Serial.print(F(" %) "));
    Serial.println(unit);
    return pass;
}

void drain()
{
    max30001_ecg_sample_t ecg;
    max30001_bioz_sample_t bz;
    while (sensor.getECGSample(&ecg) == MAX30001_SUCCESS) {
    }
    while (sensor.getBioZSample(&bz) == MAX30001_SUCCESS) {
    }
}

void setup()
{
    Serial.begin(115200);
    while (!Serial) {
        delay(10);
    }

    Serial.println(F("MAX30001 self-test"));
    Serial.println(F("=================="));

    if (sensor.begin() != MAX30001_SUCCESS) {
        Serial.println(F("FAIL  MAX30001 not found - check wiring"));
        while (1) {
            delay(1000);
        }
    }
    Serial.println(F("PASS  MAX30001 found"));

    bool allPass = true;

    // --- BioZ test resistor ---------------------------------------------------
    sensor.startBioZ();
    sensor.setBioZHighPassFilter(0);   // keep the DC (nominal) value
    sensor.enableBioZTestLoad(TEST_RNOM, TEST_RMOD, MAX30001_BIOZ_TEST_FREQ_1HZ);
    delay(1000);
    drain();

    float nominal;
    float step = measureStep(true, 4000, &nominal);
    float expectedNominal = MAX30001::bioZTestLoadOhms(TEST_RNOM);
    float expectedStep = MAX30001::bioZTestLoadModulationOhms(TEST_RNOM, TEST_RMOD);

    allPass &= report(F("BioZ nominal"), nominal, expectedNominal, NOMINAL_TOLERANCE, F("ohm"));
    allPass &= report(F("BioZ step"), step, expectedStep, NOMINAL_TOLERANCE, F("ohm"));
    allPass &= report(F("BioZ step/nominal"), nominal > 0 ? step / nominal * 1000 : 0,
                      expectedStep / expectedNominal * 1000, RATIO_TOLERANCE, F("x 1e-3"));

    sensor.disableBioZTestLoad();

    // --- ECG calibration signal -----------------------------------------------
    // VCALP and VCALN swing in opposite directions by 0.5 mV, so ECGP - ECGN is
    // a +/-1 mV square wave (2 mV peak-to-peak)
    sensor.startECG(MAX30001_RATE_128, MAX30001_ECG_GAIN_80);
    sensor.setECGHighPassFilter(0);    // a 0.5 Hz high-pass would droop the square wave
    sensor.enableECGTestSignal(MAX30001_CAL_FREQ_1HZ, true);
    delay(1000);
    drain();

    float ecgMean;
    float ecgPP = measureStep(false, 4000, &ecgMean);
    allPass &= report(F("ECG test signal p-p"), ecgPP / 1000.0, 2.0, ECG_TOLERANCE, F("mV"));

    sensor.disableECGTestSignal();
    sensor.stop();

    Serial.println();
    Serial.println(allPass ? F("Result: PASS") : F("Result: FAIL - see 08.Diagnostics for register values"));
}

void loop()
{
}
