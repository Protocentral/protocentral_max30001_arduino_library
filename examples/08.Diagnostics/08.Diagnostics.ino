// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: Copyright (c) 2026 Ashwin Whitchurch, Protocentral Electronics

/*
 * Diagnostics for the MAX30001 - run this first if something is wrong
 *
 * Copyright (c) 2026 Ashwin Whitchurch, Protocentral Electronics
 * Email: info@protocentral.com
 *
 * Checks, in order:
 *   1. Library version, SPI link and chip ID
 *   2. Register dump after starting ECG + BioZ
 *   3. Data path: ECG and BioZ sample rates over 5 seconds (expect ~128 and ~64)
 *   4. Status flags: FIFO overflows, BioZ out of range, PLL lock, lead-off
 *   5. Electrodes: total BioZ impedance against the input range
 *
 * Run with the electrodes on for steps 4 and 5; steps 1-3 work without them.
 *
 * This software is licensed under the MIT License.
 */

//////////////////////////////////////////////////////////////////////////////////////////
//
//    Output (115200 baud): one OK / WARN / FAIL line per check
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

MAX30001 sensor(MAX30001_CS_PIN);

void line(const __FlashStringHelper* level, const __FlashStringHelper* text)
{
    Serial.print(level);
    Serial.print(F("  "));
    Serial.println(text);
}

void setup()
{
    Serial.begin(115200);
    while (!Serial) {
        delay(10);
    }

    Serial.println(F("MAX30001 diagnostics"));
    Serial.println(F("===================="));
    Serial.print(F("Library version "));
    Serial.println(MAX30001::getLibraryVersion());

    // --- 1. SPI link and chip ID ----------------------------------------------
    if (sensor.begin() != MAX30001_SUCCESS) {
        line(F("FAIL"), F("No MAX30001 on SPI (ID register does not read 0x5x)"));
        line(F("    "), F("Check CS/SCLK/MOSI/MISO, VDD (5 V on Uno, 3.3 V on ESP32) and GND"));
        while (1) {
            delay(1000);
        }
    }

    max30001_device_info_t info;
    sensor.getDeviceInfo(&info);
    Serial.print(F("OK    MAX30001 found, part ID 0x"));
    Serial.print(info.part_id, HEX);
    Serial.print(F(", revision 0x"));
    Serial.println(info.revision, HEX);

    // --- 2. Registers -----------------------------------------------------------
    sensor.startECGBioZ(MAX30001_RATE_128, MAX30001_ECG_GAIN_80);
    sensor.setBioZLeadOff(true, false);  // flag BioZ over-range in getStatus()
    Serial.println(F("\nRegisters after startECGBioZ():"));
    sensor.printRegisters(Serial);
    Serial.println();

    max30001_status_flags_t flags;
    delay(200);
    sensor.getStatus(&flags);           // clear flags from start-up

    // --- 3. Data path -----------------------------------------------------------
    // Bypass the BioZ high-pass so the samples keep the total impedance (step 5)
    sensor.setBioZHighPassFilter(0);

    max30001_ecg_sample_t ecg;
    max30001_bioz_sample_t bz;
    uint16_t ecgCount = 0, biozCount = 0, clipped = 0;
    float biozSum = 0;

    unsigned long start = millis();
    while (millis() - start < 5000) {
        while (sensor.getECGSample(&ecg) == MAX30001_SUCCESS) {
            ecgCount++;
        }
        while (sensor.getBioZSample(&bz) == MAX30001_SUCCESS) {
            biozCount++;
            biozSum += sensor.convertBioZToOhms(bz.bioz_sample);
            if (bz.out_of_range) clipped++;
        }
        delay(5);
    }

    float ecgRate = ecgCount / 5.0;
    float biozRate = biozCount / 5.0;
    Serial.print(F("ECG  "));
    Serial.print(ecgRate, 1);
    Serial.println(F(" samples/s (expect ~128)"));
    Serial.print(F("BioZ "));
    Serial.print(biozRate, 1);
    Serial.println(F(" samples/s (expect ~64)"));

    if (ecgRate < 120 || ecgRate > 136 || biozRate < 60 || biozRate > 68) {
        line(F("FAIL"), F("Sample rates are wrong - check the FCLK oscillator on the board"));
    } else {
        line(F("OK  "), F("Data path"));
    }

    // --- 4. Status flags --------------------------------------------------------
    sensor.getStatus(&flags);
    if (flags.pll_unlocked) line(F("FAIL"), F("PLL unlocked - no 32.768 kHz clock on FCLK"));
    if (flags.ecg_fifo_overflow || flags.bioz_fifo_overflow) line(F("WARN"), F("FIFO overflow - the host is not reading fast enough"));
    if (flags.fast_recovery) line(F("WARN"), F("ECG fast recovery engaged - the ECG input saturated"));
    if (flags.bioz_over_range || clipped) line(F("WARN"), F("BioZ out of range - impedance too high for the drive current"));

    max30001_leadoff_t leadoff;
    sensor.getLeadOffDetail(&leadoff);
    if (leadoff.p_high || leadoff.p_low) line(F("WARN"), F("ECGP (LA) electrode off"));
    if (leadoff.n_high || leadoff.n_low) line(F("WARN"), F("ECGN (RA) electrode off"));
    if (!leadoff.any) line(F("OK  "), F("Both ECG electrodes on"));

    // --- 5. Electrode impedance -------------------------------------------------
    float total = biozCount ? biozSum / biozCount : 0;
    float limit = 90e-3 / 16e-6;  // 90 mV input range at 16 uA
    Serial.print(F("BioZ total impedance (body + electrodes): "));
    Serial.print(total, 0);
    Serial.println(F(" ohm"));
    if (total > 0.8 * limit) {
        line(F("WARN"), F("Close to the BioZ input range - fresh gel electrodes, or setBioZCurrent(8 uA)"));
    } else if (leadoff.any) {
        line(F("    "), F("(electrodes off - the impedance reading is not meaningful)"));
    } else {
        line(F("OK  "), F("Electrode impedance"));
    }

    sensor.stop();
    Serial.println(F("\nDone. For a test without a person, run 07.BioZSelfTest."));
}

void loop()
{
}
