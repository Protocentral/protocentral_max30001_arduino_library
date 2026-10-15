// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: Copyright (c) 2025 Ashwin Whitchurch, Protocentral Electronics

/*
 * ECG + BioZ streaming to ProtoCentral OpenView
 *
 * Copyright (c) 2025 Ashwin Whitchurch, Protocentral Electronics
 * Email: info@protocentral.com
 *
 * Streams ECG, BioZ (respiration), heart rate and R-R interval to ProtoCentral
 * OpenView (https://github.com/Protocentral/protocentral_openview).
 * In OpenView, choose the "MAX30001 ECG & BioZ Breakout" board and this serial port.
 *
 * Wire format (must match OpenView's max30001 descriptor - change both together):
 *   115200 baud, one packet per ECG sample (128 packets/s)
 *   0x0A 0xFA <len=12> 0x00 <type=0x02> | ECG int32 | BioZ int32 | HR int16 | RR int16 | 0x00 0x0B
 *   (all little-endian; BioZ runs at 64 SPS, so each BioZ value is sent twice)
 *
 * This sketch prints nothing but packets - any other text would corrupt the stream.
 * If the MAX30001 is not found, the built-in LED blinks (on boards that have one).
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
//      ESP32:
//        - MISO: GPIO 19
//        - MOSI: GPIO 23
//        - SCLK: GPIO 18
//        - CS: GPIO 5
//        - VCC: +5V
//        - GND: GND
//
//    Electrode Connections (3.5 mm cable):
//      - RA, LA and RL electrodes; ECG and 2-electrode BioZ share RA/LA
//
//////////////////////////////////////////////////////////////////////////////////////////

#include <SPI.h>
#include <protocentral_max30001.h>

// Pin Configuration
// Change to 5 if using ESP32
#define MAX30001_CS_PIN 7

// ProtoCentral OpenView packet format constants
const uint8_t PKT_START_1 = 0x0A;
const uint8_t PKT_START_2 = 0xFA;
const uint8_t PKT_TYPE_DATA = 0x02;
const uint8_t PKT_STOP = 0x0B;
const uint8_t PKT_PAYLOAD_LEN = 12;

MAX30001 ecgSensor(MAX30001_CS_PIN);

int32_t lastBioZ = 0;
uint16_t heartRate = 0;
uint16_t rrInterval = 0;

void sendDataPacket(int32_t ecg, int32_t bioz, uint16_t hr, uint16_t rr)
{
    uint8_t packet[5 + PKT_PAYLOAD_LEN + 2];
    uint8_t i = 0;

    packet[i++] = PKT_START_1;
    packet[i++] = PKT_START_2;
    packet[i++] = PKT_PAYLOAD_LEN;
    packet[i++] = 0x00;
    packet[i++] = PKT_TYPE_DATA;

    for (uint8_t b = 0; b < 4; b++) packet[i++] = (uint8_t)(ecg >> (8 * b));
    for (uint8_t b = 0; b < 4; b++) packet[i++] = (uint8_t)(bioz >> (8 * b));
    packet[i++] = (uint8_t)hr;
    packet[i++] = (uint8_t)(hr >> 8);
    packet[i++] = (uint8_t)rr;
    packet[i++] = (uint8_t)(rr >> 8);

    packet[i++] = 0x00;
    packet[i++] = PKT_STOP;

    Serial.write(packet, i);
}

void setup()
{
    Serial.begin(115200);  // ProtoCentral OpenView baud rate

    if (ecgSensor.begin() != MAX30001_SUCCESS ||
        ecgSensor.startECGBioZ(MAX30001_RATE_128, MAX30001_ECG_GAIN_80) != MAX30001_SUCCESS) {
        // No text on the serial port: blink the built-in LED instead
        pinMode(LED_BUILTIN, OUTPUT);
        while (1) {
            digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
            delay(200);
        }
    }
}

void loop()
{
    max30001_ecg_sample_t ecgSample;
    max30001_bioz_sample_t biozSample;
    max30001_rtor_data_t rtorData;

    // BioZ (64 SPS): keep the newest value
    while (ecgSensor.getBioZSample(&biozSample) == MAX30001_SUCCESS) {
        lastBioZ = biozSample.bioz_sample;
    }

    // R-to-R: update HR/RR when a new beat is detected
    if (ecgSensor.getRtoRData(&rtorData) == MAX30001_SUCCESS && rtorData.rr_detected) {
        heartRate = rtorData.heart_rate_bpm;
        rrInterval = rtorData.rr_interval_ms;
    }

    // ECG (128 SPS): one packet per sample
    while (ecgSensor.getECGSample(&ecgSample) == MAX30001_SUCCESS) {
        sendDataPacket(ecgSample.ecg_sample, lastBioZ, heartRate, rrInterval);
    }
}
