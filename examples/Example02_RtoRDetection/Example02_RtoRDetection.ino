//////////////////////////////////////////////////////////////////////////////////////////
//
//    R-R Interval Detection Demo for MAX30001 (New API)
//
//    This example demonstrates R-R interval (heartbeat timing) detection using the
//    hardware R-R detection feature built into the MAX30001.
//    
//    The MAX30001 has a dedicated R-wave detector that identifies peaks in the ECG
//    signal and calculates the R-R interval (time between consecutive heartbeats).
//    
//    Output includes:
//      - R-R interval in milliseconds
//      - Heart rate in beats per minute (BPM)
//      - Individual R-wave detection events
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
//        - INT1: D2 (optional, for interrupt-driven mode)
//        - VCC: +5V
//        - GND: GND
//
//      ESP32:
//        - MISO: GPIO 19
//        - MOSI: GPIO 23
//        - SCLK: GPIO 18
//        - CS: GPIO 5
//        - INT1: GPIO 4 (optional)
//        - VCC: +5V
//        - GND: GND
//
//    Electrode Connections:
//      - Connect ECG electrodes to ECGP and ECGN
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

// Measurement Configuration
#define SAMPLE_RATE MAX30001_RATE_128  // 128 samples per second
#define SAMPLE_DELAY_MS 8              // ~128 SPS

// Create sensor instance
MAX30001 ecgSensor(MAX30001_CS_PIN);

// Timing
unsigned long lastDisplayTime = 0;
const unsigned long DISPLAY_INTERVAL = 1000;  // Update display every 1 second

// R-R detection counters
uint32_t rr_count = 0;
uint32_t last_rr_interval = 0;
uint32_t heart_rate_bpm = 0;

void setup() {
    Serial.begin(115200);  // Higher baud rate for detailed output
    while (!Serial) {
        delay(10);  // Wait for serial port to connect
    }
    
    Serial.println("===============================================");
    Serial.println("   MAX30001 R-R Detection Demo (New API)");
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
    
    Serial.println("\nStarting ECG with R-R detection mode...");
    
    // Start ECG monitoring (includes hardware R-R detection)
    result = ecgSensor.startECG(SAMPLE_RATE, MAX30001_ECG_GAIN_160);
    if (result != MAX30001_SUCCESS) {
        Serial.print("✗ Failed to start ECG. Error code: ");
        Serial.println(result);
        while (1) {
            delay(1000);
        }
    }
    Serial.println("✓ ECG with R-R detection started");
    
    Serial.println("\n" + String("Sample Rate: ") + String(SAMPLE_RATE) + " SPS");
    Serial.println("ECG Gain: 160 V/V");
    Serial.println("R-wave detection enabled");
    Serial.println("\nWaiting for heartbeats...\n");
    
    lastDisplayTime = millis();
}

void loop() {
    max30001_rtor_data_t rtorData;
    
    // Get R-R interval data
    max30001_error_t result = ecgSensor.getRtoRData(&rtorData);
    
    if (result == MAX30001_SUCCESS && rtorData.rr_detected) {
        // New R-R data available
        rr_count++;
        last_rr_interval = rtorData.rr_interval_ms;
        
        // Calculate heart rate from R-R interval
        if (last_rr_interval > 0) {
            heart_rate_bpm = 60000 / last_rr_interval;
        }
        
        // Print immediately on R-wave detection
        Serial.print("♥ R-wave detected #");
        Serial.print(rr_count);
        Serial.print(" | RR Interval: ");
        Serial.print(last_rr_interval);
        Serial.print(" ms | Heart Rate: ");
        Serial.print(heart_rate_bpm);
        Serial.println(" BPM");
    }
    
    // Periodic status display
    unsigned long now = millis();
    if ((now - lastDisplayTime) >= DISPLAY_INTERVAL) {
        if (rr_count == 0) {
            Serial.println("⋯ Waiting for R-wave detection...");
        }
        lastDisplayTime = now;
    }
    
    // Small delay to prevent flooding
    delay(SAMPLE_DELAY_MS);
}
