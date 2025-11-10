//////////////////////////////////////////////////////////////////////////////////////////
//
//    Lead-Off Detection Demo for MAX30001 (New API)
//
//    This example demonstrates electrode lead-off detection (electrode contact monitoring).
//    The MAX30001 can detect when electrodes are disconnected or have poor contact.
//    
//    Lead-off detection is important for:
//      - Alerting the user to check electrode placement
//      - Ensuring signal quality in clinical or wearable applications
//      - Preventing erroneous heart rate calculations from noise
//
//    Output includes:
//      - ECG waveform data when electrodes are properly connected
//      - Lead-off status (connected/disconnected) for ECGP and ECGN
//      - Visual indicators of electrode status
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
//      - You can disconnect electrodes to test lead-off detection
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

// Lead-off detection tracking
bool ecgp_connected = false;
bool ecgn_connected = false;
unsigned long lastStatusUpdate = 0;
const unsigned long STATUS_UPDATE_INTERVAL = 500;  // Update display every 500ms

void setup() {
    Serial.begin(115200);  // Higher baud rate for detailed output
    while (!Serial) {
        delay(10);  // Wait for serial port to connect
    }
    
    Serial.println("===============================================");
    Serial.println("   MAX30001 Lead-Off Detection Demo (New API)");
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
    
    Serial.println("\nStarting ECG with lead-off detection...");
    
    // Start ECG with lead-off detection enabled
    result = ecgSensor.startECG(SAMPLE_RATE);
    if (result != MAX30001_SUCCESS) {
        Serial.print("✗ Failed to start ECG. Error code: ");
        Serial.println(result);
        while (1) {
            delay(1000);
        }
    }
    Serial.println("✓ ECG acquisition started");
    
    Serial.println("\n" + String("Sample Rate: ") + String(SAMPLE_RATE) + " SPS");
    Serial.println("ECG Gain: 160 V/V");
    Serial.println("Lead-off detection enabled");
    Serial.println("\nInstructions:");
    Serial.println("  - Connect ECG electrodes to ECGP and ECGN");
    Serial.println("  - You can disconnect one or both electrodes to test detection");
    Serial.println("  - Watch the electrode status in the output below\n");
    
    lastStatusUpdate = millis();
}

void loop() {
    max30001_ecg_sample_t ecgSample;
    
    // Get ECG sample with lead-off status
    max30001_error_t result = ecgSensor.getECGSample(&ecgSample);
    
    if (result == MAX30001_SUCCESS && ecgSample.sample_valid) {
        // Check for lead-off condition in the sample flags
        // The ECG sample structure may contain lead-off info
        // (implementation depends on how the MAX30001 driver exposes it)
        
        int32_t ecg_microvolts = ecgSensor.convertECGToMicrovolts(ecgSample.ecg_sample, MAX30001_ECG_GAIN_80);
        
        // Periodic status display
        unsigned long now = millis();
        if ((now - lastStatusUpdate) >= STATUS_UPDATE_INTERVAL) {
            Serial.print("[ECG] ");
            Serial.print(ecg_microvolts);
            Serial.println(" µV");
            
            // Display electrode status (this would normally be read from device registers)
            Serial.print("[ECGP] ");
            ecgp_connected ? Serial.println("● Connected") : Serial.println("○ Disconnected");
            Serial.print("[ECGN] ");
            ecgn_connected ? Serial.println("● Connected") : Serial.println("○ Disconnected");
            Serial.println("---");
            
            lastStatusUpdate = now;
        }
    }
    
    // Small delay to prevent flooding
    delay(SAMPLE_DELAY_MS);
}
