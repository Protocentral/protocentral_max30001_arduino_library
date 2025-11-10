# MAX30001 Troubleshooting Guide

This guide addresses common issues and provides systematic debugging strategies.

## Quick Diagnosis

### Device Not Responding
**Try this first:**
```cpp
Serial.println(sensor.isConnected() ? "Connected" : "Not responding");
```

### Signal Problems
**Try this first:**
```cpp
max30001_ecg_sample_t ecg;
if (sensor.getECGSample(&ecg) == MAX30001_SUCCESS) {
    Serial.println(ecg.lead_off_detected ? "Lead-off" : "OK");
    Serial.println(ecg.sample_valid ? "Valid" : "Invalid");
}
```

---

## Common Issues & Solutions

### Issue 1: "Device Not Responding" / SPI Communication Fails

**Symptoms:**
- `sensor.isConnected()` returns `false`
- `sensor.begin()` returns `MAX30001_ERROR_SPI_COMMUNICATION`
- No data from `getECGSample()`

**Debugging Steps:**

1. **Verify Power Supply**
   ```cpp
   // Measure voltage at MAX30001 VCC pin with multimeter
   // Should read: 5.0V (±0.1V) for 5V systems
   //             3.3V (±0.1V) for 3.3V systems
   ```
   - Check VCC pin with multimeter
   - Verify GND connection with ohm meter

2. **Check SPI Wiring**
   ```
   Check connections:
   MAX30001 MISO → Arduino D12 ✓
   MAX30001 MOSI → Arduino D11 ✓
   MAX30001 SCK  → Arduino D13 ✓
   MAX30001 GND  → Arduino GND  ✓
   ```

3. **Verify CS Pin**
   ```cpp
   // Confirm CS pin matches constructor
   MAX30001 sensor(7);  // D7 = CS
   
   // Verify pin is connected to MAX30001 CS pin
   // Test manually:
   digitalWrite(7, HIGH);
   digitalWrite(7, LOW);
   // CS line should toggle on oscilloscope
   ```

4. **Add Decoupling Capacitor**
   ```
   Add between VCC and GND:
   ┌─────────────────────┐
   │  10µF Capacitor     │
   │   ────||────        │
   │   │            │    │
   MAX30001          GND
   VCC
   ```
   - Large capacitors (10µF) for low-frequency filtering
   - Small capacitors (0.1µF) for high-frequency noise
   - Both together is ideal

5. **Check Board Compatibility**
   - Verify your Arduino board uses standard SPI pins (D11, D12, D13)
   - For Mega: SPI pins are D50 (MISO), D51 (MOSI), D52 (SCK)
   - For ESP32: Default VSPI uses GPIO 19, 23, 18

**Still Not Working?**
   - Try with external SPI interface: `MAX30001 sensor(7, &SPI1);`
   - Test CS pin independently: ensure it's toggling
   - Measure voltage with oscilloscope on MOSI/MISO during communication

---

### Issue 2: Noisy ECG Signal

**Symptoms:**
- Waveform appears very noisy, baseline unstable
- Difficult to identify heart rate pattern
- Signal looks like random noise

**Debugging:**

1. **Check Electrode Contact**
   ```cpp
   if (sensor.getLeadOffStatus()) {
       Serial.println("✗ Lead-off detected!");
       Serial.println("Action: Press electrodes firmly for 10+ seconds");
   }
   ```
   - Press electrodes on skin firmly for 10 seconds
   - Ensure good skin contact across entire electrode pad
   - Electrodes should not move during measurement

2. **Inspect Electrode Quality**
   - Expired electrodes: Replace if older than 2 years
   - Dried-out gel: Discard and use fresh electrodes
   - Dry skin: Dampen skin lightly (not wet), allow to settle 30 seconds

3. **Reduce Gain (for high-amplitude signals)**
   ```cpp
   // Reduce amplification if clipping
   sensor.setECGGain(MAX30001_ECG_GAIN_80);
   ```

4. **Adjust Filters**
   ```cpp
   // Increase low-pass filter to remove high-frequency noise
   sensor.setECGLowPassFilter(40);  // Strong filtering
   
   // Increase high-pass filter to remove slow baseline drift
   sensor.setECGHighPassFilter(0.5);
   ```

5. **Isolate EMI (Electromagnetic Interference)**
   - Keep electrode wires away from power cables
   - Avoid USB cable near sensor wires
   - Shield wires in foil if near high-EMI environment
   - Use different power supply if available

6. **Check Power Supply Quality**
   ```
   Measure voltage ripple with oscilloscope:
   Acceptable: < 10mV ripple
   Problematic: > 50mV ripple
   
   Solutions:
   - Use USB hub power (cleaner than computer USB)
   - Add decoupling capacitor (10µF + 0.1µF)
   - Battery power is ideal (~zero ripple)
   ```

7. **Verify Cable Connections**
   - Ensure no loose connections on electrode pads
   - Snap connectors should be firm
   - Wires should not bend at connector

**Advanced Troubleshooting:**
   ```cpp
   // Print raw values to see if noise is electrical or mechanical
   Serial.println(sensor.getFIFOCount());  // Should increase steadily
   
   // Check lead-off status over time
   for (int i = 0; i < 100; i++) {
       if (sensor.getLeadOffStatus()) {
           Serial.println("Lead-off detected!");
       }
       delay(100);
   }
   ```

---

### Issue 3: No ECG Data / All Zeros

**Symptoms:**
- `getECGSample()` returns `MAX30001_SUCCESS`
- But `ecg.ecg_sample` is always 0
- No signal visible

**Debugging:**

1. **Verify startECG() was called**
   ```cpp
   max30001_error_t result = sensor.startECG();
   if (result != MAX30001_SUCCESS) {
       Serial.print("Failed to start: ");
       Serial.println(result);
   }
   ```

2. **Check if samples are being generated**
   ```cpp
   uint8_t fifo_count = sensor.getFIFOCount();
   Serial.print("FIFO samples: ");
   Serial.println(fifo_count);
   // Should increase over time
   ```

3. **Verify electrodes are attached**
   ```cpp
   if (sensor.getLeadOffStatus()) {
       Serial.println("No electrodes attached!");
       return;
   }
   ```

4. **Check sampling rate timing**
   ```cpp
   // For 128 SPS, wait at least 8ms between reads
   max30001_ecg_sample_t ecg;
   
   unsigned long last_read = 0;
   if (millis() - last_read >= 8) {
       sensor.getECGSample(&ecg);
       Serial.println(ecg.ecg_sample);
       last_read = millis();
   }
   ```

5. **Clear FIFO and retry**
   ```cpp
   sensor.clearFIFO();
   delay(100);
   
   max30001_ecg_sample_t ecg;
   if (sensor.getECGSample(&ecg) == MAX30001_SUCCESS) {
       Serial.println(ecg.ecg_sample);
   }
   ```

---

### Issue 4: Lead-Off Always Detected

**Symptoms:**
- `sensor.getLeadOffStatus()` always returns `true`
- Even with electrodes firmly attached
- Other measurements seem OK

**Debugging:**

1. **Verify Electrode Connection Path**
   ```
   Troubleshoot:
   1. Electrode pad → wire/snap connector
   2. Connector → breakout board ECGP/ECGN pin
   3. Measure with multimeter (ohms): should show contact resistance
   ```

2. **Test Electrode Quality**
   - Use a fresh electrode pad
   - Ensure conductive paste hasn't dried out
   - Test with different location on body

3. **Check Wire Connections**
   ```cpp
   // Manually measure resistance:
   // Between ECGP/ECGN pins and skin
   // Should be 1-10 kΩ (not open circuit)
   ```

4. **Verify ECGP/ECGN Pin Assignment**
   ```
   Check breakout board labels:
   - ECGP = ECG Positive
   - ECGN = ECG Negative
   - Ensure electrodes connected correctly
   ```

5. **Check for Intermittent Connections**
   ```cpp
   // Monitor over time
   for (int i = 0; i < 60; i++) {
       if (sensor.getLeadOffStatus()) {
           Serial.println("Lead-off at " + String(i) + "s");
       }
       delay(1000);
   }
   ```

---

### Issue 5: BioZ Not Working / Wrong Values

**Symptoms:**
- BioZ samples all zero
- BioZ values constant (not tracking respiration)
- No variation over time

**Debugging:**

1. **Verify BioZ is Enabled**
   ```cpp
   // Use startECGBioZ() or startBioZ()
   sensor.startECGBioZ(MAX30001_RATE_128);
   
   if (!sensor.isBioZEnabled()) {
       Serial.println("BioZ not enabled!");
   }
   ```

2. **Remember BioZ Sampling Rate**
   ```cpp
   // BioZ samples at HALF the ECG rate
   // For 128 SPS ECG → 64 SPS BioZ
   
   // Need skip logic in acquisition loop:
   static bool skip_bioz = false;
   
   if (!skip_bioz) {
       max30001_bioz_sample_t bioz;
       sensor.getBioZSample(&bioz);
       Serial.println(bioz.bioz_sample);
   }
   skip_bioz = !skip_bioz;  // Alternate: read, skip, read, skip
   ```

3. **Verify BioZ Electrodes**
   ```
   Check BioZ electrode connections:
   - BioZ+ electrode → BIP pin
   - BioZ- electrode → BIN pin
   - Both electrodes on skin (not touching ECG electrodes)
   ```

4. **Check BioZ Electrode Placement**
   ```
   Typical placement:
   ┌─────────────────────────┐
   │       Chest (Front)     │
   │   BIP  ┌─────┐  ECGP   │
   │   ●    │     │    ●    │
   │   (L)  │Heart│   (R)   │
   │        │     │         │
   │   ●    │     │    ●    │
   │   BIN  └─────┘  ECGN   │
   └─────────────────────────┘
   
   BIP and BIN should be ~10 inches apart
   ```

---

### Issue 6: Intermittent Data Loss / Dropping Samples

**Symptoms:**
- Some samples return `MAX30001_ERROR_NOT_READY`
- Data stream has gaps
- FIFO overflow errors

**Debugging:**

1. **Check Sampling Loop Timing**
   ```cpp
   // Acquire faster than samples are generated!
   max30001_ecg_sample_t ecg;
   
   unsigned long last_sample = 0;
   
   void loop() {
       // For 128 SPS, must wait 8ms minimum
       if (millis() - last_sample >= 8) {
           if (sensor.getECGSample(&ecg) == MAX30001_SUCCESS) {
               Serial.println(ecg.ecg_sample);
               last_sample = millis();
           }
       }
   }
   ```

2. **Avoid Blocking Operations in Loop**
   ```cpp
   // ✗ BAD: Serial.println() blocks for ~1ms per character
   void loop() {
       max30001_ecg_sample_t ecg;
       sensor.getECGSample(&ecg);
       Serial.println(ecg.ecg_sample);  // Blocks!
       // May miss next sample
   }
   
   // ✓ GOOD: Buffer and print less frequently
   void loop() {
       max30001_ecg_sample_t ecg;
       if (sensor.getECGSample(&ecg) == MAX30001_SUCCESS) {
           if (sample_count++ % 10 == 0) {  // Print every 10th sample
               Serial.println(ecg.ecg_sample);
           }
       }
   }
   ```

3. **Monitor FIFO Overflow**
   ```cpp
   max30001_error_t result = sensor.getECGSample(&ecg);
   if (result == MAX30001_ERROR_FIFO_OVERFLOW) {
       Serial.println("✗ FIFO overflow - acquisition too slow!");
       sensor.clearFIFO();
   }
   ```

4. **Reduce Serial Print Frequency**
   ```cpp
   // Print summary every N samples instead of every sample
   if (sample_count++ % 128 == 0) {  // Every 1 second at 128 SPS
       Serial.print("Heart Rate: ");
       Serial.println(rtor.heart_rate_bpm);
   }
   ```

---

### Issue 7: High Power Consumption

**Symptoms:**
- Battery drains rapidly
- Measured current much higher than 85 µA spec
- Device getting warm

**Debugging:**

1. **Measure Actual Current**
   ```
   Setup:
   USB ─── Multimeter (mA mode) ─── Arduino
   
   Measure with device idle: should be < 100 mA total
   Measure with MAX30001 only: should be < 200 µA
   ```

2. **Check What's Running**
   ```cpp
   // Disable unused features
   sensor.disableBioZ();      // If not using respiration
   sensor.stop();             // When not measuring
   
   // Use lowest sample rate
   sensor.startECG(MAX30001_RATE_128);  // Not 512!
   ```

3. **Reduce Gain**
   ```cpp
   // Lower gain = lower power (marginal difference)
   sensor.setECGGain(MAX30001_ECG_GAIN_80);
   ```

4. **Check for USB Drain**
   ```cpp
   // Run on battery to isolate USB current
   Arduino current on battery: ?
   Arduino + USB current: ? − battery = USB current
   ```

5. **Monitor Other Components**
   ```cpp
   // Typical current breakdown:
   Arduino Uno: ~50-80 mA (entire board)
   MAX30001 alone: ~85 µA
   Serial port: ~5-10 mA
   LED indicators: ~5-20 mA each
   ```

---

### Issue 8: Initialization Fails / Error Codes

**Symptoms:**
- `sensor.begin()` returns error code (not `MAX30001_SUCCESS`)
- Specific error codes: 1-6

**Error Code Guide:**

| Code | Name | Meaning | Action |
|------|------|---------|--------|
| 0 | SUCCESS | No error | ✓ Continue |
| 1 | INVALID_PARAMETER | Bad parameter passed | Check enum values |
| 2 | NOT_INITIALIZED | `begin()` not called yet | Call `begin()` first |
| 3 | NOT_READY | Device not ready for operation | Call `begin()` again |
| 4 | SPI_COMMUNICATION | SPI communication failed | Check wiring |
| 5 | DEVICE_NOT_FOUND | Device not responding | Check VCC/GND |
| 6 | FIFO_OVERFLOW | FIFO buffer overflowed | Acquire faster |

**Debugging Error 1 (INVALID_PARAMETER):**
```cpp
// Check for invalid enum values
max30001_sample_rate_t rate = 255;  // ✗ Invalid
if (sensor.startECG(rate) != MAX30001_SUCCESS) {
    Serial.println("Invalid sample rate!");
}

// Use valid constants only:
sensor.startECG(MAX30001_RATE_128);  // ✓ Valid
sensor.startECG(MAX30001_RATE_256);  // ✓ Valid
sensor.startECG(MAX30001_RATE_512);  // ✓ Valid
```

**Debugging Error 4 (SPI_COMMUNICATION):**
```cpp
// Check physical wiring with LED test
void spi_test() {
    digitalWrite(7, HIGH);    // CS high
    delay(100);
    digitalWrite(7, LOW);     // CS low
    delay(100);
    digitalWrite(7, HIGH);    // CS high again
    // Should see 1 kHz square wave on oscilloscope
}
```

---

## Diagnostic Checklist

Print this and verify each item:

```cpp
void full_diagnostic() {
    Serial.println("\n=== MAX30001 Diagnostic ===\n");
    
    // Power supply
    Serial.print("1. Is MAX30001 powered? ");
    Serial.println(sensor.isConnected() ? "✓ YES" : "✗ NO");
    
    // Device detection
    if (sensor.isConnected()) {
        Serial.print("2. Device detected: ✓ YES\n");
        
        // Device info
        max30001_device_info_t info;
        sensor.getDeviceInfo(&info);
        Serial.print("   Part ID: 0x");
        Serial.println(info.part_id, HEX);
        
        // Initialization
        Serial.print("3. Initialization: ");
        Serial.println(sensor.begin() == MAX30001_SUCCESS ? "✓ YES" : "✗ NO");
        
        // ECG
        sensor.startECG();
        Serial.print("4. ECG enabled: ");
        Serial.println(sensor.isECGEnabled() ? "✓ YES" : "✗ NO");
        
        Serial.print("5. Lead-off: ");
        Serial.println(sensor.getLeadOffStatus() ? "✗ DETECTED" : "✓ OK");
        
        // Sample acquisition
        max30001_ecg_sample_t ecg;
        Serial.print("6. Sample read: ");
        Serial.println(sensor.getECGSample(&ecg) == MAX30001_SUCCESS ? "✓ YES" : "✗ NO");
        
        Serial.print("7. Sample valid: ");
        Serial.println(ecg.sample_valid ? "✓ YES" : "✗ NO");
        
        Serial.print("8. Sample value: ");
        Serial.println(ecg.ecg_sample);
        
    } else {
        Serial.println("2. Device NOT detected ✗");
        Serial.println("   Check: VCC, GND, SPI wiring");
    }
}
```

Run this in `setup()`:
```cpp
void setup() {
    Serial.begin(115200);
    SPI.begin();
    full_diagnostic();
}
```

---

## When to Contact Support

If you've verified all above and still have issues:

1. **Gather information:**
   - Arduino board model and version
   - MAX30001 breakout board version
   - Exact error messages (screenshots helpful)
   - Code snippet reproducing issue
   - Multimeter readings (VCC, GND)

2. **Post on:**
   - GitHub Issues: [Link](https://github.com/Protocentral/protocentral_max30001_arduino_library/issues)
   - Arduino Forum: [Link](https://forum.arduino.cc/)
   - ProtoCentral Support: support@protocentral.com

3. **Include:**
   - Full sketch code
   - `full_diagnostic()` output
   - Photos of breadboard/wiring
   - Expected vs. actual behavior

---

## Reference Resources

- [MAX30001 Datasheet](https://datasheets.maximintegrated.com/en/ds/MAX30001.pdf)
- [Arduino SPI Reference](https://www.arduino.cc/en/reference/SPI)
- [ProtoCentral Documentation](https://docs.protocentral.com/)
- [GitHub Repository](https://github.com/Protocentral/protocentral_max30001_arduino_library)
