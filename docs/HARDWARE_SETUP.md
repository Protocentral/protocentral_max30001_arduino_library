# MAX30001 Hardware Setup Guide

Complete wiring instructions and electrode placement guide for MAX30001 breakout board integration with Arduino-compatible microcontrollers.

## Table of Contents

1. [Board Compatibility](#board-compatibility)
2. [SPI Wiring](#spi-wiring)
3. [Power Supply](#power-supply)
4. [Electrode Connections](#electrode-connections)
5. [Interrupt Setup (Optional)](#interrupt-setup-optional)
6. [Troubleshooting](#troubleshooting)

---

## Board Compatibility

### Officially Tested
- ✅ Arduino Uno/Uno R3
- ✅ Arduino Mega 2560
- ✅ Arduino Uno R4 Minima (Renesas)
- ✅ Arduino Due (3.3V variant)
- ✅ ESP32 DevKit

### Expected to Work
- Arduino Leonardo
- Arduino Nano (with limitations - verify SPI pins)
- Arduino MKR series (with 3.3V operation)
- Other Arduino-compatible boards with SPI

**Important:** Check your board's SPI pin assignments - they vary between architectures.

---

## SPI Wiring

### Standard Arduino (Uno, Mega, Nano)

| MAX30001 Pin | Arduino Pin | Function | Notes |
|--------------|-------------|----------|-------|
| MISO | D12 | SPI Slave Out | Required |
| MOSI | D11 | SPI Slave In | Required |
| SCK | D13 | SPI Clock | Required |
| CS | D7 | Chip Select | Configurable (default D7) |
| GND | GND | Ground | **Must connect** |
| VCC | +5V | Power Supply | See Power Supply section |

### Arduino Mega 2560

| MAX30001 Pin | Mega Pin | Function |
|--------------|----------|----------|
| MISO | D50 | SPI Slave Out |
| MOSI | D51 | SPI Slave In |
| SCK | D52 | SPI Clock |
| CS | D53 (or other) | Chip Select |

### ESP32

| MAX30001 Pin | ESP32 Pin | Function | Notes |
|--------------|-----------|----------|-------|
| MISO | GPIO 19 | SPI Slave Out | Default VSPI MISO |
| MOSI | GPIO 23 | SPI Slave In | Default VSPI MOSI |
| SCK | GPIO 18 | SPI Clock | Default VSPI CLK |
| CS | GPIO 5 | Chip Select | Configurable |
| GND | GND | Ground | Must connect |
| VCC | +3.3V | Power Supply | ⚠️ Note ESP32 is 3.3V only |

**ESP32 Note:** When using SPI with ESP32, you may need to specify the pins in your code:

```cpp
SPI.begin(18, 19, 23, 5);  // CLK, MISO, MOSI, CS
MAX30001 sensor(5);
```

### Arduino Due (3.3V)

| MAX30001 Pin | Due Pin | Function |
|--------------|---------|----------|
| MISO | D74 (SPI MISO) | SPI Slave Out |
| MOSI | D75 (SPI MOSI) | SPI Slave In |
| SCK | D76 (SPI SCK) | SPI Clock |
| CS | D77 (or other) | Chip Select |
| GND | GND | Ground |
| VCC | +3.3V | Power Supply |

---

## Power Supply

### 5V Systems (Arduino Uno, Mega, Nano)

MAX30001 can operate at 5V directly. For optimal performance:

**Option 1: Direct Connection (Simplest)**
```
Arduino +5V → MAX30001 VCC
Arduino GND → MAX30001 GND
```

**Option 2: With Decoupling Capacitor (Recommended)**
```
Arduino +5V → [10µF Capacitor] → MAX30001 VCC
Arduino GND → MAX30001 GND (and capacitor other terminal)
```

The capacitor stabilizes power supply noise and improves signal quality.

### 3.3V Systems (ESP32, Arduino Due, MKR)

MAX30001 operates natively at 3.3V and uses less power.

```
ESP32 +3.3V → MAX30001 VCC
ESP32 GND → MAX30001 GND
```

### Power Consumption

| Operating Mode | Typical Current |
|-----------------|-----------------|
| Idle (all off) | < 1 µA |
| ECG only | 50 µA |
| BioZ only | 40 µA |
| ECG + BioZ | 85 µA |
| All features active | 100 µA |

---

## Electrode Connections

### ECG Electrode Placement

The MAX30001 requires **two ECG electrodes** (no third "right leg drive" electrode needed).

**Typical Placements:**

#### Configuration 1: Wrist to Wrist (Most Common)
- Left wrist: ECGP (positive)
- Right wrist: ECGN (negative)
- Best for wearable applications

#### Configuration 2: Chest Lead-I (Clinical)
- Left shoulder/axilla: ECGP (positive)
- Right shoulder/axilla: ECGN (negative)
- Mimics standard clinical Lead-I

#### Configuration 3: Ankle to Ankle
- Left ankle: ECGP (positive)
- Right ankle: ECGN (negative)
- Less common, higher noise

**Connection to Breakout:**
```
Electrode 1 (ECG+) ── [Wet electrode pad or snap connector] ──→ ECGP pin
Electrode 2 (ECG-) ── [Wet electrode pad or snap connector] ──→ ECGN pin
```

### BioZ Electrode Placement

BioZ electrodes are used for impedance measurement (respiration monitoring).

**Configuration: Chest Band (Recommended)**
- Right side of chest below armpit: BIP (positive)
- Left side of chest below armpit: BIN (negative)

**Configuration: Wrist Band (Alternative)**
- Right wrist: BIP (positive)
- Left wrist: BIN (negative)

**Connection to Breakout:**
```
BioZ Electrode 1 (BioZ+) ──→ BIP pin
BioZ Electrode 2 (BioZ-) ──→ BIN pin
```

### Electrode Types

**Wet Electrodes (Best Signal Quality)**
- Standard disposable ECG pads (Ag/AgCl)
- Pre-gelled (conductive paste on backing)
- Available: 50/pack, cost ~$0.10 each
- Signal quality: Excellent
- Life: Single use

**Dry Electrodes (Reusable)**
- Stainless steel snaps on fabric armbands
- Requires skin contact (no gel)
- Signal quality: Good
- Advantages: Reusable, wearable comfort

**DIY Options** (for testing only)
- Copper tape + saline-soaked cloth: Poor signal, high noise
- Aluminum foil: Unreliable

### Proper Electrode Placement Tips

1. **Clean skin**: Wipe with dry cloth to remove oils/lotions
2. **Dry placement**: Apply electrodes to dry skin
3. **Firm contact**: Press electrode firmly for 5+ seconds
4. **Allow settling**: Wait 30 seconds before starting measurement (impedance settles)
5. **Recheck**: If signal is noisy, check electrode adhesion

### Common Placement Mistakes

❌ **Avoid:**
- Wet/damp skin (increases noise)
- Hairy skin (air gaps reduce contact)
- Freshly shaved skin (irritation, poor contact)
- Moving electrodes during measurement (motion artifacts)
- Placing both electrodes on same side of body

---

## Interrupt Setup (Optional)

The INT1 pin can trigger an interrupt when new samples are available. This is optional; polling works fine for most applications.

### INT1 Wiring

| MAX30001 PIN | Arduino Pin | Function |
|--------------|-------------|----------|
| INT1 | D2 (or other) | Interrupt signal, active low |
| GND | GND | Ground |

### Interrupt Configuration

```cpp
#define INT1_PIN D2

void setup() {
    pinMode(INT1_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(INT1_PIN), dataReady, FALLING);
    sensor.enableInterrupt();
}

void dataReady() {
    // Called when new sample is ready
    // Keep ISR short!
}
```

**Note:** INT1 is optional. Most applications work well with polled data acquisition (see examples).

---

## Connection Verification Checklist

Before starting acquisition:

- [ ] **SPI Lines Connected**
  - MISO, MOSI, SCK wired to correct Arduino pins
  - Test with `Serial.println(sensor.isConnected())`

- [ ] **Power Supply**
  - VCC connected to 5V (or 3.3V)
  - GND connected to Arduino GND
  - Check voltage at VCC pin (should be stable)

- [ ] **CS Pin Wired**
  - CS connected to specified pin (D7 default)
  - Check with Serial output from `sensor.begin()`

- [ ] **Electrodes Attached**
  - ECG electrodes on skin, firm contact
  - BioZ electrodes placed (if using BioZ mode)
  - Check with `sensor.getLeadOffStatus()`

- [ ] **Decoupling Capacitor (Optional but Recommended)**
  - 10µF capacitor across VCC-GND
  - Improves signal stability

---

## Troubleshooting

### Device Not Responding

**Symptom:** `sensor.isConnected()` returns false

**Solutions:**
1. ✓ Verify SPI pin connections (MISO, MOSI, SCK)
2. ✓ Check CS pin is connected and configured correctly
3. ✓ Verify power supply: measure VCC pin voltage
4. ✓ Check GND connection (must be solid)
5. ✓ Try adding 10µF decoupling capacitor
6. ✓ Verify board SPI pin assignments (different for Mega vs Uno)

### Noisy ECG Signal

**Symptom:** Waveform very noisy, hard to see heart rate

**Solutions:**
1. ✓ Check electrode skin contact (clean skin, firm pressure)
2. ✓ Ensure electrodes are not moving during measurement
3. ✓ Verify both ECG electrodes are on skin
4. ✓ Add decoupling capacitor on power supply
5. ✓ Reduce gain temporarily for testing: `sensor.setECGGain(MAX30001_ECG_GAIN_80)`
6. ✓ Increase low-pass filter: `sensor.setECGLowPassFilter(40)`
7. ✓ Keep USB cable separate from electrode cables

### Lead-Off Detection Always Triggered

**Symptom:** `sensor.getLeadOffStatus()` always returns true

**Solutions:**
1. ✓ Check electrode contact - press firmly for 10 seconds
2. ✓ Clean skin and reapply electrodes
3. ✓ Verify electrode conductivity (test with multimeter)
4. ✓ Check wire connections to ECGP/ECGN pins
5. ✓ Try different electrode location

### No BioZ Signal

**Symptom:** BioZ values remain constant at zero

**Solutions:**
1. ✓ Verify BioZ electrodes are attached to skin
2. ✓ Check BIP/BIN wiring
3. ✓ Ensure `startECGBioZ()` or `startBioZ()` called
4. ✓ Remember BioZ samples at half ECG rate
5. ✓ Check example code samples BioZ correctly (skip logic)

### High Power Consumption

**Symptom:** Battery drains quickly

**Solutions:**
1. ✓ Disable unused channels: `sensor.disableBioZ()` if only ECG needed
2. ✓ Use lower sample rate: `MAX30001_RATE_128` instead of `MAX30001_RATE_512`
3. ✓ Reduce gain if possible: `MAX30001_ECG_GAIN_80`
4. ✓ Call `sensor.stop()` when not measuring
5. ✓ Check current draw with multimeter (~100µA typical)

### Serial Communication Errors

**Symptom:** SPI errors, unreliable data

**Solutions:**
1. ✓ Add decoupling capacitor (0.1µF + 10µF)
2. ✓ Keep SPI wires short and away from power leads
3. ✓ Reduce SPI clock speed (default 1 MHz is safe)
4. ✓ Check for loose connections
5. ✓ Try different USB power supply (avoid hub)

---

## Performance Optimization Tips

### For Best Signal Quality
1. Use fresh Ag/AgCl electrodes
2. Place on dry, clean skin
3. Add decoupling capacitor
4. Use 160 V/V gain if signal is small
5. Set appropriate filters for your use case

### For Lowest Power
1. Use 128 SPS sampling rate
2. Disable BioZ if respiration not needed
3. Use 80 V/V gain if signal is large enough
4. Set higher high-pass filter cutoff (less baseline correction needed)
5. Consider duty-cycling entire system

### For Real-Time Responsiveness
1. Use highest available sample rate
2. Use ISR-based acquisition (Example04)
3. Minimize delays in loop
4. Avoid Serial.print() in fast loops
5. Use interrupt-driven data availability

---

## Board-Specific Notes

### Arduino Uno R4 Minima
- 5V digital I/O
- SPI pins: D11 (MOSI), D12 (MISO), D13 (SCK)
- Fully compatible, well-tested
- No level shifting needed
- Recommended for general use

### ESP32
- 3.3V digital I/O (not 5V tolerant!)
- SPI1 available on multiple pin configurations
- WiFi can interfere: keep SPI cables short
- Brownout triggered if power supply unstable
- Requires 3.3V power supply

### Arduino Due
- 3.3V digital I/O
- More powerful, sufficient for advanced applications
- Check SPI pin assignments (SAM3X variant)
- Good for real-time signal processing

### Arduino Mega
- 5V digital I/O
- SPI on different pins (D50-D53)
- Extra I/O for additional sensors
- Can be overkill for simple applications

---

## Additional Resources

- [MAX30001 Datasheet](https://datasheets.maximintegrated.com/en/ds/MAX30001.pdf)
- [ProtoCentral Hookup Guide](https://docs.protocentral.com/getting-started-with-max30001/)
- [Arduino SPI Reference](https://www.arduino.cc/en/reference/SPI)
- [Electrode Placement Diagrams](https://www.lifepak.com/wps/wcm/connect/2b6cfb98-e9e2-4d2c-b5ce-3e7c77ee19e7/LifePak-15-Quick-Reference-Guide.pdf)
