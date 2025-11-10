# MAX30001 Arduino Library - Copilot Instructions

## Project Overview

This is an Arduino library for the Protocentral MAX30001 ECG and Bio-Impedance (BioZ) breakout board. The MAX30001 is a single-lead ECG monitoring IC with built-in R-R detection, consuming just 85 µW and supporting two-electrode operation (no third DRL electrode required).

**Key Features:**
- Single-lead ECG acquisition at 128/256/512 SPS
- Bio-impedance (BioZ) measurement for respiration monitoring
- Hardware R-R interval detection (heartbeat timing)
- SPI communication interface
- Compatible with Arduino and ESP32

## Architecture

### Core Components

1. **MAX30001 Class** (`src/protocentral_max30001.h/cpp`): Main driver implementing SPI communication and chip configuration
2. **Register Definitions**: Extensive register unions in header for type-safe chip configuration (see `max30001_cnfg_gen_t`, `max30001_cnfg_ecg_t`, etc.)
3. **Examples**: Sample sketches demonstrating integration patterns

### Data Flow

```
MAX30001 Chip (SPI) → Arduino → Serial (ProtoCentral OpenView format) → PC Visualization
```

## Hardware Configuration

### Standard Arduino Wiring
- **MISO**: D12, **MOSI**: D11, **SCLK**: D13
- **CS**: D7 (configurable via constructor)
- **INT1**: D2 (for interrupt-driven operation)
- **Power**: +5V and GND

### ESP32 Alternative
- **MISO**: GPIO 19, **MOSI**: GPIO 23, **SCLK**: GPIO 18
- **CS**: GPIO 5 (change `MAX30001_CS_PIN` define)

## Critical Development Patterns

### Initialization Sequence

Always follow this exact order (see `BeginECGBioZ()` at line ~185 in `protocentral_max30001.cpp`):

1. Software reset via `_max30001SwReset()`
2. Configure `CNFG_GEN` (enable ECG/BioZ channels)
3. Configure `CNFG_CAL` (calibration settings)
4. Configure `CNFG_ECG` (sampling rate, gain, filters)
5. Configure `CNFG_EMUX` (ECG mux connections)
6. Configure `CNFG_BIOZ` and `CNFG_BMUX` (if using BioZ)
7. Call `_max30001Synch()` to apply settings
8. Add 100ms delays between register writes

**Wrong order will cause initialization failure or incorrect readings.**

### Configuration via Typed Unions

Use register unions for type-safe configuration instead of magic hex values:

```cpp
max30001_cnfg_ecg_t cnfg_ecg;
cnfg_ecg.bit.rate = 0b10;    // 128 SPS
cnfg_ecg.bit.gain = 0b10;    // 160 V/V
cnfg_ecg.bit.dhpf = 0b1;     // 0.5Hz high-pass
cnfg_ecg.bit.dlpf = 0b01;    // 40Hz low-pass
_max30001RegWrite(CNFG_ECG, cnfg_ecg.all);
```

### Data Acquisition Pattern

**BioZ samples at half the ECG rate** - implement skip logic:

```cpp
bool BioZSkipSample = false;
// In loop:
ecg_data = max30001.getECGSamples();
if (!BioZSkipSample) {
    bioz_data = max30001.getBioZSamples();
} else {
    bioz_data = 0x00;
}
BioZSkipSample = !BioZSkipSample;
```

### Serial Protocol (ProtoCentral OpenView)

Packet structure for visualization (see `sendData()` in Example1):
- Header: `0x0A 0xFA <length> 0x00 0x02`
- Payload: 4 bytes ECG (LSB first) + 4 bytes BioZ + 4 bytes flags/reserved
- Footer: `0x00 0x0B`

## Operating Modes

1. **ECG Only**: `BeginECGOnly()` - Single-channel ECG
2. **ECG + BioZ**: `BeginECGBioZ()` - Dual acquisition for respiration
3. **R-R Mode**: `BeginRtoRMode()` - Hardware heartbeat detection

## Common Pitfalls

- **Missing delays**: Register writes need 100ms settling time
- **Wrong CS pin**: Default is D7 for Arduino, change to 5 for ESP32
- **Incorrect SPI mode**: Must use `SPI_MODE0` at 1 MHz (`MAX30001_SPI_SPEED`)
- **BioZ oversampling**: Reading BioZ every loop iteration causes timing issues
- **Initialization check**: Always verify `max30001ReadInfo()` returns true before configuration

## Testing & Debugging

- **Device Detection**: Check for "MAX 30001 read ID Success" serial message
- **Visualization**: Use ProtoCentral OpenView GUI at 57600 baud to verify waveforms
- **Hardware Test**: Verify SPI connections if `max30001ReadInfo()` fails repeatedly

## File Structure

```
library.properties          # Arduino library metadata
src/
  protocentral_max30001.h   # Class definition + register unions
  protocentral_max30001.cpp # Implementation (531 lines)
examples/
  Example1-ECG-BioZ-stream-Openview/  # Complete working example
```

## License

Code under MIT License. Parts copyright Maxim Integrated Products (used with permission). See dual license headers in source files.
