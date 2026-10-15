# Changelog

All notable changes to this library are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and the project uses
[Semantic Versioning](https://semver.org/).

## [2.1.0] - 2026-10-15

### Fixed
- **BioZ respiration:** the digital high-pass was 0.5 Hz (a 4th-order filter that removes breathing); it is now 0.05 Hz. Gain is 10 V/V, so the input no longer saturates above ~3.1 kOhm, and the demodulation phase is 33.75°.
- **R-to-R:** the detector was never enabled, so no beats were reported. `getRtoRData()` now sets `rr_detected` only for a new beat.
- **Lead-off:** DC lead-off is enabled with every ECG start, and `getLeadOffStatus()` reads the lead-off flag (it was reading BioZ over-range).
- **Interrupts:** `enableInterrupt()` enables the FIFO interrupts on INTB (it previously only set the ECG enable bit).
- **FIFO reads:** `getECGSample()` / `getBioZSample()` check the FIFO tags and return `MAX30001_ERROR_NOT_READY` when the FIFO is empty, instead of repeating or inventing samples. The burst reads use the right BioZ tag bits and keep the last sample of each burst.
- `convertECGToMicrovolts()` was 1.64x too high.
- `setECGHighPassFilter()`, `setECGLowPassFilter()` and `setECGGain()` now map to the real register options.
- `startECGBioZ()` applies its sample rate.
- 24-bit register reads lost the top byte on 8-bit AVR boards.
- `readStatus()` was declared but not defined.
- `library.properties` named a header that does not exist.
- **01.OpenView** now matches the OpenView MAX30001 descriptor: 115200 baud, ECG/BioZ/HR/RR payload, packets only.

### Added
- **BioZ configuration:** `setBioZGain()`, `setBioZCurrent()` (checked against the drive frequency), `setBioZFrequency()`, `setBioZPhaseOffset()`, `setBioZHighPassFilter()`, `setBioZLowPassFilter()`, `setBioZAnalogHighPass()`, `setBioZExternalBias()`, `setBioZLeadOff()`, `setBioZCurrentMonitor()`, and `convertBioZToOhms()`.
- **ECG configuration:** `setECGPolarity()`, `setLeadOffDetection()` (current and threshold), `setRtoRConfig()`, `setFastRecovery()`, `setLeadBias()`. Settings are kept and applied on every start.
- **Self-test:** `enableBioZTestLoad()` (built-in 625-5000 Ohm test resistor with a modulated step) with `bioZTestLoadOhms()` / `bioZTestLoadModulationOhms()`, and `enableECGTestSignal()` (internal calibration square wave).
- **Lead-on detection:** `startLeadOnDetection()` / `isLeadOn()`, with an optional INTB interrupt.
- **Diagnostics:** `getStatus()` returns status flags latched since the last call, including those seen by other library calls. `printRegisters()` dumps the registers. `synchronize()` issues SYNCH for multi-board setups.
- `getLeadOffDetail()` for per-electrode lead-off, and `out_of_range` on BioZ samples.
- An optional ECG gain argument on `startECGBioZ()`.
- `MAX30001_LIBRARY_VERSION` and `getLibraryVersion()`.
- `MAX30001_REG_*` register names. Define `MAX30001_NO_LEGACY_NAMES` to drop the unprefixed ones (`STATUS`, `INFO`, ...).
- Examples `06.RespirationBioZ`, `07.BioZSelfTest`, `08.Diagnostics` and `09.ECGSerialPlotter`.
- `keywords.txt`, `scripts/build.sh`, `scripts/upload_uno_r4.sh`.

### Changed
- Examples renamed to `NN.Name` (`01.OpenView`, `02.HeartRateRtoR`, `03.LeadOffDetection`, `04.InterruptDriven`, `05.AdvancedConfig`).
- FIFO buffers are sized to the chip FIFOs, saving about 1 kB of RAM. Every example now fits on an Arduino Uno.
- Register writes no longer hold CS low for 4 ms, and the start methods no longer wait 100 ms after each register.
- The library no longer prints to `Serial`.
- The calibration sources are now off unless a test signal is enabled (CNFG_CAL was written with EN_VCAL set but not routed).
- CI compiles on the same board list as the other ProtoCentral libraries, including the Uno and Uno R4.

### Deprecated
- The 1.x-style API: `BeginECGOnly()`, `BeginECGBioZ()`, `BeginRtoRMode()`, `getECGSamples()`, `getBioZSamples()`, `getHRandRR()`, `max30001ReadInfo()`, `max30001SetsamplingRate()`, `max30001SetInterrupts()`, `readStatus()`, and the unprefixed register names. They will be removed in 3.0.

## [2.0.0] - 2025-11-12

New high-level API (`begin()`, `startECG()`, `getECGSample()`, ...) alongside the 1.x functions.
