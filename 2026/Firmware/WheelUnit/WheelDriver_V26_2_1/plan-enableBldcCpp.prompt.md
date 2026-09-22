## Plan: Enable BLDC.cpp

TL;DR: Fix `lib/BLDC/BLDC.cpp` by removing the accidental duplicated implementation block and fix `lib/BLDC/BLDC.hpp` by removing the invalid `extern "C"` wrapper around the C++ class definition.

**Steps**
1. Edit `lib/BLDC/BLDC.hpp`:
   - Remove the `extern "C" {` and matching `}` around `class BLDCMotor`.
   - Keep the `#ifndef BLDC_H` include guard and existing includes.
2. Edit `lib/BLDC/BLDC.cpp`:
   - Keep the first implementation block for `BLDCMotor` with `AS5600`.
   - Delete the duplicate second block that begins at line 306 with `}#include "BLDC.hpp"` and continues through the second `drive()` definition at line 603.
   - Ensure only one `BLDCMotor` implementation remains and it matches the header signature.
3. Rebuild with `make` and confirm errors are gone.

**Relevant files**
- `/Users/tsukigake/Documents/GitHub/ssl-Circuit/2026/Firmware/WheelUnit/WheelDriver_V26_2_1/lib/BLDC/BLDC.hpp`
- `/Users/tsukigake/Documents/GitHub/ssl-Circuit/2026/Firmware/WheelUnit/WheelDriver_V26_2_1/lib/BLDC/BLDC.cpp`

**Verification**
1. Run `make` in the project root.
2. Confirm `build/BLDC.o` is produced without compile errors.
3. Confirm `src/app.cpp` links successfully with `BLDCMotor`.

**Decision**
- Preserve the existing `AS5600` encoder path and remove the erroneous duplicated `AS5048A` implementation.

**Further Considerations**
1. If you want multi-encoder support later, the correct design is to add a shared encoder interface or template, not duplicate the class implementation.
