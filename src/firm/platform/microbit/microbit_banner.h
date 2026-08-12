#pragma once

namespace Platform {

// formatBanner -- render "DEVICE:NEZHA2:robot:<name>:<serial>" into `buf`
// (`size` includes the NUL; 64 bytes is enough). Byte-frozen wire format --
// docs/protocol-v4.md §2.4. ARM-only: reads the CODAL identity functions,
// which is why it lives here and not in main.cpp (DESIGN.md §1).
void formatBanner(char* buf, int size);

// formatIdLine -- render "ID:<drivetrainType>:<profileName>:<version>" into
// `buf` (`size` includes the NUL) -- the configured-robot identity
// (drivetrain type + calibration-profile name/version), distinct from
// formatBanner()'s hardware identity. Byte-frozen wire format, same as
// formatBanner() above; relocated verbatim out of main.cpp's own inline
// snprintf() (136-005, "de-junk main.cpp") -- pure string formatting, no
// CODAL dependency, kept alongside its sibling for symmetry.
void formatIdLine(char* buf, int size, const char* drivetrainType, const char* profileName,
                   const char* version);

}  // namespace Platform
