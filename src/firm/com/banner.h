#pragma once

// formatBanner -- render "DEVICE:NEZHA2:robot:<name>:<serial>" into `buf`
// (`size` includes the NUL; 64 bytes is enough). Byte-frozen wire format --
// docs/protocol-v4.md §2.4. ARM-only: reads the CODAL identity functions,
// which is why it lives here and not in main.cpp (DESIGN.md §1).
void formatBanner(char* buf, int size);
