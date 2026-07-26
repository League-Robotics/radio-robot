// wire_golden_vector_harness.cpp -- sprint 124 ticket 004 (SUC-002,
// REQUIRED, non-negotiable): the C++ reader of THE shared cross-language
// golden-vector fixture, src/tests/fixtures/wire_golden_vectors.txt.
//
// This is one of the stakeholder's two structural fixes for the defect
// class that shipped in 123: a move_wheels envelope containing a literal
// 0x0A byte was split and corrupted at the terminator because firmware's
// and host's demuxers were two independent guesses with no shared vector
// forcing them to agree. Sprint 124's own wire_runtime_harness.cpp
// (scenarioCobsKeyedOn0x0AAdversarialVectors) and the host's
// test_host_wire_codec.py (test_cobs_delimiter_0x0a_adversarial_vectors_
// from_issue) each HARDCODED the issue's own worked vectors independently
// -- exactly the "two hand-maintained copies" duplication this ticket
// exists to close. This harness reads the SAME fixture file
// src/tests/unit/test_wire_golden_vectors.py reads, computes the SAME
// composition using WireRuntime's own primitives, and compares against
// the fixture's OWN expected_wire_hex column -- not a value hardcoded in
// this file. If a future edit changes the fixture's expected bytes for a
// row without both codecs actually agreeing on the new value, ONE of the
// two suites fails.
//
// Composition each row's expected_wire_hex encodes (see the fixture
// file's own header comment for the full rationale and provenance of each
// row):
//
//   crc      = crcUpdate(crcUpdate(crcInit(), command), ":") folded with
//              payload, when command is non-empty; crcCompute(payload)
//              alone otherwise
//   combined = payload || crc16_le(crc)
//   expected_wire = cobsEncode(combined, delimiter)
//
// This mirrors comms.cpp's own crcOverScope() (an anonymous-namespace,
// not-externally-linkable function) at the exact primitive composition
// ticket 124-005's framing cutover will wire into Comms::sendReply()/
// decodeBinaryFrame() itself -- see the fixture header for why this
// ticket exercises WireRuntime's primitives directly rather than
// Comms::sendReply(), which still defaults to delimiter=0x00/no CRC scope
// as of ticket 003.
//
// Usage: wire_golden_vector_harness <path-to-wire_golden_vectors.txt>
// Prints PASS/FAIL per vector (mirrors wire_runtime_harness.cpp's own
// hand-rolled assertion shape), exits 0 iff every vector passes both the
// encode-matches-fixture and decode-round-trips checks.
#include <cctype>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>

#include "messages/wire_runtime.h"

namespace {

int g_failureCount = 0;

void fail(const std::string& vectorName, const std::string& what) {
  ++g_failureCount;
  std::printf("  FAIL [%s]: %s\n", vectorName.c_str(), what.c_str());
}

// --- Tiny hex <-> bytes helpers (test-only code, not the no-heap firmware
// primitives WireRuntime itself is held to). ---

std::vector<uint8_t> hexToBytes(const std::string& hex) {
  std::vector<uint8_t> out;
  out.reserve(hex.size() / 2);
  for (size_t i = 0; i + 1 < hex.size(); i += 2) {
    out.push_back(static_cast<uint8_t>(std::stoul(hex.substr(i, 2), nullptr, 16)));
  }
  return out;
}

std::string bytesToHex(const uint8_t* data, size_t len) {
  static const char* kDigits = "0123456789abcdef";
  std::string out;
  out.reserve(len * 2);
  for (size_t i = 0; i < len; ++i) {
    out.push_back(kDigits[(data[i] >> 4) & 0xF]);
    out.push_back(kDigits[data[i] & 0xF]);
  }
  return out;
}

// --- Fixture row + parser -------------------------------------------------
// Mirrors test_wire_golden_vectors.py's own load_golden_vectors() column
// layout exactly (see wire_golden_vectors.txt's own header comment):
//   name | delimiter_hex | command | payload_hex | expected_wire_hex | source

struct GoldenVector {
  std::string name;
  uint8_t delimiter = 0;
  std::string command;
  std::vector<uint8_t> payload;
  std::vector<uint8_t> expectedWire;
  std::string source;
};

std::vector<std::string> splitPipe(const std::string& line) {
  std::vector<std::string> fields;
  size_t start = 0;
  while (true) {
    const size_t pos = line.find('|', start);
    if (pos == std::string::npos) {
      fields.push_back(line.substr(start));
      break;
    }
    fields.push_back(line.substr(start, pos - start));
    start = pos + 1;
  }
  return fields;
}

std::string trim(const std::string& s) {
  size_t begin = 0;
  while (begin < s.size() && std::isspace(static_cast<unsigned char>(s[begin]))) ++begin;
  size_t end = s.size();
  while (end > begin && std::isspace(static_cast<unsigned char>(s[end - 1]))) --end;
  return s.substr(begin, end - begin);
}

std::vector<GoldenVector> loadGoldenVectors(const std::string& path) {
  std::ifstream in(path);
  if (!in) {
    std::fprintf(stderr, "FATAL: could not open fixture file: %s\n", path.c_str());
    std::exit(2);
  }

  std::vector<GoldenVector> vectors;
  std::string rawLine;
  int lineNo = 0;
  while (std::getline(in, rawLine)) {
    ++lineNo;
    const std::string line = trim(rawLine);
    if (line.empty() || line[0] == '#') continue;

    const std::vector<std::string> fields = splitPipe(line);
    if (fields.size() != 6) {
      std::fprintf(stderr, "FATAL: %s:%d: expected 6 pipe-delimited fields, got %zu: %s\n", path.c_str(), lineNo,
                    fields.size(), line.c_str());
      std::exit(2);
    }
    if (fields[0] == "name" && fields[1] == "delimiter_hex") continue;  // header row

    GoldenVector v;
    v.name = fields[0];
    v.delimiter = static_cast<uint8_t>(std::stoul(fields[1], nullptr, 16));
    v.command = fields[2];
    v.payload = hexToBytes(fields[3]);
    v.expectedWire = hexToBytes(fields[4]);
    v.source = fields[5];
    vectors.push_back(std::move(v));
  }

  if (vectors.empty()) {
    std::fprintf(stderr, "FATAL: %s: no vectors parsed -- fixture empty or malformed\n", path.c_str());
    std::exit(2);
  }
  return vectors;
}

// --- The composition under test: crcOverScope() (mirrors comms.cpp's own,
// not-externally-linkable anonymous-namespace function of the same name --
// see this file's header comment) + cobsEncode(delimiter). ---

uint16_t crcOverScope(const std::string& command, const std::vector<uint8_t>& payload) {
  uint16_t crc = WireRuntime::crcInit();
  if (!command.empty()) {
    crc = WireRuntime::crcUpdate(crc, reinterpret_cast<const uint8_t*>(command.data()), command.size());
    static constexpr uint8_t kSeparator = ':';
    crc = WireRuntime::crcUpdate(crc, &kSeparator, 1);
  }
  return WireRuntime::crcUpdate(crc, payload.data(), payload.size());
}

void checkVector(const GoldenVector& v) {
  const uint16_t crc = crcOverScope(v.command, v.payload);

  std::vector<uint8_t> combined = v.payload;
  combined.push_back(static_cast<uint8_t>(crc & 0xFF));
  combined.push_back(static_cast<uint8_t>((crc >> 8) & 0xFF));

  std::vector<uint8_t> wire(WireRuntime::cobsEncodedMaxLength(combined.size()));
  size_t wireLen = 0;
  const bool encodeOk = WireRuntime::cobsEncode(combined.empty() ? nullptr : combined.data(), combined.size(),
                                                 wire.data(), wire.size(), &wireLen, v.delimiter);
  if (!encodeOk) {
    fail(v.name, "cobsEncode() failed");
    return;
  }
  wire.resize(wireLen);

  if (wire != v.expectedWire) {
    fail(v.name, "encode: computed wire bytes disagree with the shared fixture -- got " + bytesToHex(wire.data(), wire.size()) +
                     ", fixture (" + v.source + ") says " + bytesToHex(v.expectedWire.data(), v.expectedWire.size()));
  }
  for (uint8_t b : wire) {
    if (b == v.delimiter) {
      fail(v.name, "encode: wire bytes contain a literal delimiter byte");
      break;
    }
  }

  // Decode round-trip against the FIXTURE's own expected_wire_hex (ground
  // truth), not against our own just-computed `wire` -- proves the fixture
  // row itself round-trips under this codec, independent of whether the
  // encode check above passed.
  std::vector<uint8_t> decoded(WireRuntime::cobsDecodedMaxLength(v.expectedWire.size()));
  size_t decodedLen = 0;
  const bool decodeOk = WireRuntime::cobsDecode(v.expectedWire.empty() ? nullptr : v.expectedWire.data(),
                                                 v.expectedWire.size(), decoded.data(), decoded.size(), &decodedLen,
                                                 v.delimiter);
  if (!decodeOk) {
    fail(v.name, "decode: cobsDecode(fixture's expected_wire_hex) failed");
    return;
  }
  if (decodedLen < 2) {
    fail(v.name, "decode: decoded combined bytes too short to hold a CRC");
    return;
  }
  const size_t decodedPayloadLen = decodedLen - 2;
  std::vector<uint8_t> decodedPayload(decoded.begin(), decoded.begin() + static_cast<long>(decodedPayloadLen));
  if (decodedPayload != v.payload) {
    fail(v.name, "decode: recovered payload disagrees with the fixture's own payload_hex column");
  }
  const uint16_t receivedCrc =
      static_cast<uint16_t>(decoded[decodedPayloadLen]) | (static_cast<uint16_t>(decoded[decodedPayloadLen + 1]) << 8);
  if (receivedCrc != crc) {
    fail(v.name, "decode: recovered CRC does not match crcOverScope()'s own computation");
  }
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    std::fprintf(stderr, "usage: %s <path-to-wire_golden_vectors.txt>\n", argc > 0 ? argv[0] : "wire_golden_vector_harness");
    return 2;
  }

  const std::vector<GoldenVector> vectors = loadGoldenVectors(argv[1]);
  std::printf("--- shared golden-vector fixture: %zu vector(s) loaded from %s\n", vectors.size(), argv[1]);
  for (const GoldenVector& v : vectors) {
    checkVector(v);
  }

  if (g_failureCount == 0) {
    std::printf("OK: all %zu golden vector(s) passed (encode-matches-fixture + decode-round-trips)\n",
                 vectors.size());
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across %zu golden vector(s)\n", g_failureCount, vectors.size());
  return 1;
}
