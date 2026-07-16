#pragma once
#include <stdint.h>

// ---------------------------------------------------------------------------
// Protocol v2 response tag constants
// ---------------------------------------------------------------------------
constexpr const char* PROTO_TAG_OK  = "OK";
constexpr const char* PROTO_TAG_ERR = "ERR";
constexpr const char* PROTO_TAG_EVT = "EVT";
constexpr const char* PROTO_TAG_TLM = "TLM";
constexpr const char* PROTO_TAG_CFG = "CFG";
constexpr const char* PROTO_TAG_ID  = "ID";

// ---------------------------------------------------------------------------
// Protocol version and firmware version string
// ---------------------------------------------------------------------------
constexpr int         PROTO_VERSION    = 2;

// FIRMWARE_VERSION_STR is emitted by scripts/gen_version.py (run from build.py's
// codegen step) into the generated, git-ignored header version_generated.h,
// sourced from pyproject.toml -- so VER/ID report the actual build version
// instead of a hand-edited constant that silently drifted (it was stuck at
// 0.20260704.6 across many bumps). The fallback keeps clangd and any
// codegen-less compile (e.g. the ad-hoc test harnesses) building.
#if __has_include("version_generated.h")
#include "version_generated.h"
#endif
#ifndef FIRMWARE_VERSION_STR
#define FIRMWARE_VERSION_STR "0.0.0-dev"
#endif
constexpr const char* FIRMWARE_VERSION = FIRMWARE_VERSION_STR;

using ReplyFn = void(*)(const char* msg, void* ctx);

struct ReplyCtx {
    bool viaSerial;
    bool viaRadio;
    bool relay;
};

// ---------------------------------------------------------------------------
// KVPair — a single key=value token pair. Used by parseKV() and ParseFn.
// Keys and values point into the working copy buffer; callers must not free.
// ---------------------------------------------------------------------------
struct KVPair {
    const char* key;
    const char* value;
};
