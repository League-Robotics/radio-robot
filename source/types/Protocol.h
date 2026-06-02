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

using ReplyFn = void(*)(const char* msg, void* ctx);

struct ReplyCtx {
    bool viaSerial;
    bool viaRadio;
    bool relay;
};
