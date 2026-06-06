#include "RadioChannel.h"

namespace radiochan {

// Key-value store key. Max KEY_VALUE_STORAGE_KEY_SIZE bytes; "rfchan" fits.
static const char* kKey = "rfchan";

int load(MicroBitStorage& storage)
{
    KeyValuePair* kv = storage.get(kKey);
    if (kv == nullptr) {
        return kDefault;
    }
    int c = (int)kv->value[0];
    delete kv;                       // CODAL: get() returns a heap pair (see MicroBit.cpp)
    if (c < kMin || c > kMax) {
        return kDefault;
    }
    return c;
}

void save(MicroBitStorage& storage, int channel)
{
    uint8_t v = (uint8_t)clamp(channel);
    storage.put(kKey, &v, 1);
}

}  // namespace radiochan
