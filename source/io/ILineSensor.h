#pragma once
// 039-001: ILineSensor moved to source/io/capability/ILineSensor.h (content
// identical — already a clean capability interface). This shim forwards to the
// canonical location so existing consumers compile unchanged during the Phase A
// transition; it is deleted in Phase F.
#include "io/capability/ILineSensor.h"
