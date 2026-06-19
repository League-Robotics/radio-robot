#pragma once
// 039-001: IColorSensor moved to source/io/capability/IColorSensor.h (content
// identical — already a clean capability interface). This shim forwards to the
// canonical location so existing consumers compile unchanged during the Phase A
// transition; it is deleted in Phase F.
#include "io/capability/IColorSensor.h"
