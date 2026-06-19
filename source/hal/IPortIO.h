#pragma once
// 039-001: IPortIO moved to source/io/capability/IPortIO.h (content identical —
// already a clean capability interface). This shim forwards to the canonical
// location so existing consumers compile unchanged during the Phase A
// transition; it is deleted in Phase F.
#include "io/capability/IPortIO.h"
