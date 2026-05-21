#pragma once

#define cv_bridge_VERSION_MAJOR 3
#define cv_bridge_VERSION_MINOR 2
#define cv_bridge_VERSION_PATCH 1

#define cv_bridge_VERSION_GTE(major, minor, patch) \
    ((major < cv_bridge_VERSION_MAJOR)   ? true    \
     : (major > cv_bridge_VERSION_MAJOR) ? false   \
     : (minor < cv_bridge_VERSION_MINOR) ? true    \
     : (minor > cv_bridge_VERSION_MINOR) ? false   \
     : (patch < cv_bridge_VERSION_PATCH) ? true    \
     : (patch > cv_bridge_VERSION_PATCH) ? false   \
                                         : true)

#if cv_bridge_VERSION_GTE(3, 4, 0)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif
