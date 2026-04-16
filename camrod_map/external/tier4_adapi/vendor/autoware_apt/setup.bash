#!/usr/bin/env bash
# HH_260408: Local non-root overlay for Autoware deb-extracted dependencies.
_VEND_PREFIX="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/extract/opt/ros/humble"
if [ -d "${_VEND_PREFIX}" ]; then
  export AMENT_PREFIX_PATH="${_VEND_PREFIX}:${AMENT_PREFIX_PATH}"
  export CMAKE_PREFIX_PATH="${_VEND_PREFIX}:${CMAKE_PREFIX_PATH}"
  export LD_LIBRARY_PATH="${_VEND_PREFIX}/lib:${_VEND_PREFIX}/local/lib:${LD_LIBRARY_PATH}"
  export PYTHONPATH="${_VEND_PREFIX}/local/lib/python3.10/dist-packages:${_VEND_PREFIX}/lib/python3.10/site-packages:${PYTHONPATH}"
  export PATH="${_VEND_PREFIX}/bin:${PATH}"
fi
unset _VEND_PREFIX
