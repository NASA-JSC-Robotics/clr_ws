#!/bin/bash

# Pixi will swallow stdout/stderr, so redirect output straight to the tty
# so it shows up.
message() { echo "[pixi] $*" > /dev/tty; }

if [ -f "${PIXI_PROJECT_ROOT}/install/setup.bash" ]; then
    message "Sourcing ${PIXI_PROJECT_ROOT}/install/setup.bash"
    # shellcheck source=/dev/null
    source "${PIXI_PROJECT_ROOT}/install/setup.bash"
else
    message "The ${PIXI_PROJECT_ROOT} workspace is not yet built."
    message "To build:"
    message "  cd ${PIXI_PROJECT_ROOT}"
    message "  colcon build"
fi
