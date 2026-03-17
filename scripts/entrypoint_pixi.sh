#!/bin/bash

# I haven't gotten printouts working in pixi, but oh well, this should technically work?
if [ -f "${PIXI_PROJECT_ROOT}/install/setup.bash" ]; then
    echo "Sourcing ${PIXI_PROJECT_ROOT}/install/setup.bash"
    # shellcheck source=/dev/null
    source "${PIXI_PROJECT_ROOT}/install/setup.bash"
else
    echo "The ${PIXI_PROJECT_ROOT} workspace is not yet built."
    echo "To build:"
    echo "  cd ${PIXI_PROJECT_ROOT}"
    echo "  colcon build"
fi
