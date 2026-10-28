#!/bin/sh
# micro_ros_platformio expects PlatformIO's virtualenv at $PLATFORMIO_CORE_DIR/penv
[ -f "$PLATFORMIO_CORE_DIR/penv/bin/activate" ] || \
    python3 -m venv --system-site-packages "$PLATFORMIO_CORE_DIR/penv"
exec "$@"
