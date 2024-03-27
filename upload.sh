
export DIR=$CARGO_TARGET_DIR/aarch64*/release

scp $DIR/gui \
    $DIR/watchdog \
    $DIR/i2c \
    $DIR/thermo \
    $DIR/urap-mod \
    $DIR/screw \
    $DIR/pid-autotune \
    operator@line1.local:/opt/firmware/bin/
