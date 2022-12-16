SCRIPTS_DIR=$(cd $(dirname "$0"); pwd)
PKG_PATH=$(dirname $SCRIPTS_DIR)

ARDUINO_LIBRARY_DIR=$PKG_PATH/arduino_libraries
ARDUINO_SKETCH_DIR=$PKG_PATH/sketches

echo "Create symlink from arduino_libraries directory to each arduino sketches."
for SKETCH in $(ls $ARDUINO_SKETCH_DIR); do
    SKETCH_DIR=$ARDUINO_SKETCH_DIR/$SKETCH
    if [ -d $SKETCH_DIR ]; then
        FROM=$ARDUINO_LIBRARY_DIR
        TO=$ARDUINO_SKETCH_DIR/$SKETCH/symlink_libs
        if [ -d $TO ]; then
            rm -f $TO
            echo "Old symlink is removed: $TO"
        fi
        ln -s $FROM $TO
        echo "Created symlink: $FROM -> $TO"
    fi
done
