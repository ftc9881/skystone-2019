# copy teamcode to given app directory (passed as argument)
TARGET_DIR=$1
rm -rf $TARGET_DIR/teamcode
cp -rf ./teamcode $TARGET_DIR
