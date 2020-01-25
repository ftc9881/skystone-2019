#!/bin/sh
CONFIG_FILE=$1
# $ANDROID_SDK_HOME/platform-tools/adb pull sdcard/Robot/$CONFIG_FILE ../configs/
vim ../configs/$CONFIG_FILE
$ANDROID_SDK_HOME/platform-tools/adb push ../configs/$CONFIG_FILE sdcard/Robot/$CONFIG_FILE
