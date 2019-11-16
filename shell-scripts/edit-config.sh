#!/bin/sh
CONFIG_FILE=$1
~/Android/Sdk/platform-tools/adb pull sdcard/Robot/$CONFIG_FILE
vim $CONFIG_FILE
~/Android/Sdk/platform-tools/adb push $CONFIG_FILE sdcard/Robot/$CONFIG_FILE
