#!/bin/sh
$ANDROID_SDK_HOME/platform-tools/adb kill-server
$ANDROID_SDK_HOME/platform-tools/adb start-server
$ANDROID_SDK_HOME/platform-tools/adb connect 192.168.43.1:5555
