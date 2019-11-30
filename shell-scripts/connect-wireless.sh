#!/bin/sh
$ANDROID_SDK_HOME/platform-tools/adb tcpip 5555
$ANDROID_SDK_HOME/platform-tools/adb connect 192.168.49.1:5555