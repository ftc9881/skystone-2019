#!/bin/sh
$ANDROID_SDK_HOME/platform-tools/adb pull storage/emulated/0/Pictures .
$ANDROID_SDK_HOME/platform-tools/adb shell rm sdcard/Pictures/*
