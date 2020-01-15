package org.firstinspires.ftc.teamcode.auto.vision;

public interface VisionSystem {
    enum SkystonePosition {LEFT, CENTER, RIGHT, NONE}
    enum Type {VUFORIA, OPENCV}
    enum TargetType {SKYSTONE, BRIDGE, PERIMETER, RUN_FOREVER}

    void initialize();
    void startLook(TargetType targetType);
    void stopLook();
    boolean found();
}
