package org.firstinspires.ftc.teamcode.auto.vision;

public interface VisionSystem {
    enum TargetType {SKYSTONE, BRIDGE, PERIMETER, NONE_JUST_RUN_FOREVER}

    void initialize();
    void startLook(TargetType targetType);
    void stopLook();
    boolean found();
}
