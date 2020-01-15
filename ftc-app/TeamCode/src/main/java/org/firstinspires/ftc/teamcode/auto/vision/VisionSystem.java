package org.firstinspires.ftc.teamcode.auto.vision;

public interface VisionSystem {
    enum SkystonePosition {
        LEFT("stone l"),
        CENTER("stone c"),
        RIGHT("stone r"),
        NONE("stone ?");

        public final String key;
        SkystonePosition(String key) {
            this.key = key;
        }
    }
    enum Type {VUFORIA, OPENCV}
    enum TargetType {SKYSTONE, BRIDGE, PERIMETER, RUN_FOREVER}

    void initialize();
    void startLook(TargetType targetType);
    void stopLook();
    boolean found();
}
