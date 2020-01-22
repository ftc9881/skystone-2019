package org.firstinspires.ftc.teamcode.auto.vision;

import org.firstinspires.ftc.teamcode.math.Pose;
import org.firstinspires.ftc.teamcode.teleop.utility.Configuration;

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

    enum CameraType {
        PHONE("Phone"), FRONT_WEBCAM("Webcam 1"), BACK_WEBCAM("Webcam 2");
        final String name;
        final Pose displacement;
        final Pose rotation;

        private Configuration config = new Configuration("HardwareConstants");

        CameraType(String name) {
            this.name = name;
            String key = name.toLowerCase();
            Pose displacement = new Pose();
            displacement.x = config.getDouble(key + " dx", 0);
            displacement.y = config.getDouble(key + " dy", 0);
            displacement.r = config.getDouble(key + " dz", 0);
            Pose rotation = new Pose();
            rotation.x = config.getDouble(key + " rx", 0);
            rotation.y = config.getDouble(key + " ry", 0);
            rotation.r = config.getDouble(key + " rz", 0);
            this.displacement = displacement;
            this.rotation = rotation;
        }

        public static CameraType stringToType(String string) {
            switch (string.toUpperCase()) {
                case "PHONE":
                    return PHONE;
                case "WEBCAM 1":
                case "FRONT WEBCAM":
                default:
                    return FRONT_WEBCAM;
                case "WEBCAM 2":
                case "BACK WEBCAM":
                    return BACK_WEBCAM;
            }
        }
    }

    enum Type {VUFORIA, OPENCV}
    enum TargetType {
        SKYSTONE, BRIDGE, PERIMETER, ALL;

        public static TargetType stringToType(String string) {
            switch (string.toUpperCase()) {
                case "SKYSTONE":
                    return SKYSTONE;
                case "BRIDGE":
                    return BRIDGE;
                case "PERIMETER":
                    return PERIMETER;
                default:
                    return ALL;
            }
        }

    }

    void startLook(TargetType targetType);
    void stopLook();
    boolean found();

}
