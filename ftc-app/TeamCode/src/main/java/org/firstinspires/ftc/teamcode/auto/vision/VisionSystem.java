package org.firstinspires.ftc.teamcode.auto.vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.math.Pose;
import org.firstinspires.ftc.teamcode.teleop.utility.Configuration;

public interface VisionSystem {
    enum SkystonePosition {
        LEFT("stone l"),
        CENTER("stone c"),
        RIGHT("stone r"),
        NONE("stone r");

        public final String key;
        SkystonePosition(String key) {
            this.key = key;
        }
    }

    enum CameraType {
        PHONE("Phone"), FRONT_WEBCAM("Webcam 1"), BACK_WEBCAM("Webcam 2");
        final String name;

        CameraType(String name) {
            this.name = name;
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

}
