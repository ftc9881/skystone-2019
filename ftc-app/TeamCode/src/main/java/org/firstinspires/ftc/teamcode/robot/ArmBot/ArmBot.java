package org.firstinspires.ftc.teamcode.robot.ArmBot;

import org.firstinspires.ftc.teamcode.robot.Intake;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.hardware.servo.ToggleServo;

public class ArmBot extends Robot {

    // Singleton pattern; constructor is private and enable only one instance at a time
    /*
        CD: Singleton isn't necessary here.
        CD: You should have an initialize() method here (and in Robot()) so that you can ensure that it is in the proper state before executing.   Have the object lifecycle clearly defined.
     */
    private static ArmBot instance;

    public static ArmBot getInstance() {
        /*
            CD: This is a little strange.  I don't think you should need to pass a Robot to the constructor in a singleton pattern.
         */
        Robot robot = Robot.getInstance();
        if (instance == null && robot != null) {
            instance = new ArmBot(robot);
        }
        return instance;
    }

    public ToggleServo stoneServo;
    public ToggleServo capstoneServo;
    public ToggleServo foundationServo;

    public Arm arm;
    public Intake intake;

    private ArmBot(Robot robot) {
        /*
            CD: These aren't fields of ArmBot and should be set in Robot
            CD: You access fields directly here (and elsewhere).  It's a good idea in general to wrap
                    fields in getters/setters to abstract things from accessing classes.
         */
        this.opMode = robot.opMode;
        this.hardwareMap = robot.hardwareMap;
        this.imu = robot.imu;
        this.driveTrain = robot.driveTrain;

        arm = new Arm(hardwareMap);
        intake = new Intake(hardwareMap);

        stoneServo = new ToggleServo(hardwareMap, "stone");
        capstoneServo = new ToggleServo(hardwareMap, "capstone");
        foundationServo = new ToggleServo(hardwareMap, "foundation");
    }

}
