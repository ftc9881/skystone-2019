package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.auto.actions.MoveWithClicks;
import org.firstinspires.ftc.teamcode.auto.actions.MoveWithSensorAndOdometry;
import org.firstinspires.ftc.teamcode.auto.actions.Turn;
import org.firstinspires.ftc.teamcode.auto.endconditions.ChangeDriveModeByDistance;
import org.firstinspires.ftc.teamcode.auto.endconditions.DeployElevatorByDistance;
import org.firstinspires.ftc.teamcode.auto.endconditions.DeployServoByDistance;
import org.firstinspires.ftc.teamcode.auto.endconditions.IWatchableDistance;
import org.firstinspires.ftc.teamcode.auto.endconditions.Timeout;
import org.firstinspires.ftc.teamcode.auto.structure.Action;
import org.firstinspires.ftc.teamcode.auto.structure.CombinedConditions;
import org.firstinspires.ftc.teamcode.auto.structure.Watcher;
import org.firstinspires.ftc.teamcode.auto.vision.OpenCV;
import org.firstinspires.ftc.teamcode.auto.vision.VisionSystem;
import org.firstinspires.ftc.teamcode.math.GeneralMath;
import org.firstinspires.ftc.teamcode.teleop.utility.Command;
import org.firstinspires.ftc.teamcode.auto.structure.AutoOpConfiguration;
import org.firstinspires.ftc.teamcode.auto.structure.IEndCondition;
import org.firstinspires.ftc.teamcode.robot.BatMobile.BatMobile;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.hardware.servo.ToggleServo;
import org.firstinspires.ftc.teamcode.teleop.utility.Button;

import static android.os.SystemClock.sleep;

/**
 * AutoRunner is the root for all autonomous OpModes.
 * This will read a set of commands from a text configuration file
 * and execute them in order.
 */
public class AutoRunner {

    public enum Side {
        RED, BLUE;
        public String getKey() {
            switch (this) {
                case RED:
                    return "red";
                case BLUE:
                    return "blue";
                default:
                    return "";
            }
        }

    }

    private static Side side = Side.RED;
    private static VisionSystem.SkystonePosition skystonePosition = VisionSystem.SkystonePosition.NONE;

    private static final String TAG_PREFIX = "TeamCode@";
    private static final String TAG = "AutoRunner";

    private AutoOpConfiguration config;
    private LinearOpMode opMode;
    private Robot robot;
    private BatMobile batMobile;

    private static AngleUnit angleUnit = AngleUnit.DEGREES;
    private boolean debugMode;
    private boolean stopped = false;
    private static boolean flipForBlue = false;

    public AutoRunner(String name, LinearOpMode opMode) {
        this(Side.RED, name, opMode);
    }

    public AutoRunner(Side side, String name, LinearOpMode opMode) {
        AutoRunner.side = side;

        opMode.msStuckDetectStop = 3000;
        this.opMode = opMode;

        config = AutoOpConfiguration.newInstance(name + ".json");

        angleUnit = config.properties.getAngleUnit("angle unit", AngleUnit.DEGREES);
        debugMode = config.properties.getBoolean("debug mode", false);

        robot = Robot.newInstance(opMode);
        robot.initializeIMU();
        robot.driveTrain.setVelocityPIDF();
        batMobile = BatMobile.createInstance();

        flipForBlue = side == Side.BLUE && config.properties.getBoolean("flip for blue", false);

        stopped = batMobile.sensorDead() || batMobile.getSideSensor().getDistance() > 300;

        for (Command command : config.initCommands) {
            String comment = command.getString("comment", "no comment.");
            if (!comment.equals("no comment.")) {
                AutoRunner.log("Comment:"+command.name, comment);
            }
            logAndTelemetry(TAG, "Init Command:" + command.name);
            execute(command);
        }

        logAndTelemetry(TAG, "Ready to run");
    }

    public static AngleUnit getAngleUnit() {
        return angleUnit;
    }

    public void run() {
        for (Command command : config.commands) {
            if (shouldStop()) {
                logAndTelemetry(TAG, "STOPPING...");
                return;
            }
            logAndTelemetry(TAG, "Command: " + command.name);
            execute(command);

            robot.driveTrain.stop();

            waitIfDebugMode();
        }
    }

    private boolean shouldStop() {
        return stopped || !opMode.opModeIsActive() || opMode.isStopRequested();
    }

    private boolean opModeIsActive() {
        return !shouldStop();
    }

    private void waitIfDebugMode() {
        if (debugMode) {
            logAndTelemetry(TAG, "DEBUG BREAKPOINT");
            waitUntilButtonPressed();
        }
    }

    public static Side getSide() {
        return side;
    }

    private void execute(Command command) {

        if (command.name.contains("SKIP")) {
            return;
        }

        switch (command.name) {

            case "USE ENCODERS": {
                batMobile.setToUseEncoder();
                break;
            }

            case "PIVOTS INSIDE": {
                batMobile.redSideArm.setPivotToInsideRestingPosition();
                batMobile.blueSideArm.setPivotToInsideRestingPosition();
                break;
            }

            case "PIVOTS DEPLOY": {
                batMobile.redSideArm.setPivotToDeployIntakePosition();
                batMobile.blueSideArm.setPivotToDeployIntakePosition();
                break;
            }

            case "IDENTIFY SKYSTONE": {
                OpenCV vision = new OpenCV(config.properties, VisionSystem.CameraType.FRONT_WEBCAM);
                vision.startLook(VisionSystem.TargetType.SKYSTONE);
                vision.startIdentifyingSkystonePosition(opMode);
                break;
            }

            case "INIT LOG": {
                while (!opMode.isStarted()) {
                    opMode.telemetry.addData("Skystone-", side == Side.RED ? skystonePosition : getFlipped(skystonePosition));
                    opMode.telemetry.addData("IMU------", GeneralMath.round(robot.imu.getIntegratedHeading().getDegrees(), 3));
                    opMode.telemetry.addData("OdometryY", GeneralMath.round(batMobile.odometryY.getInches(), 3));
                    opMode.telemetry.addData("SensorL--", GeneralMath.round(batMobile.leftSensor.getDistance(), 3));
                    opMode.telemetry.addData("SensorR--", GeneralMath.round(batMobile.rightSensor.getDistance(), 3));
                    opMode.telemetry.update();
                }
                break;
            }


            case "MOVE BY CLICKS": {
                MoveWithClicks move = new MoveWithClicks(command);
                runActionWithTimeout(move, command);
                break;
            }


            case "MOVE": {
                boolean deployArm = command.getBoolean("deploy arm", false);
                boolean deployFoundation = command.getBoolean("deploy foundation", false);
                boolean deployLift = command.getBoolean("deploy lift", false);
                boolean changeMode = command.getBoolean("change drive mode", false);
                double timeoutMs = command.getDouble("timeout", 5 * 1000);
                double blueOffsetY = AutoRunner.getSide() == AutoRunner.Side.BLUE ? command.getDouble("y blue offset", 0) : 0;

                MoveWithSensorAndOdometry move = new MoveWithSensorAndOdometry(command, skystonePosition);
                IEndCondition timeoutCondition = new Timeout(timeoutMs);
                CombinedConditions conditions = new CombinedConditions(timeoutCondition);

                IWatchableDistance trackingWatchable = command.getBoolean("track odometry", false) ? move : batMobile.driveTrain.rb;

                if (deployArm) {
                    double pivotDeployDistance = command.getDouble("pivot deploy at", -999);
                    if (pivotDeployDistance != -999) {
                        String pivotState = command.getString("pivot state", batMobile.getSideArm().pivot.getState().name());
                        Watcher deployPivot = new DeployServoByDistance(batMobile.getSideArm().pivot, ToggleServo.stringToState(pivotState), trackingWatchable, pivotDeployDistance + blueOffsetY);
                        conditions.add(deployPivot);
                    }
                    double pivotDeployDistance2 = command.getDouble("pivot deploy at 2", -999);
                    if (pivotDeployDistance2 != -999) {
                        String pivotState2 = command.getString("pivot state 2", batMobile.getSideArm().pivot.getState().name());
                        Watcher deployPivot2 = new DeployServoByDistance(batMobile.getSideArm().pivot, ToggleServo.stringToState(pivotState2), trackingWatchable, pivotDeployDistance2 + blueOffsetY);
                        conditions.add(deployPivot2);
                    }
                    double clawDeployDistance = command.getDouble("claw deploy at", -999);
                    if (clawDeployDistance != -999) {
                        String clawState = command.getString("claw state", batMobile.getSideArm().claw.getState().name());
                        Watcher deployClaw = new DeployServoByDistance(batMobile.getSideArm().claw, ToggleServo.stringToState(clawState), trackingWatchable, clawDeployDistance + blueOffsetY);
                        conditions.add(deployClaw);
                    }
                }

                if (deployFoundation) {
                    double foundationDeployDistance = command.getDouble("foundation deploy at", -999);
                    if (foundationDeployDistance != -999) {
                        String foundationState = command.getString("foundation state", batMobile.leftFoundationServo.getState().name());
                        Watcher deployLeftFoundation = new DeployServoByDistance(batMobile.leftFoundationServo, ToggleServo.stringToState(foundationState), trackingWatchable, foundationDeployDistance);
                        Watcher deployRightFoundation = new DeployServoByDistance(batMobile.rightFoundationServo, ToggleServo.stringToState(foundationState), trackingWatchable, foundationDeployDistance);
                        conditions.add(deployLeftFoundation, deployRightFoundation);
                    }
                }

                if (deployLift) {
                    double liftPower1 = command.getDouble("lift power 1", 0);
                    double liftPower2 = command.getDouble("lift power 2", 0);
                    double deployDistance1 = command.getDouble("deploy inches 1", 0f);
                    double deployDistance2 = command.getDouble("deploy inches 2", 0);
                    Watcher deployElevator1 = new DeployElevatorByDistance(liftPower1, move, deployDistance1);
                    Watcher deployElevator2 = new DeployElevatorByDistance(liftPower2, move, deployDistance2);
                    conditions.add(deployElevator1, deployElevator2);
                }

                if (changeMode) {
                    double deployDistance = command.getDouble("change at", 0);
                    boolean useVelocity = command.getBoolean("use velocity", false);
                    DcMotor.RunMode mode = useVelocity ? DcMotor.RunMode.RUN_USING_ENCODER : DcMotor.RunMode.RUN_WITHOUT_ENCODER;
                    Watcher changeModeWatcher = new ChangeDriveModeByDistance(move, deployDistance, mode);
                    conditions.add(changeModeWatcher);
                }

                move.runSynchronized(conditions);
                break;
            }

            case "SET DRIVE MODE": {
                boolean useVelocity = command.getBoolean("use velocity", false);
                robot.driveTrain.setMode(useVelocity ? DcMotor.RunMode.RUN_USING_ENCODER : DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                break;
            }

            case "RESET CLICKS": {
                batMobile.odometryY.resetEncoder();
                break;
            }

            case "LOG": {
                logAndTelemetry("Odometry:Inches", batMobile.odometryY.getInches());
                break;
            }

            case "FOUNDATION": {
                String state = command.getString("state", "REST");
                batMobile.rightFoundationServo.set(ToggleServo.stringToState(state));
                batMobile.leftFoundationServo.set(ToggleServo.stringToState(state));
                break;
            }

            case "CLAW": {
                String state = command.getString("state", "REST");
                batMobile.getSideArm().claw.set(ToggleServo.stringToState(state));
                break;
            }

            case "PIVOT": {
                String state = command.getString("state", "REST");
                batMobile.getSideArm().pivot.set(ToggleServo.stringToState(state));
                break;
            }

            case "PIVOT WAIT": {
                boolean feedbackState = command.getBoolean("touch", true);
                int defaultMinPivotWait = config.properties.getInt("pivot min wait", 0);
                int defaultMaxPivotWait = config.properties.getInt("pivot max wait", 1000);
                int minWait = command.getInt("min wait", defaultMinPivotWait);
                int maxWait = command.getInt("max wait", defaultMaxPivotWait);
                sleep(minWait);
                long start = System.currentTimeMillis();

                while (batMobile.getSideArm().feedbackIsPressed() != feedbackState && System.currentTimeMillis() - start < maxWait) {
                    sleep(20);
                }
                break;
            }

            case "ELEVATOR": {
                double liftPower = command.getDouble("lift power", 0);
                double extendPower = command.getDouble("extend power", 0);
                batMobile.elevator.setPowerLE(liftPower, extendPower);
                break;
            }

            case "TURN": {
                Action turn = new Turn(command);
                runActionWithTimeout(turn, command);
                break;
            }

            case "SLEEP": {
                long timeMs = (long) command.getDouble("time", 1000);
                sleep(timeMs);
                break;
            }

            case "BREAKPOINT": {
                waitUntilButtonPressed();
                break;
            }

            case "STOP": {
                stopped = true;
                break;
            }

            default: {
                logAndTelemetry("Command not exist");
                break;
            }
        }
    }

    private void runActionWithTimeout(Action action, Command command) {
        double timeoutMs = command.getDouble("timeout", 5 * 1000);
        IEndCondition timeoutCondition = new Timeout(timeoutMs);
        action.runSynchronized(timeoutCondition);
    }

    private void waitUntilButtonPressed() {
        Button button = new Button();
        while (!button.is(Button.State.DOWN) && opModeIsActive()) {
            boolean anyButton = opMode.gamepad1.a || opMode.gamepad1.b || opMode.gamepad1.x || opMode.gamepad1.y ||
                    opMode.gamepad2.a || opMode.gamepad2.b || opMode.gamepad2.x || opMode.gamepad2.y;
            button.update(anyButton);
            sleep(20);
        }
    }

    public static void log(String tag, Object message) {
        RobotLog.dd(TAG_PREFIX + tag, message.toString());
        Telemetry dashboard = FtcDashboard.getInstance().getTelemetry();
        dashboard.addData(tag, message.toString());
        dashboard.update();
//        TelemetryPacket packet = new TelemetryPacket();
//        packet.put(tag, message.toString());
//        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    public static void log(Object message) {
        log("Somewhere", message);
    }

    public static AngleUnit getAngleUnits() {
        return angleUnit;
    }


    public void logAndTelemetry(String tag, Object message) {
        AutoRunner.log(tag, message);
        opMode.telemetry.addData(TAG_PREFIX + tag, message);
        opMode.telemetry.update();
    }

    public static int signIfFlipForBlue() {
        return flipForBlue ? -1 : 1;
    }

    public void logAndTelemetry(Object message) {
        logAndTelemetry("Somewhere", message);
    }

    public static void setSkystonePosition(VisionSystem.SkystonePosition position) {
        if (!flipForBlue) {
            AutoRunner.skystonePosition = position;
        } else {
            AutoRunner.skystonePosition = getFlipped(position);

        }
    }

    private static VisionSystem.SkystonePosition getFlipped(VisionSystem.SkystonePosition position) {
        switch (position) {
            case LEFT:
                return VisionSystem.SkystonePosition.RIGHT;
            case RIGHT:
                return VisionSystem.SkystonePosition.LEFT;
            default:
                return AutoRunner.skystonePosition = position;
        }
    }


}
