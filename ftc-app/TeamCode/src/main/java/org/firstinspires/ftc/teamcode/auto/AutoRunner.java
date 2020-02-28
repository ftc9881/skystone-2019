package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.auto.actions.MoveWithSensorAndOdometry;
import org.firstinspires.ftc.teamcode.auto.actions.Turn;
import org.firstinspires.ftc.teamcode.auto.endconditions.DeployElevatorByDistance;
import org.firstinspires.ftc.teamcode.auto.endconditions.DeployServoByDistance;
import org.firstinspires.ftc.teamcode.auto.endconditions.IWatchableDistance;
import org.firstinspires.ftc.teamcode.auto.endconditions.Timeout;
import org.firstinspires.ftc.teamcode.auto.structure.Action;
import org.firstinspires.ftc.teamcode.auto.structure.CombinedConditions;
import org.firstinspires.ftc.teamcode.auto.structure.Watcher;
import org.firstinspires.ftc.teamcode.auto.vision.OpenCV;
import org.firstinspires.ftc.teamcode.auto.vision.VisionSystem;
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
        batMobile = BatMobile.createInstance();

        for (Command command : config.initCommands) {
            String comment = command.getString("comment", "no comment.");
            if (!comment.equals("no comment.")) {
                AutoRunner.log(command.name, comment);
            }
            logAndTelemetry(TAG, "Init Command: " + command.name);
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

            case "MOVE": {
                boolean deployArm = command.getBoolean("deploy arm", false);
                boolean deployFoundation = command.getBoolean("deploy foundation", false);
                boolean deployLift = command.getBoolean("deploy lift", false);
                double timeoutMs = command.getDouble("timeout", 5 * 1000);

                MoveWithSensorAndOdometry move = new MoveWithSensorAndOdometry(command, skystonePosition);
                IEndCondition timeoutCondition = new Timeout(timeoutMs);
                CombinedConditions conditions = new CombinedConditions(timeoutCondition);

                IWatchableDistance trackingWatchable = command.getBoolean("track odometry", false) ? move : batMobile.driveTrain.rb;

                if (deployArm) {
                    double pivotDeployDistance = command.getDouble("pivot deploy at", -999);
                    if (pivotDeployDistance != -999) {
                        String pivotState = command.getString("pivot state", batMobile.getSideArm().pivot.getState().name());
                        Watcher deployPivot = new DeployServoByDistance(batMobile.getSideArm().pivot, ToggleServo.stringToState(pivotState), trackingWatchable, pivotDeployDistance);
                        conditions.add(deployPivot);
                    }
                    double clawDeployDistance = command.getDouble("claw deploy at", -999);
                    if (clawDeployDistance != -999) {
                        String clawState = command.getString("claw state", batMobile.getSideArm().claw.getState().name());
                        Watcher deployClaw = new DeployServoByDistance(batMobile.getSideArm().claw, ToggleServo.stringToState(clawState), trackingWatchable, clawDeployDistance);
                        conditions.add(deployClaw);
                    }

                }

                if (deployFoundation) {
                    double foundationDeployDistance = command.getDouble("foundation deploy at", -999);
                    if (foundationDeployDistance != -999) {
                        String foundationState = command.getString("foundation state", batMobile.leftFoundationServo.getState().name());
                        Watcher deployLeftFoundation = new DeployServoByDistance(batMobile.getSideArm().claw, ToggleServo.stringToState(foundationState), trackingWatchable, foundationDeployDistance);
                        Watcher deployRightFoundation = new DeployServoByDistance(batMobile.getSideArm().claw, ToggleServo.stringToState(foundationState), trackingWatchable, foundationDeployDistance);
                        conditions.add(deployLeftFoundation, deployRightFoundation);
                    }
                }

                if (deployLift) {
                    double liftPower1 = command.getDouble("lift power 1", 0);
                    double liftPower2 = command.getDouble("lift power 2", 0);
                    double deployDistance1 = command.getDouble("deploy inches 1", 0f);
                    double deployDistance2 = command.getDouble("pivot deploy inches 2", 0);
                    Watcher deployElevator1 = new DeployElevatorByDistance(liftPower1, move, deployDistance1);
                    Watcher deployElevator2 = new DeployElevatorByDistance(liftPower2, move, deployDistance2);
                    conditions.add(deployElevator1, deployElevator2);
                }

                move.runSynchronized(conditions);
                break;
            }

            case "RESET CLICKS": {
                batMobile.odometryY.resetEncoder();
                break;
            }

            case "LOG": {
                logAndTelemetry("Odometry:Clicks", batMobile.odometryY.getClicks());
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
//                if (feedbackState) {
//                    // after touching, set so servo not stalling
//                    batMobile.getSideArm().setPivotToInsideRestingPosition();
//                }
                break;
            }

            case "ELEVATOR": {
                double liftPower = command.getDouble("lift power", 0);
                double extendPower = command.getDouble("extend power", 0);
                batMobile.elevator.setPowerLE(liftPower, extendPower);
                break;
            }

            case "TURN": {
                boolean deployArm = command.getBoolean("deploy arm", false);
                double timeoutMs = command.getDouble("timeout", 5 * 1000);
                int deployClicks = command.getInt("deploy clicks", 0);
                IEndCondition timeoutCondition = new Timeout(timeoutMs);
                CombinedConditions conditions = new CombinedConditions(timeoutCondition);
                if (deployArm) {
                    String pivotState = command.getString("pivot state", batMobile.getSideArm().pivot.getState().name());
                    String clawState = command.getString("claw state", batMobile.getSideArm().claw.getState().name());
                    Watcher deployPivot = new DeployServoByDistance(batMobile.getSideArm().pivot, ToggleServo.stringToState(pivotState), robot.driveTrain.rf, deployClicks);
                    Watcher deployClaw = new DeployServoByDistance(batMobile.getSideArm().claw, ToggleServo.stringToState(clawState), robot.driveTrain.rf, deployClicks);
                    conditions.add(deployClaw, deployPivot);
                }
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

    public void logAndTelemetry(Object message) {
        logAndTelemetry("Somewhere", message);
    }

    public static void setSkystonePosition(VisionSystem.SkystonePosition position) {
        AutoRunner.skystonePosition = position;
    }


}
