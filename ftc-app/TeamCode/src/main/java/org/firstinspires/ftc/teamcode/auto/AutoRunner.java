package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.auto.actions.RelativeMove;
import org.firstinspires.ftc.teamcode.auto.actions.AbsoluteTurn;
import org.firstinspires.ftc.teamcode.auto.actions.RelativeMoveWithVuforia;
import org.firstinspires.ftc.teamcode.auto.endconditions.DeployServoByDistance;
import org.firstinspires.ftc.teamcode.auto.endconditions.Timeout;
import org.firstinspires.ftc.teamcode.auto.structure.Action;
import org.firstinspires.ftc.teamcode.auto.structure.CombinedConditions;
import org.firstinspires.ftc.teamcode.auto.structure.SomethingBadHappened;
import org.firstinspires.ftc.teamcode.auto.structure.Watcher;
import org.firstinspires.ftc.teamcode.auto.vision.OpenCV;
import org.firstinspires.ftc.teamcode.auto.vision.VisionSystem;
import org.firstinspires.ftc.teamcode.auto.vision.Vuforia;
import org.firstinspires.ftc.teamcode.teleop.utility.Command;
import org.firstinspires.ftc.teamcode.auto.structure.AutoOpConfiguration;
import org.firstinspires.ftc.teamcode.auto.structure.IEndCondition;
import org.firstinspires.ftc.teamcode.robot.BatMobile.BatMobile;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.devices.ToggleServo;
import org.firstinspires.ftc.teamcode.teleop.utility.Button;

import static android.os.SystemClock.sleep;

/**
 * AutoRunner is the root for all autonomous OpModes.
 * This will read a set of commands from a text configuration file
 * and execute them in order.
 */
public class AutoRunner {

    private static final String TAG_PREFIX = "TeamCode@";
    private static final String TAG = "AutoRunner";
    private static final long SLEEP_LOOP_TIME = 20;

    private AutoOpConfiguration config;
    private LinearOpMode opMode;
    private Robot robot;

    private static AngleUnit angleUnit = AngleUnit.DEGREES;
    private boolean debugMode;
    private boolean stopped = false;

    private VisionSystem.SkystonePosition skystonePosition = VisionSystem.SkystonePosition.CENTER;

    public AutoRunner(String name, LinearOpMode opMode) {
        opMode.msStuckDetectStop = 10000;
        this.opMode = opMode;

        config = AutoOpConfiguration.newInstance(name + ".json");

        angleUnit = config.properties.getAngleUnit("angle unit", AngleUnit.DEGREES);
        debugMode = config.properties.getBoolean("debug mode", false);

        robot = Robot.newInstance(opMode);
        robot.initializeImu(angleUnit);

        for (Command command : config.initCommands) {
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
            logAndTelemetry(TAG, "Command: " + command.name);
            if (shouldStop()) {
                logAndTelemetry(TAG, "STOPPING...");
                return;
            }
            try {
                execute(command);
            } catch (SomethingBadHappened exception) {
                stopped = true;
            }
            waitIfDebugMode();
        }
    }

    private boolean shouldStop() {
        return stopped || !opMode.opModeIsActive();
    }

    private void waitIfDebugMode() {
        if (debugMode) {
            logAndTelemetry(TAG, "DEBUG BREAKPOINT");
            waitUntilButtonPressed();
        }
    }

    private void execute(Command command) {

        if (command.name.contains("SKIP")) {
            return;
        }

        switch (command.name) {

            case "INIT VUFORIA": {
                Vuforia.createInstance(opMode.hardwareMap, VisionSystem.CameraType.FRONT_WEBCAM);
                break;
            }

            case "IDENTIFY SKYSTONE": {
                OpenCV openCV = new OpenCV();
                openCV.startLook(VisionSystem.TargetType.SKYSTONE);
                skystonePosition = openCV.identifyPosition(opMode);
                logAndTelemetry("Skystone Position", skystonePosition.name());
                break;
            }

            case "MOVE": {
                BatMobile batMobile = BatMobile.getInstance();
                boolean deployArm = command.getBoolean("deploy arm", false);
                boolean useVuforia = command.getBoolean("use vuforia", false);
                double timeoutMs = command.getDouble("timeout", 5 * 1000);
                int deployClicks = command.getInt("deploy clicks", 0);
                String pivotState = command.getString("pivot state", batMobile.sideArm.pivot.getState().name());
                String clawState = command.getString("claw state", batMobile.sideArm.claw.getState().name());
                Action move = useVuforia? new RelativeMoveWithVuforia(command, skystonePosition) : new RelativeMove(command, skystonePosition);
                IEndCondition timeoutCondition = new Timeout(timeoutMs);
                CombinedConditions conditions = new CombinedConditions(timeoutCondition);
                if (deployArm) {
                    Watcher deployPivot = new DeployServoByDistance(batMobile.sideArm.pivot, ToggleServo.stringToState(pivotState), robot.driveTrain.rf, deployClicks);
                    Watcher deployClaw = new DeployServoByDistance(batMobile.sideArm.claw, ToggleServo.stringToState(clawState), robot.driveTrain.rf, deployClicks);
                    conditions.add(deployClaw, deployPivot);
                }
                runActionWithCondition(move, conditions);
                break;
            }

            case "MOVE VUFORIA": {
                Action relativeMoveVuforia = new RelativeMoveWithVuforia(command, skystonePosition);
                runActionWithTimeout(relativeMoveVuforia, command);
                break;
            }

            case "CAPSTONE": {
                BatMobile batMobile = BatMobile.getInstance();
                String state = command.getString("state", "REST");
                batMobile.capstoneServo.set(ToggleServo.stringToState(state));
            }

            case "CLAW": {
                BatMobile batMobile = BatMobile.getInstance();
                String state = command.getString("state", "REST");
                batMobile.sideArm.claw.set(ToggleServo.stringToState(state));
                break;
            }

            case "PIVOT": {
                BatMobile batMobile = BatMobile.getInstance();
                String state = command.getString("state", "REST");
                batMobile.sideArm.pivot.set(ToggleServo.stringToState(state));
                break;
            }

            case "TURN": {
                Action relativeTurn = new AbsoluteTurn(command);
                runActionWithTimeout(relativeTurn, command);
                break;
            }

            case "INTAKE": {
                BatMobile batMobile = BatMobile.getInstance();
                double power = command.getDouble("power", 0.5);
                batMobile.intake.setPower(power);
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
        runActionWithCondition(action, timeoutCondition);
    }

    private void runActionWithCondition(Action action, IEndCondition endCondition) {
        action.start();
        endCondition.start();

        // If end condition completes before action, then action is stopped.
        // If action completes before end condition, then complete.
        while (!action.isStopped() && !endCondition.isTrue()) {
            sleep(SLEEP_LOOP_TIME);
        }

        action.stop();
        endCondition.stop();

        log(TAG, "Run task completed");
    }

    private void waitUntilButtonPressed() {
        Button button = new Button();
        while (!button.is(Button.State.DOWN) && opMode.opModeIsActive()) {
            boolean anyButton = opMode.gamepad1.a || opMode.gamepad1.b || opMode.gamepad1.x || opMode.gamepad1.y ||
                    opMode.gamepad2.a || opMode.gamepad2.b || opMode.gamepad2.x || opMode.gamepad2.y;
            button.update(anyButton);
            sleep(20);
        }
    }

    public static void log(String tag, Object message) {
        RobotLog.dd(TAG_PREFIX + tag, message.toString());
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


}
