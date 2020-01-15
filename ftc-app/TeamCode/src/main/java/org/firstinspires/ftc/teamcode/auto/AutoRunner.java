package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.auto.actions.RelativeMove;
import org.firstinspires.ftc.teamcode.auto.actions.AbsoluteTurn;
import org.firstinspires.ftc.teamcode.auto.actions.RelativeMoveWithVuforia;
import org.firstinspires.ftc.teamcode.auto.endConditions.DeployServoByDistance;
import org.firstinspires.ftc.teamcode.auto.structure.Action;
import org.firstinspires.ftc.teamcode.auto.structure.CombinedConditions;
import org.firstinspires.ftc.teamcode.auto.structure.Watcher;
import org.firstinspires.ftc.teamcode.auto.vision.OpenCV;
import org.firstinspires.ftc.teamcode.auto.vision.VisionSystem;
import org.firstinspires.ftc.teamcode.auto.vision.Vuforia;
import org.firstinspires.ftc.teamcode.math.Angle;
import org.firstinspires.ftc.teamcode.auto.structure.Command;
import org.firstinspires.ftc.teamcode.auto.structure.AutoOpConfiguration;
import org.firstinspires.ftc.teamcode.auto.structure.IEndCondition;
import org.firstinspires.ftc.teamcode.auto.endConditions.Timeout;
import org.firstinspires.ftc.teamcode.robot.BatMobile.SideArm;
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

    // TODO: Put in BatMobile and figure out inheritance stuff
    private SideArm sideArm;

    private static AngleUnit angleUnit = AngleUnit.DEGREES;
    private boolean isStopped = false;

    private VisionSystem.SkystonePosition skystonePosition = VisionSystem.SkystonePosition.CENTER;

    public AutoRunner(String name, LinearOpMode opMode) {
        opMode.msStuckDetectStop = 10000;
        this.opMode = opMode;

        config = AutoOpConfiguration.newInstance(name + ".json");

        angleUnit = config.properties.getAngleUnit("angle unit", AngleUnit.DEGREES);
        robot = Robot.newInstance(opMode);
        sideArm = new SideArm(robot.hardwareMap);
        robot.initializeImu(angleUnit);

        for (Command command : config.initCommands) {
            logAndTelemetry(TAG, "Init Command: " + command.name);
            execute(command);
        }

        opMode.telemetry.addData("Skystone Position", skystonePosition.name());
        logAndTelemetry(TAG, "Ready to run");
    }


    public void run() {
        for (Command command : config.commands) {
            logAndTelemetry(TAG, "Command: " + command.name);
            if (isStopped || !opMode.opModeIsActive()) {
                logAndTelemetry(TAG, "Early stop");
                return;
            }
            execute(command);
        }
    }


    private void execute(Command command) {

        if (command.name.contains("SKIP")) {
            return;
        }

        switch (command.name) {

            case "INIT VUFORIA": {
                Vuforia vuforia = new Vuforia(opMode.hardwareMap);
                vuforia.initialize();
                vuforia.startLook(VisionSystem.TargetType.RUN_FOREVER);
                break;
            }

            case "IDENTIFY SKYSTONE": {
                OpenCV openCV = new OpenCV();
                openCV.initialize();
                openCV.startLook(VisionSystem.TargetType.SKYSTONE);
                skystonePosition = openCV.identifyPosition(opMode, config.properties);
                break;
            }
            case "ALIGN SKYSTONE": {
                Angle moveAngle = command.getAngle("move angle", 0, angleUnit);
                Angle targetAngle = command.getAngle("target angle", 0, angleUnit);
                // we are lined up with the center stone so left stone is -1, right stone is 1
                double timeoutMs = command.getDouble("timeout", 10 * 1000.0);
                double powerFactor = command.getDouble("power", 0.5);
                int accelerateClicks = command.getInt("ramp up", 0);
                int decelerateClicks = command.getInt("ramp down", 0);
                int distance = 0;

                switch (skystonePosition) {
                    case LEFT:
                        distance = config.properties.getInt("align left", 0);
                        break;
                    case CENTER:
                        distance = config.properties.getInt("align center", 0);
                        break;
                    case RIGHT:
                        distance = config.properties.getInt("align right", 0);
                        break;
                }
                Action relativeMove = new RelativeMove(distance, moveAngle, targetAngle, powerFactor, accelerateClicks, decelerateClicks);
                IEndCondition timeoutCondition = new Timeout(timeoutMs);

                runTask(relativeMove, timeoutCondition);
                break;
            }

            case "MOVE AND TOGGLE PIVOT": {
                double timeoutMs = command.getDouble("timeout", 10 * 1000.0);
                int deployClicks = command.getInt("deploy clicks", 0);

                Action relativeMove = new RelativeMove(command);
                Watcher deployServo = new DeployServoByDistance(sideArm.pivot, robot.driveTrain.rf, deployClicks);
                IEndCondition timeoutCondition = new Timeout(timeoutMs);

                CombinedConditions conditions = new CombinedConditions();
                conditions.add(deployServo).add(timeoutCondition);

                runTask(relativeMove, conditions);
                break;
            }

            case "CLAW": {
                String state = command.getString("state", "REST");
                sideArm.claw.set(ToggleServo.stringToState(state));
                break;
            }

            case "PIVOT": {
                String state = command.getString("state", "REST");
                sideArm.pivot.set(ToggleServo.stringToState(state));
                break;
            }

            case "MOVE": {
                double timeoutMs = command.getDouble("timeout", 10 * 1000.0);
                Action relativeMove = new RelativeMove(command);
                IEndCondition timeoutCondition = new Timeout(timeoutMs);
                runTask(relativeMove, timeoutCondition);
                break;
            }

            case "MOVE VUFORIA": {
                double timeoutMs = command.getDouble("timeout", 10 * 1000.0);
                Action relativeMoveVuforia = new RelativeMoveWithVuforia(command);
                IEndCondition timeoutCondition = new Timeout(timeoutMs);
                runTask(relativeMoveVuforia, timeoutCondition);
                break;
            }

            case "TURN": {
                Angle angle = command.getAngle("angle", 0, angleUnit);
                double power = command.getDouble("power", 1.0);
                double timeoutMs = command.getDouble("timeout", 5 * 1000.0);
                Action relativeTurn = new AbsoluteTurn(angle, power);
                IEndCondition timeoutCondition = new Timeout(timeoutMs);

                runTask(relativeTurn, timeoutCondition);
                break;
            }

            case "SLEEP": {
                long timeMs = (long) command.getDouble("time", 1000);
                sleep(timeMs);
                break;
            }

            case "BREAKPOINT": {
                Button button = new Button();
                while (!button.is(Button.State.HELD) && opMode.opModeIsActive()) {
                    button.update(opMode.gamepad1.a);
                    sleep(20);
                }
                break;
            }

            case "STOP": {
                isStopped = true;
                break;
            }

            default: {
                logAndTelemetry("Command not exist");
                break;
            }
        }
    }


    private void runTask(Action action, IEndCondition endCondition) {
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
