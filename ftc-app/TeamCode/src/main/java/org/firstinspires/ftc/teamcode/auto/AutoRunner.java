package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.auto.actions.RelativeMove;
import org.firstinspires.ftc.teamcode.auto.actions.AbsoluteTurn;
import org.firstinspires.ftc.teamcode.auto.endConditions.DeployServoByDistance;
import org.firstinspires.ftc.teamcode.auto.structure.Action;
import org.firstinspires.ftc.teamcode.auto.structure.CombinedConditions;
import org.firstinspires.ftc.teamcode.auto.structure.Watcher;
import org.firstinspires.ftc.teamcode.auto.vision.OpenCV;
import org.firstinspires.ftc.teamcode.auto.vision.VisionSystem;
import org.firstinspires.ftc.teamcode.math.Angle;
import org.firstinspires.ftc.teamcode.auto.structure.Command;
import org.firstinspires.ftc.teamcode.auto.structure.AutoOpConfiguration;
import org.firstinspires.ftc.teamcode.auto.structure.IEndCondition;
import org.firstinspires.ftc.teamcode.auto.endConditions.Timeout;
import org.firstinspires.ftc.teamcode.math.GeneralMath;
import org.firstinspires.ftc.teamcode.robot.BatMobile.SideArm;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.opencv.core.Rect;

import java.util.ArrayList;
import java.util.List;

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

            case "IDENTIFY SKYSTONE": {
                OpenCV openCV = new OpenCV();
                openCV.initialize();
                openCV.startLook(VisionSystem.TargetType.SKYSTONE);
//                skystonePosition = openCV.identifyPosition(opMode, config.properties);

                openCV.detector.cropRect.x = config.properties.getInt("crop x", 0);
                openCV.detector.cropRect.y = config.properties.getInt("crop y", 0);
                openCV.detector.cropRect.width = config.properties.getInt("crop w", 0);
                openCV.detector.cropRect.height= config.properties.getInt("crop h", 0);
                openCV.detector.blobDistanceThreshold = config.properties.getInt("stone blob distance", 50);
                openCV.detector.minimumArea = config.properties.getInt("stone min area", 10);
                int centerX = 0;

                while (!opMode.isStopRequested() && centerX == 0) {
                    Rect rect = openCV.detector.foundRectangle();
                    centerX = rect.x + rect.width / 2;
                    log("Rect", rect.toString());
                    log("CenterX", centerX);
                }
                openCV.writeCurrentImage();

                AutoRunner.log("Mean", centerX);
                int skystoneLeftBound = config.properties.getInt("skystone left", 0);
                int skystoneRightBound = config.properties.getInt("skystone right", 0);
                if (centerX < skystoneLeftBound) {
                    skystonePosition = VisionSystem.SkystonePosition.LEFT;
                } else if (centerX > skystoneRightBound) {
                    skystonePosition = VisionSystem.SkystonePosition.RIGHT;
                } else {
                    skystonePosition = VisionSystem.SkystonePosition.CENTER;
                }
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

            case "TOGGLE CLAW": {
                sideArm.claw.toggle();
                break;
            }

            case "TOGGLE PIVOT": {
                sideArm.pivot.toggle();
                break;
            }

            case "DEPLOY FOUNDATION": {
                sideArm.pivotToFoundation();
                break;
            }

            case "MOVE": {
                double timeoutMs = command.getDouble("timeout", 10 * 1000.0);

                Action relativeMove = new RelativeMove(command);
                IEndCondition timeoutCondition = new Timeout(timeoutMs);

                runTask(relativeMove, timeoutCondition);
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
                while (!opMode.gamepad1.a && opMode.opModeIsActive()) {
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
