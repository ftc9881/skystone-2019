package org.firstinspires.ftc.teamcode.teleop.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.auto.AutoRunner;
import org.firstinspires.ftc.teamcode.auto.vision.VisionSystem;
import org.firstinspires.ftc.teamcode.auto.vision.Vuforia;
import org.firstinspires.ftc.teamcode.hardware.motor.OdometryWheel;
import org.firstinspires.ftc.teamcode.math.GeneralMath;
import org.firstinspires.ftc.teamcode.math.PIDController;
import org.firstinspires.ftc.teamcode.math.Pose;
import org.firstinspires.ftc.teamcode.robot.BatMobile.BatMobile;
import org.firstinspires.ftc.teamcode.teleop.opmodes.drive.BaseDrive;
import org.firstinspires.ftc.teamcode.teleop.utility.Command;

@TeleOp(group="Test")
//@Disabled
public class MoveCombinedDebugTest extends BaseDrive {

    private Vuforia vuforia;

    private PIDController pidX;
    private GeneralMath.Conditional conditionalX;
    private double targetX;
    private double closeX;

    private PIDController pidY;
    private GeneralMath.Conditional conditionalY;
    private double targetY;
    private double closeY;

    private Pose currentVuforiaPose = new Pose();
    private Pose newVuforiaPose = new Pose();
    private Pose lastVuforiaPose = new Pose();
    private Pose bestGuessPose = new Pose();

    private OdometryWheel odometryWheel;
    private double initialInchesGuess;
    private double odometryInchesAtSetpoint = 0;
    private double vuforiaYAtSetpoint = 0;
    private int inchesUntilCorrectX;

    private Pose drivePose = new Pose(0, 1, 0);


    @Override
    protected void initialize() {
        super.initialize();
        robot.initializeIMU();

        Command command = config;

        odometryWheel = BatMobile.getInstance().odometryY;
        odometryWheel.resetEncoder();
//        frontSensor = BatMobile.getInstance().frontSensor;

        vuforia = Vuforia.createInstance(VisionSystem.CameraType.FRONT_WEBCAM);
        VisionSystem.TargetType target = VisionSystem.TargetType.stringToType(command.getString("vuforia target", "PERIMETER"));
        vuforia.startLook(target);

        inchesUntilCorrectX = command.getInt("inches until correct x", 40);
        initialInchesGuess = command.getInt("initial inches guess", 80);

        closeX = command.getDouble("close threshold x", 2);
        conditionalX = GeneralMath.Conditional.CLOSE;

        closeY = command.getDouble("close threshold y", 1);
        String conditionalString = command.getString("stop when y", "close");
        conditionalY = GeneralMath.Conditional.convertString(conditionalString);


        targetX = command.getDouble("target x", 0);
        pidX = new PIDController(command, "x", targetX);
        targetY = command.getDouble("target y", 0);
        pidY = new PIDController(command, "y", targetY);
    }

    @Override
    protected void update() {
        super.update();

        Pose correctedDrivePose = new Pose(drivePose);
        currentVuforiaPose = vuforia.getPose();
        newVuforiaPose = currentVuforiaPose.sameAs(lastVuforiaPose) ? new Pose() : currentVuforiaPose;
        lastVuforiaPose = currentVuforiaPose;

        bestGuessPose.r = robot.imu.getHeading().getRadians();
//        correctedDrivePose.r = anglePidController.getCorrectedOutput(bestGuessPose.r);
        bestGuessPose.y = getBestGuessY();
        correctedDrivePose.y = -pidY.getCorrectedOutput(bestGuessPose.y);
        bestGuessPose.x = getBestGuessX();
        if (bestGuessPose.y < inchesUntilCorrectX) {
            correctedDrivePose.x += pidX.getCorrectedOutput(bestGuessPose.x);
        }

//        robot.driveTrain.drive(correctedDrivePose);

        telemetry.addData("BestGuessPose", bestGuessPose);
        telemetry.addData("VuforiaPose", currentVuforiaPose);
        telemetry.addData("DrivePose", correctedDrivePose);
        telemetry.addData("==","==");
        telemetry.addData("Odometry (clicks/vuf.in)", currentVuforiaPose.y != 0 ? odometryWheel.getPosition()/currentVuforiaPose.y : "?");
        telemetry.addData("Odometry(in)", odometryWheel.getInches());
        telemetry.addData("OdometrySetpoint", odometryInchesAtSetpoint);
        telemetry.addData("OdometryDirection", odometryWheel.getDirection());
        telemetry.addData("InitialInchesGuess", initialInchesGuess);

        telemetry.update();
    }

    private double getBestGuessX() {
        if (!currentVuforiaPose.isAllZero()) {
            return currentVuforiaPose.x;
        }
        return targetX;
    }

    private double getBestGuessY() {
        if (!newVuforiaPose.isAllZero()) {
            odometryInchesAtSetpoint = odometryWheel.getInches();
            vuforiaYAtSetpoint = newVuforiaPose.y;
            return newVuforiaPose.y;
        } else if (vuforiaYAtSetpoint != 0){
            return vuforiaYAtSetpoint - (odometryWheel.getInches() - odometryInchesAtSetpoint);
        } else {
            return initialInchesGuess - odometryWheel.getInches();
        }
    }

    @Override
    protected void onStop() {
        super.onStop();
        vuforia.stopLook();
    }

}
