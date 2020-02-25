package org.firstinspires.ftc.teamcode.teleop.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.MovingStatistics;

import org.firstinspires.ftc.teamcode.auto.AutoRunner;
import org.firstinspires.ftc.teamcode.auto.vision.VisionSystem;
import org.firstinspires.ftc.teamcode.auto.vision.Vuforia;
import org.firstinspires.ftc.teamcode.hardware.motor.OdometryWheel;
import org.firstinspires.ftc.teamcode.math.GeneralMath;
import org.firstinspires.ftc.teamcode.math.PIDController;
import org.firstinspires.ftc.teamcode.math.Pose;
import org.firstinspires.ftc.teamcode.math.SimpleValuePredictor;
import org.firstinspires.ftc.teamcode.robot.BatMobile.BatMobile;
import org.firstinspires.ftc.teamcode.teleop.opmodes.drive.BaseDrive;
import org.firstinspires.ftc.teamcode.teleop.utility.Command;

@TeleOp(group="Test")
//@Disabled
public class MoveCombinedSmoothDebugTest extends BaseDrive {


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
    private Pose lastGuessPose = new Pose();

    private SimpleValuePredictor yPredictor;
    private MovingStatistics movingAverageY;

    private OdometryWheel odometryWheel;
    private double initialInchesGuess;
    private double odometryInitialInches;
    private double odometryInchesAtSetpoint = 0;
    private double vuforiaYAtSetpoint = 0;

    private double xRejectThreshold;
    private MovingStatistics movingStatisticsX;
    private double maxR;

    private int inchesUntilCorrectX;
    private int movingStatsSizeX;
    private double stdDevsRangeX;

    private Pose drivePose = new Pose(0, 1, 0);


    @Override
    protected void initialize() {
        super.initialize();
        robot.initializeIMU();

        Command command = config;

        odometryWheel = BatMobile.getInstance().odometryY;
        odometryWheel.resetEncoder();
        odometryInitialInches = odometryWheel.getInches();
//        frontSensor = BatMobile.getInstance().frontSensor;

        vuforia = Vuforia.createInstance(VisionSystem.CameraType.FRONT_WEBCAM);
        VisionSystem.TargetType target = VisionSystem.TargetType.stringToType(command.getString("vuforia target", "PERIMETER"));
        vuforia.startLook(target);

        inchesUntilCorrectX = command.getInt("inches until correct x", 40);
        initialInchesGuess = command.getInt("initial inches guess", 80);

        closeX = command.getDouble("x close threshold", 2);
        conditionalX = GeneralMath.Conditional.CLOSE;

        closeY = command.getDouble("y close threshold", 1);
        String conditionalString = command.getString("y stop when", "close");
        conditionalY = GeneralMath.Conditional.convertString(conditionalString);

        targetX = command.getDouble("target x", 0);
        pidX = new PIDController(command, "x", targetX);
        targetY = command.getDouble("target y", 0);
        pidY = new PIDController(command, "y", targetY);

        movingStatsSizeX = command.getInt("x moving stats size", 10);
        movingStatisticsX = new MovingStatistics(movingStatsSizeX);
        xRejectThreshold = command.getDouble("x reject threshold", 7.16);
        stdDevsRangeX = command.getDouble("x std devs", 2);

        int movingAverageSizeY = command.getInt("y moving stats size", 3);
        movingAverageY = new MovingStatistics(movingAverageSizeY);
        yPredictor = new SimpleValuePredictor(command, "y");

        maxR = command.getDouble("r max", 8);

        yPredictor.add(initialInchesGuess);
        yPredictor.add(initialInchesGuess);
    }

    @Override
    protected void update() {
        super.update();

        Pose correctedDrivePose = new Pose(drivePose);
        currentVuforiaPose = vuforia.getPose();
        newVuforiaPose = currentVuforiaPose.sameAs(lastVuforiaPose) ? new Pose() : currentVuforiaPose;
        lastVuforiaPose = currentVuforiaPose;
        lastGuessPose = bestGuessPose;

        bestGuessPose.x = getBestGuessX();
        bestGuessPose.y = getBestGuessYSmooth();
        bestGuessPose.r = robot.imu.getHeading().getRadians();

        yPredictor.add(bestGuessPose.y);

        if (bestGuessPose.y < inchesUntilCorrectX) {
            correctedDrivePose.x += pidX.getCorrectedOutput(bestGuessPose.x);
        }
        correctedDrivePose.y = GeneralMath.clipPower(-pidY.getCorrectedOutput(bestGuessPose.y));
//        correctedDrivePose.r = rPid.getCorrectedOutput(bestGuessPose.r);

//        robot.driveTrain.drive(correctedDrivePose);

        telemetry.addData("BestGuessPose", bestGuessPose.toString(",\t"));
        telemetry.addData("VuforiaPose", currentVuforiaPose.toString(",\t"));
        telemetry.addData("DrivePose", correctedDrivePose.toString(",\t"));
        telemetry.addData("==","==");
        telemetry.addData("PredictedY", yPredictor.getPredictedDouble());
        telemetry.addData("NearPrediction?", yPredictor.isNear(currentVuforiaPose.y));
        telemetry.addData("DeviationX", getDeltaRangeX());
        telemetry.addData("WithinExpected?", xIsWithinLast(getDeltaRangeX()));
        telemetry.addData("==","==");
        telemetry.addData("Odometry (clicks/vuf.in)", currentVuforiaPose.y != 0 ? odometryWheel.getClicks()/currentVuforiaPose.y : "?");
        telemetry.addData("Odometry(in)", odometryWheel.getInches());
        telemetry.addData("OdometrySetpoint", odometryInchesAtSetpoint);
        telemetry.addData("OdometryDirection", odometryWheel.getDirection());
        telemetry.addData("InitialInchesGuess", initialInchesGuess);

        telemetry.update();

        AutoRunner.log("BestGuessPose", bestGuessPose);
        AutoRunner.log("VuforiaPose", currentVuforiaPose);
        AutoRunner.log("DrivePose", correctedDrivePose);
    }


    private double getBestGuessX() {
        if (xIsWithinLast(getDeltaRangeX()) && Math.abs(currentVuforiaPose.r) < 6) {
            return currentVuforiaPose.x;
        }
        return targetX;
    }

    private boolean xIsWithinLast(double deltaRange) {
        return (Math.abs(lastGuessPose.x - currentVuforiaPose.x) < deltaRange && !currentVuforiaPose.isAllZero());
    }

    private double getDeltaRangeX() {
        boolean xIsWithinTarget = (Math.abs(targetX - currentVuforiaPose.x) < xRejectThreshold && !currentVuforiaPose.isAllZero());
        if (!newVuforiaPose.isAllZero() && xIsWithinTarget) {
            movingStatisticsX.add(currentVuforiaPose.x);
        }
        if (movingStatisticsX.getCount() >= movingStatsSizeX) {
            return movingStatisticsX.getStandardDeviation() * stdDevsRangeX;
        }
        return xRejectThreshold;
    }

    private double getBestGuessY() {
        if (!newVuforiaPose.isAllZero() && (yPredictor.isNear(newVuforiaPose.y) || cantMakePredictionYet())) {
            odometryInchesAtSetpoint = odometryWheel.getInches();
            vuforiaYAtSetpoint = newVuforiaPose.y;
            return newVuforiaPose.y;
        } else if (vuforiaYAtSetpoint != 0){
            return vuforiaYAtSetpoint - (odometryWheel.getInches() - odometryInchesAtSetpoint);
        } else {
            return initialInchesGuess - (odometryWheel.getInches() - odometryInitialInches);
        }
    }

    private double getBestGuessYSmooth() {
        movingAverageY.add(getBestGuessY());
        return movingAverageY.getMean();
    }

    private boolean cantMakePredictionYet() {
        return !yPredictor.canMakePrediction();
    }

    @Override
    protected void onStop() {
        super.onStop();
        AutoRunner.log("END ODOM INCHES", odometryWheel.getInches());
        vuforia.stopLook();
    }

}
