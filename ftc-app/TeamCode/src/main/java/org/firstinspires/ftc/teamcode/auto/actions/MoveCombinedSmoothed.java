package org.firstinspires.ftc.teamcode.auto.actions;

import com.qualcomm.robotcore.util.MovingStatistics;

import org.firstinspires.ftc.teamcode.auto.AutoRunner;
import org.firstinspires.ftc.teamcode.auto.vision.VisionSystem;
import org.firstinspires.ftc.teamcode.auto.vision.Vuforia;
import org.firstinspires.ftc.teamcode.hardware.motor.OdometryWheel;
import org.firstinspires.ftc.teamcode.math.GeneralMath;
import org.firstinspires.ftc.teamcode.math.GeneralMath.Conditional;
import org.firstinspires.ftc.teamcode.math.PIDController;
import org.firstinspires.ftc.teamcode.math.Pose;
import org.firstinspires.ftc.teamcode.math.SimpleValuePredictor;
import org.firstinspires.ftc.teamcode.robot.BatMobile.BatMobile;
import org.firstinspires.ftc.teamcode.teleop.utility.Command;

public class MoveCombinedSmoothed extends MoveWithClicks {

    private Vuforia vuforia;

    private PIDController pidX;
    private Conditional conditionalX;
    private double targetX;
    private double closeX;

    private PIDController pidY;
    private Conditional conditionalY;
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

    private int inchesUntilCorrectX;
    private int movingStatsSizeX;

    public MoveCombinedSmoothed(Command command) {
        super(command);
        tag = "MoveCombinedSmooth";

        odometryWheel = BatMobile.getInstance().odometryY;
        odometryInitialInches = odometryWheel.getInches();

        vuforia = Vuforia.getInstance();
        if (!vuforia.isLooking()) {
            VisionSystem.TargetType target = VisionSystem.TargetType.stringToType(command.getString("vuforia target", "PERIMETER"));
            vuforia.startLook(target);
        }

        inchesUntilCorrectX = command.getInt("inches until correct x", 40);
        initialInchesGuess = command.getInt("initial inches guess", 80);

        closeX = command.getDouble("x close threshold", 2);
        conditionalX = Conditional.CLOSE;

        closeY = command.getDouble("y close threshold", 1);
        String conditionalString = command.getString("y stop when", "close");
        conditionalY = Conditional.convertString(conditionalString);

        targetX = command.getDouble("target x", 0);
        pidX = new PIDController(command, "x", targetX);
        targetY = command.getDouble("target y", 0);
        pidY = new PIDController(command, "y", targetY);

        movingStatsSizeX = command.getInt("x moving stats size", 10);
        movingStatisticsX = new MovingStatistics(movingStatsSizeX);
        xRejectThreshold = command.getDouble("x reject threshold", 7.16);

        int movingAverageSizeY = command.getInt("y moving stats size", 3);
        movingAverageY = new MovingStatistics(movingAverageSizeY);
        yPredictor = new SimpleValuePredictor(command, "y");
    }

    public MoveCombinedSmoothed(Command command, VisionSystem.SkystonePosition skystonePosition) {
        this(command);
        targetY = command.getDouble("target y " + skystonePosition.key, targetY);
        pidY = new PIDController(command, "y", targetY);
    }

    @Override
    protected void onRun() {
        super.onRun();
        AutoRunner.log("TargetY", targetY);
    }

    @Override
    protected boolean runIsComplete() {
        return conditionalY.evaluate(bestGuessPose.y, targetY, closeY) && conditionalX.evaluate(bestGuessPose.x, targetX, closeX);
    }

    @Override
    protected void insideRun() {
        Pose correctedDrivePose = new Pose(drivePose);
        currentVuforiaPose = vuforia.getPose();
        newVuforiaPose = currentVuforiaPose.sameAs(lastVuforiaPose) ? new Pose() : currentVuforiaPose;
        lastVuforiaPose = currentVuforiaPose;

        bestGuessPose.x = getBestGuessX();
        bestGuessPose.y = getBestGuessYSmooth();
        bestGuessPose.r = robot.imu.getHeading().getRadians();

        yPredictor.add(bestGuessPose.y);

        if (bestGuessPose.y < inchesUntilCorrectX) {
            correctedDrivePose.x += pidX.getCorrectedOutput(bestGuessPose.x);
        }
        correctedDrivePose.y = GeneralMath.clipPower(-pidY.getCorrectedOutput(bestGuessPose.y), basePower);
        correctedDrivePose.r = anglePidController.getCorrectedOutput(bestGuessPose.r);

        robot.driveTrain.drive(correctedDrivePose, powerFactor);

        AutoRunner.log("BestGuessPose", bestGuessPose);
        AutoRunner.log("VuforiaPose", currentVuforiaPose);
        AutoRunner.log("DrivePose", correctedDrivePose);
    }

    private double getBestGuessX() {
        if (xIsWithin(getDeltaRangeX())) {
            return currentVuforiaPose.x;
        }
        return targetX;
    }

    private boolean xIsWithin(double deltaRange) {
        return (Math.abs(targetX - currentVuforiaPose.x) < deltaRange && !currentVuforiaPose.isAllZero());
    }

    private double getDeltaRangeX() {
        if (!newVuforiaPose.isAllZero() && xIsWithin(xRejectThreshold)) {
            movingStatisticsX.add(currentVuforiaPose.x);
        }
        if (movingStatisticsX.getCount() >= movingStatsSizeX) {
            return movingStatisticsX.getStandardDeviation() * 2;
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
    protected void onEndRun() {
        robot.driveTrain.stop();
    }

}
