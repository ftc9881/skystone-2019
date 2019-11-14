package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.math.Pose;

import Jama.Matrix;

public class OdometrySystem implements IRobotSystem {

    private DcMotor rightEncoder;
    private DcMotor leftEncoder;
    private DcMotor centerEncoder;

    private final double L_OFFSET_X = 3;
    private final double R_OFFSET_X = 3;
    private final double C_OFFSET_X = 3;

    private final double L_OFFSET_Y = 7;
    private final double R_OFFSET_Y = 7;
    private final double C_OFFSET_Y = 0;

//    private final double L_OFFSET_DIST = Math.sqrt(Math.pow(L_OFFSET_X, 2) + Math.pow(L_OFFSET_Y, 2));
//    private final double R_OFFSET_DIST = Math.sqrt(Math.pow(R_OFFSET_X, 2) + Math.pow(R_OFFSET_Y, 2));
//    private final double C_OFFSET_DIST = Math.sqrt(Math.pow(C_OFFSET_X, 2) + Math.pow(C_OFFSET_Y, 2));

    private final double TICKS_PER_RADIAN = 1400/(2.0*Math.PI);
    private final double RADIUS = 1.0;

    private double currLEncoder = 0.0, currREncoder = 0.0, currCEncoder = 0.0;
    private double prevLEncoder = 0.0, prevREncoder = 0.0, prevCEncoder = 0.0;
    // TODO given field orientation starting dir would be pi?
    private double currIMU = 0.0, prevIMU = 0.0;
    //since using abs. position add code for start depending on red/blue
    private double x = 0.0, y = 0.0, r = 0.0;

    private Pose currPose;


    public OdometrySystem(HardwareMap hardwareMap) {
//        rightEncoder = hardwareMap.dcMotor.get("right_odometry");
//        leftEncoder = hardwareMap.dcMotor.get("left_odometry");
//        centerEncoder = hardwareMap.dcMotor.get("center_odometry");

        this.currPose = new Pose(0, 0, 0);
}

    public void updatePose() {
        prevLEncoder = currLEncoder;
        prevREncoder = currREncoder;
        prevCEncoder = currCEncoder;
        prevIMU = currIMU;

        currLEncoder = getLeftPosition();
        currREncoder = getRightPosition();
        currCEncoder = getCenterPosition();
        //currIMU = robot.imu ... + DIR_OFFSET

        double angVelL = (1/TICKS_PER_RADIAN) * (currLEncoder - prevLEncoder);
        double angVelR = (1/TICKS_PER_RADIAN) * (currREncoder - prevREncoder);
        double angVelC = (1/TICKS_PER_RADIAN) * (currCEncoder - prevCEncoder);

        double[][] arrayA = {
                {1, 0, -L_OFFSET_Y},
                {1, 0, R_OFFSET_Y},
                {0, 1, C_OFFSET_X}
        };

        double [][] arrayB = {
                {angVelL}, {angVelR}, {angVelC}
        };

        Matrix matrixA = new Matrix(arrayA);
        Matrix matrixB = new Matrix(arrayB);
        matrixA = matrixA.times(1.0/RADIUS);
        Matrix matrixC = matrixA.inverse().times(matrixB);

        double dx = matrixC.getArray()[0][0];
        double dy = matrixC.getArray()[1][0];
        double dr = matrixC.getArray()[2][0];

        x += dx;
        y += dy;
        r += dr;

        currPose = new Pose(x , y, r);
    }

    public double getRightPosition() {
        return rightEncoder.getCurrentPosition();
    }
    public double getLeftPosition() {
        return leftEncoder.getCurrentPosition();
    }
    public double getCenterPosition() {
        return centerEncoder.getCurrentPosition();
    }

    public Pose getPose() {
        return currPose;
    }

}
