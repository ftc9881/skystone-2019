package org.firstinspires.ftc.teamcode;

import Jama.Matrix;

//experiments with a possible odometry set up
public class Odometry {
    private static final String TAG = "%%Odometry";

    private final double L_OFFSET_X = 3, L_OFFSET_Y = 7;
    private final double L_OFFSET_DIST = Math.sqrt(Math.pow(L_OFFSET_X, 2) + Math.pow(L_OFFSET_Y, 2));
    private final double R_OFFSET_X = 3, R_OFFSET_Y = 7;
    private final double R_OFFSET_DIST = Math.sqrt(Math.pow(R_OFFSET_X, 2) + Math.pow(R_OFFSET_Y, 2));
    private final double C_OFFSET_X = 3, C_OFFSET_Y = 0;
    private final double C_OFFSET_DIST = Math.sqrt(Math.pow(C_OFFSET_X, 2) + Math.pow(C_OFFSET_Y, 2));

    private final double TICKS_PER_RADIAN = 1400/(2.0*Math.PI);
    private final double RADIUS = 1.0;

    // ask trinity about time stuff
    private double currLEncoder = 0.0, currREncoder = 0.0, currCEncoder = 0.0;
    private double prevLEncoder = 0.0, prevREncoder = 0.0, prevCEncoder = 0.0;
    // TODO given field orientation starting dir would be pi?
    private double currIMU = 0.0, prevIMU = 0.0;
    //since using abs. position add code for start depending on red/blue
    private double x = 0.0, y = 0.0, r = 0.0;


    private Robot robot;
    private Pose currPose;

    public Odometry(Robot robot) {
        this.robot = robot;
        this.currPose = new Pose(0, 0, 0);
    }

    public void updatePose() {
        prevLEncoder = currLEncoder;
        prevREncoder = currREncoder;
        prevCEncoder = currCEncoder;
        prevIMU = currIMU;

        currLEncoder = -robot.leftOdometry.getCurrentPosition();
        currREncoder = -robot.rightOdometry.getCurrentPosition();
        currCEncoder = robot.centerOdometry.getCurrentPosition();
        //currIMU = robot.imu ... + DIR_OFFSET

        double angVelL = (1/TICKS_PER_RADIAN) * (currLEncoder - prevLEncoder);
        double angVelR = (1/TICKS_PER_RADIAN) * (currREncoder - prevREncoder);
        double angVelC = (1/TICKS_PER_RADIAN) * (currCEncoder - prevCEncoder);

//        robot.log(TAG, "angVelL: " + angVelL, false);
//        robot.log(TAG, "angVelR: " + angVelR, false);
//        robot.log(TAG, "angVelC: " + angVelC, false);

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

        //debug
//        for (double[] d : matrixC.getArray()) {
//            String temp = "";
//            for (double e : d) {
//                temp += e + " ";
//            }
//            robot.log(TAG, temp, false);
//        }

        double dx = matrixC.getArray()[0][0];
        double dy = matrixC.getArray()[1][0];
        double dr = matrixC.getArray()[2][0];

        x += dx;
        y += dy;
        r += dr;

        currPose = new Pose(x , y, r);
    }

    public Pose getPose() {
        return currPose;
    }
}