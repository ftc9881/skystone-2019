package org.firstinspires.ftc.teamcode.roadrunner;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.Path;
import com.acmerobotics.roadrunner.path.PathBuilder;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryGenerator;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robot.Robot;

import java.util.ArrayList;
import java.util.List;

@Autonomous(group = "roadrunner")
@Disabled
public class RoadrunnerTestAutoOp extends LinearOpMode {

    @Override
    public void runOpMode() {

        final Robot robot = Robot.newInstance(this);
        robot.initializeIMU();
        robot.driveTrain.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Path path = new PathBuilder(new Pose2d(0, 0, 0))
                .splineTo(new Pose2d(15, 15, 0))
                .lineTo(new Vector2d(30, 15))
                .build();

        DriveConstraints constraints = new DriveConstraints(20, 40, 80, 1, 2, 4);
        Trajectory trajectory = TrajectoryGenerator.INSTANCE.generateTrajectory(path, constraints);

        MecanumDrive drive = new MecanumDrive(2,2,2,2,2) {
            @Override
            public void setMotorPowers(double lf, double lb, double rb, double rf) {
                robot.driveTrain.lf.setPower(lf);
                robot.driveTrain.lb.setPower(lb);
                robot.driveTrain.rb.setPower(rb);
                robot.driveTrain.rf.setPower(rf);
            }

            @Override
            public List<Double> getWheelPositions() {
                List<Double> list = new ArrayList<>();
                list.add((double)robot.driveTrain.lf.getCurrentPosition());
                list.add((double)robot.driveTrain.lb.getCurrentPosition());
                list.add((double)robot.driveTrain.rb.getCurrentPosition());
                list.add((double)robot.driveTrain.rf.getCurrentPosition());
                return list;
            }

            @Override
            protected double getRawExternalHeading() {
                return robot.imu.getHeading().getDegrees();
            }
        };

        PIDCoefficients translationalPid = new PIDCoefficients(2, 0, 0);
        PIDCoefficients headingPid = new PIDCoefficients(0.1, 0, 0);
        HolonomicPIDVAFollower follower = new HolonomicPIDVAFollower(translationalPid, translationalPid, headingPid);

        follower.followTrajectory(trajectory);

        waitForStart();
        while (opModeIsActive() && follower.isFollowing()) {
            DriveSignal signal = follower.update(drive.getPoseEstimate());
            drive.setDriveSignal(signal);
            drive.updatePoseEstimate();
        }
        robot.driveTrain.stop();

    }

}


