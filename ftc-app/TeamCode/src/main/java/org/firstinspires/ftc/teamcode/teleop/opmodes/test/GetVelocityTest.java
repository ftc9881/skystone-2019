package org.firstinspires.ftc.teamcode.teleop.opmodes.test;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.auto.AutoRunner;
import org.firstinspires.ftc.teamcode.math.Pose;
import org.firstinspires.ftc.teamcode.robot.BatMobile.BatMobile;
import org.firstinspires.ftc.teamcode.teleop.opmodes.drive.BaseDrive;
import org.firstinspires.ftc.teamcode.teleop.utility.TeleOpBase;

@TeleOp(group="Test")
//@Disabled
public class GetVelocityTest extends SnapshotTest {

    private BatMobile batMobile;

    private long previousTime;
    private long currentTime;

    private double previousOdometry;
    private double currentOdometry;

    private double previousEncoder;
    private double currentEncoder;

    private double previousLift;
    private double currentLift;

    private double battery;
    private double deltaTime;

    private Pose drivePose = new Pose();

    @Override
    protected void initialize() {
        batMobile = BatMobile.createInstance();
//        robot.driveTrain.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        batMobile.elevator.left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        batMobile.elevator.right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    protected void update() {
        super.update();
        updateDrive();
        updateElevator();

        previousOdometry = currentOdometry;
        previousLift = currentLift;
        previousTime = currentTime;
        previousEncoder = currentEncoder;
        currentTime = System.currentTimeMillis();
        currentOdometry = batMobile.odometryY.getInches();
        currentLift = batMobile.elevator.right.getCurrentPosition();
        currentEncoder = robot.driveTrain.rb.getCurrentPosition();
        battery = robot.getBatteryVoltage();
        deltaTime = (currentTime - previousTime) / 1000.0;

        telemetry.addData("LF Velocity", robot.driveTrain.lf.getVelocity() );
        telemetry.addData("RF Velocity", robot.driveTrain.rf.getVelocity() );
        telemetry.addData("LB Velocity", robot.driveTrain.lb.getVelocity() );
        telemetry.addData("RB Velocity", robot.driveTrain.rb.getVelocity() );
        telemetry.addData("==", "==");
        telemetry.addData("Odometry", Math.abs(currentOdometry - previousOdometry) / deltaTime );
        telemetry.addData("RBDTEncoder", Math.abs(currentEncoder - previousEncoder) / deltaTime );
        telemetry.addData("Lift", (currentLift - previousLift) / deltaTime );
        telemetry.addData("Battery", battery);
        telemetry.update();
    }

    @Override
    void onClick() {
        AutoRunner.log("Odometry", "\t" + battery + "\t" + Math.abs((currentOdometry - previousOdometry) / deltaTime) );
        AutoRunner.log("Lift", "\t" + battery + "\t" + (Math.abs(currentLift - previousLift) / deltaTime) );
        AutoRunner.log("RBDTEncoder", "\t" + battery + "\t" + (Math.abs(currentEncoder - previousEncoder) / deltaTime) + "\t" + robot.driveTrain.rb.getVelocity());
    }

    private void updateDrive() {
        drivePose.x = Math.pow(gamepad1.left_stick_x, 3);
        drivePose.y = -Math.pow(gamepad1.left_stick_y, 3);
        drivePose.r = Math.pow(gamepad1.right_stick_x, 3);
        robot.driveTrain.drive(drivePose, 1);
    }

    private void updateElevator() {
        batMobile.elevator.setPowerLE(-gamepad2.left_stick_y, gamepad2.right_stick_x);
    }


}