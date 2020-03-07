package org.firstinspires.ftc.teamcode.teleop.opmodes.test;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.MovingStatistics;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.teleop.opmodes.drive.BaseDrive;

@TeleOp(group="Test")
//@Disabled
public class RevSensorTest extends BaseDrive {

    private Rev2mDistanceSensor leftSensor;
    private Rev2mDistanceSensor rightSensor;
    private MovingStatistics leftStats;
    private MovingStatistics rightStats;

    @Override
    protected void initialize() {
        super.initialize();
        leftStats = new MovingStatistics(10);
        rightStats = new MovingStatistics(10);
        leftSensor = hardwareMap.get(Rev2mDistanceSensor.class, "left side sensor");
        rightSensor = hardwareMap.get(Rev2mDistanceSensor.class, "right side sensor");

        robot.driveTrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    @Override
    protected void update() {
//        super.update();
        double left = leftSensor.getDistance(DistanceUnit.INCH);
        if (left < 300) {
            leftStats.add(left);
        }
        double right = rightSensor.getDistance(DistanceUnit.INCH);
        if (right < 300) {
            rightStats.add(right);
        }

        telemetry.addData("left", left);
        telemetry.addData("left averaged", leftStats.getMean());
        telemetry.addData("==", "==");
        telemetry.addData("right", right);
        telemetry.addData("right averaged", rightStats.getMean());
        telemetry.update();
    }

}
