package org.firstinspires.ftc.teamcode.auto.actions;

import org.firstinspires.ftc.teamcode.auto.AutoRunner;
import org.firstinspires.ftc.teamcode.auto.vision.VisionSystem;
import org.firstinspires.ftc.teamcode.auto.vision.Vuforia;
import org.firstinspires.ftc.teamcode.hardware.motor.OdometryWheel;
import org.firstinspires.ftc.teamcode.hardware.sensor.IDistanceSensor;
import org.firstinspires.ftc.teamcode.robot.BatMobile.BatMobile;
import org.firstinspires.ftc.teamcode.teleop.utility.Command;

import java.util.List;

public class MoveDebug extends MoveWithClicks {

    private OdometryWheel odometryY;
    private IDistanceSensor sensor;

    public MoveDebug(Command command) {
        super(command);
        tag = "RelativeMoveDebug";

        odometryY = BatMobile.getInstance().odometryY;
        sensor = BatMobile.getInstance().frontSensor;
    }

    @Override
    protected void onRun() {
        super.onRun();
        robot.driveTrain.drive(drivePose);
        AutoRunner.log("MoveDebug", String.format("\t%s\t%s", "Sensor", "Odometry" ));
    }

    @Override
    protected void insideRun() {
        AutoRunner.log("MoveDebug", String.format("\t%s\t%s", sensor.getDistance(), odometryY.getClicks() ));
    }
}
