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

    private Vuforia vuforia;
    private OdometryWheel odometryY;
    private IDistanceSensor sensor;

    public MoveDebug(Command command) {
        super(command);
        tag = "RelativeMoveDebug";

        odometryY = BatMobile.getInstance().odometryY;
        sensor = BatMobile.getInstance().frontSensor;

        vuforia = Vuforia.getInstance();
        if (!vuforia.isLooking()) {
            VisionSystem.TargetType target = VisionSystem.TargetType.stringToType(command.getString("vuforia target", "PERIMETER"));
            vuforia.startLook(target);
        }
    }

    @Override
    protected void onRun() {
        super.onRun();
        robot.driveTrain.drive(drivePose);
//        AutoRunner.log("EncoderTestData", String.format("\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s", "SensorY", "VuforiaY", "OdomY", "AverageDriveClicks", "LF", "RF", "RB", "LB"));
    }

    @Override
    protected void insideRun() {
//        List<Integer> clicksArray = getClicksArray();
//        currentClicks = (int) robot.driveTrain.getAverageClicks();
//        AutoRunner.log("EncoderTestData", String.format("\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s", sensor.getDistance(), vuforia.getPose().y, odometryY.getClicks() , currentClicks, clicksArray.get(0), clicksArray.get(1), clicksArray.get(2), clicksArray.get(3)));
        AutoRunner.log("VuforiaPoseDataOdometryY", vuforia.getPose().toString("\t") + "\t" + odometryY.getInches());
    }
}
