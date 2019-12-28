package org.firstinspires.ftc.teamcode.auto.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.AutoRunner;
import org.firstinspires.ftc.teamcode.robot.Robot;

@Autonomous(name = "TestAuto", group = "TeamCode")
//@Disabled
public class TestAutoOp extends LinearOpMode {

    @Override
    public void runOpMode() {
        AutoRunner auto = new AutoRunner("Test", this);
        waitForStart();
        try {
            auto.run();
        } catch (Exception ex) {
            Robot robot = Robot.getInstance();
            robot.driveTrain.stop();
        }
    }

}


