package org.firstinspires.ftc.teamcode.teleop.utility;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.Robot;

public abstract class TeleOpBase extends LinearOpMode {

    protected Robot robot;
    protected Configuration config;
    protected InputManager inputManager;

    protected abstract void initialize();
    protected abstract void update();

    @Override
    public void runOpMode() {
        robot = Robot.newInstance(this);
        config = new Configuration("TeleOp");
        inputManager = new InputManager(this);
        initialize();

        waitForStart();
        while (opModeIsActive()) {
            inputManager.update();
            update();
        }

    }

}
