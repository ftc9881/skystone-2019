package org.firstinspires.ftc.teamcode.teleop.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.auto.AutoRunner;
import org.firstinspires.ftc.teamcode.math.PIDController;
import org.firstinspires.ftc.teamcode.math.Pose;
import org.firstinspires.ftc.teamcode.sensors.SharpPair;
import org.firstinspires.ftc.teamcode.sensors.SharpDistanceSensor;
import org.firstinspires.ftc.teamcode.teleop.utility.Button;
import org.firstinspires.ftc.teamcode.teleop.utility.Configuration;
import org.firstinspires.ftc.teamcode.teleop.utility.TeleOpBase;

import java.io.*;
import java.util.PriorityQueue;

@TeleOp
public class FileWriteTest extends TeleOpBase {

    Button writeButton = new Button();
    PrintWriter printWriter;

    @Override
    protected void initialize() {
        try {
            printWriter = new PrintWriter(new BufferedWriter(new FileWriter(Configuration.PATH + "Test.csv")));
        }
        catch  (IOException e) {
            AutoRunner.log("FileWriteTest", e.getMessage());
        }
    }

    @Override
    protected void update() {
        writeButton.update(gamepad1.a);

        double value = 1.0;

        if (writeButton.is(Button.State.DOWN)) {
            printWriter.print(value + ",");
        }
    }

    @Override
    protected void onStop() {
        super.onStop();
        printWriter.close();
    }

}
