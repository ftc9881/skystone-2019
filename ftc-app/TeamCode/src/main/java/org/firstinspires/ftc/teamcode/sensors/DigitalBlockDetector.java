// simple wrapper class for 2 digital IR sensors

package org.firstinspires.ftc.teamcode.sensors;

import com.qualcomm.robotcore.hardware.DigitalChannel;

public class DigitalBlockDetector {
    private DigitalChannel sensorR;
    private DigitalChannel sensorL;

    public DigitalBlockDetector(DigitalChannel r, DigitalChannel l) {
        this.sensorR = r;
        this.sensorL = l;
    }

    public boolean blockDetected() {
        // device output is flipped, so flip it again software side
        return !sensorL.getState() && !sensorR.getState();
    }

}
