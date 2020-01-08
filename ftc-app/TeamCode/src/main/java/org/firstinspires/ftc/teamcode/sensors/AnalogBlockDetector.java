/* Michael: Dual IR-sensor block detection system (ANALOG)
 */

package org.firstinspires.ftc.teamcode.sensors;
import org.firstinspires.ftc.teamcode.sensors.SharpDistanceSensor;

public class AnalogBlockDetector {
    private SharpDistanceSensor sensorL;
    private SharpDistanceSensor sensorR;

    private double detectMin = 19;
    private double detectMax = 23;

    public AnalogBlockDetector(SharpDistanceSensor l, SharpDistanceSensor r, double dist, double margin) {
        this.sensorL = l;
        this.sensorR = r;
        this.detectMin = dist-margin;
        this.detectMax = dist+margin;
    }

    public String getState() {
        return  String.format("Left: %f\nRight: %f", this.sensorL.getDistance(), this.sensorR.getDistance());
    }

    // TODO: Add something PID-compatible

    public boolean blockDetected() {
        return this.sensorL.getDistance() < detectMax &&
               this.sensorL.getDistance() > detectMin &&
               this.sensorR.getDistance() > detectMin &&
               this.sensorR.getDistance() > detectMin;
    }
}
