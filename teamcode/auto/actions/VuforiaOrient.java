package org.firstinspires.ftc.teamcode.auto.actions;

import org.firstinspires.ftc.teamcode.auto.structure.Action;
import org.firstinspires.ftc.teamcode.auto.vision.Vuforia;

public class VuforiaOrient extends Action {
    Vuforia vuforia;

    public VuforiaOrient (Vuforia vuforia) {
        this.vuforia = vuforia;
    }

    @Override
    protected void onRun() {
        vuforia.startLook(Vuforia.TargetType.PERIMETER);
    }

    @Override
    protected boolean runIsComplete() {
        return vuforia.found();
    }

    @Override
    protected void insideRun() {

    }

    @Override
    protected void onEndRun() {
        vuforia.stopLook();
    }
}
