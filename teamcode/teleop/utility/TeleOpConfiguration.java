package org.firstinspires.ftc.teamcode.teleop.utility;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.auto.structure.Command;
import org.firstinspires.ftc.teamcode.auto.structure.Configuration;
import org.json.JSONException;
import org.json.JSONObject;

import java.io.IOException;

public class TeleOpConfiguration extends Command {

    public TeleOpConfiguration(String name) {
        super(name, new JSONObject());

        try {
            String fileContents = Configuration.readFile(name + ".json");
            json = new JSONObject(fileContents);
        } catch (JSONException | IOException ex) {
            RobotLog.e("TeleOpConfiguration", ex.getMessage());
        }
    }

}
