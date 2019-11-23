package org.firstinspires.ftc.teamcode.teleop.utility;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.auto.structure.Command;
import org.firstinspires.ftc.teamcode.auto.structure.AutoOpConfiguration;
import org.json.JSONException;
import org.json.JSONObject;

import java.io.IOException;

public class Configuration extends Command {

    public Configuration(String name) {
        super(name, new JSONObject());

        try {
            String fileContents = AutoOpConfiguration.readFile(name + ".json");
            json = new JSONObject(fileContents);
        } catch (JSONException | IOException ex) {
            RobotLog.e("TeleOpConfiguration", ex.getMessage());
        }
    }

}
