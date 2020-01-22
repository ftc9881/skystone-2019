package org.firstinspires.ftc.teamcode.teleop.utility;

import android.os.Environment;

import com.qualcomm.robotcore.util.RobotLog;

import org.json.JSONException;
import org.json.JSONObject;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;

import static android.os.SystemClock.sleep;

public class Configuration extends Command {

    private static final String PATH = Environment.getExternalStorageDirectory().getAbsolutePath() + "/Robot/";

    public Configuration(String name) {
        super(name, new JSONObject());

        try {
            String fileContents = readFile(name + ".json");
            json = new JSONObject(fileContents);
        } catch (JSONException | IOException ex) {
            RobotLog.e("TeleOpConfiguration", ex.getMessage());
        }
    }

    static public String readFile(String name) throws IOException {
        File file = new File(PATH, name);
        BufferedReader br = new BufferedReader(new FileReader(file));
        StringBuilder sb = new StringBuilder();
        String line;
        do {
            line = br.readLine();
            sb.append(line);
        } while (line != null);
        br.close();
        if (sb.toString().length() < 1) {
            sleep(100);
            return readFile(name);
        }
        return sb.toString();
    }
}
