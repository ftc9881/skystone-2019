package org.firstinspires.ftc.teamcode.auto.structure;

import android.os.Environment;

import org.firstinspires.ftc.teamcode.teleop.utility.Command;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;

/**
 * Instantiating Configuration will read a file and return a list of commands and properties.
 *
 * @author Trinity Chung
 */
public class AutoOpConfiguration {

    // Singleton pattern; constructor is private and enable only one instance at a time
    private static AutoOpConfiguration instance;

    public static AutoOpConfiguration newInstance(String fileName) {
        instance = new AutoOpConfiguration(fileName);
        return instance;
    }

    public static AutoOpConfiguration getInstance() {
        return instance;
    }


    private static final String PATH = Environment.getExternalStorageDirectory().getAbsolutePath() + "/Robot/";
    public ArrayList<Command> commands;
    public ArrayList<Command> initCommands;
    public Command properties;

    private AutoOpConfiguration(String fileName) {

        try {
            String fileContents = readFile(fileName);
            JSONObject config = new JSONObject(fileContents);

            commands = getCommandList(config, "commands");
            initCommands = getCommandList(config, "init");
            properties = new Command("PROPERTIES", config.getJSONObject("properties"));

        } catch (Exception ex) {
            throw new RuntimeException("Configuration", ex);
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
        return sb.toString();
    }

    private ArrayList<Command> getCommandList(JSONObject config, String name) throws JSONException {
        ArrayList<Command> commandList = new ArrayList<>();
        JSONArray allCommandsJson = config.getJSONArray(name);
        for (int i = 0; i < allCommandsJson.length(); i++) {
            commandList.add(new Command(allCommandsJson.getJSONObject(i)));
        }
        return commandList;
    }
}
