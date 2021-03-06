package org.firstinspires.ftc.teamcode.auto.structure;

import org.firstinspires.ftc.teamcode.auto.AutoRunner;
import org.firstinspires.ftc.teamcode.teleop.utility.Command;
import org.firstinspires.ftc.teamcode.teleop.utility.Configuration;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.io.IOException;
import java.util.ArrayList;

import static android.os.SystemClock.sleep;

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


    public ArrayList<Command> commands;
    public ArrayList<Command> initCommands;
    public Command properties;
    private String fileName;

    private AutoOpConfiguration(String fileName) {
        this.fileName = fileName;

        tryReadFromFile();

    }

    private void tryReadFromFile() {
        try {
            readFromFile();
        } catch (IOException | JSONException ex) {
//            throw new RuntimeException("Configuration", ex);
            sleep(500);
            tryReadFromFile();
        }
    }

    private void readFromFile() throws IOException, JSONException {
        String fileContents = Configuration.readFile(fileName);
        JSONObject config = new JSONObject(fileContents);

        commands = getCommandList(config, "commands");
        initCommands = getCommandList(config, "init");
        properties = new Command("PROPERTIES", config.getJSONObject("properties"));
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
