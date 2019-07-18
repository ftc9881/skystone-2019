// import java.io.FileInputStream;
import java.io.FileReader;
import java.io.BufferedReader;
import java.io.IOException;

public class DirectionsInterpreter {
    public static void main(String[] args) throws IOException {
        // we can read filename from config if we REALLY need to
        BufferedReader infile = new BufferedReader(new FileReader("directions.txt"));
        String currentLine = infile.readLine();

        while (currentLine != null) { // while there's still something to read
            // split current line around regex " "
            String cmdArgs[] = currentLine.split(" ");
            currentLine = infile.readLine();

            // parse commands
            switch(cmdArgs[0]) {
                case "fd" : {
                    System.out.printf("Forward %f\n", Double.parseDouble(cmdArgs[1]));
                    break;
                }
                case "bk" : {
                    System.out.printf("Backwards %f\n", Double.parseDouble(cmdArgs[1]));
                    break;
                }
                // not gonna waste my time adding commands that I'll inevitably have to rewrite later
            }
        }
        infile.close();
    }
}