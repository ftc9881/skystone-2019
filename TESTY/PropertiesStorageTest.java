/* .PROPERTIES FORMAT - TEST/DEMONSTRATION
 * Test of the `properties` storage format
 * 
 * We'll probably need this later, since it's
 * inconvenient to set variables and rebuild 
 * every time we want to try something out.
 * 
 * Feel free to use this for reference if we end up using the `.properties` format.
 */

import java.util.Properties;
import java.io.FileInputStream;
import java.io.IOException;

public class PropertiesStorageTest {
    public static void main(String[] args) throws IOException {
        FileInputStream inFile = new FileInputStream("TestyConfig.properties");
        Properties props = new Properties();
        props.load(inFile);
        inFile.close();

        System.out.printf("setting1: %s\nsetting2: %s\n", props.getProperty("setting1"), props.getProperty("setting2"));
    }
}