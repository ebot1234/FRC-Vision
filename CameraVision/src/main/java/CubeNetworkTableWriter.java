import edu.wpi.first.wpilibj.networktables.*;

/**
 * For writing specific data to Network Tables for Power Up Cubes
 * 
 * @author Chuck Benedict, Mentor, Team 997, Original Project
 * @author Ethen Brandenburg, Team 1080, Modified Project
 */
public class CubeNetworkTableWriter extends NetworkTableWriter
{
    public CubeNetworkTableWriter(CubePipelineInterpreter interpreter, NetworkTable publishingTable) {
        super(interpreter, publishingTable);
    }

    public String getCubeFoundKey() {
        return "CubeFound";
    }
    public String getCubeCountKey() {
        return "CubeCount";
    }

   
}