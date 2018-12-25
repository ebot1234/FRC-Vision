import edu.wpi.first.wpilibj.networktables.*;

/**
 * Factory class to wire up a blue ball image processor
 * with all the correct dependencies.
 */
public class CubeImageProcessorFactory {
  /**
   * Static helper to create an image processor instance.
   * @param networkTable  The network table to write to
   * @return
   */
    public static ImageProcessor CreateImageProcessor(NetworkTable networkTable) {
        ICubePipeline cubePipeline = new CubePipeline();
        return 
          new ImageProcessor(
            cubePipeline, 
            new CubeNetworkTableWriter(
              new CubePipelineInterpreter(cubePipeline), 
              networkTable));
    }
}