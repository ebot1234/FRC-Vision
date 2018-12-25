import org.opencv.core.*;

public class CubePipeline implements ICubePipeline {
    private CubeGripPipeline pipeline = new CubeGripPipeline();

    public void process(Mat source0) {
        pipeline.process(source0);
    }
    public MatOfKeyPoint findBlobsOutput() {
        return pipeline.findBlobsOutput();
    }
    public String getColor() {
        return "Yellow";
    }
}