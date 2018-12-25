import org.opencv.core.*;

interface ICubePipeline {
    public void process(Mat source0);
    public MatOfKeyPoint findBlobsOutput();
    public String getColor();//might not need that for 2018
}