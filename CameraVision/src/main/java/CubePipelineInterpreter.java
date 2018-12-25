/**
* Interpret the result of the ball pipeline.  This abstracts out the logic
* from the pipeline class.
*
* @author Chuck Benedict, Mentor, Team 997, original project
* @author Ethen Brandenburg, Team 1080, 2018 Modification
*/
public class CubePipelineInterpreter {

	// Processed pipeline that we will do the interpretation against
	private ICubePipeline pipeline;

	/**
	* Constructor taking a processed pipeline
	*
	* @param pipeline	A processed pipeline that returns blob found results
	*/
	public CubePipelineInterpreter(ICubePipeline pipeline) {
		if (pipeline == null)
		{
			throw new IllegalArgumentException("Pipline cannot be null.");
		}
		this.pipeline = pipeline;
	}

	/**
	 * Did we find at least one cube on a processed frame?
	 * 
	 * @return True if at least one cube was found
	 */
	public boolean cubesFound() {
		return !this.pipeline.findBlobsOutput().empty();
	}

	/**
	 * Get the count of the number of cubes found on a processed frame.
	 *  
	 * @return The count of the number of cubes found
	 */
	public long cubeCount() {
		return this.pipeline.findBlobsOutput().total();
	}
}