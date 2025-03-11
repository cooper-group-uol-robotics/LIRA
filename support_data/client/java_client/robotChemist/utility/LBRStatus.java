package robotChemist.utility;

import com.kuka.roboticsAPI.controllerModel.sunrise.SunriseSafetyState.SafetyStopType;
	
/***
 * A utility class used to represent the LBRiiwa arm status. This includes:<br>
 * <br>
 * 1- the motion enabled state of the arm such that it can accept new commands.<br>
 * 2- GMS reference state of the arm joints.<br>
 * 3- Position reference state of the arm joints.<br>
 * 4- the safety stop status.<br>
 * @author stoic-roboticist
 *
 */
public class LBRStatus 
{
		
	private boolean motionEnabled;
	private boolean axesGMSRefernced = false;
	private boolean axesPosRefernced = false;
	private SafetyStopType safety;
	
	/***
	 * Constructs a LBRStatus message
	 * @param motionEnabled - the arm's motion enabled state.
	 * @param axesGMSRefernced - the arm joints' GMS references state.
	 * @param axesPosRefernced - the arm joints' position reference state.
	 * @param safety - the arm safety stop state.
	 */
	public LBRStatus (boolean motionEnabled, boolean axesGMSRefernced, boolean axesPosRefernced, SafetyStopType safety)
	{
		this.motionEnabled = motionEnabled;
		this.axesGMSRefernced = axesGMSRefernced;
		this.axesPosRefernced = axesPosRefernced;
		this.safety = safety;
	}
	
	/***
	 * 
	 * @return true if the arm is motion enabled such that it can receive new commands.
	 */
	public boolean isMotionEnabled()
	{
		return motionEnabled;
	}
	
	/***
	 * 
	 * @return true if the arm has been GMS referenced.
	 */
	public boolean areAxesGMSReferenced()
	{
		return axesGMSRefernced;
	}
	
	/***
	 * 
	 * @return true if the arm has been position referenced.
	 */
	public boolean areAxesPosReferenced()
	{
		return axesPosRefernced;
	}
	
	/***
	 *
	 * @return the {@link SafetyStopType} of the arm.
	 */
	public SafetyStopType getSafetyStopState()
	{
		return safety;
	}

}
