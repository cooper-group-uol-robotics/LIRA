package robotChemist.utility;
/***
 * This class describes and manages the robot operational state. Possible state:</br>
 * - INVALID: robot is not initialised</br>
 * - IDLE: robot ready to execute tasks</br>
 * - MANIPULATING: robot is busy executing a manipulation task</br>
 * - CHARGING: robot is charging and not available for tasks </br>
 * - CALIBRATING: robot is calibrating and not available for tasks </br>
 * @author stoic-roboticist
 *
 */
public class RobotOpState
{
	public enum OpState 
	{
		INVALID("INVALID"),
		IDLE("IDLE"),
		EXECUTING_ROS_TASK("EXECUTING_ROS_TASK"),
		EXECUTING("EXECUTING"),
		CHARGING("CHARGING"),
		CALIBRATING("CALIBRATING");
		
		private final String name;
		private OpState(String name)
		{
			this.name = name;
		}
	};
	
	private OpState currentState;
	
	public RobotOpState()
	{
		currentState = OpState.INVALID;
	}

	public OpState getCurrentState() 
	{
		return currentState;
	}

	public void setCurrentState(OpState currentState) 
	{
		this.currentState = currentState;
	}
	
}
