package robotChemist.processes;

import robotChemist.nodes.LBRArchmQNode;
import robotChemist.utility.LBRTask;
import robotChemist.utility.LBRTaskMonitor;
import robotChemist.utility.RobotOpState;
import robotChemist.utility.RobotOpState.OpState;

import com.kuka.task.ITaskLogger;

public class RosExecutionProcess extends StateMachineProcess 
{
	public enum RosExecutionState 
	{
		WAITING_ON_CMD("WAITING_ON_CMD"),
		EXECUTING("EXECUTING");
	
		private final String name;
		private RosExecutionState(String name)
		{
			this.name = name;
		}
	};
	
	private LBRArchmQNode lbrNode;
	private RosExecutionState currentState;
	private kmriiwa_chemist_msgs.LBRCommand currentCmd;
	
	public RosExecutionProcess(LBRArchmQNode lbrNode, LBRTaskMonitor taskMonitor, RobotOpState robotOpState, ITaskLogger logger) 
	{
		super(taskMonitor, robotOpState, logger);
		this.lbrNode = lbrNode;
		this.currentState = RosExecutionState.WAITING_ON_CMD;
	}

	@Override
	public String getCurrentState() 
	{
		return currentState.name();
	}

	@Override
	protected void executeStateProcess() 
	{
		if (currentState == RosExecutionState.EXECUTING)
		{
			LBRTask task = new LBRTask(currentCmd);
			taskMonitor.assignTask(task);
		}
	}

	@Override
	protected boolean executeStateTransitions() 
	{
		boolean transitionOccured = false;
		if (currentState ==  RosExecutionState.WAITING_ON_CMD)
		{
			kmriiwa_chemist_msgs.LBRCommand taskMsg = lbrNode.getLBRCommandfromQueue();
			if (taskMsg != null)
			{
				currentCmd = taskMsg;
				lbrNode.removeLBRCommandFromQueue(taskMsg);
				robotOpState.setCurrentState(OpState.EXECUTING_ROS_TASK);
				currentState = RosExecutionState.EXECUTING;
				transitionOccured = true;
			}
		}
		else if (currentState ==  RosExecutionState.EXECUTING)
		{
			if (taskMonitor.isTaskFinished())
			{
				robotOpState.setCurrentState(OpState.IDLE);
				currentState = RosExecutionState.WAITING_ON_CMD;
				transitionOccured = true;
			}
		}
		return transitionOccured;
	}

	@Override
	public void execute() 
	{
		if (robotOpState.getCurrentState() == OpState.IDLE || robotOpState.getCurrentState() == OpState.EXECUTING_ROS_TASK)
		{
			updateStateMachine();
		}
	}

}
