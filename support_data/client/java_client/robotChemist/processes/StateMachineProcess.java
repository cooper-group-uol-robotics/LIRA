package robotChemist.processes;

import com.kuka.task.ITaskLogger;

import robotChemist.utility.LBRTaskMonitor;
import robotChemist.utility.RobotOpState;

public abstract class StateMachineProcess 
{
	protected LBRTaskMonitor taskMonitor;
	protected RobotOpState robotOpState;
	protected ITaskLogger logger;

	public StateMachineProcess(LBRTaskMonitor taskMonitor, RobotOpState robotOpState, ITaskLogger logger)
	{
		this.taskMonitor = taskMonitor;
		this.robotOpState = robotOpState;
		this.logger = logger;
	}
	
	public abstract String getCurrentState();
	protected abstract void executeStateProcess();
	protected abstract boolean executeStateTransitions();
	
	protected void updateStateMachine()
	{
		String oldState = getCurrentState();
		boolean transitionOccured = executeStateTransitions();
		if (transitionOccured)
		{
			String newState = getCurrentState();
			logger.info(String.format("state_changed: %s -> %s", oldState, newState));
			executeStateProcess();
		}
	}
	
	public abstract void execute();
}
