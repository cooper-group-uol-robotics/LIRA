package robotChemist.processes;

import java.io.IOException;
import java.util.concurrent.LinkedBlockingQueue;

import robotChemist.net.AsyncJobClient;
import robotChemist.net.JobMsg;
import robotChemist.utility.AppStateMonitor;
import robotChemist.utility.LBRTask;
import robotChemist.utility.LBRTaskExecutor;
import robotChemist.utility.LBRTaskMonitor;
import robotChemist.utility.RobotOpState;
import robotChemist.utility.RobotOpState.OpState;

import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplicationState;
import com.kuka.task.ITaskLogger;

public class TaskExecutionProcess extends StateMachineProcess 
{
	
	public enum TaskExecutionState 
	{
		INVALID("INVALID"),
		CHECKING_FOR_TASK("CHECKING_FOR_TASK"),
		CHECKING_APP_STATE("CHECKING_APP_STATE"),
		EXECUTING("EXECUTING"),
		REQUEST_APP_RESUME("REQUEST_APP_RESUME"),
		ERROR("ERROR");
	
		private final String name;
		private TaskExecutionState(String name)
		{
			this.name = name;
		}
	};
	
	private LBRTaskExecutor taskExecutor;
	private AppStateMonitor appStateMonitor;
	private AsyncJobClient kmrClient;
	private TaskExecutionState currentState;
	private boolean execSuccessful;
	private OpState previousOpState;

	public TaskExecutionProcess(LBRTaskExecutor taskExecutor, AsyncJobClient kmrClient, AppStateMonitor appStateMonitor, LBRTaskMonitor taskMonitor,RobotOpState robotOpState, ITaskLogger logger) 
	{
		super(taskMonitor, robotOpState, logger);
		this.taskExecutor = taskExecutor;
		this.execSuccessful = false;
		this.kmrClient = kmrClient;
		this.appStateMonitor = appStateMonitor;
		this.currentState = TaskExecutionState.CHECKING_FOR_TASK;
	}

	@Override
	public String getCurrentState() 
	{
		return currentState.name();
	}

	@Override
	protected void executeStateProcess() 
	{
		try
		{
			if (currentState == TaskExecutionState.REQUEST_APP_RESUME)
			{
				kmrClient.sendMessage(new JobMsg("need_to_resume",0));
			}
			else if (currentState == TaskExecutionState.EXECUTING)
			{
				taskMonitor.setTaskToExecuting();
				LBRTask task = taskMonitor.getAssignedTask();
				execSuccessful = taskExecutor.execute(task);
			}
		}
		catch (IOException e)
		{
			logger.error("An IOException error occured in the manipulation process when state executing");
			logger.error(e.toString());
		}
	}

	@Override
	protected boolean executeStateTransitions() 
	{
		boolean transitionOccured = false;
		try
		{
			if (currentState == TaskExecutionState.CHECKING_FOR_TASK)
			{
				if (taskMonitor.isTaskAssigned() && !taskMonitor.isTaskFinished())
				{
					execSuccessful = false;
					previousOpState = robotOpState.getCurrentState();
					robotOpState.setCurrentState(OpState.EXECUTING);
					currentState = TaskExecutionState.CHECKING_APP_STATE;
					transitionOccured = true;
				}
			}
			else if (currentState == TaskExecutionState.CHECKING_APP_STATE)
			{
				if (appStateMonitor.getAppState() == RoboticsAPIApplicationState.RESUMING)
				{
					currentState = TaskExecutionState.EXECUTING;
					transitionOccured = true;
				}
				else if (appStateMonitor.getAppState() == RoboticsAPIApplicationState.MOTIONPAUSING)
				{
					currentState = TaskExecutionState.REQUEST_APP_RESUME;
					transitionOccured = true;
				}
			}
			else if (currentState == TaskExecutionState.REQUEST_APP_RESUME)
			{
				LinkedBlockingQueue<JobMsg> msgQueue = kmrClient.getMessageQueue();
				if (!msgQueue.isEmpty() && msgQueue.remove(new JobMsg("app_resumed", 0)))
				{
					currentState = TaskExecutionState.EXECUTING;
					transitionOccured = true;
				}
			}
			else if (currentState == TaskExecutionState.EXECUTING)
			{
				if (execSuccessful)
				{
					taskMonitor.setTaskToFinished();
					robotOpState.setCurrentState(previousOpState);
					currentState = TaskExecutionState.CHECKING_FOR_TASK;
					transitionOccured = true;
				}
				else
				{
					taskMonitor.setTaskToError();
					currentState = TaskExecutionState.ERROR;
					transitionOccured = true;
				}
			}
		}
		catch (Exception e)
		{
			logger.error("An Exception error occured in the manipulation process when executing");
			logger.error(e.toString());
		}
		return transitionOccured;
	}

	@Override
	public void execute() 
	{
		updateStateMachine();
	}

}
