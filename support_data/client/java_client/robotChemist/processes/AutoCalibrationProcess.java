package robotChemist.processes;

import java.io.IOException;
import java.util.Date;
import java.util.concurrent.LinkedBlockingQueue;

import robotChemist.net.AsyncJobClient;
import robotChemist.net.JobMsg;
import robotChemist.utility.LBRTask;
import robotChemist.utility.LBRTaskMonitor;
import robotChemist.utility.RobotOpState;
import robotChemist.utility.RobotOpState.OpState;

import com.kuka.roboticsAPI.applicationModel.IApplicationData;
import com.kuka.task.ITaskLogger;

public class AutoCalibrationProcess extends StateMachineProcess 
{
	enum CalibrationState 
	{
		INVALID("INVALID"),
		CHECKING_FOR_CALIBRATION("CHECKING_FOR_CALIBRATION"),
		PREP_CALIBRATTION("PREP_CALIBRATTION"),
		CALIBRATING("CALIBRATING"),
		POST_CALIBRATION("POST_CALIBRATION");
	
		private final String name;
		private CalibrationState(String name)
		{
			this.name = name;
		}
	};
	
	private AsyncJobClient kmrClient;
	private IApplicationData appData;
	private CalibrationState currentState;
	
	public AutoCalibrationProcess(AsyncJobClient kmrClient, IApplicationData appData, LBRTaskMonitor taskMonitor, RobotOpState robotOpState, ITaskLogger logger)
	{
		super(taskMonitor, robotOpState, logger);
		this.kmrClient = kmrClient;
		this.appData = appData;
		this.currentState = CalibrationState.CHECKING_FOR_CALIBRATION;
	}
	
	@Override
	protected void executeStateProcess()
	{
		try
		{
			if (currentState == CalibrationState.PREP_CALIBRATTION)
			{
				kmrClient.sendMessage(new JobMsg("goto_calibrate",0));
				
			}
			else if (currentState == CalibrationState.CALIBRATING)
			{
				logger.info("Robot is calibrating");
				LBRTask task = new LBRTask("ReferenceArm", true);
				taskMonitor.assignTask(task);
			}
			else if (currentState == CalibrationState.POST_CALIBRATION)
			{
				kmrClient.sendMessage(new JobMsg("done_calibrating",0));
				
			}
		}
		catch (IOException e)
		{
			logger.error("An IOException error occured in the auto calibration process when state executing");
			logger.error(e.toString());
		}
	}
	
	@Override
	protected boolean executeStateTransitions() 
	{
		boolean transitionOccured = false;
		try
		{
			if (currentState == CalibrationState.CHECKING_FOR_CALIBRATION)
			{
				boolean needToCalibrate = false;
				long lastCalibration = Long.parseLong((String) appData.getProcessData("lastCalibrationOfArm").getValue());
				int noCalibrationDuration = (Integer) appData.getProcessData("maximal_time_without_calibration").getValue();
				if ((new Date().getTime() - lastCalibration) > noCalibrationDuration*60*1000)
				{
					needToCalibrate = true;
				}
				if (needToCalibrate)
				{
					robotOpState.setCurrentState(OpState.CALIBRATING);
					currentState = CalibrationState.PREP_CALIBRATTION;
					transitionOccured = true;
				}
			}
			else if (currentState == CalibrationState.PREP_CALIBRATTION)
			{
				LinkedBlockingQueue<JobMsg> msgQueue = kmrClient.getMessageQueue();
				if (!msgQueue.isEmpty() && msgQueue.remove(new JobMsg("goto_calibrate", 1)))
				{
					currentState = CalibrationState.CALIBRATING;
					transitionOccured = true;
				}
			}
			else if (currentState == CalibrationState.CALIBRATING)
			{
				if (taskMonitor.isTaskFinished())
				{
					currentState = CalibrationState.POST_CALIBRATION;
					transitionOccured = true;
				}

			}
			else if (currentState == CalibrationState.POST_CALIBRATION)
			{
				robotOpState.setCurrentState(OpState.IDLE);
				currentState = CalibrationState.CHECKING_FOR_CALIBRATION;
				transitionOccured = true;
			}
		}
		catch (Exception e)
		{
			logger.error("An InterruptedException error occured in the auto calibration process while state transitioning");
			logger.error(e.toString());
		}
		return transitionOccured;
	}

	@Override
	public String getCurrentState() {
		return currentState.name();
	}

	@Override
	public void execute() 
	{
		if (robotOpState.getCurrentState() == OpState.IDLE || robotOpState.getCurrentState() == OpState.CALIBRATING)
		{
			updateStateMachine();
		}	
	}

}
