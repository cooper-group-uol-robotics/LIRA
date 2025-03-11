package robotChemist.processes;

import java.io.IOException;
import java.util.concurrent.LinkedBlockingQueue;

import com.kuka.task.ITaskLogger;

import robotChemist.interfaces.BatteryChargeManager;
import robotChemist.net.AsyncJobClient;
import robotChemist.net.JobMsg;
import robotChemist.utility.LBRTaskMonitor;
import robotChemist.utility.RobotOpState;
import robotChemist.utility.RobotOpState.OpState;

public class AutoChargingProcess extends StateMachineProcess 
{
	enum ChargingState 
	{
		INVALID("INVALID"),
		CHECKING_FOR_CHARGING("CHECKING_FOR_CHARGING"),
		PREP_CHARGING("PREP_CHARGING"),
		READY_TO_CHARGE("READY_TO_CHARGE"),
		CHARGING("CHARGING"),
		POST_CHARGE("POST_CHARGE"),
		FINISH_CHARGING("FINISH_CHARGING");
	
		private final String name;
		private ChargingState(String name)
		{
			this.name = name;
		}
	};
	
	private BatteryChargeManager batteryManager;
	private AsyncJobClient kmrClient;
	private ChargingState currentState;
	
	public AutoChargingProcess(AsyncJobClient kmrClient, BatteryChargeManager batteryManager, LBRTaskMonitor taskMonitor, RobotOpState robotOpState, ITaskLogger logger)
	{
		super(taskMonitor, robotOpState, logger);
		this.batteryManager = batteryManager;
		this.kmrClient = kmrClient;
		this.currentState = ChargingState.CHECKING_FOR_CHARGING;
	}

	@Override
	protected void executeStateProcess()
	{
		try
		{
			if (currentState == ChargingState.PREP_CHARGING)
			{
				kmrClient.sendMessage(new JobMsg("goto_charge",0));
				
			}
			else if (currentState == ChargingState.READY_TO_CHARGE)
			{
				batteryManager.startCharging();
				Thread.sleep(30*1000);
				kmrClient.sendMessage(new JobMsg("started_charging",0));
				logger.info("Robot started charging");
			}
			else if (currentState == ChargingState.POST_CHARGE)
			{
				kmrClient.sendMessage(new JobMsg("done_charging",0));
			}
		}
		catch (IOException e)
		{
			logger.error("An IOException error occured in the charging process when state executing");
			logger.error(e.toString());
		}
		catch (InterruptedException e)
		{
			logger.error("An InterruptedException error occured in the charging process when state executing");
			logger.error(e.toString());
		}
	}

	@Override
	protected boolean executeStateTransitions() 
	{
		boolean transitionOccured = false;
		try
		{
			if (currentState == ChargingState.CHECKING_FOR_CHARGING)
			{
				if (batteryManager.isChargingNeeded())
				{
					robotOpState.setCurrentState(OpState.CHARGING);
					currentState = ChargingState.PREP_CHARGING;
					transitionOccured = true;
				}
			}
			else if (currentState == ChargingState.PREP_CHARGING)
			{
				LinkedBlockingQueue<JobMsg> msgQueue = kmrClient.getMessageQueue();
				if (!msgQueue.isEmpty() && msgQueue.remove(new JobMsg("goto_charge", 1)))
				{
					currentState = ChargingState.READY_TO_CHARGE;
					transitionOccured = true;
				}
			}
			else if (currentState == ChargingState.READY_TO_CHARGE)
			{
				LinkedBlockingQueue<JobMsg> msgQueue = kmrClient.getMessageQueue();
				if (!msgQueue.isEmpty() && msgQueue.remove(new JobMsg("started_charging", 1)))
				{
					currentState = ChargingState.CHARGING;
					transitionOccured = true;
				}
			}
			else if (currentState == ChargingState.CHARGING)
			{
				if (batteryManager.isChargingProcessNotWorking())
				{
					logger.error("Something went wrong in the charging process.");
					throw new RuntimeException("Charging process stopped for some reason!!!");
				}
				if(batteryManager.isChargingDone())
				{
					batteryManager.stopCharging();
					batteryManager.updateChargingProcessData();
					currentState = ChargingState.POST_CHARGE;
					transitionOccured = true;
				}
			}
			else if (currentState == ChargingState.POST_CHARGE)
			{
				LinkedBlockingQueue<JobMsg> msgQueue = kmrClient.getMessageQueue();
				if (!msgQueue.isEmpty() && msgQueue.remove(new JobMsg("done_charging", 1)))
				{
					robotOpState.setCurrentState(OpState.IDLE);
					currentState = ChargingState.CHECKING_FOR_CHARGING;
					transitionOccured = true;
				}
			}
		}
		catch (InterruptedException e)
		{
			logger.error("An InterruptedException error occured in the charging process while state transitioning");
			logger.error(e.toString());
		}
		return transitionOccured;
	};
	
	@Override
	public void execute()
	{
		if (robotOpState.getCurrentState() == OpState.IDLE || robotOpState.getCurrentState() == OpState.CHARGING)
		{
			updateStateMachine();
		}
	}

	@Override
	public String getCurrentState() {
		return currentState.name();
	}
}
