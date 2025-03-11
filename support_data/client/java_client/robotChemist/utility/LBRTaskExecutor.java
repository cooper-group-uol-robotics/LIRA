package robotChemist.utility;

import com.kuka.roboticsAPI.applicationModel.IApplicationData;
import com.kuka.task.ITaskLogger;

import robotChemist.interfaces.BatteryChargeManager;
import robotChemist.interfaces.LBRCommander;

public class LBRTaskExecutor 
{
	protected LBRCommander robot;
	private BatteryChargeManager battaryManager;
	protected IApplicationData appData;
	protected ITaskLogger logger;
	
	public LBRTaskExecutor(LBRCommander robot,BatteryChargeManager battaryManager, IApplicationData appData, ITaskLogger logger)
	{
		this.robot = robot;
		this.battaryManager = battaryManager;
		this.appData = appData;
		this.logger = logger;
	}
	
	public boolean execute(LBRTask task)
	{
		boolean executionRes = false;
		if (task.isPriority())
		{
			if (task.getName().equals("ArmDrivePos"))
			{
				logger.info("Asserting arm in drive position");
				robot.getArm().assertArmInDrivePos();
				executionRes = true;
			}
			else if (task.getName().equals("ReferenceArm"))
			{
				logger.info("Referencing the arm");
				robot.getArm().Reference();
				executionRes = true;
			}
			else if (task.getName().equals("ChargeRobot"))
			{
				logger.info("Attempting manual robot charging");
				int targetMaxCharge = Integer.valueOf(task.getParams().get(1));
				battaryManager.forceStartCharging(targetMaxCharge);
				executionRes = true;
			}
			else if (task.getName().equals("StopCharge"))
			{
				logger.info("Attempting manual robot charging stop");
				battaryManager.forceStopCharging();
				executionRes = true;
			}
			else if (task.getName().equals("DiableAutoFunctions"))
			{
				logger.info("Disabling auto charging and calibration.");
				appData.getProcessData("AllowAutomatedCharging").setValue(false);
				appData.getProcessData("AllowAutomatedArmCalibration").setValue(false);
				executionRes = true;
			}
			else if (task.getName().equals("EnableAutoFunctions"))
			{
				logger.info("Enabling auto charging and calibration.");
				appData.getProcessData("AllowAutomatedCharging").setValue(true);
				appData.getProcessData("AllowAutomatedArmCalibration").setValue(true);
				executionRes = true;
			}
		}
		else
		{
			executionRes = executeWorkflowTasks(task);
		}
		return executionRes;
	}
	
	protected boolean executeWorkflowTasks(LBRTask task)
	{
		// empty stump method, override in children
		return true;
	}
}
