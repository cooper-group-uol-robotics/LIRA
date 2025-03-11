package robotChemist.interfaces;

import java.util.Random;

import com.kuka.generated.ioAccess.BMSIOGroup;
import com.kuka.generated.ioAccess.ExternalControlIOGroup;
import com.kuka.roboticsAPI.applicationModel.IApplicationData;
import com.kuka.task.ITaskLogger;

public class BatteryChargeManager 
{
	
	private ITaskLogger logger;
	private IApplicationData appData;
	
	private BMSIOGroup bms;

	private ExternalControlIOGroup externalControl;
	
	public BatteryChargeManager(BMSIOGroup bms, ExternalControlIOGroup externalControl, ITaskLogger logger, IApplicationData appData)
	{
		this.logger = logger;
		this.appData = appData;
		this.bms = bms;
		this.externalControl = externalControl;
	}
	
	public int getStateOfCharge()
	{
		return bms.getStateOfCharge();
	}
	
	public boolean isChargingNeeded()
	{
		return bms.getStateOfCharge() <= (Integer) appData.getProcessData("tempMinimalChargeBattery").getValue();
	}
	
	public boolean isChargingDone()
	{
		return bms.getStateOfCharge() > (Integer) appData.getProcessData("tempMaximalChargeBattery").getValue();
	}
	
	public void startCharging() throws InterruptedException 
	{
		Thread.sleep(1000);
		bms.setChargingEnable(true);
		Thread.sleep(1000);
		bms.setChargingRelayEnable(true);
		Thread.sleep(1000);
		externalControl.setChargingRelayEnabled(true);
	}
	
	public void stopCharging() throws InterruptedException 
	{
		Thread.sleep(1000);
		bms.setChargingEnable(false);
		Thread.sleep(1000);
		bms.setChargingRelayEnable(false);
		Thread.sleep(1000);
		externalControl.setChargingRelayEnabled(false);
	}
	
	public boolean isChargingProcessNotWorking() 
	{
		return bms.getStatus() == 2 && bms.getChargingEnable() && bms.getStateOfCharge() < 50;
	}
	
	public void updateChargingProcessData()
	{
		Random rand = new Random();
		int newMaximalCharge = (Integer) appData.getProcessData("maximalChargeBattery").getValue() + (int) Math.floor(rand.nextDouble() * 11d - 5d);
		appData.getProcessData("tempMaximalChargeBattery").setValue(newMaximalCharge);
		int newTempMinimalCharge = Math.min(99,(Integer) appData.getProcessData("minimalChargeBattery").getValue() + (int) Math.floor(rand.nextDouble() * 11d - 5d));
		appData.getProcessData("tempMinimalChargeBattery").setValue(newTempMinimalCharge);
	}
	
	public boolean forceStartCharging(int targetMaxCharge)
	{
		boolean chargeOverriden = false;
		int currentCharge = getStateOfCharge();
		if (targetMaxCharge > currentCharge + 1)
		{
			logger.info(String.format("Charge override command received. Robot will charge to %d%%", targetMaxCharge));
			appData.getProcessData("tempMaximalChargeBattery").setValue(targetMaxCharge);
			appData.getProcessData("tempMinimalChargeBattery").setValue(getStateOfCharge()+1);
			chargeOverriden = true;
		}
		else
		{
			logger.warn(String.format("Charge override command ignored because robot charge >= %d%% charge", targetMaxCharge));
		}
		return chargeOverriden;
	}
	
	public void forceStopCharging()
	{
		int currentCharge = getStateOfCharge();
		appData.getProcessData("tempMaximalChargeBattery").setValue(currentCharge);
	}

}
