package robotChemist.utility;

import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplicationState;

public class AppStateMonitor 
{
	private RoboticsAPIApplicationState appState;
	
	public AppStateMonitor(RoboticsAPIApplicationState appState)
	{
		this.appState = appState;
	}

	public RoboticsAPIApplicationState getAppState() {
		return appState;
	}

	public void setAppState(RoboticsAPIApplicationState appState) {
		this.appState = appState;
	}
	
	
}
