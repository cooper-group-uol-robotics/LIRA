package robotChemist.utility;

public class LBRTaskMonitor 
{
	private LBRTask task;
	
	public LBRTaskMonitor()
	{
		this.task = null;
	}
	
	public LBRTask getAssignedTask()
	{
		if (task != null)
		{
			LBRTask copyTask = new LBRTask(task);
			return copyTask;
		}
		else
		{
			return task;
		}
	}
	
	public void assignTask(LBRTask task)
	{
		this.task = new LBRTask(task);
	}
	
	public void setTaskToExecuting()
	{
		if (task != null)
		{
			task.setStatus(kmriiwa_chemist_msgs.TaskStatus.EXECUTING);
		}
	}
	
	public void setTaskToFinished()
	{
		if (task != null && task.getStatus() == kmriiwa_chemist_msgs.TaskStatus.EXECUTING)
		{
			task.setStatus(kmriiwa_chemist_msgs.TaskStatus.FINISHED);
		}
	}
	
	public void setTaskToError()
	{
		if (task != null && task.getStatus() == kmriiwa_chemist_msgs.TaskStatus.EXECUTING)
		{
			task.setStatus(kmriiwa_chemist_msgs.TaskStatus.ERROR);
		}
	}
	
	public boolean isTaskAssigned()
	{
		return task != null;
	}
	
	public boolean isTaskFinished()
	{
		return task != null && task.getStatus() == kmriiwa_chemist_msgs.TaskStatus.FINISHED;
	}

}
