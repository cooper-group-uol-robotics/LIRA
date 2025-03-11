package robotChemist.utility;

import java.util.List;

public class LBRTask 
{
	private String name;
	private List<String> params;
	private boolean priority;
	private int seq;
	private int status;
	
	public LBRTask(String name, List<String> params, boolean priority)
	{
		this.name = name;
		this.params = params; 
		this.priority = priority;
		this.status = kmriiwa_chemist_msgs.TaskStatus.WAITING;
		this.seq = -1;
	}
	
	public LBRTask(String name, boolean priority)
	{
		this.name = name;
		this.params = null; 
		this.priority = priority;
		this.status = kmriiwa_chemist_msgs.TaskStatus.WAITING;
		this.seq = -1;
	}

	public LBRTask(kmriiwa_chemist_msgs.LBRCommand lbrCmd)
	{
		this.name = lbrCmd.getTaskName();
		this.params = lbrCmd.getTaskParameters();
		this.priority = lbrCmd.getPriorityTask();
		this.seq = lbrCmd.getCmdSeq();
		this.status = kmriiwa_chemist_msgs.TaskStatus.WAITING;
	}
	
	public LBRTask(LBRTask copyTask)
	{
		this.name = copyTask.name;
		this.params = copyTask.params; 
		this.priority = copyTask.priority;
		this.status = copyTask.status;
		this.seq = copyTask.seq;
	}
	
	public String getName() 
	{
		return name;
	}

	public boolean isPriority() 
	{
		return priority;
	}

	public int getStatus() {
		return status;
	}

	public void setStatus(int status) 
	{
		this.status = status;
	}
	
	public List<String> getParams()
	{
		return params;
	}
	
	public int getSeq()
	{
		return seq;
	}
	
	@Override
	public String toString()
	{
		return name;
	}
}
