package robotChemist.net;

/***
 * A class that represents job messages exchanged between the LBRiiwa
 * arm and the KMR base.
 * @author stoic-roboticist
 *
 */
public class JobMsg
{
	private int jobCode = 0;
	private String jobInfo = "";
	
	public JobMsg()
	{}
	
	public JobMsg(String jobInfo, int jobCode)
	{
		this.jobInfo = jobInfo;
		this.jobCode = jobCode;
	}
	
	public JobMsg(JobMsg msg)
	{
		this.jobInfo = msg.jobInfo;
		this.jobCode = msg.jobCode;
	}
	
	@Override
	public boolean equals(Object obj)
	{
		if (this == obj)
		{
			return true;
		}
		if (obj instanceof JobMsg)
		{
			boolean checkInfo = ((JobMsg) obj).jobInfo.equals(this.jobInfo);
			boolean checkCode = ((JobMsg) obj).jobCode == this.jobCode;
			return checkInfo && checkCode;
		}
		return false;
	}

	/***
	 * Gets the job info of the message
	 * @return a string with the job info
	 */
	public String getJobInfo()
	{
		return jobInfo;
	}
	
	/***
	 * Set the job info of the message
	 * @param msg - a string with the job info
	 */
	public void setJobInfo(String msg)
	{
		this.jobInfo = msg;
	}
	
	/***
	 * Gets the job code number
	 * @return the job code number
	 */
	public int getJobCode()
	{
		return jobCode;
	}
	
	/***
	 * Sets the job code number
	 * @param code - the job code number
	 */
	public void setJobCode(int code)
	{
		this.jobCode = code;
	}
	
	public String toString()
	{
		return String.format("Job info: %s, Job ID: %d", jobInfo, jobCode);
	}
}
