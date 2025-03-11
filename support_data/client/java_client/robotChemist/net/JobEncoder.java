package robotChemist.net;

import java.nio.ByteBuffer;


/***
 * A utility class that converts {@link JobMsg} to a ByteBuffer in order for it to
 * be sent across the communication channel.
 * @author stoic-roboticist
 *
 */
public class JobEncoder 
{
	
	/***
	 * Converts the given JobMsg data and put it inside the supplied ByteBuffer.
	 * @param job - the {@link JobMsg} whose data to be encoded to a byte buffer.
	 * @param buffer - the byte buffer that would contain the encoded JobMsg data.
	 */
	public void encode(JobMsg job, ByteBuffer buffer)
	{
		buffer.clear();
		
		byte[] jobInfo = job.getJobInfo().getBytes();
		int infoLen = jobInfo.length;
		buffer.putInt(infoLen);
		buffer.put(jobInfo);
		buffer.putInt(job.getJobCode());
		
		buffer.flip();
	}

}
