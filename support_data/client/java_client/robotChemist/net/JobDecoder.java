package robotChemist.net;

import java.nio.ByteBuffer;


/***
 * A utility class that retrieves a decoded {@link JobMsg} from a supplied ByteBuffer.
 * @author stoic-roboticist
 *
 */
public class JobDecoder 
{
	
	/***
	 * Gets the encoded JobMsg from the supplied ByteBuffer by decoding its data.
	 * @param buffer - the ByteBuffer that contains the encoded message.
	 * @return decoded {@link JobMsg} from the input ByteBuffer.
	 */
	public JobMsg decode(ByteBuffer buffer)
	{
		JobMsg job = new JobMsg();
		
		int infoLen = buffer.getInt();
		byte[] jobInfo = new byte[infoLen];
		buffer.get(jobInfo);
		job.setJobInfo(new String(jobInfo));
		job.setJobCode(buffer.getInt());
		
		return job;
	}
}
