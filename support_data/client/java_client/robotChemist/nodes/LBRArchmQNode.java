package robotChemist.nodes;

import java.util.concurrent.LinkedBlockingDeque;

import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import com.kuka.task.ITaskLogger;

import robotChemist.utility.LBRStatus;
import robotChemist.utility.LBRTask;
import robotChemist.utility.LBRTaskMonitor;
import robotChemist.utility.RobotOpState;

/***
 * A class that implements a ROS node to be used in the ARCHemist architecture.
 * It exposes all the required interfaces for the architecture to interact with the LBR
 * arm.
 * @author stoic-roboticist
 *
 */
public class LBRArchmQNode extends LBRBaseNode
{
	
	private ConnectedNode node = null;
	private ITaskLogger logger;
	private String robotName = "kmriiwa";
	
	// ROS subscribers
	private Subscriber<kmriiwa_chemist_msgs.LBRCommand> lbrCmdSubscriber;
	
	// ROS publishers
	// A publisher to request station jobs
	private Publisher<kmriiwa_chemist_msgs.TaskStatus> taskStatusPublisher;
	// KMR base state publisher
	private Publisher<kmriiwa_chemist_msgs.LBRStatus> lbrStatusPublisher;
	
	private int cmdSeqCounter = 0;
	private LinkedBlockingDeque<kmriiwa_chemist_msgs.LBRCommand> lbrCmdQueue;
	
	
	/***
	 * Constructs the node with the given robot name.
	 * @param robotName - the robot name to be used in the message headers.
	 */
	public LBRArchmQNode(String robotName, ITaskLogger logger)
	{
		this.robotName = robotName;
		this.logger = logger;
		this.lbrCmdQueue = new LinkedBlockingDeque<kmriiwa_chemist_msgs.LBRCommand>(10);
	}
	
	/***
	 * Get the ROS node name.
	 */
	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of(robotName + "/lbr_node");
	}
	
	/***
	 * This methods is called when the ROS node connects to the master.
	 * It represent the point at which the different ROS interfaces gets constructed.
	 */
	@Override
	public void onStart(ConnectedNode connectedNode)
	{
		node = connectedNode;
		taskStatusPublisher = node.newPublisher(robotName + "/lbr/task_status", kmriiwa_chemist_msgs.TaskStatus._TYPE);
		lbrStatusPublisher = node.newPublisher(robotName + "/lbr/robot_status", kmriiwa_chemist_msgs.LBRStatus._TYPE);
		lbrCmdSubscriber = node.newSubscriber(robotName + "/lbr/command", kmriiwa_chemist_msgs.LBRCommand._TYPE);
		lbrCmdSubscriber.addMessageListener( new MessageListener<kmriiwa_chemist_msgs.LBRCommand>() {
			@Override
			public void onNewMessage(kmriiwa_chemist_msgs.LBRCommand msg)
			{
				if (msg.getCmdSeq() > cmdSeqCounter)
				{
					if (msg.getCmdSeq() > cmdSeqCounter + 1)
					{
						logger.warn("A ROS task message was lost somewhere");
					}
					cmdSeqCounter++;
					boolean msgAdded;
					try
					{
						if(msg.getPriorityTask())
						{
							msgAdded = lbrCmdQueue.offerFirst(msg);
						}
						else
						{
							msgAdded = lbrCmdQueue.offerLast(msg);
						}
						if (!msgAdded)
						{
							logger.warn("ROS tasks queue is full, the most recent message couldn't be added to the queue");
						}
					}
					catch (Exception e)
					{
						e.printStackTrace();
						logger.error("failed to retrieve message on callback");
					}
				}
				
			}
		});
		
		connectedToMaster = true;
	}
	
	/***
	 * Retrieve the LBRCommand at the head of the command queue received via the <em>[robot_name]/lbr/command</em> topic without
	 * deleting it.
	 * @return LBRCommand message from the head of the command queue received on the <em>[robot_name]/lbr/command</em>. If the queue is empty, a null is returned.
	 */
	public kmriiwa_chemist_msgs.LBRCommand getLBRCommandfromQueue()
	{
		return lbrCmdQueue.peek();
	}
	
	/***
	 * Remove the LBRCommand at the head of the command queue. This method is used after the command message has
	 * been executed.
	 */
	public void removeLBRCommandFromQueue(kmriiwa_chemist_msgs.LBRCommand lbrCmd)
	{
		lbrCmdQueue.remove(lbrCmd);
	}
	
	/***
	 * Publishes the given LBRStatus on the <em>[robot_name]/lbr/robot_status</em> topic.
	 * @param status - the robot status to be published.
	 */
	public void publishRobotStatus(LBRStatus status, RobotOpState robotOpState)
	{
		kmriiwa_chemist_msgs.LBRStatus rosMessage = node.getTopicMessageFactory().newFromType(kmriiwa_chemist_msgs.LBRStatus._TYPE);
		rosMessage.getHeader().setStamp(node.getCurrentTime());
		rosMessage.setMotionEnabled(status.isMotionEnabled());
		rosMessage.setAxesGmsReferenced(status.areAxesGMSReferenced());
		rosMessage.setAxesPositionReferenced(status.areAxesPosReferenced());
		switch (status.getSafetyStopState())
		{
			case NOSTOP:
				rosMessage.setSafetyStopState(rosMessage.NOSTOP);
				break;
			case STOP0:
				rosMessage.setSafetyStopState(rosMessage.STOP0);
				break;
			case STOP1:
				rosMessage.setSafetyStopState(rosMessage.STOP1_ON_PATH);
				break;
			case STOP2:
				rosMessage.setSafetyStopState(rosMessage.STOP2);
				break;
		}
		rosMessage.setRobotOpState(robotOpState.getCurrentState().name());
		lbrStatusPublisher.publish(rosMessage);
	}
	
	/***
	 * Publishes task status update on the <em>[robot_name]/lbr/task_status</em> topic.
	 * @param taskName - name of the task.
	 * @param taskStatus - updated task status reported using the {@link kmriiwa_chemist_msgs.TaskStatus} enum.
	 */
	public synchronized void publishTaskStatus(LBRTaskMonitor taskMonitor) 
	{
		kmriiwa_chemist_msgs.TaskStatus taskMsg = node.getTopicMessageFactory().newFromType(kmriiwa_chemist_msgs.TaskStatus._TYPE);
		LBRTask task = taskMonitor.getAssignedTask();
		if (task != null)
		{
			taskMsg.setTaskName(task.getName());
			taskMsg.setTaskState(task.getStatus());
			taskMsg.setCmdSeq(task.getSeq());
		}
		else
		{
			taskMsg.setTaskName("");
			taskMsg.setTaskState(kmriiwa_chemist_msgs.TaskStatus.WAITING);
			taskMsg.setCmdSeq(-1);
		}
    	taskStatusPublisher.publish(taskMsg);
    }
}