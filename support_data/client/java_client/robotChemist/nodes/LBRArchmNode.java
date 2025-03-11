package robotChemist.nodes;

import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import robotChemist.utility.LBRStatus;
import robotChemist.utility.RobotOpState;

/***
 * A class that implements a ROS node to be used in the ARCHemist architecture.
 * It exposes all the required interfaces for the architecture to interact with the LBR
 * arm.
 * @author stoic-roboticist
 *
 */
public class LBRArchmNode extends LBRBaseNode
{
	
	private ConnectedNode node = null;
	private String robotName = "kmriiwa";
	
	// ROS subscribers
	private Subscriber<kmriiwa_chemist_msgs.LBRCommand> lbrCmdSubscriber;
	private Subscriber<kmriiwa_chemist_msgs.TaskUpdate> taskUpdatesSubscriber;
	
	// ROS publishers
	// A publisher to request station jobs
	private Publisher<kmriiwa_chemist_msgs.TaskStatus> taskStatusPublisher;
	// KMR base state publisher
	private Publisher<kmriiwa_chemist_msgs.LBRStatus> lbrStatusPublisher;
	
	private kmriiwa_chemist_msgs.LBRCommand lbrCmd;
	private Boolean new_lbrCmd = new Boolean(false);
	
	private kmriiwa_chemist_msgs.TaskUpdate task_update;
	private Boolean new_taskUpdate = new Boolean(false);
	
	/***
	 * Constructs the node with the given robot name.
	 * @param robotName - the robot name to be used in the message headers.
	 */
	public LBRArchmNode(String robotName)
	{
		this.robotName = robotName;
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
				synchronized(new_lbrCmd)
				{
					lbrCmd = msg;
					new_lbrCmd = true;
				}
			}
		});
		
		taskUpdatesSubscriber = node.newSubscriber(robotName + "/lbr/task_updates", kmriiwa_chemist_msgs.TaskUpdate._TYPE);
		taskUpdatesSubscriber.addMessageListener( new MessageListener<kmriiwa_chemist_msgs.TaskUpdate>() {
			@Override
			public void onNewMessage(kmriiwa_chemist_msgs.TaskUpdate msg)
			{
				synchronized(new_taskUpdate)
				{
					task_update = msg;
					new_taskUpdate = true;
				}
			}
		});
		
		connectedToMaster = true;
	}
	
	/***
	 * Retrieve the latest LBRCommand received via the <em>[robot_name]/lbr/command</em> topic.
	 * @return latest LBRCommand message received on the <em>[robot_name]/lbr/command</em>. If the command was
	 * already retrieved, a null is returned.
	 */
	public kmriiwa_chemist_msgs.LBRCommand getLBRCommand()
	{
		synchronized (new_lbrCmd) 
		{
			if (new_lbrCmd) 
			{
				new_lbrCmd = false;
				return lbrCmd;
			}
			else 
			{
				return null;
			}
	    }
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
	public synchronized void publishTaskStatus(String taskName, int taskStatus) 
	{
		kmriiwa_chemist_msgs.TaskStatus taskMsg = node.getTopicMessageFactory().newFromType(kmriiwa_chemist_msgs.TaskStatus._TYPE);
		taskMsg.setTaskName(taskName);
		taskMsg.setTaskState(taskStatus);
    	taskStatusPublisher.publish(taskMsg);
    }
	
	/***
	 * Get the latest task update on the <em>[robot_name]/lbr/task_status</em> topic.
	 * @return the latest TaskUpdate received on the the <em>[robot_name]/lbr/task_status</em> topic.
	 * If the latest update was retrieved, a null is returned.
	 */
	public int getTaskUpdate()
	{
		synchronized (new_taskUpdate) 
		{
			if (new_taskUpdate) 
			{
				new_taskUpdate = false;
				return task_update.getTaskUpdate();
			}
			else 
			{
				return -1;
			}
	    }
	}
	
	/***
	 * A blocking method used to make the robot wait on a task update from an external station. This method
	 * is used to synchronise the robot operation with external stations when such mode of operation is needed.
	 * To achieve that the robot waits on the <em>[robot_name]/lbr/task_status</em> topic to receive the task update.
	 * @return true if the task status update commands the robot to continue its operations.
	 * @throws InterruptedException
	 */
	public boolean waitForTaskUpdate() throws InterruptedException
	{
		synchronized (new_taskUpdate)
		{
			while (!new_taskUpdate);
			{
				Thread.sleep(10);
			}
			return task_update.equals("continue");
		}
	}
}