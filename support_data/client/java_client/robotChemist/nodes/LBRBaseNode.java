package robotChemist.nodes;

import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;

/***
 * A base class that all other nodes to be used in RobotChemist application need to derive from.
 * @author stoic-roboticist
 *
 */
public class LBRBaseNode extends AbstractNodeMain{

	protected boolean connectedToMaster = false;
	
	@Override
	public GraphName getDefaultNodeName() {
		// TODO Auto-generated method stub
		return null;
	}
	
	/***
	 * 
	 * @return true if the node is connected to ROS master.
	 */
	public boolean isConnectedToMaster()
	{
		return connectedToMaster;
	}

}
