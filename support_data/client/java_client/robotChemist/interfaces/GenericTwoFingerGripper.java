package robotChemist.interfaces;

import com.kuka.task.ITaskLogger;

/***
 * This class represents a generic Gripper. It provides different methods
 * to command the gripper and query its status.
 * @author stoic-roboticist
 *
 */
public abstract class GenericTwoFingerGripper {
	
	protected ITaskLogger logger;
	
	/***
	 * An enumeration that describes the available gripper position.
	 * These include OPEN, HALF_OPEN and CLOSED
	 * @author stoic-roboticist
	 *
	 */
	public enum GripperPos{
		OPEN(2, "Open"),
		HALF_OPEN(3, "Half open"),
		CLOSE(1, "Close");
		
		public final int value;
		public final String name;
		private GripperPos(int pos, String posName){
			this.value = pos;
			this.name = posName;
		}
	}
	
	/***
	 * Commands the gripper to fully closed position.
	 */
	public abstract void close();
	
	/***
	 * Commands the gripper to the fully open position.
	 */
	public abstract void open();
	
	/***
	 * Commands the gripper to the desired position.
	 * @param int - desired gripper position.
	 */
	//public abstract void moveToPos(int gripperPos);
	
	/***
	 * Commands the gripper to the desired position.
	 * @param GripperPos - desired gripper position. This position is defined using the {@link GripperPos} enumeration.
	 */
	public abstract void moveToPos(GripperPos gripperPos);
	
	/***
	 * Commands the gripper to grasp an object by moving its fingers to the fully close position
	 * till the desired grasp force is achieved. That is the fingers move inwards till the desired force is reached.
	 * @param force - desired grasp force described as a percentage of the maximum grasp force [0 - 100].
	 */
	public abstract void graspWithForce(int force);
	
	/***
	 * Commands the gripper to grasp an object by moving its fingers to the fully open position
	 * till the desired grasp force is achieved. That is the fingers move outwards till the desired force is reached.
	 * @param force - desired grasp force described as a percentage of the maximum grasp force [0 - (-100)].
	 */
	public abstract void invGraspWithForce(int force);
	
	/***
	 * Queries the gripper for its current grasp width.
	 * @return the current grasp width in millimetres [mm].
	 */
	public abstract int getCurrentWidth();

}
