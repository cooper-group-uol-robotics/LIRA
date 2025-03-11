package robotChemist.tasks;

import robotChemist.exceptions.UnexpectedCollisionDetected;
import robotChemist.interfaces.GenericTwoFingerGripper;
import robotChemist.interfaces.LBRiiwaArm;
import robotChemist.interfaces.GenericTwoFingerGripper.GripperPos;
import robotChemist.utility.Rack;
import com.kuka.roboticsAPI.applicationModel.IApplicationData;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.math.CoordinateAxis;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.task.ITaskLogger;

/***
 * A class that provides ease-of-use methods for racks pick and place tasks. It relies on the 
 * @{link Rack} object to define the rack location and its grasp information.
 * @author stoic-roboticist
 *
 */
public class RackHandling{
	private GenericTwoFingerGripper gripper;
	private LBRiiwaArm iiwaArm;
	private ITaskLogger logger;
	private IApplicationData appData;
	private double ptpRelVel;
	private double linCartVel;
	private int preGraspOffset;
	private double forceThreshold;
	private LBRiiwaArm.CollisionBehvaior collisionBehaviour;
	
	/***
	 * Constructs a RackHandling task object with the following default parameters:<br>
	 * - The relative speed of all the PTP movements is 20%.<br>
	 * - The Cartesian speed of all the LIN movements is 20 [mm/s].<br>
	 * - The rack pre-grasp offset is 50 [mm].<br>
	 * - The allowed force threshold for all force sensitive movements is 50 [N].<br><br>
	 * All these parameters can be altered using the appropriate set methods.
	 * @param gripper - the FestoGripper interface.
	 * @param arm - the LBR arm interface.
	 * @param appData - RoboticsAPI application data interface.
	 * @param logger - RoboticsAPI logging interface.
	 */
	public RackHandling(GenericTwoFingerGripper gripper, LBRiiwaArm arm, IApplicationData appData, ITaskLogger logger)
	{
		
		this.logger = logger;
		this.iiwaArm = arm;
		this.gripper = gripper;
		this.appData = appData;
		ptpRelVel = 0.2;
		linCartVel = 20;
		preGraspOffset = 50;
		forceThreshold = 50;
		collisionBehaviour = LBRiiwaArm.CollisionBehvaior.STOP_AT_COLLISION;
	}	
	
	/***
	 * Set the relative speed of all the PTP movements of this task to the given value.
	 * @param vel - new relative speed of all the PTP movements.
	 */
	public void setPtpVelocity(double vel){
		ptpRelVel = vel;
	}
	
	/***
	 * Sets the Cartesian speed of all the LIN movements of this task to the given value.
	 * @param cartVel - new Cartesian speed of all the LIN movements.
	 */
	public void setLinCartVelocity(double cartVel){
		linCartVel = cartVel;
	}
	
	/***
	 * Sets the allowed force threshold for all force sensitive movements of this task to the given value.
	 * @param threshold - new allowed force threshold for all force sensitive movements.
	 */
	public void setForceThreshold(double threshold){
		forceThreshold = threshold;
	}
	
	/***
	 * Sets the rack pre-grasp offset of this task to the given value.
	 * @param offset - new rack pre-grasp offset.
	 */
	public void setPreGraspOffset(int offset){
		preGraspOffset = offset;
	}
	
	public void setCollisionBehaviour(LBRiiwaArm.CollisionBehvaior behaviour)
	{
		collisionBehaviour = behaviour;
	}
	
	/***
	 * Instructs the robot to pick the given rack with the supplied initial gripper position.
	 * This method goes through the following steps to achieve that:<br>
	 * 1- Move the gripper fingers to the given initial position.<br>
	 * 2- Move to the rack pre-grasp frame using PTP motion.<br>
	 * 3- Move to the rack grasp frame using LIN motion.<br>
	 * 4- Grasp the rack by opening the gripper fingers till the openForceRack force is reached.<br>
	 * 5- Move the gripper back along the Z-axis half the distance towards the per-grasp position while operating in Impedance control mode. If the 
	 * allowed force threshold is exceeded the operation fails.<br>
	 * 6- Move to the rack pre-grasp frame using LIN motion.<br><br>
	 * 
	 * @param rack - the rack to be picked from the workspace.
	 * @param halfOpen - the gripper initial position as defined by the enum {@link GripperPos}.
	 * @throws UnexpectedCollisionDetected if a collision happen during any of the task's steps.
	 */
	public void pickupRack(Rack rack, GripperPos halfOpen) throws UnexpectedCollisionDetected
	{
		logger.info("Starting to pick up rack");
		
		// Pregrasp phase
		gripper.moveToPos(halfOpen);
		iiwaArm.moveToolPTP(getRackPreGraspPosition(rack.getGraspFrame()), "/spacer/tcp", ptpRelVel);
		
		//Grasp phase
		iiwaArm.moveToolLIN(rack.getGraspFrame(), "/spacer/tcp", linCartVel);
		gripper.invGraspWithForce((Integer) appData.getProcessData("openForceRack").getValue());
		
		//iiwaArm.switchTool(EETools.ATTACHED_RACK);
		
		// Grasp retreat phase
		// TODO fix the impedance motion as it seems it goes on a curve for some reason instead of going along
		// a straight line. Tried to change to LIN and tried changing the impedance control parameters and to no avail
		//int pickupStiffnessArray[] = {2000,2000,5000,150,300,300}; 
		//CartesianImpedanceControlMode impedanceConfig = iiwaArm.createCartesianImpedanceConfig(pickupStiffnessArray);
		//iiwaArm.moveLINRelWithImpedance(-1*(preGraspOffset/2), forceThreshold, linCartVel, CoordinateAxis.Z, impedanceConfig, LBRiiwaArm.CollisionBehvaior.FAIL_AT_COLLISION);
		iiwaArm.moveLINSensitively(getRackPreGraspPosition(rack.getGraspFrame()),"/spacer/tcp",forceThreshold,linCartVel,collisionBehaviour);
	}
	
	/***
	 * Instructs the robot to place the rack in the new target rack holder frame.
	 * This method goes through the following steps to achieve that:<br>
	 * 1- Move to the rack pre-place frame using PTP motion.<br>
	 * 2- Move the rack sensitively half the distance towards the place frame along the Z-axis such that the measured force doesn't exceed the allowed threshold.
	 * If that happens the method fails.<br>
	 * 3- Move the reminder of the distance to the place frame along the Z-axis in Cartesian Impedance mode.<br>
	 * 4- Open the gripper to the half open position.<br>
	 * 5- Move the arm back to the per-place frame using LIN motion.<br><br>
	 * @param rack - the rack to be placed in the workspace.
	 * @param targetRackHolderFrame - the target frame for placing the rack.
	 * @throws UnexpectedCollisionDetected if a collision happen during any of the task's steps.
	 */
	public void placeRack(Rack rack, ObjectFrame targetRackHolderFrame) throws UnexpectedCollisionDetected
	{
		logger.info("Starting to place rack");
		
		// attach the rack to the gripper
		
		// Pre-place phase
		iiwaArm.moveToolPTP(getRackPreGraspPosition(targetRackHolderFrame), "/spacer/tcp", ptpRelVel);
		iiwaArm.moveLINSensitively(targetRackHolderFrame,"/spacer/tcp", forceThreshold, linCartVel, collisionBehaviour);
		
		// Place phase
		// TODO fix the impedance motion as it seems it goes on a curve for some reason instead of going along
		// a straight line. Tried to change to LIN and tried changing the impedance control parameters and to no avail
		//int placeStiffnessArray[] = {5000,5000,1000,300,300,300};
		//CartesianImpedanceControlMode impedanceConfig = iiwaArm.createCartesianImpedanceConfig(placeStiffnessArray);
		//iiwaArm.moveLINRelWithImpedance(preGraspOffset/2, forceThreshold, linCartVel, CoordinateAxis.Z, impedanceConfig, LBRiiwaArm.CollisionBehvaior.STOP_AT_COLLISION);
		
		gripper.moveToPos(GripperPos.HALF_OPEN);
		
		//iiwaArm.switchTool(EETools.FESTO_GRIPPER);
		
		// Place retreat phase
		iiwaArm.moveToolLIN(getRackPreGraspPosition(targetRackHolderFrame), "/spacer/tcp", linCartVel);
		
		//de-attach the rack from the gripper and attach rack to station;
		
		rack.moveToFrame(targetRackHolderFrame);
	}
	
	private Frame getRackPreGraspPosition(ObjectFrame graspFrame)
	{
		Frame preGraspFrame = new Frame();
		preGraspFrame.setParent(graspFrame);
		preGraspFrame.setZ(preGraspFrame.getZ() - preGraspOffset);
		return preGraspFrame;
	}
}
