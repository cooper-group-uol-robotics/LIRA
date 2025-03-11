package robotChemist.tasks;

import com.kuka.roboticsAPI.applicationModel.IApplicationData;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.task.ITaskLogger;

import robotChemist.exceptions.GraspPositionException;
import robotChemist.interfaces.GenericTwoFingerGripper;
import robotChemist.interfaces.GenericTwoFingerGripper.GripperPos;
import robotChemist.interfaces.LBRiiwaArm;
import robotChemist.utility.Rack;

/***
 * A class that provides ease-of-use methods for vial pick and place tasks. It relies on the 
 * @{link Rack} object to define the vials location and their grasp information.
 * @author stoic-roboticist
 *
 */
public class VialHandling {
	private ITaskLogger logger;
	private IApplicationData appData;
	private GenericTwoFingerGripper gripper;
	private LBRiiwaArm iiwaArm;
	private double ptpRelVel;
	private double linCartVel;
	private double forceThreshold;
	private int preGraspOffset;
	
	/***
	 * Constructs a VialHandling task object with the following default parameters:<br>
	 * - The relative speed of all the PTP movements is 20%.<br>
	 * - The Cartesian speed of all the LIN movements is 20 [mm/s].<br>
	 * - The rack pre-grasp offset is 60 [mm].<br>
	 * - The allowed force threshold for all force sensitive movements is 20 [N].<br><br>
	 * All these parameters can be altered using the appropriate set methods.
	 * @param gripper - the Generic two finger gripper interface.
	 * @param arm - the LBR arm interface.
	 * @param appData - RoboticsAPI application data interface.
	 * @param logger - RoboticsAPI logging interface.
	 */
	public VialHandling(GenericTwoFingerGripper gripper, LBRiiwaArm iiwaArm, IApplicationData appData, ITaskLogger logger)
	{
		this.iiwaArm = iiwaArm;
		this.logger = logger;
		this.gripper = gripper;	
		this.appData = appData;
		ptpRelVel = 0.2;
		linCartVel = 20;
		preGraspOffset = 60;
		forceThreshold = 20;
	}
	
	/***
	 * Set the relative speed of all the PTP movements of this task to the given value.
	 * @param vel - new relative speed of all the PTP movements.
	 */
	public void setPtpVelocity(double vel)
	{
		ptpRelVel = vel;
	}
	
	/***
	 * Sets the Cartesian speed of all the LIN movements of this task to the given value.
	 * @param cartVel - new Cartesian speed of all the LIN movements.
	 */
	public void setLinCartVelocity(double cartVel)
	{
		linCartVel = cartVel;
	}
	
	/***
	 * Sets the allowed force threshold for all force sensitive movements of this task to the given value.
	 * @param threshold - new allowed force threshold for all force sensitive movements.
	 */
	public void setForceThreshold(double threshold)
	{
		forceThreshold = threshold;
	}
	
	/***
	 * Sets the rack pre-grasp offset of this task to the given value.
	 * @param offset - new rack pre-grasp offset.
	 */
	public void setPreGraspOffset(int offset)
	{
		preGraspOffset = offset;
	}
	
	/***
	 * Instructs the robot to pick the vial with the given index with the supplied initial gripper position.
	 * This method goes through the following steps to achieve that:<br>
	 * 1- Move the gripper fingers to the given initial position.<br>
	 * 2- Move to the vial pre-grasp frame using PTP motion.<br>
	 * 3- Move to the vial grasp frame using LIN motion.<br>
	 * 4- Grasp the vial with the closeForceVial force.<br>
	 * 5- Move to the vial pre-grasp frame using LIN motion.<br><br>
	 * 
	 * @param rack - the rack which holds the vial to be picked.
	 * @param vialIndex - the to be picked vial index.
	 * @param gripperInitPos - the gripper initial position as defined by the enum {@link GripperPos}.
	 */
	public void pickupVial(Rack rack, int vialIndex, GripperPos gripperInitPos)
	{
		try {
			logger.info("Starting to pick up vial " + String.valueOf(vialIndex)+  " from rack");
			gripper.moveToPos(gripperInitPos);
			logger.info("Moving to Pre-Grasp");
			iiwaArm.moveToolPTP(getVialPreGraspPosition(rack.getVialGraspFrame(vialIndex)), "/spacer/tcp", ptpRelVel);
			//Use Compliance to Ensure Grasp is solid?
			logger.info("Moving to Grasp");
			iiwaArm.moveToolLIN(rack.getVialGraspFrame(vialIndex), "/spacer/tcp", linCartVel);
			logger.info("Grasping");
			gripper.graspWithForce((Integer) appData.getProcessData("closeForceVial").getValue());
			iiwaArm.moveToolLIN(getVialPreGraspPosition(rack.getVialGraspFrame(vialIndex)), "/spacer/tcp", linCartVel);
		}
		catch (Exception e) {
			logger.error("Grasp Exception: " + e.getMessage());
		}
	}
	
	/***
	 * Instructs the robot to pick the vial at the given frame with the supplied initial gripper position.
	 * This method goes through the following steps to achieve that:<br>
	 * 1- Move the gripper fingers to the given initial position.<br>
	 * 2- Move to the vial pre-grasp frame using PTP motion.<br>
	 * 3- Move to the vial grasp frame using LIN motion.<br>
	 * 4- Grasp the vial with the closeForceVial force.<br>
	 * 5- Move to the vial pre-grasp frame using LIN motion.<br><br>
	 * 
	 * @param pickupFrame - the vial grasp frame. This string should include the full path to the target frame as described
	 * in the project frame tree.
	 * @param gripperInitPos - the gripper initial position as defined by the enum {@link GripperPos}.
	 */
	public void pickupVial(String pickupFrame, GripperPos gripperInitPos)
	{
		try 
		{
			ObjectFrame pickupFrameObj = appData.getFrame(pickupFrame);
			logger.info("Starting to pick up vial from frame " + pickupFrame);
			gripper.moveToPos(gripperInitPos);
			logger.info("Moving to Pre-Grasp");
			iiwaArm.moveToolPTP(getVialPreGraspPosition(pickupFrameObj), "/spacer/tcp", ptpRelVel);
			//Use Compliance to Ensure Grasp is solid?
			logger.info("Moving to Grasp");
			iiwaArm.moveToolLIN(pickupFrameObj, "/spacer/tcp", linCartVel);
			logger.info("Grasping");
			gripper.graspWithForce((Integer) appData.getProcessData("closeForceVial").getValue());
			iiwaArm.moveToolLIN(getVialPreGraspPosition(pickupFrameObj), "/spacer/tcp", linCartVel);
		}
		catch (Exception e) {
			logger.error("Grasp Exception: " + e.getMessage());
		}
	}
	
	/***
	 * Instructs the robot to place the vial at the specified rack well given by its index.
	 * This method goes through the following steps to achieve that:<br>
	 * 1- Move to the vial pre-place frame using PTP motion.<br>
	 * 2- Move to the vial place frame using LIN motion.<br>
	 * 3- Open the gripper to the half open position.<br>
	 * 4- Move the arm back to the per-place frame using LIN motion.<br><br>
	 * 
	 * @param rack - the rack where the vial will be placed.
	 * @param vialIndex - the index of the well where the vial will be placed in the rack.
	 */
	public void placeVial(Rack rack, int vialIndex){
		try {
			logger.info("Starting to place vial in slot " + String.valueOf(vialIndex) +  " in rack");
			iiwaArm.moveToolPTP(getVialPreGraspPosition(rack.getVialGraspFrame(vialIndex)), "/spacer/tcp", ptpRelVel);
			//Use Compliance to Ensure Grasp is solid?
			iiwaArm.moveToolLIN(rack.getVialGraspFrame(vialIndex), "/spacer/tcp", linCartVel);
			gripper.moveToPos(GripperPos.HALF_OPEN);
			iiwaArm.moveToolLIN(getVialPreGraspPosition(rack.getVialGraspFrame(vialIndex)), "/spacer/tcp", linCartVel);}
		catch (Exception e) {
			logger.error("Grasp Exception: " + e.getMessage());
		}
	}
	
	/***
	 * Instructs the robot to place the vial at the specified frame.
	 * This method goes through the following steps to achieve that:<br>
	 * 1- Move to the vial pre-place frame using PTP motion.<br>
	 * 2- Move to the vial place frame using LIN motion.<br>
	 * 3- Open the gripper to the half open position.<br>
	 * 4- Move the arm back to the per-place frame using LIN motion.<br><br>
	 * 
	 * @param placeFrame - the vial place frame. This string should include the full path to the target frame as described
	 * in the project frame tree.
	 */
	public void placeVial(String placeFrame){
		try 
		{
			ObjectFrame placeFrameObj = appData.getFrame(placeFrame);
			logger.info("Starting to place vial from frame "  + placeFrame);
			iiwaArm.moveToolPTP(getVialPreGraspPosition(placeFrameObj), "/spacer/tcp", ptpRelVel);
			//Use Compliance to Ensure Grasp is solid?
			iiwaArm.moveToolLIN(placeFrame, "/spacer/tcp", linCartVel);
			gripper.moveToPos(GripperPos.HALF_OPEN);
			iiwaArm.moveToolLIN(getVialPreGraspPosition(placeFrameObj), "/spacer/tcp", linCartVel);}
		catch (Exception e) {
			logger.error("Grasp Exception: " + e.getMessage());
		}
	}
	
	private Frame getVialPreGraspPosition(ObjectFrame vialGraspFrame) 
	{
		Frame preGraspFrame = new Frame();;
		preGraspFrame.setParent(vialGraspFrame);
		preGraspFrame.setZ(preGraspFrame.getZ() - preGraspOffset);
		return preGraspFrame;
	}
}
