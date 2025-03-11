package robotChemist.interfaces;

import java.util.Date;

import robotChemist.exceptions.UnexpectedCollisionDetected;
import robotChemist.utility.LBRStatus;

import com.kuka.task.ITaskLogger;
import com.kuka.roboticsAPI.applicationModel.IApplicationData;
import com.kuka.roboticsAPI.conditionModel.ForceCondition;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.controllerModel.sunrise.ISunriseRequestService;
import com.kuka.roboticsAPI.controllerModel.sunrise.api.SSR;
import com.kuka.roboticsAPI.controllerModel.sunrise.api.SSRFactory;
import com.kuka.roboticsAPI.controllerModel.sunrise.connectionLib.Message;
import com.kuka.roboticsAPI.controllerModel.sunrise.positionMastering.PositionMastering;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.deviceModel.LBRAlphaRedundancy;
import com.kuka.roboticsAPI.geometricModel.AbstractFrame;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.Workpiece;
import com.kuka.roboticsAPI.geometricModel.World;
import com.kuka.roboticsAPI.geometricModel.math.CoordinateAxis;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.PTP;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.persistenceModel.PersistenceException;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.lin;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.linRel;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptpHome;

/**
 * A class that represent the LBR-iiwa arm. It provides methods to command the arm in different control modes 
 * using taught frames.
 * @author stoic-roboticist
 *
 */
public class LBRiiwaArm 
{
	
	private ITaskLogger logger;
	private Tool gripperTool;
	private Tool currentTool;
	private LBR lbr;
	private Controller kukaCont;
	private IApplicationData appData;
	
	/**
	 * An enumeration that describes the tool attached
	 * to the end effector.
	 * @author stoic-roboticist
	 *
	 */
	public enum EETools
	{
		FESTO_GRIPPER("Festo Gripper"),
		ATTACHED_RACK("Rack");
		
		public final String name;
		
		private EETools(String name)
		{
			this.name = name;
		}
	}
	
	/**
	 * An enumeration that describes the expected
	 * collision behaviour when commanding the robot in force
	 * Sensitive modes.
	 * 
	 * @author stoic-roboticist
	 *
	 */
	public enum CollisionBehvaior
	{
		STOP_AT_COLLISION,
		FAIL_AT_COLLISION
	}
	
	/**
	 * Constructs the LBRiiwaArm object
	 * @param lbr - RoboticsAPI LBRiiwa interface. 
	 * @param gripperTool - the geometric tool representing the attached gripper to the arm flange.
	 * @param emptyRackTool - the geometric tool representing the attached rack to the gripper.
	 * @param appData - RoboticsAPI application data interface.
	 * @param kukaConts - RoboticsAPI controller interface.
	 * @param logger - RoboticsAPI logging interface.
	 */
	public LBRiiwaArm(LBR lbr, Tool gripperTool, IApplicationData appData, Controller kukaCont, ITaskLogger logger)
	{
		this.logger = logger;
		this.kukaCont = kukaCont;
		this.lbr = lbr;
		this.appData = appData;
		this.gripperTool = gripperTool;
		this.currentTool = gripperTool;
		
	}
	
	/***
	 * Asserts that the arm is in the drive position. If not the arm is moved to that
	 * position.
	 */
	public void assertArmInDrivePos()
	{
		double distance = lbr.getCurrentCartesianPosition(lbr.getFlange()).distanceTo(appData.getFrame("/DrivePos"));
		if (distance > 20){
			logger.info("Robot not in Drive Position, moving");
			moveArmToDrivePos(0.3);
		} else {
			logger.info("Robot already in Drive Position");
		}
		
	}
	
	/***
	 * Commands the arm to the drive position.
	 * @param relvel - the desired execution velocity described as a percentage of the maximum ptp velocity [0 - 1]. This varies
	 * between the different robot operation modes.
	 */
	public void moveArmToDrivePos(double relvel)
	{
		moveToolPTP("/DrivePos", lbr.getFlange(), relvel);
	}
	
	/***
	 * Commands the arm to the rack position.
	 * @param relvel - the desired execution velocity described as a percentage of the maximum ptp velocity [0 - 1]. This varies
	 * between the different robot operation modes.
	 */
	public void moveArmToRackPos(double relvel)
	{
		moveToolPTP("/RackPos", "/spacer/tcp", relvel);
	}
	
	/***
	 * Commands the arm to the neutral position.
	 * @param relvel - the desired execution velocity described as a percentage of the maximum ptp velocity [0 - 1]. This varies
	 * between the different robot operation modes.
	 */
	public void moveArmToNeutralPos(double relvel)
	{
		moveToolPTP("/NeutralPos", "/spacer/tcp", relvel);
	}
	
	/***
	 * Commands the arm to move the given tool frame to the requested target frame using point-to-point (ptp) motion.
	 * @param targetFrame - the requested target frame. This string should include the full path to the target frame as described
	 * in the project frame tree. 
	 * @param toolFrame - the tool frame used in this move, e.g., "/spacer/tcp"
	 * @param relVel - the desired execution velocity described as a percentage of the maximum ptp velocity [0 - 1]. This varies
	 * between the different robot operation modes.
	 * @throws PersistenceException if any of the given frame names don't exist in the application data
	 * @throws IllegalStateException if the arm cannot move to the desired target frame.
	 */
	public void moveToolPTP(String targetFrame, String toolFrame, double relVel) throws PersistenceException, IllegalStateException 
	{
		try {
//			logger.info(String.format("Moving PTP to %s",  targetFrame));
			ObjectFrame goalFrame = appData.getFrame(targetFrame);
			moveToolPTP(goalFrame, toolFrame, relVel);
		}
		catch (PersistenceException e){
			logger.error("Target Frame Not Found: " + targetFrame);
			throw e;
		}
	}
	
	/***
	 * Commands the arm to move the given tool frame to the requested target frame using point-to-point (ptp) motion.
	 * @param targetFrame - the requested target frame using RoboticsAPI frame object.
	 * @param toolFrame - the tool frame used in this move, e.g., "/spacer/tcp"
	 * @param relVel - the desired execution velocity described as a percentage of the maximum ptp velocity [0 - 1]. This varies
	 * between the different robot operation modes.
	 * @throws PersistenceException if any of the given frame names don't exist in the application data
	 * @throws IllegalStateException if the arm cannot move to the desired target frame.
	 */
	public void moveToolPTP(AbstractFrame targetFrame, String toolFrame, double relVel) throws PersistenceException, IllegalStateException
	{
		ObjectFrame startFrame = currentTool.getFrame(toolFrame);
		try {
			moveToolPTP(targetFrame, startFrame, relVel);
		}
		catch (IllegalStateException e){
			if (startFrame == null) {
				logger.error("Tool Frame Not Found");
			}
			else {
				logger.error(e.getMessage());
			}
			throw e;
		}
	}
	
	private void moveToolPTP(String targetFrame, ObjectFrame toolFrame, double relVel) throws PersistenceException, IllegalStateException 
	{
		try {
			logger.info(String.format("Moving PTP to %s",  targetFrame));
			ObjectFrame goalFrame = appData.getFrame(targetFrame);
			moveToolPTP(goalFrame, toolFrame, relVel);
		}
		catch (PersistenceException e){
			logger.error("Target Frame Not Found: " + targetFrame);
			throw e;
		}
	}
	
	private void moveToolPTP(AbstractFrame targetFrame, ObjectFrame toolFrame, double relVel) throws PersistenceException, IllegalStateException 
	{
		try {
			toolFrame.move(ptp(targetFrame).setJointVelocityRel(relVel));
		}
		catch (IllegalStateException e){
			if (toolFrame == null | targetFrame == null) {
				logger.error("Tool Frame Not Found");
			}
			else {
				logger.error(e.getMessage());
			}
			throw e;
		}
	}
	
	/***
	 * Commands the arm to move the given tool frame to the requested target frame using Cartesian/linear motion.
	 * @param targetFrame - the requested target frame. This string should include the full path to the target frame as described
	 * in the project frame tree.
	 * @param toolFrame - the tool frame used in this move, e.g., "/spacer/tcp".
	 * @param cartVel - the desired Cartesian execution velocity in millimetres per second [mm/s].
	 * @throws PersistenceException if any of the given frame names don't exist in the application data
	 * @throws IllegalStateException if the arm cannot move to the desired target frame.
	 */
	public void moveToolLIN(String targetFrame, String toolFrame, double cartVel) throws PersistenceException, IllegalStateException 
	{
		try {
//			logger.info(String.format("Moving LIN to %s",  targetFrame));
			ObjectFrame goalFrame = appData.getFrame(targetFrame);
			moveToolLIN(goalFrame, toolFrame, cartVel);
		}
		catch (PersistenceException e){
			logger.error("Target Frame Not Found: " + targetFrame);
			throw e;
		}
	}
	
	/***
	 * Commands the arm to move the given tool frame to the requested target frame using Cartesian/linear motion.
	 * @param targetFrame - the requested target frame using RoboticsAPI frame object.
	 * @param toolFrame - the tool frame used in this move, e.g., "/spacer/tcp"
	 * @param cartVel - the desired Cartesian execution velocity in millimetres per second [mm/s].
	 * @throws PersistenceException if any of the given frame names don't exist in the application data
	 * @throws IllegalStateException if the arm cannot move to the desired target frame.
	 */
	public void moveToolLIN(AbstractFrame targetFrame, String toolFrame, double cartVel) throws PersistenceException, IllegalStateException 
	{
		ObjectFrame startFrame = currentTool.getFrame(toolFrame);
		try {
			moveToolLIN(targetFrame, startFrame, cartVel);
		}
		catch (IllegalStateException e){
			if (startFrame == null) {
				logger.error("Tool Frame Not Found");
			}
			else {
				logger.error(e.getMessage());
			}
			throw e;
		}
	}
	
	private void moveToolLIN(AbstractFrame targetFrame, ObjectFrame toolFrame, double cartVel) throws PersistenceException, IllegalStateException 
	{
		try {
			toolFrame.move(lin(targetFrame).setCartVelocity(cartVel));
		}
		catch (IllegalStateException e){
			if (toolFrame == null | targetFrame == null) {
				logger.error("Tool Frame Not Found");
			}
			else {
				logger.error(e.getMessage());
			}
			throw e;
		}
	}
	
//	public void attachWorkpiece(Workpiece object)
//	{
//		object.attachTo(gripperTool.getDefaultMotionFrame());
//		currentTool = object; change currenTool to PhysicalObject
//		
//		lbr.detachAll();
//		switch (tool){
//			case FESTO_GRIPPER:
//			{
//				gripperTool.attachTo(lbr.getFlange());
//				currentTool = gripperTool;
//				break;
//			}
//			case ATTACHED_RACK:
//			{
//				emptyRackTool.attachTo(lbr.getFlange());
//				currentTool = emptyRackTool;
//				break;
//			}
//			default:
//			{
//				logger.error("Provided end-effector tool not recognised. Unable to attach tool to flange.");
//			}
//			logger.info(String.format("End-effector tool switched. %s attached to the LBR flange", tool.name));
//		}
//	}
	
	/***
	 * Creates a Cartesian impedance control configuration for the end effector using the given stiffness and damping arrays.
	 * @param stiffnessArray - the desired impedance stiffness array of the end effector. This array has the following form
	 * [k_x, k_y, k_z, k_rot_a, k_rot_b, k_rot_c], where k is the linear stiffness in [N/m] and k_rot is rotational stiffness in Nm/rad
	 * along the specified axis
	 * @param dampingArray - the desired impedance damping array of the end effector. This array has the following form
	 * [d_x, d_y, d_z, d_rot_a, d_rot_b, d_rot_c], where d and d_rot are the linear and rotation damping ratios respectively.
	 * @return The desired {@link CartesianImpedanceControlMode} configuration to be used in other motion commands.
	 */
	public CartesianImpedanceControlMode createCartesianImpedanceConfig(int[] stiffnessArray, double[] dampingArray)
	{
		CartesianImpedanceControlMode controlMode = new CartesianImpedanceControlMode();
		if (stiffnessArray.length == 6)
		{
			controlMode.parametrize(CartDOF.X).setStiffness(stiffnessArray[0]);
			controlMode.parametrize(CartDOF.Y).setStiffness(stiffnessArray[1]);
			controlMode.parametrize(CartDOF.Z).setStiffness(stiffnessArray[2]);
			controlMode.parametrize(CartDOF.A).setStiffness(stiffnessArray[3]);
			controlMode.parametrize(CartDOF.B).setStiffness(stiffnessArray[4]);
			controlMode.parametrize(CartDOF.C).setStiffness(stiffnessArray[5]);
		}
		else
		{
			logger.warn("Stiffness array is not complete. Cannot create impedance control config");
			return null;
		}
		if (dampingArray.length == 6)
		{
			controlMode.parametrize(CartDOF.X).setDamping(dampingArray[0]);
			controlMode.parametrize(CartDOF.Y).setDamping(dampingArray[1]);
			controlMode.parametrize(CartDOF.Z).setDamping(dampingArray[2]);
			controlMode.parametrize(CartDOF.A).setDamping(dampingArray[3]);
			controlMode.parametrize(CartDOF.B).setDamping(dampingArray[4]);
			controlMode.parametrize(CartDOF.C).setDamping(dampingArray[5]);
		}
		else
		{
			logger.warn("Damping array is not complete. Cannot create impedance control config");
			return null;
		}
		return controlMode;
	}
	
	/***
	 * Creates a Cartesian impedance control configuration for the end effector using the given stiffness array.
	 * Default values for damping array are used. Refer to {@link CartesianImpedanceControlMode} for these values.
	 * @param stiffnessArray - the desired impedance stiffness array of the end effector. This array has the following form
	 * [k_x, k_y, k_z, k_rot_a, k_rot_b, k_rot_c], where k is the linear stiffness in [N/m] and k_rot is rotational stiffness in Nm/rad
	 * along the specified axis
	 * @return The desired {@link CartesianImpedanceControlMode} configuration to be used in other motion commands.
	 */
	public CartesianImpedanceControlMode createCartesianImpedanceConfig(int[] stiffnessArray)
	{
		CartesianImpedanceControlMode controlMode = new CartesianImpedanceControlMode();
		if (stiffnessArray.length == 6)
		{
			controlMode.parametrize(CartDOF.X).setStiffness(stiffnessArray[0]);
			controlMode.parametrize(CartDOF.Y).setStiffness(stiffnessArray[1]);
			controlMode.parametrize(CartDOF.Z).setStiffness(stiffnessArray[2]);
			controlMode.parametrize(CartDOF.A).setStiffness(stiffnessArray[3]);
			controlMode.parametrize(CartDOF.B).setStiffness(stiffnessArray[4]);
			controlMode.parametrize(CartDOF.C).setStiffness(stiffnessArray[5]);
		}
		else
		{
			logger.warn("Stiffness array is not complete. Cannot create impedance control config");
			return null;
		}
		return controlMode;
	}
	
	/***
	 * Commands the arm to move the given tool frame to the requested target frame using Cartesian/linear motion while monitoring spatial force measurements.
	 * The arm would stop moving if the measured force exceeds the given threshold. This command would throw an exception if
	 * {@link CollisionBehvaior.FAIL_AT_COLLISION} is specified. Otherwise, it would just stop.
	 * @param targetFrame - the requested target frame. This string should include the full path to the target frame as described
	 * in the project frame tree.
	 * @param toolFrame - the tool frame used in this move, e.g., "/spacer/tcp".
	 * @param forceThreshold - the measured spatial force threshold that would cause the robot to stop in Newton [N].
	 * @param cartesianVel - the desired Cartesian execution velocity in millimetres per second [mm/s].
	 * @param collisionBehavior - the desired behaviour when the spatial force threshold is exceeded. 
	 * @throws UnexpectedCollisionDetected if the spatial force threshold is exceeded when {@link CollisionBehvaior.FAIL_AT_COLLISION} is specified.
	 */
	public void moveLINSensitively(AbstractFrame targetFrame, String toolFrame , double forceThreshold, double cartesianVel, CollisionBehvaior collisionBehavior) throws UnexpectedCollisionDetected
	{	
		ForceCondition collisionCond = ForceCondition.createSpatialForceCondition(currentTool.getFrame(toolFrame), forceThreshold);
		
		IMotionContainer motionContainer = currentTool.getFrame(toolFrame).move(lin(targetFrame).setCartVelocity(cartesianVel).breakWhen(collisionCond));
		
		if (motionContainer.hasFired(collisionCond) && collisionBehavior == CollisionBehvaior.FAIL_AT_COLLISION)
		{
			throw new UnexpectedCollisionDetected("Collision occured!!!"); 
		}
	}
	
	/***
	 * Commands the arm to move the given tool frame to the requested target frame using Cartesian/linear motion in Cartesian impedance control mode.
	 * This make the robot complaint when executing the motion.
	 * @param targetFrame - the requested target frame. This string should include the full path to the target frame as described
	 * in the project frame tree.
	 * @param toolFrame - the tool frame used in this move, e.g., "/spacer/tcp".
	 * @param cartVel - the desired Cartesian execution velocity in millimetres per second [mm/s].
	 * @param impedanceConfig - the desired end-effector Cartesian impedance control configuration.
	 * @throws UnexpectedCollisionDetected If a collision is encountered that exceeds the force limits allowed by the Cartesian impedance control mode.
	 */
	public void moveLINWithImpedance(String targetFrame, String toolFrame, double cartVel, CartesianImpedanceControlMode impedanceConfig) throws UnexpectedCollisionDetected
	{
		ObjectFrame goalFrame = appData.getFrame(targetFrame);
		currentTool.getFrame(toolFrame).move(lin(goalFrame).setMode(impedanceConfig).setCartVelocity(cartVel));
	}
	
	/***
	 * Commands the arm to move the given tool frame to the requested target frame using point-to-point (ptp) motion in Cartesian impedance control mode.
	 * This make the robot complaint when executing the motion.
	 * @param targetFrame - the requested target frame. This string should include the full path to the target frame as described
	 * in the project frame tree.
	 * @param toolFrame - the tool frame used in this move, e.g., "/spacer/tcp".
	 * @param relVel - the desired execution velocity described as a percentage of the maximum ptp velocity [0 - 1]. This varies
	 * between the different robot operation modes.
	 * @param impedanceConfig - the desired end-effector Cartesian impedance control configuration.
	 * @throws UnexpectedCollisionDetected If a collision is encountered that exceeds the force limits allowed by the Cartesian impedance control mode.
	 */
	public void movePTPWithImpedance(String targetFrame, String toolFrame, double relVel, CartesianImpedanceControlMode impedanceConfig) throws UnexpectedCollisionDetected
	{
		ObjectFrame goalFrame = appData.getFrame(targetFrame);
		currentTool.getFrame(toolFrame).move(ptp(goalFrame).setMode(impedanceConfig).setJointVelocityRel(relVel));
	}
	
	/***
	 * Commands the arm to move in the specified direction by the given displacement while monitoring force measurements in that direction.
	 * The arm would stop moving if the measured force exceeds the given threshold. This command would throw an exception if
	 * {@link CollisionBehvaior.FAIL_AT_COLLISION} is specified. Otherwise, it would just stop.
	 * @param distance - the displacement the arm would move in millimetres [mm]. Negative values indicate motion along the negative axis.
	 * @param forceThreshold - the measured force threshold in the given direction that would cause the robot to stop in Newton [N]. 
	 * @param cartesianVel - the desired Cartesian execution velocity in millimetres per second [mm/s].
	 * @param axis - the desired axis that the end-effector would move along.
	 * @param collisionBehavior - the desired behaviour when the force threshold is exceeded. 
	 * @throws UnexpectedCollisionDetected if the force threshold along the given axis is exceeded when {@link CollisionBehvaior.FAIL_AT_COLLISION} is specified.
	 */
	public void moveLINRelSensitively(double distance, double forceThreshold, double cartesianVel, CoordinateAxis axis, CollisionBehvaior collisionBehavior) throws UnexpectedCollisionDetected
	{
		double distX = 0, distY = 0, distZ = 0;
		
		switch(axis)
		{
			case X:
				distX = distance;
				break;
			case Y:
				distY = distance;
				break;
			case Z:
				distZ = distance;;
				break;
			default:
				logger.error("Incorrect Axis");
		}
		
//		ForceCondition detectCollision_Z = ForceCondition.createNormalForceCondition(geoGripperTool.getDefaultMotionFrame(), CoordinateAxis.Z, 20);
		ForceCondition collisionCond = ForceCondition.createNormalForceCondition(currentTool.getDefaultMotionFrame(), axis, forceThreshold);
//		
//		IMotionContainer motionContainer = currentTool.getDefaultMotionFrame().move(linRel(distX,distY,distZ).setCartVelocity(cartesianVel).breakWhen(collisionCond));
		
//		currentTool.move(linRel(distX, distY, distZ, currentTool.getDefaultMotionFrame()).setCartVelocity(50));
		currentTool.move(linRel(distX, distY, distZ, currentTool.getDefaultMotionFrame()).setCartVelocity(25).breakWhen(collisionCond));
		
	}

	public void moveLINRelForce(double distance, double forceThreshold, double cartesianVel, CoordinateAxis axis)
	{
		double distX = 0, distY = 0, distZ = 0;
		
		switch(axis)
		{
			case X:
				distX = distance;
				break;
			case Y:
				distY = distance;
				break;
			case Z:
				distZ = distance;;
				break;
			default:
				logger.error("Incorrect Axis");
		}
		
//		ForceCondition detectCollision_Z = ForceCondition.createNormalForceCondition(geoGripperTool.getDefaultMotionFrame(), CoordinateAxis.Z, 20);
		ForceCondition collisionCond = ForceCondition.createNormalForceCondition(currentTool.getDefaultMotionFrame(), axis, forceThreshold);
//		
//		IMotionContainer motionContainer = currentTool.getDefaultMotionFrame().move(linRel(distX,distY,distZ).setCartVelocity(cartesianVel).breakWhen(collisionCond));
		
//		currentTool.move(linRel(distX, distY, distZ, currentTool.getDefaultMotionFrame()).setCartVelocity(50));
		currentTool.move(linRel(distX, distY, distZ, currentTool.getDefaultMotionFrame()).setCartVelocity(25).breakWhen(collisionCond));
		
	}
	
	/***
	 * Commands the arm to move in the specified direction by the given displacement while monitoring force measurements in that direction in Cartesian impedance control mode.
	 * The arm is complaint in this mode however it would stop moving if the measured force exceeds the given threshold. This command would throw an exception if
	 * {@link CollisionBehvaior.FAIL_AT_COLLISION} is specified. Otherwise, it would just stop.
	 * @param distance - the displacement the arm would move in millimetres [mm]. Negative values indicate motion along the negative axis.
	 * @param forceThreshold - the measured force threshold in the given direction that would cause the robot to stop in Newton [N].
	 * @param cartesianVel - the desired Cartesian execution velocity in millimetres per second [mm/s].
	 * @param axis - the desired axis that the end-effector would move along.
	 * @param impedanceConfig - the desired end-effector Cartesian impedance control configuration.
	 * @param collisionBehavior - the desired behaviour when the force threshold is exceeded. 
	 * @throws UnexpectedCollisionDetected if the force threshold along the given axis is exceeded when {@link CollisionBehvaior.FAIL_AT_COLLISION} is specified; or 
	 * a collision is encountered that exceeds the force limits allowed by the Cartesian impedance control mode.
	 */
	public void moveLINRelWithImpedance(double distance, double forceThreshold, double cartesianVel, CoordinateAxis axis, CartesianImpedanceControlMode impedanceConfig, CollisionBehvaior collisionBehavior) throws UnexpectedCollisionDetected
	{
		double distX = 0, distY = 0, distZ = 0;
		
		switch(axis)
		{
			case X:
				distX = distance;
				break;
			case Y:
				distY = distance;
				break;
			case Z:
				distZ = distance;;
				break;
			default:
				logger.error("Incorrect Axis");
		}
		
		ForceCondition collisionCond = ForceCondition.createNormalForceCondition(currentTool.getDefaultMotionFrame(), axis, forceThreshold);
		IMotionContainer motionContainer = currentTool.getDefaultMotionFrame().move(linRel(distX,distY,distZ).setMode(impedanceConfig).setCartVelocity(cartesianVel).breakWhen(collisionCond));
		
		if (motionContainer.hasFired(collisionCond) && collisionBehavior == CollisionBehvaior.FAIL_AT_COLLISION)
		{
			throw new UnexpectedCollisionDetected("Collision occured!!!");
		}
	}
	
	/***
	 * Commands the arm to move in the specified direction by the given displacement in that direction in Cartesian impedance control mode.
	 * This make the robot complaint when executing the motion
	 * @param distance - the displacement the arm would move in millimetres [mm]. Negative values indicate motion along the negative axis.
	 * @param cartesianVel - the desired Cartesian execution velocity in millimetres per second [mm/s].
	 * @param axis - the desired axis that the end-effector would move along.
	 * @param impedanceConfig - the desired end-effector Cartesian impedance control configuration.
	 * @throws UnexpectedCollisionDetected if a collision is encountered that exceeds the force limits allowed by the Cartesian impedance control mode.
	 */
	public void moveLINRelWithImpedance(double distance, double cartesianVel, CoordinateAxis axis, CartesianImpedanceControlMode impedanceConfig) throws UnexpectedCollisionDetected
	{
		double distX = 0, distY = 0, distZ = 0;
		
		switch(axis)
		{
			case X:
				distX = distance;
				break;
			case Y:
				distY = distance;
				break;
			case Z:
				distZ = distance;;
				break;
			default:
				logger.error("Incorrect Axis");
		}
		currentTool.getDefaultMotionFrame().move(linRel(distX,distY,distZ).setMode(impedanceConfig).setCartVelocity(cartesianVel));
	}
	
	private void performCalibrationMotion(JointPosition position) {
		int positionCounter = 0;
		logger.info("Moving to position #" + (++positionCounter));

		PTP mainMotion = new PTP(position).setJointVelocityRel(0.2);
		lbr.move(mainMotion);

		logger.info("Moving to current position from negative direction");
		JointPosition position1 = new JointPosition(lbr.getJointCount());
		for (int i = 0; i < lbr.getJointCount(); ++i) 
		{
			position1.set(i, position.get(i) - Math.toRadians(5));
		}
		PTP motion1 = new PTP(position1).setJointVelocityRel(0.2);
		lbr.move(motion1);
		lbr.move(mainMotion);

		// Send the command to safety to trigger the measurement
		sendSafetyCommand();

		logger.info("Moving to current position from positive direction");
		JointPosition position2 = new JointPosition(lbr.getJointCount());
		for (int i = 0; i < lbr.getJointCount(); ++i)
		{
			position2.set(i, position.get(i) + Math.toRadians(5));
		}
		PTP motion2 = new PTP(position2).setJointVelocityRel(0.2);
		lbr.move(motion2);
		lbr.move(mainMotion);

		// Send the command to safety to trigger the measurement
		sendSafetyCommand();
	}
	
	private void sendSafetyCommand() {
		int COMMAND_SUCCESSFUL = 1;
		int GMS_REFERENCING_COMMAND = 2;
		ISunriseRequestService requestService = (ISunriseRequestService) (kukaCont.getRequestService());
		SSR ssr = SSRFactory.createSafetyCommandSSR(GMS_REFERENCING_COMMAND);
		Message response = requestService.sendSynchronousSSR(ssr);
		int result = response.getParamInt(0);
		if (COMMAND_SUCCESSFUL != result) {
			logger.warn(
					"Command did not execute successfully, response = "
							+ result);
		}
	}
	
	/***
	 * Gets the LBRiiwa arm status
	 * @return the arm status {@link LBRStatus}
	 */
	public LBRStatus getStatus()
	{
		return new LBRStatus(lbr.isMotionEnabled(), lbr.getSafetyState().areAllAxesGMSReferenced(), lbr.getSafetyState().areAllAxesPositionReferenced(), lbr.getSafetyState().getSafetyStopSignal());
	}
	
	/***
	 * Performs position and GMS referencing with 5 positions. 
	 * This method needs to be performed to use the arm in AUT mode.
	 */
	public void Reference(){
		int axisId[] = { 0, 1, 2, 3, 4, 5, 6 };
		
		PositionMastering mastering = new PositionMastering(lbr);
		boolean allAxesMastered = true;
		boolean[] isMastered = mastering.getMasteringStates();
		for (int i = 0; i < axisId.length; ++i) 
		{
			// Check if the axis is mastered - if not, no referencing is
			// possible
			
			if (!isMastered[i]) 
			{
				logger.warn("Axis with axisId "+ axisId[i] + " is not mastered, therefore it cannot be referenced");
			}

			allAxesMastered &= isMastered[i];
		}

		if (allAxesMastered) 
		{
			logger.info(
					"Perform position and GMS referencing with 5 positions");

			// Move to home position
			logger.info("Moving to home position");

			lbr.move(ptpHome().setJointVelocityRel(0.2));

			// In this example 5 positions are defined, though each one
			// will be reached from negative and from positive axis
			// direction resulting 10 measurements. The safety needs
			// exactly 10 measurements to perform the referencing.
			performCalibrationMotion(new JointPosition(Math.toRadians(0.0),
					Math.toRadians(16.18), Math.toRadians(23.04),
					Math.toRadians(37.35), Math.toRadians(-67.93),
					Math.toRadians(38.14), Math.toRadians(-2.13)));

			performCalibrationMotion(new JointPosition(Math.toRadians(18.51),
					Math.toRadians(9.08), Math.toRadians(-1.90),
					Math.toRadians(49.58), Math.toRadians(-2.92),
					Math.toRadians(18.60), Math.toRadians(-31.18)));

			performCalibrationMotion(new JointPosition(Math.toRadians(-18.53),
					Math.toRadians(-25.76), Math.toRadians(-47.03),
					Math.toRadians(-49.55), Math.toRadians(30.76),
					Math.toRadians(-30.73), Math.toRadians(20.11)));

			performCalibrationMotion(new JointPosition(Math.toRadians(-48.66),
					Math.toRadians(24.68), Math.toRadians(-11.52),
					Math.toRadians(10.48), Math.toRadians(-11.38),
					Math.toRadians(-20.70), Math.toRadians(20.87)));

			performCalibrationMotion(new JointPosition(Math.toRadians(9.01),
					Math.toRadians(-35.00), Math.toRadians(24.72),
					Math.toRadians(-82.04), Math.toRadians(14.65),
					Math.toRadians(-29.95), Math.toRadians(1.57)));
			
			logger.info("Moving to home position");

			lbr.move(ptpHome().setJointVelocityRel(0.2));

			// Move to home position at the end
			logger.info("Moving to drive position");
			moveArmToDrivePos(0.2);
			appData.getProcessData("lastCalibrationOfArm").setValue((new Date().getTime() + ""));
		}
	}
	
	
}
