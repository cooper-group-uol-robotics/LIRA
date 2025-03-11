package robotChemist.interfaces;
import robotChemist.exceptions.CriticalActionFailException;
import robotChemist.utility.CubeFinderAngled;

import com.kuka.roboticsAPI.RoboticsAPIContext;
import com.kuka.roboticsAPI.applicationModel.IApplicationData;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.persistenceModel.IPersistenceEngine;
import com.kuka.roboticsAPI.persistenceModel.PersistenceException;
import com.kuka.roboticsAPI.persistenceModel.XmlApplicationDataSource;
import com.kuka.task.ITaskLogger;

/***
 * This class represents a high level abstraction of the LBRiiwa arm and its attached Festo gripper.
 * It allows access to these individual components and other high level operations such as performing six point calibration.
 * @author stoic-roboticist
 *
 */
public class LBRCommander
{
	private GenericTwoFingerGripper gripper;
	private LBRiiwaArm lbrArm;
	private IApplicationData appData;
	private LBR lbr;
	private RoboticsAPIContext context;
	private Tool geoGripperTool;
	private ITaskLogger logger;
	
	/***
	 * Creates an instance of the LBRCommander class.
	 * @param lbr - RoboticsAPI LBRiiwa interface.
	 * @param gripper - the gripper interface.
	 * @param context - RoboticAPI application context.
	 * @param appData - RoboticsAPI application data interface.
	 * @param kukaCont - RoboticsAPI controller interface.
	 * @param geoGripperTool - the geometric tool representing the attached gripper to the arm flange.
	 * @param logger - RoboticsAPI logging interface.
	 */
	public LBRCommander(LBR lbr, GenericTwoFingerGripper gripper, RoboticsAPIContext context, IApplicationData appData, Controller kukaCont, Tool geoGripperTool, ITaskLogger logger){
		this.appData = appData;
		this.lbr = lbr;
		this.context = context;
		this.geoGripperTool = geoGripperTool;
		this.logger = logger;
		this.gripper = gripper;
		this.lbrArm = new LBRiiwaArm(lbr, geoGripperTool, appData, kukaCont, logger);
	}
	
    /***
     * Factory method to create an LBRCommander instance with default dependencies.
     * @param context - RoboticAPI application context.
     * @param appData - RoboticsAPI application data interface.
     * @param logger - RoboticsAPI logging interface.
     * @return A fully initialized LBRCommander instance.
     */
    public static LBRCommander createWithDefaults(RoboticsAPIContext context, IApplicationData appData, ITaskLogger logger) {
        LBR lbr = context.getDeviceFromType(LBR.class);
        Controller kukaCont = context.getController("KUKA_Sunrise_Cabinet_1");
        return new LBRCommander(lbr, null, context, appData, kukaCont, null, logger);
    }

	/***
	 * Gets the interface to Festo Gripper.
	 * @return Festo Gripper interface.
	 */
	public GenericTwoFingerGripper getGripper(){
		return gripper;
	}
	
	public RoboticsAPIContext getContext() {
	    return context;
	}
	
	/***
	 * Gets the interface to the LBRiiwa arm.
	 * @return LBRiiwa interface.
	 */
	public LBRiiwaArm getArm(){
		return lbrArm;
	}
	
	/***
	 * Commands the arm to perform six point calibration to update the taught frames
	 * relative to the calibration cube.
	 * @param station - the station name whose frames are being calibrated.
	 * @param goToDrivePos - if True move the arm to the drive position after completing the calibration.
	 * @throws CriticalActionFailException if an error occurred when performing the calibration
	 */
	public void PerformSixPointCalibration(String station, boolean goToDrivePos) throws CriticalActionFailException{
		gripper.close();
		String Cube_Pre_Estimation = "/" + station + "/Pre_Cube_Pos";
		String Cube_Estimation = "/" + station + "/Cube_Pos";
		String Cube_Origin =  "/" + station + "/Cube_Corner";
		try{
			ObjectFrame cubeTarget = appData.getFrame(Cube_Estimation);
			Frame target = cubeTarget.copyWithRedundancy();
			logger.info("Pre Calibration Cube: " + appData.getFrame(Cube_Origin).toString());
			ObjectFrame pre_target = appData.tryGetFrame(Cube_Pre_Estimation);
			if (pre_target != null){
				lbrArm.moveToolPTP(pre_target, "/spacer/tcp", 0.25);
			}else {
				logger.info("Station does not have a pre-cube position, ignoring.");
			}
			CubeFinderAngled calibration = new CubeFinderAngled(appData, lbr, geoGripperTool, logger);
			Frame cube = calibration.FindCube(target);
			
			logger.info("Post Calibration Cube: " + cube.toString());
			//replace station cube_corner with new cube_origin
			XmlApplicationDataSource xmlDatasource = context.getEngine(IPersistenceEngine.class).getDataSource(XmlApplicationDataSource.class);
			appData.getFrame(Cube_Origin).getRedundancyInformation().putAll(cube.getRedundancyInformation());
			xmlDatasource.changeFrameTransformation(appData.getFrame(Cube_Origin), appData.getFrame(Cube_Origin).getParent().transformationTo(cube));
			if (pre_target != null){
				lbrArm.moveToolPTP(pre_target, "/spacer/tcp", 0.4);
			}
		} catch (PersistenceException e){logger.error("Couldn't find frame error: " + e.getMessage());}

	
		if (goToDrivePos)
		{
			lbrArm.moveArmToDrivePos(0.2);
		}
	}
}
