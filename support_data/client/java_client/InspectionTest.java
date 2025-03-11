package application;
import aruco.service.ArucoClient;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import javax.inject.Inject;
import robotChemist.inspect.InspectionHandler;
import robotChemist.interfaces.LBRCommander;
import robotChemist.tasks.ForceDetect;

public class InspectionTest extends RoboticsAPIApplication {  
    @Inject
//    private LBR lbr;
    private LBRCommander commander;
    private ArucoClient arucoClient;
    

    public void run() {
    	String stationName = "Base";
    	String TargetName = "PXRDRack";
    	int id = 10;
    	arucoClient.updateMarkerFrame(stationName, id);
    	ForceDetect forceDetect = new ForceDetect(commander); 
        InspectionHandler inspector = new InspectionHandler(commander, forceDetect);
//        Check_PXRDRack, Check_ChemRack
        inspector.runInspection("Has the black rack been placed onto the holder properly?", TargetName, stationName, 10);
    }
}
