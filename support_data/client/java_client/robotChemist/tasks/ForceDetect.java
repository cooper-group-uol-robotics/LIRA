package robotChemist.tasks;

import robotChemist.interfaces.LBRCommander;
import com.kuka.roboticsAPI.geometricModel.math.CoordinateAxis;

public class ForceDetect {
    private LBRCommander commander;

    public ForceDetect(LBRCommander commander) {
        this.commander = commander;
    }

    public void forceDetect(CoordinateAxis axis, double distance) { 
        System.out.println("Starting Recovery Motion...");

        // Ensure commander is initialized before calling movement functions
        if (commander == null) {
            System.err.println("Error: Commander is not initialized in ForceDetect!");
            return;
        }

        while (true) {
            try {
                commander.getArm().moveLINRelForce(distance, 7, 10, axis);
                Thread.sleep(1000);

                commander.getArm().moveLINRelForce(-distance, 7, 10, axis);
                Thread.sleep(1000);

                commander.getArm().moveLINRelForce(-30, 10, 10, CoordinateAxis.Z);
                
            } catch (InterruptedException e) {
                System.err.println("Interrupted during recovery motion: " + e.getMessage());
                Thread.currentThread().interrupt(); // Restore interrupted status
                break; // Exit loop on interruption
            }
        }
    }
}
