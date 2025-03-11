package robotChemist.inspect;

import robotChemist.interfaces.LBRCommander;
import vlm.sceneclient;
import robotChemist.tasks.ForceDetect;
import com.kuka.roboticsAPI.geometricModel.math.CoordinateAxis;

public class InspectionHandler {
    private static final int MAX_RETRIES = 10;
    private int retryCount = 0;
    private sceneclient client;
    private LBRCommander commander;
    private ForceDetect forceDetect;
    
    private String buildCheckPosePath(String stationName, int id, String rackName) {
        if ("Base".equals(stationName)) {
            return String.format(
                "/RobotRacks/Check_%s", 
                rackName
            );
        } else {
            return String.format(
                "/%s/CheckPose_%d/CameraFrame/MarkerFrame/Check_%s", 
                stationName, 
                id, 
                rackName
            );
        }
    }
    
    public InspectionHandler(LBRCommander commander, ForceDetect forceDetect) {
        this.client = new sceneclient();
        this.commander = commander;
        this.forceDetect = forceDetect;
    }

    
    
    public void runInspection(String question, String RackName, String StationName, int id) {
        while (true) {
        	
        	
            if (retryCount >= MAX_RETRIES) {
                System.out.println("Max retries exceeded. Aborting inspection.");
                resetRetryCount();
                return;
            }
            try {
            	String checkPosePath = buildCheckPosePath(StationName, id, RackName);
            	
            	commander.getArm().moveToolPTP(checkPosePath, "/spacer/tcp", 0.3);
            	
                String result = client.describeScene(question);
                
                System.out.println("Inspect Result: " + result);
                
                if (result.contains("True")) {
                	System.out.println("Inspection succeeded. Exiting.");
                	return;

                } else if (result.contains("False")) {
                    if (result.contains("recoverable")) {
                        System.out.println("Recoverable error detected.");
                        executeRecovery(RackName, StationName, result, id);
                        retryCount++;
                        runInspection(question, RackName, StationName, id);
                    } else {
                    	System.out.println("Unrecoverable error. Stopping and waiting for human intervention.");
                        boolean resolved = startPeriodicChecks(question, RackName, StationName, id);
                        if (resolved) {
                            return; // 
                        } else {
                            System.out.println("Human intervention timeout. Aborting.");
                            resetRetryCount();
                            return;
                        }
                    }
                }

            } catch (Exception e) {
                System.err.println("An error occurred:");
                e.printStackTrace();
                resetRetryCount(); // Stop execution in case of an exception
            }
        }
    }
    
    private void resetRetryCount() {
        retryCount = 0;
    }
    
    private boolean startPeriodicChecks(String question, String rackName, String stationName, int id) {
        final long MAX_WAIT_MS = 5 * 60 * 1000; 
        final long CHECK_INTERVAL_MS = 60 * 1000; 
        long startTime = System.currentTimeMillis();

        while (System.currentTimeMillis() - startTime < MAX_WAIT_MS) {
            try {
                
                Thread.sleep(CHECK_INTERVAL_MS);
                
                System.out.println("Performing periodic check...");
                String result = client.describeScene(question);
                System.out.println("Periodic Check Result: " + result);

                if (result.contains("True")) {
                    System.out.println("Issue resolved by human. Resuming workflow.");
                    return true;
                } else if (result.contains("False")) {
                    System.out.println("Issue still unresolved.");
                }
            } catch (InterruptedException e) {
                System.err.println("Periodic checks interrupted:");
                e.printStackTrace();
                return false;
            } catch (Exception e) {
                System.err.println("Error during periodic check:");
                e.printStackTrace();
            }
        }
        return false;
    }
    
    private void executeRecovery(String TargetName, String StationName, String result, int id) {
        if (result.contains("left")) {
            System.out.println("Recovering left...");
            if (TargetName.equals("ChemRack")) {
            	if (StationName.equals("ChemSpeedLDStation_QR")) {
            		commander.getArm().moveToolPTP("/ChemSpeedLDStation_QR/CheckPose_10/CameraFrame/MarkerFrame/Recover/ChemRack_left", "/spacer/tcp", 0.3);
            		forceDetect.forceDetect(CoordinateAxis.X, -50);
            	}
            	if (StationName.equals("Base")) {
            		commander.getArm().moveToolPTP("/RobotRacks/Recover/ChemRack_left", "/spacer/tcp", 0.3);
            		forceDetect.forceDetect(CoordinateAxis.Y, 50);
            	}
            	if (StationName.equals("YumiStation_QR")) {
            		commander.getArm().moveToolPTP("/YumiStation_QR/CheckPose_10/CameraFrame/MarkerFrame/Recover/ChemRack_left", "/spacer/tcp", 0.3);
            		forceDetect.forceDetect(CoordinateAxis.X, -50);                            		
            	}
            }
            if (TargetName.equals("PXRDRack")) {
            	if (StationName.equals("Base")) {
            		commander.getArm().moveToolPTP("/RobotRacks/Recover/PXRDRack_left", "/spacer/tcp", 0.3);
            		forceDetect.forceDetect(CoordinateAxis.Y, -50);
            		
            	}
            	if (StationName.equals("YumiStation_QR")) {
            		commander.getArm().moveToolPTP("/YumiStation_QR/CheckPose_10/CameraFrame/MarkerFrame/Recover/PXRDRack_left", "/spacer/tcp", 0.3);
            		forceDetect.forceDetect(CoordinateAxis.X, -50); 
            	}
            	if (StationName.equals("PXRDLoadingStation_QR")) {
            		commander.getArm().moveToolPTP("/PXRDLoadingStation_QR/CheckPose_5/CameraFrame/MarkerFrame/Recover/PXRDRack_left", "/spacer/tcp", 0.3);
            		forceDetect.forceDetect(CoordinateAxis.X, -50); 
            	}
            }
        } else if (result.contains("right")) {
            System.out.println("Recovering right...");

            if (TargetName.equals("ChemRack")) {
            	if (StationName.equals("ChemSpeedLDStation_QR")) {
            		commander.getArm().moveToolPTP("/ChemSpeedLDStation_QR/CheckPose_10/CameraFrame/MarkerFrame/Recover/ChemRack_right", "/spacer/tcp", 0.3);
            		forceDetect.forceDetect(CoordinateAxis.X, 50);
            	}
            	if (StationName.equals("Base")) {
            		commander.getArm().moveToolPTP("/RobotRacks/Recover/ChemRack_right", "/spacer/tcp", 0.3);
            		forceDetect.forceDetect(CoordinateAxis.Y, -50);
            	}
            	if (StationName.equals("YumiStation_QR")) {
            		commander.getArm().moveToolPTP("/YumiStation_QR/CheckPose_10/CameraFrame/MarkerFrame/Recover/ChemRack_right", "/spacer/tcp", 0.3);
            		forceDetect.forceDetect(CoordinateAxis.X, 50);                           	}

            }
            if (TargetName.equals("PXRDRack")) {
            	if (StationName.equals("Base")) {
            		commander.getArm().moveToolPTP("/RobotRacks/Recover/PXRDRack_right", "/spacer/tcp", 0.3);
            		forceDetect.forceDetect(CoordinateAxis.Y, -50);
            	}
            	if (StationName.equals("YumiStation_QR")) {
            		commander.getArm().moveToolPTP("/YumiStation_QR/CheckPose_10/CameraFrame/MarkerFrame/Recover/PXRDRack_right", "/spacer/tcp", 0.3);
            		forceDetect.forceDetect(CoordinateAxis.X, 50); 
            	}
            	if (StationName.equals("PXRDLoadingStation_QR")) {
            		commander.getArm().moveToolPTP("/PXRDLoadingStation_QR/CheckPose_5/CameraFrame/MarkerFrame/Recover/PXRDRack_right", "/spacer/tcp", 0.3);
            		forceDetect.forceDetect(CoordinateAxis.X, 50); 
            	}
            }
        }else if (result.contains("back")) {
            if (TargetName.equals("ChemRack")) {
            	if (StationName.equals("ChemSpeedLDStation_QR")) {
            		commander.getArm().moveToolPTP("/ChemSpeedLDStation_QR/CheckPose_10/CameraFrame/MarkerFrame/Recover/ChemRack_back", "/spacer/tcp", 0.3);
            		forceDetect.forceDetect(CoordinateAxis.Y, 50);
            	}
            	if (StationName.equals("Base")) {
            		commander.getArm().moveToolPTP("/RobotRacks/Recover/ChemRack_back", "/spacer/tcp", 0.3);
            		forceDetect.forceDetect(CoordinateAxis.X, 50);
            	}
            	if (StationName.equals("YumiStation_QR")) {
            		commander.getArm().moveToolPTP("/YumiStation_QR/CheckPose_10/CameraFrame/MarkerFrame/Recover/ChemRack_back", "/spacer/tcp", 0.3);
            		forceDetect.forceDetect(CoordinateAxis.Y, 50);                           	}

            }
            if (TargetName.equals("PXRDRack")) {
            	if (StationName.equals("Base")) {
            		commander.getArm().moveToolPTP("/RobotRacks/Recover/PXRDRack_back", "/spacer/tcp", 0.3);
            		forceDetect.forceDetect(CoordinateAxis.Y, 50);
            	}
            	if (StationName.equals("YumiStation_QR")) {
            		commander.getArm().moveToolPTP("/YumiStation_QR/CheckPose_10/CameraFrame/MarkerFrame/Recover/PXRDRack_back", "/spacer/tcp", 0.3);
            		forceDetect.forceDetect(CoordinateAxis.Y, 50);  
            	}
            	if (StationName.equals("PXRDLoadingStation_QR")) {
            		commander.getArm().moveToolPTP("/PXRDLoadingStation_QR/CheckPose_5/CameraFrame/MarkerFrame/Recover/PXRDRack_back", "/spacer/tcp", 0.3);
            		forceDetect.forceDetect(CoordinateAxis.Y, 50);  
            	}
            }
        	
        	
        }else if (result.contains("forward")) {
            if (TargetName.equals("ChemRack")) {
            	if (StationName.equals("ChemSpeedLDStation_QR")) {
            		commander.getArm().moveToolPTP("/ChemSpeedLDStation_QR/CheckPose_10/CameraFrame/MarkerFrame/Recover/ChemRack_forward", "/spacer/tcp", 0.3);
            		forceDetect.forceDetect(CoordinateAxis.Y, -50);
            	}
            	if (StationName.equals("Base")) {
            		commander.getArm().moveToolPTP("/RobotRacks/Recover/ChemRack_forward", "/spacer/tcp", 0.3);
            		forceDetect.forceDetect(CoordinateAxis.X, -50);
            	}
            	if (StationName.equals("YumiStation_QR")) {
            		commander.getArm().moveToolPTP("/YumiStation_QR/CheckPose_10/CameraFrame/MarkerFrame/Recover/ChemRack_forward", "/spacer/tcp", 0.3);
            		forceDetect.forceDetect(CoordinateAxis.Y, -50);                            	}

            }
        }
            if (TargetName.equals("PXRDRack")) {
            	if (StationName.equals("Base")) {
            		commander.getArm().moveToolPTP("/RobotRacks/Recover/PXRDRack_forward", "/spacer/tcp", 0.3);
            		forceDetect.forceDetect(CoordinateAxis.Y, -50);
            	}
            	if (StationName.equals("YumiStation_QR")) {
            		commander.getArm().moveToolPTP("/YumiStation_QR/CheckPose_10/CameraFrame/MarkerFrame/Recover/PXRDRack_forward", "/spacer/tcp", 0.3);
            		forceDetect.forceDetect(CoordinateAxis.Y, -50);  
            	}
            	if (StationName.equals("PXRDLoadingStation_QR")) {
            		commander.getArm().moveToolPTP("/PXRDLoadingStation_QR/CheckPose_5/CameraFrame/MarkerFrame/Recover/PXRDRack_forward", "/spacer/tcp", 0.3);
            		forceDetect.forceDetect(CoordinateAxis.Y, -50);  
            }
        }
        	

        try {
            Thread.sleep(2000); 
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    
    
}
