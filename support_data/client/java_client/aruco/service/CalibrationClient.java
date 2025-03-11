package aruco.service;

import java.io.ByteArrayOutputStream;

import javax.xml.soap.*;

import com.kuka.roboticsAPI.RoboticsAPIContext;
import com.kuka.roboticsAPI.applicationModel.IApplicationData;
import com.kuka.roboticsAPI.geometricModel.math.XyzAbcTransformation;
import com.kuka.roboticsAPI.persistenceModel.IPersistenceEngine;
import com.kuka.roboticsAPI.persistenceModel.XmlApplicationDataSource;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.math.ITransformation;
import com.kuka.roboticsAPI.geometricModel.math.Transformation;
import com.kuka.roboticsAPI.geometricModel.math.Vector;
import com.kuka.roboticsAPI.geometricModel.math.Rotation;


//this client is used to interact with xx.py on Jetson
//the client will send Calibration_i and Marker_i to the server
//After running Calibration_UpdateMarker.java, finally, run this client

public class CalibrationClient {

    private static final String SERVICE_URL = "http://172.31.1.79:65432/acl.kuka.soap";
    private IApplicationData appData;
    private RoboticsAPIContext context;

    public CalibrationClient(IApplicationData appData, RoboticsAPIContext context) {
        this.appData = appData;
        this.context = context;
    }


    public void updateCalibrationFrame(String stationName, int id) {
        try {
            // Fetch frames
            String calibrationFramePath = String.format("/%s/Calibration_%d", stationName, id);
            String markerFramePath = String.format("/%s/Marker_%d", stationName, id);
            
            ObjectFrame calibrationFrame = appData.getFrame(calibrationFramePath);
            ObjectFrame markerFrame = appData.getFrame(markerFramePath);
            
            // Extract translation and orientation for both frames
            double[] calibrationTranslation = getTranslation(calibrationFrame);
            double[] calibrationRotation = getRotationABC(calibrationFrame); // Assuming you've implemented this method
            
            double[] markerTranslation = getTranslation(markerFrame);
            double[] markerRotation = getRotationABC(markerFrame); 
            

            // Send data to server
            SOAPConnectionFactory soapConnectionFactory = SOAPConnectionFactory.newInstance();
            SOAPConnection soapConnection = soapConnectionFactory.createConnection();

            MessageFactory messageFactory = MessageFactory.newInstance();
            SOAPMessage soapMessage = messageFactory.createMessage();

            createSoapEnvelope(soapMessage, stationName, id, calibrationTranslation, calibrationRotation, markerTranslation, markerRotation);

            MimeHeaders headers = soapMessage.getMimeHeaders();
            headers.addHeader("SOAPAction", "acl.kuka.soap" + "/UpdateFrames");

            soapMessage.saveChanges();
            SOAPMessage soapResponse = soapConnection.call(soapMessage, SERVICE_URL);
            printSOAPResponse(soapResponse);

            soapConnection.close();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
    private double[] getTranslation(ObjectFrame frame) {
        Transformation transformation = frame.getTransformationFromParent(); // Assuming global coordinates
        Vector position = transformation.getTranslation();
        return new double[]{position.getX(), position.getY(), position.getZ()};
    }

    private double[] getRotationABC(ObjectFrame frame) {
        Transformation transformation = frame.getTransformationFromParent(); // Assuming global coordinates
        Rotation eulerABC = transformation.getRotation();
        return new double[]{
            eulerABC.getAlphaRad(), 
            eulerABC.getBetaRad(), 
            eulerABC.getGammaRad()
        };
    }
    
    
    private void createSoapEnvelope(SOAPMessage soapMessage, String stationName, int id, 
            double[] calibrationTranslation, double[] calibrationRotation, 
            double[] markerTranslation, double[] markerRotation) throws SOAPException {
			SOAPPart soapPart = soapMessage.getSOAPPart();
			SOAPEnvelope envelope = soapPart.getEnvelope();
			SOAPBody soapBody = envelope.getBody();
			
			SOAPElement soapBodyElem = soapBody.addChildElement("UpdateFrames", "ns1", "http://acl.kuka.soap/");
			
			// Add Station Name
			SOAPElement stationNameElement = soapBodyElem.addChildElement("stationName", "ns1");
			stationNameElement.addTextNode(stationName);
			
			// Add ID
			SOAPElement idElement = soapBodyElem.addChildElement("id", "ns1");
			idElement.addTextNode(String.valueOf(id));
			
			// Calibration Data
			SOAPElement calibrationDataElem = soapBodyElem.addChildElement("CalibrationData", "ns1");
			addFrameData(calibrationDataElem, "Calibration", calibrationTranslation, calibrationRotation);
			
			// Marker Data
			SOAPElement markerDataElem = soapBodyElem.addChildElement("MarkerData", "ns1");
			addFrameData(markerDataElem, "Marker", markerTranslation, markerRotation);
			}
			
			private void addFrameData(SOAPElement parentElement, String frameType, 
			      double[] translation, double[] rotation) throws SOAPException {
			// Add Translation
			SOAPElement translationElem = parentElement.addChildElement("Translation", "ns1");
			translationElem.addChildElement("x", "ns1").addTextNode(String.format("%.6f", translation[0]));
			translationElem.addChildElement("y", "ns1").addTextNode(String.format("%.6f", translation[1]));
			translationElem.addChildElement("z", "ns1").addTextNode(String.format("%.6f", translation[2]));
			
			// Add Rotation (in degrees)
			SOAPElement rotationElem = parentElement.addChildElement("Rotation", "ns1");
			rotationElem.addChildElement("a", "ns1").addTextNode(String.format("%.6f", Math.toDegrees(rotation[0])));
			rotationElem.addChildElement("b", "ns1").addTextNode(String.format("%.6f", Math.toDegrees(rotation[1])));
			rotationElem.addChildElement("c", "ns1").addTextNode(String.format("%.6f", Math.toDegrees(rotation[2])));
			
			}
			
			

    private void printSOAPResponse(SOAPMessage soapResponse) throws Exception {
        ByteArrayOutputStream out = new ByteArrayOutputStream();
        soapResponse.writeTo(out);
        System.out.println("SOAP Response: \n" + new String(out.toByteArray()));
    }
}
