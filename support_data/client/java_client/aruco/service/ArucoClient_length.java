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

//tested and work

public class ArucoClient_length {

    private static final String SERVICE_URL = "http://172.31.1.79:65432/acl.kuka.soap";
    private IApplicationData appData;
    private RoboticsAPIContext context;

    public ArucoClient_length(IApplicationData appData, RoboticsAPIContext context) {
        this.appData = appData;
        this.context = context;
    }

    public void updateMarkerFrame(String stationName, int id, double markerSize) {
        try {
            // Create SOAP Connection
            SOAPConnectionFactory soapConnectionFactory = SOAPConnectionFactory.newInstance();
            SOAPConnection soapConnection = soapConnectionFactory.createConnection();

            // Create SOAP Message
            MessageFactory messageFactory = MessageFactory.newInstance();
            SOAPMessage soapMessage = messageFactory.createMessage();
            SOAPPart soapPart = soapMessage.getSOAPPart();

            // Fill SOAP Body
            SOAPEnvelope envelope = soapPart.getEnvelope();
            SOAPBody soapBody = envelope.getBody();
            SOAPElement bodyElement = soapBody.addChildElement("UpdateMarkerFrame", "ns", "acl.kuka.soap");
            //bodyElement.addChildElement("UpdateMarker").addTextNode("UpdateMarker");
            SOAPElement idElement = bodyElement.addChildElement("id", "ns"); 
            idElement.addTextNode(String.valueOf(id));
            
            // Add marker size to the SOAP body
            SOAPElement sizeElement = bodyElement.addChildElement("marker_size", "ns");
            sizeElement.addTextNode(String.valueOf(markerSize));

            // Send SOAP Request
            // ?????????Similar to ReqMsg?????????
            SOAPMessage soapResponse = soapConnection.call(soapMessage, SERVICE_URL);
            
            ByteArrayOutputStream out = new ByteArrayOutputStream();
            soapResponse.writeTo(out);
            String strMsg = new String(out.toByteArray());
            System.out.println(strMsg);
            // Process the SOAP Response
            processSoapMarkerResponse(soapResponse, stationName, id);

        } catch (Exception e) {
            e.printStackTrace();
        }
    }
    
    
    //handle the response
    private void processSoapMarkerResponse(SOAPMessage soapResponse, String stationName, int id) {
        try {
            // Extract data from the SOAP response and then update MarkerFrame

        	
            SOAPBody soapBody = soapResponse.getSOAPBody();
            SOAPElement responseElement = (SOAPElement) soapBody.getFirstChild();
            System.out.println("response Element: " + responseElement);
            SOAPElement poseElement = (SOAPElement) responseElement.getElementsByTagName("tns:pose").item(0);
            System.out.println("Pose Element: " + poseElement);
            
            double posX = Double.parseDouble(poseElement.getElementsByTagName("tns:x").item(0).getTextContent());
            double posY = Double.parseDouble(poseElement.getElementsByTagName("tns:y").item(0).getTextContent());
            double posZ = Double.parseDouble(poseElement.getElementsByTagName("tns:z").item(0).getTextContent());
            double rotA = Math.toDegrees(Double.parseDouble(poseElement.getElementsByTagName("tns:a").item(0).getTextContent()));
            double rotB = Math.toDegrees(Double.parseDouble(poseElement.getElementsByTagName("tns:b").item(0).getTextContent()));
            double rotC = Math.toDegrees(Double.parseDouble(poseElement.getElementsByTagName("tns:c").item(0).getTextContent()));

            ITransformation transformation = XyzAbcTransformation.ofDeg(posX, posY, posZ, rotA, rotB, rotC);
            
            String framePath = String.format("/%s/CheckPose_%d/CameraFrame/MarkerFrame", stationName, id);
            
            XmlApplicationDataSource xmlDatasource = context.getEngine(IPersistenceEngine.class).getDataSource(XmlApplicationDataSource.class);
            
            ObjectFrame existingMarkerFrame = appData.getFrame(framePath);
            
            xmlDatasource.changeFrameTransformation(existingMarkerFrame, transformation);

        } catch (Exception e) {
            e.printStackTrace();
        }
    }

}
