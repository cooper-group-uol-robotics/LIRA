package vlm;

import java.io.ByteArrayOutputStream;
import javax.xml.soap.*;

public class sceneclient {

    private static final String SERVICE_URL = "http://172.31.1.79:65432/acl.kuka.soap";

    public String describeScene(String question) {
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

            // Add DescribeScene request with dynamic question
            SOAPElement bodyElement = soapBody.addChildElement("DescribeScene", "tns", "acl.kuka.soap");
            SOAPElement questionElement = bodyElement.addChildElement("question", "tns");
            questionElement.addTextNode(question);  // Dynamically set the question

            // Save changes to SOAP message
            soapMessage.saveChanges();

            // Print the SOAP request (optional, for debugging)
            System.out.println("Request SOAP Message:");
            soapMessage.writeTo(System.out);
            System.out.println("\n");

            // Send SOAP Request and receive response
            SOAPMessage soapResponse = soapConnection.call(soapMessage, SERVICE_URL);

            // Convert response to string
            ByteArrayOutputStream out = new ByteArrayOutputStream();
            soapResponse.writeTo(out);
            String responseString = new String(out.toByteArray());

            // Close the connection
            soapConnection.close();

            return responseString;

        } catch (Exception e) {
            e.printStackTrace();
            return "Error occurred while describing the scene: " + e.getMessage();
        }
    }

    // Usage example
    public static void main(String[] args) {
    	sceneclient client = new sceneclient();
        try {
            String question = "What objects are visible in the scene?";
            String result = client.describeScene(question);
            System.out.println("Scene Description:");
            System.out.println(result);
        } catch (Exception e) {
            System.err.println("An error occurred:");
            e.printStackTrace();
        }
    }
}
