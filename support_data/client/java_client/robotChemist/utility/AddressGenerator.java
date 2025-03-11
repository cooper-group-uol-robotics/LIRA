package robotChemist.utility;

/***
 * A utility class that is used by the ROS node manager to keep track and assign port numbers
 * to the given ROS nodes.
 * @author stoic-roboticist
 *
 */
public class AddressGenerator 
{
  int address = 30000;

  /***
   * Provides a new port number that is within the accepted range by the Sunrise.OS controller
   * @return a port number that is within the acceptable range between 30000 to 30010
   */
  public int getNewAddress() 
  {
    // Only port numbers from 30000 to 30010 are available.
    // See KUKA SI Manual "Network communication via UDP and TCP/IP".
    if (address > 30010) 
    {
      throw new RuntimeException("Only port numbers from 30000 to 30010 are available");
    }
    else 
    {
      int newAddress = address;
      address++;
      return newAddress;
    }
  }
}